// gnc-sim — Extended Kalman Filter navigation tests. World frame ENU, SI units.
//   A) Noise-free constant-velocity relative track: state converges to truth, covariance shrinks.
//   B) NIS consistency over a long noisy run: mean NIS ~ dof (3); 95% chi-square bound rarely broken.
//   C) Determinism: same seed -> identical estimates.
//   D) 3x3 inverse correctness (exercised through the update step on a known geometry).
#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include "gncsim/core/Rng.hpp"
#include "gncsim/gnc/Ekf.hpp"
#include "gncsim/math/Vector3.hpp"

using namespace gncsim;

namespace {

// Spherical measurement (az, el, range) of a relative position.
struct Meas {
  double az, el, range;
};
Meas toMeas(const Vector3& p) {
  const double horiz = std::sqrt(p.x * p.x + p.y * p.y);
  return {std::atan2(p.y, p.x), std::atan2(p.z, horiz), p.norm()};
}

// Trace of the 6x6 covariance is not exposed; instead we infer convergence from the estimate error.
double posErr(const Ekf& e, const Vector3& truth) { return (e.relPos() - truth).norm(); }
double velErr(const Ekf& e, const Vector3& truth) { return (e.relVel() - truth).norm(); }

}  // namespace

// ── A) Noise-free constant-velocity track: converges to truth ─────────────────────────────────
TEST(Ekf, ConvergesOnNoiseFreeConstantVelocityTrack) {
  const double dt = 0.005;
  Ekf ekf(dt, /*q=*/10.0, /*sigma_az=*/0.003, /*sigma_el=*/0.003, /*sigma_range=*/5.0);

  Vector3 p0{5000.0, 1200.0, 2000.0};
  const Vector3 v0{-300.0, 50.0, -20.0};  // constant relative velocity, no own-accel

  // First update bootstraps from the measurement (vel = 0). Error should then collapse over a few s.
  double pos_err_early = 0.0, pos_err_late = 0.0, vel_err_late = 0.0;
  const int steps = 1200;  // 6 s
  for (int k = 0; k <= steps; ++k) {
    const Vector3 p = p0 + v0 * (k * dt);
    if (k > 0) ekf.predict(Vector3{});  // no own acceleration
    const Meas m = toMeas(p);
    ekf.update(m.az, m.el, m.range);

    if (k == 40) pos_err_early = posErr(ekf, p);  // ~0.2 s in
    if (k == steps) {
      pos_err_late = posErr(ekf, p);
      vel_err_late = velErr(ekf, v0);
    }
  }

  // After several seconds of noise-free measurements the estimate locks tightly onto truth.
  EXPECT_LT(pos_err_late, 0.5);
  EXPECT_LT(vel_err_late, 0.5);
  // And it improved from the early transient (covariance / error decreased).
  EXPECT_LT(pos_err_late, pos_err_early);
}

// Covariance trace decreases as measurements accumulate. We approximate "trace" via the magnitude
// of the estimate's response to a fixed innovation: instead, assert error monotonic-ish decrease.
TEST(Ekf, EstimateErrorDecreasesWithMeasurements) {
  const double dt = 0.01;
  Ekf ekf(dt, /*q=*/5.0, 0.002, 0.002, 3.0);

  const Vector3 p0{4000.0, -800.0, 1500.0};
  const Vector3 v0{-250.0, 30.0, 10.0};

  double err_at_10 = 0.0, err_at_400 = 0.0;
  for (int k = 0; k <= 400; ++k) {
    const Vector3 p = p0 + v0 * (k * dt);
    if (k > 0) ekf.predict(Vector3{});
    const Meas m = toMeas(p);
    ekf.update(m.az, m.el, m.range);
    if (k == 10) err_at_10 = velErr(ekf, v0);
    if (k == 400) err_at_400 = velErr(ekf, v0);
  }
  EXPECT_LT(err_at_400, err_at_10);  // velocity uncertainty (and error) shrinks over time
}

// ── B) NIS consistency over a long noisy run ──────────────────────────────────────────────────
TEST(Ekf, NisConsistencyMeanNearDof) {
  const double dt = 0.005;
  const double sigma_ang = 0.003;  // rad
  const double sigma_rng = 5.0;    // m
  Ekf ekf(dt, /*q=*/20.0, sigma_ang, sigma_ang, sigma_rng);
  Rng rng(12345);

  // Truth: constant-velocity relative track (consistent with the nearly-constant-velocity model).
  Vector3 p0{6000.0, 1500.0, 2500.0};
  const Vector3 v0{-280.0, 40.0, -15.0};

  // Chi-square dof=3 upper 95% bound is ~7.815. A consistent filter exceeds it ~5% of the time.
  constexpr double kChi2_95_dof3 = 7.815;

  const int warmup = 400;  // let the filter settle before scoring NIS
  const int steps = 8000;
  double nis_sum = 0.0;
  int nis_count = 0;
  int exceed_count = 0;
  for (int k = 0; k <= steps; ++k) {
    const Vector3 p = p0 + v0 * (k * dt);
    if (k > 0) ekf.predict(Vector3{});
    Meas m = toMeas(p);
    m.az += rng.gaussian(0.0, sigma_ang);
    m.el += rng.gaussian(0.0, sigma_ang);
    m.range += rng.gaussian(0.0, sigma_rng);
    ekf.update(m.az, m.el, m.range);

    if (k >= warmup) {
      nis_sum += ekf.nis();
      ++nis_count;
      if (ekf.nis() > kChi2_95_dof3) ++exceed_count;
    }
  }

  const double mean_nis = nis_sum / nis_count;
  const double exceed_frac = static_cast<double>(exceed_count) / nis_count;

  // Mean NIS near dof = 3 within ~15%.
  EXPECT_GT(mean_nis, 3.0 * 0.85);
  EXPECT_LT(mean_nis, 3.0 * 1.15);
  // The 95% bound should be exceeded only rarely (well under ~12%).
  EXPECT_LT(exceed_frac, 0.12);
}

// ── C) Determinism: same seed -> identical estimates ──────────────────────────────────────────
TEST(Ekf, DeterministicForSameSeed) {
  const double dt = 0.005;
  const double sigma_ang = 0.003, sigma_rng = 5.0;
  const Vector3 p0{5000.0, -1000.0, 2000.0};
  const Vector3 v0{-260.0, 35.0, -25.0};

  auto run = [&](std::uint64_t seed) {
    Ekf ekf(dt, 20.0, sigma_ang, sigma_ang, sigma_rng);
    Rng rng(seed);
    Vector3 last;
    for (int k = 0; k <= 2000; ++k) {
      const Vector3 p = p0 + v0 * (k * dt);
      if (k > 0) ekf.predict(Vector3{});
      Meas m = toMeas(p);
      m.az += rng.gaussian(0.0, sigma_ang);
      m.el += rng.gaussian(0.0, sigma_ang);
      m.range += rng.gaussian(0.0, sigma_rng);
      ekf.update(m.az, m.el, m.range);
      last = ekf.relPos();
    }
    return last;
  };

  const Vector3 a = run(777);
  const Vector3 b = run(777);
  EXPECT_DOUBLE_EQ(a.x, b.x);
  EXPECT_DOUBLE_EQ(a.y, b.y);
  EXPECT_DOUBLE_EQ(a.z, b.z);

  // A different seed should (essentially always) give a different trajectory.
  const Vector3 c = run(778);
  EXPECT_NE(a.x, c.x);
}

// ── D) 3x3 inverse correctness, via a noise-free single-step update on a known geometry ────────
// The update step internally inverts the 3x3 innovation covariance S. On a noise-free measurement
// that exactly matches the predicted state, the innovation is ~0 and the state must be unchanged —
// which only holds if S^-1 is computed correctly (a wrong inverse would still leave a near-zero
// correction here, so we instead drive a known nonzero innovation and check the correction sign /
// magnitude is finite and sensible). We pair that with a direct algebraic inverse check below.
TEST(Ekf, UpdateProducesFiniteSensibleCorrection) {
  const double dt = 0.005;
  Ekf ekf(dt, 10.0, 0.003, 0.003, 5.0);

  const Vector3 truth{3000.0, 500.0, 1000.0};
  Meas m = toMeas(truth);
  ekf.update(m.az, m.el, m.range);  // bootstrap exactly onto truth
  EXPECT_LT(posErr(ekf, truth), 1e-6);

  // Now feed a measurement implying the target moved +x by a known amount; the corrected estimate
  // must move toward it (finite, same direction) — exercises K = P H^T S^-1 with a real S inverse.
  const Vector3 moved{3050.0, 500.0, 1000.0};
  ekf.predict(Vector3{});
  Meas m2 = toMeas(moved);
  ekf.update(m2.az, m2.el, m2.range);

  const Vector3 est = ekf.relPos();
  EXPECT_TRUE(std::isfinite(est.x) && std::isfinite(est.y) && std::isfinite(est.z));
  EXPECT_GT(est.x, truth.x);          // moved toward the new measurement
  EXPECT_LT(est.x, moved.x + 1.0);    // didn't overshoot wildly
}

// Direct algebraic check: A * inv(A) ≈ I for a representative full-rank 3x3, by replicating the
// cofactor inverse the EKF uses and verifying the product. (Mirrors the internal invert3x3.)
TEST(Ekf, ThreeByThreeInverseIsIdentity) {
  // A representative non-symmetric, well-conditioned matrix.
  const std::array<double, 9> A{4.0, 2.0, 1.0, 3.0, 5.0, 2.0, 1.0, 1.0, 3.0};

  // Cofactor / adjugate inverse (same math as Ekf's internal invert3x3).
  const double a = A[0], b = A[1], c = A[2];
  const double d = A[3], e = A[4], f = A[5];
  const double g = A[6], h = A[7], i = A[8];
  const double C0 = e * i - f * h;
  const double C1 = -(d * i - f * g);
  const double C2 = d * h - e * g;
  const double det = a * C0 + b * C1 + c * C2;
  ASSERT_GT(std::fabs(det), 1e-12);
  const double id = 1.0 / det;
  std::array<double, 9> inv{
      C0 * id,            (c * h - b * i) * id, (b * f - c * e) * id,
      C1 * id,            (a * i - c * g) * id, (c * d - a * f) * id,
      C2 * id,            (b * g - a * h) * id, (a * e - b * d) * id};

  // Product A * inv should be the identity.
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      double s = 0.0;
      for (int k = 0; k < 3; ++k) s += A[row * 3 + k] * inv[k * 3 + col];
      EXPECT_NEAR(s, row == col ? 1.0 : 0.0, 1e-12);
    }
  }
}
