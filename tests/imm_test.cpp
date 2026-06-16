// gnc-sim — Interacting Multiple Model (IMM) navigation tests (issue #36). World frame ENU, SI.
//   A) NIS consistency on a known maneuvering trajectory: mean combined NIS sits near the
//      measurement dof (3) across BOTH the quiescent and the maneuvering phases, where a single
//      constant-velocity EKF goes inconsistent.
//   B) Mode switching: the maneuver-model probability is low while the target coasts and rises
//      sharply once it pulls a hard lateral maneuver.
//   C) Determinism: same seed -> bit-identical estimates and mode probabilities.
//   D) The IMM tracks a maneuvering target more accurately (lower terminal position error) than a
//      single CV EKF tuned for quiescent flight.
#include "gncsim/gnc/Imm.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "gncsim/core/Rng.hpp"
#include "gncsim/gnc/Ekf.hpp"
#include "gncsim/math/Vector3.hpp"

using namespace gncsim;

namespace {

constexpr double kDt = 0.005;
constexpr double kSigAng = 0.003;  // seeker angular noise [rad]
constexpr double kSigRng = 5.0;    // range noise [m]

struct Meas {
  double az, el, range;
};
Meas toMeas(const Vector3& p) {
  const double horiz = std::sqrt(p.x * p.x + p.y * p.y);
  return {std::atan2(p.y, p.x), std::atan2(p.z, horiz), p.norm()};
}

// A known maneuvering relative trajectory: constant velocity for the first `t_man` seconds, then a
// hard constant lateral (y) acceleration. Returns relative position at time t.
Vector3 trajectory(double t, double t_man, const Vector3& p0, const Vector3& v0, double a_lat) {
  if (t < t_man) return p0 + v0 * t;
  const double dt = t - t_man;
  const Vector3 p_man = p0 + v0 * t_man;
  const Vector3 v_man = v0;
  Vector3 p = p_man + v_man * dt;
  p.y += 0.5 * a_lat * dt * dt;  // lateral pull
  return p;
}

// Build a noisy measurement at time t from a deterministic Rng (parity-preserving Gaussian).
Meas noisyMeas(const Vector3& p, Rng& rng) {
  Meas m = toMeas(p);
  m.az += rng.gaussian(0.0, kSigAng);
  m.el += rng.gaussian(0.0, kSigAng);
  m.range += rng.gaussian(0.0, kSigRng);
  return m;
}

}  // namespace

// ── A) NIS consistency across quiescent + maneuvering phases ──────────────────────────────────
TEST(Imm, NisConsistentAcrossManeuver) {
  Rng rng(20240614ULL);
  Imm imm(kDt, /*q_cv=*/0.5, /*q_man=*/3000.0, kSigAng, kSigAng, kSigRng, /*p_stay=*/0.999);

  const Vector3 p0{5000.0, 200.0, 1500.0};
  const Vector3 v0{-300.0, 20.0, -15.0};
  const double t_man = 3.0;   // maneuver begins at 3 s
  const double a_lat = 90.0;  // ~9 g lateral
  const int steps = 1200;     // 6 s

  double nis_sum = 0.0;
  int nis_n = 0;
  for (int k = 0; k <= steps; ++k) {
    const double t = k * kDt;
    const Vector3 p = trajectory(t, t_man, p0, v0, a_lat);
    if (k > 0) imm.predict(Vector3{});  // no own-accel control in this open-loop test
    const Meas m = noisyMeas(p, rng);
    imm.update(m.az, m.el, m.range);
    if (t > 0.5) {  // skip the bootstrap transient
      nis_sum += imm.nis();
      ++nis_n;
    }
  }
  const double mean_nis = nis_sum / nis_n;
  // A consistent 3-dof filter has E[NIS] = 3. The IMM stays in a reasonable band straddling the
  // maneuver; allow generous margins for the open-loop nonlinearity.
  EXPECT_GT(mean_nis, 1.0);
  EXPECT_LT(mean_nis, 8.0);
}

// ── B) Mode probability switches to the maneuver model during the maneuver ────────────────────
TEST(Imm, ModeProbabilitySwitchesOnManeuver) {
  Rng rng(777ULL);
  Imm imm(kDt, 0.5, 3000.0, kSigAng, kSigAng, kSigRng, 0.999);

  const Vector3 p0{6000.0, 0.0, 1800.0};
  const Vector3 v0{-280.0, 0.0, -20.0};
  const double t_man = 3.0;
  const double a_lat = 120.0;  // hard pull
  const int steps = 1200;

  double man_prob_quiescent = 0.0;  // sampled at ~2.5 s (still coasting)
  double man_prob_maneuver = 0.0;   // peak during the maneuver
  for (int k = 0; k <= steps; ++k) {
    const double t = k * kDt;
    const Vector3 p = trajectory(t, t_man, p0, v0, a_lat);
    if (k > 0) imm.predict(Vector3{});
    const Meas m = noisyMeas(p, rng);
    imm.update(m.az, m.el, m.range);

    if (std::fabs(t - 2.5) < 0.5 * kDt) man_prob_quiescent = imm.maneuverProbability();
    if (t > t_man && t < t_man + 1.0)
      man_prob_maneuver = std::max(man_prob_maneuver, imm.maneuverProbability());
  }
  // Coasting: the CV model dominates (low maneuver probability). Under maneuver the bank shifts
  // decisively toward the maneuver model.
  EXPECT_LT(man_prob_quiescent, 0.3);
  EXPECT_GT(man_prob_maneuver, 0.6);
  EXPECT_GT(man_prob_maneuver, man_prob_quiescent + 0.3);
}

// ── C) Determinism: same seed -> identical estimates and mode probabilities ────────────────────
TEST(Imm, DeterministicGivenSeed) {
  auto run = [](std::array<double, 6>& out_state, std::vector<double>& out_modes) {
    Rng rng(424242ULL);
    Imm imm(kDt, 0.5, 3000.0, kSigAng, kSigAng, kSigRng, 0.999);
    const Vector3 p0{4500.0, 100.0, 1600.0};
    const Vector3 v0{-260.0, 15.0, -18.0};
    for (int k = 0; k <= 1000; ++k) {
      const double t = k * kDt;
      const Vector3 p = trajectory(t, 2.5, p0, v0, 80.0);
      if (k > 0) imm.predict(Vector3{});
      const Meas m = noisyMeas(p, rng);
      imm.update(m.az, m.el, m.range);
    }
    const Vector3 rp = imm.relPos(), rv = imm.relVel();
    out_state = {rp.x, rp.y, rp.z, rv.x, rv.y, rv.z};
    out_modes = imm.modeProbabilities();
  };
  std::array<double, 6> a{}, b{};
  std::vector<double> ma, mb;
  run(a, ma);
  run(b, mb);
  for (int i = 0; i < 6; ++i) EXPECT_EQ(a[i], b[i]);
  ASSERT_EQ(ma.size(), mb.size());
  for (std::size_t i = 0; i < ma.size(); ++i) EXPECT_EQ(ma[i], mb[i]);
}

// ── D) IMM beats a single quiescent-tuned CV EKF on a maneuvering target ──────────────────────
TEST(Imm, OutperformsSingleEkfUnderManeuver) {
  const Vector3 p0{5500.0, 0.0, 1700.0};
  const Vector3 v0{-290.0, 0.0, -20.0};
  const double t_man = 3.0;
  const double a_lat = 110.0;
  const int steps = 1200;

  // Same seed -> same measurement noise sequence for both filters (fair comparison).
  auto avgLateErr = [&](bool use_imm) {
    Rng rng(135790ULL);
    Imm imm(kDt, 0.5, 3000.0, kSigAng, kSigAng, kSigRng, 0.999);
    Ekf ekf(kDt, /*q=*/5.0, kSigAng, kSigAng, kSigRng);  // tuned for quiescent flight
    double err_sum = 0.0;
    int n = 0;
    for (int k = 0; k <= steps; ++k) {
      const double t = k * kDt;
      const Vector3 p = trajectory(t, t_man, p0, v0, a_lat);
      const Meas m = noisyMeas(p, rng);
      if (use_imm) {
        if (k > 0) imm.predict(Vector3{});
        imm.update(m.az, m.el, m.range);
        if (t > t_man) {
          err_sum += (imm.relPos() - p).norm();
          ++n;
        }
      } else {
        if (k > 0) ekf.predict(Vector3{});
        ekf.update(m.az, m.el, m.range);
        if (t > t_man) {
          err_sum += (ekf.relPos() - p).norm();
          ++n;
        }
      }
    }
    return err_sum / n;
  };

  const double imm_err = avgLateErr(true);
  const double ekf_err = avgLateErr(false);
  // The IMM's maneuver model keeps up with the lateral pull; the single quiescent EKF lags badly.
  EXPECT_LT(imm_err, ekf_err);
}
