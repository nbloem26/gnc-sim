// gnc-sim — Interacting Multiple Model estimator (see Imm.hpp). World frame ENU, SI units.
//
// Each ImmModel is a 6-state nearly-constant-velocity EKF with the same sparse F = [[I, dt I],[0,
// I]] propagation and the same nonlinear az/el/range measurement as Ekf.cpp; the only per-model
// difference is the white-noise-acceleration process PSD q. All matrices are fixed-size, row-major
// std::array and the arithmetic is done in a fixed order, so native and WASM stay bit-identical.
//
// The IMM layer adds: interaction/mixing of the per-model estimates (weighted by the Markov
// transition matrix and the current mode probabilities), independent per-model measurement updates
// returning Gaussian likelihoods, a mode-probability update, and a probability-weighted
// recombination of the bank into the fused output. No RNG, no std distributions — pure
// deterministic arithmetic.
#include "gncsim/gnc/Imm.hpp"

#include <cmath>

namespace gncsim {

namespace {

constexpr double kPi = 3.14159265358979323846;

double wrapPi(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

// 3x3 inverse via cofactors; also returns the determinant (needed for the Gaussian likelihood).
// Matches Ekf.cpp's invert3x3 exactly so the shared update math is bit-identical.
bool invert3x3(const std::array<double, 9>& m, std::array<double, 9>& out, double& det_out) {
  const double a = m[0], b = m[1], c = m[2];
  const double d = m[3], e = m[4], f = m[5];
  const double g = m[6], h = m[7], i = m[8];

  const double A = e * i - f * h;
  const double B = -(d * i - f * g);
  const double C = d * h - e * g;
  const double det = a * A + b * B + c * C;
  det_out = det;
  if (std::fabs(det) < 1e-300) return false;
  const double inv_det = 1.0 / det;

  out[0] = A * inv_det;
  out[1] = (c * h - b * i) * inv_det;
  out[2] = (b * f - c * e) * inv_det;
  out[3] = B * inv_det;
  out[4] = (a * i - c * g) * inv_det;
  out[5] = (c * d - a * f) * inv_det;
  out[6] = C * inv_det;
  out[7] = (b * g - a * h) * inv_det;
  out[8] = (a * e - b * d) * inv_det;
  return true;
}

}  // namespace

// =============================================================================================
// ImmModel — a single nearly-constant-velocity EKF in the bank
// =============================================================================================

ImmModel::ImmModel(double dt, double q, double sigma_az, double sigma_el, double sigma_range)
    : dt_(dt),
      q_(q),
      r_diag_{sigma_az * sigma_az, sigma_el * sigma_el, sigma_range * sigma_range} {}

void ImmModel::setState(const std::array<double, 6>& x, const std::array<double, 36>& p) {
  x_ = x;
  p_ = p;
  initialized_ = true;
}

// ── Time update (identical structure to Ekf::predict) ─────────────────────────────────────────
void ImmModel::predict(const Vector3& a_vehicle) {
  if (!initialized_) return;

  const double ux = -a_vehicle.x, uy = -a_vehicle.y, uz = -a_vehicle.z;
  const double dt = dt_;
  const double half_dt2 = 0.5 * dt * dt;

  x_[0] += x_[3] * dt + half_dt2 * ux;
  x_[1] += x_[4] * dt + half_dt2 * uy;
  x_[2] += x_[5] * dt + half_dt2 * uz;
  x_[3] += ux * dt;
  x_[4] += uy * dt;
  x_[5] += uz * dt;

  auto P = [&](int row, int col) -> double& { return p_[row * 6 + col]; };

  std::array<double, 36> fp{};
  for (int j = 0; j < 6; ++j) {
    for (int i = 0; i < 3; ++i) fp[i * 6 + j] = P(i, j) + dt * P(i + 3, j);
    for (int i = 3; i < 6; ++i) fp[i * 6 + j] = P(i, j);
  }
  std::array<double, 36> fpft{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 3; ++j) fpft[i * 6 + j] = fp[i * 6 + j] + dt * fp[i * 6 + (j + 3)];
    for (int j = 3; j < 6; ++j) fpft[i * 6 + j] = fp[i * 6 + j];
  }

  const double q_pp = q_ * dt * dt * dt / 3.0;
  const double q_pv = q_ * dt * dt / 2.0;
  const double q_vv = q_ * dt;
  for (int k = 0; k < 36; ++k) p_[k] = fpft[k];
  for (int a = 0; a < 3; ++a) {
    P(a, a) += q_pp;
    P(a + 3, a + 3) += q_vv;
    P(a, a + 3) += q_pv;
    P(a + 3, a) += q_pv;
  }
}

// ── Measurement update (identical structure to Ekf::update; returns the Gaussian likelihood) ──
double ImmModel::update(double az, double el, double range) {
  if (!initialized_) {
    const double cos_el = std::cos(el);
    x_[0] = range * cos_el * std::cos(az);
    x_[1] = range * cos_el * std::sin(az);
    x_[2] = range * std::sin(el);
    x_[3] = x_[4] = x_[5] = 0.0;

    for (int k = 0; k < 36; ++k) p_[k] = 0.0;
    auto P0 = [&](int idx) -> double& { return p_[idx * 6 + idx]; };
    const double pos_var = 1.0e6;
    const double vel_var = 1.0e6;
    P0(0) = P0(1) = P0(2) = pos_var;
    P0(3) = P0(4) = P0(5) = vel_var;

    initialized_ = true;
    nis_ = 0.0;
    return 1.0;  // neutral likelihood on the bootstrap step
  }

  const double px = x_[0], py = x_[1], pz = x_[2];
  const double rho2 = px * px + py * py;
  double rho = std::sqrt(rho2);
  const double r2 = rho2 + pz * pz;
  double r = std::sqrt(r2);
  if (rho < 1e-9 || r < 1e-9) {
    nis_ = 0.0;
    return 1.0;
  }

  const double az_pred = std::atan2(py, px);
  const double el_pred = std::atan2(pz, rho);
  const double r_pred = r;

  std::array<double, 18> H{};
  H[0] = -py / rho2;
  H[1] = px / rho2;
  H[2] = 0.0;
  H[6] = -px * pz / (r2 * rho);
  H[7] = -py * pz / (r2 * rho);
  H[8] = rho / r2;
  H[12] = px / r;
  H[13] = py / r;
  H[14] = pz / r;

  auto P = [&](int row, int col) -> double { return p_[row * 6 + col]; };

  std::array<double, 18> PHt{};  // 6x3
  for (int i = 0; i < 6; ++i) {
    for (int k = 0; k < 3; ++k) {
      double s = 0.0;
      for (int j = 0; j < 6; ++j) s += P(i, j) * H[k * 6 + j];
      PHt[i * 3 + k] = s;
    }
  }

  std::array<double, 9> S{};
  for (int a = 0; a < 3; ++a) {
    for (int b = 0; b < 3; ++b) {
      double s = 0.0;
      for (int j = 0; j < 6; ++j) s += H[a * 6 + j] * PHt[j * 3 + b];
      S[a * 3 + b] = s;
    }
    S[a * 3 + a] += r_diag_[a];
  }

  std::array<double, 9> Sinv{};
  double detS = 0.0;
  if (!invert3x3(S, Sinv, detS)) {
    nis_ = 0.0;
    return 1.0;
  }

  std::array<double, 18> K{};  // 6x3
  for (int i = 0; i < 6; ++i) {
    for (int b = 0; b < 3; ++b) {
      double s = 0.0;
      for (int a = 0; a < 3; ++a) s += PHt[i * 3 + a] * Sinv[a * 3 + b];
      K[i * 3 + b] = s;
    }
  }

  std::array<double, 3> y{wrapPi(az - az_pred), wrapPi(el - el_pred), range - r_pred};

  double nis = 0.0;
  for (int a = 0; a < 3; ++a) {
    double s = 0.0;
    for (int b = 0; b < 3; ++b) s += Sinv[a * 3 + b] * y[b];
    nis += y[a] * s;
  }
  nis_ = nis;

  for (int i = 0; i < 6; ++i) {
    x_[i] += K[i * 3 + 0] * y[0] + K[i * 3 + 1] * y[1] + K[i * 3 + 2] * y[2];
  }

  std::array<double, 36> A{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      double kh = 0.0;
      for (int a = 0; a < 3; ++a) kh += K[i * 3 + a] * H[a * 6 + j];
      A[i * 6 + j] = (i == j ? 1.0 : 0.0) - kh;
    }
  }
  std::array<double, 36> AP{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      double s = 0.0;
      for (int k = 0; k < 6; ++k) s += A[i * 6 + k] * P(k, j);
      AP[i * 6 + j] = s;
    }
  }
  std::array<double, 36> APAt{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      double s = 0.0;
      for (int k = 0; k < 6; ++k) s += AP[i * 6 + k] * A[j * 6 + k];
      APAt[i * 6 + j] = s;
    }
  }
  std::array<double, 36> KRKt{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      double s = 0.0;
      for (int a = 0; a < 3; ++a) s += K[i * 3 + a] * r_diag_[a] * K[j * 3 + a];
      KRKt[i * 6 + j] = s;
    }
  }
  for (int k = 0; k < 36; ++k) p_[k] = APAt[k] + KRKt[k];

  // Gaussian measurement likelihood Λ = exp(-NIS/2) / sqrt((2π)^3 |S|). The IMM only needs relative
  // likelihoods across models (the normalizer cancels in the mode-probability update), but we keep
  // the full form so the absolute scale is correct and small-likelihood models are properly damped.
  const double two_pi = 2.0 * kPi;
  const double denom = std::sqrt(two_pi * two_pi * two_pi * std::fabs(detS));
  if (denom < 1e-300) return 1e-300;
  double like = std::exp(-0.5 * nis) / denom;
  if (like < 1e-300) like = 1e-300;  // floor so a stale model can always recover
  return like;
}

// =============================================================================================
// Imm — the interacting multiple-model orchestration
// =============================================================================================

Imm::Imm(double dt, double q_cv, double q_man, double sigma_az, double sigma_el, double sigma_range,
         double p_stay)
    : dt_(dt) {
  models_.emplace_back(dt, q_cv, sigma_az, sigma_el, sigma_range);   // index 0: constant-velocity
  models_.emplace_back(dt, q_man, sigma_az, sigma_el, sigma_range);  // index 1: maneuver

  const std::size_t n = models_.size();
  // Start biased toward the constant-velocity model (targets are quiescent more often than not).
  mu_.assign(n, 0.0);
  mu_[0] = 0.8;
  for (std::size_t j = 1; j < n; ++j) mu_[j] = 0.2 / static_cast<double>(n - 1);

  // Sticky Markov transition matrix: stay with probability p_stay, split the rest evenly.
  trans_.assign(n, std::vector<double>(n, 0.0));
  const double off = (n > 1) ? (1.0 - p_stay) / static_cast<double>(n - 1) : 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < n; ++j) trans_[i][j] = (i == j) ? p_stay : off;
  }
}

void Imm::predict(const Vector3& a_vehicle) {
  if (!initialized_) return;

  const std::size_t n = models_.size();

  // --- Interaction / mixing -------------------------------------------------------------------
  // Predicted mode probabilities cbar[j] = Σ_i trans[i][j] μ[i], and mixing weights
  // ω[i|j] = trans[i][j] μ[i] / cbar[j]. Each model j is reseeded with the mixed estimate
  // (mean + spread-corrected covariance) before its own predict.
  std::vector<double> cbar(n, 0.0);
  for (std::size_t j = 0; j < n; ++j) {
    for (std::size_t i = 0; i < n; ++i) cbar[j] += trans_[i][j] * mu_[i];
  }

  std::vector<std::array<double, 6>> mixed_x(n);
  std::vector<std::array<double, 36>> mixed_p(n);
  for (std::size_t j = 0; j < n; ++j) {
    std::array<double, 6> xj{};
    if (cbar[j] > 1e-300) {
      for (std::size_t i = 0; i < n; ++i) {
        const double w = trans_[i][j] * mu_[i] / cbar[j];
        const auto& xi = models_[i].state();
        for (int k = 0; k < 6; ++k) xj[k] += w * xi[k];
      }
    } else {
      xj = models_[j].state();
    }
    std::array<double, 36> pj{};
    if (cbar[j] > 1e-300) {
      for (std::size_t i = 0; i < n; ++i) {
        const double w = trans_[i][j] * mu_[i] / cbar[j];
        const auto& xi = models_[i].state();
        const auto& pi = models_[i].cov();
        std::array<double, 6> d{};
        for (int k = 0; k < 6; ++k) d[k] = xi[k] - xj[k];
        for (int a = 0; a < 6; ++a) {
          for (int b = 0; b < 6; ++b) pj[a * 6 + b] += w * (pi[a * 6 + b] + d[a] * d[b]);
        }
      }
    } else {
      pj = models_[j].cov();
    }
    mixed_x[j] = xj;
    mixed_p[j] = pj;
  }
  for (std::size_t j = 0; j < n; ++j) models_[j].setState(mixed_x[j], mixed_p[j]);

  // --- Per-model time update ------------------------------------------------------------------
  for (auto& m : models_) m.predict(a_vehicle);
}

void Imm::update(double az, double el, double range) {
  const std::size_t n = models_.size();

  if (!initialized_) {
    // Bootstrap every model from the first measurement, keep the prior mode probabilities.
    for (auto& m : models_) m.update(az, el, range);
    initialized_ = true;
    recombine();
    nis_ = 0.0;
    return;
  }

  // --- Per-model measurement update + likelihoods ---------------------------------------------
  std::vector<double> like(n, 0.0);
  for (std::size_t j = 0; j < n; ++j) like[j] = models_[j].update(az, el, range);

  // --- Mode-probability update ----------------------------------------------------------------
  // μ[j] ∝ Λ[j] * cbar[j], with cbar[j] = Σ_i trans[i][j] μ[i] (the predicted mode probability).
  std::vector<double> cbar(n, 0.0);
  for (std::size_t j = 0; j < n; ++j) {
    for (std::size_t i = 0; i < n; ++i) cbar[j] += trans_[i][j] * mu_[i];
  }
  double norm = 0.0;
  std::vector<double> mu_new(n, 0.0);
  for (std::size_t j = 0; j < n; ++j) {
    mu_new[j] = like[j] * cbar[j];
    norm += mu_new[j];
  }
  if (norm > 1e-300) {
    for (std::size_t j = 0; j < n; ++j) mu_[j] = mu_new[j] / norm;
  }  // else keep the prior μ (degenerate likelihoods)

  // --- Combination ----------------------------------------------------------------------------
  recombine();

  // Combined NIS: the mode-probability-weighted average of the per-model innovation NIS. A
  // consistent IMM keeps this near the measurement dof (3) across both quiescent and maneuver
  // phases — that is the headline improvement over a single CV EKF, which spikes during maneuvers.
  double nis = 0.0;
  for (std::size_t j = 0; j < n; ++j) nis += mu_[j] * models_[j].nis();
  nis_ = nis;
}

void Imm::recombine() {
  const std::size_t n = models_.size();
  x_ = std::array<double, 6>{};
  for (std::size_t j = 0; j < n; ++j) {
    const auto& xj = models_[j].state();
    for (int k = 0; k < 6; ++k) x_[k] += mu_[j] * xj[k];
  }
  p_ = std::array<double, 36>{};
  for (std::size_t j = 0; j < n; ++j) {
    const auto& xj = models_[j].state();
    const auto& pj = models_[j].cov();
    std::array<double, 6> d{};
    for (int k = 0; k < 6; ++k) d[k] = xj[k] - x_[k];
    for (int a = 0; a < 6; ++a) {
      for (int b = 0; b < 6; ++b) p_[a * 6 + b] += mu_[j] * (pj[a * 6 + b] + d[a] * d[b]);
    }
  }
}

}  // namespace gncsim
