// gnc-sim — data association + lifecycle + track-to-track fusion implementation (issue #38).
// See DataAssociation.hpp for the model rationale. Fixed-size hand-rolled linear algebra on
// std::array; pure arithmetic + libm (no std distributions), fixed FP order, so native and WASM
// stay bit-identical.
#include "gncsim/gnc/DataAssociation.hpp"

#include <algorithm>
#include <cmath>

namespace gncsim {

namespace {

// In-place Gauss-Jordan inverse of an m x m matrix (m <= 6) with partial pivoting, row-major in a
// fixed 36-element array. Returns false if (near-)singular. Mirrors the EKF's invert but sized for
// the 6x6 information-form fusion.
bool invert6x6(std::array<double, 36>& a, int m, std::array<double, 36>& out) {
  std::array<double, 36> inv{};
  for (int i = 0; i < m; ++i) inv[i * m + i] = 1.0;
  for (int col = 0; col < m; ++col) {
    int pivot = col;
    double best = std::fabs(a[col * m + col]);
    for (int r = col + 1; r < m; ++r) {
      const double v = std::fabs(a[r * m + col]);
      if (v > best) {
        best = v;
        pivot = r;
      }
    }
    if (best < 1e-300) return false;
    if (pivot != col) {
      for (int k = 0; k < m; ++k) {
        std::swap(a[col * m + k], a[pivot * m + k]);
        std::swap(inv[col * m + k], inv[pivot * m + k]);
      }
    }
    const double diag = a[col * m + col];
    const double inv_diag = 1.0 / diag;
    for (int k = 0; k < m; ++k) {
      a[col * m + k] *= inv_diag;
      inv[col * m + k] *= inv_diag;
    }
    for (int r = 0; r < m; ++r) {
      if (r == col) continue;
      const double factor = a[r * m + col];
      if (factor == 0.0) continue;
      for (int k = 0; k < m; ++k) {
        a[r * m + k] -= factor * a[col * m + k];
        inv[r * m + k] -= factor * inv[col * m + k];
      }
    }
  }
  out = inv;
  return true;
}

// trace of a 6x6 row-major matrix.
double trace6(const std::array<double, 36>& p) {
  double s = 0.0;
  for (int i = 0; i < 6; ++i) s += p[i * 6 + i];
  return s;
}

}  // namespace

// =============================================================================================
// JPDA associator
// =============================================================================================

JpdaResult jpdaAssociateAndUpdate(TargetTrackEkf& ekf, const TrackSensor& sensor,
                                  const std::vector<AssocDetection>& detections,
                                  const JpdaParams& params) {
  JpdaResult out;
  if (!ekf.initialized() || detections.empty()) {
    out.beta0 = 1.0;
    return out;
  }

  // Gate each detection: keep those whose measurement NIS is within gate_chi2, recording the
  // Gaussian likelihood for the association weight.
  std::vector<std::vector<double>> gated_z;  // gated measurement vectors (for the PDA update)
  std::vector<double> likelihoods;           // matching Gaussian likelihoods
  std::vector<int> source_index;             // original detection index of each gated entry
  std::vector<bool> source_is_target;        // truth label of each gated entry (scoring only)

  for (int i = 0; i < static_cast<int>(detections.size()); ++i) {
    const AssocDetection& d = detections[static_cast<std::size_t>(i)];
    std::array<double, 4> y{};
    std::array<double, 16> s_inv{};
    int m = 0;
    double nis = 0.0;
    double like = 0.0;
    if (!ekf.innovation(sensor, d.z, y, s_inv, m, nis, &like)) continue;
    if (nis > params.gate_chi2) continue;
    gated_z.push_back(d.z);
    likelihoods.push_back(like);
    source_index.push_back(i);
    source_is_target.push_back(d.from_target_truth);
  }

  if (gated_z.empty()) {
    out.any_gated = false;
    out.beta0 = 1.0;
    return out;
  }
  out.any_gated = true;

  // PDA association probabilities (Bar-Shalom, non-parametric clutter model):
  //   weight of detection j     w_j  = P_D * L_j
  //   weight of no-detection    w_0  = (1 - P_D) * lambda
  // where L_j is the gated Gaussian likelihood and lambda the clutter spatial density. Normalise
  // so beta0 + sum(beta_j) = 1.
  const double pd = std::clamp(params.prob_detect, 1e-6, 1.0 - 1e-9);
  const double lambda = std::max(params.clutter_density, 1e-300);

  std::vector<double> w(gated_z.size(), 0.0);
  double w0 = (1.0 - pd) * lambda;
  double sum = w0;
  for (std::size_t j = 0; j < gated_z.size(); ++j) {
    w[j] = pd * likelihoods[j];
    sum += w[j];
  }
  if (sum <= 0.0) {
    out.beta0 = 1.0;
    return out;
  }
  const double inv_sum = 1.0 / sum;
  std::vector<double> betas(gated_z.size(), 0.0);
  out.beta0 = w0 * inv_sum;
  int best = -1;
  double best_beta = 0.0;
  for (std::size_t j = 0; j < gated_z.size(); ++j) {
    betas[j] = w[j] * inv_sum;
    if (betas[j] > best_beta) {
      best_beta = betas[j];
      best = static_cast<int>(j);
    }
  }
  if (best >= 0) {
    out.best_index = source_index[static_cast<std::size_t>(best)];
    out.best_beta = best_beta;
    out.best_is_target = source_is_target[static_cast<std::size_t>(best)];
  }

  // Apply the probabilistic combination to the EKF.
  out.nis = ekf.updatePda(sensor, gated_z, betas, out.beta0);
  return out;
}

// =============================================================================================
// Track lifecycle
// =============================================================================================

TrackLifecycle::TrackLifecycle(const TrackLifecycleParams& params)
    : params_(params), window_(static_cast<std::size_t>(std::max(params.confirm_n, 1)), 0u) {}

int TrackLifecycle::hits_in_window() const {
  int h = 0;
  for (std::uint8_t v : window_) h += (v != 0u) ? 1 : 0;
  return h;
}

TrackStatus TrackLifecycle::update(bool associated) {
  if (status_ == TrackStatus::Deleted) return status_;

  window_[static_cast<std::size_t>(head_)] = associated ? 1u : 0u;
  head_ = (head_ + 1) % static_cast<int>(window_.size());
  if (filled_ < static_cast<int>(window_.size())) ++filled_;

  if (associated) {
    consecutive_misses_ = 0;
  } else {
    ++consecutive_misses_;
  }

  if (status_ == TrackStatus::Tentative && hits_in_window() >= params_.confirm_m) {
    status_ = TrackStatus::Confirmed;
  }
  if (consecutive_misses_ >= params_.delete_misses) {
    status_ = TrackStatus::Deleted;
  }
  return status_;
}

// =============================================================================================
// Track-to-track fusion (Covariance Intersection)
// =============================================================================================

double TrackEstimate::covTrace() const { return trace6(p); }

namespace {

// Fuse two estimates by CI at a fixed weight w in (0,1): P_f^-1 = w Pa^-1 + (1-w) Pb^-1,
// x_f = P_f (w Pa^-1 xa + (1-w) Pb^-1 xb). Returns false (and leaves `out` untouched) if a
// covariance is singular.
bool ciAtWeight(const TrackEstimate& a, const TrackEstimate& b, double w, TrackEstimate& out) {
  std::array<double, 36> pa = a.p;
  std::array<double, 36> pb = b.p;
  std::array<double, 36> ia{};
  std::array<double, 36> ib{};
  if (!invert6x6(pa, 6, ia)) return false;
  if (!invert6x6(pb, 6, ib)) return false;

  std::array<double, 36> info{};
  for (int k = 0; k < 36; ++k) info[k] = w * ia[k] + (1.0 - w) * ib[k];

  std::array<double, 36> info_copy = info;
  std::array<double, 36> pf{};
  if (!invert6x6(info_copy, 6, pf)) return false;

  // Information-weighted mean: y = w Pa^-1 xa + (1-w) Pb^-1 xb, then x_f = P_f y.
  std::array<double, 6> y{};
  for (int i = 0; i < 6; ++i) {
    double sa = 0.0;
    double sb = 0.0;
    for (int j = 0; j < 6; ++j) {
      sa += ia[i * 6 + j] * a.x[static_cast<std::size_t>(j)];
      sb += ib[i * 6 + j] * b.x[static_cast<std::size_t>(j)];
    }
    y[static_cast<std::size_t>(i)] = w * sa + (1.0 - w) * sb;
  }
  std::array<double, 6> xf{};
  for (int i = 0; i < 6; ++i) {
    double s = 0.0;
    for (int j = 0; j < 6; ++j) s += pf[i * 6 + j] * y[static_cast<std::size_t>(j)];
    xf[static_cast<std::size_t>(i)] = s;
  }
  out.x = xf;
  out.p = pf;
  return true;
}

}  // namespace

double covarianceIntersectionWeight(const TrackEstimate& a, const TrackEstimate& b) {
  // Golden-section search for w in (0,1) minimising trace(P_f(w)). Deterministic, fixed iteration
  // count (parity-safe). If a fusion fails (singular), fall back to the lower-trace single
  // estimate.
  const double inv_phi = 0.6180339887498949;  // 1/golden ratio
  double lo = 0.0;
  double hi = 1.0;
  auto cost = [&](double w) -> double {
    TrackEstimate f;
    if (!ciAtWeight(a, b, w, f)) return 1e300;
    return f.covTrace();
  };
  double c = hi - inv_phi * (hi - lo);
  double d = lo + inv_phi * (hi - lo);
  double fc = cost(c);
  double fd = cost(d);
  for (int it = 0; it < 40; ++it) {
    if (fc < fd) {
      hi = d;
      d = c;
      fd = fc;
      c = hi - inv_phi * (hi - lo);
      fc = cost(c);
    } else {
      lo = c;
      c = d;
      fc = fd;
      d = lo + inv_phi * (hi - lo);
      fd = cost(d);
    }
  }
  const double w_opt = 0.5 * (lo + hi);
  // When the cost is (near-)flat in w — e.g. two equally-informative inputs — the search converges
  // to an arbitrary endpoint, which would discard one estimate. Prefer the symmetric interior
  // weight w=0.5 whenever it is within a small relative tolerance of the found optimum, so equal
  // inputs are genuinely averaged (and the result never degenerates to a single input by accident).
  const double f_opt = cost(w_opt);
  const double f_half = cost(0.5);
  if (f_half <= f_opt * (1.0 + 1e-6)) return 0.5;
  return w_opt;
}

TrackEstimate covarianceIntersection(const TrackEstimate& a, const TrackEstimate& b) {
  const double w = covarianceIntersectionWeight(a, b);
  TrackEstimate out;
  if (ciAtWeight(a, b, w, out)) return out;
  // Singular fallback: return whichever input has the smaller covariance trace.
  return (a.covTrace() <= b.covTrace()) ? a : b;
}

}  // namespace gncsim
