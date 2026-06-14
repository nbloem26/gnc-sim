// gnc-sim — feature-based target discriminator (see Discriminator.hpp).
#include "gncsim/gnc/Discriminator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace gncsim {

namespace {

// Per-feature variance used by the Mahalanobis-style scoring. The expected scatter of a TRUE-target
// measurement around the target signature is the combination of the static per-object signature
// spread and the per-step seeker measurement noise. Inverse of this is the feature weight, so noisy
// / highly-variable features count less in the score. Floored so the weight never blows up.
double featureVariance(const DecoysConfig& cfg) {
  const double v =
      cfg.feature_spread * cfg.feature_spread + cfg.measurement_noise * cfg.measurement_noise;
  return std::max(v, 1e-6);
}

}  // namespace

Discriminator::Discriminator(const DecoysConfig& cfg, int num_objects, int target_index)
    : num_objects_(std::max(num_objects, 1)),
      target_index_(std::clamp(target_index, 0, std::max(num_objects, 1) - 1)),
      target_sig_{cfg.target_intensity, cfg.target_size, cfg.target_decel} {
  const double var = featureVariance(cfg);
  const double w = 1.0 / var;
  weights_ = {w, w, w};

  // First-order low-pass blend for temporal score integration: blend = dt/tau clamped to 1. The
  // discriminator is sampled once per sim step; we fold the step cadence into score_filter_tau by
  // treating tau as "number of steps' worth of smoothing" via the per-step blend below. To keep the
  // module independent of dt we accept score_filter_tau already in seconds and let the runner-side
  // cadence (dt) be captured here is unavailable; instead use a fixed-fraction blend derived purely
  // from tau treated as a unitless smoothing horizon in steps (>=1 step).
  const double horizon_steps = std::max(cfg.score_filter_tau, 0.0) * 200.0;  // ~dt=0.005 default
  blend_ = horizon_steps > 1.0 ? 1.0 / horizon_steps : 1.0;

  scores_.assign(static_cast<std::size_t>(num_objects_), 0.0);
  selected_ = target_index_;  // default selection before any observation
}

double Discriminator::instantScore(const FeatureVec& z) const {
  double s = 0.0;
  for (int k = 0; k < 3; ++k) {
    const double d = z[static_cast<std::size_t>(k)] - target_sig_[static_cast<std::size_t>(k)];
    s -= weights_[static_cast<std::size_t>(k)] * d * d;
  }
  return s;
}

void Discriminator::observe(const std::vector<FeatureVec>& z) {
  const std::size_t n = static_cast<std::size_t>(num_objects_);
  if (z.size() != n) return;  // defensive: caller contract is one feature vec per object

  for (std::size_t i = 0; i < n; ++i) {
    const double inst = instantScore(z[i]);
    if (!have_prev_) {
      scores_[i] = inst;  // seed the integrator with the first measurement
    } else {
      scores_[i] += (inst - scores_[i]) * blend_;  // first-order temporal integration
    }
  }
  have_prev_ = true;

  // Select the highest-scoring object; compute the margin to the runner-up for confidence.
  int best = 0;
  for (std::size_t i = 1; i < n; ++i) {
    if (scores_[i] > scores_[static_cast<std::size_t>(best)]) best = static_cast<int>(i);
  }
  selected_ = best;

  if (n >= 2) {
    double second = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < n; ++i) {
      if (static_cast<int>(i) == best) continue;
      second = std::max(second, scores_[i]);
    }
    margin_ = scores_[static_cast<std::size_t>(best)] - second;
  } else {
    margin_ = 0.0;
  }
}

}  // namespace gncsim
