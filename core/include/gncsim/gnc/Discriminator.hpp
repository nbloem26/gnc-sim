// gnc-sim — seeker target discrimination against decoys / closely-spaced objects (issue #6).
//
// A real IR/RF seeker resolving multiple objects in its field of view must decide WHICH object is
// the lethal target before committing terminal guidance. Decoys (flares, chaff, light replicas) are
// cheap and plentiful; the seeker discriminates the true warhead from them on FEATURES — IR
// intensity, apparent size, and kinematics (a light decoy has a lower ballistic coefficient and
// decelerates faster than the heavy warhead). This module is the feature-based discriminator.
//
// Each object carries a static 3-feature SIGNATURE [intensity, size, decel]. Every step the seeker
// produces a NOISY measurement of each object's signature; the discriminator scores each object by
// how close its measured features are to the expected lethal-target signature (a Mahalanobis-style
// weighted squared distance, negated so higher = more target-like), integrates that score over time
// with a first-order low-pass (temporal integration beats the per-step measurement noise), and
// SELECTS the highest-scoring object. Guidance then homes on the selected object — if that's a
// decoy, the resulting miss is large.
//
// Hand-rolled arithmetic, no external deps. Deterministic given the same measurement stream, so the
// native<->WASM parity holds.
#pragma once

#include <array>
#include <vector>

#include "gncsim/core/Config.hpp"

namespace gncsim {

// A measured (or true) object feature signature. Order: [intensity, size, decel].
using FeatureVec = std::array<double, 3>;

// Feature-based target discriminator. Construct once per run with the configured decoy/target
// feature model; call observe() each step with the per-object measured features; read selected().
class Discriminator {
 public:
  // num_objects includes the true target (index target_index) plus the decoys.
  Discriminator(const DecoysConfig& cfg, int num_objects, int target_index);

  // Feed this step's noisy measured feature vectors (one per object, indexed 0..num_objects-1).
  // Updates each object's time-integrated score and re-selects the highest-scoring object. `z` must
  // have exactly num_objects entries.
  void observe(const std::vector<FeatureVec>& z);

  int selected() const { return selected_; }  // index of the object guidance should home on
  int targetIndex() const { return target_index_; }
  bool correct() const { return selected_ == target_index_; }  // selected the true target?

  // Margin = (best score) - (second-best score). Larger -> more confident discrimination. 0 when
  // there is only one object or before the first observation.
  double margin() const { return margin_; }

  double score(int obj) const { return scores_[static_cast<std::size_t>(obj)]; }
  int numObjects() const { return num_objects_; }

  // Instantaneous (un-integrated) per-feature-weighted score of a measured signature against the
  // expected target signature: -sum_k w_k * (z_k - mu_k)^2. Higher = more target-like. Exposed for
  // unit testing the scoring kernel directly.
  double instantScore(const FeatureVec& z) const;

 private:
  int num_objects_;
  int target_index_;
  FeatureVec target_sig_{};  // expected lethal-target signature [intensity, size, decel]
  FeatureVec weights_{};     // per-feature inverse-variance weights
  double blend_;             // per-step low-pass blend factor for temporal score integration

  std::vector<double> scores_;  // time-integrated score per object
  bool have_prev_ = false;
  int selected_ = 0;
  double margin_ = 0.0;
};

}  // namespace gncsim
