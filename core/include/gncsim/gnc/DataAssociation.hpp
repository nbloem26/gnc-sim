// gnc-sim — multi-target data association + track lifecycle + track-to-track fusion (issue #38).
//
// The multi-tracker fusion path (#5) assumes a single, always-correctly-associated measurement per
// sensor per step: it hands every detection straight to the TargetTrackEkf. That is fine for a lone
// non-fluctuating target, but a real scene has DECOYS, closely-spaced objects, and CLUTTER false
// alarms — so the tracker must DECIDE which of several detections (if any) belongs to the target
// track before it updates. This module adds that missing data-association front-end:
//
//   * Joint Probabilistic Data Association (JPDA) — for a confirmed track, gate the candidate
//     detections, weight each by its measurement likelihood (plus a no-detection / clutter
//     hypothesis), and fold the probabilistic combination back into the EKF (PDA update with the
//     spread-of-the-means term). For a single confirmed track the JPDA marginal association
//     probabilities reduce to the classical PDA weights, which is what we compute; the joint
//     enumeration only matters when multiple tracks compete for the same detections, which we keep
//     as a bounded follow-up (MHT). The lifecycle below is what lets the true track survive a decoy
//     crossing: clutter/decoy returns are down-weighted, not blindly absorbed.
//
//   * Track LIFECYCLE — a track is INITIATED from an unassociated detection, CONFIRMED once it is
//     associated on M of the last N looks, and DELETED after a run of consecutive misses. This is
//     the standard M-of-N logic that keeps spurious clutter tracks from persisting.
//
//   * TRACK-TO-TRACK FUSION — combine two independent track estimates (e.g. a ground-radar track
//   and
//     a space-IR track of the same object) with Covariance Intersection, which is consistent under
//     UNKNOWN cross-correlation (the usual case when the two trackers share process noise / a
//     common target model). CI never underestimates the fused covariance, so the fused NIS stays
//     consistent.
//
// Everything here is fixed-size hand-rolled linear algebra on std::array + the project Rng (no
// Eigen, no std distributions, fixed FP order, libm only) so native and WASM stay bit-identical.
// The whole module is opt-in: it is reached only when trackers.association.mode == "jpda", so every
// existing config (including the #5 fused path) is byte-identical.
//
// Units carry SI suffixes per AGENTS.md (pos_m, vel_mps, gate_chi2 is dimensionless, density_per_m2
// / density_per_rad2 is the clutter spatial density in measurement space, ...).
#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "gncsim/gnc/TargetTrackEkf.hpp"
#include "gncsim/math/Vector3.hpp"

namespace gncsim {

// ---------------------------------------------------------------------------------------------
// JPDA / PDA configuration
// ---------------------------------------------------------------------------------------------

// Parameters for the probabilistic data associator. All dimensionless except the clutter density,
// which is a spatial density in MEASUREMENT space (returns per unit measurement volume) used by the
// non-parametric clutter model.
struct JpdaParams {
  double prob_detect = 0.9;       // P_D: probability the true target is detected on a given look
  double gate_chi2 = 16.0;        // validation-gate threshold on the measurement NIS (chi-square)
  double clutter_density = 1e-4;  // spatial density of clutter returns in measurement space lambda
};

// One detection presented to the associator for a single sensor look: the raw measurement vector
// (radar [az,el,range,range_rate] / IR [az,el]) and whether it originated (in truth) from the
// lethal target — used only by the test harness to score association PURITY; the associator itself
// never sees this label.
struct AssocDetection {
  std::vector<double> z;           // measurement vector
  bool from_target_truth = false;  // ground-truth origin label (scoring only; not used by JPDA)
};

// Result of associating one sensor look against one confirmed track.
struct JpdaResult {
  bool any_gated = false;       // did at least one detection fall inside the validation gate?
  double beta0 = 1.0;           // probability of the no-detection (all-clutter) hypothesis
  int best_index = -1;          // index (into the look's detections) of the highest-beta detection
  double best_beta = 0.0;       // that detection's association probability
  bool best_is_target = false;  // truth label of the best-beta detection (scoring only)
  double nis = 0.0;  // combined PDA innovation NIS of the applied update (0 if none applied)
};

// ---------------------------------------------------------------------------------------------
// JPDA associator (single confirmed track; PDA-marginal weights)
// ---------------------------------------------------------------------------------------------
//
// Gate the candidate detections against the track's predicted measurement, weight the validated
// ones by their Gaussian likelihood and the no-detection / clutter hypothesis, and apply the
// probabilistic combination to the EKF. `ekf` must already be predicted to this step. Returns the
// per-look association summary (used by the lifecycle manager + the tests).
JpdaResult jpdaAssociateAndUpdate(TargetTrackEkf& ekf, const TrackSensor& sensor,
                                  const std::vector<AssocDetection>& detections,
                                  const JpdaParams& params);

// ---------------------------------------------------------------------------------------------
// Track lifecycle (M-of-N confirmation, miss-run deletion)
// ---------------------------------------------------------------------------------------------

enum class TrackStatus { Tentative, Confirmed, Deleted };

// Parameters for the M-of-N track-lifecycle logic.
struct TrackLifecycleParams {
  int confirm_m = 3;      // need this many associations ...
  int confirm_n = 5;      // ... within the last N looks to CONFIRM a tentative track
  int delete_misses = 6;  // delete after this many CONSECUTIVE missed looks
};

// Sliding-window M-of-N track-lifecycle state machine. A new track starts Tentative; each look it
// is told whether it was associated (a gated detection assigned to it). It promotes to Confirmed
// once it has M associations in the last N looks, and to Deleted after `delete_misses` consecutive
// misses.
class TrackLifecycle {
 public:
  explicit TrackLifecycle(const TrackLifecycleParams& params);

  // Advance the lifecycle one look. `associated` = a detection was assigned to this track this
  // look. Returns the (possibly updated) status.
  TrackStatus update(bool associated);

  TrackStatus status() const { return status_; }
  int hits_in_window() const;  // associations within the current N-look window
  int consecutive_misses() const { return consecutive_misses_; }

 private:
  TrackLifecycleParams params_;
  TrackStatus status_ = TrackStatus::Tentative;
  std::vector<std::uint8_t> window_;  // ring buffer of the last N association flags (1 = hit)
  int head_ = 0;                      // next write index into the ring
  int filled_ = 0;                    // number of looks seen so far (caps at N)
  int consecutive_misses_ = 0;
};

// ---------------------------------------------------------------------------------------------
// Track-to-track fusion (Covariance Intersection)
// ---------------------------------------------------------------------------------------------

// One 6-state track estimate (pos, vel) with its 6x6 covariance (row-major), as produced by a
// TargetTrackEkf. Used as the input/output of the track-to-track fuser.
struct TrackEstimate {
  std::array<double, 6> x{};   // [pos(3), vel(3)] ENU, SI units
  std::array<double, 36> p{};  // 6x6 covariance, row-major

  double covTrace() const;
};

// Covariance Intersection of two track estimates with UNKNOWN cross-correlation. The fused estimate
// is the convex combination in information form:
//   P_f^-1 = w P_a^-1 + (1-w) P_b^-1,   x_f = P_f ( w P_a^-1 x_a + (1-w) P_b^-1 x_b ),
// with w in (0,1) chosen to MINIMISE trace(P_f) by a bounded golden-section search (deterministic,
// fixed iteration count — parity-safe). CI is guaranteed consistent (never optimistic) for any true
// cross-correlation, so the fused track's NIS stays statistically valid.
TrackEstimate covarianceIntersection(const TrackEstimate& a, const TrackEstimate& b);

// The weight w in (0,1) that CI would pick for these two estimates (exposed for tests).
double covarianceIntersectionWeight(const TrackEstimate& a, const TrackEstimate& b);

}  // namespace gncsim
