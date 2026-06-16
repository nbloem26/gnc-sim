// gnc-sim — threat-suite models (issue #42). Concrete IThreat variants for advanced threats:
//   - IcbmThreat       : multi-stage boosting ICBM (staging events, ballistic midcourse)
//   - HgvThreat        : hypersonic glide vehicle (lift/drag skip-trajectory dynamics)
//   - RvPenaidsThreat  : ballistic reentry vehicle + a midcourse penaid/decoy spread
//
// These are exposed in a public header (rather than the anonymous namespace in Registry.cpp, where
// the legacy constant/weave threats live) so the dedicated GoogleTest suite can construct them and
// query their characteristic features directly — mass schedule + staging times for the ICBM, skip
// oscillation for the HGV, penaid deployment + scoring for the RV.
//
// All three implement IThreat::accel(tgt, t) as a PURE function of the target's current state and
// time plus the threat config. The Runner integrates the returned acceleration with its existing
// Euler step, so nothing about the per-step orchestration changes. Because the flat-Earth target
// propagation does not otherwise apply gravity to the target, these threats include gravity in
// their returned acceleration (the legacy constant/weave threats return only the maneuver accel and
// fly straight — unchanged). No RNG, no I/O: deterministic and WASM-parity-safe.
#pragma once

#include <vector>

#include "gncsim/core/Config.hpp"
#include "gncsim/math/Vector3.hpp"
#include "gncsim/model/Interfaces.hpp"

namespace gncsim {

// ---------------------------------------------------------------------------------------------
// Multi-stage boosting ICBM
// ---------------------------------------------------------------------------------------------
//
// Stage stack burns in order. While stage k is burning, thrust acts along the velocity direction
// (or +x if near-stationary) with magnitude thrust_n_k; the vehicle mass decreases linearly as the
// stage's propellant is consumed. At each stage's burn-out the spent dry mass is dropped (the
// staging event) and the next stage ignites. After the last stage the threat coasts ballistically.
// Gravity (-g0 in +z) is always included so the boost-then-coast trajectory makes a realistic
// lofted ballistic arc with a high apogee and long downrange.
class IcbmThreat final : public IThreat {
 public:
  IcbmThreat(const IcbmConfig& cfg, double g0_mps2);

  // Total applied acceleration [m/s^2] = thrust/mass(t) along velocity + gravity.
  Vector3 accel(const EntityState& tgt, double t) const override;

  // Vehicle mass [kg] at time t: full stack minus burned propellant minus already-staged dry mass.
  // Exposed for the V&V benchmark (mass must drop by the stage dry mass at each staging time).
  double massAt(double t_s) const;

  // Burn-out / staging time [s] of stage k (cumulative), i.e. the instant stage k's dry mass drops.
  double stagingTimeS(std::size_t stage) const;

  std::size_t stageCount() const { return cfg_.stages.size(); }
  double totalBurnTimeS() const { return total_burn_time_s_; }

 private:
  IcbmConfig cfg_;
  double g0_mps2_;
  double initial_mass_kg_;              // full stacked launch mass
  double total_burn_time_s_;            // sum of all stage burn times
  std::vector<double> burnout_time_s_;  // cumulative burn-out time per stage
};

// ---------------------------------------------------------------------------------------------
// Hypersonic glide vehicle (skip-glide)
// ---------------------------------------------------------------------------------------------
//
// Below pull_up_alt_m the vehicle generates aerodynamic lift perpendicular to its velocity in the
// vertical plane, with magnitude (L/D)*drag, "up" (away from the planet). Drag opposes velocity,
// scaled by an exponential atmosphere and the ballistic coefficient. The lift pulls the descending
// vehicle back up out of the dense air; it arcs over ballistically, re-enters, and skips again —
// the characteristic damped altitude oscillation of a boost-glide weapon. Above the pull-up
// altitude the air is too thin to matter and the motion is effectively ballistic.
class HgvThreat final : public IThreat {
 public:
  HgvThreat(const HgvConfig& cfg, double g0_mps2);

  // Total applied acceleration [m/s^2] = gravity + drag (along -v) + lift (perp to v, in-plane).
  Vector3 accel(const EntityState& tgt, double t) const override;

  // Air density [kg/m^3] from the threat's exponential atmosphere at altitude alt_m. Exposed for
  // the benchmark.
  double densityAt(double alt_m) const;

 private:
  HgvConfig cfg_;
  double g0_mps2_;
};

// ---------------------------------------------------------------------------------------------
// Reentry vehicle + penaids
// ---------------------------------------------------------------------------------------------
//
// The true RV is purely ballistic (gravity only). The threat's accel() therefore returns gravity,
// exactly like a heavy non-maneuvering reentry vehicle. The penaid/decoy spread is a separate
// kinematic concept handled by the RvPenaids helper below, which deploys a cluster of lighter
// objects about the RV at the dispense time and scores how distinguishable they are from the true
// RV (heavier objects shed less speed) — the kinematic discrimination cue used by issue #6.
class RvPenaidsThreat final : public IThreat {
 public:
  RvPenaidsThreat(const RvPenaidsConfig& cfg, double g0_mps2);

  // Ballistic RV: gravity only.
  Vector3 accel(const EntityState& tgt, double t) const override;

 private:
  RvPenaidsConfig cfg_;
  double g0_mps2_;
};

// One deployed object in an RV+penaids scene: the true RV (index 0) or a penaid/decoy.
struct RvObject {
  EntityState state;  // ENU truth state
  bool is_true_rv;    // true only for the lethal reentry vehicle
  double decel_mps2;  // extra atmospheric deceleration (0 for the heavy RV; >0 for penaids)
};

// Deterministic RV+penaids scene + scoring helper (feeds the issue-#6 discrimination stack).
//
// build() places the true RV at index 0 and `penaid_count` penaids spread about it by a fixed
// (deterministic) velocity-dispersion pattern at deploy. propagate() advances every object
// ballistically under gravity, with penaids carrying an extra atmospheric deceleration so they
// separate from the heavy RV over time — the kinematic cue. scoreAgainstTrueRv() returns, for the
// current scene, the fraction of penaids whose kinematic deceleration feature is correctly judged
// MORE penaid-like than RV-like, i.e. how well the spread is discriminated from the true RV.
class RvPenaids {
 public:
  RvPenaids(const RvPenaidsConfig& cfg, double g0_mps2);

  // Build the deployed scene from the RV's state at the dispense instant (deterministic spread).
  std::vector<RvObject> deploy(const EntityState& rv) const;

  // Advance every object one step (gravity + per-object deceleration), in place.
  void propagate(std::vector<RvObject>& scene, double dt_s) const;

  // Fraction in [0,1] of penaids whose deceleration feature is correctly scored as penaid-like
  // (decel above the RV's). 1.0 = all penaids cleanly separable from the true RV.
  double scoreAgainstTrueRv(const std::vector<RvObject>& scene) const;

  bool deployed(double t_s) const { return t_s >= cfg_.deploy_time_s; }

 private:
  RvPenaidsConfig cfg_;
  double g0_mps2_;
};

}  // namespace gncsim
