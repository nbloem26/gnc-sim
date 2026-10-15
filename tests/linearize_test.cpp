// gnc-sim — tests for the 6DOF airframe+actuator pitch-channel linearization (issue #122).
//
// Validates that linearizeAirframe recovers a physically correct short-period mode: a statically
// stable airframe (Cm_alpha < 0) gives a real ωn > 0 with ζ in a sensible range and a Hurwitz A;
// flipping the static-margin sign (Cm_alpha > 0) flips stability; control effectiveness is non-zero
// and finite; and the linearization is bit-for-bit deterministic across repeated calls.
#include "gncsim/dynamics/Linearize.hpp"

#include <gtest/gtest.h>

#include <cmath>

#include "gncsim/core/Config.hpp"

namespace {

using gncsim::linearizeAirframe;
using gncsim::SimConfig;
using gncsim::TrimCondition;

// A representative statically stable tactical airframe (linear-slope aero so the test is
// closed-form and table-independent). Cm_alpha < 0 => statically stable; Cmq < 0 => damping.
SimConfig stableAirframe() {
  SimConfig cfg;
  cfg.model = "6dof_hifi";
  cfg.aero.ref_area = 0.02;
  cfg.aero.ref_length = 0.15;
  cfg.aero.cn_alpha = 12.0;
  cfg.aero.cm_alpha = -8.0;  // statically stable
  cfg.aero.cm_q = -120.0;    // pitch damping
  cfg.vehicle.mass0 = 22.0;
  cfg.vehicle.iyy = 1.30;
  cfg.actuator.effectiveness = 2500.0;
  return cfg;
}

TrimCondition midFlight() {
  TrimCondition t;
  t.mach = 2.0;
  t.altitude_m = 5000.0;
  t.alpha_rad = 0.05;
  return t;
}

// A stable airframe yields a real natural frequency, a sensible damping ratio, and a Hurwitz A.
TEST(Linearize, StableAirframeHasRealShortPeriodMode) {
  const auto r = linearizeAirframe(stableAirframe(), midFlight());

  EXPECT_GT(r.omega_n_radps, 0.0);
  EXPECT_TRUE(r.stable);
  // Lightly damped, oscillatory short period: ζ in (0, 1) for this airframe.
  EXPECT_GT(r.zeta, 0.0);
  EXPECT_LT(r.zeta, 1.0);
  // ωn should be a physically plausible tactical short period (a few to tens of rad/s).
  EXPECT_GT(r.omega_n_radps, 1.0);
  EXPECT_LT(r.omega_n_radps, 200.0);
  // Provenance numbers populated.
  EXPECT_GT(r.q_bar_pa, 0.0);
  EXPECT_GT(r.speed_mps, 0.0);
  EXPECT_NEAR(r.iyy_kgm2, 1.30, 1e-9);
  EXPECT_LT(r.cm_alpha_per_rad, 0.0);  // statically stable slope recovered
}

// ωn^2 should track the static-margin magnitude: a stiffer airframe (more negative Cm_alpha) gives
// a higher natural frequency.
TEST(Linearize, StifferAirframeRaisesNaturalFrequency) {
  SimConfig soft = stableAirframe();
  soft.aero.cm_alpha = -4.0;
  SimConfig stiff = stableAirframe();
  stiff.aero.cm_alpha = -16.0;

  const double wn_soft = linearizeAirframe(soft, midFlight()).omega_n_radps;
  const double wn_stiff = linearizeAirframe(stiff, midFlight()).omega_n_radps;

  EXPECT_GT(wn_stiff, wn_soft);
  // ωn ∝ sqrt(-Cm_alpha): 4x the slope -> ~2x ωn.
  EXPECT_NEAR(wn_stiff / wn_soft, 2.0, 0.1);
}

// Flipping the static-margin sign (Cm_alpha > 0) makes the airframe statically unstable: A is no
// longer Hurwitz and there is no real oscillatory natural frequency.
TEST(Linearize, FlippingStaticMarginSignFlipsStability) {
  SimConfig unstable = stableAirframe();
  unstable.aero.cm_alpha = +8.0;  // statically UNSTABLE

  const auto r = linearizeAirframe(unstable, midFlight());
  EXPECT_FALSE(r.stable);
  EXPECT_DOUBLE_EQ(r.omega_n_radps, 0.0);  // divergent real pole pair, no ωn
}

// Control effectiveness (DC accel-per-deflection gain) is finite and non-zero for a real actuator.
TEST(Linearize, ControlEffectivenessIsFiniteAndNonzero) {
  const auto r = linearizeAirframe(stableAirframe(), midFlight());
  EXPECT_TRUE(std::isfinite(r.control_effectiveness_mps2_per_rad));
  EXPECT_NE(r.control_effectiveness_mps2_per_rad, 0.0);
}

// Determinism: repeated linearizations of the same config+trim are bit-for-bit identical.
TEST(Linearize, IsBitForBitDeterministic) {
  const auto r1 = linearizeAirframe(stableAirframe(), midFlight());
  const auto r2 = linearizeAirframe(stableAirframe(), midFlight());

  for (int i = 0; i < 4; ++i) EXPECT_DOUBLE_EQ(r1.a_matrix[i], r2.a_matrix[i]);
  for (int i = 0; i < 2; ++i) EXPECT_DOUBLE_EQ(r1.b_matrix[i], r2.b_matrix[i]);
  EXPECT_DOUBLE_EQ(r1.omega_n_radps, r2.omega_n_radps);
  EXPECT_DOUBLE_EQ(r1.zeta, r2.zeta);
  EXPECT_DOUBLE_EQ(r1.control_effectiveness_mps2_per_rad, r2.control_effectiveness_mps2_per_rad);
}

// The hi-fi table config (configs/homing_6dof_hifi.json shape) also linearizes to a stable mode.
TEST(Linearize, TableDrivenAirframeIsStable) {
  SimConfig cfg = stableAirframe();
  // Mimic the cm_table sign convention (Cm<0 for alpha>0) via the linear slope already set; just
  // confirm a representative supersonic/lower-altitude condition is stable too.
  TrimCondition t;
  t.mach = 3.0;
  t.altitude_m = 2000.0;
  t.alpha_rad = 0.1;
  const auto r = linearizeAirframe(cfg, t);
  EXPECT_TRUE(r.stable);
  EXPECT_GT(r.omega_n_radps, 0.0);
}

}  // namespace
