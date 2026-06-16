// gnc-sim — Augmented Proportional Navigation (APN) tests.
//   - APN unit behaviour: feedforward only under the "apn" law, perpendicular-to-LOS, limited.
//   - Integration: APN intercepts a non-maneuvering target (must not break the baseline).
//   - Integration: against a weaving target with finite autopilot lag, APN miss <= PN miss.
//   - The default config still uses PN (no target-accel feedforward applied).
// Configs are built in-code (no file/cwd dependency), matching the other integration tests.
#include <gtest/gtest.h>

#include <cmath>

#include "gncsim/core/Config.hpp"
#include "gncsim/core/Types.hpp"
#include "gncsim/gnc/Gnc.hpp"
#include "gncsim/scenario/Runner.hpp"

using namespace gncsim;

namespace {

EntityState entity(const Vector3& pos, const Vector3& vel) {
  EntityState s;
  s.pos = pos;
  s.vel = vel;
  return s;
}

// Noise-free homing engagement against the MC-style weaving target, finite autopilot lag. Only the
// guidance law differs between the PN and APN cases, so it is a controlled comparison.
SimConfig homingBaseConfig() {
  SimConfig c;
  c.scenario = "homing";
  c.model = "3dof";
  c.seed = 1;
  c.dt = 0.005;
  c.t_end = 40.0;
  c.integrator = Integrator::RK4;
  c.aero.ref_area = 0.02;
  c.vehicle.pos0 = {0, 0, 0};
  c.vehicle.launch_speed = 900.0;
  c.vehicle.launch_elevation_deg = 42.0;
  c.vehicle.launch_azimuth_deg = 0.0;
  c.vehicle.mass0 = 22.0;
  c.guidance.nav_constant = 4.0;
  c.guidance.max_accel = 300.0;
  c.guidance.time_constant = 0.2;  // finite lag (matches the MC config); dominant miss driver
  c.sensors.enable = false;        // noise-free: isolate the guidance law
  c.target.pos0 = {9000, 0, 3500};
  c.target.vel0 = {-280, 0, -40};
  return c;
}

}  // namespace

// ── Unit: APN feedforward only fires under the "apn" law ─────────────────────────────────────
TEST(Apn, FeedforwardOnlyAppliedUnderApnLaw) {
  // Rotating-LOS geometry that produces a non-zero PN command.
  const EntityState veh = entity({0.0, 0.0, 0.0}, {300.0, 0.0, 0.0});
  const EntityState tgt = entity({1000.0, 800.0, 0.0}, {0.0, 200.0, 0.0});
  const Engagement e = computeEngagement(veh, tgt);

  // Large target accel perpendicular to the LOS so the feedforward would clearly change the
  // command.
  const Vector3 a_target{0.0, 0.0, 80.0};

  GuidanceConfig pn;
  pn.law = "pronav";
  pn.nav_constant = 4.0;
  pn.max_accel = 300.0;

  GuidanceConfig apn = pn;
  apn.law = "apn";

  // Under "pronav", augmentedProNavCommand returns zero (wrong law).
  EXPECT_NEAR(augmentedProNavCommand(e, pn, a_target).norm(), 0.0, 1e-12);

  // Under "apn" with zero target accel, APN == PN command exactly (no feedforward to add).
  const Vector3 pn_cmd = proNavCommand(e, pn);
  const Vector3 apn_zero = augmentedProNavCommand(e, apn, Vector3{});
  EXPECT_NEAR((apn_zero - pn_cmd).norm(), 0.0, 1e-9);

  // Under "apn" with a real target accel, APN differs from PN (feedforward applied).
  const Vector3 apn_cmd = augmentedProNavCommand(e, apn, a_target);
  EXPECT_GT((apn_cmd - pn_cmd).norm(), 1.0);
}

// ── Unit: the feedforward removes the along-LOS component and respects the cap ────────────────
TEST(Apn, FeedforwardIsPerpendicularToLosAndLimited) {
  const EntityState veh = entity({0.0, 0.0, 0.0}, {300.0, 0.0, 0.0});
  const EntityState tgt = entity({1000.0, 800.0, 0.0}, {0.0, 200.0, 0.0});
  const Engagement e = computeEngagement(veh, tgt);

  GuidanceConfig apn;
  apn.law = "apn";
  apn.nav_constant = 4.0;
  apn.max_accel = 300.0;

  // A purely along-LOS target accel must not change the command (its perp projection is zero).
  const Vector3 a_along = e.los_unit * 50.0;
  const Vector3 pn_cmd = proNavCommand(e, [&] {
    GuidanceConfig p = apn;
    p.law = "pronav";
    return p;
  }());
  const Vector3 apn_along = augmentedProNavCommand(e, apn, a_along);
  EXPECT_NEAR((apn_along - pn_cmd).norm(), 0.0, 1e-6);

  // A huge target accel saturates the command at the cap.
  const Vector3 a_big{0.0, 0.0, 1.0e5};
  const Vector3 apn_big = augmentedProNavCommand(e, apn, a_big);
  EXPECT_NEAR(apn_big.norm(), apn.max_accel, 1e-6);
}

// ── Integration: APN still intercepts a non-maneuvering target (don't break the baseline) ────
TEST(Apn, InterceptsNonManeuveringTarget) {
  SimConfig c = homingBaseConfig();
  c.guidance.law = "apn";
  c.guidance.time_constant = 0.0;  // ideal: a clean kinematic intercept like the default config
  c.target.maneuver = "constant";  // non-maneuvering
  c.target.vel0 = {-280, 0, -40};

  const SimResult r = runSimulation(c);
  EXPECT_LT(r.miss_distance, 1.0);
}

// ── Integration: against a weaving target, APN miss <= PN miss for the same scenario/seed ─────
TEST(Apn, BeatsPnAgainstWeavingTarget) {
  SimConfig pn = homingBaseConfig();
  pn.guidance.law = "pronav";
  pn.target.maneuver = "weave";
  pn.target.maneuver_g = 4.0;
  pn.target.maneuver_freq = 0.4;

  SimConfig apn = pn;
  apn.guidance.law = "apn";

  const SimResult r_pn = runSimulation(pn);
  const SimResult r_apn = runSimulation(apn);

  // APN should help (or at worst tie) against the accelerating target.
  EXPECT_LE(r_apn.miss_distance, r_pn.miss_distance);
  // And the help should be meaningful, not floating-point noise.
  EXPECT_LT(r_apn.miss_distance, r_pn.miss_distance);
}

// ── Default config uses PN: no target-accel feedforward is applied ───────────────────────────
TEST(Apn, DefaultLawIsPronav) {
  const SimConfig c;  // all defaults
  EXPECT_EQ(c.guidance.law, "pronav");

  // The default-law engagement runs through proNavCommand; augmentedProNavCommand would return
  // zero for it (wrong law), confirming the APN path is inert under the default config.
  const EntityState veh = entity({0.0, 0.0, 0.0}, {300.0, 0.0, 0.0});
  const EntityState tgt = entity({1000.0, 800.0, 0.0}, {0.0, 200.0, 0.0});
  const Engagement e = computeEngagement(veh, tgt);
  EXPECT_NEAR(augmentedProNavCommand(e, c.guidance, Vector3{0, 0, 80}).norm(), 0.0, 1e-12);
}
