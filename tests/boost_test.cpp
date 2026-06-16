// gnc-sim — boost-phase tests: thrust accel, propellant burn, staging mass drop, default-off.
// Configs are built in-code (no file/cwd dependency), matching the other integration tests.
#include <gtest/gtest.h>

#include <cmath>

#include "gncsim/scenario/Runner.hpp"

using namespace gncsim;

namespace {

// Minimal config for isolating propulsion: no gravity, no atmosphere, no guidance, far target so
// the run never terminates early. Launch straight along +x at v0.
SimConfig boostBaseConfig() {
  SimConfig c;
  c.scenario = "homing";
  c.model = "3dof";
  c.seed = 1;
  c.dt = 0.005;
  c.t_end = 10.0;
  c.integrator = Integrator::RK4;
  c.env.g0 = 0.0;  // no gravity
  c.env.altitude_dependent_g = false;
  c.env.atmosphere = false;  // no drag
  c.vehicle.pos0 = {0, 0, 0};
  c.vehicle.launch_speed = 100.0;
  c.vehicle.launch_elevation_deg = 0.0;  // straight along +x (horizontal)
  c.vehicle.launch_azimuth_deg = 0.0;
  c.vehicle.mass0 = 20.0;
  c.guidance.law = "none";        // unguided
  c.target.pos0 = {1.0e9, 0, 0};  // effectively unreachable; never terminates early
  c.target.vel0 = {0, 0, 0};
  return c;
}

}  // namespace

// Constant thrust, zero propellant (mass constant): v(t) = v0 + (thrust/mass)*t exactly under RK4
// (constant acceleration is integrated exactly). Check at a time inside the burn window.
TEST(Boost, ConstantThrustAccelerates) {
  SimConfig c = boostBaseConfig();
  c.propulsion.thrust = 5000.0;        // N
  c.propulsion.burn_time = 6.0;        // s — covers the check time
  c.propulsion.propellant_mass = 0.0;  // mass stays constant
  c.vehicle.mass0 = 20.0;

  SimResult r = runSimulation(c);

  const double v0 = c.vehicle.launch_speed;
  const double a = c.propulsion.thrust / c.vehicle.mass0;
  const double t_check = 4.0;  // inside burn window
  // Find the frame at t_check.
  const int idx = static_cast<int>(std::lround(t_check / c.dt));
  ASSERT_LT(static_cast<std::size_t>(idx), r.frames.size());
  ASSERT_NEAR(r.frames[idx].t, t_check, 1e-9);

  const double v_expected = v0 + a * t_check;
  const double v_actual = r.frames[idx].veh_vel.norm();
  EXPECT_NEAR(v_actual, v_expected, 1e-6);
}

// Linear propellant burn: final mass after burn_time == mass0 - propellant_mass.
TEST(Boost, PropellantBurnReducesMass) {
  SimConfig c = boostBaseConfig();
  c.propulsion.thrust = 5000.0;
  c.propulsion.burn_time = 3.0;
  c.propulsion.propellant_mass = 8.0;
  c.vehicle.mass0 = 20.0;

  SimResult r = runSimulation(c);

  // Frame just after the burn completes (t >= burn_time): mass should be at the dry-mass floor.
  const int idx = static_cast<int>(std::lround(c.propulsion.burn_time / c.dt));
  ASSERT_LT(static_cast<std::size_t>(idx), r.frames.size());
  const double mass_final = r.frames[idx].mass;
  const double mass_expected = c.vehicle.mass0 - c.propulsion.propellant_mass;
  EXPECT_NEAR(mass_final, mass_expected, 1e-9);

  // And mass stays there afterward.
  EXPECT_NEAR(r.frames.back().mass, mass_expected, 1e-9);
}

// Staging: mass drops by stage_mass_drop around stage_time, exactly once.
TEST(Boost, StagingDropsMass) {
  SimConfig c = boostBaseConfig();
  c.propulsion.thrust = 0.0;  // isolate staging from thrust/burn
  c.propulsion.propellant_mass = 0.0;
  c.propulsion.stage_time = 2.0;
  c.propulsion.stage_mass_drop = 5.0;
  c.vehicle.mass0 = 20.0;

  SimResult r = runSimulation(c);

  const int idx_before = static_cast<int>(std::lround((c.propulsion.stage_time - 0.5) / c.dt));
  const int idx_after = static_cast<int>(std::lround((c.propulsion.stage_time + 0.5) / c.dt));
  ASSERT_LT(static_cast<std::size_t>(idx_after), r.frames.size());

  EXPECT_NEAR(r.frames[idx_before].mass, c.vehicle.mass0, 1e-9);
  EXPECT_NEAR(r.frames[idx_after].mass, c.vehicle.mass0 - c.propulsion.stage_mass_drop, 1e-9);
  // Dropped exactly once (final mass unchanged from just-after-staging).
  EXPECT_NEAR(r.frames.back().mass, c.vehicle.mass0 - c.propulsion.stage_mass_drop, 1e-9);
}

// Default-off: a config with no propulsion has the thrust telemetry column all zero.
TEST(Boost, DefaultOffNoThrust) {
  SimConfig c = boostBaseConfig();
  // propulsion left at defaults (all zero).
  SimResult r = runSimulation(c);
  ASSERT_GT(r.frames.size(), 10u);
  for (const auto& f : r.frames) {
    EXPECT_EQ(f.thrust, 0.0);
  }
}
