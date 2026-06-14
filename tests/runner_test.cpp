// gnc-sim — Phase 2 integration tests: the full GNC loop end-to-end via runSimulation().
// Configs are built in-code (no file/cwd dependency). Also covers determinism — the native half
// of the native<->WASM parity guarantee (the cross-build check runs in CI via Node).
#include <cmath>

#include <gtest/gtest.h>

#include "gncsim/scenario/Runner.hpp"

using namespace gncsim;

namespace {

SimConfig homingConfig() {
  SimConfig c;
  c.scenario = "homing";
  c.model = "3dof";
  c.seed = 1;
  c.dt = 0.005;
  c.t_end = 40.0;
  c.aero.ref_area = 0.02;
  c.aero.cd_mach = {{0.0, 0.28}, {0.9, 0.42}, {1.0, 0.58}, {2.0, 0.36}, {4.0, 0.27}};
  c.vehicle.launch_speed = 900.0;
  c.vehicle.launch_elevation_deg = 42.0;
  c.vehicle.mass0 = 22.0;
  c.guidance.nav_constant = 4.0;
  c.guidance.max_accel = 400.0;
  c.target.pos0 = {9000, 0, 3500};
  c.target.vel0 = {-280, 0, -40};
  return c;
}

}  // namespace

TEST(Runner, NoiseFreeProNavIntercepts) {
  // Classic PN result: against a non-maneuvering target with no sensor noise, miss ~ 0.
  SimResult r = runSimulation(homingConfig());
  EXPECT_TRUE(r.intercept);
  EXPECT_LT(r.miss_distance, 1.0);
  EXPECT_GT(r.frames.size(), 100u);
}

TEST(Runner, Deterministic) {
  SimConfig c = homingConfig();
  c.sensors.enable = true;  // exercise the RNG path too
  c.sensors.seeker.los_white = 3e-3;
  c.sensors.imu.gyro_white = 9e-5;

  SimResult a = runSimulation(c);
  SimResult b = runSimulation(c);
  ASSERT_EQ(a.frames.size(), b.frames.size());
  EXPECT_DOUBLE_EQ(a.miss_distance, b.miss_distance);
  // Spot-check a mid-trajectory frame is bit-identical.
  const std::size_t k = a.frames.size() / 2;
  EXPECT_DOUBLE_EQ(a.frames[k].veh_pos.x, b.frames[k].veh_pos.x);
  EXPECT_DOUBLE_EQ(a.frames[k].seeker_los_meas, b.frames[k].seeker_los_meas);
}

TEST(Runner, UnguidedProjectileFallsBack) {
  SimConfig c = homingConfig();
  c.guidance.law = "none";
  c.vehicle.launch_speed = 300.0;       // slow enough to land within the window
  c.vehicle.launch_elevation_deg = 45.0;
  c.t_end = 120.0;
  c.target.pos0 = {80000, 0, 0};        // far away, unreachable
  SimResult r = runSimulation(c);
  EXPECT_FALSE(r.intercept);
  // The run stops when the projectile returns to the ground (z<0).
  EXPECT_LT(r.frames.back().veh_pos.z, 50.0);
  EXPECT_GT(r.frames.back().veh_pos.x, 1000.0);  // it flew downrange
}

TEST(Runner, MonteCarloReproducibleAndDispersed) {
  SimConfig c = homingConfig();
  c.seed = 12345;
  c.sensors.enable = true;
  c.sensors.seeker.los_white = 3e-3;
  c.monte_carlo.num_cases = 40;
  c.monte_carlo.launch_speed_sigma = 15.0;
  c.monte_carlo.target_pos_sigma = 50.0;

  auto a = runMonteCarlo(c);
  auto b = runMonteCarlo(c);
  ASSERT_EQ(a.size(), 40u);
  ASSERT_EQ(b.size(), a.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].seed, b[i].seed);
    EXPECT_DOUBLE_EQ(a[i].miss_distance, b[i].miss_distance);
  }
  // Dispersion should be non-degenerate (cases differ from each other).
  bool varied = false;
  for (std::size_t i = 1; i < a.size(); ++i)
    if (std::abs(a[i].miss_distance - a[0].miss_distance) > 1e-6) varied = true;
  EXPECT_TRUE(varied);
}
