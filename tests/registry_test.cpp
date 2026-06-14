// gnc-sim — ModelRegistry tests (issue #31). Verify that each interface key resolves to the right
// concrete model, that swapping guidance pronav<->apn via the config string changes behavior as
// expected, and that an unknown key errors cleanly. Configs are built in-code (no file/cwd
// dependency), matching the rest of the suite.
#include "gncsim/model/Registry.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

#include "gncsim/gnc/Gnc.hpp"
#include "gncsim/model/Interfaces.hpp"
#include "gncsim/scenario/Runner.hpp"

using namespace gncsim;

namespace {

// A simple closing engagement with a non-zero LOS rate so PN produces a finite command.
Engagement closingEngagement() {
  EntityState veh, tgt;
  veh.pos = {0, 0, 0};
  veh.vel = {300, 0, 0};
  tgt.pos = {1000, 50, 0};
  tgt.vel = {-100, 20, 0};  // crossing -> non-zero LOS rate
  return computeEngagement(veh, tgt);
}

}  // namespace

// --- Guidance keys resolve to the right law -------------------------------------------------

TEST(Registry, GuidanceKeysResolve) {
  ModelRegistry reg;
  GuidanceConfig cfg;
  cfg.law = "pronav";
  cfg.nav_constant = 4.0;
  cfg.max_accel = 400.0;

  EXPECT_NE(reg.makeGuidance("pronav", cfg), nullptr);
  EXPECT_NE(reg.makeGuidance("apn", cfg), nullptr);
  EXPECT_NE(reg.makeGuidance("zemzev", cfg), nullptr);  // optimal ZEM/ZEV (issue #40)
  EXPECT_NE(reg.makeGuidance("none", cfg), nullptr);
}

TEST(Registry, NoneGuidanceCommandsZero) {
  ModelRegistry reg;
  GuidanceConfig cfg;
  cfg.law = "none";
  auto g = reg.makeGuidance("none", cfg);
  const Vector3 cmd = g->command(closingEngagement(), GuidanceState{});
  EXPECT_DOUBLE_EQ(cmd.norm(), 0.0);
}

// Swapping pronav<->apn via the config string changes behavior: with a non-zero target-acceleration
// feedforward, APN's command differs from PN's on the same geometry.
TEST(Registry, GuidanceSwapChangesBehavior) {
  ModelRegistry reg;
  GuidanceConfig cfg;
  cfg.nav_constant = 4.0;
  cfg.max_accel = 1e6;  // unsaturated so the feedforward shows through

  const Engagement e = closingEngagement();

  auto pn = reg.makeGuidance("pronav", cfg);
  auto apn = reg.makeGuidance("apn", cfg);

  GuidanceState gs;
  gs.a_target_est = {0, 80, 0};  // ~8 g lateral target maneuver

  const Vector3 cmd_pn = pn->command(e, gs);    // PN ignores the feedforward
  const Vector3 cmd_apn = apn->command(e, gs);  // APN adds the perpendicular feedforward
  EXPECT_GT((cmd_apn - cmd_pn).norm(), 1.0);    // the two laws command materially differently
}

// --- Navigation keys resolve; alpha-beta has no innovation, EKF does ------------------------

TEST(Registry, NavigatorKeysResolve) {
  ModelRegistry reg;
  NavConfig nav;
  SeekerNoise seeker;
  seeker.los_white = 3e-3;

  auto ab = reg.makeNavigator("alpha_beta", nav, seeker, 0.005);
  auto ekf = reg.makeNavigator("ekf", nav, seeker, 0.005);
  ASSERT_NE(ab, nullptr);
  ASSERT_NE(ekf, nullptr);

  // Alpha-beta converges toward the fed measurement and reports no innovation.
  NavMeasurement z;
  z.measured_rel_pos = {1000, 0, 0};
  ab->update(z);
  ab->update(z);
  EXPECT_DOUBLE_EQ(ab->nis(), 0.0);
  EXPECT_NEAR(ab->relPos().x, 1000.0, 1.0);
}

// --- Sensor keys resolve to the right type / measurement dimensionality ---------------------

TEST(Registry, SensorKeysResolve) {
  ModelRegistry reg;
  Rng rng(1);

  TrackerSensorConfig radar;
  radar.type = "radar";
  radar.pos = {0, 0, 0};
  auto radar_model = reg.makeSensor(radar);
  EXPECT_EQ(radar_model->spec().type, TrackSensorType::Radar);
  EXPECT_EQ(radar_model->measure({1000, 200, 50}, {-100, 0, 0}, rng).size(), 4u);  // az,el,r,rdot

  TrackerSensorConfig ir;
  ir.type = "ir";
  ir.pos = {0, 0, 0};
  auto ir_model = reg.makeSensor(ir);
  EXPECT_EQ(ir_model->spec().type, TrackSensorType::Ir);
  EXPECT_EQ(ir_model->measure({1000, 200, 50}, {-100, 0, 0}, rng).size(), 2u);  // az,el only
}

// --- Dynamics / environment / threat keys resolve ------------------------------------------

TEST(Registry, DynamicsKeysResolve) {
  ModelRegistry reg;
  VehicleConfig veh;
  auto d3 = reg.makeDynamics("3dof", veh, Integrator::RK4);
  auto d6 = reg.makeDynamics("6dof", veh, Integrator::RK4);
  EXPECT_FALSE(d3->is6dof());
  EXPECT_TRUE(d6->is6dof());
}

TEST(Registry, EnvironmentKeysResolve) {
  ModelRegistry reg;
  EnvConfig env;  // default frame "flat"
  auto e = reg.makeEnvironment(env);
  ASSERT_NE(e, nullptr);
  // Default surface gravity points down (negative Up) on the flat path.
  EXPECT_LT(e->gravity({0, 0, 0}).z, 0.0);
  // USSA76 density falls with altitude.
  EXPECT_GT(e->atmosphere(0.0).density, e->atmosphere(10000.0).density);
}

TEST(Registry, ThreatKeysResolve) {
  ModelRegistry reg;
  TargetConfig constant;
  constant.maneuver = "constant";
  auto t_const = reg.makeThreat(constant);
  EntityState tgt;
  tgt.vel = {-250, 0, 0};
  EXPECT_DOUBLE_EQ(t_const->accel(tgt, 1.0).norm(), 0.0);

  TargetConfig weave;
  weave.maneuver = "weave";
  weave.maneuver_g = 3.0;
  weave.maneuver_freq = 0.4;
  auto t_weave = reg.makeThreat(weave);
  // At a quarter period the sinusoid is near its peak -> non-zero lateral accel.
  EXPECT_GT(t_weave->accel(tgt, 0.625).norm(), 1.0);
}

// --- Unknown keys error cleanly -------------------------------------------------------------

TEST(Registry, UnknownKeysThrow) {
  ModelRegistry reg;
  GuidanceConfig gcfg;
  NavConfig nav;
  SeekerNoise seeker;
  VehicleConfig veh;
  EnvConfig env;
  TargetConfig tgt;
  TrackerSensorConfig sc;

  EXPECT_THROW(reg.makeGuidance("zem", gcfg), std::invalid_argument);
  EXPECT_THROW(reg.makeNavigator("ukf", nav, seeker, 0.005), std::invalid_argument);
  EXPECT_THROW(reg.makeDynamics("9dof", veh, Integrator::RK4), std::invalid_argument);
  env.frame = "torus";
  EXPECT_THROW(reg.makeEnvironment(env), std::invalid_argument);
  tgt.maneuver = "barrel_roll";
  EXPECT_THROW(reg.makeThreat(tgt), std::invalid_argument);
  sc.type = "lidar";
  EXPECT_THROW(reg.makeSensor(sc), std::invalid_argument);
}

// --- End-to-end: default config strings resolve to today's models (behavior preserved) ------
//
// Swapping guidance pronav -> apn via the SimConfig string must change the run outcome (different
// guidance law), proving the registry actually drives the runner's resolution.
TEST(Registry, RunnerGuidanceSwapViaConfigString) {
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
  c.guidance.nav_constant = 4.0;
  c.guidance.max_accel = 400.0;
  // A maneuvering target so APN's feedforward measurably changes the trajectory vs PN.
  c.target.pos0 = {9000, 0, 3500};
  c.target.vel0 = {-280, 0, -40};
  c.target.maneuver = "weave";
  c.target.maneuver_g = 4.0;
  c.target.maneuver_freq = 0.5;

  c.guidance.law = "pronav";
  SimResult pn = runSimulation(c);
  c.guidance.law = "apn";
  SimResult apn = runSimulation(c);

  ASSERT_GT(pn.frames.size(), 100u);
  ASSERT_GT(apn.frames.size(), 100u);
  // The two laws fly materially different mid-course trajectories.
  const std::size_t k = std::min(pn.frames.size(), apn.frames.size()) / 2;
  const double dx = pn.frames[k].veh_pos.x - apn.frames[k].veh_pos.x;
  const double dz = pn.frames[k].veh_pos.z - apn.frames[k].veh_pos.z;
  EXPECT_GT(dx * dx + dz * dz, 1e-6);
}
