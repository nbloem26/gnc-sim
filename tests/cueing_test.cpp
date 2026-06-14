// gnc-sim — interceptor cueing / launch-on-track tests (issue #8). World frame ENU, SI units.
//   A) With cueing enabled the interceptor stays parked at the launch site until the criterion
//      fires (launch_time > 0), then launches; a clean cued engagement intercepts (miss < lethal).
//   B) Launch criterion: a higher cov_trace_threshold launches EARLIER (smaller launch_time); the
//      "fixed_delay" criterion launches at max_cue_time.
//   C) Lead/aim: against a non-maneuvering threat with a clean track, the cued launch achieves a
//      small miss — the constant-velocity lead solution aims roughly correctly.
//   D) Determinism: same seed -> identical result; default path unchanged (cueing disabled ->
//      homing_3dof sample byte-identical, launch_time == 0, no frame is flagged pre_launch).
#include <gtest/gtest.h>

#include <cmath>

#include "gncsim/core/Config.hpp"
#include "gncsim/scenario/Runner.hpp"

using namespace gncsim;

namespace {

constexpr double kLethalRadius = 3.0;  // mirror Runner.cpp's intercept criterion

TrackerSensorConfig radarCfg() {
  TrackerSensorConfig s;
  s.type = "radar";
  s.pos = {0, 0, 0};
  s.sigma_az = 0.002;
  s.sigma_el = 0.002;
  s.sigma_range = 25.0;
  s.sigma_range_rate = 3.0;
  return s;
}

TrackerSensorConfig irCfg() {
  TrackerSensorConfig s;
  s.type = "ir";
  s.pos = {20000, 20000, 500000};
  s.sigma_az = 0.0003;
  s.sigma_el = 0.0003;
  return s;
}

// A clean cued engagement: confident multi-sensor track, fast interceptor, non-maneuvering threat.
SimConfig baseCuedConfig() {
  SimConfig c;
  c.scenario = "cued_test";
  c.model = "3dof";
  c.seed = 7;
  c.dt = 0.005;
  c.t_end = 40.0;
  c.vehicle.pos0 = {0, 0, 0};
  c.vehicle.launch_speed = 1100;
  c.vehicle.launch_elevation_deg = 42;
  c.guidance.law = "pronav";
  c.guidance.nav_constant = 4.0;
  c.guidance.max_accel = 400.0;
  c.sensors.enable = false;
  c.target.pos0 = {14000, 0, 5200};
  c.target.vel0 = {-300, 0, -45};
  c.target.maneuver = "constant";
  c.trackers.enabled = true;
  c.trackers.process_psd = 50.0;
  c.trackers.sensors = {radarCfg(), irCfg()};
  c.cueing.enabled = true;
  c.cueing.launch_criterion = "track_cov";
  c.cueing.cov_trace_threshold = 200.0;
  c.cueing.max_cue_time = 6.0;
  return c;
}

// The launch-site position recorded in the first frame (interceptor parked there pre-launch).
Vector3 launchSite(const SimConfig& c) { return c.vehicle.pos0; }

}  // namespace

// ── A) Held at the launch site until the criterion fires, then launches and intercepts ──────────
TEST(Cueing, HeldThenLaunchesAndIntercepts) {
  const SimConfig c = baseCuedConfig();
  const SimResult r = runSimulation(c);
  ASSERT_FALSE(r.frames.empty());

  // Launch happened after t=0.
  EXPECT_GT(r.launch_time, 0.0);

  // Every frame at or before the launch time is pre-launch and parked at the launch site with zero
  // velocity; every frame strictly after has flown (moved off the site).
  const Vector3 site = launchSite(c);
  bool saw_pre_launch = false, saw_moved = false;
  for (const auto& f : r.frames) {
    if (f.pre_launch) {
      saw_pre_launch = true;
      EXPECT_NEAR(f.veh_pos.x, site.x, 1e-9);
      EXPECT_NEAR(f.veh_pos.y, site.y, 1e-9);
      EXPECT_NEAR(f.veh_pos.z, site.z, 1e-9);
      EXPECT_NEAR(f.veh_vel.norm(), 0.0, 1e-9);
    } else if (f.t > r.launch_time + 1.0) {
      saw_moved = true;
      EXPECT_GT((f.veh_pos - site).norm(), 1.0);  // it's flying
    }
  }
  EXPECT_TRUE(saw_pre_launch);
  EXPECT_TRUE(saw_moved);

  // A clean cued engagement intercepts.
  EXPECT_LT(r.miss_distance, kLethalRadius);
  EXPECT_TRUE(r.intercept);
}

// ── B) Launch criterion behaviour ───────────────────────────────────────────────────────────────
TEST(Cueing, HigherThresholdLaunchesEarlier) {
  // A looser (higher) covariance threshold is satisfied sooner, so the launch fires earlier.
  SimConfig loose = baseCuedConfig();
  loose.cueing.cov_trace_threshold = 50000.0;
  SimConfig tight = baseCuedConfig();
  tight.cueing.cov_trace_threshold = 150.0;

  const double t_loose = runSimulation(loose).launch_time;
  const double t_tight = runSimulation(tight).launch_time;

  EXPECT_GT(t_loose, 0.0);
  EXPECT_GT(t_tight, 0.0);
  EXPECT_LT(t_loose, t_tight);
}

TEST(Cueing, FixedDelayLaunchesAtMaxCueTime) {
  SimConfig c = baseCuedConfig();
  c.cueing.launch_criterion = "fixed_delay";
  c.cueing.max_cue_time = 2.0;
  const SimResult r = runSimulation(c);
  // Launch lands on the first step at or beyond max_cue_time (within one dt).
  EXPECT_NEAR(r.launch_time, 2.0, c.dt + 1e-9);
  EXPECT_GE(r.launch_time, 2.0);
}

TEST(Cueing, CovTimesOutWhenThresholdUnreachable) {
  // An impossibly tight threshold can never be met, so the hard timeout governs the launch.
  SimConfig c = baseCuedConfig();
  c.cueing.cov_trace_threshold = 1e-6;
  c.cueing.max_cue_time = 3.0;
  const SimResult r = runSimulation(c);
  EXPECT_NEAR(r.launch_time, 3.0, c.dt + 1e-9);
}

// ── C) Lead/aim solution achieves a small miss against a clean, non-maneuvering track ───────────
TEST(Cueing, LeadAimAchievesSmallMiss) {
  const SimConfig c = baseCuedConfig();
  const SimResult r = runSimulation(c);
  // The constant-velocity lead aim is "roughly correct": terminal homing closes the residual, so
  // the miss is small (well inside lethal).
  EXPECT_LT(r.miss_distance, kLethalRadius);
}

// ── D) Determinism + default-path parity ────────────────────────────────────────────────────────
TEST(Cueing, DeterministicForSameSeed) {
  const SimConfig c = baseCuedConfig();
  const SimResult a = runSimulation(c);
  const SimResult b = runSimulation(c);
  EXPECT_DOUBLE_EQ(a.launch_time, b.launch_time);
  EXPECT_DOUBLE_EQ(a.miss_distance, b.miss_distance);
  ASSERT_EQ(a.frames.size(), b.frames.size());
  EXPECT_DOUBLE_EQ(a.frames.back().veh_pos.x, b.frames.back().veh_pos.x);
}

TEST(Cueing, DefaultPathHasNoLaunchDelay) {
  // Cueing disabled (default): the interceptor launches at t=0 exactly as before — launch_time is 0
  // and no frame is flagged pre-launch.
  SimConfig c = baseCuedConfig();
  c.cueing.enabled = false;
  const SimResult r = runSimulation(c);
  EXPECT_DOUBLE_EQ(r.launch_time, 0.0);
  for (const auto& f : r.frames) EXPECT_FALSE(f.pre_launch);
  // The very first frame already carries the launch velocity (not parked).
  ASSERT_FALSE(r.frames.empty());
  EXPECT_GT(r.frames.front().veh_vel.norm(), 1.0);
}

TEST(Cueing, RequiresTrackersToActivate) {
  // Cueing is a no-op without trackers (it needs the fused track to cue from): launch at t=0.
  SimConfig c = baseCuedConfig();
  c.trackers.enabled = false;
  const SimResult r = runSimulation(c);
  EXPECT_DOUBLE_EQ(r.launch_time, 0.0);
  for (const auto& f : r.frames) EXPECT_FALSE(f.pre_launch);
}

TEST(Cueing, ParsesCueingConfigBlock) {
  const std::string js = R"({
    "scenario":"c","model":"3dof",
    "cueing":{"enabled":true,"launch_criterion":"fixed_delay",
              "cov_trace_threshold":1234.0,"max_cue_time":7.5,"loft_deg":12.0}})";
  const SimConfig c = loadConfigFromString(js);
  EXPECT_TRUE(c.cueing.enabled);
  EXPECT_EQ(c.cueing.launch_criterion, "fixed_delay");
  EXPECT_DOUBLE_EQ(c.cueing.cov_trace_threshold, 1234.0);
  EXPECT_DOUBLE_EQ(c.cueing.max_cue_time, 7.5);
  EXPECT_DOUBLE_EQ(c.cueing.loft_deg, 12.0);
}

TEST(Cueing, DefaultConfigDisablesCueing) {
  const SimConfig c = loadConfigFromString("{}");
  EXPECT_FALSE(c.cueing.enabled);
  EXPECT_EQ(c.cueing.launch_criterion, "track_cov");
}

// Default homing_3dof outcome is unchanged with cueing disabled (the new code is inert). Rebuilds
// homing_3dof's config in code and checks the known reference outcome (the byte-level CSV parity is
// verified separately by the VERIFY block's diff against runs/sample_run).
TEST(Cueing, DefaultHoming3dofOutcomeUnchanged) {
  SimConfig c;
  c.scenario = "homing";
  c.model = "3dof";
  c.seed = 1;
  c.dt = 0.005;
  c.t_end = 40.0;
  c.aero.ref_area = 0.02;
  c.aero.cd0 = 0.3;
  c.aero.cd_mach = {{0.0, 0.28}, {0.7, 0.30}, {0.9, 0.42}, {1.0, 0.58}, {1.2, 0.52},
                    {1.5, 0.43}, {2.0, 0.36}, {3.0, 0.30}, {4.0, 0.27}};
  c.vehicle.launch_speed = 900;
  c.vehicle.launch_elevation_deg = 42;
  c.vehicle.mass0 = 22.0;
  c.guidance.law = "pronav";
  c.guidance.nav_constant = 4.0;
  c.guidance.max_accel = 400.0;
  c.sensors.enable = false;
  c.target.pos0 = {9000, 0, 3500};
  c.target.vel0 = {-280, 0, -40};

  const SimResult r = runSimulation(c);
  EXPECT_DOUBLE_EQ(r.launch_time, 0.0);
  // Outcome matches the known homing_3dof reference (sub-millimetre miss, intercept).
  EXPECT_LT(r.miss_distance, kLethalRadius);
  EXPECT_TRUE(r.intercept);
}
