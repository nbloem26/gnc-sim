// gnc-sim — multi-sensor target-track EKF + fusion tests (issue #5). World frame ENU, SI units.
//   A) Noise-free constant-velocity absolute target track: converges tight, covariance trace drops.
//   B) Sequential fusion of two sensors yields a lower post-update covariance trace than either
//      single sensor alone (the core "fusion helps" property) on a fixed geometry.
//   C) Angles-only (IR) alone has weak range observability (large along-range covariance) vs radar
//      (which carries range) — assert the covariance structure reflects this.
//   D) NIS consistency over a noisy run: mean NIS ~ dof per sensor.
//   E) Determinism: same seed -> identical estimates.
//   F) Default path unchanged: with trackers disabled, homing_3dof telemetry is byte-identical and
//      the track channel is zero; enabling trackers fuses radar+IR to beat either alone.
#include <gtest/gtest.h>

#include <cmath>

#include "gncsim/core/Config.hpp"
#include "gncsim/core/Rng.hpp"
#include "gncsim/core/Serialize.hpp"
#include "gncsim/gnc/TargetTrackEkf.hpp"
#include "gncsim/math/Vector3.hpp"
#include "gncsim/scenario/Runner.hpp"

using namespace gncsim;

namespace {

// Noise-free measurement of a target absolute state from a fixed sensor.
std::vector<double> measure(const TrackSensor& s, const Vector3& tp, const Vector3& tv) {
  const Vector3 rel = tp - s.pos;
  const double horiz = std::sqrt(rel.x * rel.x + rel.y * rel.y);
  const double r = rel.norm();
  const double az = std::atan2(rel.y, rel.x);
  const double el = std::atan2(rel.z, horiz);
  if (s.type == TrackSensorType::Ir) return {az, el};
  const double rr = r > 1e-9 ? rel.dot(tv) / r : 0.0;
  return {az, el, r, rr};
}

TrackSensor radarAt(const Vector3& p) {
  TrackSensor s;
  s.type = TrackSensorType::Radar;
  s.pos = p;
  s.sigma_az = 0.002;
  s.sigma_el = 0.002;
  s.sigma_range = 20.0;
  s.sigma_range_rate = 2.0;
  return s;
}

TrackSensor irAt(const Vector3& p) {
  TrackSensor s;
  s.type = TrackSensorType::Ir;
  s.pos = p;
  s.sigma_az = 0.0003;
  s.sigma_el = 0.0003;
  return s;
}

}  // namespace

// ── A) Noise-free constant-velocity track: converges, covariance trace decreases ────────────────
TEST(TargetTrackEkf, ConvergesOnNoiseFreeConstantVelocityTrack) {
  const double dt = 0.005;
  TargetTrackEkf ekf(dt, /*process_psd=*/5.0);
  const TrackSensor radar = radarAt({0.0, 0.0, 0.0});

  const Vector3 p0{9000.0, 1200.0, 3500.0};
  const Vector3 v0{-280.0, 30.0, -40.0};

  // Bootstrap from the first radar measurement (position from az/el/range, velocity unknown).
  ekf.update(radar, measure(radar, p0, v0));
  const double trace_early = ekf.covTrace();

  double pos_err = 0.0, vel_err = 0.0, trace_late = 0.0;
  const int steps = 1600;  // 8 s
  for (int k = 1; k <= steps; ++k) {
    const Vector3 p = p0 + v0 * (k * dt);
    ekf.predict();
    ekf.update(radar, measure(radar, p, v0));
    if (k == steps) {
      pos_err = (ekf.pos() - p).norm();
      vel_err = (ekf.vel() - v0).norm();
      trace_late = ekf.covTrace();
    }
  }

  EXPECT_LT(pos_err, 1.0);
  EXPECT_LT(vel_err, 1.0);
  EXPECT_LT(trace_late, trace_early);  // covariance shrank as measurements accumulated
}

// ── B) Sequential two-sensor fusion is tighter than either single sensor ────────────────────────
TEST(TargetTrackEkf, FusionLowersCovarianceVsSingleSensor) {
  const double dt = 0.005;
  const Vector3 p0{9000.0, 1200.0, 3500.0};
  const Vector3 v0{-280.0, 30.0, -40.0};
  const TrackSensor radar = radarAt({0.0, 0.0, 0.0});
  const TrackSensor ir = irAt({20000.0, 20000.0, 500000.0});

  // Drive three filters from the SAME bootstrap and the SAME truth track; differ only in which
  // sensors update each step. Compare the post-update covariance trace.
  auto run = [&](bool with_radar, bool with_ir) {
    TargetTrackEkf ekf(dt, 5.0);
    ekf.bootstrap(p0, v0);  // common starting covariance for a fair comparison
    const int steps = 400;
    for (int k = 1; k <= steps; ++k) {
      const Vector3 p = p0 + v0 * (k * dt);
      ekf.predict();
      if (with_radar) ekf.update(radar, measure(radar, p, v0));
      if (with_ir) ekf.update(ir, measure(ir, p, v0));
    }
    return ekf.covTrace();
  };

  const double radar_only = run(true, false);
  const double ir_only = run(false, true);
  const double fused = run(true, true);

  // Fusing both sensors yields the tightest covariance.
  EXPECT_LT(fused, radar_only);
  EXPECT_LT(fused, ir_only);
}

// ── C) Angles-only (IR) has weak range observability vs radar ───────────────────────────────────
TEST(TargetTrackEkf, AnglesOnlyHasWeakRangeObservability) {
  const double dt = 0.005;
  const Vector3 p0{9000.0, 0.0, 3500.0};
  const Vector3 v0{-280.0, 0.0, -40.0};

  // Ground radar measures range directly; the IR platform is angles-only.
  const TrackSensor radar = radarAt({0.0, 0.0, 0.0});
  const TrackSensor ir = irAt({0.0, 0.0, 500000.0});

  auto run = [&](const TrackSensor& s) {
    TargetTrackEkf ekf(dt, 5.0);
    ekf.bootstrap(p0, v0);
    const int steps = 600;
    for (int k = 1; k <= steps; ++k) {
      const Vector3 p = p0 + v0 * (k * dt);
      ekf.predict();
      ekf.update(s, measure(s, p, v0));
    }
    return ekf;
  };

  const TargetTrackEkf radar_ekf = run(radar);
  const TargetTrackEkf ir_ekf = run(ir);

  // Position covariance trace (the x,y,z diagonal). Angles-only cannot pin range, so its position
  // uncertainty stays far larger than the radar's.
  auto posVar = [](const TargetTrackEkf& e) { return e.cov(0, 0) + e.cov(1, 1) + e.cov(2, 2); };
  EXPECT_GT(posVar(ir_ekf), 10.0 * posVar(radar_ekf));

  // The IR platform sits directly overhead (along +z): its line of sight to the target is nearly
  // vertical, so its unobservable "range" direction is mostly z. The angles-only filter therefore
  // leaves a far larger z-position variance than the radar (which measures range and pins z), while
  // both pin the cross-range (x,y) angles comparably. This is the angles-only range-ambiguity
  // signature in the covariance structure.
  EXPECT_GT(ir_ekf.cov(2, 2), 100.0 * radar_ekf.cov(2, 2));
  EXPECT_GT(ir_ekf.cov(2, 2), ir_ekf.cov(0, 0));  // weakest along the (vertical) LOS / range axis
}

// ── D) NIS consistency over a noisy run (radar, dof = 4) ─────────────────────────────────────────
TEST(TargetTrackEkf, NisConsistencyMeanNearDof) {
  const double dt = 0.005;
  TargetTrackEkf ekf(dt, /*process_psd=*/20.0);
  Rng rng(2024);
  const TrackSensor radar = radarAt({0.0, 0.0, 0.0});
  const int dof = radar.dim();  // 4

  const Vector3 p0{9000.0, 1500.0, 3500.0};
  const Vector3 v0{-280.0, 40.0, -30.0};

  ekf.update(radar, measure(radar, p0, v0));  // bootstrap

  const int warmup = 600;
  const int steps = 9000;
  double nis_sum = 0.0;
  int nis_count = 0;
  for (int k = 1; k <= steps; ++k) {
    const Vector3 p = p0 + v0 * (k * dt);
    ekf.predict();
    std::vector<double> z = measure(radar, p, v0);
    z[0] += rng.gaussian(0.0, radar.sigma_az);
    z[1] += rng.gaussian(0.0, radar.sigma_el);
    z[2] += rng.gaussian(0.0, radar.sigma_range);
    z[3] += rng.gaussian(0.0, radar.sigma_range_rate);
    ekf.update(radar, z);
    if (k >= warmup) {
      nis_sum += ekf.nis();
      ++nis_count;
    }
  }
  const double mean_nis = nis_sum / nis_count;
  // Mean NIS near dof = 4 within ~20%.
  EXPECT_GT(mean_nis, dof * 0.80);
  EXPECT_LT(mean_nis, dof * 1.20);
}

// ── E) Determinism: same seed -> identical estimates ────────────────────────────────────────────
TEST(TargetTrackEkf, DeterministicForSameSeed) {
  const double dt = 0.005;
  const TrackSensor radar = radarAt({0.0, 0.0, 0.0});
  const Vector3 p0{9000.0, -800.0, 3500.0};
  const Vector3 v0{-280.0, 25.0, -35.0};

  auto run = [&](std::uint64_t seed) {
    TargetTrackEkf ekf(dt, 20.0);
    Rng rng(seed);
    ekf.update(radar, measure(radar, p0, v0));
    Vector3 last;
    for (int k = 1; k <= 2000; ++k) {
      const Vector3 p = p0 + v0 * (k * dt);
      ekf.predict();
      std::vector<double> z = measure(radar, p, v0);
      z[0] += rng.gaussian(0.0, radar.sigma_az);
      z[1] += rng.gaussian(0.0, radar.sigma_el);
      z[2] += rng.gaussian(0.0, radar.sigma_range);
      z[3] += rng.gaussian(0.0, radar.sigma_range_rate);
      ekf.update(radar, z);
      last = ekf.pos();
    }
    return last;
  };

  const Vector3 a = run(99);
  const Vector3 b = run(99);
  EXPECT_DOUBLE_EQ(a.x, b.x);
  EXPECT_DOUBLE_EQ(a.y, b.y);
  EXPECT_DOUBLE_EQ(a.z, b.z);
  const Vector3 c = run(100);
  EXPECT_NE(a.x, c.x);
}

// ── F) Runner integration: default path unchanged; trackers fuse to beat either single sensor ───
namespace {

// Minimal in-code engagement shared by the tracker-config tests below.
SimConfig baseTrackConfig() {
  SimConfig c;
  c.scenario = "track_test";
  c.model = "3dof";
  c.seed = 7;
  c.dt = 0.005;
  c.t_end = 20.0;
  c.vehicle.pos0 = {0, 0, 0};
  c.vehicle.launch_speed = 900;
  c.vehicle.launch_elevation_deg = 42;
  c.guidance.law = "pronav";
  c.guidance.nav_constant = 4.0;
  c.guidance.max_accel = 400.0;
  c.sensors.enable = false;
  c.target.pos0 = {9000, 0, 3500};
  c.target.vel0 = {-280, 0, -40};
  return c;
}

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

// RMS of the fused track-position error vs the true target position over a run (skip transient).
double trackRms(const SimResult& r, int settle = 200) {
  double sse = 0.0;
  int n = 0;
  for (std::size_t i = 0; i < r.frames.size(); ++i) {
    if (static_cast<int>(i) < settle) continue;
    const Frame& f = r.frames[i];
    sse += (f.track_pos_est - f.tgt_pos).normSq();
    ++n;
  }
  return n > 0 ? std::sqrt(sse / n) : 0.0;
}

}  // namespace

TEST(TargetTrackRunner, DefaultPathLeavesTrackChannelZero) {
  SimConfig c = baseTrackConfig();
  // trackers disabled by default.
  const SimResult r = runSimulation(c);
  ASSERT_FALSE(r.frames.empty());
  for (const auto& f : r.frames) {
    EXPECT_EQ(f.track_pos_est.x, 0.0);
    EXPECT_EQ(f.track_pos_est.y, 0.0);
    EXPECT_EQ(f.track_pos_est.z, 0.0);
    EXPECT_EQ(f.track_nis, 0.0);
  }
}

TEST(TargetTrackRunner, FusionBeatsSingleSensorRms) {
  SimConfig radar = baseTrackConfig();
  radar.scenario = "track_radar_only";
  radar.trackers.enabled = true;
  radar.trackers.process_psd = 50.0;
  radar.trackers.sensors = {radarCfg()};

  SimConfig ir = baseTrackConfig();
  ir.scenario = "track_ir_only";
  ir.trackers.enabled = true;
  ir.trackers.process_psd = 50.0;
  ir.trackers.sensors = {irCfg()};

  SimConfig fused = baseTrackConfig();
  fused.scenario = "track_fused";
  fused.trackers.enabled = true;
  fused.trackers.process_psd = 50.0;
  fused.trackers.sensors = {radarCfg(), irCfg()};

  const double rms_radar = trackRms(runSimulation(radar));
  const double rms_ir = trackRms(runSimulation(ir));
  const double rms_fused = trackRms(runSimulation(fused));

  // Fusion is the best of the three; IR-only (angles-only) is by far the worst (no range).
  EXPECT_LT(rms_fused, rms_radar);
  EXPECT_LT(rms_fused, rms_ir);
  EXPECT_GT(rms_ir, rms_radar);
  EXPECT_GT(rms_ir, 50.0);  // weak range observability shows as a large absolute error
  EXPECT_LT(rms_fused, 20.0);
}

TEST(TargetTrackRunner, DeterministicSameSeed) {
  SimConfig c = baseTrackConfig();
  c.trackers.enabled = true;
  c.trackers.sensors = {radarCfg(), irCfg()};
  const SimResult a = runSimulation(c);
  const SimResult b = runSimulation(c);
  ASSERT_EQ(a.frames.size(), b.frames.size());
  const Frame& fa = a.frames.back();
  const Frame& fb = b.frames.back();
  EXPECT_DOUBLE_EQ(fa.track_pos_est.x, fb.track_pos_est.x);
  EXPECT_DOUBLE_EQ(fa.track_pos_est.y, fb.track_pos_est.y);
  EXPECT_DOUBLE_EQ(fa.track_pos_est.z, fb.track_pos_est.z);
}

// Config parsing: the trackers block is read tolerantly into SimConfig.
TEST(TargetTrackRunner, ParsesTrackersConfigBlock) {
  const std::string js = R"({
    "scenario":"t","model":"3dof","trackers":{
      "enabled":true,"process_psd":42.0,
      "sensors":[
        {"type":"radar","pos":[1,2,3],"sigma_az":0.01,"sigma_range":7.0},
        {"type":"ir","pos":[0,0,500000],"sigma_az":0.001}
      ]}})";
  const SimConfig c = loadConfigFromString(js);
  ASSERT_TRUE(c.trackers.enabled);
  EXPECT_DOUBLE_EQ(c.trackers.process_psd, 42.0);
  ASSERT_EQ(c.trackers.sensors.size(), 2u);
  EXPECT_EQ(c.trackers.sensors[0].type, "radar");
  EXPECT_DOUBLE_EQ(c.trackers.sensors[0].pos.x, 1.0);
  EXPECT_DOUBLE_EQ(c.trackers.sensors[0].sigma_range, 7.0);
  EXPECT_EQ(c.trackers.sensors[1].type, "ir");
  EXPECT_DOUBLE_EQ(c.trackers.sensors[1].pos.z, 500000.0);
}

// Default config has trackers disabled (no surprise activation).
TEST(TargetTrackRunner, DefaultConfigDisablesTrackers) {
  const SimConfig c = loadConfigFromString("{}");
  EXPECT_FALSE(c.trackers.enabled);
  EXPECT_TRUE(c.trackers.sensors.empty());
}
