// gnc-sim — battle-management / C2 fire-control datalink tests (issue #46). World frame ENU, SI.
//
// The C2 loop fuses a sensor network into one track picture (issue #5), the cueing logic decides
// when/where to launch on it (issue #8), and the datalink (issue #46) carries that picture through
// a configurable latency + stochastic dropout. These tests pin the model's load-bearing claims:
//
//   A) ZERO latency + ZERO dropout reduces EXACTLY to the existing cued behaviour: enabling the
//      datalink with latency_s=0, dropout_prob=0 gives the byte-identical miss/launch_time of the
//      same run with the datalink disabled (the fused picture passes straight through).
//   B) Increasing datalink latency degrades P(kill) MONOTONICALLY: a staler picture aims the
//      interceptor further off the (fast/weaving) threat's true position, so miss grows with
//      latency and the single-shot P(kill) falls monotonically.
//   C) Increasing dropout probability degrades P(kill) monotonically (held/stale picture ages).
//   D) The fused track NETWORK beats an isolated sensor: radar+IR fusion lowers the miss (and lands
//      the intercept) versus either the radar-only or IR-only track.
//   E) Determinism: same seed -> bit-identical outcome, even with dropout active.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "gncsim/core/Config.hpp"
#include "gncsim/scenario/Runner.hpp"

using namespace gncsim;

namespace {

constexpr double kLethalRadius = 3.0;  // mirror Runner.cpp's intercept criterion [m]

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

// A cued engagement on a fused radar+IR track against a fast, weaving threat — the regime where a
// stale fire-control picture matters. At zero latency/dropout the network intercepts.
SimConfig baseBmc2Config() {
  SimConfig c;
  c.scenario = "bmc2_test";
  c.model = "3dof";
  c.seed = 7;
  c.dt = 0.005;
  c.t_end = 40.0;
  c.vehicle.pos0 = {0, 0, 0};
  c.vehicle.launch_speed = 1300;
  c.vehicle.launch_elevation_deg = 42;
  c.guidance.law = "pronav";
  c.guidance.nav_constant = 4.0;
  c.guidance.max_accel = 500.0;
  c.guidance.time_constant = 0.1;
  c.sensors.enable = false;
  c.target.pos0 = {14000, 0, 5200};
  c.target.vel0 = {-320, 0, -45};
  c.target.maneuver = "weave";
  c.target.maneuver_g = 4.0;
  c.target.maneuver_freq = 0.4;
  c.trackers.enabled = true;
  c.trackers.process_psd = 50.0;
  c.trackers.sensors = {radarCfg(), irCfg()};
  c.cueing.enabled = true;
  c.cueing.launch_criterion = "track_cov";
  c.cueing.cov_trace_threshold = 200.0;
  c.cueing.max_cue_time = 6.0;
  return c;
}

// Gaussian (Carleton) single-shot P(kill) from the analytic CPA miss distance: a standard lethality
// damage function. Strictly decreasing in miss, so a smaller miss == higher P(kill).
double pKill(double miss_m, double sigma_m = 5.0, double pk_max = 0.95) {
  return pk_max * std::exp(-0.5 * (miss_m / sigma_m) * (miss_m / sigma_m));
}

}  // namespace

// ── A) Zero latency + zero dropout == the datalink-disabled cued run (byte-identical) ────────────
TEST(Bmc2Datalink, ZeroLatencyZeroDropoutMatchesNoDatalink) {
  const SimConfig disabled = baseBmc2Config();  // datalink.enabled defaults false
  SimConfig zero = baseBmc2Config();
  zero.trackers.datalink.enabled = true;
  zero.trackers.datalink.latency_s = 0.0;
  zero.trackers.datalink.dropout_prob = 0.0;

  const SimResult rd = runSimulation(disabled);
  const SimResult rz = runSimulation(zero);

  // Identical engagement: same miss, same launch time, same intercept verdict.
  EXPECT_DOUBLE_EQ(rd.miss_distance, rz.miss_distance);
  EXPECT_DOUBLE_EQ(rd.launch_time, rz.launch_time);
  EXPECT_EQ(rd.intercept, rz.intercept);
  // And the no-datalink network run actually intercepts (sanity for the regime).
  EXPECT_TRUE(rd.intercept);
  EXPECT_LT(rd.miss_distance, kLethalRadius);
}

// ── B) Increasing latency degrades P(kill) monotonically ─────────────────────────────────────────
TEST(Bmc2Datalink, LatencyDegradesPkillMonotonically) {
  const std::vector<double> latencies_s = {0.0, 0.1, 0.2, 0.4, 0.8, 1.5};
  double prev_miss = -1.0;
  double prev_pk = 2.0;
  for (double latency_s : latencies_s) {
    SimConfig c = baseBmc2Config();
    c.trackers.datalink.enabled = latency_s > 0.0;
    c.trackers.datalink.latency_s = latency_s;
    const SimResult r = runSimulation(c);
    const double pk = pKill(r.miss_distance);
    // Miss grows (P(kill) falls) with every increase in latency.
    EXPECT_GT(r.miss_distance, prev_miss) << "latency_s=" << latency_s;
    EXPECT_LT(pk, prev_pk + 1e-12) << "latency_s=" << latency_s;
    prev_miss = r.miss_distance;
    prev_pk = pk;
  }
  // The largest latency has decisively lost the intercept the zero-latency case won.
  SimConfig fresh = baseBmc2Config();
  SimConfig stale = baseBmc2Config();
  stale.trackers.datalink.enabled = true;
  stale.trackers.datalink.latency_s = 1.5;
  EXPECT_TRUE(runSimulation(fresh).intercept);
  EXPECT_FALSE(runSimulation(stale).intercept);
}

// ── C) Increasing dropout probability degrades P(kill) monotonically (in the ENSEMBLE mean) ──────
// A single drop is stochastic — one seed's held picture can happen to align favourably — so the
// claim is statistical: averaged over a seed ensemble, a higher message-loss rate ages the picture
// more on average, growing the mean miss (lowering mean P(kill)). Compares well-separated dropout
// levels at a fixed baseline transport lag.
TEST(Bmc2Datalink, DropoutDegradesPkillMonotonically) {
  const std::vector<double> dropouts = {0.0, 0.3, 0.6, 0.9};
  constexpr int kSeeds = 40;
  double prev_mean_miss = -1.0;
  double prev_mean_pk = 2.0;
  for (double dropout_prob : dropouts) {
    double sum_miss = 0.0;
    double sum_pk = 0.0;
    for (int s = 0; s < kSeeds; ++s) {
      SimConfig c = baseBmc2Config();
      c.seed = 100 + s;
      c.trackers.datalink.enabled = true;
      c.trackers.datalink.latency_s = 0.1;  // a baseline transport lag so a drop genuinely ages it
      c.trackers.datalink.dropout_prob = dropout_prob;
      const SimResult r = runSimulation(c);
      sum_miss += r.miss_distance;
      sum_pk += pKill(r.miss_distance);
    }
    const double mean_miss = sum_miss / kSeeds;
    const double mean_pk = sum_pk / kSeeds;
    EXPECT_GT(mean_miss, prev_mean_miss) << "dropout_prob=" << dropout_prob;
    EXPECT_LT(mean_pk, prev_mean_pk + 1e-12) << "dropout_prob=" << dropout_prob;
    prev_mean_miss = mean_miss;
    prev_mean_pk = mean_pk;
  }
}

// ── D) The fused sensor NETWORK beats an isolated sensor (lower error, lands the intercept)
// ───────
TEST(Bmc2Network, FusedNetworkBeatsSingleSensor) {
  SimConfig network = baseBmc2Config();  // radar + IR
  SimConfig radar_only = baseBmc2Config();
  radar_only.trackers.sensors = {radarCfg()};
  SimConfig ir_only = baseBmc2Config();
  ir_only.trackers.sensors = {irCfg()};

  const SimResult rn = runSimulation(network);
  const SimResult rr = runSimulation(radar_only);
  const SimResult ri = runSimulation(ir_only);

  // The two-sensor track is more accurate than either single sensor's track.
  EXPECT_LT(rn.miss_distance, rr.miss_distance);
  EXPECT_LT(rn.miss_distance, ri.miss_distance);
  // The network lands the intercept; the angles-only IR-only track (no range) does not.
  EXPECT_TRUE(rn.intercept);
  EXPECT_FALSE(ri.intercept);
  // And higher P(kill) follows the smaller miss.
  EXPECT_GT(pKill(rn.miss_distance), pKill(rr.miss_distance));
  EXPECT_GT(pKill(rn.miss_distance), pKill(ri.miss_distance));
}

// ── E) Determinism: same seed -> identical outcome, even with dropout active ─────────────────────
TEST(Bmc2Datalink, DeterministicForFixedSeed) {
  SimConfig c = baseBmc2Config();
  c.trackers.datalink.enabled = true;
  c.trackers.datalink.latency_s = 0.15;
  c.trackers.datalink.dropout_prob = 0.3;

  const SimResult a = runSimulation(c);
  const SimResult b = runSimulation(c);
  EXPECT_DOUBLE_EQ(a.miss_distance, b.miss_distance);
  EXPECT_DOUBLE_EQ(a.launch_time, b.launch_time);
  EXPECT_EQ(a.intercept, b.intercept);
  ASSERT_EQ(a.frames.size(), b.frames.size());
  // Frame-level: the delivered track picture (and everything downstream) is bit-identical.
  for (std::size_t k = 0; k < a.frames.size(); ++k) {
    EXPECT_DOUBLE_EQ(a.frames[k].track_pos_est.x, b.frames[k].track_pos_est.x);
    EXPECT_DOUBLE_EQ(a.frames[k].veh_pos.x, b.frames[k].veh_pos.x);
  }

  // A different seed perturbs the dropout/measurement draws -> a different (still valid) outcome.
  SimConfig d = c;
  d.seed = 12345;
  const SimResult rd = runSimulation(d);
  EXPECT_NE(a.miss_distance, rd.miss_distance);
}
