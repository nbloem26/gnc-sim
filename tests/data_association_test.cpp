// gnc-sim — multi-target data association + lifecycle + track-to-track fusion tests (issue #38).
// World frame ENU, SI units.
//   A) JPDA maintains the TRUE track through a decoy/clutter scene: quantified track PURITY (the
//      fraction of looks whose highest-probability gated detection truly came from the target)
//      stays high and DEGRADES MONOTONICALLY as clutter density rises; the track stays accurate.
//   B) Track lifecycle: Tentative -> Confirmed (M-of-N) -> Deleted (consecutive misses)
//   transitions. C) Track-to-track fusion (Covariance Intersection) lowers the covariance vs each
//   input and stays
//      consistent (the fused estimate's NIS against truth is statistically reasonable, never
//      optimistic).
//   D) Determinism: same seed -> identical JPDA track + purity.
//   E) Runner integration: the JPDA config tracks well + intercepts; the default associator path is
//      byte-identical to the issue-#5 fused path.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "gncsim/core/Config.hpp"
#include "gncsim/core/Rng.hpp"
#include "gncsim/gnc/DataAssociation.hpp"
#include "gncsim/gnc/TargetTrackEkf.hpp"
#include "gncsim/math/Vector3.hpp"
#include "gncsim/scenario/Runner.hpp"

using namespace gncsim;

namespace {

TrackSensor radarAt(const Vector3& p) {
  TrackSensor s;
  s.type = TrackSensorType::Radar;
  s.pos = p;
  s.sigma_az = 0.002;
  s.sigma_el = 0.002;
  s.sigma_range = 25.0;
  s.sigma_range_rate = 3.0;
  return s;
}

// Noisy radar measurement of an object's absolute state, drawing from the run Rng (az,el,range,rr).
std::vector<double> measureNoisy(const TrackSensor& s, const Vector3& tp, const Vector3& tv,
                                 Rng& rng) {
  const Vector3 rel = tp - s.pos;
  const double horiz = std::sqrt(rel.x * rel.x + rel.y * rel.y);
  const double r = rel.norm();
  const double az = std::atan2(rel.y, rel.x) + rng.gaussian(0.0, s.sigma_az);
  const double el = std::atan2(rel.z, horiz) + rng.gaussian(0.0, s.sigma_el);
  const double rr = (r > 1e-9 ? rel.dot(tv) / r : 0.0) + rng.gaussian(0.0, s.sigma_range_rate);
  return {az, el, r + rng.gaussian(0.0, s.sigma_range), rr};
}

int poisson(double mean, Rng& rng) {
  if (mean <= 0.0) return 0;
  const double l = std::exp(-mean);
  int k = 0;
  double p = 1.0;
  do {
    ++k;
    p *= rng.uniform(0.0, 1.0);
  } while (p > l && k < 64);
  return k - 1;
}

// Drive a single-radar JPDA track through a 3-decoy scene + Poisson clutter and report the achieved
// track purity (fraction of scored looks whose best-beta detection truly came from the target) and
// the steady-state position RMS. Deterministic for a fixed seed.
struct JpdaRun {
  double purity = 0.0;
  double rms_m = 0.0;
  TrackStatus final_status = TrackStatus::Tentative;
};

JpdaRun runRadarJpda(double clutter_rate, int num_cso, std::uint64_t seed) {
  const double dt = 0.005;
  const TrackSensor radar = radarAt({0.0, 0.0, 0.0});
  const Vector3 p0{9000.0, 0.0, 3500.0};
  const Vector3 v0{-280.0, 0.0, -40.0};

  Rng rng(seed);
  std::vector<Vector3> cso_pos;
  std::vector<Vector3> cso_vel;
  for (int k = 0; k < num_cso; ++k) {
    cso_pos.push_back(p0 + rng.gaussianVec(120.0));
    cso_vel.push_back(v0 + rng.gaussianVec(4.0));
  }

  TargetTrackEkf ekf(dt, 50.0);
  ekf.bootstrap(p0, v0, 30.0, 30.0);
  TrackLifecycle life(TrackLifecycleParams{3, 5, 6});
  const JpdaParams jp{0.95, 16.0, 1e-4};

  int hits = 0;
  int total = 0;
  double sse = 0.0;
  int n = 0;
  const int steps = 4000;
  for (int i = 1; i <= steps; ++i) {
    const Vector3 tp = p0 + v0 * (i * dt);
    const Vector3 tv = v0;
    ekf.predict();

    std::vector<AssocDetection> dets;
    dets.push_back(AssocDetection{measureNoisy(radar, tp, tv, rng), true});
    for (int k = 0; k < num_cso; ++k) {
      const Vector3 op =
          cso_pos[static_cast<std::size_t>(k)] + cso_vel[static_cast<std::size_t>(k)] * (i * dt);
      dets.push_back(AssocDetection{
          measureNoisy(radar, op, cso_vel[static_cast<std::size_t>(k)], rng), false});
    }
    const int nc = poisson(clutter_rate, rng);
    if (nc > 0) {
      const std::vector<double> zt = measureNoisy(radar, tp, tv, rng);
      for (int c = 0; c < nc; ++c) {
        std::vector<double> z = zt;
        z[0] += rng.uniform(-0.02, 0.02);
        z[1] += rng.uniform(-0.02, 0.02);
        z[2] += rng.uniform(-300.0, 300.0);
        dets.push_back(AssocDetection{z, false});
      }
    }

    const JpdaResult jr = jpdaAssociateAndUpdate(ekf, radar, dets, jp);
    life.update(jr.any_gated);
    if (jr.any_gated) {
      ++total;
      if (jr.best_is_target) ++hits;
    }
    if (i >= 400) {
      sse += (ekf.pos() - tp).normSq();
      ++n;
    }
  }

  JpdaRun out;
  out.purity = total > 0 ? static_cast<double>(hits) / total : 0.0;
  out.rms_m = n > 0 ? std::sqrt(sse / n) : 0.0;
  out.final_status = life.status();
  return out;
}

// 6-state estimate with diagonal covariance for the CI tests.
TrackEstimate diagEstimate(const Vector3& pos, const Vector3& vel, double pos_var, double vel_var) {
  TrackEstimate t;
  t.x = {pos.x, pos.y, pos.z, vel.x, vel.y, vel.z};
  t.p[0] = t.p[7] = t.p[14] = pos_var;
  t.p[21] = t.p[28] = t.p[35] = vel_var;
  return t;
}

}  // namespace

// ── A) JPDA holds the true track through decoys + clutter; purity degrades with clutter ─────────
TEST(Jpda, MaintainsTrueTrackThroughDecoysAndClutter) {
  const JpdaRun r = runRadarJpda(/*clutter_rate=*/2.0, /*num_cso=*/3, /*seed=*/7);
  // The associator is essentially never fooled: best-beta detection is the true target on the vast
  // majority of looks, and the track stays metres-accurate despite 3 decoys + clutter.
  EXPECT_GT(r.purity, 0.90);
  EXPECT_LT(r.rms_m, 10.0);
  EXPECT_EQ(r.final_status, TrackStatus::Confirmed);
}

TEST(Jpda, PurityDegradesMonotonicallyWithClutterDensity) {
  const JpdaRun c0 = runRadarJpda(0.0, 3, 7);
  const JpdaRun c2 = runRadarJpda(2.0, 3, 7);
  const JpdaRun c10 = runRadarJpda(10.0, 3, 7);
  // More clutter -> lower (or equal) track purity. The clean scene is near-perfect; even at heavy
  // clutter the true track is retained well above chance.
  EXPECT_GT(c0.purity, 0.95);
  EXPECT_GE(c0.purity + 1e-9, c2.purity);
  EXPECT_GE(c2.purity + 1e-9, c10.purity);
  EXPECT_GT(c10.purity, 0.85);
}

TEST(Jpda, NoClutterNoDecoysIsNearPerfectPurity) {
  const JpdaRun r = runRadarJpda(0.0, 0, 7);
  EXPECT_GT(r.purity, 0.99);
  EXPECT_LT(r.rms_m, 5.0);
}

// ── B) Track lifecycle: confirmation (M-of-N) and deletion (consecutive misses) ─────────────────
TEST(TrackLifecycleTest, ConfirmsOnMofNThenDeletesOnMissRun) {
  TrackLifecycle life(TrackLifecycleParams{3, 5, 4});
  EXPECT_EQ(life.status(), TrackStatus::Tentative);

  // Two hits: still tentative (need 3 of last 5).
  EXPECT_EQ(life.update(true), TrackStatus::Tentative);
  EXPECT_EQ(life.update(true), TrackStatus::Tentative);
  // Third hit confirms.
  EXPECT_EQ(life.update(true), TrackStatus::Confirmed);

  // Now a run of misses up to the deletion threshold.
  EXPECT_EQ(life.update(false), TrackStatus::Confirmed);  // 1 miss
  EXPECT_EQ(life.update(false), TrackStatus::Confirmed);  // 2
  EXPECT_EQ(life.update(false), TrackStatus::Confirmed);  // 3
  EXPECT_EQ(life.update(false), TrackStatus::Deleted);    // 4 -> delete
  // Deleted is terminal.
  EXPECT_EQ(life.update(true), TrackStatus::Deleted);
}

TEST(TrackLifecycleTest, SparseHitsNeverConfirm) {
  // Alternating hit/miss gives only 2-3 hits in a 5-window with M=4 -> never confirms.
  TrackLifecycle life(TrackLifecycleParams{4, 5, 100});
  TrackStatus st = TrackStatus::Tentative;
  for (int i = 0; i < 20; ++i) st = life.update(i % 2 == 0);
  EXPECT_EQ(st, TrackStatus::Tentative);
}

TEST(TrackLifecycleTest, MissRunResetsOnHit) {
  TrackLifecycle life(TrackLifecycleParams{1, 3, 3});
  life.update(true);  // confirm
  EXPECT_EQ(life.update(false), TrackStatus::Confirmed);
  EXPECT_EQ(life.update(false), TrackStatus::Confirmed);
  EXPECT_EQ(life.consecutive_misses(), 2);
  life.update(true);  // reset the miss run
  EXPECT_EQ(life.consecutive_misses(), 0);
  EXPECT_EQ(life.update(false), TrackStatus::Confirmed);  // miss run starts over -> not deleted yet
  EXPECT_EQ(life.status(), TrackStatus::Confirmed);
}

// ── C) Track-to-track fusion (Covariance Intersection) lowers covariance and stays consistent ───
TEST(CovarianceIntersection, LowersCovarianceWithComplementaryInputs) {
  // The realistic two-sensor case: estimate `a` is tight on position but loose on velocity (a
  // range-resolved radar fix), `b` is tight on velocity but loose on position. CI exploits the
  // COMPLEMENTARY information, so the fused covariance is far tighter than either input. (For two
  // IDENTICAL inputs CI is deliberately conservative and does NOT beat a single sensor — that is
  // the price of robustness to unknown cross-correlation; the win comes from complementary
  // information.)
  const Vector3 truth_pos{9000.0, 1000.0, 3500.0};
  const Vector3 truth_vel{-280.0, 30.0, -40.0};
  const TrackEstimate a = diagEstimate(truth_pos, truth_vel, /*pos_var=*/4.0, /*vel_var=*/1.0e6);
  const TrackEstimate b = diagEstimate(truth_pos, truth_vel, /*pos_var=*/1.0e6, /*vel_var=*/4.0);

  const TrackEstimate f = covarianceIntersection(a, b);
  EXPECT_LT(f.covTrace(), a.covTrace());
  EXPECT_LT(f.covTrace(), b.covTrace());
  // The fused estimate inherits each sensor's STRONG axis: position from a, velocity from b.
  EXPECT_LT(f.p[0], 100.0);   // position variance pinned by the position-accurate sensor
  EXPECT_LT(f.p[21], 100.0);  // velocity variance pinned by the velocity-accurate sensor
}

TEST(CovarianceIntersection, FavoursTheTighterInput) {
  const Vector3 truth_pos{0.0, 0.0, 0.0};
  const Vector3 truth_vel{0.0, 0.0, 0.0};
  // a is much tighter than b; the fused weight should favour a, so the fused mean sits near a.
  const TrackEstimate a = diagEstimate(Vector3{1.0, 0.0, 0.0}, truth_vel, 1.0, 1.0);
  const TrackEstimate b = diagEstimate(Vector3{100.0, 0.0, 0.0}, truth_vel, 1.0e6, 1.0e6);
  const double w = covarianceIntersectionWeight(a, b);
  EXPECT_GT(w, 0.5);  // weight on a (the tighter estimate) dominates
  const TrackEstimate f = covarianceIntersection(a, b);
  EXPECT_LT(std::fabs(f.x[0] - a.x[0]), std::fabs(f.x[0] - b.x[0]));
  // Fusing in a near-useless second sensor never makes the result worse than the good one alone.
  EXPECT_LE(f.covTrace(), a.covTrace() + 1e-6);
}

TEST(CovarianceIntersection, FusedEstimateStaysConsistent) {
  // Monte Carlo: two noisy INDEPENDENT measurements of a fixed truth. CI reports a CONSERVATIVE
  // covariance (it cannot know the inputs are independent), so the fused NIS against truth averages
  // BELOW its nominal DOF (6) — the hallmark of a consistent, non-optimistic fuser. The key safety
  // property is that the mean NIS never RISES much above DOF (which would mean an over-confident,
  // inconsistent covariance).
  const Vector3 truth_pos{500.0, -200.0, 100.0};
  const Vector3 truth_vel{10.0, -5.0, 2.0};
  const double pos_var = 25.0;  // 5 m std
  const double vel_var = 4.0;   // 2 m/s std
  Rng rng(2024);

  double nis_sum = 0.0;
  const int trials = 4000;
  for (int t = 0; t < trials; ++t) {
    const Vector3 pa = truth_pos + rng.gaussianVec(std::sqrt(pos_var));
    const Vector3 va = truth_vel + rng.gaussianVec(std::sqrt(vel_var));
    const Vector3 pb = truth_pos + rng.gaussianVec(std::sqrt(pos_var));
    const Vector3 vb = truth_vel + rng.gaussianVec(std::sqrt(vel_var));
    const TrackEstimate a = diagEstimate(pa, va, pos_var, vel_var);
    const TrackEstimate b = diagEstimate(pb, vb, pos_var, vel_var);
    const TrackEstimate f = covarianceIntersection(a, b);

    // NIS = e^T P_f^{-1} e against truth (diagonal P_f -> sum e_i^2 / P_ii).
    const std::array<double, 6> e{f.x[0] - truth_pos.x, f.x[1] - truth_pos.y, f.x[2] - truth_pos.z,
                                  f.x[3] - truth_vel.x, f.x[4] - truth_vel.y, f.x[5] - truth_vel.z};
    double nis = 0.0;
    // P_f is generally full; invert the 6x6 by reusing the consistency of CI: use the diagonal of
    // P_f as a conservative scale (CI keeps off-diagonals small for axis-aligned inputs). Use the
    // full quadratic form via the stored covariance trace bound: here inputs are axis-aligned, so
    // P_f is diagonal and this is exact.
    for (int i = 0; i < 6; ++i) {
      const double pii = f.p[i * 6 + i];
      nis += (e[static_cast<std::size_t>(i)] * e[static_cast<std::size_t>(i)]) / pii;
    }
    nis_sum += nis;
  }
  const double mean_nis = nis_sum / trials;
  // Conservative: independent inputs make the true fused error covariance ~P/2, while CI reports
  // ~P, so the mean NIS sits near DOF/2 = 3 — below DOF and definitely not over-confident.
  EXPECT_LT(mean_nis, 6.0);  // not optimistic / over-confident
  EXPECT_GT(mean_nis, 1.5);  // and not absurdly inflated
}

// ── D) Determinism ──────────────────────────────────────────────────────────────────────────────
TEST(Jpda, DeterministicForSameSeed) {
  const JpdaRun a = runRadarJpda(2.0, 3, 7);
  const JpdaRun b = runRadarJpda(2.0, 3, 7);
  EXPECT_DOUBLE_EQ(a.purity, b.purity);
  EXPECT_DOUBLE_EQ(a.rms_m, b.rms_m);
  const JpdaRun c = runRadarJpda(2.0, 3, 8);
  EXPECT_NE(a.rms_m, c.rms_m);
}

// ── E) Runner integration: JPDA config + default associator unchanged ───────────────────────────
namespace {

SimConfig baseJpdaConfig() {
  SimConfig c;
  c.scenario = "jpda_test";
  c.model = "3dof";
  c.seed = 7;
  c.dt = 0.005;
  c.t_end = 25.0;
  c.vehicle.pos0 = {0, 0, 0};
  c.vehicle.launch_speed = 900;
  c.vehicle.launch_elevation_deg = 42;
  c.guidance.law = "pronav";
  c.guidance.nav_constant = 4.0;
  c.guidance.max_accel = 400.0;
  c.sensors.enable = false;
  c.target.pos0 = {9000, 0, 3500};
  c.target.vel0 = {-280, 0, -40};
  c.trackers.enabled = true;
  c.trackers.process_psd = 50.0;
  TrackerSensorConfig radar;
  radar.type = "radar";
  radar.pos = {0, 0, 0};
  radar.sigma_az = 0.002;
  radar.sigma_el = 0.002;
  radar.sigma_range = 25.0;
  radar.sigma_range_rate = 3.0;
  TrackerSensorConfig ir;
  ir.type = "ir";
  ir.pos = {20000, 20000, 500000};
  ir.sigma_az = 0.0003;
  ir.sigma_el = 0.0003;
  c.trackers.sensors = {radar, ir};
  return c;
}

double trackRms(const SimResult& r, int settle = 400) {
  double sse = 0.0;
  int n = 0;
  for (std::size_t i = 0; i < r.frames.size(); ++i) {
    if (static_cast<int>(i) < settle) continue;
    sse += (r.frames[i].track_pos_est - r.frames[i].tgt_pos).normSq();
    ++n;
  }
  return n > 0 ? std::sqrt(sse / n) : 0.0;
}

}  // namespace

TEST(JpdaRunner, DefaultAssociatorIsByteIdenticalToFusedPath) {
  // association.mode defaults to "none" -> the issue-#5 single-target fusion path. A config with
  // the associator left at default must produce the exact same telemetry as the same config without
  // the association block at all.
  SimConfig fused = baseJpdaConfig();
  SimConfig fused_default = baseJpdaConfig();
  fused_default.trackers.association.mode = "none";

  const SimResult a = runSimulation(fused);
  const SimResult b = runSimulation(fused_default);
  ASSERT_EQ(a.frames.size(), b.frames.size());
  for (std::size_t i = 0; i < a.frames.size(); ++i) {
    EXPECT_DOUBLE_EQ(a.frames[i].track_pos_est.x, b.frames[i].track_pos_est.x);
    EXPECT_DOUBLE_EQ(a.frames[i].track_pos_est.z, b.frames[i].track_pos_est.z);
    EXPECT_DOUBLE_EQ(a.frames[i].veh_pos.x, b.frames[i].veh_pos.x);
  }
  EXPECT_DOUBLE_EQ(a.track_purity, 1.0);  // no association attempted -> purity is trivially 1
}

TEST(JpdaRunner, JpdaModeTracksAccuratelyAndStaysPure) {
  SimConfig c = baseJpdaConfig();
  c.trackers.association.mode = "jpda";
  c.trackers.association.num_cso = 3;
  c.trackers.association.clutter_rate = 2.0;
  const SimResult r = runSimulation(c);
  EXPECT_GT(r.track_purity, 0.90);
  EXPECT_LT(trackRms(r), 15.0);
  EXPECT_TRUE(r.intercept);  // an accurate fused track flies the interceptor home
}

TEST(JpdaRunner, DeterministicSameSeed) {
  SimConfig c = baseJpdaConfig();
  c.trackers.association.mode = "jpda";
  const SimResult a = runSimulation(c);
  const SimResult b = runSimulation(c);
  ASSERT_EQ(a.frames.size(), b.frames.size());
  EXPECT_DOUBLE_EQ(a.track_purity, b.track_purity);
  EXPECT_DOUBLE_EQ(a.frames.back().track_pos_est.x, b.frames.back().track_pos_est.x);
}

TEST(JpdaRunner, ParsesAssociationConfigBlock) {
  const std::string js = R"({
    "scenario":"t","model":"3dof","trackers":{
      "enabled":true,"process_psd":50.0,
      "sensors":[{"type":"radar","pos":[0,0,0]}],
      "association":{"mode":"jpda","prob_detect":0.8,"gate_chi2":12.0,"clutter_rate":3.0,
                     "confirm_m":2,"confirm_n":4,"delete_misses":5,"num_cso":4,
                     "cso_separation_m":60.0,"init_pos_sigma_m":40.0}
    }})";
  const SimConfig c = loadConfigFromString(js);
  EXPECT_EQ(c.trackers.association.mode, "jpda");
  EXPECT_DOUBLE_EQ(c.trackers.association.prob_detect, 0.8);
  EXPECT_DOUBLE_EQ(c.trackers.association.gate_chi2, 12.0);
  EXPECT_EQ(c.trackers.association.confirm_m, 2);
  EXPECT_EQ(c.trackers.association.num_cso, 4);
  EXPECT_DOUBLE_EQ(c.trackers.association.init_pos_sigma_m, 40.0);
}

TEST(JpdaRunner, UnknownAssociatorFallsBackToNone) {
  const std::string js = R"({
    "trackers":{"enabled":true,"sensors":[{"type":"radar"}],
                "association":{"mode":"bogus"}}})";
  const SimConfig c = loadConfigFromString(js);
  EXPECT_EQ(c.trackers.association.mode, "none");
}
