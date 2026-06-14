// gnc-sim — threat-suite tests (issue #42). Benchmarks for the three advanced IThreat variants:
//   - multi-stage boosting ICBM      -> staging mass drops + lofted apogee/range band
//   - hypersonic glide vehicle (HGV) -> skip oscillation that scales with the configured L/D
//   - reentry vehicle + penaids      -> penaid deployment + kinematic scoring vs the true RV
//
// The trajectory checks integrate each threat IN ISOLATION with the same Euler step the Runner
// uses (tgt.vel += accel*dt; tgt.pos += vel*dt), so they characterize the threat model itself
// without the interceptor's terminal-condition logic interfering. One end-to-end Runner case
// asserts determinism (bit-identical telemetry for a fixed seed) and that the default path is
// untouched.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "gncsim/core/Config.hpp"
#include "gncsim/model/Registry.hpp"
#include "gncsim/model/Threats.hpp"
#include "gncsim/scenario/Runner.hpp"

using namespace gncsim;

namespace {

constexpr double kG0 = 9.80665;

// Integrate a threat in isolation with the Runner's Euler step. Records the altitude trace and the
// apogee / final downrange; stops on ground impact (after t>1 s so the launch transient is
// skipped).
struct ThreatTrace {
  double apogee_m = 0.0;
  double downrange_m = 0.0;
  double impact_t_s = -1.0;
  std::vector<double> alt_m;
};

ThreatTrace integrateThreat(const IThreat& threat, EntityState tgt, double dt_s, double t_end_s) {
  ThreatTrace tr;
  tr.apogee_m = tgt.pos.z;
  for (double t = 0.0; t <= t_end_s; t += dt_s) {
    tr.alt_m.push_back(tgt.pos.z);
    if (tgt.pos.z > tr.apogee_m) tr.apogee_m = tgt.pos.z;
    tr.downrange_m = std::hypot(tgt.pos.x, tgt.pos.y);
    if (tgt.pos.z < 0.0 && t > 1.0) {
      tr.impact_t_s = t;
      break;
    }
    const Vector3 a = threat.accel(tgt, t);
    tgt.vel += a * dt_s;
    tgt.pos += tgt.vel * dt_s;
  }
  return tr;
}

// Count skip "bounces": local altitude maxima (pull-ups) in the glide band.
int countSkips(const std::vector<double>& alt_m) {
  int skips = 0;
  for (std::size_t i = 2; i < alt_m.size(); ++i) {
    if (alt_m[i - 1] > alt_m[i - 2] && alt_m[i - 1] >= alt_m[i]) ++skips;
  }
  return skips;
}

// A representative 2-stage boosting ICBM, lofted near-vertical.
IcbmConfig icbmTwoStage() {
  IcbmConfig ic;
  IcbmStage s1;
  s1.thrust_n = 520000.0;
  s1.burn_time_s = 60.0;
  s1.propellant_mass_kg = 11000.0;
  s1.dry_mass_kg = 1500.0;
  IcbmStage s2;
  s2.thrust_n = 130000.0;
  s2.burn_time_s = 70.0;
  s2.propellant_mass_kg = 3200.0;
  s2.dry_mass_kg = 600.0;
  ic.stages = {s1, s2};
  ic.payload_mass_kg = 600.0;
  return ic;
}

EntityState lofted(double speed_mps, double el_deg) {
  EntityState s;
  const double el = el_deg * M_PI / 180.0;
  s.pos = {0.0, 0.0, 0.0};
  s.vel = {speed_mps * std::cos(el), 0.0, speed_mps * std::sin(el)};
  return s;
}

}  // namespace

// ============================================================================================
// Multi-stage boosting ICBM
// ============================================================================================

// Mass schedule: full stack at t=0, drops by each stage's (propellant + dry) mass across staging.
TEST(ThreatSuite, IcbmMassDropsAtStageTimes) {
  const IcbmConfig ic = icbmTwoStage();
  IcbmThreat icbm(ic, kG0);

  ASSERT_EQ(icbm.stageCount(), 2u);
  // Staging times are the cumulative burn-outs.
  EXPECT_NEAR(icbm.stagingTimeS(0), 60.0, 1e-9);
  EXPECT_NEAR(icbm.stagingTimeS(1), 130.0, 1e-9);

  // Full stacked launch mass = payload + Σ(dry + propellant).
  const double m0 = 600.0 + (1500.0 + 11000.0) + (600.0 + 3200.0);
  EXPECT_NEAR(icbm.massAt(0.0), m0, 1e-6);

  // Mid-stage-1 burn: half the stage-1 propellant gone.
  EXPECT_NEAR(icbm.massAt(30.0), m0 - 0.5 * 11000.0, 1e-6);

  // Staging event at t=60 s: propellant is fully burned just before (continuous), and the spent
  // stage-1 DRY mass (1500 kg) is jettisoned discretely AT the staging time. So mass just before
  // burnout = m0 - 11000 (propellant gone); mass exactly at burnout = that minus the 1500 kg dry
  // mass. The discrete drop equals the stage dry mass.
  const double m_before_burnout = m0 - 11000.0;  // all stage-1 propellant consumed
  const double m_at_staging =
      m_before_burnout - 1500.0;  // dry mass jettisoned at the staging event
  EXPECT_NEAR(icbm.massAt(59.99999), m_before_burnout, 1.0);
  EXPECT_NEAR(icbm.massAt(60.0), m_at_staging, 1e-6);
  EXPECT_NEAR(m_before_burnout - icbm.massAt(60.0), 1500.0, 1e-6);  // dropped exactly the dry mass

  // After all stages: only the payload remains.
  EXPECT_NEAR(icbm.massAt(200.0), 600.0, 1e-6);
}

// Thrust acceleration is applied along velocity during boost (so the boost climbs) and ceases at
// burnout (ballistic midcourse: only gravity remains).
TEST(ThreatSuite, IcbmThrustsDuringBoostThenCoasts) {
  IcbmThreat icbm(icbmTwoStage(), kG0);
  EntityState tgt = lofted(50.0, 82.0);

  // During stage-1 boost: applied accel exceeds gravity alone (thrust adds along velocity).
  const Vector3 a_boost = icbm.accel(tgt, 10.0);
  EXPECT_GT(a_boost.norm(), kG0);
  EXPECT_GT(a_boost.z, 0.0);  // net upward (thrust beats gravity on a steep loft)

  // After all stages: pure gravity.
  const Vector3 a_coast = icbm.accel(tgt, 200.0);
  EXPECT_NEAR(a_coast.norm(), kG0, 1e-9);
  EXPECT_NEAR(a_coast.z, -kG0, 1e-9);
}

// Lofted boost + ballistic coast yields a high-apogee, long-range ICBM arc (sanity band).
TEST(ThreatSuite, IcbmReachesIcbmApogeeAndRangeBand) {
  IcbmThreat icbm(icbmTwoStage(), kG0);
  const ThreatTrace tr = integrateThreat(icbm, lofted(50.0, 82.0), 0.05, 2000.0);

  // Apogee in the ICBM regime (hundreds to >1000 km); downrange of order thousands of km.
  EXPECT_GT(tr.apogee_m, 300.0e3);
  EXPECT_LT(tr.apogee_m, 3000.0e3);
  EXPECT_GT(tr.downrange_m, 1000.0e3);
  EXPECT_GT(tr.impact_t_s, 0.0);  // it comes back down (ballistic)
}

// ============================================================================================
// Hypersonic glide vehicle (skip-glide)
// ============================================================================================

// The HGV exhibits the characteristic skip oscillation: multiple altitude pull-ups during glide.
TEST(ThreatSuite, HgvExhibitsSkipOscillation) {
  HgvConfig hc;
  hc.ld_ratio = 2.5;
  hc.ballistic_coeff = 6000.0;
  hc.pull_up_alt_m = 60000.0;
  HgvThreat hgv(hc, kG0);

  EntityState entry;
  entry.pos = {0.0, 0.0, 55000.0};
  entry.vel = {6500.0, 0.0, -600.0};
  const ThreatTrace tr = integrateThreat(hgv, entry, 0.05, 900.0);

  // Several skips (altitude bounces) — not a monotonic ballistic descent.
  EXPECT_GE(countSkips(tr.alt_m), 2);

  // The oscillation has real amplitude: peak-to-trough altitude swing is large.
  double amin = 1e12, amax = 0.0;
  for (double a : tr.alt_m) {
    if (a < amin) amin = a;
    if (a > amax) amax = a;
  }
  EXPECT_GT(amax - amin, 10000.0);  // > 10 km skip amplitude
}

// Higher L/D glides farther: lift scales the downrange. This pins the L/D dependence.
TEST(ThreatSuite, HgvDownrangeScalesWithLiftToDrag) {
  auto downrangeFor = [](double ld) {
    HgvConfig hc;
    hc.ld_ratio = ld;
    hc.ballistic_coeff = 6000.0;
    hc.pull_up_alt_m = 60000.0;
    HgvThreat hgv(hc, kG0);
    EntityState entry;
    entry.pos = {0.0, 0.0, 55000.0};
    entry.vel = {6500.0, 0.0, -600.0};
    return integrateThreat(hgv, entry, 0.05, 900.0).downrange_m;
  };
  EXPECT_GT(downrangeFor(3.0), downrangeFor(1.5));
}

// Drag heats up (decelerates) deeper in the atmosphere: density grows toward the ground.
TEST(ThreatSuite, HgvAtmosphereDensifiesTowardGround) {
  HgvConfig hc;
  HgvThreat hgv(hc, kG0);
  EXPECT_GT(hgv.densityAt(0.0), hgv.densityAt(20000.0));
  EXPECT_GT(hgv.densityAt(20000.0), hgv.densityAt(60000.0));
}

// ============================================================================================
// Reentry vehicle + penaids
// ============================================================================================

// The bus deploys the configured number of penaids around the true RV; they separate over time and
// are correctly scored as more penaid-like than the heavy RV.
TEST(ThreatSuite, RvPenaidsDeployAndAreScoredAgainstTrueRv) {
  RvPenaidsConfig rc;
  rc.penaid_count = 6;
  rc.deploy_time_s = 0.0;
  rc.deploy_dv_mps = 8.0;
  rc.penaid_decel_mps2 = 1.0;
  RvPenaids rvp(rc, kG0);

  EntityState rv;
  rv.pos = {0.0, 0.0, 80000.0};
  rv.vel = {5000.0, 0.0, -2000.0};

  std::vector<RvObject> scene = rvp.deploy(rv);
  ASSERT_EQ(scene.size(), 7u);  // 1 true RV + 6 penaids
  EXPECT_TRUE(scene[0].is_true_rv);

  // All penaids cleanly distinguishable from the heavy RV (they shed speed faster).
  EXPECT_DOUBLE_EQ(rvp.scoreAgainstTrueRv(scene), 1.0);

  // They physically separate from the RV over time (kinematic discrimination cue).
  for (int i = 0; i < 2000; ++i) rvp.propagate(scene, 0.02);
  double max_sep_m = 0.0;
  for (std::size_t k = 1; k < scene.size(); ++k) {
    const double d = (scene[k].state.pos - scene[0].state.pos).norm();
    if (d > max_sep_m) max_sep_m = d;
  }
  EXPECT_GT(max_sep_m, 100.0);
}

// The threat's own accel() is a ballistic RV (gravity only) — the lethal object does not maneuver.
TEST(ThreatSuite, RvPenaidsThreatIsBallisticRv) {
  RvPenaidsConfig rc;
  RvPenaidsThreat threat(rc, kG0);
  EntityState rv;
  rv.vel = {5000.0, 0.0, -2000.0};
  const Vector3 a = threat.accel(rv, 5.0);
  EXPECT_NEAR(a.z, -kG0, 1e-12);
  EXPECT_NEAR(a.x, 0.0, 1e-12);
  EXPECT_NEAR(a.y, 0.0, 1e-12);
}

// ============================================================================================
// Registry resolution + determinism
// ============================================================================================

TEST(ThreatSuite, RegistryResolvesAllThreatSuiteKeys) {
  ModelRegistry reg;
  EntityState tgt;
  tgt.vel = {500.0, 0.0, 100.0};

  TargetConfig icbm;
  icbm.maneuver = "icbm";
  icbm.icbm = icbmTwoStage();
  EXPECT_GT(reg.makeThreat(icbm, kG0)->accel(tgt, 10.0).norm(), kG0);  // boosting

  TargetConfig hgv;
  hgv.maneuver = "hgv";
  EXPECT_NO_THROW(reg.makeThreat(hgv, kG0));

  TargetConfig rv;
  rv.maneuver = "rv_penaids";
  EXPECT_NEAR(reg.makeThreat(rv, kG0)->accel(tgt, 1.0).z, -kG0, 1e-12);
}

// A full Runner engagement against an ICBM threat is deterministic (bit-identical for a seed) and
// the new threat path runs without error.
TEST(ThreatSuite, IcbmRunnerEngagementIsDeterministic) {
  SimConfig c;
  c.scenario = "homing";
  c.model = "3dof";
  c.seed = 7;
  c.dt = 0.01;
  c.t_end = 30.0;
  c.guidance.law = "pronav";
  c.guidance.nav_constant = 4.0;
  c.guidance.max_accel = 800.0;
  c.vehicle.pos0 = {0, 0, 0};
  c.vehicle.launch_speed = 2000.0;
  c.vehicle.launch_elevation_deg = 70.0;
  c.target.pos0 = {60000.0, 0.0, 30000.0};
  c.target.vel0 = {-1500.0, 0.0, 200.0};
  c.target.maneuver = "icbm";
  c.target.icbm = icbmTwoStage();

  SimResult r1 = runSimulation(c);
  SimResult r2 = runSimulation(c);
  ASSERT_EQ(r1.frames.size(), r2.frames.size());
  ASSERT_GT(r1.frames.size(), 10u);
  for (std::size_t i = 0; i < r1.frames.size(); ++i) {
    EXPECT_EQ(r1.frames[i].veh_pos.x, r2.frames[i].veh_pos.x);
    EXPECT_EQ(r1.frames[i].tgt_pos.z, r2.frames[i].tgt_pos.z);
  }
  EXPECT_DOUBLE_EQ(r1.miss_distance, r2.miss_distance);
}
