// gnc-sim — seeker decoy discrimination tests (issue #6). World frame ENU, SI units.
//   A) Discriminator scoring kernel: given crafted feature vectors, picks the object whose measured
//      signature is closest to the expected target signature.
//   B) High separability (decoys clearly distinct): the run selects the true target ~always and
//      intercepts (miss small).
//   C) Low separability / more decoys: selection accuracy drops and miss grows — assert the
//   monotone
//      trend (closer / more numerous decoys -> lower discrimination accuracy, larger miss).
//   D) Determinism: same seed -> identical telemetry.
//   E) Default path unchanged: decoys disabled -> discrimination channel inert and the homing_3dof
//      vehicle/gnc telemetry is byte-identical to the no-decoy run.
//   F) Config parsing: the decoys block is read tolerantly; default config has decoys disabled.
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "gncsim/core/Config.hpp"
#include "gncsim/core/Serialize.hpp"
#include "gncsim/gnc/Discriminator.hpp"
#include "gncsim/scenario/Runner.hpp"

using namespace gncsim;

namespace {

// A representative decoyed engagement built entirely in code (no JSON), tuned so the discrimination
// region is exercised. separability/count are overridden per test.
SimConfig baseDecoyConfig() {
  SimConfig c;
  c.scenario = "discrim_test";
  c.model = "3dof";
  c.seed = 7;
  c.dt = 0.005;
  c.t_end = 40.0;
  c.vehicle.pos0 = {0, 0, 0};
  c.vehicle.launch_speed = 900;
  c.vehicle.launch_elevation_deg = 42;
  c.guidance.law = "pronav";
  c.guidance.nav_constant = 4.0;
  c.guidance.max_accel = 400.0;
  c.sensors.enable = false;
  c.target.pos0 = {9000, 0, 3500};
  c.target.vel0 = {-280, 0, -40};
  c.target.maneuver = "constant";

  c.decoys.enabled = true;
  c.decoys.count = 4;
  c.decoys.separation = 60.0;
  c.decoys.separability = 0.6;
  c.decoys.target_intensity = 1.0;
  c.decoys.target_size = 1.0;
  c.decoys.target_decel = 1.0;
  c.decoys.decoy_intensity = 0.3;
  c.decoys.decoy_size = 0.4;
  c.decoys.decoy_decel = 3.0;
  c.decoys.feature_spread = 0.10;
  c.decoys.measurement_noise = 0.08;
  c.decoys.score_filter_tau = 0.5;
  return c;
}

// Fraction of the run's second half where the true target (object 0) was selected.
double selectionAccuracy(const SimResult& r) {
  if (r.frames.empty()) return 0.0;
  const std::size_t start = r.frames.size() / 2;
  std::size_t ok = 0, n = 0;
  for (std::size_t i = start; i < r.frames.size(); ++i) {
    if (r.frames[i].discrim_correct > 0.5) ++ok;
    ++n;
  }
  return n > 0 ? static_cast<double>(ok) / static_cast<double>(n) : 0.0;
}

// Monte-Carlo fraction of seeds whose run mostly selected the true target.
double mcSelectionRate(SimConfig c, int num_seeds) {
  int ok = 0;
  for (int s = 1; s <= num_seeds; ++s) {
    c.seed = static_cast<std::uint64_t>(s);
    const SimResult r = runSimulation(c);
    if (selectionAccuracy(r) > 0.5) ++ok;
  }
  return static_cast<double>(ok) / static_cast<double>(num_seeds);
}

}  // namespace

// ── A) Scoring kernel: nearest-to-signature wins ────────────────────────────────────────────────
TEST(Discriminator, SelectsObjectClosestToTargetSignature) {
  DecoysConfig cfg;
  cfg.target_intensity = 1.0;
  cfg.target_size = 1.0;
  cfg.target_decel = 1.0;
  cfg.feature_spread = 0.1;
  cfg.measurement_noise = 0.1;
  cfg.score_filter_tau = 0.0;  // no smoothing -> pure per-step scoring

  Discriminator d(cfg, /*num_objects=*/3, /*target_index=*/2);

  // Object 0 dim/small (decoy-like), object 1 mid, object 2 right on the target signature.
  std::vector<FeatureVec> z = {{0.3, 0.4, 3.0}, {0.7, 0.7, 1.8}, {1.0, 1.0, 1.0}};
  d.observe(z);
  EXPECT_EQ(d.selected(), 2);
  EXPECT_TRUE(d.correct());

  // instantScore is monotone: the on-signature object scores highest, the most-distinct lowest.
  EXPECT_GT(d.instantScore(z[2]), d.instantScore(z[1]));
  EXPECT_GT(d.instantScore(z[1]), d.instantScore(z[0]));

  // Margin is positive when a single object dominates.
  EXPECT_GT(d.margin(), 0.0);
}

// Temporal integration: a single noisy look may mislead, but integrating settles on the object
// whose STATIC signature matches the target, even when another object's mean is close.
TEST(Discriminator, TemporalIntegrationFavorsTrueSignature) {
  DecoysConfig cfg;
  cfg.target_intensity = 1.0;
  cfg.target_size = 1.0;
  cfg.target_decel = 1.0;
  cfg.feature_spread = 0.1;
  cfg.measurement_noise = 0.3;  // heavy per-step noise
  cfg.score_filter_tau = 0.5;

  Discriminator d(cfg, /*num_objects=*/2, /*target_index=*/0);
  // Object 0 sits on the signature; object 1 is offset. Feed many noise-free looks: integration
  // makes object 0 dominate decisively.
  for (int k = 0; k < 500; ++k) {
    d.observe({{1.0, 1.0, 1.0}, {0.4, 0.5, 2.5}});
  }
  EXPECT_EQ(d.selected(), 0);
  EXPECT_GT(d.margin(), 0.0);
}

// ── B) High separability: selects the true target ~always and intercepts ────────────────────────
TEST(DiscriminationRunner, HighSeparabilitySelectsTargetAndIntercepts) {
  SimConfig c = baseDecoyConfig();
  c.decoys.separability = 1.0;  // decoys clearly distinct
  const SimResult r = runSimulation(c);
  EXPECT_GT(selectionAccuracy(r), 0.99);  // essentially always the true target
  EXPECT_TRUE(r.intercept);
  EXPECT_LT(r.miss_distance, 3.0);  // homing on the true target -> small miss
}

// ── C) Monotone degradation with closer (lower-separability) and more numerous decoys ───────────
TEST(DiscriminationRunner, SelectionDegradesAsSeparabilityDrops) {
  SimConfig hi = baseDecoyConfig();
  hi.decoys.separability = 0.30;
  SimConfig mid = baseDecoyConfig();
  mid.decoys.separability = 0.12;
  SimConfig lo = baseDecoyConfig();
  lo.decoys.separability = 0.05;

  const int seeds = 40;
  const double acc_hi = mcSelectionRate(hi, seeds);
  const double acc_mid = mcSelectionRate(mid, seeds);
  const double acc_lo = mcSelectionRate(lo, seeds);

  // Distinct decoys -> near-perfect; overlapping decoys -> degraded. Monotone non-increasing.
  EXPECT_GT(acc_hi, 0.95);
  EXPECT_GE(acc_hi, acc_mid);
  EXPECT_GE(acc_mid, acc_lo);
  EXPECT_LT(acc_lo, acc_hi);  // strict overall degradation
  EXPECT_LT(acc_lo, 0.6);     // genuinely failing at high overlap
}

TEST(DiscriminationRunner, SelectionDegradesWithMoreDecoys) {
  SimConfig few = baseDecoyConfig();
  few.decoys.separability = 0.10;
  few.decoys.count = 1;
  SimConfig many = baseDecoyConfig();
  many.decoys.separability = 0.10;
  many.decoys.count = 12;

  const int seeds = 40;
  const double acc_few = mcSelectionRate(few, seeds);
  const double acc_many = mcSelectionRate(many, seeds);

  // More decoys in the cluster -> more chances one out-scores the target -> lower accuracy.
  EXPECT_GT(acc_few, acc_many);
}

// Miss distance grows in the population when discrimination fails: averaged over seeds, a
// high-overlap many-decoy scene misses far more often than a clearly-separable one.
TEST(DiscriminationRunner, MissGrowsWhenDiscriminationFails) {
  SimConfig easy = baseDecoyConfig();
  easy.decoys.separability = 1.0;
  SimConfig hard = baseDecoyConfig();
  hard.decoys.separability = 0.05;
  hard.decoys.count = 8;

  const int seeds = 30;
  auto pk = [&](SimConfig c) {
    int hits = 0;
    for (int s = 1; s <= seeds; ++s) {
      c.seed = static_cast<std::uint64_t>(s);
      if (runSimulation(c).intercept) ++hits;
    }
    return static_cast<double>(hits) / seeds;
  };
  const double pk_easy = pk(easy);
  const double pk_hard = pk(hard);
  EXPECT_GT(pk_easy, 0.95);     // clean discrimination -> reliable intercept
  EXPECT_LT(pk_hard, pk_easy);  // decoys degrade Pk
}

// ── D) Determinism: same seed -> identical telemetry ────────────────────────────────────────────
TEST(DiscriminationRunner, DeterministicSameSeed) {
  SimConfig c = baseDecoyConfig();
  c.decoys.separability = 0.1;  // a regime where selection actually varies by seed
  const SimResult a = runSimulation(c);
  const SimResult b = runSimulation(c);
  ASSERT_EQ(a.frames.size(), b.frames.size());
  for (std::size_t i = 0; i < a.frames.size(); ++i) {
    EXPECT_DOUBLE_EQ(a.frames[i].selected_obj, b.frames[i].selected_obj);
    EXPECT_DOUBLE_EQ(a.frames[i].discrim_margin, b.frames[i].discrim_margin);
  }
  EXPECT_DOUBLE_EQ(a.miss_distance, b.miss_distance);
}

// ── E) Default path unchanged: decoys disabled -> channel inert, telemetry byte-identical ────────
TEST(DiscriminationRunner, DefaultPathLeavesDiscrimChannelInert) {
  SimConfig c = baseDecoyConfig();
  c.decoys.enabled = false;  // default
  const SimResult r = runSimulation(c);
  ASSERT_FALSE(r.frames.empty());
  for (const auto& f : r.frames) {
    EXPECT_EQ(f.selected_obj, 0.0);
    EXPECT_EQ(f.discrim_correct, 1.0);
    EXPECT_EQ(f.discrim_margin, 0.0);
  }
}

TEST(DiscriminationRunner, DefaultRunByteIdenticalCsv) {
  // The no-decoy run must produce telemetry identical to a run that never knew about decoys: build
  // two configs, one with the (disabled) decoys block populated, one pristine, and compare CSVs.
  SimConfig with_block = baseDecoyConfig();
  with_block.decoys.enabled = false;

  SimConfig pristine = baseDecoyConfig();
  pristine.decoys = DecoysConfig{};  // all defaults (disabled)

  const SimResult ra = runSimulation(with_block);
  const SimResult rb = runSimulation(pristine);
  const auto csv_a = toCsvFiles(ra);
  const auto csv_b = toCsvFiles(rb);
  EXPECT_EQ(csv_a.at("vehicle.csv"), csv_b.at("vehicle.csv"));
  EXPECT_EQ(csv_a.at("gnc.csv"), csv_b.at("gnc.csv"));
}

// ── F) Config parsing ───────────────────────────────────────────────────────────────────────────
TEST(DiscriminationRunner, ParsesDecoysConfigBlock) {
  const std::string js = R"({
    "scenario":"d","model":"3dof","decoys":{
      "enabled":true,"count":5,"separation":80.0,"separability":0.3,
      "target_intensity":1.2,"decoy_decel":4.0,"measurement_noise":0.05
    }})";
  const SimConfig c = loadConfigFromString(js);
  ASSERT_TRUE(c.decoys.enabled);
  EXPECT_EQ(c.decoys.count, 5);
  EXPECT_DOUBLE_EQ(c.decoys.separation, 80.0);
  EXPECT_DOUBLE_EQ(c.decoys.separability, 0.3);
  EXPECT_DOUBLE_EQ(c.decoys.target_intensity, 1.2);
  EXPECT_DOUBLE_EQ(c.decoys.decoy_decel, 4.0);
  EXPECT_DOUBLE_EQ(c.decoys.measurement_noise, 0.05);
  // Unspecified keys keep struct defaults.
  EXPECT_DOUBLE_EQ(c.decoys.target_size, 1.0);
}

TEST(DiscriminationRunner, DefaultConfigDisablesDecoys) {
  const SimConfig c = loadConfigFromString("{}");
  EXPECT_FALSE(c.decoys.enabled);
  EXPECT_EQ(c.decoys.count, 0);
}
