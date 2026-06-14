// gnc-sim — parallel Monte Carlo determinism (issue #43).
//
// The contract: runMonteCarloParallel must be BIT-IDENTICAL to the serial runMonteCarlo for the
// same config + seed, for ANY worker count, and a checkpoint/restart must reproduce the identical
// aggregate. These tests are the evidence for that invariant (and run clean under ASan/UBSan/TSan).
#include <gtest/gtest.h>

#include <vector>

#include "gncsim/core/Config.hpp"
#include "gncsim/scenario/Runner.hpp"

namespace {

// A dispersed homing config whose Monte Carlo block actually spreads the miss (weave target +
// nonzero sigmas), so the cases are genuinely distinct — a stronger determinism test than identical
// cases would be.
gncsim::SimConfig makeBatchConfig(int num_cases) {
  gncsim::SimConfig cfg;
  cfg.scenario = "homing";
  cfg.model = "3dof";
  cfg.seed = 20240614;
  cfg.target.maneuver = "weave";
  cfg.monte_carlo.num_cases = num_cases;
  cfg.monte_carlo.launch_speed_sigma = 12.0;
  cfg.monte_carlo.launch_elevation_sigma_deg = 1.5;
  cfg.monte_carlo.target_pos_sigma = 60.0;
  return cfg;
}

void expectIdentical(const std::vector<gncsim::MonteCarloCase>& a,
                     const std::vector<gncsim::MonteCarloCase>& b) {
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].index, b[i].index) << "case " << i;
    EXPECT_EQ(a[i].seed, b[i].seed) << "case " << i;
    // Bit-identical: exact equality, not approximate. Use the raw double compare.
    EXPECT_EQ(a[i].miss_distance, b[i].miss_distance) << "case " << i;
    EXPECT_EQ(a[i].intercept_time, b[i].intercept_time) << "case " << i;
    EXPECT_EQ(a[i].intercept, b[i].intercept) << "case " << i;
  }
}

}  // namespace

// Parallel(workers=N) is element-wise identical to the serial batch for the same seed + cases.
TEST(ParallelMonteCarlo, BitIdenticalToSerial) {
  const gncsim::SimConfig cfg = makeBatchConfig(64);
  const std::vector<gncsim::MonteCarloCase> serial = gncsim::runMonteCarlo(cfg);
  ASSERT_EQ(serial.size(), 64u);

  for (int workers : {2, 3, 4, 8, 16}) {
    gncsim::MonteCarloRunOptions opts;
    opts.num_workers = workers;
    const std::vector<gncsim::MonteCarloCase> par = gncsim::runMonteCarloParallel(cfg, opts);
    SCOPED_TRACE("workers=" + std::to_string(workers));
    expectIdentical(serial, par);
  }
}

// runMonteCarloParallel with workers<=1 equals the legacy serial path exactly.
TEST(ParallelMonteCarlo, SerialDriverMatchesLegacy) {
  const gncsim::SimConfig cfg = makeBatchConfig(32);
  gncsim::MonteCarloRunOptions opts;
  opts.num_workers = 1;
  expectIdentical(gncsim::runMonteCarlo(cfg), gncsim::runMonteCarloParallel(cfg, opts));
}

// Thread count does not change results: every worker count produces the identical vector.
TEST(ParallelMonteCarlo, ThreadCountInvariant) {
  const gncsim::SimConfig cfg = makeBatchConfig(50);
  gncsim::MonteCarloRunOptions o1;
  o1.num_workers = 1;
  const std::vector<gncsim::MonteCarloCase> ref = gncsim::runMonteCarloParallel(cfg, o1);
  for (int workers : {2, 5, 7, 12}) {
    gncsim::MonteCarloRunOptions o;
    o.num_workers = workers;
    SCOPED_TRACE("workers=" + std::to_string(workers));
    expectIdentical(ref, gncsim::runMonteCarloParallel(cfg, o));
  }
}

// Checkpoint -> restart reproduces the identical aggregate: run the first half, "resume" with those
// cases marked complete, and confirm the merged result equals a full fresh run.
TEST(ParallelMonteCarlo, CheckpointRestartReproducesAggregate) {
  const gncsim::SimConfig cfg = makeBatchConfig(40);
  const std::vector<gncsim::MonteCarloCase> full = gncsim::runMonteCarlo(cfg);

  // Phase 1: run a partial batch (pretend it was interrupted) and collect the first 17 cases via
  // the on_case_done sink — exactly how the CLI builds its checkpoint file.
  std::vector<gncsim::MonteCarloCase> collected;
  gncsim::MonteCarloRunOptions phase1;
  phase1.num_workers = 4;
  phase1.on_case_done = [&](const gncsim::MonteCarloCase& c) {
    if (c.index < 17) collected.push_back(c);
  };
  gncsim::runMonteCarloParallel(cfg, phase1);
  ASSERT_EQ(collected.size(), 17u);

  // Phase 2: resume with those 17 cases pre-completed; only the remaining 23 should be
  // (re)computed.
  int recomputed = 0;
  gncsim::MonteCarloRunOptions phase2;
  phase2.num_workers = 4;
  for (const auto& c : collected) {
    phase2.completed.push_back(c.index);
    phase2.completed_results.push_back(c);
  }
  phase2.on_case_done = [&](const gncsim::MonteCarloCase&) { ++recomputed; };
  const std::vector<gncsim::MonteCarloCase> resumed = gncsim::runMonteCarloParallel(cfg, phase2);

  EXPECT_EQ(recomputed, 23) << "resumed cases must not be re-run";
  expectIdentical(full, resumed);

  // And the resumed seeds for the skipped cases must match the replayed plan (proves the dispersion
  // phase is deterministic across the interruption).
  for (const auto& c : collected) {
    EXPECT_EQ(resumed[static_cast<std::size_t>(c.index)].seed,
              full[static_cast<std::size_t>(c.index)].seed);
  }
}

// Empty batch is well-defined (no cases, no threads).
TEST(ParallelMonteCarlo, EmptyBatch) {
  gncsim::SimConfig cfg = makeBatchConfig(0);
  gncsim::MonteCarloRunOptions opts;
  opts.num_workers = 8;
  EXPECT_TRUE(gncsim::runMonteCarloParallel(cfg, opts).empty());
}
