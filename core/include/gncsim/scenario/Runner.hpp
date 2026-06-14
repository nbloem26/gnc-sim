// gnc-sim — top-level simulation entry. Pure (no file I/O) so it runs identically native & WASM.
// The CLI and the WASM embind wrapper both call runSimulation(). Phase 2 (integration) owns the
// real implementation in core/src/scenario/Runner.cpp.
#pragma once

#include <cstdint>
#include <functional>
#include <vector>

#include "gncsim/core/Config.hpp"
#include "gncsim/core/Types.hpp"

namespace gncsim {

// Run one simulation to completion, returning the full in-memory telemetry + outcome.
SimResult runSimulation(const SimConfig& cfg);

// One Monte Carlo case outcome (no telemetry — just the summary row).
struct MonteCarloCase {
  int index = 0;
  std::uint64_t seed = 0;
  double miss_distance = 0.0;
  double intercept_time = 0.0;
  bool intercept = false;
};

// Run cfg.monte_carlo.num_cases dispersed cases (launch speed/elevation + target position sigmas,
// independent per-case RNG seed). Deterministic given cfg.seed. Returns one row per case.
//
// The batch is split in two phases so it can be parallelized without changing any numbers:
//   1) the DISPERSION phase draws each case's seed + initial-condition offsets from the single
//      master RNG stream (cfg.seed) — strictly serial, so the RNG sequence is identical to the
//      historical serial loop and independent of how cases are later scheduled;
//   2) the SIMULATION phase runs each independent runSimulation(), which is what
//   runMonteCarloParallel
//      distributes across a thread pool.
// Therefore parallel results are BIT-IDENTICAL to serial for the same seed + N.
std::vector<MonteCarloCase> runMonteCarlo(const SimConfig& cfg);

// The fully-resolved, per-case dispersed config (phase 1 output). Pure data — computing the whole
// vector touches the master RNG once, deterministically, in case order.
struct MonteCarloPlan {
  std::vector<SimConfig> case_configs;  // one dispersed SimConfig per case, index == position
  std::vector<std::uint64_t> seeds;     // per-case seed (== case_configs[i].seed), for provenance
};

// Phase 1 only: derive every case's dispersed config from the master RNG. Deterministic; no
// simulation is run. runMonteCarlo / runMonteCarloParallel both build on this so they share one
// RNG-draw order.
MonteCarloPlan planMonteCarlo(const SimConfig& cfg);

// Options for the parallel / resumable batch driver.
struct MonteCarloRunOptions {
  // Worker threads. <= 1 runs serially (no pool spun up). Capped at the case count.
  int num_workers = 1;

  // Checkpoint/restart. Cases whose index is in `completed` are NOT re-run; their result is taken
  // from `completed_results` instead. This lets a campaign resume after an interruption and still
  // reproduce the identical final aggregate (the dispersion phase is replayed in full, so skipped
  // cases keep the exact seeds/ICs they had originally). `completed` and `completed_results` are
  // index-aligned; an empty `completed` means a fresh run.
  std::vector<int> completed;
  std::vector<MonteCarloCase> completed_results;

  // Optional sink invoked once per freshly-computed case (NOT for resumed ones), as soon as that
  // case finishes, so a driver can append it to an on-disk checkpoint. Called from worker threads:
  // the implementation serializes these calls under an internal mutex, so the callback need not be
  // thread-safe itself. Pure core stays I/O-free — the callback (and any file writing) lives in the
  // CLI / SDK, never inside runSimulation.
  std::function<void(const MonteCarloCase&)> on_case_done;
};

// Parallel + resumable Monte Carlo. Same dispersion (phase 1) as runMonteCarlo, then runs the
// not-yet-completed cases across `opts.num_workers` threads and merges with any resumed cases. The
// returned vector is ordered by case index and is bit-identical to runMonteCarlo(cfg) for the same
// cfg — regardless of worker count or which cases were resumed from a checkpoint.
std::vector<MonteCarloCase> runMonteCarloParallel(const SimConfig& cfg,
                                                  const MonteCarloRunOptions& opts);

}  // namespace gncsim
