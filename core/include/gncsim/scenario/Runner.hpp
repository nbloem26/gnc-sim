// gnc-sim — top-level simulation entry. Pure (no file I/O) so it runs identically native & WASM.
// The CLI and the WASM embind wrapper both call runSimulation(). Phase 2 (integration) owns the
// real implementation in core/src/scenario/Runner.cpp.
#pragma once

#include <cstdint>
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
std::vector<MonteCarloCase> runMonteCarlo(const SimConfig& cfg);

}  // namespace gncsim
