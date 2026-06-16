// gnc-sim — per-step cost micro-benchmark + perf-budget regression gate (issue #53).
//
// Measures the wall-clock cost of the core per-step GNC loop (runSimulation) for a scenario,
// reported as nanoseconds PER INTEGRATION STEP so the number is independent of run length. The
// benchmark is offline developer tooling (native only, never WASM): it does file I/O to read the
// config and an optional budget file, which is why it lives in apps/ and not the pure core.
//
// Usage:
//   gncsim-bench --config configs/homing_3dof.json [--iters N] [--warmup W]
//   gncsim-bench --config configs/homing_3dof.json --budget configs/perf_budget.json --label
//   homing_3dof
//
// With --budget the benchmark loads the committed ns/step ceiling for --label and EXITS NON-ZERO if
// the measured cost exceeds it (budget * (1 + headroom)). The headroom is read from the budget file
// so CI flags real regressions, not measurement noise. Determinism is unaffected: the benchmark
// only TIMES runSimulation, it never changes its inputs or outputs.
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "gncsim/core/Config.hpp"
#include "gncsim/scenario/Runner.hpp"

namespace {

std::string readFile(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open: " + path);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::string arg(int argc, char** argv, const std::string& key, const std::string& fallback) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (key == argv[i]) return argv[i + 1];
  }
  return fallback;
}

// Tiny budget-file reader: pulls a per-label ns/step ceiling and the shared headroom fraction out
// of configs/perf_budget.json without dragging a JSON dependency into this offline tool. The file
// is a flat object: {"headroom": 0.5, "budgets_ns_per_step": {"homing_3dof": 1234.0, ...}}.
// Returns false if the label has no committed budget.
bool readBudget(const std::string& path, const std::string& label, double& budget_ns,
                double& headroom) {
  const std::string text = readFile(path);
  // headroom (default 0.5 if absent).
  headroom = 0.5;
  const auto hpos = text.find("\"headroom\"");
  if (hpos != std::string::npos) {
    const auto colon = text.find(':', hpos);
    if (colon != std::string::npos) headroom = std::stod(text.substr(colon + 1));
  }
  // budgets_ns_per_step."<label>": <number>
  const std::string key = "\"" + label + "\"";
  const auto bpos = text.find("budgets_ns_per_step");
  const auto kpos = text.find(key, bpos == std::string::npos ? 0 : bpos);
  if (kpos == std::string::npos) return false;
  const auto colon = text.find(':', kpos + key.size());
  if (colon == std::string::npos) return false;
  budget_ns = std::stod(text.substr(colon + 1));
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const std::string config_path = arg(argc, argv, "--config", "configs/homing_3dof.json");
    const std::string budget_path = arg(argc, argv, "--budget", "");
    const std::string label = arg(argc, argv, "--label", config_path);
    const int iters = std::stoi(arg(argc, argv, "--iters", "200"));
    const int warmup = std::stoi(arg(argc, argv, "--warmup", "20"));

    gncsim::SimConfig cfg = gncsim::loadConfigFromString(readFile(config_path));
    // Benchmark a single deterministic engagement: neutralize any Monte Carlo batch so we time the
    // per-step loop, not the dispersion driver.
    cfg.monte_carlo.num_cases = 0;

    // Determine the step count for the ns/step normalization (and as a divisor) from one run.
    const gncsim::SimResult probe = gncsim::runSimulation(cfg);
    const std::size_t steps = probe.frames.size();
    if (steps == 0) throw std::runtime_error("scenario produced no frames");

    // Warmup (page-in, branch-predictor/cache warm) — not timed.
    volatile double sink = 0.0;
    for (int i = 0; i < warmup; ++i) {
      const gncsim::SimResult r = gncsim::runSimulation(cfg);
      sink += r.miss_distance;
    }

    // Timed loop: best (minimum) per-run time over `iters` is the most stable estimator of the
    // underlying cost (it filters scheduler/OS jitter, which only ever adds time).
    using clock = std::chrono::steady_clock;
    std::int64_t best_ns = INT64_MAX;
    double total_ns = 0.0;
    for (int i = 0; i < iters; ++i) {
      const auto t0 = clock::now();
      const gncsim::SimResult r = gncsim::runSimulation(cfg);
      const auto t1 = clock::now();
      sink += r.miss_distance;
      const std::int64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
      if (ns < best_ns) best_ns = ns;
      total_ns += static_cast<double>(ns);
    }
    (void)sink;

    const double best_ns_per_step = static_cast<double>(best_ns) / static_cast<double>(steps);
    const double mean_ns_per_step = (total_ns / iters) / static_cast<double>(steps);

    std::cout.precision(2);
    std::cout << std::fixed;
    std::cout << "scenario   : " << label << "\n";
    std::cout << "steps/run  : " << steps << "\n";
    std::cout << "iters      : " << iters << " (warmup " << warmup << ")\n";
    std::cout << "ns/step    : " << best_ns_per_step << " (best)   " << mean_ns_per_step
              << " (mean)\n";
    std::cout << "ns/run     : " << best_ns << " (best)\n";

    if (!budget_path.empty()) {
      double budget_ns_per_step = 0.0;
      double headroom = 0.5;
      if (!readBudget(budget_path, label, budget_ns_per_step, headroom)) {
        std::cerr << "FAIL: no committed budget for label '" << label << "' in " << budget_path
                  << "\n";
        return 3;
      }
      const double ceiling = budget_ns_per_step * (1.0 + headroom);
      std::cout << "budget     : " << budget_ns_per_step << " ns/step  (ceiling " << ceiling
                << " ns/step, +" << (headroom * 100.0) << "% headroom)\n";
      // Gate on the BEST (minimum) timing: it is the noise-floor estimate, so exceeding the ceiling
      // there is a real, reproducible regression rather than transient jitter.
      if (best_ns_per_step > ceiling) {
        std::cerr << "FAIL: perf regression — " << best_ns_per_step << " ns/step exceeds ceiling "
                  << ceiling << " ns/step\n";
        return 1;
      }
      std::cout << "PERF OK: within budget.\n";
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "bench error: " << e.what() << "\n";
    return 2;
  }
}
