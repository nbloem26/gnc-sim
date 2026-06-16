// gnc-sim — Monte Carlo driver. Disperses the initial conditions per the monte_carlo sigmas and
// runs N independent cases, each with its own deterministic seed derived from the master seed.
// This is where the characterized sensor noise turns into a miss-distance / CEP distribution.
//
// Determinism + parallelism (issue #43): the batch is split into a strictly-serial DISPERSION phase
// (planMonteCarlo: draws every case's seed + IC offsets from one master RNG stream, in case order)
// and an embarrassingly-parallel SIMULATION phase (one independent runSimulation per case). Because
// the master RNG sequence is consumed entirely in phase 1 — before any simulation runs — the result
// of a case never depends on thread scheduling. The parallel driver is therefore BIT-IDENTICAL to
// the historical serial loop for the same seed + N. The core stays pure: no file I/O here (the
// optional checkpoint sink is a caller-supplied callback; the actual disk writes live in the
// CLI/SDK).
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <mutex>
#include <thread>
#include <vector>

#include "gncsim/core/Rng.hpp"
#include "gncsim/scenario/Runner.hpp"

namespace gncsim {

namespace {

// Run a single planned case. Pure: depends only on the dispersed config, so it is safe to call from
// any worker thread (no shared mutable state).
MonteCarloCase runPlannedCase(int index, const SimConfig& case_cfg) {
  const SimResult res = runSimulation(case_cfg);
  MonteCarloCase mc;
  mc.index = index;
  mc.seed = case_cfg.seed;
  mc.miss_distance = res.miss_distance;
  mc.intercept_time = res.intercept_time;
  mc.intercept = res.intercept;
  return mc;
}

}  // namespace

MonteCarloPlan planMonteCarlo(const SimConfig& cfg) {
  MonteCarloPlan plan;
  const int n = cfg.monte_carlo.num_cases;
  if (n <= 0) return plan;
  plan.case_configs.reserve(static_cast<std::size_t>(n));
  plan.seeds.reserve(static_cast<std::size_t>(n));

  // Master RNG produces the dispersions + per-case seeds, so the whole batch is reproducible. The
  // draw order below MUST match the legacy serial loop exactly (seed, launch speed, elevation, 3x
  // target position, optional weave phase) — that is what keeps results bit-identical.
  Rng master(cfg.seed);

  for (int i = 0; i < n; ++i) {
    SimConfig c = cfg;
    c.monte_carlo.num_cases = 0;  // each case is a single run
    c.seed = master.engine()();   // independent per-case stream for the sensor noise

    c.vehicle.launch_speed += master.gaussian(0.0, cfg.monte_carlo.launch_speed_sigma);
    c.vehicle.launch_elevation_deg +=
        master.gaussian(0.0, cfg.monte_carlo.launch_elevation_sigma_deg);
    c.target.pos0.x += master.gaussian(0.0, cfg.monte_carlo.target_pos_sigma);
    c.target.pos0.y += master.gaussian(0.0, cfg.monte_carlo.target_pos_sigma);
    c.target.pos0.z += master.gaussian(0.0, cfg.monte_carlo.target_pos_sigma);

    // A weaving target is caught at a random point in its maneuver cycle each engagement — this
    // (with terminal seeker glint and finite interceptor agility) is what spreads the miss.
    if (c.target.maneuver == "weave") c.target.maneuver_phase_deg = master.uniform(0.0, 360.0);

    plan.seeds.push_back(c.seed);
    plan.case_configs.push_back(std::move(c));
  }
  return plan;
}

std::vector<MonteCarloCase> runMonteCarlo(const SimConfig& cfg) {
  const MonteCarloPlan plan = planMonteCarlo(cfg);
  std::vector<MonteCarloCase> out;
  out.reserve(plan.case_configs.size());
  for (std::size_t i = 0; i < plan.case_configs.size(); ++i) {
    out.push_back(runPlannedCase(static_cast<int>(i), plan.case_configs[i]));
  }
  return out;
}

std::vector<MonteCarloCase> runMonteCarloParallel(const SimConfig& cfg,
                                                  const MonteCarloRunOptions& opts) {
  const MonteCarloPlan plan = planMonteCarlo(cfg);
  const std::size_t n = plan.case_configs.size();

  // Output slots indexed by case → order-independent aggregation regardless of completion order.
  std::vector<MonteCarloCase> out(n);
  std::vector<char> done(n, 0);  // 1 = slot filled (either resumed or freshly computed)

  // Phase 0: drop in any checkpointed cases so we skip them. Seeds in `completed_results` are kept
  // as-is, but they MUST equal the replayed plan seed for the same index (the dispersion phase is
  // deterministic), which the restart test asserts.
  for (std::size_t k = 0; k < opts.completed.size() && k < opts.completed_results.size(); ++k) {
    const int idx = opts.completed[k];
    if (idx >= 0 && static_cast<std::size_t>(idx) < n) {
      out[static_cast<std::size_t>(idx)] = opts.completed_results[k];
      done[static_cast<std::size_t>(idx)] = 1;
    }
  }

  // Build the worklist of remaining (not-yet-done) case indices.
  std::vector<std::size_t> todo;
  todo.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    if (!done[i]) todo.push_back(i);
  }

  // on_case_done is invoked from possibly many threads; serialize it so the callback need not be
  // thread-safe and so checkpoint appends are well-ordered per call.
  std::mutex sink_mutex;
  auto fire_sink = [&](const MonteCarloCase& mc) {
    if (opts.on_case_done) {
      std::lock_guard<std::mutex> lock(sink_mutex);
      opts.on_case_done(mc);
    }
  };

  const int requested = opts.num_workers;
  const std::size_t workers =
      (requested <= 1) ? 1 : std::min(static_cast<std::size_t>(requested), todo.size());

  if (workers <= 1 || todo.size() <= 1) {
    // Serial path: identical results, no threads spun up.
    for (const std::size_t i : todo) {
      out[i] = runPlannedCase(static_cast<int>(i), plan.case_configs[i]);
      fire_sink(out[i]);
    }
    return out;
  }

  // Hand-rolled thread pool: a shared atomic cursor over `todo` (work stealing by index). Each
  // worker writes only its own case's output slot, so there is no data race on `out` (distinct
  // indices) — the only shared mutable state is the atomic cursor and the sink mutex.
  std::atomic<std::size_t> next{0};
  auto worker = [&]() {
    for (;;) {
      const std::size_t k = next.fetch_add(1, std::memory_order_relaxed);
      if (k >= todo.size()) break;
      const std::size_t i = todo[k];
      MonteCarloCase mc = runPlannedCase(static_cast<int>(i), plan.case_configs[i]);
      out[i] = mc;  // distinct i per loop iteration across threads → race-free
      fire_sink(out[i]);
    }
  };

  std::vector<std::thread> pool;
  pool.reserve(workers);
  for (std::size_t w = 0; w < workers; ++w) pool.emplace_back(worker);
  for (std::thread& t : pool) t.join();

  return out;
}

}  // namespace gncsim
