// gnc-sim — Monte Carlo driver. Disperses the initial conditions per the monte_carlo sigmas and
// runs N independent cases, each with its own deterministic seed derived from the master seed.
// This is where the characterized sensor noise turns into a miss-distance / CEP distribution.
#include "gncsim/core/Rng.hpp"
#include "gncsim/scenario/Runner.hpp"

namespace gncsim {

std::vector<MonteCarloCase> runMonteCarlo(const SimConfig& cfg) {
  std::vector<MonteCarloCase> out;
  const int n = cfg.monte_carlo.num_cases;
  if (n <= 0) return out;
  out.reserve(n);

  // Master RNG produces the dispersions + per-case seeds, so the whole batch is reproducible.
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

    const SimResult res = runSimulation(c);

    MonteCarloCase mc;
    mc.index = i;
    mc.seed = c.seed;
    mc.miss_distance = res.miss_distance;
    mc.intercept_time = res.intercept_time;
    mc.intercept = res.intercept;
    out.push_back(mc);
  }
  return out;
}

}  // namespace gncsim
