// gnc-sim — Python bindings (pybind11). Exposes the pure C++ core to Python so analysts can script
// engagements directly, with results as numpy arrays — no CSV/JSON round-trip (see
// docs/TARGET_ARCHITECTURE.md §4). This is a CLIENT of the core, exactly like apps/cli and
// apps/wasm; it adds no numerics and reuses the SAME config parser (loadConfigFromString) the CLI
// and WASM entry use, so run(cfg) matches the CLI/golden values bit-for-bit for the same config.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstddef>
#include <string>
#include <vector>

#include "gncsim/core/Config.hpp"
#include "gncsim/scenario/Runner.hpp"

namespace py = pybind11;

namespace {

// Build a 1-D numpy float64 array from a per-frame accessor. Mirrors the columnar JSON series in
// core/src/.../Serialize.cpp (toJsonString) so the channel names match the web/CSV contract.
template <typename F>
py::array_t<double> column(const gncsim::SimResult& r, F&& accessor) {
  const std::size_t n = r.frames.size();
  py::array_t<double> arr(static_cast<py::ssize_t>(n));
  double* out = arr.mutable_data();
  for (std::size_t i = 0; i < n; ++i) out[i] = accessor(r.frames[i]);
  return arr;
}

// Assemble the result dict: scalar metadata + a "series" dict of numpy arrays. The series keys are
// IDENTICAL to the columnar JSON the web app consumes (toJsonString), so Python analysis and the
// web share one channel vocabulary.
py::dict resultToDict(const gncsim::SimResult& r) {
  py::dict series;
  series["t"] = column(r, [](const gncsim::Frame& f) { return f.t; });
  series["veh_x"] = column(r, [](const gncsim::Frame& f) { return f.veh_pos.x; });
  series["veh_y"] = column(r, [](const gncsim::Frame& f) { return f.veh_pos.y; });
  series["veh_z"] = column(r, [](const gncsim::Frame& f) { return f.veh_pos.z; });
  series["veh_vx"] = column(r, [](const gncsim::Frame& f) { return f.veh_vel.x; });
  series["veh_vy"] = column(r, [](const gncsim::Frame& f) { return f.veh_vel.y; });
  series["veh_vz"] = column(r, [](const gncsim::Frame& f) { return f.veh_vel.z; });
  series["roll"] = column(r, [](const gncsim::Frame& f) { return f.veh_att.toEuler().x; });
  series["pitch"] = column(r, [](const gncsim::Frame& f) { return f.veh_att.toEuler().y; });
  series["yaw"] = column(r, [](const gncsim::Frame& f) { return f.veh_att.toEuler().z; });
  series["mass"] = column(r, [](const gncsim::Frame& f) { return f.mass; });
  series["mach"] = column(r, [](const gncsim::Frame& f) { return f.mach; });
  series["thrust"] = column(r, [](const gncsim::Frame& f) { return f.thrust; });
  series["tgt_x"] = column(r, [](const gncsim::Frame& f) { return f.tgt_pos.x; });
  series["tgt_y"] = column(r, [](const gncsim::Frame& f) { return f.tgt_pos.y; });
  series["tgt_z"] = column(r, [](const gncsim::Frame& f) { return f.tgt_pos.z; });
  series["tgt_vx"] = column(r, [](const gncsim::Frame& f) { return f.tgt_vel.x; });
  series["tgt_vy"] = column(r, [](const gncsim::Frame& f) { return f.tgt_vel.y; });
  series["tgt_vz"] = column(r, [](const gncsim::Frame& f) { return f.tgt_vel.z; });
  series["accel_cmd_x"] = column(r, [](const gncsim::Frame& f) { return f.accel_cmd.x; });
  series["accel_cmd_y"] = column(r, [](const gncsim::Frame& f) { return f.accel_cmd.y; });
  series["accel_cmd_z"] = column(r, [](const gncsim::Frame& f) { return f.accel_cmd.z; });
  series["los_angle"] = column(r, [](const gncsim::Frame& f) { return f.los_angle; });
  series["los_rate"] = column(r, [](const gncsim::Frame& f) { return f.los_rate; });
  series["v_closing"] = column(r, [](const gncsim::Frame& f) { return f.v_closing; });
  series["range"] = column(r, [](const gncsim::Frame& f) { return f.range; });
  series["nav_x"] = column(r, [](const gncsim::Frame& f) { return f.nav_pos_est.x; });
  series["nav_y"] = column(r, [](const gncsim::Frame& f) { return f.nav_pos_est.y; });
  series["nav_z"] = column(r, [](const gncsim::Frame& f) { return f.nav_pos_est.z; });
  series["nav_nis"] = column(r, [](const gncsim::Frame& f) { return f.nav_nis; });
  series["track_x"] = column(r, [](const gncsim::Frame& f) { return f.track_pos_est.x; });
  series["track_y"] = column(r, [](const gncsim::Frame& f) { return f.track_pos_est.y; });
  series["track_z"] = column(r, [](const gncsim::Frame& f) { return f.track_pos_est.z; });
  series["track_nis"] = column(r, [](const gncsim::Frame& f) { return f.track_nis; });
  series["selected_obj"] = column(r, [](const gncsim::Frame& f) { return f.selected_obj; });
  series["discrim_correct"] = column(r, [](const gncsim::Frame& f) { return f.discrim_correct; });
  series["discrim_margin"] = column(r, [](const gncsim::Frame& f) { return f.discrim_margin; });
  series["imu_accel_true_x"] = column(r, [](const gncsim::Frame& f) { return f.imu_accel_true.x; });
  series["imu_accel_meas_x"] = column(r, [](const gncsim::Frame& f) { return f.imu_accel_meas.x; });
  series["imu_gyro_true_x"] = column(r, [](const gncsim::Frame& f) { return f.imu_gyro_true.x; });
  series["imu_gyro_meas_x"] = column(r, [](const gncsim::Frame& f) { return f.imu_gyro_meas.x; });
  series["seeker_los_true"] = column(r, [](const gncsim::Frame& f) { return f.seeker_los_true; });
  series["seeker_los_meas"] = column(r, [](const gncsim::Frame& f) { return f.seeker_los_meas; });

  py::dict origin;
  origin["lat0_deg"] = r.origin.lat0_deg;
  origin["lon0_deg"] = r.origin.lon0_deg;
  origin["alt0_m"] = r.origin.alt0_m;

  py::dict out;
  out["scenario"] = r.scenario;
  out["model"] = r.model;
  out["seed"] = r.seed;
  out["dt"] = r.dt;
  out["t_end"] = r.t_end;
  out["intercept"] = r.intercept;
  out["miss_distance"] = r.miss_distance;
  out["intercept_time"] = r.intercept_time;
  out["launch_time"] = r.launch_time;
  out["git_sha"] = r.git_sha;
  out["origin"] = origin;
  out["series"] = series;
  return out;
}

// Run one engagement. `config_json` is a JSON document string (the Python wrapper serializes
// dicts); it goes straight through the SAME parser the CLI/WASM use, so results match bit-for-bit.
py::dict run(const std::string& config_json) {
  const gncsim::SimConfig cfg = gncsim::loadConfigFromString(config_json);
  const gncsim::SimResult result = gncsim::runSimulation(cfg);
  return resultToDict(result);
}

// Run a dispersed Monte Carlo batch (IC dispersion lives in the C++ core — MonteCarlo.cpp). `n`
// overrides cfg.monte_carlo.num_cases when > 0. `workers` distributes the cases across a C++ thread
// pool (issue #43); the result is BIT-IDENTICAL to the serial batch for the same seed + N because
// every case's RNG stream is drawn up-front in case order, independent of scheduling. `workers <=
// 1` runs serially. Returns columnar numpy arrays (one row per case): index, seed, miss_distance,
// intercept_time, intercept.
py::dict monteCarlo(const std::string& config_json, int n, int workers) {
  gncsim::SimConfig cfg = gncsim::loadConfigFromString(config_json);
  if (n > 0) cfg.monte_carlo.num_cases = n;

  std::vector<gncsim::MonteCarloCase> cases;
  {
    // Release the GIL: the batch is pure C++ compute (the cases never touch Python), so this lets
    // the thread pool actually use multiple cores instead of serializing on the interpreter lock.
    py::gil_scoped_release release;
    gncsim::MonteCarloRunOptions opts;
    opts.num_workers = workers;
    cases = gncsim::runMonteCarloParallel(cfg, opts);
  }
  const std::size_t m = cases.size();

  py::array_t<long long> index(static_cast<py::ssize_t>(m));
  py::array_t<unsigned long long> seed(static_cast<py::ssize_t>(m));
  py::array_t<double> miss(static_cast<py::ssize_t>(m));
  py::array_t<double> tca(static_cast<py::ssize_t>(m));
  py::array_t<bool> hit(static_cast<py::ssize_t>(m));
  long long* idx_p = index.mutable_data();
  unsigned long long* seed_p = seed.mutable_data();
  double* miss_p = miss.mutable_data();
  double* tca_p = tca.mutable_data();
  bool* hit_p = hit.mutable_data();
  int hits = 0;
  for (std::size_t i = 0; i < m; ++i) {
    idx_p[i] = cases[i].index;
    seed_p[i] = cases[i].seed;
    miss_p[i] = cases[i].miss_distance;
    tca_p[i] = cases[i].intercept_time;
    hit_p[i] = cases[i].intercept;
    hits += cases[i].intercept ? 1 : 0;
  }

  py::dict out;
  out["num_cases"] = m;
  out["intercepts"] = hits;
  out["p_kill"] = m ? static_cast<double>(hits) / static_cast<double>(m) : 0.0;
  out["index"] = index;
  out["seed"] = seed;
  out["miss_distance"] = miss;
  out["intercept_time"] = tca;
  out["intercept"] = hit;
  return out;
}

}  // namespace

PYBIND11_MODULE(_gncsim, m) {
  m.doc() =
      "gnc-sim core bindings (pybind11). Run the deterministic C++ engine from Python; results as "
      "numpy arrays. Prefer the `gncsim` package wrapper, which accepts dict or JSON-string "
      "configs.";
  m.def(
      "run", &run, py::arg("config_json"),
      "Run one engagement from a JSON config string; returns a result dict with a numpy 'series'.");
  m.def("monte_carlo", &monteCarlo, py::arg("config_json"), py::arg("n") = 0,
        py::arg("workers") = 1,
        "Run a dispersed Monte Carlo batch across `workers` threads; bit-identical to the serial "
        "batch for the same seed + N. Returns columnar numpy arrays (one row per case).");
}
