// gnc-sim native CLI: run a scenario and write CSV telemetry + manifest.json to an output folder.
// Usage: gncsim --config configs/homing_3dof.json --out runs/run_001 [--seed N]
//
// This is the OFFLINE / Monte-Carlo entry point. It wraps the pure core (runSimulation +
// serializers); the only file I/O in the whole project lives here and in the Monte Carlo batch.
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "gncsim/core/Config.hpp"
#include "gncsim/core/Serialize.hpp"
#include "gncsim/scenario/Runner.hpp"

namespace {

std::string readFile(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open config: " + path);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

void writeFile(const std::string& path, const std::string& content) {
  std::ofstream out(path);
  if (!out) throw std::runtime_error("cannot write: " + path);
  out << content;
}

// Minimal `mkdir -p` without <filesystem> coupling concerns on the CLI side. This is offline
// developer tooling (not the pure core, never WASM), so shelling out to `mkdir` is acceptable here.
// NOLINTNEXTLINE(bugprone-command-processor,concurrency-mt-unsafe): trusted CLI-side path only.
void ensureDir(const std::string& dir) {
  const std::string cmd = "mkdir -p '" + dir + "'";
  // NOLINTNEXTLINE(bugprone-command-processor,concurrency-mt-unsafe): offline CLI tooling.
  if (std::system(cmd.c_str()) != 0) throw std::runtime_error("cannot create dir: " + dir);
}

std::string arg(int argc, char** argv, const std::string& key, const std::string& fallback) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (key == argv[i]) return argv[i + 1];
  }
  return fallback;
}

// One row of a summary.csv / checkpoint: "case,seed,miss_distance,intercept_time,intercept".
std::string formatCase(const gncsim::MonteCarloCase& c) {
  std::ostringstream row;
  row.precision(9);
  row << c.index << ',' << c.seed << ',' << c.miss_distance << ',' << c.intercept_time << ','
      << (c.intercept ? 1 : 0);
  return row.str();
}

// Parse a previously-written summary.csv back into completed cases so a campaign can RESUME: any
// case already present is skipped on restart and its stored result is reused (the dispersion phase
// is replayed in full, so seeds line up). Tolerates a missing file (fresh run -> empty).
std::vector<gncsim::MonteCarloCase> loadCheckpoint(const std::string& path) {
  std::vector<gncsim::MonteCarloCase> done;
  std::ifstream in(path);
  if (!in) return done;
  std::string line;
  std::getline(in, line);  // header
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::istringstream ls(line);
    std::string tok;
    gncsim::MonteCarloCase c;
    if (!std::getline(ls, tok, ',')) continue;
    c.index = std::stoi(tok);
    if (!std::getline(ls, tok, ',')) continue;
    c.seed = static_cast<std::uint64_t>(std::stoull(tok));
    if (!std::getline(ls, tok, ',')) continue;
    c.miss_distance = std::stod(tok);
    if (!std::getline(ls, tok, ',')) continue;
    c.intercept_time = std::stod(tok);
    if (!std::getline(ls, tok, ',')) continue;
    c.intercept = (std::stoi(tok) != 0);
    done.push_back(c);
  }
  return done;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const std::string config_path = arg(argc, argv, "--config", "");
    const std::string out_dir = arg(argc, argv, "--out", "runs/run");
    const std::string seed_override = arg(argc, argv, "--seed", "");
    const std::string json_path = arg(argc, argv, "--json", "");  // also emit WASM-format JSON
    const std::string workers_str = arg(argc, argv, "--workers", "1");  // MC thread-pool size
    const bool checkpoint =
        !arg(argc, argv, "--checkpoint", "").empty() || arg(argc, argv, "--resume", "") == "1";
    const int num_workers = std::stoi(workers_str);

    if (config_path.empty()) {
      std::cerr << "usage: gncsim --config <file.json> --out <dir> [--seed N] "
                   "[--workers K] [--checkpoint 1]\n";
      return 2;
    }

    const std::string config_text = readFile(config_path);
    gncsim::SimConfig cfg = gncsim::loadConfigFromString(config_text);
    if (!seed_override.empty()) {
      cfg.seed = static_cast<std::uint64_t>(std::stoull(seed_override));
    }

    ensureDir(out_dir);

    // Monte Carlo batch: write summary.csv (one row/case) + full telemetry for case 0. With
    // --workers K the cases run on a K-thread pool; the result is bit-identical to the serial
    // batch. With --checkpoint the summary.csv doubles as a restart log: completed cases are loaded
    // and skipped, and each freshly-finished case is appended immediately so an interrupted long
    // campaign can resume and still reproduce the identical aggregate.
    if (cfg.monte_carlo.num_cases > 0) {
      const std::string summary_path = out_dir + "/summary.csv";
      const std::string header = "case,seed,miss_distance,intercept_time,intercept\n";

      gncsim::MonteCarloRunOptions opts;
      opts.num_workers = num_workers;

      std::ofstream append_stream;
      std::mutex append_mutex;
      if (checkpoint) {
        const auto resumed = loadCheckpoint(summary_path);
        for (const auto& c : resumed) {
          opts.completed.push_back(c.index);
          opts.completed_results.push_back(c);
        }
        // Re-open in append mode (write the header first if starting fresh) and stream each new
        // case to disk as it completes. Serialized inside the core, but we also guard the stream
        // here.
        const bool had_rows = !resumed.empty();
        append_stream.open(summary_path, had_rows ? std::ios::app : std::ios::out);
        if (!append_stream) throw std::runtime_error("cannot open checkpoint: " + summary_path);
        if (!had_rows) append_stream << header;
        if (!resumed.empty())
          std::cout << "checkpoint: resuming, " << resumed.size() << " cases already done\n";
        opts.on_case_done = [&](const gncsim::MonteCarloCase& c) {
          std::lock_guard<std::mutex> lock(append_mutex);
          append_stream << formatCase(c) << '\n';
          append_stream.flush();
        };
      }

      const auto cases = gncsim::runMonteCarloParallel(cfg, opts);
      append_stream.close();

      // Always (re)write a complete, index-ordered summary.csv from the merged result so the file
      // is canonical regardless of completion order or a partial checkpoint.
      std::ostringstream summary;
      summary.precision(9);
      summary << header;
      int hits = 0;
      for (const auto& c : cases) {
        summary << formatCase(c) << '\n';
        hits += c.intercept ? 1 : 0;
      }
      writeFile(summary_path, summary.str());

      // Representative full run (case 0's seed) for plotting a sample trajectory.
      if (!cases.empty()) {
        gncsim::SimConfig c0 = cfg;
        c0.monte_carlo.num_cases = 0;
        c0.seed = cases.front().seed;
        const gncsim::SimResult r0 = gncsim::runSimulation(c0);
        for (const auto& [name, content] : gncsim::toCsvFiles(r0)) {
          writeFile(out_dir + "/" + name, content);
        }
        writeFile(out_dir + "/manifest.json", gncsim::toManifestJson(r0, config_text));
      }

      std::cout << "monte carlo: " << cases.size() << " cases, " << hits << " intercepts ("
                << (100.0 * hits / cases.size()) << "%)\n"
                << "wrote " << out_dir << "/summary.csv + sample telemetry\n";
      return 0;
    }

    const gncsim::SimResult result = gncsim::runSimulation(cfg);
    for (const auto& [name, content] : gncsim::toCsvFiles(result)) {
      writeFile(out_dir + "/" + name, content);
    }
    writeFile(out_dir + "/manifest.json", gncsim::toManifestJson(result, config_text));
    if (!json_path.empty()) {
      writeFile(json_path, gncsim::toJsonString(result));  // identical shape to the WASM output
    }

    std::cout << "scenario=" << result.scenario << " model=" << result.model
              << " seed=" << result.seed << " frames=" << result.frames.size()
              << " miss_distance=" << result.miss_distance << " m"
              << " intercept=" << (result.intercept ? "yes" : "no") << "\n"
              << "wrote " << out_dir << "/{vehicle,target,gnc,sensors}.csv + manifest.json\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
