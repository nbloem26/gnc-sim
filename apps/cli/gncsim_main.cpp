// gnc-sim native CLI: run a scenario and write CSV telemetry + manifest.json to an output folder.
// Usage: gncsim --config configs/homing_3dof.json --out runs/run_001 [--seed N]
//
// This is the OFFLINE / Monte-Carlo entry point. It wraps the pure core (runSimulation +
// serializers); the only file I/O in the whole project lives here and in the Monte Carlo batch.
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

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

}  // namespace

int main(int argc, char** argv) {
  try {
    const std::string config_path = arg(argc, argv, "--config", "");
    const std::string out_dir = arg(argc, argv, "--out", "runs/run");
    const std::string seed_override = arg(argc, argv, "--seed", "");
    const std::string json_path = arg(argc, argv, "--json", "");  // also emit WASM-format JSON

    if (config_path.empty()) {
      std::cerr << "usage: gncsim --config <file.json> --out <dir> [--seed N]\n";
      return 2;
    }

    const std::string config_text = readFile(config_path);
    gncsim::SimConfig cfg = gncsim::loadConfigFromString(config_text);
    if (!seed_override.empty()) {
      cfg.seed = static_cast<std::uint64_t>(std::stoull(seed_override));
    }

    ensureDir(out_dir);

    // Monte Carlo batch: write summary.csv (one row/case) + full telemetry for case 0.
    if (cfg.monte_carlo.num_cases > 0) {
      const auto cases = gncsim::runMonteCarlo(cfg);
      std::ostringstream summary;
      summary.precision(9);
      summary << "case,seed,miss_distance,intercept_time,intercept\n";
      int hits = 0;
      for (const auto& c : cases) {
        summary << c.index << ',' << c.seed << ',' << c.miss_distance << ',' << c.intercept_time
                << ',' << (c.intercept ? 1 : 0) << '\n';
        hits += c.intercept ? 1 : 0;
      }
      writeFile(out_dir + "/summary.csv", summary.str());

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
