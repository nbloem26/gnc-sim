// gnc-sim — WebAssembly entry point (Emscripten). Exposes run_sim(configJson) -> resultJson to JS.
// The Next.js app calls this client-side; same pure core as the native CLI, so results are
// bit-for-bit comparable (see the native<->WASM parity test).
#include <exception>
#include <string>

#include <emscripten/bind.h>

#include "gncsim/core/Config.hpp"
#include "gncsim/core/Serialize.hpp"
#include "gncsim/scenario/Runner.hpp"

// Takes a JSON config string, returns a JSON result string (columnar series + metadata).
// On error returns {"error": "..."} so the frontend can surface it.
std::string run_sim(const std::string& config_json) {
  try {
    const gncsim::SimConfig cfg = gncsim::loadConfigFromString(config_json);
    const gncsim::SimResult result = gncsim::runSimulation(cfg);
    return gncsim::toJsonString(result);
  } catch (const std::exception& e) {
    return std::string("{\"error\":\"") + e.what() + "\"}";
  }
}

EMSCRIPTEN_BINDINGS(gncsim_module) {
  emscripten::function("run_sim", &run_sim);
}
