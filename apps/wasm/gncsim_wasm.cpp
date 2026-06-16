// gnc-sim — WebAssembly entry point (Emscripten). Exposes run_sim(configJson) -> resultJson to JS,
// plus the additive linearize(configJson) -> jsonResult analysis entry (issue #122). The Next.js
// app calls run_sim client-side; same pure core as the native CLI, so results are bit-for-bit
// comparable (see the native<->WASM parity test). linearize is a SEPARATE entry and does NOT affect
// run_sim.
#include <emscripten/bind.h>

#include <exception>
#include <sstream>
#include <string>

#include "gncsim/core/Config.hpp"
#include "gncsim/core/Serialize.hpp"
#include "gncsim/dynamics/Linearize.hpp"
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

namespace {

// Pull an optional numeric field out of a flat "trim" object in the config JSON. We avoid pulling
// nlohmann into this header by reusing the core loader: the trim condition rides in the SAME config
// document under an optional top-level "trim" block. loadConfigFromString ignores unknown keys, so
// we re-parse the trim numbers here with a tiny tolerant scan (no new dep, deterministic).
double readTrimField(const std::string& json, const std::string& key, double fallback) {
  const std::string needle = "\"" + key + "\"";
  const auto block = json.find("\"trim\"");
  if (block == std::string::npos) return fallback;
  const auto kpos = json.find(needle, block);
  if (kpos == std::string::npos) return fallback;
  const auto colon = json.find(':', kpos + needle.size());
  if (colon == std::string::npos) return fallback;
  try {
    return std::stod(json.substr(colon + 1));
  } catch (...) {
    return fallback;
  }
}

std::string fieldJson(const char* key, double value) {
  std::ostringstream os;
  os << "\"" << key << "\":" << value;
  return os.str();
}

}  // namespace

// Linearize the airframe+actuator pitch channel about a flight condition (issue #122). The flight
// condition rides in an optional top-level "trim": { "mach", "altitude_m", "alpha_rad" } block of
// the SAME config JSON (defaults applied when absent). Returns the short-period A/B and the derived
// ωn / ζ / control-effectiveness as JSON. Pure analysis — never runs the engagement.
std::string linearize(const std::string& config_json) {
  try {
    const gncsim::SimConfig cfg = gncsim::loadConfigFromString(config_json);
    gncsim::TrimCondition trim;
    trim.mach = readTrimField(config_json, "mach", trim.mach);
    trim.altitude_m = readTrimField(config_json, "altitude_m", trim.altitude_m);
    trim.alpha_rad = readTrimField(config_json, "alpha_rad", trim.alpha_rad);

    const gncsim::LinearizeResult r = gncsim::linearizeAirframe(cfg, trim);

    std::ostringstream os;
    os << "{";
    os << "\"a_matrix\":[" << r.a_matrix[0] << "," << r.a_matrix[1] << "," << r.a_matrix[2] << ","
       << r.a_matrix[3] << "],";
    os << "\"b_matrix\":[" << r.b_matrix[0] << "," << r.b_matrix[1] << "],";
    os << fieldJson("omega_n_radps", r.omega_n_radps) << ",";
    os << fieldJson("zeta", r.zeta) << ",";
    os << "\"stable\":" << (r.stable ? "true" : "false") << ",";
    os << fieldJson("control_effectiveness_mps2_per_rad", r.control_effectiveness_mps2_per_rad)
       << ",";
    os << fieldJson("q_bar_pa", r.q_bar_pa) << ",";
    os << fieldJson("speed_mps", r.speed_mps) << ",";
    os << fieldJson("iyy_kgm2", r.iyy_kgm2) << ",";
    os << fieldJson("cm_alpha_per_rad", r.cm_alpha_per_rad) << ",";
    os << fieldJson("cn_alpha_per_rad", r.cn_alpha_per_rad);
    os << "}";
    return os.str();
  } catch (const std::exception& e) {
    return std::string("{\"error\":\"") + e.what() + "\"}";
  }
}

EMSCRIPTEN_BINDINGS(gncsim_module) {
  emscripten::function("run_sim", &run_sim);
  emscripten::function("linearize", &linearize);
}
