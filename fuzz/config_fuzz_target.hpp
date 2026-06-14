// gnc-sim — shared config-fuzz target (issue #51).
//
// One function exercised by BOTH the real libFuzzer harness (config_fuzzer.cpp, clang-only) and the
// portable deterministic stress test (config_fuzz_stress.cpp, runs in CTest on any compiler). It
// feeds arbitrary bytes to the JSON config loader and asserts the only failure mode is a *caught*
// exception — never a crash, hang, or undefined behaviour. Under ASan/UBSan/libFuzzer any real
// memory error in the parse path becomes a hard failure here.
//
// CONTRACT: loadConfigFromString() must, for ANY input, either return a SimConfig or throw a
// std::exception (malformed JSON / wrong value types). It must not crash or leak. To keep the
// fuzzer fast we stop at parsing — we do not run the (much heavier) full simulation on fuzzed
// configs.
#pragma once

#include <cstddef>
#include <cstdint>
#include <exception>
#include <string>

#include "gncsim/core/Config.hpp"

namespace gncsim::fuzz {

// Returns true on a clean parse, false when the loader rejected the input with a caught exception.
// Either outcome is "no crash" — the caller asserts only that we returned normally. A non-exception
// failure (segfault / UB) is caught by the sanitizer / libFuzzer harness, not by this function.
inline bool exerciseConfigLoader(const std::uint8_t* data, std::size_t size) {
  const std::string json_text(reinterpret_cast<const char*>(data), size);
  try {
    const SimConfig cfg = loadConfigFromString(json_text);
    // Touch a few parsed fields so the optimizer cannot elide the parse, and so any UB while
    // reading back parsed values is exercised under sanitizers.
    volatile double sink = cfg.dt + cfg.t_end + static_cast<double>(cfg.seed);
    sink += cfg.vehicle.launch_speed + cfg.guidance.nav_constant;
    (void)sink;
    return true;
  } catch (const std::exception&) {
    // Expected: malformed JSON or a type mismatch. This is a clean rejection, not a crash.
    return false;
  }
}

}  // namespace gncsim::fuzz
