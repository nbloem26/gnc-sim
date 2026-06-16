// gnc-sim — portable deterministic config-loader stress test (issue #51).
//
// The libFuzzer harness (config_fuzzer.cpp) is the PREFERRED fuzzer but needs a Clang toolchain.
// This is the portable fallback that runs in CTest on ANY compiler (it is a GoogleTest target so it
// is GLOB'd into the suite like every other *_test.cpp). It does coverage-blind but DETERMINISTIC
// mutation: starting from a small in-source seed corpus of valid/near-valid configs, it applies
// Rng-driven byte flips, truncations, and structural noise, then asserts the loader never crashes
// (it either parses or throws a caught std::exception). Combined with the ASan+UBSan build job, a
// memory error in the parser surfaces here without needing Clang/libFuzzer.
//
// Determinism: the project Rng (fixed seed) drives all mutation, so the exact same byte streams are
// generated on every run and every platform. Nothing here can flake.
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "config_fuzz_target.hpp"
#include "gncsim/core/Rng.hpp"

using namespace gncsim;

namespace {

// Seed corpus: a spread of shapes the mutator perturbs. Valid configs, empty/partial objects,
// wrong-typed values, and deliberately malformed JSON — so mutations explore around real inputs.
const std::vector<std::string> kSeedCorpus = {
    R"({"scenario":"homing","model":"3dof","seed":1,"dt":0.005,"t_end":40.0})",
    R"({"scenario":"ballistic","env":{"frame":"round","atmosphere":false},"vehicle":{"launch_speed":2500}})",
    R"({"model":"6dof_hifi","vehicle":{"ixx":1.0,"iyy":2.0,"izz":3.0},"actuator":{"tau":0.02}})",
    R"({"trackers":{"enabled":true,"sensors":[{"type":"radar","pos":[0,0,0]}]}})",
    R"({"decoys":{"enabled":true,"count":5,"separability":0.5}})",
    R"({})",
    R"([])",
    R"({"dt":"not-a-number","seed":-1,"t_end":[1,2,3]})",
    R"({"target":{"pos0":[8000,0,3000],"maneuver":"weave","maneuver_g":7}})",
    R"(not json at all)",
    R"({"unterminated":)",
};

// Deterministic byte mutation of a seed string, driven entirely by the project Rng.
std::string mutate(const std::string& seed, Rng& rng) {
  std::string s = seed;
  if (!s.empty()) {
    const int num_flips = static_cast<int>(rng.uniform(0.0, 6.0));
    for (int k = 0; k < num_flips; ++k) {
      const std::size_t idx =
          static_cast<std::size_t>(rng.uniform(0.0, static_cast<double>(s.size())));
      // Flip to a random byte across the full 0..255 range (incl. control / high bytes / NUL).
      s[idx] = static_cast<char>(static_cast<int>(rng.uniform(0.0, 256.0)));
    }
  }
  // Occasionally truncate to a random prefix (exercises incomplete-document handling).
  if (rng.uniform(0.0, 1.0) < 0.3 && !s.empty()) {
    s.resize(static_cast<std::size_t>(rng.uniform(0.0, static_cast<double>(s.size()))));
  }
  // Occasionally append random bytes (exercises trailing-garbage handling).
  if (rng.uniform(0.0, 1.0) < 0.3) {
    const int extra = static_cast<int>(rng.uniform(0.0, 32.0));
    for (int k = 0; k < extra; ++k)
      s.push_back(static_cast<char>(static_cast<int>(rng.uniform(0.0, 256.0))));
  }
  return s;
}

}  // namespace

// Every seed corpus entry must itself load without crashing (valid ones parse; malformed ones throw
// a caught exception). This pins the "no crash for any input" contract on the curated inputs.
TEST(ConfigFuzz, SeedCorpusNeverCrashes) {
  for (const std::string& seed : kSeedCorpus) {
    EXPECT_NO_THROW({
      (void)fuzz::exerciseConfigLoader(reinterpret_cast<const std::uint8_t*>(seed.data()),
                                       seed.size());
    }) << "seed: "
       << seed;
  }
}

// Deterministic mutation sweep: thousands of fuzzed inputs, each must be handled without a crash.
// exerciseConfigLoader catches the expected std::exception internally and returns; a segfault / UB
// would trip the (optional) sanitizer build or abort the process here.
TEST(ConfigFuzz, MutatedInputsNeverCrash) {
  Rng rng(0xF0FA11);  // fixed seed -> identical byte streams every run
  constexpr int kIterations = 20000;
  for (int i = 0; i < kIterations; ++i) {
    const std::string& seed = kSeedCorpus[static_cast<std::size_t>(i) % kSeedCorpus.size()];
    const std::string input = mutate(seed, rng);
    // Returns true (parsed) or false (caught exception); either way it must return normally.
    EXPECT_NO_THROW({
      (void)fuzz::exerciseConfigLoader(reinterpret_cast<const std::uint8_t*>(input.data()),
                                       input.size());
    }) << "iteration "
       << i;
  }
}
