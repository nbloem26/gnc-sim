// gnc-sim — libFuzzer entry point for the JSON config loader (issue #51).
//
// Built only when -DGNCSIM_FUZZ=ON with a Clang toolchain (libFuzzer ships with Clang). The CMake
// option wires in `-fsanitize=fuzzer,address,undefined`, so every fuzz iteration runs the parse
// path under ASan+UBSan and libFuzzer's coverage-guided mutation. Run it as:
//
//   cmake -S . -B build-fuzz -DGNCSIM_FUZZ=ON -DCMAKE_CXX_COMPILER=clang++
//   cmake --build build-fuzz --target config_fuzzer
//   ./build-fuzz/fuzz/config_fuzzer -max_total_time=60 fuzz/corpus
//
// A non-zero exit / crash artifact means a real defect (UB, OOB, leak) in the parser.
#include <cstddef>
#include <cstdint>

#include "config_fuzz_target.hpp"

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
  (void)gncsim::fuzz::exerciseConfigLoader(data, size);
  return 0;  // libFuzzer treats any crash/sanitizer trap as the failure signal.
}
