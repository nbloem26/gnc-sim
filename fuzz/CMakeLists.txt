# gnc-sim — libFuzzer config-loader harness (issue #51). Built only when GNCSIM_FUZZ=ON, which
# requires a Clang toolchain (libFuzzer ships with Clang). Kept entirely separate from the default
# build so native/WASM/test paths are untouched. See fuzz/config_fuzzer.cpp for run instructions.

if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  message(FATAL_ERROR
    "GNCSIM_FUZZ=ON requires a Clang toolchain (libFuzzer). "
    "Re-run with -DCMAKE_CXX_COMPILER=clang++ (and clang as the C compiler).")
endif()

add_executable(config_fuzzer config_fuzzer.cpp)
target_link_libraries(config_fuzzer PRIVATE gncsim_core)
target_include_directories(config_fuzzer PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")

# libFuzzer + ASan + UBSan: coverage-guided mutation with memory-safety instrumentation on the parse
# path. Applied only to this target's compile/link, never to the library defaults.
target_compile_options(config_fuzzer PRIVATE
  -fsanitize=fuzzer,address,undefined -fno-omit-frame-pointer -g -O1)
target_link_options(config_fuzzer PRIVATE
  -fsanitize=fuzzer,address,undefined)
