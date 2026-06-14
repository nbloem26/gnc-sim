# gnc-sim — top-level build. Native (CLI + tests) and WASM (apps/wasm) share the same `core` lib.
cmake_minimum_required(VERSION 3.16)
project(gncsim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(GNCSIM_BUILD_TESTS "Build C++ unit tests" ON)
option(GNCSIM_BUILD_CLI "Build the native CLI" ON)
option(GNCSIM_BUILD_PYTHON "Build the pybind11 Python bindings (bindings/)" OFF)
# Per-step cost benchmark + perf-budget gate (issue #53). Native-only offline tooling; on by default
# alongside the CLI so CI can exercise the budget check. Never built under Emscripten.
option(GNCSIM_BUILD_BENCH "Build the native per-step perf benchmark" ON)

# FMI 2.0 Co-Simulation FMU export (issue #44). Opt-in: when OFF (default) the native build, CTest
# and every golden run are byte-for-byte unchanged. When ON, builds the co-sim slave shared library
# (the .so packaged into gncsim.fmu), the modelDescription.xml generator, and the in-repo master +
# its bit-match CTest case. The vendored FMI 2.0 headers (fmi/include/fmi2/, BSD-2-Clause) mean NO
# new package dependency. See fmi/ and docs/FMI.md.
option(GNCSIM_BUILD_FMI "Build the FMI 2.0 Co-Simulation FMU export (fmi/)" OFF)

# --- Test-depth tooling (issue #51), all opt-in so the DEFAULT build/test path is unchanged. ---
# Comma-separated sanitizer set, e.g. -DGNCSIM_SANITIZE=address,undefined. Empty (default) = off.
# Applies to every native target (library + CLI + tests) so CTest runs the whole suite instrumented.
set(GNCSIM_SANITIZE "" CACHE STRING
  "Comma-separated sanitizers to enable (e.g. address,undefined). Empty = off.")
# Coverage instrumentation (gcov/--coverage) for the coverage CI job. Off by default.
option(GNCSIM_COVERAGE "Build native targets with gcov coverage instrumentation" OFF)
# libFuzzer config-loader harness (fuzz/). Requires Clang. Off by default; never affects the
# default build or CTest (the portable fallback lives in tests/config_fuzz_stress_test.cpp).
option(GNCSIM_FUZZ "Build the libFuzzer config-loader harness (requires Clang)" OFF)

if(GNCSIM_SANITIZE AND NOT EMSCRIPTEN)
  message(STATUS "gnc-sim: enabling sanitizers: ${GNCSIM_SANITIZE}")
  add_compile_options(-fsanitize=${GNCSIM_SANITIZE} -fno-omit-frame-pointer -g)
  add_link_options(-fsanitize=${GNCSIM_SANITIZE})
endif()

if(GNCSIM_COVERAGE AND NOT EMSCRIPTEN)
  message(STATUS "gnc-sim: enabling gcov coverage instrumentation")
  # -fprofile-update=atomic makes gcov counters thread-safe so parallel Monte Carlo
  # (#43) doesn't corrupt .gcda files (negative-hit gcov parse errors). See gcc PR68080.
  add_compile_options(--coverage -O0 -g -fprofile-update=atomic)
  add_link_options(--coverage)
endif()

include(FetchContent)

# nlohmann/json — header-only, also works under Emscripten.
FetchContent_Declare(
  nlohmann_json
  URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
)
FetchContent_MakeAvailable(nlohmann_json)

add_subdirectory(core)

# Native-only targets (Emscripten builds apps/wasm instead, via build-wasm.sh).
if(NOT EMSCRIPTEN)
  if(GNCSIM_BUILD_CLI)
    add_subdirectory(apps/cli)
  endif()
  if(GNCSIM_BUILD_BENCH)
    add_subdirectory(apps/bench)
  endif()
  if(GNCSIM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
  endif()
  # libFuzzer harness (issue #51), opt-in and Clang-only — see fuzz/CMakeLists.txt.
  if(GNCSIM_FUZZ)
    add_subdirectory(fuzz)
  endif()
  # Python SDK (pybind11) — native-only client of the pure core, off by default (issue #32).
  if(GNCSIM_BUILD_PYTHON)
    add_subdirectory(bindings)
  endif()
  # FMI 2.0 Co-Simulation FMU export (issue #44) — wraps the pure core, off by default.
  if(GNCSIM_BUILD_FMI)
    add_subdirectory(fmi)
  endif()
else()
  add_subdirectory(apps/wasm)
endif()
