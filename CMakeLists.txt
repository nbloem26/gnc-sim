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
  if(GNCSIM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
  endif()
  # Python SDK (pybind11) — native-only client of the pure core, off by default (issue #32).
  if(GNCSIM_BUILD_PYTHON)
    add_subdirectory(bindings)
  endif()
else()
  add_subdirectory(apps/wasm)
endif()
