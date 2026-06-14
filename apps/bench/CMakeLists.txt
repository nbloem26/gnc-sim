# Per-step cost micro-benchmark + perf-budget gate (issue #53). Native-only offline tooling,
# linked against the same pure core as the CLI; never built under Emscripten.
add_executable(gncsim-bench bench_main.cpp)
target_link_libraries(gncsim-bench PRIVATE gncsim_core)

# Register the perf-budget gate as a CTest case so `ctest` exercises it locally and in CI. Uses a
# small iter count for a quick smoke-level check; the budget headroom (configs/perf_budget.json)
# absorbs slow CI hardware. Only registered when the test suite is being built so a CLI-only build
# stays test-free.
# Only gate on timing in an optimized, NON-instrumented build — under coverage (gcov) or
# sanitizers the per-step cost is far higher than the budget and timing is meaningless, so the
# gate would false-fail those jobs. The standalone CI `perf-budget` job runs an explicit Release build.
if(GNCSIM_BUILD_TESTS AND NOT EMSCRIPTEN AND CMAKE_BUILD_TYPE STREQUAL "Release"
   AND NOT GNCSIM_COVERAGE AND NOT GNCSIM_SANITIZE)
  enable_testing()
  add_test(
    NAME perf_budget_homing_3dof
    COMMAND gncsim-bench
      --config "${CMAKE_SOURCE_DIR}/configs/homing_3dof.json"
      --budget "${CMAKE_SOURCE_DIR}/configs/perf_budget.json"
      --label homing_3dof
      --iters 100 --warmup 10)
endif()
