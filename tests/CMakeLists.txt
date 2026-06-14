# gnc-sim tests. Each tests/*_test.cpp becomes its own CTest executable (GLOB'd, CONFIGURE_DEPENDS),
# so Phase 1 module agents drop in e.g. tests/env_test.cpp with no edits to this file.
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

file(GLOB GNCSIM_TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*_test.cpp")

foreach(test_src ${GNCSIM_TEST_SOURCES})
  get_filename_component(test_name "${test_src}" NAME_WE)
  add_executable(${test_name} "${test_src}")
  target_link_libraries(${test_name} PRIVATE gncsim_core GTest::gtest_main)
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
