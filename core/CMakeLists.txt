# gnc-sim core simulation library. Sources are GLOB'd (CONFIGURE_DEPENDS) so Phase 1 module agents
# can add core/src/<module>/*.cpp without ever editing this file — avoids parallel-edit conflicts.
file(GLOB_RECURSE GNCSIM_CORE_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_library(gncsim_core STATIC ${GNCSIM_CORE_SOURCES})

target_include_directories(gncsim_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(gncsim_core PUBLIC nlohmann_json::nlohmann_json)
target_compile_features(gncsim_core PUBLIC cxx_std_17)

if(NOT MSVC)
  target_compile_options(gncsim_core PRIVATE -Wall -Wextra)
endif()
