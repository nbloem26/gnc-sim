# gnc-sim — FMI 2.0 Co-Simulation export (issue #44). Built ONLY when -DGNCSIM_BUILD_FMI=ON, so
# the default native/CLI/test build is byte-for-byte unaffected. Three targets:
#
#   gncsim_fmi              the co-sim slave shared library (the .so packaged into gncsim.fmu),
#                           exporting the canonical fmi2* C entry points. Wraps the pure core.
#   generate_model_description  build-time tool emitting modelDescription.xml from the shared
#                           variable layout (fmi/src/GncsimFmuVars.hpp) — kept in lockstep.
#   fmi_master              the in-repo co-simulation master that dlopen()s the slave and asserts
#                           its streamed outputs bit-match runSimulation() (CTest: fmi_master_test).
#
# The vendored FMI 2.0 standard headers live under fmi/include/fmi2/ (BSD-2-Clause, header-only):
# NO package-manager dependency.

# --- The co-simulation slave shared library -------------------------------------------------
# A MODULE library (loadable plugin) exporting the fmi2* symbols. It links the same pure
# gncsim_core the CLI uses, so the engagement it runs is bit-identical to runSimulation().
add_library(gncsim_fmi MODULE src/GncsimFmu.cpp)
target_include_directories(gncsim_fmi PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(gncsim_fmi PRIVATE gncsim_core)
target_compile_features(gncsim_fmi PRIVATE cxx_std_17)
# PIC: the core is static; a shared FMU module that links it needs position-independent code.
set_target_properties(gncsim_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
# The FMI shared-object name inside the .fmu is "<modelIdentifier>.so" on Linux. Drop the "lib"
# prefix so the file is exactly `gncsim.so`, matching modelIdentifier="gncsim".
set_target_properties(gncsim_fmi PROPERTIES PREFIX "" OUTPUT_NAME "gncsim")
if(NOT MSVC)
  target_compile_options(gncsim_fmi PRIVATE -Wall -Wextra)
  # Hide everything except the FMI2_Export-annotated fmi2* symbols (clean export table).
  target_compile_options(gncsim_fmi PRIVATE -fvisibility=hidden)
endif()

# --- modelDescription.xml generator ---------------------------------------------------------
add_executable(generate_model_description src/GenerateModelDescription.cpp)
target_include_directories(generate_model_description PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_compile_features(generate_model_description PRIVATE cxx_std_17)
if(NOT MSVC)
  target_compile_options(generate_model_description PRIVATE -Wall -Wextra)
endif()

# Generate the descriptor at build time into this module's build dir (where the slave library is
# also emitted), so the packaging script finds both in one place.
set(GNCSIM_FMI_MODELDESC "${CMAKE_CURRENT_BINARY_DIR}/modelDescription.xml")
add_custom_command(
  OUTPUT "${GNCSIM_FMI_MODELDESC}"
  COMMAND generate_model_description "${GNCSIM_FMI_MODELDESC}"
  DEPENDS generate_model_description
  COMMENT "Generating FMU modelDescription.xml")
add_custom_target(gncsim_fmi_modeldesc ALL DEPENDS "${GNCSIM_FMI_MODELDESC}")
add_dependencies(gncsim_fmi_modeldesc gncsim_fmi)

# --- The in-repo co-simulation master + bit-match test --------------------------------------
add_executable(fmi_master master/FmiMaster.cpp)
target_include_directories(fmi_master PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(fmi_master PRIVATE gncsim_core ${CMAKE_DL_LIBS})
target_compile_features(fmi_master PRIVATE cxx_std_17)
if(NOT MSVC)
  target_compile_options(fmi_master PRIVATE -Wall -Wextra)
endif()
add_dependencies(fmi_master gncsim_fmi)

# Register the master as a CTest case: it dlopen()s the freshly-built slave .so and asserts the
# bit-for-bit match. Only added when CTest is enabled (GNCSIM_BUILD_TESTS, the default).
if(GNCSIM_BUILD_TESTS)
  add_test(NAME fmi_master_test
           COMMAND fmi_master "$<TARGET_FILE:gncsim_fmi>")
endif()
