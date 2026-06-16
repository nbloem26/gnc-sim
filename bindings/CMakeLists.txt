# gnc-sim — Python bindings (pybind11). Built only when -DGNCSIM_BUILD_PYTHON=ON. Links the same
# pure `gncsim_core` library the CLI uses, so run(cfg) is bit-for-bit identical to the CLI. The
# compiled extension is named `_gncsim`; the importable `gncsim` package wraps it (see gncsim/).
#
# pybind11 is found via find_package first (pip: `pip install pybind11`), falling back to FetchContent
# so a clean checkout still builds. Provide the cmake dir explicitly with, e.g.:
#   cmake -S . -B build-python -DGNCSIM_BUILD_PYTHON=ON \
#         -Dpybind11_DIR=$(python -c "import pybind11; print(pybind11.get_cmake_dir())")

find_package(pybind11 CONFIG QUIET)
if(NOT pybind11_FOUND)
  message(STATUS "pybind11 not found via find_package; fetching it.")
  include(FetchContent)
  FetchContent_Declare(
    pybind11
    URL https://github.com/pybind/pybind11/archive/refs/tags/v2.13.6.tar.gz
  )
  FetchContent_MakeAvailable(pybind11)
endif()

pybind11_add_module(_gncsim gncsim_bindings.cpp)
target_link_libraries(_gncsim PRIVATE gncsim_core)
target_compile_features(_gncsim PRIVATE cxx_std_17)

# Drop the compiled extension next to the Python package so `import gncsim` works from this dir
# (the build script also exposes it on PYTHONPATH).
set_target_properties(_gncsim PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/gncsim")
