#!/usr/bin/env bash
# Build the gnc-sim Python SDK (pybind11) and make `import gncsim` work from the repo venv.
#
# Requires pybind11 (`pip install pybind11`, or it's in postproc/requirements.txt). The compiled
# extension is emitted into bindings/gncsim/ next to the Python package; this script prints the
# PYTHONPATH entry needed to import it (the pytest in bindings/ already adds it via conftest).
set -euo pipefail
cd "$(dirname "$0")/.."

BUILD_DIR="${BUILD_DIR:-build-python}"

# Locate pybind11's CMake config from whatever Python is active (venv).
PYBIND11_DIR="$(python -c 'import pybind11; print(pybind11.get_cmake_dir())' 2>/dev/null || true)"
EXTRA=()
if [[ -n "${PYBIND11_DIR}" ]]; then
  EXTRA+=("-Dpybind11_DIR=${PYBIND11_DIR}")
fi

cmake -S . -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DGNCSIM_BUILD_PYTHON=ON \
  -DGNCSIM_BUILD_TESTS=OFF \
  -DGNCSIM_BUILD_CLI=OFF \
  -DPYTHON_EXECUTABLE="$(command -v python)" \
  "${EXTRA[@]}"
cmake --build "${BUILD_DIR}" --target _gncsim -j"$(nproc)"

echo "python SDK built. To import:"
echo "  export PYTHONPATH=\"$(pwd)/bindings:\${PYTHONPATH:-}\""
echo "  python -c 'import gncsim; print(gncsim.run({\"scenario\":\"homing\"}).keys())'"
