#!/usr/bin/env bash
# Build the native simulation (CLI + tests) and run the test suite.
set -euo pipefail
cd "$(dirname "$0")/.."

BUILD_DIR="${BUILD_DIR:-build-native}"
cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" -j"$(nproc)"
ctest --test-dir "$BUILD_DIR" --output-on-failure

echo "native build complete: $BUILD_DIR/apps/cli/gncsim"
