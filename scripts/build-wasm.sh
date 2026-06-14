#!/usr/bin/env bash
# Cross-compile the C++ core to WebAssembly via Emscripten and copy artifacts into the web app so
# Vercel can build the Next.js site with no Emscripten toolchain. Requires emsdk on PATH (emcmake).
set -euo pipefail
cd "$(dirname "$0")/.."

if ! command -v emcmake >/dev/null 2>&1; then
  echo "error: emcmake not found. Activate the Emscripten SDK (source /opt/emsdk/emsdk_env.sh)." >&2
  exit 1
fi

BUILD_DIR="${BUILD_DIR:-build-wasm}"
OUT_DIR="web/public/wasm"

emcmake cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release -DGNCSIM_BUILD_TESTS=OFF -DGNCSIM_BUILD_CLI=OFF
cmake --build "$BUILD_DIR" -j"$(nproc)"

mkdir -p "$OUT_DIR"
cp "$BUILD_DIR/apps/wasm/gncsim.js" "$OUT_DIR/"
cp "$BUILD_DIR/apps/wasm/gncsim.wasm" "$OUT_DIR/"

# Bundle the current sensor params so the web app's default run matches the characterized sensor.
cp configs/sensor_params.json web/public/sensor_params.json

echo "WASM artifacts copied to $OUT_DIR/"
