# gnc-sim

A guided-interceptor **Guidance, Navigation & Control** simulation: a deterministic C++ core
(built and tested on Linux) that **also compiles to WebAssembly** and runs **client-side in the
browser** via a Next.js app on Vercel — adjust inputs, re-run the actual C++ core in your browser,
and see the 2D/3D trajectory, ground track, and miss distance instantly. Sensor noise is
**characterized from data with Allan-variance analysis** and fed back into the sim; Python drives
the offline validation and Monte Carlo rigor.

**Validated results:** noise-free Proportional Navigation intercept < 1 cm · analytical checks
(ballistic, terminal velocity, USSA76) within tolerance · sensor loop-closure +0.1 % · Monte Carlo
**CEP 0.56 m** · native↔WASM **bit-identical** (parity |Δ| = 0).

### Engineering write-ups
- [docs/ANSWERS.md](docs/ANSWERS.md) — the C++/Linux + Python validation story and the
  measured-data → realistic-sim-inputs fidelity story, with results.
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — one core, two targets; the per-step GNC loop.
- [docs/ROADMAP.md](docs/ROADMAP.md) — staged path toward a multi-tracker launch-engagement sim.
- [docs/DATA_CONTRACT.md](docs/DATA_CONTRACT.md) — the schema shared by C++, Python, and the web app.

## What's here

| Path | What |
|---|---|
| `core/` | Pure C++17 simulation library (math, env, aero, dynamics, sensors, GNC). No file I/O → WASM-safe. |
| `apps/cli/` | Native CLI → CSV telemetry + manifest (offline runs, Monte Carlo). |
| `apps/wasm/` | Emscripten entry: `run_sim(configJson) -> resultJson` for the browser. |
| `sensors/` | Python: synthetic IMU → Allan variance → `configs/sensor_params.json`. |
| `postproc/` | Python: validation against closed-form solutions + Monte Carlo + figures. |
| `web/` | Next.js + React-Leaflet interactive demo (deployed to Vercel). |
| `docs/DATA_CONTRACT.md` | The shared schema all of the above agree on. |

## Quick start (native)

```bash
./scripts/build-native.sh                       # build + run C++ tests
./build-native/apps/cli/gncsim \
    --config configs/homing_3dof.json --out runs/run_001
```

## Build the browser sim (WASM)

```bash
source /opt/emsdk/emsdk_env.sh   # or use the devcontainer / Dockerfile
./scripts/build-wasm.sh          # emits web/public/wasm/gncsim.{js,wasm}
cd web && npm install && npm run dev
```

## Reproducible toolchain

`Dockerfile` / `.devcontainer/` provide gcc, CMake, Ninja, the **Emscripten SDK**, and Python — the
same environment CI uses. This is a build environment, not a runtime server (the deployed app is
fully client-side).
