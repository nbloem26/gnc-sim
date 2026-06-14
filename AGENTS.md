# AGENTS.md — working in gnc-sim

Authoritative guide for autonomous coding agents (and humans) working in this repo.
`CLAUDE.md` is a symlink to this file. Keep this file current when conventions change.

## What this project is

A guided-interceptor **Guidance, Navigation & Control** simulation. A deterministic **pure C++17
core** (`core/`) computes an engagement; the *same source* runs two ways:

- **native** (`apps/cli/`) → CSV telemetry + JSON manifest, for offline runs / Monte Carlo / validation.
- **WebAssembly** (`apps/wasm/`, Emscripten) → `run_sim(configJson) -> resultJson`, runs **client-side
  in the browser** under the Next.js app in `web/` (deployed to Vercel).

Python (`sensors/`, `postproc/`) drives offline rigor: sensor characterization (Allan variance),
closed-form validation, Monte Carlo, and figures. MATLAB/Octave reference scripts live in `matlab/`.

**The contract is sacred.** `docs/DATA_CONTRACT.md` defines the one JSON/CSV schema that C++, Python,
the web app, and MATLAB all agree on. Change the schema → update the contract doc, all four surfaces,
and re-baseline golden runs in the same change.

## Golden rules (do not violate without explicit reason)

1. **`core/` is pure.** No file I/O, no globals, no platform calls in the hot path — that purity is
   what lets it compile to WASM. If you add I/O to `core/`, you break the browser build.
2. **Determinism is a feature.** Seeded `std::mt19937_64` + fixed-step RK4. Native and WASM must stay
   **bit-identical** for the same config+seed — `scripts/parity-check.mjs` asserts `|Δ| = 0` in CI.
   Do not introduce nondeterminism (unordered iteration over hash maps, time-based seeds, FP reordering).
3. **Miss distance is analytic CPA**, sub-`dt` accurate (`Runner.cpp`), independent of step size. Don't
   "simplify" it to nearest-sample distance.
4. **Monte Carlo IC dispersion lives in C++** (`core/src/scenario/MonteCarlo.cpp`), driven by the
   `monte_carlo` sigma block in the config. The web panel is a *separate, simplified* reimplementation
   — see Known issues. Prefer reusing the contract's sigmas over inventing new ones.

## Repo map

| Path | What |
|---|---|
| `core/include/gncsim/`, `core/src/` | Pure C++17 library: `math · core(data contract) · env · aero · dynamics · sensors · gnc · scenario(Runner)`. |
| `core/src/scenario/Runner.cpp` | **The per-step GNC loop** and the three scenario runners (homing, ballistic round, cued engagement). Terminal conditions live here. |
| `core/src/scenario/MonteCarlo.cpp` | Disperses initial conditions per `monte_carlo` sigmas; runs N reproducible cases. |
| `apps/cli/` | Native entry → CSV + manifest. |
| `apps/wasm/` | Emscripten entry: `run_sim(json) -> json`. |
| `tests/` | C++ GoogleTest suites (one per module + integration/determinism), run via CTest. |
| `sensors/` | Python: synthetic IMU → Allan variance → `configs/sensor_params.json`. |
| `postproc/gncpost/` | Python: validation vs closed-form, Monte Carlo, figures; `golden.py` regression harness. |
| `postproc/tests/` | Python pytest suite (loaders, USSA76/ballistic/terminal-velocity math, Allan slope, loop closure). |
| `web/` | Next.js + React. `app/page.tsx` (tabs), `components/` (per-subsystem panels), `lib/wasmRunner.ts`. |
| `configs/` | 18 scenario presets (the data contract in practice). Start here to understand inputs. |
| `matlab/` | Reference analysis scripts (Octave fallback). Not CI-executed. |
| `docs/` | `DATA_CONTRACT.md` (schema), `ARCHITECTURE.md` (one core/two targets + the loop), `ROADMAP.md`, `GOLDEN_RUNS.md`. |
| `scripts/` | `build-native.sh`, `build-wasm.sh`, `parity-check.mjs`. |

## Build / test / run — exact commands

**Native (build + C++ tests):**
```bash
./scripts/build-native.sh                 # cmake Release + ctest
./build-native/apps/cli/gncsim --config configs/homing_3dof.json --out runs/run_001
```

**WASM (requires emsdk on PATH — devcontainer/Dockerfile provide it):**
```bash
source /opt/emsdk/emsdk_env.sh
./scripts/build-wasm.sh                    # emits web/public/wasm/gncsim.{js,wasm}
```

**Web:**
```bash
cd web && npm install && npm run dev       # http://localhost:3000
npm run lint                               # next lint (+ tsc via build)
npm run build                              # static export; must pass before deploy
```
Without a built WASM artifact the web app runs in **MOCK mode** (serves the committed
`web/public/sample_result.json`); every seed returns the same sample, so live Monte Carlo is disabled.
Build WASM first for real client-side runs.

**Python rigor:**
```bash
pip install -r postproc/requirements.txt
python sensors/generate_imu_data.py && python sensors/fit_noise_params.py   # writes configs/sensor_params.json
python -m pytest postproc/tests -q
python -m postproc.gncpost.golden                 # golden-run regression (8 configs); --update to re-baseline
```

**Parity (native must equal WASM):**
```bash
node scripts/parity-check.mjs
```

## Linting / formatting (CI gates — run before pushing)

- **Python**: `ruff check .` · `ruff format --check .` · `mypy` (config in `pyproject.toml`).
- **C++**: `clang-format` (`.clang-format`); CI fails on diff.
- **Web**: `cd web && npm run lint` + `tsc`.
- **CI** (`.github/workflows/ci.yml`): lint/type-check → native build+CTest → Python rigor → WASM build
  → native↔WASM parity → web build. `golden.yml` runs the golden harness separately. Keep `ci.yml`
  untouched when adding unrelated workflows.

## Per-step pipeline (so you know where to make changes)

`core/src/scenario/Runner.cpp`, each fixed `dt`: **Environment** (gravity + USSA76) → **Aero**
(Mach-dependent drag; 6DOF normal force) → **Sensors** (seeker LOS + IMU with characterized noise,
when enabled) → **Navigation** (tracker → relative-state estimate) → **Guidance** (ProNav
`a_cmd = N·V_c·(Ω×û_LOS)`, magnitude-limited) → **Control** (3DOF direct; 6DOF autopilot→moment→attitude)
→ **Dynamics** (RK4). See `docs/ARCHITECTURE.md`.

### Terminal conditions (see issue #26)
A run ends on the **first** of: (1) **time** — `steps = t_end/dt`; (2) **ground impact** — interceptor
returns below the WGS-84 ellipsoid (`alt < 0`). Intercept is **not** an early-stop: the miss is the
continuous CPA tracked across the whole flight, `intercept = best_range < kLethalRadius (3 m)`. The
target is ballistic and not independently terminated.

## Naming conventions — units in variable names (see issue #69)

**Physical-quantity variables MUST carry an explicit SI unit suffix.** Units belong in the name, not
just a comment, so every use site is unambiguous. Applies to **all new and changed code** (C++, Python,
TS).

| Quantity | Suffix | Example |
|---|---|---|
| length / position | `_m` | `range_m`, `alt_m` |
| time | `_s` | `t_end_s`, `dt_s` |
| speed | `_mps` | `launch_speed_mps`, `v_closing_mps` |
| acceleration | `_mps2` | `accel_cmd_mps2` |
| mass | `_kg` | `mass_kg` |
| angle | `_rad` / `_deg` | `elevation_deg`, `aoa_rad` |
| angular rate | `_radps` | `los_rate_radps`, `body_rate_radps` |
| force | `_n` | `thrust_n` |
| frequency | `_hz` | `maneuver_freq_hz` |
| dimensionless | (none) | `nav_constant`, `mach` |

Use one token per unit (`_radps`, `_mps2`), not `_rad_s`. Dimensionless quantities take no suffix.

**Data-contract keys are special.** Config JSON keys and telemetry channel names in `SimResult.series`
/ CSV headers are the cross-language schema (see `docs/DATA_CONTRACT.md`). Renaming **those** to add
units is a **breaking change** — it needs a `schema_version` bump, all four surfaces updated together,
regenerated samples, and a golden re-baseline. So: apply the convention freely to **internal** variable
names; touch **contract keys** only via a deliberate, schema-versioned migration. Migration is phased
under #69 (Phase 1 = this rule; Phase 2 = internal sweep; Phase 3 = optional contract-key rename).

## Contribution / PR workflow

- **Branch naming:** `feature/<issue#>-<slug>` (e.g. `feature/5-multitracker`). One issue per branch.
- **Commits/PRs:** Conventional-commit titles (`feat(scope): …`, `fix(scope): …`). End commit messages
  and PR bodies with the `Co-Authored-By:` / generated-by trailers already used in history.
- **Closing issues — IMPORTANT (see issue #28):** GitHub only auto-closes a linked issue when the PR
  merges into the **default branch (`main`)**. If you **stack** PRs (base = another feature branch),
  the `Closes #N` keyword **will not fire** even after the work reaches `main`. Either:
  - put `Closes #N` on the PR that merges **into `main`**, or
  - merge each feature PR directly to `main` (rebase, don't stack), or
  - close the linked issues **manually** after merging a stack.
  This is exactly why issues #2–#8 stayed open despite merged PRs.
- **When you change the schema:** update `docs/DATA_CONTRACT.md` + every surface (C++ serializers,
  Python loaders, `web/lib/types.ts`, MATLAB `load_run.m`) and re-run `golden.py --update`.

## Known issues / gotchas (open tickets)

- **#23** — Web Monte Carlo varies *only the seed*; with sensors off it yields N identical runs. The
  fix is to mirror `core/.../MonteCarlo.cpp` IC dispersion in `web/components/MonteCarloPanel.tsx`.
- **#24** — Web has no scenario selector; `ParamForm.tsx` hardcodes `scenario: 'homing'`. The
  `configs/*.json` presets aren't exposed to the web build.
- **#25** — Trajectory and Ground Track are separate tabs; should be merged. Keep the Leaflet map's
  lazy `dynamic(ssr:false)` mount when combining.
- **#26** — Terminal conditions are correct but undocumented; possible opt-in post-CPA early-out.
- **#27** — Cesium 3D globe display (large feature; mind bundle size + ion token).
- **#28** — PR auto-close workflow (the stacking issue above).

Web-specific: panels mount **lazily per active tab** (see `web/app/page.tsx`) to avoid a Canvas2D
readback storm — don't eagerly instantiate every Plotly/Leaflet chart on load.
