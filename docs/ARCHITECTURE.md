# Architecture

## One core, two targets

The simulation is a **pure C++17 library** (`core/`) that takes a config and returns an in-memory
`SimResult` — no file I/O, no globals, no platform calls in the hot path. That purity is what lets
the *exact same source* run in two places:

```
                         ┌─────────────────────────┐
                         │   core/  (pure C++17)    │
                         │  math · env · aero ·     │
                         │  dynamics · sensors ·    │
                         │  gnc · scenario(Runner)  │
                         └───────────┬─────────────┘
                                     │ runSimulation(SimConfig) -> SimResult
                  ┌──────────────────┴───────────────────┐
                  ▼                                       ▼
        apps/cli  (native, Linux)              apps/wasm  (Emscripten)
        SimResult -> CSV + manifest            run_sim(json) -> json
        offline runs, Monte Carlo              client-side, in the browser
                  │                                       │
                  ▼                                       ▼
        postproc/ + sensors/ (Python)          web/  (Next.js on Vercel)
        Allan variance, validation,            interactive form, Plotly,
        Monte Carlo, figures                   3D trajectory, Leaflet ground track
```

A **native↔WASM parity check** (`scripts/parity-check.mjs`, run in CI) asserts both targets produce
bit-identical results for the same config + seed. Determinism comes from a seeded `std::mt19937_64`
and fixed-step RK4 — the foundation of a *valid, repeatable* environment.

## Per-step pipeline (the GNC loop)

`core/src/scenario/Runner.cpp`, each fixed `dt`:

1. **Environment** — gravity (constant or inverse-square) + US Standard Atmosphere 1976 (ρ, T, P, a).
2. **Aerodynamics** — Mach-dependent drag; 6DOF adds angle-of-attack normal force.
3. **Sensors** — seeker LOS + strapdown IMU corrupted with the *characterized* noise (when enabled).
4. **Navigation** — alpha-beta tracker turns the noisy measurement into a relative-state estimate.
5. **Guidance** — Proportional Navigation: `a_cmd = N · V_c · (Ω × û_LOS)`, magnitude-limited.
6. **Control** — 3DOF applies the command directly; 6DOF autopilot → body moment → attitude → aero.
7. **Dynamics** — RK4 step of translational (and 6DOF rotational quaternion) state.

Miss distance is the **analytic closest approach** over each step (sub-`dt` accurate), so the metric
is independent of step size rather than quantized by `dt·V_c`.

## Module map

| Dir | Responsibility | Key types |
|---|---|---|
| `core/.../math` | Vector3, Quaternion, RK2/RK4 integrators | header-only |
| `core/.../core` | `EntityState`, `Frame`, `SimResult`, RNG, JSON config, serializers | data contract |
| `core/.../env` | `GravityModel`, `atmosphereUSSA76` | `AtmSample` |
| `core/.../aero` | `AeroModel` — `Cd(Mach)`, drag, normal force | |
| `core/.../dynamics` | `step3dof`, `step6dof` | EOM + integration |
| `core/.../sensors` | `Imu`, `Seeker` — error models | reproduce Allan params |
| `core/.../gnc` | `computeEngagement`, `proNavCommand`, `Navigator`, `Autopilot` | |
| `core/.../scenario` | `runSimulation`, `runMonteCarlo` | the loop |

## Terminal conditions (issue #26)

A run integrates a fixed-step loop and ends on the **first** of:

1. **Time limit** — `steps = t_end / dt`; the loop stops at `t_end`.
2. **Ground impact** — the interceptor returns below the surface (`alt < 0`); on the round-Earth
   path that surface is the WGS-84 ellipsoid.

Intercept is **not** an early stop. The miss is the **analytic closest point of approach (CPA)**,
interpolated within each step and tracked across the whole flight; `intercept = best_range <
kLethalRadius` (3 m). The threat is propagated independently and is not separately terminated.

**Decision — no post-CPA early-out (for now).** The sim deliberately runs to `t_end` / ground impact
rather than stopping once CPA is passed, so full-trajectory telemetry and bit-identical
golden/determinism guarantees are preserved. A config-gated early-out (stop after the closing velocity
flips sign post-CPA) is a possible future optimization if per-step cost becomes a constraint (see the
performance issue #53) — but it must stay **opt-in** so default trajectories and golden baselines never
change.

## Testing & CI

- **C++**: GoogleTest via CTest — one suite per module + an integration/determinism suite (7 total).
- **Python**: pytest over loaders, the USSA76/terminal-velocity/ballistic math, Allan slope recovery,
  and loop closure (27 tests).
- **CI** (`.github/workflows/ci.yml`): native build + CTest → Python rigor → Emscripten WASM build →
  native↔WASM parity → web build. Continuous build/test/deploy end to end.

See [DATA_CONTRACT.md](DATA_CONTRACT.md) for the schema shared across all four surfaces.
