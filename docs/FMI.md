# FMI 2.0 Co-Simulation — exporting gnc-sim as an FMU (issue #44)

This document describes the **FMI 2.0 for Co-Simulation** export of the gnc-sim engagement: how to
build and package it, the variable interface, the time-stepping + determinism contract, and the
status of external-FMU *import* (e.g. a Simulink controller).

The goal is to **meet Simulink / MATLAB-centric teams where they are** (TARGET_ARCHITECTURE §7):
the pure C++ core can be dropped into any FMI-capable master (Simulink, Dymola, OpenModelica,
FMPy, PyFMI, …) as a standard `.fmu`.

> **Scope note.** This is a *credible, tested subset*. The export side (gnc-sim **as** an FMU) is
> implemented, packaged, and bit-exactly verified by an in-repo master. The import side (running an
> external Simulink controller in-the-loop) has its interface reserved and design documented here;
> closing that loop is a scoped follow-up (see [§5](#5-importing-an-external-fmu-status--design)).

---

## 1. What is built

Everything lives under [`fmi/`](../fmi) and is built **only** behind the opt-in CMake flag
`-DGNCSIM_BUILD_FMI=ON`. With the flag **off (the default)** the native build, CTest and every
golden run are byte-for-byte unchanged — the pure core (`core/`) is never touched; the FMU is a thin
wrapper that lives entirely in `fmi/`.

| Target | What |
|---|---|
| `gncsim_fmi` (`fmi/src/GncsimFmu.cpp`) | The FMI 2.0 Co-Simulation **slave** shared library — exports the canonical `fmi2*` C entry points. Packaged into the `.fmu` as `binaries/linux64/gncsim.so`. |
| `generate_model_description` (`fmi/src/GenerateModelDescription.cpp`) | Build-time tool that emits `modelDescription.xml` **from the same variable-layout header** the slave uses (`fmi/src/GncsimFmuVars.hpp`), so the XML and the binary can never drift. Not shipped in the `.fmu`. |
| `fmi_master` (`fmi/master/FmiMaster.cpp`) | A minimal **in-repo co-simulation master** (no external simulator) that `dlopen()`s the slave, drives a full engagement via `fmi2DoStep`, and asserts the streamed outputs **bit-match** `runSimulation()`. Wired into CTest as `fmi_master_test`. |

The FMI 2.0 standard headers (`fmi2Functions.h`, `fmi2FunctionTypes.h`, `fmi2TypesPlatform.h`) are
**vendored** under [`fmi/include/fmi2/`](../fmi/include/fmi2) (BSD-2-Clause, header-only) — there is
**no new package-manager dependency**.

---

## 2. Build, package, run

```bash
# Configure + build behind the opt-in flag (separate build dir; the default build is untouched).
cmake -S . -B build-fmi -DCMAKE_BUILD_TYPE=Release -DGNCSIM_BUILD_FMI=ON
cmake --build build-fmi -j"$(nproc)"

# The bit-match master test (dlopen the slave, step it, compare to runSimulation):
ctest --test-dir build-fmi -R fmi_master_test --output-on-failure

# Package the standards-compliant .fmu (zip of modelDescription.xml + the shared library):
./fmi/scripts/package_fmu.sh build-fmi build-fmi/gncsim.fmu
```

The resulting `gncsim.fmu` has the FMI 2.0 archive layout:

```
gncsim.fmu
├── modelDescription.xml          # at the archive root
└── binaries/
    └── linux64/
        └── gncsim.so             # <modelIdentifier>.so == gncsim.so
```

You can hand `gncsim.fmu` to any FMI-2.0-capable master. For example, with the Python
[FMPy](https://github.com/CATIA-Systems/FMPy) reference master:

```python
# pip install fmpy   (illustrative; FMPy is NOT a gnc-sim dependency)
from fmpy import simulate_fmu

# Override the engagement parameters by their modelDescription.xml names.
result = simulate_fmu(
    "build-fmi/gncsim.fmu",
    start_time=0.0, stop_time=20.0, step_size=0.005,
    start_values={
        "seed": 7, "launch_speed_mps": 900.0, "launch_elevation_deg": 35.0,
        "target_pos_x_m": 9000.0, "target_pos_z_m": 3500.0,
    },
    output=["time_s", "veh_pos_x_m", "veh_pos_z_m", "range_m", "miss_m", "intercept"],
)
print("final miss_m:", result["miss_m"][-1])
```

The worked, dependency-free reference master is `fmi/master/FmiMaster.cpp` — read it as the canonical
example of the call sequence.

---

## 3. Variable interface

The FMU exposes a **flat, all-`Real`** interface (a uniform numeric surface for co-sim masters).
Value references are defined once in `fmi/src/GncsimFmuVars.hpp` and used by both the binary and the
descriptor. Names carry SI unit suffixes per the repo convention.

**Parameters** (causality `parameter`, set before `fmi2ExitInitializationMode`): `seed`,
`dt_step_s`, `t_end_s`, `launch_speed_mps`, `launch_elevation_deg`, `launch_azimuth_deg`,
`target_pos_{x,y,z}_m`, `target_vel_{x,y,z}_mps`.

**Inputs** (causality `input`, reserved for external-controller import — see §5):
`accel_cmd_override_{x,y,z}_mps2`, `accel_cmd_override_enable`.

**Outputs** (causality `output`, streamed one per communication step): `time_s`,
`veh_pos_{x,y,z}_m`, `veh_vel_{x,y,z}_mps`, `tgt_pos_{x,y,z}_m`, `accel_cmd_{x,y,z}_mps2`,
`range_m`, `closing_speed_mps`, `miss_m`, `intercept`, `done`.

Only the parameters/inputs are writable; the outputs are read-only (`fmi2SetReal` of an output
returns `fmi2Error`).

---

## 4. Time-stepping + determinism (the important part)

gnc-sim's `runSimulation()` runs a **fixed-step (RK4), seeded-`mt19937_64`** GNC loop to completion
and returns **one telemetry `Frame` per `dt` step** (AGENTS.md golden rules #2/#3: native↔WASM
bit-identity, analytic sub-`dt` CPA miss). A co-simulation master normally advances a slave
incrementally via `fmi2DoStep`. We must reconcile those two without re-implementing — and so risking
floating-point reordering of — the GNC pipeline.

**Design: precompute-then-stream.** At `fmi2ExitInitializationMode` (i.e. once all parameters are
set), the slave runs the **whole** `runSimulation()` **once** from the parameter-derived
`SimConfig`, caches the resulting `SimResult`, and resets a frame cursor. Each `fmi2DoStep` advances
that cursor by `round(communicationStepSize / dt_step_s)` frames; `fmi2GetReal` reads the current
frame's state. Because the streamed values **are** `runSimulation()`'s output frames, they are
**bit-identical** to a direct `runSimulation(cfg)` of the same config — which is exactly what
`fmi_master_test` asserts (every frame + the final analytic-CPA `miss_m`, via an
object-representation `memcmp`, zero tolerance).

Consequences a master must respect:

- **Communication step = core `dt` (or an integer multiple).** The core only knows `dt`-granular
  frames. The descriptor advertises `canHandleVariableCommunicationStepSize="true"`, but a
  *sub-`dt`* communication step is not meaningful and returns `fmi2Warning`. The slave rounds
  `communicationStepSize / dt_step_s` to the nearest whole number of frames, so floating-point
  accumulation of the communication point can never drop or duplicate a frame.
- **`miss_m` / `intercept` are authoritative only when `done == 1`.** While stepping, `miss_m` is
  the running *minimum sampled range* (a monotonic in-progress estimate, never below the true CPA);
  once the engagement is fully stepped, the slave reports the core's **analytic, sub-`dt`** CPA miss
  and intercept verdict verbatim (equal to `SimResult.miss_distance` / `.intercept`).
- **Determinism is total.** The seed is a parameter; the same `(config, seed)` yields a
  bit-identical engagement on every run, master, and platform — the same guarantee the native↔WASM
  parity test enforces for the core.

This design also means the FMU is **causal and side-effect-free per step**: `fmi2DoStep` does no
computation beyond advancing an index, so it is trivially real-time and cannot stall a master.

---

## 5. Importing an external FMU (status + design)

The issue's stretch goal — running an *external* Simulink controller **in-the-loop** with the core —
is **deferred**, with the interface reserved and the design recorded here:

- The FMU already exposes the `accel_cmd_override_*` **inputs** + an `accel_cmd_override_enable`
  flag. The intended closed-loop master pattern is: each `dt`, the master reads the gnc-sim FMU's
  relative-state outputs, feeds them to the external controller FMU's `fmi2DoStep`, reads back its
  commanded acceleration, writes it to gnc-sim's override inputs, then steps gnc-sim.
- Closing that loop requires the **core** to accept a *per-step external guidance command* instead
  of its internal ProNav — i.e. a step-wise `runSimulation` entry (a `stepEngagement(state) ->
  state` API on the pure core, or a guidance-injection hook). The current `runSimulation()` runs to
  completion internally, which is why the precompute-then-stream design above is bit-exact but
  *open-loop*. Adding the step-wise core API is the clean, scoped follow-up; it touches `core/`
  (under the same determinism rules) and is intentionally **not** bundled into this wrapper-only
  change.
- Why not demo the import now: a credible in-loop demo needs a *sample external FMU* to import, and
  building/redistributing a Simulink-generated FMU is not possible in CI without MATLAB. The master
  *plumbing* to load and co-step a second FMU is a direct extension of `fmi/master/FmiMaster.cpp`
  (it already `dlopen()`s + drives an FMU); the missing piece is the core-side injection hook above,
  not the master.

When the step-wise core API lands, the override inputs documented here become live and the
in-the-loop worked example (gnc-sim airframe + external guidance FMU) can be added without changing
the FMU's variable layout.

---

## 6. Files

| File | Role |
|---|---|
| `fmi/include/fmi2/*.h` | Vendored FMI 2.0 standard headers (BSD-2-Clause). |
| `fmi/src/GncsimFmuVars.hpp` | Single source of truth for the variable layout / value references. |
| `fmi/src/GncsimFmu.cpp` | The FMI 2.0 Co-Simulation slave (wraps `runSimulation()`). |
| `fmi/src/GenerateModelDescription.cpp` | Emits `modelDescription.xml` from the layout header. |
| `fmi/master/FmiMaster.cpp` | In-repo master + bit-match determinism test (`fmi_master_test`). |
| `fmi/scripts/package_fmu.sh` | Zips the `.so` + `modelDescription.xml` into a `.fmu`. |
| `fmi/CMakeLists.txt` | Build wiring (only under `-DGNCSIM_BUILD_FMI=ON`). |
