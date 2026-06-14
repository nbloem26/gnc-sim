# Engineering Write-ups

This project was built to answer two questions directly, with running code rather than prose.

---

## 1. A C++ GNC simulation tool on Linux, with Python post-processing & validation

**What it is.** A 3-DOF/6-DOF guided-interceptor simulation for terminal-homing GNC analysis. The
core is a pure C++17 library (`core/`) built with CMake on Linux and exercised by GoogleTest/CTest.
It models a flat-Earth engagement: US Standard Atmosphere 1976, Mach-dependent aerodynamic drag,
rigid-body dynamics integrated with RK4, a strapdown IMU and an angle-tracking seeker, an alpha-beta
navigation filter, **Proportional Navigation** guidance, and a 6-DOF acceleration autopilot. A
command-line driver (`apps/cli`) runs single engagements and Monte Carlo batches, emitting CSV
telemetry + a JSON manifest.

**How Python does post-processing & validation.** `postproc/` (numpy/scipy/pandas/matplotlib) loads
the telemetry and runs an **analytical validation suite** that re-runs the C++ binary and compares
against closed-form truth:

| Check | Result |
|---|---|
| Drag-free ballistic vs analytic parabola | max error **0.007 mm** |
| Terminal velocity vs `√(2mg / ρ C_d A)` | **0.43 %** |
| USSA76 density/temperature vs published tables | within **4.8 %** |
| Noise-free Proportional Navigation intercept | miss **< 1 cm** |
| RK4 vs RK2 error-vs-step-size slope | ≈ 4 (confirms integrator order) |

**Why it's credible (repeatability).** The same C++ source compiles to a native Linux binary *and*
to WebAssembly; a CI parity check asserts they produce **bit-identical** results (|Δ miss| = 0 over
2806 frames). Determinism is enforced by a seeded `mt19937_64` and fixed-step integration. This is
the "valid, repeatable environment" that must exist before layering on more fidelity.

> MATLAB note: the post-processing is Python by choice (license-free, reproducible for any reviewer).
> The analysis is organized so an equivalent MATLAB script set is a drop-in — the CSV/JSON data
> contract is language-neutral.

---

## 2. Statistical processing of measured sensor data → realistic simulation inputs

**The problem.** A guidance-phase Monte Carlo is only as trustworthy as its sensor models. Feeding in
hand-picked noise numbers produces miss-distance distributions that mean nothing. The right approach
is to **characterize the sensor statistically and feed the recovered parameters back into the sim.**

**Method — the Allan-variance fidelity loop** (`sensors/`):

1. **Generate / ingest measured data.** A long static IMU record (4 h @ 100 Hz) with injected
   white noise, a first-order Gauss-Markov bias instability, and rate random walk
   (`generate_imu_data.py`). (In a program setting this is bench data from the actual unit; the
   synthetic generator stands in and provides ground truth to validate the method.)
2. **Characterize.** Compute the overlapping **Allan deviation** σ(τ) and identify the three regimes
   by slope — white noise (−½, read as ARW/VRW at τ=1 s), bias instability (flat floor), and rate
   random walk (+½) — with a nonlinear least-squares regime fit (`allan_variance.py`).
3. **Feed back.** Write the recovered parameters to `configs/sensor_params.json`, which the C++
   `Imu`/`Seeker` models consume to reproduce the same error characteristics in-sim
   (`fit_noise_params.py`).
4. **Prove fidelity (loop closure).** Re-run the Allan analysis on the simulator's own logged sensor
   stream and confirm it reproduces the input spec — the in-sim white noise matched the configured
   value to **+0.1 %**, and the recovered bias instability tracked truth within ~5–10 %.

**The payoff.** With the characterized seeker + IMU noise driving a 200-case Monte Carlo (plus launch
dispersions), the guidance phase yields a **CEP of 0.56 m** (mean 0.63 m, P90 1.2 m) — a defensible
performance distribution traceable all the way back to the sensor's measured statistics. Noise-free,
the same engagement intercepts to < 1 cm, isolating the contribution of sensor error to miss.

Figures for all of the above are on the deployed app's **Engineering Validation** page (Allan
deviation, loop-closure, Monte Carlo CEP, validation summary).

---

## Capabilities demonstrated

| Area | In this project |
|---|---|
| C++ GNC algorithms & simulation on **Linux** | `core/` + `apps/cli`, CMake, CTest |
| **Python** (MATLAB-ready) analysis toolchain | `sensors/`, `postproc/`, notebooks |
| **Statistical processing of measured data** into realistic sim inputs | Allan-variance loop → `sensor_params.json` |
| **Seeker / target tracking** | angle-tracking seeker model + alpha-beta tracker (→ EKF, see roadmap) |
| **Guidance** to a target state | Proportional Navigation; PN intercept validated |
| **Control-law** design | 6-DOF acceleration autopilot |
| **Verification & testing** | analytical validation suite + native↔WASM parity + determinism |
| **Continuous build/test/deploy** | GitHub Actions: build → test → WASM → parity → deploy |

The next steps toward a multi-sensor launch-engagement environment (boost-phase threat, ground +
space-based trackers, multi-sensor fusion with a Kalman filter, interceptor coordination) are laid
out in [ROADMAP.md](ROADMAP.md) — deliberately staged on top of the repeatable core proven here.
