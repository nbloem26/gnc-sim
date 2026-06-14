# Sensor Fidelity — IMU Noise Characterization

**Question this answers:** *"How did you ensure the simulated sensor data is realistic?"*

This directory characterizes a tactical-grade MEMS IMU error model with **Allan variance**,
recovers its noise parameters from data, feeds them into the C++ flight simulator, and then
**proves loop-closure** — the in-sim sensor reproduces the exact noise it was given. It is a
self-contained, deterministic, employer-facing demonstration of inertial-sensor metrology.

```
generate_imu_data.py ──▶ imu_raw.npz ──▶ allan_variance.py ──▶ allan_deviation.png
                                              │
                                              ▼
                                        fit_noise_params.py ──▶ configs/sensor_params.json
                                              │                          │
                                              │                          ▼
                                              │                  C++ Imu (core/src/sensors)
                                              │                          │
                                              ▼                          ▼  runs/<id>/sensors.csv
                              postproc/gncpost/loop_closure.py ◀─────────┘
                                              │
                                              ▼  re-run Allan variance on the sim's *_meas_* column
                              loop_closure_allan.png  (recovered ≈ configs/sensor_params.json)
```

---

## 1. The IMU error model

Per axis, sampled once per step at fixed `dt` (this is the exact convention the C++
`Imu::measure` reproduces — see `core/src/sensors/Sensors.cpp` and
`docs/DATA_CONTRACT.md §5`):

| Term | Per-sample update | Continuous units (accel / gyro) | Allan signature |
|---|---|---|---|
| **White noise** | `w ~ N(0, white/√dt)` | `(m/s²)/√Hz` (VRW) / `(rad/s)/√Hz` (ARW) | slope **−1/2** |
| **Bias instability** (Gauss-Markov) | `b ← b·e^(−dt/τ) + N(0, σ_GM·√(1−e^(−2dt/τ)))` | `m/s²` / `rad/s` (σ_GM, steady-state std) | **hump** at τ≈1.89·τ_corr |
| **Rate random walk** | `b += N(0, rrw·√dt)` (on the *same* `b`) | `(m/s²)/√s` / `(rad/s)/√s` | slope **+1/2** (when unbounded) |
| **Scale factor** | `× (1+scale_factor)` | fractional | (only on a non-zero true signal) |

```
measured = true·(1 + scale_factor) + b + w
```

`noise_model.py` is the Python twin of this model; both the synthetic generator and the
loop-closure re-derivation use it, so any drift from the C++ surfaces immediately.

---

## 2. Allan variance theory — the three slopes

The **overlapping Allan deviation** σ(τ) (`allan_variance.overlapping_allan_deviation`)
bins the record into clusters of length τ and takes the RMS of successive cluster-average
differences. On a log-log σ(τ) plot each error source has a characteristic slope, which is
why Allan variance cleanly separates noise processes that are tangled in a raw PSD:

* **slope −1/2 → white noise.** σ(τ) = N/√τ, so the angle/velocity random-walk
  coefficient **N** (ARW/VRW) is σ read at **τ = 1 s**.
* **flat floor / hump → bias instability.** An ideal flicker-noise IMU shows a flat floor
  with **B = min(σ)/0.664**. Our bias is a first-order **Gauss-Markov** process, which
  instead produces a *hump* peaking at **τ ≈ 1.89·τ_corr** with **peak ≈ 0.617·σ_GM**. We
  report that hump-peak deviation as B and recover τ_corr from the peak location.
* **slope +1/2 → rate random walk.** σ(τ) = K·√(τ/3).

### Extraction method

Rather than read each regime off a hand-windowed slope (brittle when the regimes overlap),
`identify_regimes` fits the **entire combined analytic Allan curve**

```
σ²(τ) = N²/τ  +  2·σ_GM²·(T/τ)·[1 − (T/2τ)(3 − 4e^(−τ/T) + e^(−2τ/T))]  +  K²·τ/3
```

by weighted nonlinear least squares (log-space, weighted by each point's equivalent
degrees of freedom). This is the standard rigorous IMU-characterization method and, by
construction, returns the same answer on a clean analytic curve as on a noisy measured one.
The leading **factor 2** on the GM term matches the overlapping estimator (validated against
simulated GM data; the textbook formula without it under-reads by √2).

---

## 3. Recovered vs. true (run `python sensors/fit_noise_params.py`)

A 4-hour, 100 Hz static record (3 axes) recovers the injected parameters to:

| Channel | white (N) | bias instability (B) | τ_corr | RRW (K) |
|---|---|---|---|---|
| accel | **−0 %** | **−5 %** | **−2 %** | below floor* |
| gyro  | **−0 %** | **+10 %** | **+15 %** | below floor* |

\* **RRW caveat (intentional and honest):** because the C++ model adds the rate random walk
onto the *same* bias state the GM term decays, the GM decay **bounds** the walk — there is no
free, unbounded +1/2 ramp, so RRW sits below the bias floor and is not separately identifiable
in a multi-hour record. The ground-truth labels are produced by **Monte-Carlo characterizing
the exact generator** (not the idealized independent-RRW formula), so "true" and "recovered"
refer to the same physically realizable quantity.

---

## 4. How recovered params feed the C++ sim

`fit_noise_params.py` writes `configs/sensor_params.json` in the schema the C++ `Imu`
consumes (`docs/DATA_CONTRACT.md §5`). Note the field mapping:

* `*_white`           ← recovered N (continuous density)
* `*_bias_instability`← recovered **σ_GM** (GM steady-state std — *not* the hump-peak B;
  they differ by the fixed ~0.617 GM factor)
* `*_bias_tau`        ← recovered correlation time T
* `*_rrw`             ← recovered K (small, below floor)
* `*_scale_factor`    ← echoed injected scale factor (deterministic, not Allan-recoverable
  from a static record)

The C++ `Imu` then reproduces these characteristics in every flight run, writing
`runs/<id>/sensors.csv`.

---

## 5. Loop-closure — the fidelity proof

`postproc/gncpost/loop_closure.py`:

1. runs the native CLI with sensors enabled on a long, smooth trajectory,
2. reads the `imu_accel_meas_x` column from `sensors.csv` and forms the noise residual
   `meas − true·(1+scale_factor)`,
3. re-runs the **same** Allan-variance characterization on that residual,
4. confirms the recovered N (and bias instability where flight length permits) match
   `configs/sensor_params.json` within tolerance,
5. writes `postproc/figures/loop_closure_allan.png`.

This closes the loop: the noise we *characterized* from synthetic data is the noise the
*compiled simulator* actually emits. White noise (the dominant, τ=1 s parameter) closes
tightly; bias instability is bounded by the available flight duration.

---

## Files

| File | Role |
|---|---|
| `noise_model.py` | Per-sample IMU error model (Python twin of the C++ `Imu`) + analytic Allan forms |
| `generate_imu_data.py` | Generate the 4 h static record + Monte-Carlo ground-truth labels → `imu_raw.npz` |
| `allan_variance.py` | Overlapping Allan deviation, NLS regime fit, annotated figure |
| `characterize.py` | Shared multi-axis characterization helper (used by fit + loop-closure + tests) |
| `fit_noise_params.py` | Recover params, print comparison, write `configs/sensor_params.json` |
| `figures/allan_deviation.png` | The characterization plot |

Run order: `python sensors/generate_imu_data.py && python sensors/fit_noise_params.py`.
Everything is seeded (`numpy.random.default_rng`) and deterministic.
