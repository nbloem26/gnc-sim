# Data Contract

This is the single source of truth for the interfaces between the C++ core, the native CLI, the
Python post-processing, and the Next.js web app. **All four must agree on the field names below.**
The in-code authority is [`core/include/gncsim/core/Types.hpp`](../core/include/gncsim/core/Types.hpp)
(the `Frame` struct) and [`Config.hpp`](../core/include/gncsim/core/Config.hpp) (`SimConfig`).

Units: SI throughout — metres, m/s, m/s², radians, kg, seconds, kelvin, pascal. Frame is local
**ENU** (x=East, y=North, z=Up) about a geodetic origin recorded in the manifest.

---

## 1. Input — config JSON (`configs/*.json`)

Consumed by `loadConfigFromString()`. Missing keys fall back to documented defaults, so a minimal
config is valid. Shape (defaults shown):

```jsonc
{
  "scenario": "homing",
  "model": "3dof",                 // "3dof" | "6dof" | "6dof_hifi"  (hi-fi: issue #35)
  "seed": 1,
  "dt": 0.005,                      // integration step [s]
  "t_end": 60.0,                    // max sim time [s]
  "integrator": "rk4",             // "euler" | "rk2" | "rk4"
  "origin": { "lat0_deg": 28.4889, "lon0_deg": -80.5778, "alt0_m": 0.0 },
  "env":   { "g0": 9.80665, "altitude_dependent_g": false, "atmosphere": true,
             "frame": "flat", "j2": false },   // frame: "flat" (default) | "round" (WGS-84/ECI)
  "aero":  { "ref_area": 0.05, "cd0": 0.3,
             "cd_mach": [[0.0,0.28],[0.8,0.30],[1.0,0.55],[1.2,0.48],[2.0,0.34],[4.0,0.27]],
             "cn_alpha": 12.0,
             // --- 6dof_hifi only (issue #35): reference length + Cn/Cm tables + damping ---
             "ref_length": 0.15,            // moment reference length (diameter) [m]
             "cn_table": [],                // (alpha_rad, mach, Cn) rows; supersedes cn_alpha
             "cm_table": [],                // (alpha_rad, mach, Cm) rows; supersedes cm_alpha (<0 stable)
             "cm_alpha": -8.0,              // linear pitch-moment slope [1/rad] (cm_table fallback)
             "cm_q": -120.0,                // pitch/yaw rate damping Cmq [1/rad]
             "cl_p": -1.0 },                // roll rate damping Clp [1/rad]
  "vehicle": { "pos0": [0,0,0], "launch_speed": 600, "launch_elevation_deg": 45,
               "launch_azimuth_deg": 0, "mass0": 22.0, "inertia": 1.2,
               // --- 6dof_hifi only: full inertia tensor; principal moment <=0 => use `inertia` ---
               "ixx": -1.0, "iyy": -1.0, "izz": -1.0, "ixy": 0.0, "ixz": 0.0, "iyz": 0.0 },
  "propulsion": { "thrust": 0.0, "burn_time": 0.0, "propellant_mass": 0.0,
                  "boost_ref_area": 0.0, "stage_time": 0.0, "stage_mass_drop": 0.0 },
  "guidance": { "law": "pronav", "nav_constant": 3.0, "max_accel": 300.0 },
  "control":  { "kp": 8.0, "kd": 2.5, "max_fin_deflection": 0.35 },   // 6DOF only
  "actuator": { "tau": 0.02, "rate_limit": 6.0, "deflection_limit": 0.35,
                "effectiveness": 60.0 },   // 6dof_hifi fin actuators (issue #35)
  "sensors":  { "enable": false, "imu": { ...see sensor_params.json... },
                "seeker": { "los_white": 0.0, "los_bias": 0.0, "glint": 0.0 } },
  "nav":      { "filter": "alpha_beta",       // "alpha_beta" | "ekf" | "imm" navigation filter
                "process_accel_psd": 50.0,    // EKF target-accel PSD q [m^2/s^3] per axis
                "range_white": 5.0,           // EKF/IMM range measurement noise std [m]
                "imm_q_cv": 0.5,              // IMM constant-velocity model process-accel PSD [m^2/s^3]
                "imm_q_man": 3000.0,          // IMM maneuver model process-accel PSD [m^2/s^3]
                "imm_p_stay": 0.999 },        // IMM Markov mode self-transition prob (stickiness)
  "target":   { "pos0": [8000,0,3000], "vel0": [-250,0,0],
                "maneuver": "constant", "maneuver_g": 3.0, "maneuver_freq": 0.4 },
  "trackers": { "enabled": false, "process_psd": 50.0, "sensors": [] },   // multi-sensor fusion
  "decoys":   { "enabled": false, "count": 0, "separation": 50.0, "separability": 1.0,
                "target_intensity": 1.0, "target_size": 1.0, "target_decel": 1.0,
                "decoy_intensity": 0.3, "decoy_size": 0.4, "decoy_decel": 3.0,
                "feature_spread": 0.10, "measurement_noise": 0.08, "score_filter_tau": 0.5 },
  "cueing":   { "enabled": false, "launch_criterion": "track_cov", "cov_trace_threshold": 1.0e4,
                "max_cue_time": 10.0, "loft_deg": 0.0 },   // launch-on-track (needs trackers)
  "monte_carlo": { "num_cases": 0, "launch_speed_sigma": 0.0,
                   "launch_elevation_sigma_deg": 0.0, "target_pos_sigma": 0.0 }
}
```

The `trackers` block (default `enabled:false`) is the opt-in **multi-sensor target-track fusion**
path. When enabled, the Runner builds a `TargetTrackEkf` (absolute 6-state target track in ENU,
nearly-constant-velocity, process-noise PSD `process_psd` [m²/s³ per axis]) and fuses each fixed
sensor's measurement **sequentially** (Joseph form, per-sensor H/R, per-update NIS) into one estimate
of the target's absolute state. Guidance then uses `track_est − vehicle` instead of the seeker.
Each `sensors[]` entry:

```jsonc
{ "type": "radar",        // "radar" (az,el,range,range_rate) | "ir" (angles-only az,el)
  "pos": [0,0,0],         // fixed sensor location, ENU [m] (ground site, space platform, ...)
  "sigma_az": 0.002,      // azimuth noise std [rad]    (radar + ir)
  "sigma_el": 0.002,      // elevation noise std [rad]  (radar + ir)
  "sigma_range": 25.0,    // range noise std [m]         (radar only)
  "sigma_range_rate": 3.0 // range-rate noise std [m/s]  (radar only)
}
```

When `trackers.enabled` is false the new path is never entered: the default RNG draw order, the
trajectory, and all four legacy CSVs are byte-identical, and the track channel (below) is zero.
Sample configs: `configs/track_radar_only.json`, `track_ir_only.json`, `track_fused.json` (the same
engagement differing only in `trackers.sensors`). Demonstration: `postproc/gncpost/fusion.py`
computes the RMS track error for each and shows fusion < radar-only < IR-only.

The `decoys` block (default `enabled:false`) is the opt-in **seeker decoy / closely-spaced-object
discrimination** path (issue #6). When enabled, the Runner places `count` decoys in a Gaussian
cluster of std `separation` [m] about the true target, gives every object a 3-feature SIGNATURE
(`[intensity, size, decel]` — IR brightness, apparent size, ballistic deceleration; a light decoy
sheds speed faster), and builds a `Discriminator`. Each step the seeker measures every object's
features noisily (`measurement_noise` std), the discriminator scores each against the expected
lethal-target signature (inverse-variance-weighted squared distance), integrates the scores over
time (`score_filter_tau`), and SELECTS the most target-like object. **Guidance homes on the SELECTED
object**, while the miss / CPA is always scored against the TRUE target — so a wrong selection
produces a large miss. `separability` (0..1) controls how distinct the decoy feature distributions
are from the target: 1 = obviously different (easy), 0 = statistically indistinguishable (hard). The
true target is index 0 by convention. The decoy path uses RNG only inside its own branch, so the
default (decoys disabled) RNG draw order, trajectory, and all CSVs are byte-identical, and the
discrimination channel (below) is inert. Sample config: `configs/discrimination.json`. Demonstration:
`postproc/gncpost/discrimination.py` sweeps decoy count + separability over a small Monte-Carlo set
and plots discrimination accuracy and Pk vs each.

The `cueing` block (default `enabled:false`) is the opt-in **interceptor cueing / launch-on-track**
path (issue #8). It **requires `trackers.enabled`** — the fused multi-sensor track is what cues the
launch. When enabled, the engagement runs in two phases within one run. **Phase 1 (pre-launch):** the
interceptor is held stationary at its launch site (`vehicle.pos0`) while the `TargetTrackEkf` fuses
the sensor measurements on the incoming threat; the threat keeps closing. **Launch:** when the
criterion fires the cue/launch time is recorded and the interceptor velocity is set to `launch_speed`
aimed by a constant-velocity **lead/intercept** solution from the current track estimate — solve
`|r0 + v_tgt·t| = launch_speed·t` (smallest positive root) for the predicted intercept point, aim
there, then add `loft_deg` to the elevation. **Phase 2 (post-launch):** normal terminal homing
(PN/APN) off the track exactly as before. The `launch_criterion` is `"track_cov"` (launch once the
fused-track position+velocity covariance trace drops below `cov_trace_threshold` [m²], i.e. a
confident track) or `"fixed_delay"` (launch at `max_cue_time`); either way `max_cue_time` is a hard
timeout. The cueing path draws RNG only inside its own (tracker) branch, so the default (cueing
disabled) launches at `t=0` exactly as before and the trajectory + all CSVs are byte-identical.
Sample configs: `configs/engagement_cued.json` (a single cued launch-on-track engagement) and
`configs/engagement_pkill.json` (the dispersed Monte-Carlo variant used by the P(kill) campaign).
Demonstration: `postproc/gncpost/pkill.py` runs an engagement Monte-Carlo grid over threat
`maneuver_g` and `launch_delay`, computes P(kill) per cell, and plots the P(kill) surface + marginal
curves (`postproc/figures/pkill.png`).

The `model: "6dof_hifi"` value selects the opt-in **high-fidelity 6DOF** dynamics (issue #35): the
full 3×3 inertia tensor (`vehicle.ixx…iyz`) with the gyroscopic coupling `−ω×(Iω)` in the rotational
EOM, table-interpolated normal-force and pitch/yaw moment coefficients (`aero.cn_table` /
`aero.cm_table` over angle-of-attack and Mach, with `cn_alpha`/`cm_alpha` linear fallbacks), aero
rate damping (`cm_q`, `cl_p`), and first-order **fin-actuator dynamics** with rate/travel limits
(`actuator` block) driven by a moment→deflection control allocation. A principal moment `≤ 0` falls
back to the scalar `inertia` proxy, so a minimal hi-fi config is a uniform-inertia body. The legacy
`"3dof"` and `"6dof"` models are untouched (byte-identical). Sample config:
`configs/homing_6dof_hifi.json`. This path uses no RNG draws beyond the legacy seeker/IMU ones, so
native↔WASM parity is preserved.

The `sensors.imu` block matches `configs/sensor_params.json`, which is **generated by the Python
Allan-variance pipeline** (`sensors/`) and fed back into the sim — see §5.

---

## 2. Output — result JSON (WASM, for the web app)

`toJsonString(SimResult)` returns **columnar** data (arrays per channel — Plotly-friendly):

```jsonc
{
  "scenario": "homing", "model": "3dof", "seed": 1, "dt": 0.005, "t_end": 60,
  "intercept": true, "miss_distance": 1.83, "intercept_time": 9.42, "launch_time": 0.0,
  "git_sha": "...",
  "origin": { "lat0_deg": 28.4889, "lon0_deg": -80.5778, "alt0_m": 0.0 },
  "series": {
    "t": [...],
    "veh_x": [...], "veh_y": [...], "veh_z": [...],
    "veh_vx": [...], "veh_vy": [...], "veh_vz": [...],
    "roll": [...], "pitch": [...], "yaw": [...],
    "mass": [...], "mach": [...], "thrust": [...],
    "tgt_x": [...], "tgt_y": [...], "tgt_z": [...],
    "tgt_vx": [...], "tgt_vy": [...], "tgt_vz": [...],
    "accel_cmd_x": [...], "accel_cmd_y": [...], "accel_cmd_z": [...],
    "los_angle": [...], "los_rate": [...], "v_closing": [...], "range": [...],
    "nav_x": [...], "nav_y": [...], "nav_z": [...], "nav_nis": [...],
    "track_x": [...], "track_y": [...], "track_z": [...], "track_nis": [...],
    "selected_obj": [...], "discrim_correct": [...], "discrim_margin": [...],
    "imu_accel_true_x": [...], "imu_accel_meas_x": [...],
    "imu_gyro_true_x": [...], "imu_gyro_meas_x": [...],
    "seeker_los_true": [...], "seeker_los_meas": [...]
  }
}
```

`nav_nis` is the navigation filter's normalized innovation squared (chi-square, dof = 3) each step.
It is `0` on the default alpha-beta path; on the EKF path (`nav.filter == "ekf"`) its mean should sit
near 3 for a consistent filter. On the IMM path (`nav.filter == "imm"`) the same column carries the
mode-probability-weighted *combined* NIS across the model bank — for a maneuvering target this stays
near 3 where a single CV EKF's NIS spikes during the maneuver. No new column is added; the IMM reuses
the `nav_nis` channel. On error the WASM entry returns `{"error": "<message>"}`.

`track_x/y/z` are the fused **absolute target-position** estimate [m] (ENU) from the multi-sensor
target-track fusion path, and `track_nis` is the last fused sensor update's NIS. All four are `0` on
every non-tracker path (the default). They are populated only when `trackers.enabled` is true.

`selected_obj` is the index of the object the seeker homed on each step (0 = the true target by
convention), `discrim_correct` is `1` when the true target was selected and `0` otherwise, and
`discrim_margin` is the integrated-score gap to the runner-up (selection confidence). On every
non-decoy path (the default) these are `0` / `1` / `0` respectively — i.e. always "selected the true
target". They are meaningful only when `decoys.enabled` is true.

`launch_time` is the interceptor cue/launch time [s] (issue #8). It is `0.0` on the default
launch-at-`t=0` path and `> 0` on the cued launch-on-track path, where it marks the end of Phase 1
(when the launch criterion fired). The series carry no extra column for cueing: during Phase 1 the
interceptor is simply logged at its (stationary) launch site, so the existing `veh_*` channels show
it parked, then flying after `launch_time`.

`thrust` is the boost-motor thrust magnitude [N] applied each step (along velocity in 3DOF, along
the body nose in 6DOF). It is `0` on the default unpowered path (`propulsion.thrust == 0`) and
during/after the burn window otherwise. The `propulsion` block (defaults all `0`) adds a boost
phase: linear propellant burn over `burn_time`, an optional larger `boost_ref_area` booster drag
area during the boost window, and optional booster jettison (`stage_time`, `stage_mass_drop`).

---

## 3. Output — CSV files (native CLI, for Python / Monte Carlo)

`toCsvFiles(SimResult)` writes one folder per run, `runs/<id>/`:

| File | Columns |
|---|---|
| `vehicle.csv` | `t,x,y,z,vx,vy,vz,roll,pitch,yaw,mass,mach,thrust` |
| `target.csv`  | `t,x,y,z,vx,vy,vz` |
| `gnc.csv`     | `t,accel_cmd_x,accel_cmd_y,accel_cmd_z,fin_x,fin_y,fin_z,los_angle,los_rate,v_closing,range,nav_x,nav_y,nav_z,nav_nis` |
| `sensors.csv` | `t,imu_accel_true_x,imu_accel_meas_x,imu_gyro_true_x,imu_gyro_meas_x,seeker_los_true,seeker_los_meas` |
| `track.csv`   | `t,track_x,track_y,track_z,tgt_x,tgt_y,tgt_z,track_nis` |
| `discrim.csv` | `t,selected_obj,discrim_correct,discrim_margin` |

All files share the `t` column (same length, same timestamps). `track.csv` carries the fused
absolute target-track estimate vs the true target position (issue #5); its `track_*` / `track_nis`
columns are `0` on every non-tracker path, so it is always present but zero by default. `discrim.csv`
carries the seeker decoy-discrimination channel (issue #6): the homed-on object index, whether it was
the true target, and the score margin. On every non-decoy path it is `0` / `1` / `0`, so it too is
always present but inert by default.

---

## 4. `manifest.json` (native, per run)

Run metadata + an echo of the input config:

```jsonc
{
  "scenario": "...", "model": "3dof", "seed": 1, "dt": 0.005, "t_end": 60,
  "intercept": true, "miss_distance": 1.83, "intercept_time": 9.42, "launch_time": 0.0,
  "git_sha": "...", "origin": {...}, "num_frames": 1884,
  "config": { ...verbatim input config... }
}
```

### Monte Carlo batches
`runs/<batch>/case_NNNN/` per case, plus `runs/<batch>/summary.csv` with one row per case:
`case,seed,miss_distance,intercept_time,intercept`.

---

## 5. Sensor-fidelity loop

1. Python generates synthetic IMU data → Allan variance → recovered noise params.
2. Params written to `configs/sensor_params.json` (schema = the `sensors.imu`/`sensors.seeker`
   blocks above).
3. C++ `Imu`/`Seeker` consume those params and reproduce the error characteristics in-sim.
4. Re-running Allan variance on `sensors.csv` (`*_meas_*` columns) must recover the same params —
   the loop-closure validation.

---

## 6. ENU → geodetic (web map only)

The C++ core stays in local ENU metres. The web app (`web/lib/enuToGeodetic.ts`) projects ENU to
lat/lon about `origin` (local tangent plane) purely for the Leaflet ground track. No geodesy in C++.

### Round-Earth mode (`env.frame == "round"`)

Opt-in via `env.frame: "round"` (default `"flat"`). In round mode the translational state is
propagated in **ECI** (Earth-Centred Inertial; coincident with ECEF at t=0, GMST=0), under WGS-84
central point-mass gravity (`-GM/r²·r̂`) plus an optional J2 oblateness term (`env.j2`). Earth's
rotation enters only kinematically (ECI→ECEF rotation by ω·t) when sampling the atmosphere and
emitting output. All telemetry (CSV/JSON `veh_*`/`tgt_*`) is still converted back to **local ENU
about `origin`**, so the data contract, engagement geometry, and the web map are unchanged. The
geodesy lives in `core/src/env/Frames.cpp` (only reached in round mode); the flat-Earth path is
byte-identical to before. Round mode targets the unguided ballistic launch-engagement case — no
guidance/sensors/EKF are applied on this path.

#### High-fidelity environment (issue #41)

Opt-in, additive extensions on the round path (`core/src/env/EnvFidelity.cpp`). All default off, so
flat mode and the existing round mode are byte-identical. New `env` keys:

```jsonc
"env": {
  "frame": "round",
  "gravity_model": "egm",          // "central" (default) | "egm" — truncated zonal harmonics
  "gravity": { "j2": true, "j3": true, "j4": true },  // zonals when gravity_model == "egm"
  "atmosphere_model": "extended",  // "ussa76" (default, <=86 km) | "extended" (adds 86..1000 km)
  "rotating_ecef": false,          // integrate in rotating ECEF (Coriolis+centrifugal) vs ECI
  "wind": {                        // parameterized horizontal wind for drag (ENU, sheared profile)
    "enabled": true, "surface_mps": 5.0, "jet_mps": 35.0,
    "jet_alt_m": 11000.0, "decay_scale_m": 9000.0, "dir_deg": 90.0
  }
}
```

- **Gravity (`gravity_model: "egm"`)** — central point-mass plus the selected zonal harmonics J2,
  J3, J4 (unnormalized WGS-84/EGM-96 coefficients, cited in `EnvFidelity.cpp`). With only J2 it is
  bit-identical to the legacy central+J2 model. The legacy `env.j2` flag still works and also turns
  on J2 for the EGM model.
- **Atmosphere (`atmosphere_model: "extended"`)** — USSA76 unchanged below 86 km, then a
  log-linear (continuous, monotone) interpolation of the US Standard Atmosphere 1976 upper tables up
  to ~1000 km, with the 86 km handover density pinned to USSA76. **Reduced model:** no
  solar-activity (F10.7), geomagnetic (Ap), or diurnal/seasonal variation — that is a follow-up
  (full NRLMSISE-00 port).
- **`rotating_ecef`** — alternative formulation that integrates directly in the rotating ECEF frame
  with explicit Coriolis (`-2 ω×v`) and centrifugal (`-ω×(ω×r)`) terms. Physically equivalent to the
  ECI path (agrees to integrator precision).
- **`wind`** — a horizontal ENU wind that shears linearly from `surface_mps` up to `jet_mps` at
  `jet_alt_m`, then decays exponentially above (scale `decay_scale_m`), blowing toward `dir_deg`
  (East→North). Drag is computed against the air-relative velocity.

No telemetry columns are added; the CSV/JSON schema is unchanged. Example config:
`configs/ballistic_round_hifi.json`.
