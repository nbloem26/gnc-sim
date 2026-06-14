# MATLAB / Octave analysis parity

Reference scripts that reproduce the headline gnc-sim post-processing figures in
MATLAB (and, where easy, GNU Octave). They are a **language-neutral parity layer**:
they read the **same CSV + JSON data contract** the Python suite reads
(see [`../docs/DATA_CONTRACT.md`](../docs/DATA_CONTRACT.md) §3–4), so a MATLAB user
can load any run the native CLI produced and regenerate the trajectory and
Allan-deviation plots without touching Python.

> **Note — not run in CI.** MATLAB cannot be executed in this repository's CI, so
> these scripts are **reference/parity** code, validated by inspection and by
> matching the Python implementations they mirror. The authoritative, tested
> pipeline is the Python suite under [`../postproc`](../postproc) and
> [`../sensors`](../sensors); these scripts exist so the analysis is reproducible
> in a MATLAB shop. They are written to be Octave-compatible where it costs
> nothing (the only MATLAB-only function used is `readtable`, with an Octave
> `dlmread` fallback in `load_run.m`).

## Scripts

| Script | Mirrors (Python) | What it does |
|---|---|---|
| `load_run.m` | `gncpost.loaders.load_run` | Load a `runs/<id>/` folder — `vehicle/target/gnc/sensors`(+`track`/`discrim`/`summary`) CSVs and `manifest.json` — into a struct. |
| `getcol.m` | (helper) | Fetch a named column from a loaded frame, portably across MATLAB `table` and the Octave struct-of-columns fallback. |
| `validation_plot.m` | `gncpost.plots.plot_trajectory` + `plot_states` | Trajectory (East/Up) plus speed-vs-time and altitude-vs-time for a run. |
| `allan_deviation.m` | `sensors.allan_variance.overlapping_allan_deviation` + `plot_allan` | Overlapping Allan deviation of a signal, with the −1/2 (white) and +1/2 (RRW) slope guide lines and a bias-instability marker. |

## Requirements

- **MATLAB** R2019b or newer (uses `readtable`, `jsondecode`, `print`), **or**
- **GNU Octave** 6+ (uses the `dlmread` fallback in `load_run.m`). `jsondecode` is
  available in Octave 7+; on older Octave install the `io` package for JSON.

No toolboxes are required — everything uses base MATLAB / Octave.

## Running

From the repository root, add this folder to the path and load a run:

```matlab
addpath('matlab');

% 1. Load a run the native CLI produced (see the repo README for how to run it):
%    ./build-native/apps/cli/gncsim --config configs/homing_3dof.json --out runs/myrun
run = load_run('runs/sample_run');
fprintf('miss = %.3f m, intercept = %d, frames = %d\n', ...
        run.miss_distance, run.intercept, numel(run.t));

% 2. Trajectory + speed + altitude (optionally save a PNG):
validation_plot(run, 'trajectory_matlab.png');

% 3. Allan deviation of the in-sim measured IMU accel channel:
dt = run.manifest.dt;
a  = getcol(run.sensors, 'imu_accel_meas_x');
allan_deviation(a, dt, 'Label', 'accel meas', 'OutPath', 'allan_matlab.png');
```

In **Octave** the workflow is identical; `load_run` returns a struct of column
vectors instead of `table`s, and `getcol` handles both transparently.

## Data contract (the contract these scripts honour)

All telemetry is SI, local **ENU** (x=East, y=North, z=Up). Column names per CSV:

| File | Columns |
|---|---|
| `vehicle.csv` | `t,x,y,z,vx,vy,vz,roll,pitch,yaw,mass,mach,thrust` |
| `target.csv` | `t,x,y,z,vx,vy,vz` |
| `gnc.csv` | `t,accel_cmd_x,accel_cmd_y,accel_cmd_z,fin_x,fin_y,fin_z,los_angle,los_rate,v_closing,range,nav_x,nav_y,nav_z,nav_nis` |
| `sensors.csv` | `t,imu_accel_true_x,imu_accel_meas_x,imu_gyro_true_x,imu_gyro_meas_x,seeker_los_true,seeker_los_meas` |
| `track.csv` | `t,track_x,track_y,track_z,tgt_x,tgt_y,tgt_z,track_nis` (zero unless `trackers.enabled`) |
| `discrim.csv` | `t,selected_obj,discrim_correct,discrim_margin` (inert unless `decoys.enabled`) |

`manifest.json` carries the run metadata (`scenario`, `model`, `seed`, `dt`,
`t_end`, `intercept`, `miss_distance`, `intercept_time`, `launch_time`,
`num_frames`, `origin`) plus a verbatim echo of the input config under `config`.
A Monte-Carlo batch additionally writes `summary.csv`
(`case,seed,miss_distance,intercept_time,intercept`).

If the C++ data contract changes, update both the Python suite **and** these
scripts so the parity holds.
