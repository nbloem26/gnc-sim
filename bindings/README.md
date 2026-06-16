# gnc-sim Python SDK (`gncsim`)

Run the deterministic C++ guided-interceptor core directly from Python, with results returned
as **numpy arrays** — no CSV/JSON round-trip. The package is a thin pybind11 wrapper over the
compiled `_gncsim` extension; **all** numerics live in the pure C++ core
([`core/`](../core/)), the exact same engine the native CLI (`apps/cli/`) and the WebAssembly
web build (`apps/wasm/`) run. Results from `run()` match the CLI / golden values **bit-for-bit**
for the same config + seed.

This is the hand-written API reference. The authoritative docstrings live in
[`gncsim/__init__.py`](gncsim/__init__.py) (`help(gncsim.run)` in a REPL); the math behind the
models is in [`docs/THEORY.md`](../docs/THEORY.md); the config schema is
[`docs/DATA_CONTRACT.md`](../docs/DATA_CONTRACT.md).

## Install / build

The extension is built behind an opt-in CMake flag and emitted into `bindings/gncsim/` next to
the Python package:

```bash
pip install pybind11 numpy            # (already in postproc/requirements.txt)
./scripts/build-python.sh             # builds the _gncsim extension
export PYTHONPATH="$PWD/bindings:${PYTHONPATH:-}"
python -c "import gncsim; print(gncsim.run({'scenario': 'homing'}).keys())"
```

The pytest suite (`bindings/test_sdk.py`) adds the `PYTHONPATH` entry via `conftest.py` and
asserts the SDK matches the CLI bit-for-bit (set `GNCSIM_CLI` to the built CLI path).

## API

### `gncsim.run(cfg) -> dict`

Run one engagement to completion.

- **`cfg`** — a config `dict` (same shape as `configs/*.json`) **or** a JSON string. Both go
  through the same `loadConfigFromString` parser the CLI and WASM entry use, so there is one
  schema, not a fork. Missing keys fall back to the core defaults.

**Returns** a dict of scalar metadata plus a `series` dict:

| Key | Type | Meaning |
|---|---|---|
| `scenario`, `model` | str | resolved scenario / dynamics model |
| `seed` | int | RNG seed used |
| `dt`, `t_end` | float | step size [s], end time [s] |
| `intercept` | bool | miss < lethal radius (3 m) |
| `miss_distance` | float | analytic CPA miss [m] |
| `intercept_time`, `launch_time` | float | [s] |
| `git_sha` | str | build provenance |
| `origin` | dict | geodetic origin (`lat0_deg`, `lon0_deg`, `alt0_m`) |
| `series` | dict[str, np.ndarray] | per-frame channels, `float64` |

The `series` channel names are **identical** to the columnar JSON the web app consumes and the
CSV headers (`docs/DATA_CONTRACT.md`): `t`, `veh_x/y/z`, `veh_vx/vy/vz`, `roll/pitch/yaw`,
`mass`, `mach`, `thrust`, `tgt_x/y/z`, `tgt_vx/vy/vz`, `accel_cmd_x/y/z`, `los_angle`,
`los_rate`, `v_closing`, `range`, `nav_x/y/z`, `nav_nis`, `track_x/y/z`, `track_nis`,
`selected_obj`, `discrim_correct`, `discrim_margin`, `imu_accel_*`, `imu_gyro_*`,
`seeker_los_*`. So a plot script written against the SDK works unchanged on a CLI run's CSV.

```python
import gncsim
res = gncsim.run({"scenario": "homing", "model": "3dof", "seed": 1})
print(res["miss_distance"], res["intercept"])
print(res["series"]["range"].min())          # numpy float64 array
```

### `gncsim.monte_carlo(cfg, n=0, workers=1) -> dict`

Run a dispersed Monte Carlo batch. Initial-condition dispersion lives in the C++ core
([`MonteCarlo.cpp`](../core/src/scenario/MonteCarlo.cpp)), driven by the `monte_carlo` sigma
block in `cfg` and deterministic given the seed.

- **`cfg`** — config dict / JSON string; its `monte_carlo` block supplies the dispersion sigmas.
- **`n`** — number of cases; when `> 0` overrides `cfg.monte_carlo.num_cases`.
- **`workers`** — C++ worker threads across which the cases are distributed (issue #43).
  `workers <= 1` runs serially. The batch is **bit-identical to the serial run** for the same
  seed + N regardless of `workers` — each case's RNG stream is derived up-front in case order,
  so results never depend on thread scheduling.

**Returns** summary scalars (`num_cases`, `intercepts`, `p_kill`) plus columnar numpy arrays,
one entry per case: `index`, `seed`, `miss_distance`, `intercept_time`, `intercept`.

```python
batch = gncsim.monte_carlo({"scenario": "homing",
                            "monte_carlo": {"num_cases": 1000}}, n=1000, workers=8)
print(batch["p_kill"], batch["intercepts"])
```

## Determinism contract

`run(cfg)` is a pure function of `cfg` (config + seed): same input → bit-identical output,
across native CLI, WASM, and this SDK (a parity guarantee enforced in CI). `monte_carlo` is
bit-identical regardless of `workers`. Do not rely on iteration order of the returned dict
beyond the documented keys; do rely on the numeric values.
