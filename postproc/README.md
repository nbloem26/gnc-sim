# postproc — Python Post-Processing & Validation

Reads the gnc-sim native CLI's output (`docs/DATA_CONTRACT.md`), validates the C++ core
against analytic solutions, runs the sensor loop-closure, and produces the portfolio
figures the web app displays.

## Layout

```
postproc/
  gncpost/                 # the package
    loaders.py             # load a run folder's CSVs + manifest; invoke the native CLI
    atmosphere.py          # USSA76 twin of the C++ model (validation reference)
    plots.py               # trajectory / states / guidance / miss-distance figures
    validate.py            # 4 analytic checks (ballistic, v_term, USSA76, PN intercept)
    montecarlo.py          # CEP / dispersion stats from a summary.csv
    loop_closure.py        # re-derive IMU noise from sim output (fidelity proof)
  notebooks/               # 01 single run · 02 validation · 03 Monte-Carlo / CEP
  tests/                   # pytest: loaders, atmosphere/v_term math, Allan recovery, loop-closure
  make_figures.py          # regenerate all figures and copy the key ones to web/public/figures/
  figures/                 # generated PNGs (committed)
```

## Quick start (from repo root)

```bash
python3 -m venv .venv && . .venv/bin/activate && pip install -r postproc/requirements.txt

# Analytic validation (runs the native CLI):
python -m gncpost.validate                  # from inside postproc/, or:
PYTHONPATH=postproc python -m gncpost.validate

# Sensor loop-closure (fidelity proof):
PYTHONPATH=postproc:sensors python -m gncpost.loop_closure

# Tests:
python -m pytest postproc/tests -q

# Regenerate every figure + copy to web/public/figures/:
python postproc/make_figures.py
```

## What the validation proves

| Check | Compares against | Result |
|---|---|---|
| `ballistic_parabola` | analytic projectile `x=v_x t`, `z=v_z t − ½gt²` (atmosphere off) | sub-mm |
| `terminal_velocity` | `v_term = √(2mg/(ρ C_d A))` with USSA76 density | < 1 % |
| `ussa76_table` | published USSA76 table (density/temperature, 0–47 km) | < 5 % |
| `pronav_intercept` | noise-free proportional navigation | miss < 1 m |

## The fidelity loop

`sensors/` characterizes a synthetic IMU with Allan variance and writes
`configs/sensor_params.json`. The C++ `Imu` consumes it; `loop_closure.py` re-derives the
noise from the simulator's own `sensors.csv` and confirms the white-noise density matches
the config to ~0.1 %. See `sensors/README.md` for the full methodology.
