"""Author the three portfolio notebooks as .ipynb JSON, then they are executed in place.

Run from repo root:  python postproc/notebooks/_build_notebooks.py
This only writes the notebook source; execution is done separately via nbconvert so the
outputs (figures, tables) are embedded.
"""

from __future__ import annotations

import json
from pathlib import Path

NB_DIR = Path(__file__).resolve().parent


def md(text: str) -> dict:
    return {"cell_type": "markdown", "metadata": {}, "source": text.splitlines(keepends=True)}


def code(text: str) -> dict:
    return {
        "cell_type": "code",
        "execution_count": None,
        "metadata": {},
        "outputs": [],
        "source": text.splitlines(keepends=True),
    }


def notebook(cells: list[dict]) -> dict:
    return {
        "cells": cells,
        "metadata": {
            "kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"},
            "language_info": {"name": "python"},
        },
        "nbformat": 4,
        "nbformat_minor": 5,
    }


# Common bootstrap: make gncpost + sensors importable regardless of CWD.
BOOTSTRAP = """import sys
from pathlib import Path

# Repo-relative imports (works whether run from repo root or notebooks dir).
REPO = Path.cwd()
while not (REPO / "build-native").exists() and REPO != REPO.parent:
    REPO = REPO.parent
sys.path.insert(0, str(REPO / "postproc"))
sys.path.insert(0, str(REPO / "sensors"))

import matplotlib.pyplot as plt
import numpy as np
%matplotlib inline
"""


# ---------------------------------------------------------------------------
# 01 — Single run
# ---------------------------------------------------------------------------
nb01 = notebook(
    [
        md(
            "# 01 — Single Run\n\n"
            "Load `runs/sample_run`, plot the trajectory, vehicle states, and guidance signals."
        ),
        code(BOOTSTRAP),
        code(
            "from gncpost.loaders import load_run\n"
            "from gncpost.plots import plot_trajectory, plot_states, plot_guidance\n\n"
            "run = load_run(REPO / 'runs' / 'sample_run')\n"
            "print('scenario:', run.manifest.get('scenario'), '| model:', run.manifest.get('model'))\n"
            "print(f'intercept: {run.intercept} | miss: {run.miss_distance:.3f} m '\n"
            "      f\"| t_intercept: {run.manifest.get('intercept_time'):.2f} s\")\n"
            "run.vehicle.head()"
        ),
        md(
            "## Trajectory (East-Up)\nVehicle vs target ground track; the X marks the target endpoint."
        ),
        code("fig = plot_trajectory(run, mode='east_up'); plt.show()"),
        md("## Vehicle states vs time\nPosition, velocity, speed and Mach."),
        code("fig = plot_states(run); plt.show()"),
        md(
            "## Guidance\nLine-of-sight angle/rate, range & closing velocity, and the "
            "acceleration-command magnitude. A well-tuned proportional-navigation loop drives the "
            "LOS rate toward zero as range collapses."
        ),
        code("fig = plot_guidance(run); plt.show()"),
    ]
)


# ---------------------------------------------------------------------------
# 02 — Validation
# ---------------------------------------------------------------------------
nb02 = notebook(
    [
        md(
            "# 02 — Analytic Validation\n\n"
            "Run the analytic validation harness (it invokes the native CLI) and show a pass/fail "
            "table plus the supporting plots:\n\n"
            "1. **Drag-free ballistic** vs analytic parabola\n"
            "2. **Terminal velocity** vs $v_\\\\mathrm{term}=\\\\sqrt{2mg/(\\\\rho C_d A)}$\n"
            "3. **USSA76 atmosphere** vs published table values\n"
            "4. **Noise-free proportional navigation** intercept (miss < 1 m)"
        ),
        code(BOOTSTRAP),
        code(
            "import pandas as pd\n"
            "from gncpost.validate import run_all\n\n"
            "results = run_all()\n"
            "df = pd.DataFrame([{ 'check': r.name, 'result': 'PASS' if r.passed else 'FAIL',\n"
            "                     'metric': r.metric, 'tolerance': r.tolerance,\n"
            "                     'detail': r.detail } for r in results])\n"
            "df"
        ),
        code(
            "assert all(r.passed for r in results), 'a validation check failed'\n"
            "print('All analytic validation checks PASSED')"
        ),
        md(
            "## USSA76 density & temperature profile\n"
            "The Python twin of the C++ atmosphere, overlaid with the published reference points "
            "used in the table check."
        ),
        code(
            "from gncpost.atmosphere import ussa76, REFERENCE_TABLE\n\n"
            "alts = np.linspace(0, 80000, 400)\n"
            "rho = [ussa76(a).density for a in alts]\n"
            "T = [ussa76(a).temperature for a in alts]\n"
            "fig, (a0, a1) = plt.subplots(1, 2, figsize=(11, 4))\n"
            "a0.semilogy(rho, alts/1000, color='#1f77b4'); a0.set_xlabel('density [kg/m^3]')\n"
            "a0.set_ylabel('altitude [km]'); a0.set_title('USSA76 density'); a0.grid(alpha=0.3)\n"
            "a0.scatter([REFERENCE_TABLE[a][1] for a in REFERENCE_TABLE],\n"
            "           [a/1000 for a in REFERENCE_TABLE], color='#d62728', zorder=5, label='ref table')\n"
            "a0.legend()\n"
            "a1.plot(T, alts/1000, color='#2ca02c'); a1.set_xlabel('temperature [K]')\n"
            "a1.set_title('USSA76 temperature'); a1.grid(alpha=0.3)\n"
            "plt.tight_layout(); plt.show()"
        ),
        md(
            "## Terminal-velocity convergence\n"
            "A heavy body dropped from 2 km asymptotes to the closed-form terminal velocity."
        ),
        code(
            "import tempfile, json\n"
            "from pathlib import Path\n"
            "from gncpost.loaders import run_cli, load_run\n"
            "from gncpost.atmosphere import terminal_velocity\n\n"
            "cfg = { 'scenario':'vt','model':'3dof','seed':1,'dt':0.002,'t_end':120.0,'integrator':'rk4',\n"
            "  'env':{'g0':9.80665,'altitude_dependent_g':False,'atmosphere':True},\n"
            "  'aero':{'ref_area':0.05,'cd_mach':[[0.0,1.0],[5.0,1.0]]},\n"
            "  'vehicle':{'pos0':[0,0,2000],'launch_speed':0.001,'launch_elevation_deg':-90,\n"
            "             'launch_azimuth_deg':0,'mass0':5.0},\n"
            "  'guidance':{'law':'none'},'sensors':{'enable':False},\n"
            "  'target':{'pos0':[9999999,0,0],'vel0':[0,0,0],'maneuver':'constant'} }\n"
            "with tempfile.TemporaryDirectory() as td:\n"
            "    p = Path(td)/'vt.json'; p.write_text(json.dumps(cfg))\n"
            "    out = run_cli(p, Path(td)/'out'); veh = load_run(out).vehicle\n"
            "vt = terminal_velocity(5.0,1.0,0.05, veh['z'].clip(lower=0).mean())\n"
            "fig, ax = plt.subplots(figsize=(8,4))\n"
            "ax.plot(veh['t'], -veh['vz'], color='#1f77b4', label='sim |vz|')\n"
            "ax.axhline(vt, color='#d62728', ls='--', label=f'analytic v_term ~ {vt:.1f} m/s')\n"
            "ax.set_xlabel('t [s]'); ax.set_ylabel('descent speed [m/s]'); ax.legend(); ax.grid(alpha=0.3)\n"
            "ax.set_title('Terminal velocity convergence'); plt.show()"
        ),
    ]
)


# ---------------------------------------------------------------------------
# 03 — Monte Carlo miss distance
# ---------------------------------------------------------------------------
nb03 = notebook(
    [
        md(
            "# 03 — Monte-Carlo Miss Distance & CEP\n\n"
            "Run (or load) a Monte-Carlo batch, compute the **CEP** (Circular Error Probable — the "
            "median miss distance), and tie the dispersion back to the characterized sensor noise "
            "from the Allan-variance pipeline (`configs/sensor_params.json`)."
        ),
        code(BOOTSTRAP),
        code(
            "import json\n"
            "from gncpost.loaders import run_cli, load_summary\n"
            "from gncpost.montecarlo import compute_stats, plot_cep\n\n"
            "batch = REPO / 'runs' / 'mc_batch'\n"
            "if not (batch / 'summary.csv').exists():\n"
            "    run_cli(REPO / 'configs' / 'montecarlo.json', batch, timeout=180)\n"
            "summary = load_summary(batch)\n"
            "stats = compute_stats(summary)\n"
            "print(stats.summary_line())"
        ),
        md(
            "## Miss-distance distribution + CEP\n"
            "The dashed line is the CEP (50% of impacts fall within it); the dotted line is the mean."
        ),
        code("fig = plot_cep(summary, stats); plt.show()"),
        md(
            "## Tie to the characterized sensor noise\n"
            "The Monte-Carlo runs use the IMU + seeker noise in `configs/sensor_params.json`, which "
            "was produced by the Allan-variance characterization in `sensors/`. The CEP below is the "
            "terminal-accuracy consequence of that noise budget."
        ),
        code(
            "imu = json.loads((REPO / 'configs' / 'sensor_params.json').read_text())['imu']\n"
            "print('IMU noise driving these results (from Allan characterization):')\n"
            "print(f\"  accel white (VRW): {imu['accel_white']:.3e} (m/s^2)/sqrt(Hz)\")\n"
            "print(f\"  accel bias instab (sigma_GM): {imu['accel_bias_instability']:.3e} m/s^2\")\n"
            "print(f\"  gyro  white (ARW): {imu['gyro_white']:.3e} (rad/s)/sqrt(Hz)\")\n"
            "print()\n"
            "print(f'  -> CEP = {stats.cep:.2f} m, P90 = {stats.p90:.2f} m over {stats.n} cases')"
        ),
    ]
)


def main():
    for name, nb in (
        ("01_single_run", nb01),
        ("02_validation", nb02),
        ("03_montecarlo_miss_distance", nb03),
    ):
        path = NB_DIR / f"{name}.ipynb"
        path.write_text(json.dumps(nb, indent=1))
        print(f"wrote {path.relative_to(NB_DIR.parent.parent)}")


if __name__ == "__main__":
    main()
