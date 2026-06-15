"""Sensor loop-closure — the fidelity proof.

Runs the native CLI with sensors enabled on a long, smooth trajectory, extracts the IMU
accelerometer noise residual from ``sensors.csv``, re-runs the SAME Allan-variance
characterization used to derive ``configs/sensor_params.json``, and confirms the in-sim
sensor reproduces the configured white noise (and bias instability where flight length
permits). This closes the loop:

    characterized noise (sensor_params.json)  ==  noise the compiled sim emits

The residual is ``imu_accel_meas_x - imu_accel_true_x*(1+accel_scale_factor)``, which
isolates the additive bias+white process (the scale-factor term is removed analytically
because the C++ applies it multiplicatively to the true signal).

Writes ``postproc/figures/loop_closure_allan.png``.
"""

from __future__ import annotations

import json
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from . import REPO_ROOT
from .loaders import load_run, run_cli

# Make the sensors/ Allan pipeline importable (single source of truth for the estimator).
_SENSORS_DIR = REPO_ROOT / "sensors"
if str(_SENSORS_DIR) not in sys.path:
    sys.path.insert(0, str(_SENSORS_DIR))

from allan_variance import identify_regimes, overlapping_allan_deviation, plot_allan  # noqa: E402

CONFIG_PATH = REPO_ROOT / "configs" / "sensor_params.json"
FIG_DIR = REPO_ROOT / "postproc" / "figures"


@dataclass
class LoopClosureResult:
    config_white: float
    recovered_white: float
    white_err_frac: float
    config_sigma_gm: float
    recovered_sigma_gm: float
    n_samples: int
    dt: float
    passed: bool

    def summary_line(self) -> str:
        return (
            f"white: config {self.config_white:.3e} vs sim {self.recovered_white:.3e} "
            f"({self.white_err_frac * 100:+.1f}%)  |  "
            f"sigma_GM: config {self.config_sigma_gm:.3e} vs sim {self.recovered_sigma_gm:.3e}  "
            f"| {self.n_samples} samples @ {1 / self.dt:.0f} Hz  "
            f"-> {'PASS' if self.passed else 'FAIL'}"
        )


def _loop_closure_config(dt: float = 0.01) -> dict:
    """A long, gentle, sensors-on trajectory so the IMU runs for many seconds.

    A near-vertical lob with no target maximizes flight time and keeps the true specific
    force smooth (easy to subtract), giving the cleanest noise residual for Allan analysis.
    Sensors read ``configs/sensor_params.json`` via the embedded imu block.
    """
    imu = json.loads(CONFIG_PATH.read_text())["imu"]
    return {
        "scenario": "loop_closure",
        "model": "3dof",
        "seed": 7,
        "dt": dt,
        "t_end": 300.0,
        "integrator": "rk4",
        "env": {"g0": 9.80665, "altitude_dependent_g": False, "atmosphere": False},
        "aero": {"ref_area": 0.001, "cd_mach": [[0.0, 0.05]]},
        "vehicle": {
            "pos0": [0, 0, 0],
            "launch_speed": 1400,
            "launch_elevation_deg": 89.5,
            "launch_azimuth_deg": 0,
            "mass0": 100.0,
        },
        "guidance": {"law": "none"},
        "sensors": {
            "enable": True,
            "imu": imu,
            "seeker": {"los_white": 0.0, "los_bias": 0.0, "glint": 0.0},
        },
        "target": {"pos0": [9_999_999, 0, 0], "vel0": [0, 0, 0], "maneuver": "constant"},
    }


def run_loop_closure(white_tol_frac: float = 0.15, make_figure: bool = True) -> LoopClosureResult:
    """Run the sensors-on sim, re-characterize the IMU, and compare to the config."""
    cfg = _loop_closure_config()
    imu_cfg = cfg["sensors"]["imu"]
    with tempfile.TemporaryDirectory(prefix="gncloop_") as td:
        tmp = Path(td)
        (tmp / "loop.json").write_text(json.dumps(cfg, indent=2))
        out = run_cli(tmp / "loop.json", tmp / "loop", timeout=180.0)
        sensors = load_run(out).sensors

    if sensors.empty or "imu_accel_meas_x" not in sensors:
        raise RuntimeError("sensors.csv missing imu_accel_meas_x — was sensors.enable true?")

    dt_s = cfg["dt"]
    sf = imu_cfg["accel_scale_factor"]
    # Residual isolates additive bias + white (remove the multiplicative scale-factor term).
    residual = sensors["imu_accel_meas_x"].to_numpy() - sensors["imu_accel_true_x"].to_numpy() * (
        1.0 + sf
    )
    # Drop any leading transient (first 1 s) and de-mean.
    skip = int(round(1.0 / dt_s))
    residual = residual[skip:]
    residual = residual - np.mean(residual)

    taus, adev, edf = overlapping_allan_deviation(residual, dt_s, num_points=80)
    fit = identify_regimes(taus, adev, edf)

    cfg_white = imu_cfg["accel_white"]
    cfg_sigma_gm = imu_cfg["accel_bias_instability"]
    white_err = abs(fit.white / cfg_white - 1.0)
    passed = white_err < white_tol_frac

    if make_figure:
        FIG_DIR.mkdir(parents=True, exist_ok=True)
        plot_allan(
            {"accel residual": (taus, adev, fit)},
            FIG_DIR / "loop_closure_allan.png",
            title="Loop closure — Allan deviation of in-sim IMU residual",
        )

    return LoopClosureResult(
        config_white=cfg_white,
        recovered_white=fit.white,
        white_err_frac=white_err,
        config_sigma_gm=cfg_sigma_gm,
        recovered_sigma_gm=fit.sigma_gm,
        n_samples=residual.size,
        dt=dt_s,
        passed=passed,
    )


def main() -> int:
    res = run_loop_closure()
    print("\nSensor loop-closure (in-sim IMU re-characterization)")
    print("=" * 78)
    print(" ", res.summary_line())
    print("  figure -> postproc/figures/loop_closure_allan.png")
    print("=" * 78)
    return 0 if res.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
