"""Analytic validation of the gnc-sim C++ core.

Each check writes a small config to a temp dir, runs the native CLI, and compares the
telemetry against a closed-form solution. Run as a script for a pass/fail table:

    python postproc/gncpost/validate.py

Checks:
    (i)   drag-free ballistic   — position vs analytic parabola (mm-level)
    (ii)  terminal velocity     — heavy-drag vertical drop vs sqrt(2 m g/(rho Cd A))
    (iii) USSA76 atmosphere     — density/temperature vs published table values
    (iv)  noise-free PN homing  — proportional-navigation intercept, miss < 1 m
"""

from __future__ import annotations

import json
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .atmosphere import REFERENCE_TABLE, terminal_velocity, ussa76
from .loaders import load_run, run_cli


@dataclass
class CheckResult:
    name: str
    passed: bool
    detail: str
    metric: float = float("nan")
    tolerance: float = float("nan")


def _write_config(tmp: Path, name: str, cfg: dict) -> Path:
    p = tmp / f"{name}.json"
    p.write_text(json.dumps(cfg, indent=2))
    return p


# --------------------------------------------------------------------------------------
# (i) Drag-free ballistic vs analytic parabola
# --------------------------------------------------------------------------------------
def check_ballistic(tmp: Path, tol_m: float = 0.05) -> CheckResult:
    """Atmosphere off, guidance none: vehicle follows x=vx0 t, z=vz0 t - 0.5 g t^2."""
    g_mps2 = 9.80665
    speed_mps, elev_deg = 300.0, 45.0
    cfg = {
        "scenario": "val_ballistic",
        "model": "3dof",
        "seed": 1,
        "dt": 0.002,
        "t_end": 30.0,
        "integrator": "rk4",
        "env": {"g0": g_mps2, "altitude_dependent_g": False, "atmosphere": False},
        "aero": {"ref_area": 0.02, "cd_mach": [[0.0, 0.3]]},
        "vehicle": {
            "pos0": [0, 0, 0],
            "launch_speed": speed_mps,
            "launch_elevation_deg": elev_deg,
            "launch_azimuth_deg": 0,
            "mass0": 22.0,
        },
        "guidance": {"law": "none"},
        "sensors": {"enable": False},
        "target": {"pos0": [9_999_999, 0, 0], "vel0": [0, 0, 0], "maneuver": "constant"},
    }
    out = run_cli(_write_config(tmp, "ballistic", cfg), tmp / "ballistic")
    veh = load_run(out).vehicle
    t_s = veh["t"].to_numpy()
    vx0_mps = speed_mps * np.cos(np.radians(elev_deg))
    vz0_mps = speed_mps * np.sin(np.radians(elev_deg))
    x_an_m = vx0_mps * t_s
    z_an_m = vz0_mps * t_s - 0.5 * g_mps2 * t_s * t_s
    err_m = np.sqrt((veh["x"] - x_an_m) ** 2 + (veh["z"] - z_an_m) ** 2).max()
    return CheckResult(
        "ballistic_parabola",
        err_m < tol_m,
        f"max |pos - analytic| = {err_m * 1e3:.3f} mm over {len(t_s)} steps",
        metric=float(err_m),
        tolerance=tol_m,
    )


# --------------------------------------------------------------------------------------
# (ii) Terminal velocity vs closed form
# --------------------------------------------------------------------------------------
def check_terminal_velocity(tmp: Path, tol_frac: float = 0.02) -> CheckResult:
    """Heavy-drag vertical drop reaches v_term = sqrt(2 m g / (rho Cd A))."""
    g_mps2, mass_kg, cd, area = 9.80665, 5.0, 1.0, 0.05
    drop_alt_m = 2000.0
    cfg = {
        "scenario": "val_vterm",
        "model": "3dof",
        "seed": 1,
        "dt": 0.002,
        "t_end": 120.0,
        "integrator": "rk4",
        "env": {"g0": g_mps2, "altitude_dependent_g": False, "atmosphere": True},
        "aero": {"ref_area": area, "cd_mach": [[0.0, cd], [5.0, cd]]},
        "vehicle": {
            "pos0": [0, 0, drop_alt_m],
            "launch_speed": 0.001,
            "launch_elevation_deg": -90,
            "launch_azimuth_deg": 0,
            "mass0": mass_kg,
        },
        "guidance": {"law": "none"},
        "sensors": {"enable": False},
        "target": {"pos0": [9_999_999, 0, 0], "vel0": [0, 0, 0], "maneuver": "constant"},
    }
    out = run_cli(_write_config(tmp, "vterm", cfg), tmp / "vterm")
    veh = load_run(out).vehicle
    # Sample the equilibrated descent: take the window where the vehicle is between 200 and
    # 800 m (well past the transient, density ~ constant), use its mean altitude for rho.
    mask = (veh["z"] > 200) & (veh["z"] < 800) & (veh["vz"] < 0)
    if mask.sum() < 5:
        mask = veh["vz"] < 0
    vz_sim_mps = float(veh.loc[mask, "vz"].mean())
    z_mean_m = float(veh.loc[mask, "z"].mean())
    v_term_mps = terminal_velocity(mass_kg, cd, area, z_mean_m, g_mps2)
    err = abs(abs(vz_sim_mps) - v_term_mps) / v_term_mps
    return CheckResult(
        "terminal_velocity",
        err < tol_frac,
        f"sim |vz|={abs(vz_sim_mps):.2f} m/s vs analytic {v_term_mps:.2f} m/s "
        f"(rho @ z={z_mean_m:.0f} m), err {err * 100:.2f}%",
        metric=float(err),
        tolerance=tol_frac,
    )


# --------------------------------------------------------------------------------------
# (iii) USSA76 density/temperature vs published table
# --------------------------------------------------------------------------------------
def check_atmosphere(tol_frac: float = 0.05) -> CheckResult:
    """USSA76 twin reproduces published table density & temperature within tolerance."""
    worst = 0.0
    detail_bits = []
    for alt, (T_ref, rho_ref) in REFERENCE_TABLE.items():
        s = ussa76(alt)
        e_rho = abs(s.density / rho_ref - 1.0)
        e_T = abs(s.temperature / T_ref - 1.0)
        worst = max(worst, e_rho, e_T)
        detail_bits.append(f"{alt // 1000}km:rho{e_rho * 100:.1f}%")
    return CheckResult(
        "ussa76_table",
        worst < tol_frac,
        "max table error " + f"{worst * 100:.2f}% (" + ", ".join(detail_bits) + ")",
        metric=float(worst),
        tolerance=tol_frac,
    )


# --------------------------------------------------------------------------------------
# (iv) Noise-free proportional-navigation intercept
# --------------------------------------------------------------------------------------
def check_pronav_intercept(tmp: Path, tol_m: float = 1.0) -> CheckResult:
    """Sensors off, PN guidance: miss distance < 1 m against a constant-velocity target."""
    cfg = {
        "scenario": "val_pronav",
        "model": "3dof",
        "seed": 1,
        "dt": 0.002,
        "t_end": 40.0,
        "integrator": "rk4",
        "env": {"g0": 9.80665, "altitude_dependent_g": False, "atmosphere": True},
        "aero": {
            "ref_area": 0.02,
            "cd_mach": [[0.0, 0.28], [0.9, 0.42], [1.0, 0.58], [1.5, 0.43], [3.0, 0.3]],
        },
        "vehicle": {
            "pos0": [0, 0, 0],
            "launch_speed": 900,
            "launch_elevation_deg": 42,
            "launch_azimuth_deg": 0,
            "mass0": 22.0,
        },
        "guidance": {"law": "pronav", "nav_constant": 4.0, "max_accel": 400.0},
        "sensors": {"enable": False},
        "target": {"pos0": [9000, 0, 3500], "vel0": [-280, 0, -40], "maneuver": "constant"},
    }
    out = run_cli(_write_config(tmp, "pronav", cfg), tmp / "pronav")
    run = load_run(out)
    miss_m = run.miss_distance
    return CheckResult(
        "pronav_intercept",
        run.intercept and miss_m < tol_m,
        f"intercept={run.intercept}, miss_distance={miss_m * 1e2:.2f} cm",
        metric=float(miss_m),
        tolerance=tol_m,
    )


def run_all() -> list[CheckResult]:
    """Run every analytic validation check and return the results."""
    with tempfile.TemporaryDirectory(prefix="gncval_") as td:
        tmp = Path(td)
        results = [
            check_ballistic(tmp),
            check_terminal_velocity(tmp),
            check_atmosphere(),
            check_pronav_intercept(tmp),
        ]
    return results


def print_table(results: list[CheckResult]) -> bool:
    """Pretty-print the results table; return True iff all passed."""
    print("\nAnalytic validation of gnc-sim core")
    print("=" * 78)
    print(f"  {'check':22s} {'result':8s}  detail")
    print("-" * 78)
    all_ok = True
    for r in results:
        all_ok &= r.passed
        status = "PASS" if r.passed else "FAIL"
        print(f"  {r.name:22s} [{status}]  {r.detail}")
    print("=" * 78)
    print(f"  {'ALL PASSED' if all_ok else 'SOME CHECKS FAILED'}\n")
    return all_ok


def main() -> int:
    results = run_all()
    ok = print_table(results)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
