"""UQ campaign driver: convergence + Sobol figures for the validation page (issue #33).

Two deterministic products, both saved to ``postproc/figures/`` and copied to
``web/public/figures/``:

    uq_convergence.png  — CEP vs number of Monte-Carlo cases with a running 95%
                          bootstrap CI band (from one dispersed MC batch).
    uq_sobol.png        — first/total-order Sobol indices ranking the dispersion
                          inputs (launch speed, launch elevation, target position,
                          weave phase) by how much each drives miss distance.

The Sobol model evaluation is a single native-CLI run with the four dispersion
inputs set explicitly (a Saltelli design needs ``n_base*(d+2)`` runs, so ``n_base``
is kept modest). Everything is seeded; the convergence batch reuses the existing
``configs/montecarlo.json`` Monte-Carlo path.

Run from repo root:  python -m gncpost.uq_campaign
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import numpy as np

if __package__ in (None, ""):
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from gncpost import REPO_ROOT
    from gncpost.loaders import load_run, load_summary, run_cli
    from gncpost.uq import (
        cep_stat,
        convergence,
        plot_convergence,
        plot_sobol,
        sobol_indices,
    )
else:
    from . import REPO_ROOT
    from .loaders import load_run, load_summary, run_cli
    from .uq import (
        cep_stat,
        convergence,
        plot_convergence,
        plot_sobol,
        sobol_indices,
    )

POSTPROC_FIG = REPO_ROOT / "postproc" / "figures"
WEB_FIG = REPO_ROOT / "web" / "public" / "figures"
CONFIG = REPO_ROOT / "configs" / "montecarlo.json"

# The four dispersion inputs (matching core/src/scenario/MonteCarlo.cpp) and a
# +/- range to sweep for the sensitivity study. Ranges are +/-3 sigma around the
# nominal for the Gaussian inputs, and the full weave cycle for the phase.
SOBOL_INPUTS = ["launch_speed", "launch_elevation", "target_pos", "weave_phase"]


# --------------------------------------------------------------------------------------
# Convergence figure (one dispersed MC batch)
# --------------------------------------------------------------------------------------


def make_convergence_figure(
    num_cases: int = 256, seed: int = 12345, out_root: Path | None = None
) -> Path:
    """Run one dispersed MC batch and plot CEP-vs-N with a running bootstrap CI band."""
    out_root = Path(out_root) if out_root is not None else REPO_ROOT / "runs" / "uq_convergence"
    cfg = json.loads(CONFIG.read_text())
    cfg.setdefault("monte_carlo", {})["num_cases"] = num_cases
    out_root.mkdir(parents=True, exist_ok=True)
    cfg_path = out_root / "config.json"
    cfg_path.write_text(json.dumps(cfg))
    run_cli(cfg_path, out_root, seed=seed)

    summary = load_summary(out_root)
    miss_m = np.asarray(summary["miss_distance"], dtype=float)
    points = convergence(miss_m, cep_stat, seed=seed)
    fig = plot_convergence(points, metric_label="CEP [m]")

    POSTPROC_FIG.mkdir(parents=True, exist_ok=True)
    out_png = POSTPROC_FIG / "uq_convergence.png"
    fig.savefig(out_png, dpi=130, bbox_inches="tight")
    import matplotlib.pyplot as plt

    plt.close(fig)
    print(
        f"  convergence: final CEP={points[-1].estimate:.2f} m "
        f"[{points[-1].low:.2f}, {points[-1].high:.2f}] over N={points[-1].n}"
    )
    return out_png


# --------------------------------------------------------------------------------------
# Sobol figure (Saltelli design over the four dispersion inputs)
# --------------------------------------------------------------------------------------


def _sobol_bounds(cfg: dict) -> list[tuple[float, float]]:
    """+/-3 sigma sweep ranges per input (weave phase = full cycle)."""
    mc = cfg["monte_carlo"]
    veh = cfg["vehicle"]
    speed_s = float(mc["launch_speed_sigma"])
    elev_s = float(mc["launch_elevation_sigma_deg"])
    pos_s = float(mc["target_pos_sigma"])
    return [
        (veh["launch_speed"] - 3 * speed_s, veh["launch_speed"] + 3 * speed_s),
        (veh["launch_elevation_deg"] - 3 * elev_s, veh["launch_elevation_deg"] + 3 * elev_s),
        (-3 * pos_s, 3 * pos_s),  # target position offset (applied to all 3 axes)
        (0.0, 360.0),  # weave phase [deg]
    ]


def _make_sim_model(cfg: dict, work_dir: Path):
    """Build a vectorised model X -> miss_distance via single native-CLI runs.

    Each input row is (launch_speed, launch_elevation_deg, target_pos_offset,
    weave_phase_deg). Runs are single-case (monte_carlo disabled) and seeded so the
    only variation across evaluations is the dispersion inputs themselves.
    """
    base = json.loads(json.dumps(cfg))  # deep copy
    base.pop("monte_carlo", None)
    tgt_nominal = list(base["target"]["pos0"])
    counter = {"i": 0}

    def model(x: np.ndarray) -> np.ndarray:
        x = np.atleast_2d(np.asarray(x, dtype=float))
        out = np.empty(x.shape[0])
        for k, row in enumerate(x):
            speed_mps, elev_deg, pos_off_m, phase_deg = row
            c = json.loads(json.dumps(base))
            c["vehicle"]["launch_speed"] = float(speed_mps)
            c["vehicle"]["launch_elevation_deg"] = float(elev_deg)
            c["target"]["pos0"] = [v + float(pos_off_m) for v in tgt_nominal]
            c["target"]["maneuver_phase_deg"] = float(phase_deg)
            run_d = work_dir / f"s{counter['i']:05d}"
            counter["i"] += 1
            cfg_path = run_d / "config.json"
            run_d.mkdir(parents=True, exist_ok=True)
            cfg_path.write_text(json.dumps(c))
            run_cli(cfg_path, run_d, seed=1)
            out[k] = load_run(run_d).manifest["miss_distance"]
        return out

    return model


def make_sobol_figure(n_base: int = 24, seed: int = 2024, out_root: Path | None = None) -> Path:
    """Saltelli + Sobol over the four dispersion inputs; plot the ranking bar chart."""
    out_root = Path(out_root) if out_root is not None else REPO_ROOT / "runs" / "uq_sobol"
    out_root.mkdir(parents=True, exist_ok=True)
    cfg = json.loads(CONFIG.read_text())
    cfg["target"]["maneuver"] = "weave"  # ensure phase actually matters

    bounds = _sobol_bounds(cfg)
    model = _make_sim_model(cfg, out_root)
    n_runs = n_base * (len(SOBOL_INPUTS) + 2)
    print(f"  sobol: running {n_runs} single CLI evaluations (n_base={n_base}) ...")
    result = sobol_indices(model, bounds, SOBOL_INPUTS, n_base=n_base, seed=seed)

    fig = plot_sobol(result, title="Sobol sensitivity — drivers of miss distance")
    POSTPROC_FIG.mkdir(parents=True, exist_ok=True)
    out_png = POSTPROC_FIG / "uq_sobol.png"
    fig.savefig(out_png, dpi=130, bbox_inches="tight")
    import matplotlib.pyplot as plt

    plt.close(fig)
    ranking = ", ".join(f"{n}={s:.2f}" for n, s in result.ranking())
    print(f"  sobol ranking (total-order): {ranking}")
    return out_png


# --------------------------------------------------------------------------------------


def copy_to_web(*names: str) -> None:
    WEB_FIG.mkdir(parents=True, exist_ok=True)
    for name in names:
        src = POSTPROC_FIG / name
        if src.exists():
            shutil.copy2(src, WEB_FIG / name)
            print(f"  copied {name}")
        else:
            print(f"  MISSING {src}")


def main() -> None:
    print("UQ campaign: convergence + Sobol sensitivity ...")
    make_convergence_figure()
    make_sobol_figure()
    copy_to_web("uq_convergence.png", "uq_sobol.png")
    print("done.")


if __name__ == "__main__":
    main()
