"""Generate the portfolio figures and copy the key ones to web/public/figures/.

Produces:
    postproc/figures/trajectory_sample.png     — sample-run vehicle-vs-target trajectory
    postproc/figures/validation_summary.png    — analytic-check pass/fail table image
    postproc/figures/montecarlo_cep.png        — miss-distance histogram + CEP
    postproc/figures/loop_closure_allan.png    — in-sim IMU Allan (from loop_closure)
    sensors/figures/allan_deviation.png        — synthetic IMU Allan (from allan_variance)

then copies allan_deviation.png, loop_closure_allan.png, montecarlo_cep.png,
validation_summary.png, trajectory_sample.png into web/public/figures/.

Run from repo root:  python postproc/make_figures.py
"""

from __future__ import annotations

import shutil
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from gncpost import REPO_ROOT
from gncpost.loaders import load_run, load_summary
from gncpost.montecarlo import compute_stats, plot_cep
from gncpost.plots import plot_trajectory
from gncpost.validate import run_all

POSTPROC_FIG = REPO_ROOT / "postproc" / "figures"
SENSORS_FIG = REPO_ROOT / "sensors" / "figures"
WEB_FIG = REPO_ROOT / "web" / "public" / "figures"

WEB_FIGURES = [
    "allan_deviation.png",
    "loop_closure_allan.png",
    "montecarlo_cep.png",
    "validation_summary.png",
    "trajectory_sample.png",
    "fusion_track_error.png",
    "uq_convergence.png",
    "uq_sobol.png",
]


def make_trajectory(run_dir: Path) -> None:
    run = load_run(run_dir)
    fig = plot_trajectory(run, mode="east_up")
    POSTPROC_FIG.mkdir(parents=True, exist_ok=True)
    fig.savefig(POSTPROC_FIG / "trajectory_sample.png", dpi=130)
    plt.close(fig)


def make_validation_summary() -> bool:
    results = run_all()
    all_ok = all(r.passed for r in results)
    fig, ax = plt.subplots(figsize=(12, 3.2))
    ax.axis("off")
    rows = [["Check", "Result", "Detail"]]
    for r in results:
        rows.append([r.name, "PASS" if r.passed else "FAIL", r.detail])
    table = ax.table(
        cellText=rows[1:],
        colLabels=rows[0],
        loc="center",
        cellLoc="left",
        colWidths=[0.18, 0.10, 0.72],
    )
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 1.7)
    for j in range(3):
        table[0, j].set_facecolor("#3f6e5c")
        table[0, j].set_text_props(color="white", fontweight="bold")
    for i, r in enumerate(results, start=1):
        color = "#e8f5ef" if r.passed else "#fef2f2"
        for j in range(3):
            table[i, j].set_facecolor(color)
    ax.set_title("Analytic validation of gnc-sim core", fontweight="bold", pad=14)
    fig.tight_layout()
    fig.savefig(POSTPROC_FIG / "validation_summary.png", dpi=130, bbox_inches="tight")
    plt.close(fig)
    return all_ok


def make_montecarlo(batch_dir: Path) -> None:
    summary = load_summary(batch_dir)
    stats = compute_stats(summary)
    fig = plot_cep(summary, stats)
    fig.savefig(POSTPROC_FIG / "montecarlo_cep.png", dpi=130)
    plt.close(fig)
    print("  MC:", stats.summary_line())


def make_fusion() -> None:
    from gncpost.fusion import plot_fusion, run_fusion

    results = run_fusion()
    fig = plot_fusion(results)
    fig.savefig(POSTPROC_FIG / "fusion_track_error.png", dpi=130, bbox_inches="tight")
    plt.close(fig)
    for r in results:
        print("  fusion:", r.summary_line())


def copy_to_web() -> None:
    WEB_FIG.mkdir(parents=True, exist_ok=True)
    for name in WEB_FIGURES:
        src = SENSORS_FIG / name if name == "allan_deviation.png" else POSTPROC_FIG / name
        if src.exists():
            shutil.copy2(src, WEB_FIG / name)
            print(f"  copied {name}")
        else:
            print(f"  MISSING {src}")


def main() -> None:
    print("Generating figures...")
    make_trajectory(REPO_ROOT / "runs" / "sample_run")
    make_validation_summary()
    make_montecarlo(REPO_ROOT / "runs" / "mc_batch")
    make_fusion()

    # Synthetic-IMU Allan figure (sensors/figures/allan_deviation.png).
    import allan_variance

    allan_variance.main()

    # In-sim IMU loop-closure figure (postproc/figures/loop_closure_allan.png).
    from gncpost.loop_closure import run_loop_closure

    res = run_loop_closure(make_figure=True)
    print("  loop-closure:", res.summary_line())

    print("Copying to web/public/figures/ ...")
    copy_to_web()
    print("done.")


if __name__ == "__main__":
    import sys

    sys.path.insert(0, str(REPO_ROOT / "sensors"))
    main()
