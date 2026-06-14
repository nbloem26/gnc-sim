"""Engagement-level probability-of-kill (P(kill)) campaign for the cued interceptor (issue #8).

The headline acceptance product for launch-on-track cueing. Sweeps two engagement variables over a
grid and, at each grid point, runs a dispersed Monte-Carlo batch through the native CLI to estimate
P(kill) = fraction of cases that intercept (closest approach inside the lethal radius — the
``intercept`` flag in ``summary.csv``):

    (a) threat maneuver_g    — how hard the incoming threat weaves; harder -> lower P(kill).
    (b) launch_delay [s]     — the cue-to-launch delay (``cueing.max_cue_time`` with the
                               ``fixed_delay`` criterion); a longer delay lets the threat close in
                               and steals interceptor energy/time -> lower P(kill).

Each grid point is a full Monte-Carlo batch (``configs/engagement_pkill.json`` with dispersions on
launch speed / elevation and target position), so every P(kill) is an average over the same seeded
dispersion set. The cueing path is exercised on every case: the interceptor is held at the launch
site until ``launch_delay`` seconds elapse, then launched on a constant-velocity lead solution from
the fused track.

Two products:
    - a P(kill) SURFACE over the (maneuver_g, launch_delay) grid (left panel, heatmap), and
    - marginal P(kill) CURVES vs each axis (right panels).

Acceptance: P(kill) falls as the threat maneuvers harder and as the launch delay grows.

Run from repo root:  python postproc/gncpost/pkill.py
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

if __package__ in (None, ""):
    # Allow `python postproc/gncpost/pkill.py` (no package context).
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from gncpost import REPO_ROOT
    from gncpost.loaders import load_summary, run_cli
else:
    from . import REPO_ROOT
    from .loaders import load_summary, run_cli

# Sweep grid. The threat-maneuver axis spans benign -> hard weaves; the launch-delay axis spans a
# prompt cued launch -> a sluggish one. The reference engagement (configs/engagement_pkill.json) is
# a CROSSING threat — it flies past the launch site — so a longer cue-to-launch delay opens the
# geometry and steals the slower interceptor's reach, and a late-enough launch misses entirely.
MANEUVER_G_SWEEP = [0.0, 3.0, 6.0, 9.0]
LAUNCH_DELAY_SWEEP = [1.0, 4.0, 7.0, 10.0, 13.0]


@dataclass
class GridPoint:
    maneuver_g: float
    launch_delay: float
    pk: float  # fraction of Monte-Carlo cases that intercepted


def _base_config() -> dict:
    return json.loads((REPO_ROOT / "configs" / "engagement_pkill.json").read_text())


def _run_batch(maneuver_g: float, launch_delay: float, out_dir: Path) -> float:
    """Run one dispersed Monte-Carlo batch at a grid point; return P(kill) = intercept fraction."""
    cfg = _base_config()
    cfg["target"]["maneuver"] = "constant" if maneuver_g <= 0.0 else "weave"
    cfg["target"]["maneuver_g"] = float(maneuver_g)
    # launch_delay maps onto the fixed-delay cue: the interceptor launches at max_cue_time seconds.
    cfg["cueing"]["launch_criterion"] = "fixed_delay"
    cfg["cueing"]["max_cue_time"] = float(launch_delay)

    out_dir.mkdir(parents=True, exist_ok=True)
    cfg_path = out_dir / "config.json"
    cfg_path.write_text(json.dumps(cfg))
    run_cli(cfg_path, out_dir)

    summary = load_summary(out_dir)
    intercepts = np.asarray(summary["intercept"], dtype=float)
    return float(np.mean(intercepts)) if intercepts.size else 0.0


def run_pkill(out_root: Path | None = None) -> list[GridPoint]:
    """Run the full (maneuver_g, launch_delay) grid; return one GridPoint per cell."""
    out_root = Path(out_root) if out_root is not None else REPO_ROOT / "runs" / "pkill"
    out_root.mkdir(parents=True, exist_ok=True)

    points: list[GridPoint] = []
    print(f"{'maneuver_g':>10} {'launch_delay':>13} {'P(kill)':>9}")
    for g in MANEUVER_G_SWEEP:
        for d in LAUNCH_DELAY_SWEEP:
            cell = out_root / f"g{g}_d{d}"
            pk = _run_batch(g, d, cell)
            points.append(GridPoint(maneuver_g=g, launch_delay=d, pk=pk))
            print(f"{g:>10.1f} {d:>13.1f} {pk:>9.2f}")
    return points


def pk_grid(points: list[GridPoint]) -> np.ndarray:
    """Reshape the flat point list into a [maneuver_g x launch_delay] P(kill) matrix."""
    grid = np.zeros((len(MANEUVER_G_SWEEP), len(LAUNCH_DELAY_SWEEP)))
    for p in points:
        i = MANEUVER_G_SWEEP.index(p.maneuver_g)
        j = LAUNCH_DELAY_SWEEP.index(p.launch_delay)
        grid[i, j] = p.pk
    return grid


def plot_pkill(points: list[GridPoint]):
    """Three panels: P(kill) surface (heatmap) + marginal curves vs each axis."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    grid = pk_grid(points)

    fig, (ax_surf, ax_g, ax_d) = plt.subplots(1, 3, figsize=(16, 4.6))

    # --- P(kill) surface (heatmap) over the grid ---
    im = ax_surf.imshow(
        grid,
        origin="lower",
        aspect="auto",
        cmap="viridis",
        vmin=0.0,
        vmax=1.0,
        extent=(
            min(LAUNCH_DELAY_SWEEP),
            max(LAUNCH_DELAY_SWEEP),
            min(MANEUVER_G_SWEEP),
            max(MANEUVER_G_SWEEP),
        ),
    )
    ax_surf.set_xlabel("launch delay [s]")
    ax_surf.set_ylabel("threat maneuver [g]")
    ax_surf.set_title("P(kill) surface", fontweight="bold")
    fig.colorbar(im, ax=ax_surf, label="P(kill)")
    # Annotate each cell with its value.
    for i, g in enumerate(MANEUVER_G_SWEEP):
        for j, d in enumerate(LAUNCH_DELAY_SWEEP):
            ax_surf.text(
                d, g, f"{grid[i, j]:.2f}", ha="center", va="center", color="white", fontsize=8
            )

    # --- Marginal: P(kill) vs maneuver_g, one curve per launch_delay ---
    for j, d in enumerate(LAUNCH_DELAY_SWEEP):
        ax_g.plot(MANEUVER_G_SWEEP, grid[:, j], "o-", label=f"delay={d:.1f}s")
    ax_g.set_xlabel("threat maneuver [g]")
    ax_g.set_ylabel("P(kill)")
    ax_g.set_ylim(-0.03, 1.03)
    ax_g.set_title("P(kill) vs threat maneuver", fontweight="bold")
    ax_g.grid(True, alpha=0.25)
    ax_g.legend(fontsize=8, loc="upper right")

    # --- Marginal: P(kill) vs launch_delay, one curve per maneuver_g ---
    for i, g in enumerate(MANEUVER_G_SWEEP):
        ax_d.plot(LAUNCH_DELAY_SWEEP, grid[i, :], "s-", label=f"{g:.0f}g")
    ax_d.set_xlabel("launch delay [s]")
    ax_d.set_ylabel("P(kill)")
    ax_d.set_ylim(-0.03, 1.03)
    ax_d.set_title("P(kill) vs launch delay", fontweight="bold")
    ax_d.grid(True, alpha=0.25)
    ax_d.legend(fontsize=8, loc="upper right")

    fig.suptitle(
        "Cued interceptor P(kill): degrades with harder threat maneuver and longer launch delay",
        fontweight="bold",
        y=1.02,
    )
    fig.tight_layout()
    return fig


def main() -> None:
    import shutil

    print("Running P(kill) campaign over (maneuver_g, launch_delay) ...\n")
    points = run_pkill()

    grid = pk_grid(points)
    # Acceptance checks: P(kill) trends down as the threat maneuvers harder (averaged over delay) and
    # as the launch delay grows (averaged over maneuver).
    pk_by_g = grid.mean(axis=1)
    pk_by_d = grid.mean(axis=0)
    print(
        "\nmean P(kill) by maneuver_g: "
        + ", ".join(f"{g:.0f}g={p:.2f}" for g, p in zip(MANEUVER_G_SWEEP, pk_by_g, strict=False))
    )
    print(
        "mean P(kill) by launch_delay: "
        + ", ".join(f"{d:.1f}s={p:.2f}" for d, p in zip(LAUNCH_DELAY_SWEEP, pk_by_d, strict=False))
    )
    print(f"P(kill) falls with harder maneuver: {pk_by_g[0] >= pk_by_g[-1] - 1e-9}")
    print(f"P(kill) falls with longer delay:    {pk_by_d[0] >= pk_by_d[-1] - 1e-9}")

    fig = plot_pkill(points)
    postproc_fig = REPO_ROOT / "postproc" / "figures"
    postproc_fig.mkdir(parents=True, exist_ok=True)
    out_png = postproc_fig / "pkill.png"
    fig.savefig(out_png, dpi=130, bbox_inches="tight")
    print(f"\nwrote {out_png}")

    web_fig = REPO_ROOT / "web" / "public" / "figures"
    web_fig.mkdir(parents=True, exist_ok=True)
    shutil.copy2(out_png, web_fig / "pkill.png")
    print(f"copied to {web_fig / 'pkill.png'}")


if __name__ == "__main__":
    main()
