"""Multi-sensor target-track fusion analysis (issue #5).

Runs three otherwise-identical engagements that differ only in the ``trackers`` sensor array:

    radar-only  — a single ground radar  ([az, el, range, range_rate])
    IR-only     — a single space-based IR platform (angles-only [az, el])
    fused       — both sensors fused sequentially into one TargetTrackEkf

and computes the RMS of the fused/track position error (``track_*`` vs the true ``tgt_*``,
both in ``track.csv`` — see ``docs/DATA_CONTRACT.md §3``). The acceptance criterion is that
fusion beats either single sensor: angles-only IR has weak along-range observability (large
error), radar pins range, and fusing the two is tighter than radar alone.

Run from repo root:  python postproc/gncpost/fusion.py
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

if __package__ in (None, ""):
    # Allow `python postproc/gncpost/fusion.py` (no package context) by putting the postproc/
    # directory on sys.path so `gncpost` resolves, then importing absolutely.
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from gncpost import REPO_ROOT
    from gncpost.loaders import run_cli
else:
    from . import REPO_ROOT
    from .loaders import run_cli

# Discard the initial filter-settling transient before scoring the steady-state RMS.
SETTLE_STEPS = 200

CASES = [
    ("radar-only", "track_radar_only.json"),
    ("IR-only", "track_ir_only.json"),
    ("fused", "track_fused.json"),
    # Multi-target data association (issue #38): the same radar+IR geometry but with the JPDA
    # associator rejecting a closely-spaced-object cluster + Poisson clutter. The fused track RMS
    # stays in the same metres-class band as `fused` despite the decoys/clutter — i.e. the
    # associator keeps the true track. (Per-clutter-density track PURITY is quantified
    # deterministically by the C++ `data_association_test` benchmark.)
    ("jpda", "track_jpda.json"),
]


@dataclass
class FusionResult:
    label: str
    rms: float  # RMS track-position error [m]
    track: pd.DataFrame  # the loaded track.csv

    def summary_line(self) -> str:
        return f"{self.label:>10s}  RMS track error = {self.rms:8.2f} m"


def track_rms(track: pd.DataFrame, settle_steps: int = SETTLE_STEPS) -> float:
    """RMS Euclidean error of the fused track estimate vs the true target position."""
    df = track.iloc[settle_steps:] if len(track) > settle_steps else track
    dx_m = df["track_x"].to_numpy() - df["tgt_x"].to_numpy()
    dy_m = df["track_y"].to_numpy() - df["tgt_y"].to_numpy()
    dz_m = df["track_z"].to_numpy() - df["tgt_z"].to_numpy()
    sq = dx_m * dx_m + dy_m * dy_m + dz_m * dz_m
    return float(np.sqrt(np.mean(sq)))


def run_case(config_name: str, out_dir: Path) -> pd.DataFrame:
    """Run one tracker config through the native CLI and load its track.csv."""
    config_path = REPO_ROOT / "configs" / config_name
    run_cli(config_path, out_dir)
    track_path = out_dir / "track.csv"
    if not track_path.exists():
        raise FileNotFoundError(f"track.csv not written for {config_name}")
    return pd.read_csv(track_path)


def run_fusion(out_root: Path | None = None) -> list[FusionResult]:
    """Run radar-only, IR-only, and fused cases; return their RMS track errors."""
    out_root = Path(out_root) if out_root is not None else REPO_ROOT / "runs" / "fusion"
    out_root.mkdir(parents=True, exist_ok=True)
    results: list[FusionResult] = []
    for label, config_name in CASES:
        slug = config_name.replace(".json", "")
        track = run_case(config_name, out_root / slug)
        results.append(FusionResult(label=label, rms=track_rms(track), track=track))
    return results


def plot_fusion(results: list[FusionResult]):
    """Bar chart of RMS track error per case + a track-error-vs-time trace."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax_bar, ax_time) = plt.subplots(1, 2, figsize=(12, 4.6))

    labels = [r.label for r in results]
    rmss = [r.rms for r in results]
    colors = ["#5a8fc4", "#c4a24d", "#3f6e5c", "#a4565a"]
    bars = ax_bar.bar(labels, rmss, color=colors[: len(results)])
    ax_bar.set_ylabel("RMS track-position error [m]")
    ax_bar.set_title("Fusion beats either single sensor", fontweight="bold")
    ax_bar.set_yscale("log")
    for b, v in zip(bars, rmss, strict=False):
        ax_bar.annotate(
            f"{v:.2f} m",
            (b.get_x() + b.get_width() / 2, v),
            textcoords="offset points",
            xytext=(0, 4),
            ha="center",
            fontsize=9,
        )

    for r, color in zip(results, colors[: len(results)], strict=False):
        df = r.track.iloc[SETTLE_STEPS:]
        err_m = np.sqrt(
            (df["track_x"].to_numpy() - df["tgt_x"].to_numpy()) ** 2
            + (df["track_y"].to_numpy() - df["tgt_y"].to_numpy()) ** 2
            + (df["track_z"].to_numpy() - df["tgt_z"].to_numpy()) ** 2
        )
        ax_time.plot(df["t"].to_numpy(), err_m, label=r.label, color=color, lw=1.2)
    ax_time.set_xlabel("time [s]")
    ax_time.set_ylabel("track-position error [m]")
    ax_time.set_yscale("log")
    ax_time.set_title("Track error vs time", fontweight="bold")
    ax_time.legend(loc="upper right", fontsize=9)
    ax_time.grid(True, which="both", alpha=0.25)

    fig.suptitle(
        "Multi-sensor target-track fusion: ground radar + space-based IR",
        fontweight="bold",
        y=1.02,
    )
    fig.tight_layout()
    return fig


def main() -> None:
    import shutil

    print("Running multi-sensor fusion cases (radar-only / IR-only / fused) ...")
    results = run_fusion()
    for r in results:
        print("  " + r.summary_line())

    by_label = {r.label: r.rms for r in results}
    fused = by_label["fused"]
    radar = by_label["radar-only"]
    ir = by_label["IR-only"]
    print(f"\nfused < radar-only: {fused < radar}  ({fused:.2f} < {radar:.2f})")
    print(f"fused < IR-only:    {fused < ir}  ({fused:.2f} < {ir:.2f})")

    fig = plot_fusion(results)
    postproc_fig = REPO_ROOT / "postproc" / "figures"
    postproc_fig.mkdir(parents=True, exist_ok=True)
    out_png = postproc_fig / "fusion_track_error.png"
    fig.savefig(out_png, dpi=130, bbox_inches="tight")
    print(f"\nwrote {out_png}")

    web_fig = REPO_ROOT / "web" / "public" / "figures"
    web_fig.mkdir(parents=True, exist_ok=True)
    shutil.copy2(out_png, web_fig / "fusion_track_error.png")
    print(f"copied to {web_fig / 'fusion_track_error.png'}")


if __name__ == "__main__":
    main()
