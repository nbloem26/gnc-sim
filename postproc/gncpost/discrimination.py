"""Seeker decoy / closely-spaced-object discrimination analysis (issue #6).

Sweeps the decoy COUNT and the feature SEPARABILITY of a representative decoyed engagement,
runs the native CLI per configuration over a small Monte-Carlo set of seeds, and quantifies how
discrimination accuracy and probability-of-kill (Pk) degrade as decoys become more numerous and
more target-like.

Two products:
    (a) discrimination accuracy = fraction of runs whose seeker mostly homed on the TRUE target
        (``discrim_correct`` in ``discrim.csv`` — see ``docs/DATA_CONTRACT.md §3``).
    (b) Pk = fraction of runs that intercepted (``manifest.json:intercept``).

Acceptance: both fall monotonically as decoy count rises and as separability drops — i.e. decoys
work. The figure plots accuracy and Pk against each axis.

Run from repo root:  python postproc/gncpost/discrimination.py
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

if __package__ in (None, ""):
    # Allow `python postproc/gncpost/discrimination.py` (no package context).
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from gncpost import REPO_ROOT
    from gncpost.loaders import run_cli
else:
    from . import REPO_ROOT
    from .loaders import run_cli

# Score the back half of each run (after the discriminator's score integrator has settled).
SETTLE_FRACTION = 0.5
# Small Monte-Carlo set of seeds per configuration — enough for a stable accuracy/Pk estimate.
SEEDS = list(range(1, 31))

# Sweep axes. The interesting discrimination transition lives in the low-separability region, so the
# separability sweep is denser there.
COUNT_SWEEP = [1, 2, 4, 8, 16]
SEPARABILITY_SWEEP = [0.30, 0.20, 0.15, 0.12, 0.10, 0.07, 0.05]

# Fixed operating point for the OTHER axis while one axis is swept.
FIXED_SEPARABILITY = 0.10  # for the count sweep (hard regime so count matters)
FIXED_COUNT = 4  # for the separability sweep


@dataclass
class SweepPoint:
    x: float  # the swept value (count or separability)
    accuracy: float  # fraction of seeds that mostly selected the true target
    pk: float  # fraction of seeds that intercepted


def _base_config() -> dict:
    return json.loads((REPO_ROOT / "configs" / "discrimination.json").read_text())


def _run_one(cfg: dict, out_dir: Path) -> tuple[bool, bool]:
    """Run one config; return (selected_true_target_mostly, intercepted)."""
    cfg_path = out_dir / "config.json"
    out_dir.mkdir(parents=True, exist_ok=True)
    cfg_path.write_text(json.dumps(cfg))
    run_cli(cfg_path, out_dir)

    discrim = np.genfromtxt(out_dir / "discrim.csv", delimiter=",", names=True)
    correct = np.atleast_1d(discrim["discrim_correct"])
    start = int(len(correct) * SETTLE_FRACTION)
    acc = float(np.mean(correct[start:])) if len(correct) > start else 0.0

    manifest = json.loads((out_dir / "manifest.json").read_text())
    return acc > 0.5, bool(manifest.get("intercept", False))


def _sweep(set_value, values, out_root: Path, label: str) -> list[SweepPoint]:
    points: list[SweepPoint] = []
    for v in values:
        sel_hits = 0
        pk_hits = 0
        for seed in SEEDS:
            cfg = _base_config()
            set_value(cfg, v)
            cfg["seed"] = seed
            selected_true, intercepted = _run_one(cfg, out_root / f"{label}_{v}" / f"seed_{seed}")
            sel_hits += int(selected_true)
            pk_hits += int(intercepted)
        n = len(SEEDS)
        points.append(SweepPoint(x=float(v), accuracy=sel_hits / n, pk=pk_hits / n))
        print(f"  {label}={v:<6} accuracy={points[-1].accuracy:5.2f}  Pk={points[-1].pk:5.2f}")
    return points


def run_discrimination(out_root: Path | None = None) -> dict[str, list[SweepPoint]]:
    """Run the count and separability sweeps; return both as lists of SweepPoints."""
    out_root = Path(out_root) if out_root is not None else REPO_ROOT / "runs" / "discrimination"
    out_root.mkdir(parents=True, exist_ok=True)

    def set_count(cfg: dict, v: float) -> None:
        cfg["decoys"]["count"] = int(v)
        cfg["decoys"]["separability"] = FIXED_SEPARABILITY

    def set_sep(cfg: dict, v: float) -> None:
        cfg["decoys"]["separability"] = float(v)
        cfg["decoys"]["count"] = FIXED_COUNT

    print(f"Decoy COUNT sweep (separability={FIXED_SEPARABILITY}):")
    count_pts = _sweep(set_count, COUNT_SWEEP, out_root, "count")
    print(f"\nSEPARABILITY sweep (count={FIXED_COUNT}):")
    sep_pts = _sweep(set_sep, SEPARABILITY_SWEEP, out_root, "sep")

    return {"count": count_pts, "separability": sep_pts}


def plot_discrimination(sweeps: dict[str, list[SweepPoint]]):
    """Two panels: accuracy + Pk vs decoy count, and vs separability."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax_cnt, ax_sep) = plt.subplots(1, 2, figsize=(12, 4.8))

    cnt = sweeps["count"]
    cx = [p.x for p in cnt]
    ax_cnt.plot(
        cx, [p.accuracy for p in cnt], "o-", color="#3f6e5c", label="discrimination accuracy"
    )
    ax_cnt.plot(cx, [p.pk for p in cnt], "s--", color="#c45a5a", label="Pk (intercept rate)")
    ax_cnt.set_xlabel("decoy count")
    ax_cnt.set_ylabel("fraction")
    ax_cnt.set_ylim(-0.03, 1.03)
    ax_cnt.set_title(
        f"Accuracy & Pk vs decoy count\n(separability={FIXED_SEPARABILITY})", fontweight="bold"
    )
    ax_cnt.grid(True, alpha=0.25)
    ax_cnt.legend(loc="lower left", fontsize=9)

    sep = sweeps["separability"]
    sx = [p.x for p in sep]
    ax_sep.plot(
        sx, [p.accuracy for p in sep], "o-", color="#3f6e5c", label="discrimination accuracy"
    )
    ax_sep.plot(sx, [p.pk for p in sep], "s--", color="#c45a5a", label="Pk (intercept rate)")
    ax_sep.set_xlabel("separability  (1 = decoys distinct, 0 = decoys look like target)")
    ax_sep.set_ylabel("fraction")
    ax_sep.set_ylim(-0.03, 1.03)
    ax_sep.invert_xaxis()  # harder (lower separability) toward the right
    ax_sep.set_title(f"Accuracy & Pk vs separability\n(count={FIXED_COUNT})", fontweight="bold")
    ax_sep.grid(True, alpha=0.25)
    ax_sep.legend(loc="lower left", fontsize=9)

    fig.suptitle(
        "Seeker decoy discrimination: feature scoring degrades with more / closer decoys",
        fontweight="bold",
        y=1.02,
    )
    fig.tight_layout()
    return fig


def main() -> None:
    import shutil

    print("Running decoy discrimination sweeps ...\n")
    sweeps = run_discrimination()

    cnt = sweeps["count"]
    sep = sweeps["separability"]
    acc_by_count = [p.accuracy for p in cnt]
    acc_by_sep = [p.accuracy for p in sep]
    print(
        "\naccuracy monotone non-increasing with count:        "
        f"{all(a >= b - 1e-9 for a, b in zip(acc_by_count, acc_by_count[1:], strict=False))}"
    )
    print(
        "accuracy monotone non-increasing as separability drops: "
        f"{all(a >= b - 1e-9 for a, b in zip(acc_by_sep, acc_by_sep[1:], strict=False))}"
    )

    fig = plot_discrimination(sweeps)
    postproc_fig = REPO_ROOT / "postproc" / "figures"
    postproc_fig.mkdir(parents=True, exist_ok=True)
    out_png = postproc_fig / "discrimination.png"
    fig.savefig(out_png, dpi=130, bbox_inches="tight")
    print(f"\nwrote {out_png}")

    web_fig = REPO_ROOT / "web" / "public" / "figures"
    web_fig.mkdir(parents=True, exist_ok=True)
    shutil.copy2(out_png, web_fig / "discrimination.png")
    print(f"copied to {web_fig / 'discrimination.png'}")


if __name__ == "__main__":
    main()
