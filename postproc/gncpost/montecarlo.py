"""Monte-Carlo dispersion analysis: CEP, mean, std from a ``summary.csv``.

CEP (Circular Error Probable) is the median miss distance — the radius containing 50%
of impacts. We report the empirical median (distribution-free) alongside the mean/std
and the intercept fraction.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pandas as pd

from .plots import plot_miss_histogram
from .uq import bootstrap_ci, cep_stat, pkill_stat

# Seed for the bootstrap CIs reported with every summary, so the reported
# half-widths are deterministic for a given sample.
CI_SEED = 0xC1


@dataclass
class MonteCarloStats:
    n: int
    cep: float  # circular error probable (median miss distance) [m]
    mean: float  # mean miss distance [m]
    std: float  # std of miss distance [m]
    p90: float  # 90th-percentile miss [m]
    intercept_rate: float  # fraction of cases flagged intercept
    rms: float  # RMS miss distance [m]
    # Bootstrap 95% confidence half-widths on the two headline metrics. Always
    # reported so a campaign's sampling noise is visible (issue #33). NaN for P(kill)
    # when the summary carries no intercept flag.
    cep_ci_halfwidth: float = float("nan")
    pkill_ci_halfwidth: float = float("nan")
    ci_level: float = 0.95

    def summary_line(self) -> str:
        cep_ci = f"+/-{self.cep_ci_halfwidth:.2f}" if np.isfinite(self.cep_ci_halfwidth) else ""
        pk_ci = (
            f"+/-{self.pkill_ci_halfwidth * 100:.1f}"
            if np.isfinite(self.pkill_ci_halfwidth)
            else ""
        )
        return (
            f"n={self.n}  CEP={self.cep:.2f}{cep_ci} m  mean={self.mean:.2f} m  "
            f"std={self.std:.2f} m  P90={self.p90:.2f} m  "
            f"RMS={self.rms:.2f} m  intercept={self.intercept_rate * 100:.1f}{pk_ci}% "
            f"({self.ci_level:.0%} CI)"
        )


def _intercept_flags(summary: pd.DataFrame) -> np.ndarray | None:
    """Decode the ``intercept`` column to a 0/1 float array, or None if absent."""
    intercept = summary.get("intercept")
    if intercept is None:
        return None
    inter = np.asarray(intercept)
    if inter.dtype == object:
        inter = np.array([str(v).strip().lower() in ("1", "true", "yes") for v in inter])
    return inter.astype(float)


def compute_stats(summary: pd.DataFrame, *, ci_level: float = 0.95) -> MonteCarloStats:
    """Compute CEP and dispersion metrics (with bootstrap CIs) from a summary DataFrame."""
    miss = np.asarray(summary["miss_distance"], dtype=float)
    miss = miss[np.isfinite(miss)]
    if miss.size == 0:
        raise ValueError("no finite miss_distance values in summary")

    inter = _intercept_flags(summary)
    intercept_rate = float(np.mean(inter)) if inter is not None else float("nan")

    # Bootstrap CIs on the two headline metrics (seeded -> deterministic half-widths).
    cep_hw = float("nan")
    if miss.size > 1:
        cep_hw = bootstrap_ci(miss, cep_stat, level=ci_level, seed=CI_SEED).half_width
    pkill_hw = float("nan")
    if inter is not None and inter.size > 1:
        pkill_hw = bootstrap_ci(inter, pkill_stat, level=ci_level, seed=CI_SEED).half_width

    return MonteCarloStats(
        n=int(miss.size),
        cep=float(np.median(miss)),
        mean=float(np.mean(miss)),
        std=float(np.std(miss, ddof=1)) if miss.size > 1 else 0.0,
        p90=float(np.percentile(miss, 90)),
        intercept_rate=intercept_rate,
        rms=float(np.sqrt(np.mean(miss**2))),
        cep_ci_halfwidth=cep_hw,
        pkill_ci_halfwidth=pkill_hw,
        ci_level=ci_level,
    )


def plot_cep(summary: pd.DataFrame, stats: MonteCarloStats | None = None):
    """Miss-distance histogram annotated with CEP and mean."""
    if stats is None:
        stats = compute_stats(summary)
    return plot_miss_histogram(summary, cep=stats.cep, mean=stats.mean)
