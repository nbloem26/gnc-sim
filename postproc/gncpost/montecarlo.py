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


@dataclass
class MonteCarloStats:
    n: int
    cep: float  # circular error probable (median miss distance) [m]
    mean: float  # mean miss distance [m]
    std: float  # std of miss distance [m]
    p90: float  # 90th-percentile miss [m]
    intercept_rate: float  # fraction of cases flagged intercept
    rms: float  # RMS miss distance [m]

    def summary_line(self) -> str:
        return (
            f"n={self.n}  CEP={self.cep:.2f} m  mean={self.mean:.2f} m  "
            f"std={self.std:.2f} m  P90={self.p90:.2f} m  "
            f"RMS={self.rms:.2f} m  intercept={self.intercept_rate * 100:.1f}%"
        )


def compute_stats(summary: pd.DataFrame) -> MonteCarloStats:
    """Compute CEP and dispersion metrics from a Monte-Carlo summary DataFrame."""
    miss = np.asarray(summary["miss_distance"], dtype=float)
    miss = miss[np.isfinite(miss)]
    if miss.size == 0:
        raise ValueError("no finite miss_distance values in summary")

    intercept = summary.get("intercept")
    if intercept is not None:
        # accept bool, 0/1, or 'yes'/'no' encodings
        inter = np.asarray(intercept)
        if inter.dtype == object:
            inter = np.array([str(v).strip().lower() in ("1", "true", "yes") for v in inter])
        intercept_rate = float(np.mean(inter.astype(float)))
    else:
        intercept_rate = float("nan")

    return MonteCarloStats(
        n=int(miss.size),
        cep=float(np.median(miss)),
        mean=float(np.mean(miss)),
        std=float(np.std(miss, ddof=1)) if miss.size > 1 else 0.0,
        p90=float(np.percentile(miss, 90)),
        intercept_rate=intercept_rate,
        rms=float(np.sqrt(np.mean(miss**2))),
    )


def plot_cep(summary: pd.DataFrame, stats: MonteCarloStats | None = None):
    """Miss-distance histogram annotated with CEP and mean."""
    if stats is None:
        stats = compute_stats(summary)
    return plot_miss_histogram(summary, cep=stats.cep, mean=stats.mean)
