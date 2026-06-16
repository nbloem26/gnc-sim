"""Uncertainty quantification for Monte-Carlo campaigns (issue #33).

Three deterministic, seeded tools layered on top of the existing CEP / P(kill)
dispersion analysis:

1. **Confidence intervals** (:func:`bootstrap_ci`) — a seeded nonparametric
   bootstrap CI for any statistic (CEP = median miss, P(kill) = intercept
   fraction). The reported *half-width* quantifies how much sampling noise sits
   on a headline number, so a campaign's sufficiency is legible at a glance.

2. **Convergence diagnostic** (:func:`convergence`) — the running statistic vs
   the number of cases ``N`` with a bootstrap CI band, so you can see the metric
   settle (and the band tighten as ``1/sqrt(N)``) and judge whether enough cases
   were run.

3. **Global sensitivity** (:func:`sobol_indices`) — a hand-rolled Saltelli
   sampler + first/total-order Sobol estimator (numpy only, no SALib). Ranks the
   dispersion inputs (launch speed, launch elevation, target position, weave
   phase) by how much each drives the output variance — i.e. *what drives miss*.

Everything takes an explicit ``numpy.random.Generator`` (or an integer seed) so
the same seed yields bit-identical CIs / Sobol indices. No bare ``np.random``.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass

import numpy as np

# --------------------------------------------------------------------------------------
# Seeding helper
# --------------------------------------------------------------------------------------


def as_rng(seed: int | np.random.Generator) -> np.random.Generator:
    """Coerce an int seed (or an existing Generator) into a seeded ``Generator``.

    Centralises seeding so every UQ entry point is deterministic and never touches
    the global ``np.random`` state.
    """
    if isinstance(seed, np.random.Generator):
        return seed
    return np.random.default_rng(seed)


# --------------------------------------------------------------------------------------
# Statistics + bootstrap confidence intervals
# --------------------------------------------------------------------------------------

Statistic = Callable[[np.ndarray], float]


def cep_stat(miss: np.ndarray) -> float:
    """CEP = median miss distance (the 50% radius)."""
    return float(np.median(miss))


def pkill_stat(intercept: np.ndarray) -> float:
    """P(kill) = mean of the (0/1) intercept flags."""
    return float(np.mean(intercept))


@dataclass(frozen=True)
class ConfidenceInterval:
    """A two-sided confidence interval for a point estimate.

    Attributes:
        estimate: the point estimate on the full sample.
        low: lower confidence bound.
        high: upper confidence bound.
        level: the nominal coverage (e.g. 0.95).
    """

    estimate: float
    low: float
    high: float
    level: float

    @property
    def half_width(self) -> float:
        """Half the CI width — the +/- reported alongside the point estimate."""
        return 0.5 * (self.high - self.low)

    def __str__(self) -> str:
        return f"{self.estimate:.3g} +/- {self.half_width:.3g} ({self.level:.0%} CI)"


def bootstrap_ci(
    data: Sequence[float] | np.ndarray,
    statistic: Statistic = cep_stat,
    *,
    level: float = 0.95,
    n_resamples: int = 2000,
    seed: int | np.random.Generator = 0,
) -> ConfidenceInterval:
    """Seeded nonparametric (percentile) bootstrap CI for ``statistic``.

    Resamples ``data`` with replacement ``n_resamples`` times, recomputes the
    statistic on each resample, and takes the empirical ``[alpha/2, 1-alpha/2]``
    percentiles as the interval. Distribution-free, so it works for the median
    (CEP) and the mean of a 0/1 vector (P(kill)) alike.

    Determinism: the resampling indices are drawn from the supplied ``Generator``
    (or ``default_rng(seed)``), so a fixed seed reproduces the interval exactly.
    """
    arr = np.asarray(data, dtype=float)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        raise ValueError("bootstrap_ci: no finite data")
    rng = as_rng(seed)

    n = arr.size
    # (n_resamples x n) index matrix -> vectorised resampling.
    idx = rng.integers(0, n, size=(n_resamples, n))
    resampled = arr[idx]
    stats = np.array([statistic(row) for row in resampled], dtype=float)

    alpha = 1.0 - level
    low, high = np.percentile(stats, [100.0 * alpha / 2.0, 100.0 * (1.0 - alpha / 2.0)])
    return ConfidenceInterval(
        estimate=float(statistic(arr)),
        low=float(low),
        high=float(high),
        level=float(level),
    )


# --------------------------------------------------------------------------------------
# Convergence diagnostic
# --------------------------------------------------------------------------------------


@dataclass(frozen=True)
class ConvergencePoint:
    n: int
    estimate: float
    low: float
    high: float


def convergence(
    data: Sequence[float] | np.ndarray,
    statistic: Statistic = cep_stat,
    *,
    level: float = 0.95,
    n_points: int = 20,
    min_n: int = 10,
    n_resamples: int = 1000,
    seed: int | np.random.Generator = 0,
) -> list[ConvergencePoint]:
    """Running ``statistic`` vs sample size ``N`` with a bootstrap CI band.

    Evaluates the statistic (and a bootstrap CI) on the first ``N`` cases for
    ``n_points`` values of ``N`` spaced log-uniformly between ``min_n`` and the
    full sample size. The CI band should tighten roughly as ``1/sqrt(N)`` and the
    estimate should stop wandering once enough cases have been run — that is the
    visual sufficiency check.

    Determinism: each sub-sample's CI is drawn from a child of the master RNG, so
    the whole curve is reproducible from a single seed.
    """
    arr = np.asarray(data, dtype=float)
    arr = arr[np.isfinite(arr)]
    total = arr.size
    if total < min_n:
        raise ValueError(f"convergence: need >= {min_n} finite points, got {total}")
    rng = as_rng(seed)

    ns = np.unique(np.round(np.geomspace(min_n, total, num=n_points)).astype(int))
    points: list[ConvergencePoint] = []
    for n in ns:
        # A distinct, deterministic child seed per N keeps each CI independent yet reproducible.
        child = np.random.default_rng(rng.integers(0, 2**63 - 1))
        ci = bootstrap_ci(
            arr[:n],
            statistic,
            level=level,
            n_resamples=n_resamples,
            seed=child,
        )
        points.append(ConvergencePoint(n=int(n), estimate=ci.estimate, low=ci.low, high=ci.high))
    return points


# --------------------------------------------------------------------------------------
# Global sensitivity analysis: Saltelli sampling + Sobol indices
# --------------------------------------------------------------------------------------


@dataclass(frozen=True)
class SobolResult:
    """First- and total-order Sobol indices for a set of named inputs.

    Attributes:
        names: input names in order.
        first_order: S_i — fraction of output variance from input i alone.
        total_order: S_Ti — fraction including all interactions involving i.
    """

    names: list[str]
    first_order: np.ndarray
    total_order: np.ndarray

    def ranking(self) -> list[tuple[str, float]]:
        """Inputs sorted by total-order index, descending (what drives the output)."""
        order = np.argsort(self.total_order)[::-1]
        return [(self.names[i], float(self.total_order[i])) for i in order]


def _unit_base(n_base: int, dim: int, seed: int | np.random.Generator) -> np.ndarray:
    """``(n_base x dim)`` points in ``[0, 1)``.

    Prefers a scrambled Sobol low-discrepancy sequence (``scipy.stats.qmc``) when
    ``n_base`` is a power of two — low-discrepancy points dramatically cut the
    variance of the Sobol-index estimators versus plain pseudo-random draws.
    Falls back to seeded uniform draws otherwise. Both paths are deterministic in
    the supplied seed; no bare ``np.random``.
    """
    rng = as_rng(seed)
    is_pow2 = n_base > 0 and (n_base & (n_base - 1)) == 0
    try:
        from scipy.stats import qmc

        if is_pow2:
            # Derive an integer seed for the QMC engine from the Generator (stays seeded).
            qseed = int(rng.integers(0, 2**63 - 1))
            engine = qmc.Sobol(d=dim, scramble=True, seed=qseed)
            return np.asarray(engine.random_base2(int(np.log2(n_base))), dtype=float)
    except ImportError:
        pass
    return rng.random((n_base, dim))


def saltelli_sample(
    n_base: int,
    bounds: Sequence[tuple[float, float]],
    *,
    seed: int | np.random.Generator = 0,
) -> tuple[np.ndarray, np.ndarray, list[np.ndarray]]:
    """Generate Saltelli sample matrices A, B and the cross matrices AB_i.

    A single ``(n_base x 2d)`` low-discrepancy block is split into two ``(n_base x
    d)`` matrices ``A`` and ``B`` (drawing both halves from one sequence keeps them
    jointly well-distributed); ``AB_i`` is ``A`` with column ``i`` replaced by
    ``B``'s column ``i``. Evaluating the model on ``A``, ``B`` and every ``AB_i``
    gives the pieces the first/total-order Sobol estimators need. Total model
    evaluations: ``n_base * (d + 2)``.

    Determinism: the base sequence comes from the supplied seeded source.
    """
    d = len(bounds)
    lo = np.array([b[0] for b in bounds], dtype=float)
    hi = np.array([b[1] for b in bounds], dtype=float)

    unit = _unit_base(n_base, 2 * d, seed)
    a = lo + unit[:, :d] * (hi - lo)
    b = lo + unit[:, d:] * (hi - lo)

    ab: list[np.ndarray] = []
    for i in range(d):
        abi = a.copy()
        abi[:, i] = b[:, i]
        ab.append(abi)
    return a, b, ab


def sobol_indices(
    model: Callable[[np.ndarray], np.ndarray],
    bounds: Sequence[tuple[float, float]],
    names: Sequence[str],
    *,
    n_base: int = 4096,
    seed: int | np.random.Generator = 0,
) -> SobolResult:
    """Estimate first- and total-order Sobol indices via the Saltelli design.

    ``model`` maps an ``(m x d)`` array of input rows to a length-``m`` output
    vector (vectorised). Estimators (Saltelli 2010 / Jansen 1999), both standard:

        S_i   = mean(yB * (yAB_i - yA)) / Var           (first order)
        S_Ti  = mean((yA - yAB_i)^2) / (2 * Var)         (total order)

    Indices are clipped to [0, 1]. Determinism follows from the seeded sampler.
    """
    if len(bounds) != len(names):
        raise ValueError("bounds and names must have the same length")
    a, b, ab = saltelli_sample(n_base, bounds, seed=seed)

    ya = np.asarray(model(a), dtype=float)
    yb = np.asarray(model(b), dtype=float)
    yab = [np.asarray(model(abi), dtype=float) for abi in ab]

    # Pool A and B outputs for the total-variance estimate.
    var = float(np.var(np.concatenate([ya, yb]), ddof=1))
    d = len(bounds)
    first = np.zeros(d)
    total = np.zeros(d)
    if var <= 0.0:
        # Degenerate (constant) output: no input explains any variance.
        return SobolResult(names=list(names), first_order=first, total_order=total)

    for i in range(d):
        first[i] = float(np.mean(yb * (yab[i] - ya))) / var
        total[i] = float(np.mean((ya - yab[i]) ** 2)) / (2.0 * var)

    return SobolResult(
        names=list(names),
        first_order=np.clip(first, 0.0, 1.0),
        total_order=np.clip(total, 0.0, 1.0),
    )


def ishigami(x: np.ndarray, a: float = 7.0, b: float = 0.1) -> np.ndarray:
    """Ishigami test function — the standard Sobol benchmark.

    ``f = sin(x1) + a*sin(x2)^2 + b * x3^4 * sin(x1)`` on ``[-pi, pi]^3``. Has
    known analytic Sobol indices. For the standard ``a=7, b=0.1`` the first-order
    ranking is x2 > x1 > x3 and the total-order ranking is x1 > x2 > x3 — x3's
    first-order index is exactly 0, all of its influence is interaction with x1.
    """
    x = np.atleast_2d(np.asarray(x, dtype=float))
    x1, x2, x3 = x[:, 0], x[:, 1], x[:, 2]
    return np.sin(x1) + a * np.sin(x2) ** 2 + b * x3**4 * np.sin(x1)


def ishigami_analytic_indices(a: float = 7.0, b: float = 0.1) -> tuple[np.ndarray, np.ndarray]:
    """Closed-form first- and total-order Sobol indices for :func:`ishigami`.

    Variance partition (e.g. Sobol & Levitan / Marrel et al.):
        D1  = b*pi^4/5 + b^2*pi^8/50 + 1/2
        D2  = a^2/8
        D3  = 0
        D13 = 8*b^2*pi^8/225   (only nonzero interaction term)
    with total variance D = D1 + D2 + D13.
    """
    pi = np.pi
    d1 = b * pi**4 / 5.0 + b**2 * pi**8 / 50.0 + 0.5
    d2 = a**2 / 8.0
    d3 = 0.0
    d13 = 8.0 * b**2 * pi**8 / 225.0
    var = d1 + d2 + d3 + d13
    first = np.array([d1, d2, d3]) / var
    total = np.array([d1 + d13, d2, d3 + d13]) / var
    return first, total


# --------------------------------------------------------------------------------------
# Trajectory uncertainty propagation (issue #49)
#
# The CI / Sobol tools above summarize the *terminal* output (miss). This propagates the
# dispersion across the *whole engagement*: given an ensemble of per-case trajectories
# (each a time series sampled on a common grid), it returns per-time percentile bands so
# the state's uncertainty growth/collapse is visible over the flight, not just at CPA.
# --------------------------------------------------------------------------------------


@dataclass(frozen=True)
class DispersionBand:
    """Per-time uncertainty band for one scalar channel across a Monte-Carlo ensemble.

    Attributes:
        t_s: common time grid [s], shape ``(T,)``.
        mean: ensemble mean per time, shape ``(T,)``.
        std: ensemble std (ddof=1) per time, shape ``(T,)``.
        low: lower percentile envelope per time, shape ``(T,)``.
        high: upper percentile envelope per time, shape ``(T,)``.
        median: ensemble median per time, shape ``(T,)``.
        level: the central coverage of ``[low, high]`` (e.g. 0.90).
        n: number of ensemble members.
    """

    t_s: np.ndarray
    mean: np.ndarray
    std: np.ndarray
    low: np.ndarray
    high: np.ndarray
    median: np.ndarray
    level: float
    n: int

    @property
    def peak_std(self) -> float:
        """The largest per-time std over the flight — the worst-case dispersion."""
        return float(np.max(self.std)) if self.std.size else float("nan")

    @property
    def terminal_std(self) -> float:
        """The dispersion at the final time sample (collapses toward CPA)."""
        return float(self.std[-1]) if self.std.size else float("nan")


def trajectory_band(
    t_s: Sequence[float] | np.ndarray,
    ensemble: Sequence[Sequence[float]] | np.ndarray,
    *,
    level: float = 0.90,
) -> DispersionBand:
    """Per-time dispersion band for one channel over a Monte-Carlo ensemble.

    Args:
        t_s: common time grid, shape ``(T,)``.
        ensemble: stacked per-case series, shape ``(N, T)`` — row ``i`` is case ``i``'s
            channel sampled on ``t_s``. Cases must already share the grid (resample
            upstream with :func:`resample_to_grid` if they don't).
        level: central coverage for the ``[low, high]`` envelope (0.90 -> 5th/95th pct).

    Returns:
        A :class:`DispersionBand`. The mean/median trend with a percentile envelope is
        the trajectory analogue of the terminal CEP/Pk CI: it shows where in the flight
        the state is most uncertain.

    Deterministic: pure reductions over the ensemble, no RNG.
    """
    grid = np.asarray(t_s, dtype=float)
    arr = np.asarray(ensemble, dtype=float)
    if arr.ndim != 2:
        raise ValueError("ensemble must be 2-D (N cases x T samples)")
    if arr.shape[1] != grid.size:
        raise ValueError(f"ensemble width {arr.shape[1]} != len(t_s) {grid.size}")
    if arr.shape[0] == 0:
        raise ValueError("trajectory_band: empty ensemble")

    alpha = 1.0 - level
    lo_pct = 100.0 * alpha / 2.0
    hi_pct = 100.0 * (1.0 - alpha / 2.0)
    n = arr.shape[0]
    return DispersionBand(
        t_s=grid,
        mean=np.mean(arr, axis=0),
        std=(np.std(arr, axis=0, ddof=1) if n > 1 else np.zeros(grid.size)),
        low=np.percentile(arr, lo_pct, axis=0),
        high=np.percentile(arr, hi_pct, axis=0),
        median=np.median(arr, axis=0),
        level=float(level),
        n=int(n),
    )


def resample_to_grid(
    t_s: Sequence[float] | np.ndarray,
    values: Sequence[float] | np.ndarray,
    grid_s: Sequence[float] | np.ndarray,
) -> np.ndarray:
    """Linearly resample one case's ``(t_s, values)`` series onto a common ``grid_s``.

    Monte-Carlo cases terminate at different times / step counts; to stack them into an
    ``(N, T)`` ensemble for :func:`trajectory_band` they must share a grid. Times in
    ``grid_s`` beyond a case's last sample are held at its final value (the case has
    ended — e.g. post-CPA), so the band reflects "still flying" vs "done".
    """
    tt = np.asarray(t_s, dtype=float)
    vv = np.asarray(values, dtype=float)
    g = np.asarray(grid_s, dtype=float)
    if tt.size == 0:
        raise ValueError("resample_to_grid: empty source series")
    # np.interp holds endpoints for out-of-range queries (left=vv[0], right=vv[-1]).
    return np.interp(g, tt, vv)


def common_time_grid(
    series_t: Sequence[Sequence[float] | np.ndarray], *, n_points: int = 200
) -> np.ndarray:
    """A shared time grid spanning ``[0, min last-time]`` across cases, ``n_points`` long.

    Uses the *shortest* case's end time as the grid's upper bound so every case has real
    (non-extrapolated) data over the whole grid — the band then reflects only genuine
    dispersion, not extrapolation artifacts.
    """
    last_times = [float(np.asarray(t, dtype=float)[-1]) for t in series_t if len(t) > 0]
    if not last_times:
        raise ValueError("common_time_grid: no non-empty series")
    t_end_s = min(last_times)
    return np.linspace(0.0, t_end_s, n_points)


def ensemble_band(
    cases: Sequence[tuple[Sequence[float] | np.ndarray, Sequence[float] | np.ndarray]],
    *,
    level: float = 0.90,
    n_points: int = 200,
) -> DispersionBand:
    """End-to-end: resample a list of ``(t_s, values)`` cases to a shared grid and band them.

    Convenience wrapper combining :func:`common_time_grid`, :func:`resample_to_grid` and
    :func:`trajectory_band` — the typical entry point when each Monte-Carlo case is a raw
    ``(t, channel)`` pair of differing length. Deterministic.
    """
    if not cases:
        raise ValueError("ensemble_band: no cases")
    grid = common_time_grid([t for t, _ in cases], n_points=n_points)
    stacked = np.vstack([resample_to_grid(t, v, grid) for t, v in cases])
    return trajectory_band(grid, stacked, level=level)


# --------------------------------------------------------------------------------------
# Figures
# --------------------------------------------------------------------------------------


def plot_trajectory_band(band: DispersionBand, *, channel_label: str = "miss range [m]"):
    """Per-time mean trajectory with a shaded percentile dispersion band."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.fill_between(
        band.t_s,
        band.low,
        band.high,
        color="#2ca02c",
        alpha=0.20,
        label=f"{band.level:.0%} band",
    )
    ax.plot(band.t_s, band.mean, "-", color="#2ca02c", lw=1.8, label="ensemble mean")
    ax.plot(band.t_s, band.median, "--", color="#1f77b4", lw=1.2, label="ensemble median")
    ax.set_xlabel("time [s]")
    ax.set_ylabel(channel_label)
    ax.set_title(f"Trajectory dispersion of {channel_label} ({band.n} cases)")
    ax.grid(True, ls="--", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    return fig


def plot_convergence(points: list[ConvergencePoint], *, metric_label: str = "CEP [m]"):
    """Running estimate vs N with a shaded bootstrap CI band."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ns = np.array([p.n for p in points])
    est = np.array([p.estimate for p in points])
    low = np.array([p.low for p in points])
    high = np.array([p.high for p in points])

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.fill_between(ns, low, high, color="#1f77b4", alpha=0.20, label="95% CI band")
    ax.plot(ns, est, "-o", color="#1f77b4", lw=1.8, ms=4, label=metric_label)
    ax.axhline(est[-1], color="#d62728", ls="--", lw=1.2, alpha=0.7, label=f"final = {est[-1]:.2f}")
    ax.set_xscale("log")
    ax.set_xlabel("number of Monte-Carlo cases N")
    ax.set_ylabel(metric_label)
    ax.set_title(f"Monte-Carlo convergence of {metric_label} (running 95% CI)")
    ax.grid(True, ls="--", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    return fig


def plot_sobol(result: SobolResult, *, title: str = "Sobol sensitivity — drivers of miss"):
    """Horizontal bar chart of first- vs total-order Sobol indices, ranked."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    order = np.argsort(result.total_order)  # ascending so largest is on top after barh
    names = [result.names[i] for i in order]
    first = result.first_order[order]
    total = result.total_order[order]

    y = np.arange(len(names))
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.barh(y + 0.18, total, height=0.34, color="#d62728", alpha=0.85, label="total-order $S_{Ti}$")
    ax.barh(y - 0.18, first, height=0.34, color="#1f77b4", alpha=0.85, label="first-order $S_i$")
    ax.set_yticks(y)
    ax.set_yticklabels(names)
    ax.set_xlabel("Sobol index (fraction of output variance)")
    ax.set_title(title)
    ax.grid(True, axis="x", ls="--", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    return fig
