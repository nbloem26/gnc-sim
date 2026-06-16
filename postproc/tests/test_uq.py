"""Uncertainty-quantification tests (issue #33): bootstrap CIs, convergence, Sobol.

All tests are seeded; no bare ``np.random`` and no native-CLI dependency, so they
run fast in CI.
"""

from __future__ import annotations

import numpy as np
import pytest
from gncpost.uq import (
    ConfidenceInterval,
    bootstrap_ci,
    cep_stat,
    convergence,
    ishigami,
    ishigami_analytic_indices,
    pkill_stat,
    sobol_indices,
)

# --------------------------------------------------------------------------------------
# Bootstrap confidence intervals
# --------------------------------------------------------------------------------------


def test_bootstrap_ci_covers_known_median():
    """A Rayleigh sample's CEP (median) should lie inside its bootstrap CI."""
    rng = np.random.default_rng(0)
    sigma = 5.0
    miss = np.sqrt(rng.normal(0, sigma, 4000) ** 2 + rng.normal(0, sigma, 4000) ** 2)
    true_cep = 1.1774 * sigma  # Rayleigh median
    ci = bootstrap_ci(miss, cep_stat, level=0.95, seed=42)
    assert ci.low <= true_cep <= ci.high
    assert ci.half_width > 0.0


def test_bootstrap_ci_coverage_rate():
    """Across many synthetic campaigns the 90% CI should cover the truth ~90% of the time."""
    truth = 0.5  # mean of a Bernoulli(0.5) == P(kill)
    rng = np.random.default_rng(7)
    covered = 0
    trials = 200
    for t in range(trials):
        sample = rng.binomial(1, truth, size=300).astype(float)
        ci = bootstrap_ci(sample, pkill_stat, level=0.90, n_resamples=800, seed=t)
        if ci.low <= truth <= ci.high:
            covered += 1
    rate = covered / trials
    # Generous band around nominal 0.90 for finite trials.
    assert 0.80 <= rate <= 0.98


def test_bootstrap_ci_deterministic():
    rng = np.random.default_rng(0)
    data = rng.normal(10.0, 2.0, 400)
    a = bootstrap_ci(data, cep_stat, seed=123)
    b = bootstrap_ci(data, cep_stat, seed=123)
    assert (a.low, a.high, a.estimate) == (b.low, b.high, b.estimate)
    # Different seed -> (almost surely) different interval on a continuous sample.
    c = bootstrap_ci(data, cep_stat, seed=999)
    assert (a.low, a.high) != (c.low, c.high)


def test_bootstrap_ci_halfwidth_str():
    ci = ConfidenceInterval(estimate=10.0, low=8.0, high=12.0, level=0.95)
    assert ci.half_width == pytest.approx(2.0)
    assert "95% CI" in str(ci)


def test_bootstrap_ci_rejects_empty():
    with pytest.raises(ValueError):
        bootstrap_ci([np.nan, np.inf], cep_stat)


# --------------------------------------------------------------------------------------
# Convergence diagnostic
# --------------------------------------------------------------------------------------


def test_convergence_band_tightens_with_n():
    rng = np.random.default_rng(1)
    miss = np.abs(rng.normal(0, 4.0, 2000))
    pts = convergence(miss, cep_stat, seed=3)
    assert pts[0].n < pts[-1].n
    first_w = pts[0].high - pts[0].low
    last_w = pts[-1].high - pts[-1].low
    # CI band should shrink as more cases are included.
    assert last_w < first_w
    # Every point's estimate sits inside its own band.
    for p in pts:
        assert p.low <= p.estimate <= p.high


def test_convergence_deterministic():
    rng = np.random.default_rng(2)
    miss = np.abs(rng.normal(0, 4.0, 500))
    a = convergence(miss, cep_stat, seed=5)
    b = convergence(miss, cep_stat, seed=5)
    assert [(p.n, p.low, p.high, p.estimate) for p in a] == [
        (p.n, p.low, p.high, p.estimate) for p in b
    ]


# --------------------------------------------------------------------------------------
# Sobol indices on the Ishigami benchmark
# --------------------------------------------------------------------------------------


def test_sobol_recovers_ishigami_ranking():
    bounds = [(-np.pi, np.pi)] * 3
    names = ["x1", "x2", "x3"]
    res = sobol_indices(ishigami, bounds, names, n_base=16384, seed=0)

    first_true, total_true = ishigami_analytic_indices()

    # Total-order ranking for the standard Ishigami (a=7, b=0.1) is x1 > x2 > x3.
    assert [n for n, _ in res.ranking()] == ["x1", "x2", "x3"]
    # First-order ranking is x2 > x1 > x3.
    first_order = np.argsort(res.first_order)[::-1]
    assert [names[i] for i in first_order] == ["x2", "x1", "x3"]

    # Quantitative recovery within tolerance of the closed form.
    assert res.first_order == pytest.approx(first_true, abs=0.03)
    assert res.total_order == pytest.approx(total_true, abs=0.03)
    # x3 has zero first-order influence but nonzero total (interaction with x1).
    assert res.first_order[2] < 0.03
    assert res.total_order[2] > 0.10


def test_sobol_deterministic():
    bounds = [(-np.pi, np.pi)] * 3
    names = ["x1", "x2", "x3"]
    a = sobol_indices(ishigami, bounds, names, n_base=2048, seed=11)
    b = sobol_indices(ishigami, bounds, names, n_base=2048, seed=11)
    assert np.array_equal(a.first_order, b.first_order)
    assert np.array_equal(a.total_order, b.total_order)


def test_sobol_constant_model_zero_indices():
    def constant(x: np.ndarray) -> np.ndarray:
        return np.ones(np.atleast_2d(x).shape[0])

    res = sobol_indices(constant, [(0.0, 1.0)] * 2, ["a", "b"], n_base=256, seed=0)
    assert np.all(res.first_order == 0.0)
    assert np.all(res.total_order == 0.0)


def test_sobol_additive_first_order_sums_to_one():
    """Purely additive model: first-order indices partition the variance (sum ~ 1)."""

    def additive(x: np.ndarray) -> np.ndarray:
        x = np.atleast_2d(x)
        # Variances 9 : 4 : 1 -> indices ~ 0.643 : 0.286 : 0.071.
        return 3.0 * x[:, 0] + 2.0 * x[:, 1] + 1.0 * x[:, 2]

    res = sobol_indices(additive, [(0.0, 1.0)] * 3, ["a", "b", "c"], n_base=8192, seed=4)
    assert res.first_order.sum() == pytest.approx(1.0, abs=0.05)
    assert res.total_order.sum() == pytest.approx(1.0, abs=0.05)
    assert [n for n, _ in res.ranking()] == ["a", "b", "c"]


# --------------------------------------------------------------------------------------
# CI integration into MonteCarloStats
# --------------------------------------------------------------------------------------


def test_montecarlo_stats_reports_ci():
    import pandas as pd
    from gncpost.montecarlo import compute_stats

    rng = np.random.default_rng(0)
    miss = np.abs(rng.normal(0, 5.0, 200))
    inter = (miss < 3.0).astype(int)
    df = pd.DataFrame({"miss_distance": miss, "intercept": inter})
    stats = compute_stats(df)
    assert np.isfinite(stats.cep_ci_halfwidth) and stats.cep_ci_halfwidth > 0
    assert np.isfinite(stats.pkill_ci_halfwidth) and stats.pkill_ci_halfwidth > 0
    line = stats.summary_line()
    assert "+/-" in line and "CI" in line
    # Determinism of the reported half-width.
    assert compute_stats(df).cep_ci_halfwidth == stats.cep_ci_halfwidth
