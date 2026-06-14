"""Monte-Carlo CEP / dispersion statistics."""

from __future__ import annotations

import numpy as np
import pandas as pd
import pytest

from gncpost.montecarlo import compute_stats


def _summary(miss, intercept=None):
    n = len(miss)
    data = {"case": range(n), "seed": range(n), "miss_distance": miss,
            "intercept_time": [4.0] * n}
    if intercept is not None:
        data["intercept"] = intercept
    return pd.DataFrame(data)


def test_cep_is_median():
    miss = [1.0, 2.0, 3.0, 4.0, 5.0]
    stats = compute_stats(_summary(miss))
    assert stats.cep == pytest.approx(3.0)  # median
    assert stats.mean == pytest.approx(3.0)
    assert stats.n == 5


def test_cep_rayleigh_relation():
    """For a 2-D Gaussian miss, CEP ~ 1.1774 * sigma (Rayleigh median)."""
    rng = np.random.default_rng(0)
    sigma = 5.0
    ex = rng.normal(0, sigma, 20000)
    ey = rng.normal(0, sigma, 20000)
    miss = np.sqrt(ex**2 + ey**2)
    stats = compute_stats(_summary(miss))
    assert stats.cep == pytest.approx(1.1774 * sigma, rel=0.05)


def test_intercept_rate_encodings():
    assert compute_stats(_summary([1, 2], [1, 0])).intercept_rate == pytest.approx(0.5)
    assert compute_stats(_summary([1, 2], ["yes", "no"])).intercept_rate == pytest.approx(0.5)
    assert compute_stats(_summary([1, 2], [True, True])).intercept_rate == pytest.approx(1.0)


def test_handles_nonfinite():
    stats = compute_stats(_summary([1.0, np.nan, 3.0]))
    assert stats.n == 2


def test_empty_raises():
    with pytest.raises(ValueError):
        compute_stats(_summary([np.nan, np.inf]))
