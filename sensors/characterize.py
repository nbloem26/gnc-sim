"""Shared characterization helper: multi-axis Allan deviation + regime fit.

Used by ``fit_noise_params.py`` (synthetic recovery), the loop-closure validation
(``postproc/gncpost/loop_closure.py``), and the tests, so every consumer characterizes
a sensor the exact same way. Averaging the per-axis Allan curves (3 independent axes)
cuts estimator variance ~sqrt(3), tightening the bias-instability / RRW recovery.
"""

from __future__ import annotations

import numpy as np
from allan_variance import RegimeFit, identify_regimes, overlapping_allan_deviation


def characterize_axes(
    samples: np.ndarray, dt: float, num_points: int = 120
) -> tuple[np.ndarray, np.ndarray, np.ndarray, RegimeFit]:
    """Characterize a multi-axis static record.

    Args:
        samples: shape ``(n,)`` or ``(n, k)`` — one column per axis.
        dt: sample period [s].
        num_points: log-spaced averaging factors.

    Returns:
        ``(taus, adev_mean, edf, fit)`` — the common tau grid, the axis-averaged Allan
        deviation, the (shared) EDF weights, and the recovered ``RegimeFit``.
    """
    samples = np.asarray(samples, dtype=float)
    if samples.ndim == 1:
        samples = samples[:, None]

    taus = None
    edf = None
    adevs = []
    for col in range(samples.shape[1]):
        t, ad, e = overlapping_allan_deviation(samples[:, col], dt, num_points=num_points)
        taus, edf = t, e  # identical across axes (same n, dt, grid)
        adevs.append(ad)
    adev_mean = np.mean(adevs, axis=0)

    assert taus is not None and edf is not None  # samples is reshaped to >=1 column
    fit = identify_regimes(taus, adev_mean, edf)
    return taus, adev_mean, edf, fit
