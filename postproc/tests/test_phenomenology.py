"""Sensor-phenomenology analytic-twin tests (issue #39).

Pure-Python (no native build): assert the CA-CFAR closed forms behave and that the Monte-Carlo
ensemble in :mod:`gncpost.phenomenology` confirms them — the same relations the C++
``core/src/sensors/Phenomenology.cpp`` implements (cross-implementation check).
"""

from __future__ import annotations

import math

from gncpost import phenomenology as ph


def test_cfar_alpha_matches_gandhi_kassam() -> None:
    n = 16
    pfa = 1e-4
    expected = n * (pfa ** (-1.0 / n) - 1.0)
    assert math.isclose(float(ph.cfar_alpha(pfa, n)), expected, rel_tol=1e-12)


def test_cfar_pd_collapses_to_pfa_at_zero_snr() -> None:
    # At SNR = 0 the cell-under-test is statistically a reference cell: Pd == Pfa.
    for pfa in (1e-2, 1e-4, 1e-6):
        pd0 = float(ph.cfar_pd(pfa, 24, 0.0))
        assert math.isclose(pd0, pfa, rel_tol=1e-6)


def test_cfar_pd_monotonic_in_snr() -> None:
    prev = -1.0
    for snr_db in range(-10, 26):
        pd = float(ph.cfar_pd(1e-4, 24, ph.db_to_linear(snr_db)))
        assert pd >= prev - 1e-12
        prev = pd


def test_radar_snr_inverse_fourth_power() -> None:
    s_ref = ph.radar_snr_linear(1.0e4, 1.0, snr_ref_db=20.0)
    assert math.isclose(s_ref, 100.0, rel_tol=1e-9)
    s_far = ph.radar_snr_linear(2.0e4, 1.0, snr_ref_db=20.0)
    assert math.isclose(s_far, 100.0 / 16.0, rel_tol=1e-9)


def test_roc_monte_carlo_confirms_closed_form() -> None:
    roc = ph.compute_roc(snr_db_list=(10.0, 16.0), mc_looks=200_000, seed=39)
    # Noise-only false-alarm rate lands on the design Pfa (binomial std at 1e-4 over 2e5 ~ 2e-5).
    assert abs(roc.mc_noise_pfa - roc.design_pfa) < 5e-4
    # Monte-Carlo Pd matches the analytic Pd at each SNR.
    for snr_db, pd_mc in roc.mc_pd.items():
        pd_analytic = float(ph.cfar_pd(roc.design_pfa, roc.num_ref_cells, ph.db_to_linear(snr_db)))
        assert abs(pd_mc - pd_analytic) < 5e-3


def test_swerling_sample_means() -> None:
    import numpy as np

    rng = np.random.default_rng(1)
    for case in (0, 1, 2, 3, 4):
        s = ph.swerling_samples(case, 2.0, 200_000, rng)
        assert abs(float(s.mean()) - 2.0) < 0.05
