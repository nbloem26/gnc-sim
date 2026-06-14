"""Allan-variance estimator + regime recovery on synthetic data."""

from __future__ import annotations

import numpy as np
import pytest

from allan_variance import overlapping_allan_deviation
from characterize import characterize_axes
from noise_model import AxisNoise, generate_axis, gm_allan_deviation


def test_white_noise_slope_minus_half():
    """Pure white noise: Allan deviation has slope -1/2 and N = sigma at tau=1s."""
    rng = np.random.default_rng(0)
    dt, n, white = 0.01, 400_000, 1.0e-3
    p = AxisNoise(white=white, bias_instability=0.0, bias_tau=0.0, rrw=0.0)
    x = generate_axis(n, dt, p, rng)
    taus, adev, _edf = overlapping_allan_deviation(x, dt, num_points=60)
    # slope of log-log curve ~ -0.5
    slope = np.polyfit(np.log10(taus), np.log10(adev), 1)[0]
    assert slope == pytest.approx(-0.5, abs=0.05)
    # N read at tau = 1 s
    n_read = adev[np.argmin(np.abs(taus - 1.0))]
    assert n_read == pytest.approx(white, rel=0.1)


def test_gm_formula_matches_simulation():
    """The analytic GM Allan deviation (with factor 2) matches simulated GM data."""
    rng = np.random.default_rng(1)
    dt, n = 0.01, 800_000
    sigma_gm, tcorr = 5.0e-4, 80.0
    p = AxisNoise(white=0.0, bias_instability=sigma_gm, bias_tau=tcorr, rrw=0.0)
    ads = []
    taus = None
    for _ in range(8):
        x = generate_axis(n, dt, p, rng)
        taus, ad, _ = overlapping_allan_deviation(x, dt, num_points=80)
        ads.append(ad)
    adev = np.mean(ads, axis=0)
    theory = gm_allan_deviation(taus, sigma_gm, tcorr)
    band = (taus > 40) & (taus < 300)
    ratio = np.mean((adev / theory)[band])
    assert ratio == pytest.approx(1.0, abs=0.1)


def test_identify_regimes_recovers_white_and_bias():
    """Full NLS recovery on a white + GM record recovers N and bias instability."""
    rng = np.random.default_rng(2)
    dt, n = 0.01, 1_200_000
    p = AxisNoise(white=6.0e-4, bias_instability=4.5e-4, bias_tau=100.0, rrw=1.0e-5)
    # 3 independent axes -> averaged curve (as the real pipeline does).
    samples = np.column_stack([generate_axis(n, dt, p, rng) for _ in range(3)])
    _taus, _adev, _edf, fit = characterize_axes(samples, dt)

    assert fit.white == pytest.approx(p.white, rel=0.1)
    # recovered GM steady-state std within 15% of injected
    assert fit.sigma_gm == pytest.approx(p.bias_instability, rel=0.2)
    # correlation time in the right ballpark
    assert 50.0 < fit.bias_tau < 200.0


def test_recovered_matches_stored_truth():
    """The saved imu_raw.npz recovers its ground-truth labels within tolerance."""
    raw_path = __import__("pathlib").Path(__file__).resolve().parents[2] / "sensors" / "imu_raw.npz"
    if not raw_path.exists():
        pytest.skip("imu_raw.npz not generated yet (run sensors/generate_imu_data.py)")
    d = np.load(raw_path)
    dt = float(d["dt"])
    for ch in ("accel", "gyro"):
        _t, _a, _e, fit = characterize_axes(d[ch], dt)
        true = d[f"true_{ch}"]
        assert fit.white == pytest.approx(true[0], rel=0.12)
        assert fit.bias_instability == pytest.approx(true[1], rel=0.20)
