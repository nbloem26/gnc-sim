"""Reference IMU error model — the Python twin of ``core/src/sensors/Sensors.cpp``.

This module implements, per-sample, exactly the noise convention documented in
``docs/DATA_CONTRACT.md`` §5 and reproduced verbatim in the C++ ``Imu::measure``:

For one axis, sampled once per step at fixed ``dt``::

    white  : per-sample draw  N(0, white / sqrt(dt))         # continuous density
    bias   : GM,  bias = bias*exp(-dt/tau)
                       + N(0, bias_instability*sqrt(1 - exp(-2*dt/tau)))
    rrw    : bias += N(0, rrw*sqrt(dt))                       # rate random walk
    measured = true*(1 + scale_factor) + bias + white

On the Allan-deviation log-log plot this produces the canonical three regimes:

    * white noise (ARW/VRW)   -> slope -1/2,  read sigma at tau = 1 s
    * bias instability        -> flat floor,  minimum * 0.664
    * rate random walk (RRW)  -> slope +1/2

Keeping this model here (rather than only in the generator) lets the loop-closure
test re-use the identical math, so any drift between Python and C++ surfaces
immediately.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# Allan-deviation conventional scale factors (Allan 1966; IEEE Std 952-1997).
BIAS_INSTABILITY_SCALE = 0.664  # B = min(ADEV) / 0.664


def gm_allan_variance(tau: np.ndarray | float, sigma_gm: float, tcorr: float) -> np.ndarray:
    """Closed-form *overlapping* Allan variance of a first-order Gauss-Markov process.

    For a GM (exponentially-correlated) process with steady-state std ``sigma_gm`` and
    correlation time ``tcorr`` (El-Sheimy et al. 2008, "Analysis and Modeling of Inertial
    Sensors Using Allan Variance", IEEE T-IM):

        sigma_A^2(tau) = (2 sigma_gm^2 tcorr / tau)
                         * [ 1 - (tcorr/2tau)(3 - 4 e^{-tau/tcorr} + e^{-2tau/tcorr}) ]

    The leading factor 2 matches the *overlapping* Allan estimator (validated against
    simulated GM data — the bare formula without it underestimates by sqrt(2) in
    deviation). The GM hump peaks at tau ~ 1.89 tcorr.
    """
    tau = np.asarray(tau, dtype=float)
    r = tcorr / tau
    var = 2.0 * sigma_gm**2 * r * (
        1.0 - (r / 2.0) * (3.0 - 4.0 * np.exp(-1.0 / r) + np.exp(-2.0 / r))
    )
    return np.clip(var, 0.0, None)


def gm_allan_deviation(tau: np.ndarray | float, sigma_gm: float, tcorr: float) -> np.ndarray:
    """Allan deviation of a first-order GM process (sqrt of :func:`gm_allan_variance`)."""
    return np.sqrt(gm_allan_variance(tau, sigma_gm, tcorr))


def combined_allan_deviation(
    tau: np.ndarray | float, white: float, sigma_gm: float, tcorr: float, rrw: float
) -> np.ndarray:
    """Analytic Allan deviation of white + GM + rate-random-walk, superimposed.

    The three error sources are independent, so their Allan *variances* add:

        sigma^2(tau) = N^2 / tau                      (white / ARW / VRW, slope -1/2)
                       + sigma_GM_allan^2(tau)        (Gauss-Markov bias hump)
                       + K^2 * tau / 3                (rate random walk, slope +1/2)

    This is the curve the synthetic data follows in expectation; the data-driven
    pipeline reads its de-whitened hump peak as the bias-instability number.
    """
    tau = np.asarray(tau, dtype=float)
    var_white = white**2 / tau if white > 0.0 else 0.0
    var_gm = gm_allan_variance(tau, sigma_gm, tcorr) if sigma_gm > 0.0 else 0.0
    var_rrw = rrw**2 * tau / 3.0 if rrw > 0.0 else 0.0
    return np.sqrt(var_white + var_gm + var_rrw)


def true_params_from_model(
    p: "AxisNoise",
    dt: float,
    n: int,
    num_points: int = 120,
    n_realizations: int = 40,
    seed: int = 20240,
) -> dict:
    """Ground-truth (N, B, T, K) characterized from an ENSEMBLE of the actual process.

    Important subtlety: in the C++ sensor (and our twin ``generate_axis``) the rate
    random walk is added on top of the *same* bias state that the Gauss-Markov term
    decays each step. The GM decay therefore bounds the walk — there is no free,
    unbounded +1/2 Allan ramp like the textbook independent-RRW model predicts. So we
    cannot label "true" RRW from the independent-term analytic formula; it would claim a
    ramp the real coupled process never produces.

    Instead we Monte-Carlo the *exact same generator*: average the overlapping Allan
    deviation over ``n_realizations`` independent records (driving estimator variance
    down ~1/sqrt(N)) and run the identical extractor. The result is the population-level
    behaviour of the real process — the honest ground truth the data fit should match.
    Returns ``{white, bias_instability, bias_tau, rrw}``.
    """
    from allan_variance import identify_regimes, overlapping_allan_deviation

    rng = np.random.default_rng(seed)
    taus = edf = None
    adevs = []
    for _ in range(n_realizations):
        x = generate_axis(n, dt, p, rng, true_signal=0.0)
        taus, ad, edf = overlapping_allan_deviation(x, dt, num_points=num_points)
        adevs.append(ad)
    adev = np.mean(adevs, axis=0)
    fit = identify_regimes(taus, adev, edf)
    return {
        "white": fit.white,
        "bias_instability": fit.bias_instability,
        "bias_tau": fit.bias_tau,
        "rrw": fit.rrw,
    }


@dataclass(frozen=True)
class AxisNoise:
    """Continuous-time noise parameters for a single inertial axis.

    Units (accel / gyro):
        white               (m/s^2)/sqrt(Hz)   /  (rad/s)/sqrt(Hz)   [VRW / ARW density]
        bias_instability     m/s^2             /   rad/s              [GM steady-state std]
        bias_tau             s                 /   s                  [GM correlation time]
        rrw                 (m/s^2)/sqrt(s)    /  (rad/s)/sqrt(s)     [random-walk density]
        scale_factor         dimensionless (fractional)
    """

    white: float
    bias_instability: float
    bias_tau: float
    rrw: float
    scale_factor: float = 0.0


def generate_axis(
    n: int,
    dt: float,
    p: AxisNoise,
    rng: np.random.Generator,
    true_signal: np.ndarray | float = 0.0,
) -> np.ndarray:
    """Generate ``n`` samples of a single noisy inertial axis.

    Mirrors the per-sample recurrence in ``stepAxisBias`` + ``Imu::measure``.

    Args:
        n: number of samples.
        dt: sample period [s].
        p: continuous-time noise parameters for this axis.
        rng: seeded numpy Generator (determinism).
        true_signal: the noise-free signal (scalar or length-``n`` array). For a
            static lab record this is 0 (gyro) or g-on-one-axis; here we keep it
            general so the same routine works on flight data.

    Returns:
        measured signal, shape ``(n,)``.
    """
    true_arr = np.broadcast_to(np.asarray(true_signal, dtype=float), (n,)).astype(float)

    # 1. White noise: discrete per-sample std = white / sqrt(dt).
    white_std = p.white / np.sqrt(dt) if p.white > 0.0 else 0.0
    white = rng.normal(0.0, white_std, size=n) if white_std > 0.0 else np.zeros(n)

    # 2 + 3. Gauss-Markov bias instability with a rate-random-walk on top.
    bias = np.zeros(n)
    b = 0.0
    gm_on = p.bias_tau > 0.0 and p.bias_instability > 0.0
    if gm_on:
        phi = np.exp(-dt / p.bias_tau)
        q_gm = p.bias_instability * np.sqrt(1.0 - phi * phi)
        gm_draw = rng.normal(0.0, q_gm, size=n)
    rrw_std = p.rrw * np.sqrt(dt) if p.rrw > 0.0 else 0.0
    rrw_draw = rng.normal(0.0, rrw_std, size=n) if rrw_std > 0.0 else np.zeros(n)

    for i in range(n):
        if gm_on:
            b = b * phi + gm_draw[i]
        b += rrw_draw[i]
        bias[i] = b

    return true_arr * (1.0 + p.scale_factor) + bias + white
