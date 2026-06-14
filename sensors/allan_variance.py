"""Overlapping Allan deviation and three-regime noise identification.

Allan variance is the workhorse for characterizing inertial-sensor noise. Given a
record sampled at rate ``1/dt``, we average the data into bins of length
``m = tau/dt`` and compute the variance of *successive differences* of those bin
averages. The overlapping estimator uses every possible bin start (maximum overlap),
which gives far more degrees of freedom — and a smoother curve — than the
non-overlapping estimator for the same data.

On the resulting sigma(tau) log-log plot:

    slope -1/2  : white noise   (angle/velocity random walk). sigma(tau)=N/sqrt(tau),
                  so the ARW/VRW coefficient N is sigma read at tau = 1 s.
    flat / hump : bias instability. For an ideal flicker-floor IMU this is the flat
                  minimum B = min(sigma)/0.664; for our Gauss-Markov bias model it is a
                  hump peaking ~0.617*sigma_GM at tau ~ 1.89*T. We report the hump peak.
    slope +1/2  : rate random walk. sigma(tau)=K*sqrt(tau/3) — only emerges as a free
                  ramp if the walk is unbounded (see note in generate_imu_data.py).

Identification is by a rigorous nonlinear least-squares fit of the full combined Allan
model (white + GM + RRW), which is robust when the three regimes overlap in tau.

This module provides:
    * ``overlapping_allan_deviation`` — the estimator (returns taus, adev, edf).
    * ``identify_regimes`` — NLS extraction of N (white), B (bias instab.), T, K (RRW).
    * ``plot_allan`` / ``main`` — annotated figure of the characterized record.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def overlapping_allan_deviation(
    x: np.ndarray,
    dt: float,
    num_points: int = 60,
    min_clusters: int = 12,
) -> tuple[np.ndarray, np.ndarray]:
    """Compute the overlapping Allan deviation sigma(tau).

    Args:
        x: 1-D time series sampled at uniform period ``dt``.
        dt: sample period [s].
        num_points: number of (log-spaced) averaging factors to evaluate.
        min_clusters: drop averaging factors that leave fewer than this many
            independent (non-overlapping) clusters ``floor(N/m)``. The longest-tau
            points of any Allan curve are dominated by estimator variance (only a
            handful of clusters); excluding them keeps the recovered slopes honest.

    Returns:
        ``(taus, adev, edf)`` — averaging times [s], Allan deviation, and an
        equivalent-degrees-of-freedom proxy (cluster count) used to weight the fit.
    """
    x = np.asarray(x, dtype=float)
    n = x.size
    if n < 4:
        raise ValueError("need at least 4 samples for Allan variance")

    # theta = integral of the rate signal (cumulative sum). The overlapping Allan
    # variance of the rate x equals the second difference of theta scaled by 1/(2 tau^2).
    theta = np.concatenate(([0.0], np.cumsum(x))) * dt  # length n+1

    # Averaging factors m (samples per bin). Cap m so each tau retains at least
    # ``min_clusters`` non-overlapping clusters (floor(n/m) >= min_clusters).
    max_m = max(1, n // max(1, min_clusters))
    if max_m < 1:
        raise ValueError("series too short for any averaging factor")
    m_values = np.unique(
        np.floor(np.logspace(0, np.log10(max_m), num_points)).astype(int)
    )
    m_values = m_values[m_values >= 1]

    taus = m_values * dt
    adev = np.empty(m_values.size)
    edf = np.empty(m_values.size)  # ~ equivalent degrees of freedom per point
    for k, m in enumerate(m_values):
        tau = m * dt
        # Overlapping estimator (Riley, NIST SP 1065, eq. for AVAR from phase data):
        #   sigma^2 = 1 / (2 tau^2 (N - 2m)) * sum_{i=0}^{N-2m-1}
        #             (theta[i+2m] - 2 theta[i+m] + theta[i])^2
        diffs = theta[2 * m :] - 2.0 * theta[m : -m] + theta[: -2 * m]
        denom = 2.0 * tau * tau * diffs.size
        avar = np.sum(diffs * diffs) / denom
        adev[k] = np.sqrt(avar)
        # Simple, robust EDF proxy: number of independent (non-overlapping) clusters.
        # Long-tau points have few clusters -> low EDF -> down-weighted in the fit.
        edf[k] = max(1.0, n / m - 1.0)

    return taus, adev, edf


@dataclass(frozen=True)
class RegimeFit:
    """Recovered noise coefficients from an Allan-deviation curve.

    All quantities are continuous-time densities matching the C++ ``Imu`` config:
        white            N  — ARW/VRW density (sigma at tau = 1 s on a pure -1/2 line)
        bias_instability B  — best in-run stability: the GM Allan hump-peak deviation
        bias_tau         T  — Gauss-Markov correlation time [s]
        rrw              K  — rate-random-walk density
        sigma_gm            — fitted GM steady-state std (B = hump peak derived from this)
    """

    white: float
    bias_instability: float
    bias_tau: float
    rrw: float
    sigma_gm: float = 0.0


def _combined_avar(tau: np.ndarray, white: float, sigma_gm: float, tcorr: float, rrw: float):
    """Combined Allan *variance* model: white + Gauss-Markov + rate random walk.

    sigma^2(tau) = N^2/tau
                   + 2 sigma_GM^2 (T/tau)[1 - (T/2tau)(3 - 4 e^{-tau/T} + e^{-2tau/T})]
                   + K^2 tau/3

    The GM term carries the factor 2 of the overlapping Allan estimator (see
    ``noise_model.gm_allan_variance``).
    """
    r = tcorr / tau
    gm = 2.0 * sigma_gm**2 * r * (
        1.0 - (r / 2.0) * (3.0 - 4.0 * np.exp(-1.0 / r) + np.exp(-2.0 / r))
    )
    return white**2 / tau + np.clip(gm, 0.0, None) + rrw**2 * tau / 3.0


def _gm_hump_peak(sigma_gm: float, tcorr: float) -> float:
    """Peak Allan deviation of the GM term alone — the reported bias-instability B."""
    if sigma_gm <= 0.0 or tcorr <= 0.0:
        return 0.0
    tau = np.logspace(np.log10(tcorr) - 1.5, np.log10(tcorr) + 1.5, 2000)
    gm = _combined_avar(tau, 0.0, sigma_gm, tcorr, 0.0)
    return float(np.sqrt(np.max(np.clip(gm, 0.0, None))))


def identify_regimes(
    taus: np.ndarray, adev: np.ndarray, edf: np.ndarray | None = None
) -> RegimeFit:
    """Recover (N, B, T, K) by nonlinear least-squares fit of the combined Allan model.

    Rather than read each regime off a hand-windowed slope (brittle when the three
    processes overlap in tau), we fit the *whole* analytic Allan curve — white +
    Gauss-Markov bias + rate random walk — to the measured deviation. This is the
    standard rigorous IMU-characterization method and, crucially, returns the same
    answer on a clean analytic curve as on a noisy measured one (so recovered-vs-true
    is apples-to-apples). The fit runs in log space so all decades weigh equally; seeds
    come from cheap slope reads.

    ``edf`` (equivalent degrees of freedom, ~cluster count) optionally weights the fit so
    the high-variance long-tau tail — which otherwise corrupts the RRW estimate on a
    single record — is trusted less. Without it every point weighs equally.

    bias_instability B is the GM hump-peak deviation implied by the fitted (sigma_GM, T)
    — the best in-run stability the sensor reaches near its correlation time.
    """
    from scipy.optimize import curve_fit  # local import keeps module import light

    taus = np.asarray(taus, dtype=float)
    adev = np.asarray(adev, dtype=float)
    log_tau = np.log10(taus)
    log_adev = np.log10(adev)
    slope = np.gradient(log_adev, log_tau)

    # --- Seed values from quick slope reads ---
    # White: -1/2 line intercept at tau = 1 s, from the short-tau region.
    short = log_tau < np.median(log_tau)
    if short.sum() >= 2:
        white0 = 10.0 ** np.mean(log_adev[short] + 0.5 * log_tau[short])
    else:
        white0 = float(adev[np.argmin(np.abs(taus - 1.0))])
    # Tau seed: tau of the flattest interior point / 1.89 (GM hump ~ 1.89 T).
    lo, hi = 2, len(taus) - 3
    i_flat = lo + int(np.argmin(np.abs(slope[lo:hi]))) if hi > lo else len(taus) // 2
    tcorr0 = max(taus[i_flat] / 1.89, taus[1])
    sigma_gm0 = max(adev[i_flat], white0 / np.sqrt(taus[i_flat])) / 0.4365
    rrw0 = max(adev[-1] * np.sqrt(3.0 / taus[-1]), 1e-12)

    # --- Nonlinear fit of the variance model in log space (equal weight per decade) ---
    def model_log(t, white, sigma_gm, tcorr, rrw):
        return 0.5 * np.log10(_combined_avar(t, abs(white), abs(sigma_gm), abs(tcorr), abs(rrw)))

    p0 = [white0, sigma_gm0, tcorr0, rrw0]
    # Per-point sigma in log space ~ 1/sqrt(2*EDF): the relative error of an Allan
    # deviation estimate scales like 1/sqrt(EDF), and d(log10 sigma) ~ relative error.
    sigma_log = None
    if edf is not None:
        sigma_log = 1.0 / np.sqrt(2.0 * np.maximum(np.asarray(edf, dtype=float), 1.0))
    try:
        popt, _ = curve_fit(
            model_log, taus, log_adev, p0=p0, sigma=sigma_log, absolute_sigma=False,
            maxfev=20000,
            bounds=([0, 0, taus[1], 0], [np.inf, np.inf, taus[-1] * 2, np.inf]),
        )
        white, sigma_gm, tcorr, rrw = (abs(v) for v in popt)
    except Exception:
        white, sigma_gm, tcorr, rrw = white0, sigma_gm0, tcorr0, rrw0

    return RegimeFit(
        white=float(white),
        bias_instability=float(_gm_hump_peak(sigma_gm, tcorr)),
        bias_tau=float(tcorr),
        rrw=float(rrw),
        sigma_gm=float(sigma_gm),
    )


def plot_allan(
    channels: dict,
    out_path,
    title: str = "IMU Allan Deviation",
):
    """Render an annotated Allan-deviation log-log plot.

    Args:
        channels: ``{label: (taus, adev, RegimeFit)}`` — one curve per channel.
        out_path: PNG path to write.
        title: figure title.
    """
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8.5, 6.0))
    colors = {"accel": "#1f77b4", "gyro": "#d62728"}
    for label, (taus, adev, fit) in channels.items():
        color = colors.get(label.split()[0].lower(), None)
        ax.loglog(taus, adev, "o", ms=3, color=color, label=f"{label} (measured)")
        # Fitted combined model overlay.
        model = np.sqrt(_combined_avar(taus, fit.white, fit.sigma_gm, fit.bias_tau, fit.rrw))
        ax.loglog(taus, model, "-", lw=1.5, color=color, alpha=0.8)

        # White -1/2 guide through tau = 1 s.
        guide_tau = np.array([taus[0], taus[-1]])
        ax.loglog(guide_tau, fit.white / np.sqrt(guide_tau), ":", color=color, alpha=0.5)
        # Bias-instability marker at the GM hump.
        hump = _gm_hump_peak(fit.sigma_gm, fit.bias_tau)
        tau_peak = 1.89 * fit.bias_tau
        ax.plot(tau_peak, hump, "s", color=color, ms=8, mfc="none", mew=1.6)
        ax.annotate(
            f"B={hump:.2e}",
            (tau_peak, hump),
            textcoords="offset points",
            xytext=(6, 8),
            fontsize=8,
            color=color,
        )

    # Slope reference triangles (-1/2 and +1/2).
    ax.text(0.02, 0.06, "slope -1/2: white (ARW/VRW)\nflat: bias instability\nslope +1/2: RRW",
            transform=ax.transAxes, fontsize=9, va="bottom",
            bbox=dict(boxstyle="round", fc="white", ec="0.7", alpha=0.85))

    ax.set_xlabel(r"averaging time $\tau$ [s]")
    ax.set_ylabel(r"Allan deviation $\sigma(\tau)$")
    ax.set_title(title)
    ax.grid(True, which="both", ls="--", alpha=0.3)
    ax.legend(loc="upper right", fontsize=9)
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def main() -> None:
    """Characterize ``sensors/imu_raw.npz`` and write the annotated Allan-deviation figure."""
    from pathlib import Path

    from characterize import characterize_axes

    sensors_dir = Path(__file__).resolve().parent
    raw = np.load(sensors_dir / "imu_raw.npz")
    dt = float(raw["dt"])

    channels = {}
    for ch in ("accel", "gyro"):
        taus, adev, _edf, fit = characterize_axes(raw[ch], dt)
        channels[ch] = (taus, adev, fit)
        print(f"{ch}: N={fit.white:.3e}  B={fit.bias_instability:.3e}  "
              f"T={fit.bias_tau:.0f}s  K={fit.rrw:.3e}  (sigma_GM={fit.sigma_gm:.3e})")

    fig_dir = sensors_dir / "figures"
    fig_dir.mkdir(exist_ok=True)
    out = fig_dir / "allan_deviation.png"
    plot_allan(channels, out, title="Static IMU Allan Deviation — 3-axis mean")
    print(f"wrote {out.relative_to(sensors_dir.parent)}")


if __name__ == "__main__":
    main()
