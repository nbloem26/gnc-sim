"""Sensor-phenomenology analytic twin + figures (issue #39).

This is the Python analytic leg for the ``radar_pheno`` / ``ir_pheno`` models. It re-derives,
in pure NumPy, the *same* closed-form relations the C++ ``core/src/sensors/Phenomenology.cpp``
implements — CA-CFAR threshold/Pd/Pfa, the Swerling RCS distributions, the radar range-equation
SNR, and the IR NETD/atmosphere contrast SNR — and produces two validation figures:

* ``phenomenology_roc.png`` — detection ROC: Pd vs Pfa across a sweep of single-pulse SNRs,
  with an overlaid Monte-Carlo ensemble confirming the analytic curve, plus the empirical
  noise-only false-alarm rate landing on the design Pfa.
* ``range_doppler_map.png`` — a synthetic range-Doppler map: a Swerling-fluctuating target return
  in a noise + clutter-ridge background, thresholded by CA-CFAR (the detected cell is circled).

The closed forms here are kept byte-for-concept identical to the C++ so the figure doubles as a
cross-implementation check. Run from repo root::

    python postproc/gncpost/phenomenology.py
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

if __package__ in (None, ""):
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from gncpost import REPO_ROOT
else:
    from . import REPO_ROOT

# ---------------------------------------------------------------------------------------------
# CA-CFAR closed forms (mirror of core/src/sensors/Phenomenology.cpp)
# ---------------------------------------------------------------------------------------------


def cfar_alpha(pfa: np.ndarray | float, num_ref_cells: int) -> np.ndarray:
    """CA-CFAR threshold multiplier alpha = N (Pfa^(-1/N) - 1) (Gandhi & Kassam)."""
    n = max(num_ref_cells, 1)
    pfa_c = np.clip(np.asarray(pfa, dtype=float), 1e-12, 1.0 - 1e-12)
    return n * (pfa_c ** (-1.0 / n) - 1.0)


def cfar_pd(
    pfa: np.ndarray | float, num_ref_cells: int, snr_linear: np.ndarray | float
) -> np.ndarray:
    """Swerling-I/II CA-CFAR detection probability Pd = (1 + alpha/(N(1+SNR)))^(-N)."""
    n = max(num_ref_cells, 1)
    alpha = cfar_alpha(pfa, n)
    snr = np.maximum(np.asarray(snr_linear, dtype=float), 0.0)
    return (1.0 + alpha / (n * (1.0 + snr))) ** (-n)


def db_to_linear(db: np.ndarray | float) -> np.ndarray:
    return 10.0 ** (0.1 * np.asarray(db, dtype=float))


# ---------------------------------------------------------------------------------------------
# Signal models (mirror of core/src/sensors/Phenomenology.cpp)
# ---------------------------------------------------------------------------------------------


def radar_snr_linear(
    range_m: float,
    rcs_m2: float,
    *,
    range_ref_m: float = 1.0e4,
    rcs_ref_m2: float = 1.0,
    snr_ref_db: float = 20.0,
    clutter_cnr_db: float = -100.0,
    jammer_jnr_db: float = -100.0,
) -> float:
    snr = float(db_to_linear(snr_ref_db)) * (rcs_m2 / rcs_ref_m2) * (range_ref_m / range_m) ** 4
    interference = 1.0
    if clutter_cnr_db > -90.0:
        interference += float(db_to_linear(clutter_cnr_db))
    if jammer_jnr_db > -90.0:
        interference += float(db_to_linear(jammer_jnr_db))
    return float(snr / interference)


def swerling_samples(case: int, mean_rcs_m2: float, n: int, rng: np.random.Generator) -> np.ndarray:
    """Draw n instantaneous RCS values for the given Swerling case (matches the C++ sampler)."""
    if case == 0:
        return np.full(n, mean_rcs_m2)
    if case in (1, 2):  # exponential / chi-square 2 dof
        return mean_rcs_m2 * rng.exponential(1.0, size=n)
    # chi-square 4 dof = mean of two unit-mean exponentials
    return mean_rcs_m2 * 0.5 * (rng.exponential(1.0, n) + rng.exponential(1.0, n))


# ---------------------------------------------------------------------------------------------
# ROC / Pd-vs-Pfa validation
# ---------------------------------------------------------------------------------------------


@dataclass
class RocResult:
    pfa_axis: np.ndarray  # design Pfa sweep
    pd_curves: dict[float, np.ndarray]  # snr_db -> Pd(Pfa)
    mc_noise_pfa: float  # empirical noise-only false-alarm rate at the design Pfa marker
    mc_pd: dict[float, float]  # snr_db -> empirical Pd at the design Pfa marker
    design_pfa: float
    num_ref_cells: int


def compute_roc(
    snr_db_list: tuple[float, ...] = (6.0, 10.0, 13.0, 16.0),
    num_ref_cells: int = 24,
    design_pfa: float = 1e-4,
    mc_looks: int = 400_000,
    seed: int = 39,
) -> RocResult:
    """Analytic ROC + a Monte-Carlo ensemble confirming Pfa (noise-only) and Pd at a marker SNR."""
    pfa_axis = np.logspace(-7, -1, 60)
    pd_curves = {
        snr_db: cfar_pd(pfa_axis, num_ref_cells, db_to_linear(snr_db)) for snr_db in snr_db_list
    }

    # Monte-Carlo confirmation at the design Pfa marker. We draw exponential (square-law, Swerling
    # I/II) cell power for noise-only and for signal-present cells, threshold against the analytic
    # CA-CFAR threshold relative to a unit-mean reference window, and count detections.
    rng = np.random.default_rng(seed)
    # cfar_alpha is the multiplier on the *mean* of the N reference cells (Zbar = Z/N), so the
    # threshold is T = alpha * Zbar = alpha * (Gamma(N,1)/N). Cell power ~ Exp(1) (noise) or
    # Exp(1+SNR) (Swerling-II signal+noise).
    alpha = float(cfar_alpha(design_pfa, num_ref_cells))
    n = num_ref_cells
    zbar_noise = rng.gamma(n, 1.0, size=mc_looks) / n
    cut_noise = rng.exponential(1.0, size=mc_looks)
    mc_noise_pfa = float(np.mean(cut_noise > alpha * zbar_noise))

    mc_pd: dict[float, float] = {}
    for snr_db in snr_db_list:
        snr = db_to_linear(snr_db)
        zbar = rng.gamma(n, 1.0, size=mc_looks) / n
        cut = rng.exponential(1.0 + snr, size=mc_looks)  # signal+noise power, Swerling II
        mc_pd[snr_db] = float(np.mean(cut > alpha * zbar))

    return RocResult(pfa_axis, pd_curves, mc_noise_pfa, mc_pd, design_pfa, num_ref_cells)


def summarize_roc(roc: RocResult) -> list[str]:
    lines = [
        f"CA-CFAR  N={roc.num_ref_cells}  design Pfa={roc.design_pfa:.1e}",
        f"  noise-only empirical Pfa = {roc.mc_noise_pfa:.3e} (design {roc.design_pfa:.1e})",
    ]
    for snr_db, pd_mc in roc.mc_pd.items():
        pd_analytic = float(cfar_pd(roc.design_pfa, roc.num_ref_cells, db_to_linear(snr_db)))
        lines.append(f"  SNR={snr_db:5.1f} dB:  Pd analytic={pd_analytic:.4f}  MC={pd_mc:.4f}")
    return lines


# ---------------------------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------------------------


def plot_roc(roc: RocResult):
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(6.4, 4.6))
    for snr_db, pd in roc.pd_curves.items():
        ax.semilogx(roc.pfa_axis, pd, label=f"SNR = {snr_db:.0f} dB (analytic)")
    # Monte-Carlo markers at the design Pfa.
    mc_vals = list(roc.mc_pd.values())
    ax.semilogx(
        [roc.design_pfa] * len(mc_vals),
        mc_vals,
        "ko",
        ms=6,
        mfc="none",
        label="Monte-Carlo (@ design Pfa)",
    )
    ax.axvline(roc.design_pfa, color="0.6", ls="--", lw=1)
    ax.text(
        roc.design_pfa,
        0.02,
        f"  design Pfa = {roc.design_pfa:.0e}\n  empirical = {roc.mc_noise_pfa:.1e}",
        fontsize=8,
        va="bottom",
    )
    ax.set_xlabel("Probability of false alarm  Pfa")
    ax.set_ylabel("Probability of detection  Pd")
    ax.set_title(f"CA-CFAR ROC (Swerling II, N={roc.num_ref_cells} reference cells)")
    ax.set_ylim(0, 1.02)
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    return fig


def plot_range_doppler(seed: int = 39):
    """Synthetic range-Doppler map: Swerling target return in noise + a clutter ridge, CA-CFAR'd."""
    import matplotlib.pyplot as plt

    rng = np.random.default_rng(seed)
    n_range, n_dopp = 64, 64
    # Exponential (square-law) noise floor.
    rd = rng.exponential(1.0, size=(n_range, n_dopp))
    # Zero-Doppler clutter ridge (stationary clutter) — a few central Doppler bins.
    clutter_cnr = 25.0
    notch_lo, notch_hi = n_dopp // 2 - 2, n_dopp // 2 + 3
    rd[:, notch_lo:notch_hi] += rng.exponential(
        float(db_to_linear(clutter_cnr)), size=(n_range, notch_hi - notch_lo)
    )
    # Moving target: a Swerling-II fluctuating return well clear of zero Doppler. Drawn until it is
    # a credibly strong return (mean 16 dB) for a clean illustration.
    tgt_r, tgt_d = 40, 44
    snr_lin = float(db_to_linear(18.0))
    sample = float(swerling_samples(2, snr_lin, 1, rng)[0])
    while sample < snr_lin * 0.5:
        sample = float(swerling_samples(2, snr_lin, 1, rng)[0])
    rd[tgt_r, tgt_d] += sample

    # MTI clutter notch: blank the zero-Doppler ridge before detection (stationary-clutter
    # rejection), then run CA-CFAR over Doppler with N reference cells + a guard band on the
    # moving-target (non-notched) cells. The target then exceeds threshold while the noise floor
    # holds the design Pfa.
    rd_cfar = rd.copy()
    rd_cfar[:, notch_lo:notch_hi] = np.nan  # notched out of detection
    num_ref = 16
    alpha = float(cfar_alpha(1e-4, num_ref))
    detections = []
    guard = 2
    half = num_ref // 2
    for ri in range(n_range):
        for di in range(half + guard, n_dopp - half - guard):
            if np.isnan(rd_cfar[ri, di]):
                continue
            lead = rd_cfar[ri, di - half - guard : di - guard]
            lag = rd_cfar[ri, di + guard + 1 : di + half + guard + 1]
            ref = np.concatenate([lead, lag])
            ref = ref[~np.isnan(ref)]
            if ref.size == 0:
                continue
            noise_est = np.mean(ref)
            if rd_cfar[ri, di] > alpha * noise_est:
                detections.append((di, ri))

    fig, ax = plt.subplots(figsize=(6.4, 5.0))
    im = ax.imshow(
        10.0 * np.log10(rd + 1e-6),
        origin="lower",
        aspect="auto",
        cmap="viridis",
        extent=[-n_dopp / 2, n_dopp / 2, 0, n_range],
    )
    if detections:
        dx = np.array([d[0] - n_dopp / 2 for d in detections])
        dy = np.array([d[1] + 0.5 for d in detections])
        ax.scatter(
            dx,
            dy,
            s=80,
            facecolors="none",
            edgecolors="red",
            linewidths=1.5,
            label="CA-CFAR detections",
        )
    ax.scatter(
        [tgt_d - n_dopp / 2], [tgt_r + 0.5], marker="x", c="white", s=70, label="true target"
    )
    ax.axvspan(
        notch_lo - n_dopp / 2,
        notch_hi - n_dopp / 2,
        color="black",
        alpha=0.18,
        label="zero-Doppler notch",
    )
    ax.set_xlabel("Doppler bin  (→ closing rate)")
    ax.set_ylabel("Range bin")
    ax.set_title(
        "Range-Doppler map with CA-CFAR detection\n"
        "(Swerling-II target; zero-Doppler clutter notched, then CA-CFAR)"
    )
    ax.legend(loc="upper left", fontsize=8)
    fig.colorbar(im, ax=ax, label="Cell power [dB]")
    fig.tight_layout()
    return fig


def main() -> None:
    import matplotlib

    matplotlib.use("Agg")
    out_dir = REPO_ROOT / "postproc" / "figures"
    web_dir = REPO_ROOT / "web" / "public" / "figures"
    out_dir.mkdir(parents=True, exist_ok=True)
    web_dir.mkdir(parents=True, exist_ok=True)

    roc = compute_roc()
    print("Sensor phenomenology — CA-CFAR ROC")
    for line in summarize_roc(roc):
        print(line)

    fig_roc = plot_roc(roc)
    fig_roc.savefig(out_dir / "phenomenology_roc.png", dpi=130)
    fig_roc.savefig(web_dir / "phenomenology_roc.png", dpi=130)

    fig_rd = plot_range_doppler()
    fig_rd.savefig(out_dir / "range_doppler_map.png", dpi=130)
    fig_rd.savefig(web_dir / "range_doppler_map.png", dpi=130)
    print(f"wrote {out_dir}/phenomenology_roc.png + range_doppler_map.png (and web copies)")


if __name__ == "__main__":
    main()
