"""Generate a long STATIC synthetic IMU record with known injected noise.

Produces a tactical-grade MEMS-class record (accel + gyro, 3 axes) by driving the
reference error model in ``noise_model.py`` with hand-picked TRUE parameters. The
record is the input to the Allan-variance characterization (``allan_variance.py``)
and ``fit_noise_params.py``; recovering the injected values back from the data is
the first half of the fidelity loop.

Static lab convention: the gyro true rate is 0 on every axis. The accelerometer
senses gravity, but for noise characterization we work with the *de-trended* signal,
so we inject around a zero mean (gravity is a constant offset that Allan variance
differences away). Output therefore stores the pure error process per axis.

Run::

    python sensors/generate_imu_data.py            # default 4 h @ 100 Hz, seed 42

Writes ``sensors/imu_raw.npz`` and prints the TRUE injected parameters.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from noise_model import AxisNoise, generate_axis, true_params_from_model

# --- Repo-relative paths (only the repo root is absolute, via this file's location) ---
SENSORS_DIR = Path(__file__).resolve().parent
RAW_PATH = SENSORS_DIR / "imu_raw.npz"

# ---------------------------------------------------------------------------
# TRUE injected noise — tactical-grade MEMS IMU (per axis, continuous-time units).
# Chosen so the three Allan regimes are cleanly separated over a few-hour record:
#   accel: VRW ~ 6e-4 (m/s)/sqrt(s) ~ 36 ug/sqrt(Hz); bias instability ~ 3e-4 m/s^2 ~ 30 ug.
#   gyro:  ARW ~ 9e-5 rad/sqrt(s)   ~ 0.31 deg/sqrt(hr); bias instab ~ 1.2e-5 rad/s ~ 2.5 deg/hr.
# ---------------------------------------------------------------------------
# NOTE on bias instability. The C++ sensor injects bias instability as a first-order
# Gauss-Markov (GM) process whose *steady-state std* (sigma_GM) is the configured value.
# On an Allan plot a GM process is a hump (rise then fall), not the flat flicker floor of
# an ideal IMU; the hump peak ~= 0.617 * sigma_GM. The headline "bias instability B" we
# REPORT below is that hump-peak deviation (the best stability the sensor reaches near its
# correlation time); the ``*_bias_instability`` field written to sensor_params.json is the
# underlying sigma_GM the C++ model expects. Both round-trip within ~10%.
#
# NOTE on RRW. In the C++ model (and our twin) the rate random walk is added onto the same
# bias state the GM term decays each step, so the GM decay BOUNDS the walk — there is no
# free, unbounded +1/2 Allan ramp. RRW is therefore deliberately kept small here and is not
# separately identifiable above the bias floor in a multi-hour record; the ground-truth
# labels reflect this (see ``noise_model.true_params_from_model``).
TRUE_ACCEL = AxisNoise(
    white=6.0e-4,             # VRW density  (m/s^2)/sqrt(Hz)  (~36 ug/sqrt(Hz))
    bias_instability=4.56e-4,  # GM steady-state std (sigma_GM); Allan hump ~2.8e-4 m/s^2
    bias_tau=100.0,            # GM correlation time [s]
    rrw=1.5e-5,               # (m/s^2)/sqrt(s) — GM-bounded, below the bias floor
    scale_factor=1.0e-3,
)
TRUE_GYRO = AxisNoise(
    white=9.0e-5,             # ARW density  (rad/s)/sqrt(Hz)  (~0.31 deg/sqrt(hr))
    bias_instability=1.82e-5,  # GM steady-state std (sigma_GM); Allan hump ~1.1e-5 rad/s
    bias_tau=100.0,            # GM correlation time [s]
    rrw=4.0e-7,               # (rad/s)/sqrt(s) — GM-bounded, below the bias floor
    scale_factor=5.0e-4,
)


def true_labels(p: AxisNoise, dt: float, n: int) -> np.ndarray:
    """``[white, bias_instability, bias_tau, rrw]`` ground truth for axis ``p``.

    Obtained by Monte-Carlo characterizing the *exact* generator with the identical
    extractor used on the data (see ``noise_model.true_params_from_model``), so "true"
    and "recovered" denote the same physically realizable quantity — including the fact
    that the GM-coupled RRW does not produce a free +1/2 ramp.
    """
    tp = true_params_from_model(p, dt, n)
    return np.array([tp["white"], tp["bias_instability"], tp["bias_tau"], tp["rrw"]])


def generate(
    duration_s: float = 4.0 * 3600.0,
    rate_hz: float = 100.0,
    seed: int = 42,
) -> dict:
    """Generate the static 3-axis IMU record.

    Returns a dict with ``dt``, ``t``, ``accel`` (n,3), ``gyro`` (n,3) and the
    TRUE parameter objects, and also persists it to ``imu_raw.npz``.
    """
    dt = 1.0 / rate_hz
    n = int(round(duration_s * rate_hz))
    rng = np.random.default_rng(seed)

    accel = np.column_stack(
        [generate_axis(n, dt, TRUE_ACCEL, rng, true_signal=0.0) for _ in range(3)]
    )
    gyro = np.column_stack(
        [generate_axis(n, dt, TRUE_GYRO, rng, true_signal=0.0) for _ in range(3)]
    )
    t = np.arange(n) * dt

    # Ground-truth labels: Monte-Carlo characterization of the exact generator (same
    # extractor as recovery), so recovered-vs-true is apples-to-apples.
    true_accel = true_labels(TRUE_ACCEL, dt, n)
    true_gyro = true_labels(TRUE_GYRO, dt, n)

    np.savez_compressed(
        RAW_PATH,
        dt=dt,
        t=t,
        accel=accel,
        gyro=gyro,
        seed=seed,
        true_accel=true_accel,
        true_gyro=true_gyro,
    )

    return {
        "dt": dt,
        "t": t,
        "accel": accel,
        "gyro": gyro,
        "true_accel": TRUE_ACCEL,
        "true_gyro": TRUE_GYRO,
        "true_accel_labels": true_accel,
        "true_gyro_labels": true_gyro,
    }


def _print_true(labels_accel: np.ndarray, labels_gyro: np.ndarray, dt: float, n: int) -> None:
    print(f"Generated static IMU record: {n} samples @ {1/dt:.0f} Hz "
          f"({n*dt/3600:.2f} h), dt={dt:.4f} s")
    print(f"  saved -> {RAW_PATH.relative_to(SENSORS_DIR.parent)}")
    print("\nTRUE injected parameters (continuous-time units):")
    hdr = f"  {'channel':8s} {'white':>11s} {'bias_instab':>12s} {'tau[s]':>8s} {'rrw':>11s}"
    print(hdr)
    for name, lab in (("accel", labels_accel), ("gyro", labels_gyro)):
        print(f"  {name:8s} {lab[0]:11.3e} {lab[1]:12.3e} {lab[2]:8.1f} {lab[3]:11.3e}")
    print("  (bias_instability = Allan hump-peak B; labels = Monte-Carlo of the exact generator)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--duration", type=float, default=4.0 * 3600.0, help="record length [s]")
    ap.add_argument("--rate", type=float, default=100.0, help="sample rate [Hz]")
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    data = generate(args.duration, args.rate, args.seed)
    _print_true(data["true_accel_labels"], data["true_gyro_labels"], data["dt"],
                data["accel"].shape[0])


if __name__ == "__main__":
    main()
