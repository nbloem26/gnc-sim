"""Plotting for gnc-sim runs: trajectory, states, guidance, miss-distance / CEP.

All functions take loaded DataFrames (or a ``Run``) and return a matplotlib Figure, so
they compose in notebooks and can be saved to PNG for the web app. Uses the Agg backend
by default for headless figure generation.
"""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .loaders import Run


def plot_trajectory(run: Run, mode: str = "east_up", ax=None):
    """Vehicle vs target trajectory.

    Args:
        run: loaded ``Run``.
        mode: ``"east_up"`` (2-D East-Up) or ``"3d"`` (East-North-Up).
    """
    veh, tgt = run.vehicle, run.target
    if mode == "3d":
        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(veh["x"], veh["y"], veh["z"], "-", color="#1f77b4", lw=1.8, label="vehicle")
        if not tgt.empty:
            ax.plot(tgt["x"], tgt["y"], tgt["z"], "--", color="#d62728", lw=1.5, label="target")
            ax.scatter(
                tgt["x"].iloc[-1], tgt["y"].iloc[-1], tgt["z"].iloc[-1], color="#d62728", s=40
            )
        ax.set_xlabel("East [m]")
        ax.set_ylabel("North [m]")
        ax.set_zlabel("Up [m]")
        ax.legend()
        ax.set_title("Trajectory (ENU)")
        return fig

    if ax is None:
        fig, ax = plt.subplots(figsize=(8, 5))
    else:
        fig = ax.figure
    ax.plot(veh["x"], veh["z"], "-", color="#1f77b4", lw=1.8, label="vehicle")
    ax.plot(veh["x"].iloc[0], veh["z"].iloc[0], "o", color="#1f77b4", ms=6)
    if not tgt.empty:
        ax.plot(tgt["x"], tgt["z"], "--", color="#d62728", lw=1.5, label="target")
        ax.plot(tgt["x"].iloc[-1], tgt["z"].iloc[-1], "X", color="#d62728", ms=10)
    title = "Vehicle vs Target — East-Up"
    if run.manifest:
        title += f"  (miss {run.miss_distance:.2f} m, intercept={run.intercept})"
    ax.set_xlabel("East [m]")
    ax.set_ylabel("Up [m]")
    ax.set_title(title)
    ax.grid(True, ls="--", alpha=0.3)
    ax.legend()
    ax.set_aspect("equal", adjustable="datalim")
    return fig


def plot_states(run: Run):
    """Position, velocity, speed, and Mach vs time (2x2 grid)."""
    veh = run.vehicle
    t = veh["t"]
    fig, axes = plt.subplots(2, 2, figsize=(11, 7))

    ax = axes[0, 0]
    for c, lab in (("x", "East"), ("y", "North"), ("z", "Up")):
        ax.plot(t, veh[c], label=lab)
    ax.set_ylabel("position [m]")
    ax.set_title("Position")
    ax.legend(fontsize=8)

    ax = axes[0, 1]
    for c, lab in (("vx", "vx"), ("vy", "vy"), ("vz", "vz")):
        ax.plot(t, veh[c], label=lab)
    ax.set_ylabel("velocity [m/s]")
    ax.set_title("Velocity")
    ax.legend(fontsize=8)

    ax = axes[1, 0]
    speed_mps = np.sqrt(veh["vx"] ** 2 + veh["vy"] ** 2 + veh["vz"] ** 2)
    ax.plot(t, speed_mps, color="#2ca02c")
    ax.set_ylabel("speed [m/s]")
    ax.set_xlabel("t [s]")
    ax.set_title("Speed")

    ax = axes[1, 1]
    if "mach" in veh:
        ax.plot(t, veh["mach"], color="#9467bd")
    ax.set_ylabel("Mach [-]")
    ax.set_xlabel("t [s]")
    ax.set_title("Mach")

    for a in axes.flat:
        a.grid(True, ls="--", alpha=0.3)
    fig.suptitle("Vehicle state vs time")
    fig.tight_layout()
    return fig


def plot_guidance(run: Run):
    """LOS angle/rate, closing velocity & range, and acceleration command magnitude."""
    gnc = run.gnc
    if gnc.empty:
        raise ValueError("run has no gnc.csv data")
    t = gnc["t"]
    fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)

    axes[0].plot(t, np.degrees(gnc["los_angle"]), label="LOS angle [deg]", color="#1f77b4")
    axes[0].plot(t, np.degrees(gnc["los_rate"]), label="LOS rate [deg/s]", color="#ff7f0e")
    axes[0].set_ylabel("LOS")
    axes[0].legend(fontsize=8)
    axes[0].set_title("Line-of-sight angle & rate")

    axes[1].plot(t, gnc["range"], label="range [m]", color="#2ca02c")
    ax1b = axes[1].twinx()
    ax1b.plot(t, gnc["v_closing"], label="v_closing [m/s]", color="#d62728", alpha=0.7)
    axes[1].set_ylabel("range [m]")
    ax1b.set_ylabel("closing [m/s]")
    axes[1].set_title("Range & closing velocity")

    acc_mps2 = np.sqrt(gnc["accel_cmd_x"] ** 2 + gnc["accel_cmd_y"] ** 2 + gnc["accel_cmd_z"] ** 2)
    axes[2].plot(t, acc_mps2, color="#9467bd")
    axes[2].set_ylabel("|a_cmd| [m/s^2]")
    axes[2].set_xlabel("t [s]")
    axes[2].set_title("Acceleration command magnitude")

    for a in axes:
        a.grid(True, ls="--", alpha=0.3)
    fig.tight_layout()
    return fig


def plot_miss_histogram(summary, cep: float | None = None, mean: float | None = None):
    """Miss-distance histogram with CEP / mean markers from a Monte-Carlo summary."""
    miss_m = np.asarray(summary["miss_distance"], dtype=float)
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.hist(miss_m, bins=30, color="#1f77b4", alpha=0.8, edgecolor="white")
    if cep is not None:
        ax.axvline(cep, color="#d62728", lw=2, ls="--", label=f"CEP (50%) = {cep:.1f} m")
    if mean is not None:
        ax.axvline(mean, color="#2ca02c", lw=2, ls=":", label=f"mean = {mean:.1f} m")
    ax.set_xlabel("miss distance [m]")
    ax.set_ylabel("count")
    ax.set_title(f"Miss-distance distribution ({len(miss_m)} cases)")
    ax.grid(True, ls="--", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    return fig
