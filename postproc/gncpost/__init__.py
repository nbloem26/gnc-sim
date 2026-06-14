"""gncpost — Python post-processing for the gnc-sim C++ flight simulator.

Reads the native CLI's CSV/JSON output (see ``docs/DATA_CONTRACT.md``), plots
trajectories / states / guidance, validates the sim against analytic solutions, and
runs the sensor loop-closure that proves the in-sim IMU reproduces the characterized
noise.

Submodules:
    loaders      — read run folders + manifest into pandas DataFrames
    atmosphere   — USSA76 twin of the C++ model (for validation)
    plots        — trajectory / state / guidance / miss-distance figures
    validate     — analytic validation harness (runs the native CLI)
    montecarlo   — CEP / dispersion analysis of a Monte-Carlo summary
    loop_closure — re-derive IMU noise from sim output (fidelity proof)
"""

from __future__ import annotations

from pathlib import Path

# Repo root = two levels up from this file (postproc/gncpost/__init__.py).
REPO_ROOT = Path(__file__).resolve().parents[2]
CLI_PATH = REPO_ROOT / "build-native" / "apps" / "cli" / "gncsim"

__all__ = ["REPO_ROOT", "CLI_PATH"]
