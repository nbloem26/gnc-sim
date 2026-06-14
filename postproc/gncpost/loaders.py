"""Load a gnc-sim run folder into pandas DataFrames + parsed manifest.

A single run folder (written by ``gncsim --out <dir>``) contains ``vehicle.csv``,
``target.csv``, ``gnc.csv``, ``sensors.csv`` (all sharing the ``t`` column) and
``manifest.json``. A Monte-Carlo batch additionally has ``summary.csv``.

See ``docs/DATA_CONTRACT.md §3-4`` for the exact column lists.
"""

from __future__ import annotations

import json
import subprocess
from dataclasses import dataclass, field
from pathlib import Path

import pandas as pd

from . import CLI_PATH


@dataclass
class Run:
    """A loaded single run: the four telemetry frames + manifest metadata."""

    path: Path
    vehicle: pd.DataFrame
    target: pd.DataFrame
    gnc: pd.DataFrame
    sensors: pd.DataFrame
    manifest: dict = field(default_factory=dict)

    @property
    def t(self) -> pd.Series:
        return self.vehicle["t"]

    @property
    def intercept(self) -> bool:
        return bool(self.manifest.get("intercept", False))

    @property
    def miss_distance(self) -> float:
        return float(self.manifest.get("miss_distance", float("nan")))


def load_run(run_dir: str | Path) -> Run:
    """Load all CSVs + manifest from a single run folder."""
    run_dir = Path(run_dir)
    if not run_dir.is_dir():
        raise FileNotFoundError(f"run folder not found: {run_dir}")

    def _csv(name: str) -> pd.DataFrame:
        p = run_dir / name
        return pd.read_csv(p) if p.exists() else pd.DataFrame()

    manifest = {}
    mpath = run_dir / "manifest.json"
    if mpath.exists():
        manifest = json.loads(mpath.read_text())

    return Run(
        path=run_dir,
        vehicle=_csv("vehicle.csv"),
        target=_csv("target.csv"),
        gnc=_csv("gnc.csv"),
        sensors=_csv("sensors.csv"),
        manifest=manifest,
    )


def load_summary(batch_dir: str | Path) -> pd.DataFrame:
    """Load a Monte-Carlo ``summary.csv`` (cols: case,seed,miss_distance,intercept_time,intercept)."""
    batch_dir = Path(batch_dir)
    summary = batch_dir / "summary.csv"
    if not summary.exists():
        raise FileNotFoundError(f"summary.csv not found in {batch_dir}")
    return pd.read_csv(summary)


def run_cli(
    config_path: str | Path,
    out_dir: str | Path,
    seed: int | None = None,
    cli_path: str | Path = CLI_PATH,
    timeout: float = 120.0,
) -> Path:
    """Invoke the native CLI and return the output directory.

    Args:
        config_path: scenario JSON.
        out_dir: where the CLI writes telemetry/summary.
        seed: optional seed override.
        cli_path: path to the gncsim binary.
        timeout: subprocess timeout [s].

    Returns:
        ``Path(out_dir)`` after a successful run.
    """
    cli_path = Path(cli_path)
    if not cli_path.exists():
        raise FileNotFoundError(
            f"native CLI not found at {cli_path} — build it first (build-native)"
        )
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = [str(cli_path), "--config", str(config_path), "--out", str(out_dir)]
    if seed is not None:
        cmd += ["--seed", str(seed)]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    if proc.returncode != 0:
        raise RuntimeError(
            f"gncsim failed (rc={proc.returncode}):\n{proc.stdout}\n{proc.stderr}"
        )
    return out_dir
