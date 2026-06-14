"""Tests for the gnc-sim Python SDK (pybind11 bindings, issue #32).

These assert the SDK is a faithful, zero-numerics-added view of the pure C++ core:

* ``run(cfg)`` returns the columnar series as numpy arrays with the same channel vocabulary as the
  web/CSV contract.
* Its results match the native CLI **bit-for-bit** for the same config + seed — the SDK and the CLI
  call the identical ``runSimulation`` over the identical ``loadConfigFromString`` parser, so any
  divergence is a real regression. The bit-for-bit reference is the CLI's full-precision ``--json``
  output (the committed CSVs are rendered at 9 significant digits and are not exact).
* dict and JSON-string configs are equivalent.
* Monte Carlo is deterministic given the seed.

The CLI comparison locates the native binary via the ``GNCSIM_CLI`` env var or common build paths;
if no binary is present (e.g. a bindings-only dev checkout) that one test skips, but the SDK-vs-golden
scalar check (full-precision miss distance / intercept time from the committed manifest) always runs.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

import gncsim
import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = REPO_ROOT / "configs" / "homing_3dof.json"
SAMPLE_MANIFEST = REPO_ROOT / "runs" / "sample_run" / "manifest.json"


def _load_cfg() -> dict:
    return json.loads(CONFIG_PATH.read_text())


def _find_cli() -> str | None:
    env = os.environ.get("GNCSIM_CLI")
    if env and Path(env).is_file():
        return env
    candidates = [
        REPO_ROOT / "build-native" / "apps" / "cli" / "gncsim",
        REPO_ROOT / "build-python" / "apps" / "cli" / "gncsim",
    ]
    for c in candidates:
        if c.is_file() and os.access(c, os.X_OK):
            return str(c)
    found = shutil.which("gncsim")
    return found


def test_run_returns_numpy_series() -> None:
    res = gncsim.run(_load_cfg())
    assert res["scenario"] == "homing"
    assert res["model"] == "3dof"
    assert "series" in res
    series = res["series"]
    # Every channel is a 1-D float64 numpy array, all the same length.
    lengths = set()
    for name, arr in series.items():
        assert isinstance(arr, np.ndarray), f"{name} is not an ndarray"
        assert arr.dtype == np.float64
        assert arr.ndim == 1
        lengths.add(arr.shape[0])
    assert len(lengths) == 1, "ragged series channels"
    assert series["range"].shape[0] > 0


def test_dict_and_json_string_equivalent() -> None:
    cfg = _load_cfg()
    a = gncsim.run(cfg)
    b = gncsim.run(json.dumps(cfg))
    assert a["miss_distance"] == b["miss_distance"]
    assert np.array_equal(a["series"]["range"], b["series"]["range"])


def test_matches_golden_manifest_scalars() -> None:
    # The committed manifest stores full-precision metadata (not the 9-sig-fig CSVs), so this is an
    # exact comparison and runs even without a CLI binary present.
    manifest = json.loads(SAMPLE_MANIFEST.read_text())
    res = gncsim.run(_load_cfg())
    assert res["seed"] == manifest["seed"]
    assert res["intercept"] == manifest["intercept"]
    assert res["miss_distance"] == manifest["miss_distance"]
    assert res["intercept_time"] == manifest["intercept_time"]


def test_matches_cli_bit_for_bit() -> None:
    cli = _find_cli()
    if cli is None:
        pytest.skip("native gncsim CLI not found (set GNCSIM_CLI or build it)")

    res = gncsim.run(_load_cfg())
    with tempfile.TemporaryDirectory() as tmp:
        json_out = Path(tmp) / "cli_result.json"
        subprocess.run(
            [cli, "--config", str(CONFIG_PATH), "--out", tmp, "--json", str(json_out)],
            check=True,
            capture_output=True,
        )
        cli_res = json.loads(json_out.read_text())

    # Scalar metadata: exact.
    for key in (
        "scenario",
        "model",
        "seed",
        "dt",
        "t_end",
        "intercept",
        "miss_distance",
        "intercept_time",
        "launch_time",
    ):
        assert res[key] == cli_res[key], f"metadata mismatch on {key}"

    # Every series channel: bit-for-bit identical to the CLI's full-precision JSON.
    cli_series = cli_res["series"]
    assert set(res["series"].keys()) == set(cli_series.keys())
    for name, arr in res["series"].items():
        expected = np.asarray(cli_series[name], dtype=np.float64)
        got = np.asarray(arr, dtype=np.float64)
        assert np.array_equal(got, expected), f"series channel {name} not bit-identical to CLI"


def test_monte_carlo_deterministic() -> None:
    cfg = _load_cfg()
    cfg["monte_carlo"] = {
        "num_cases": 0,
        "launch_speed_sigma": 10.0,
        "launch_elevation_sigma_deg": 1.0,
        "target_pos_sigma": 50.0,
    }
    a = gncsim.monte_carlo(cfg, n=16)
    b = gncsim.monte_carlo(cfg, n=16)
    assert a["num_cases"] == 16
    assert np.array_equal(a["seed"], b["seed"])
    assert np.array_equal(a["miss_distance"], b["miss_distance"])
    assert np.array_equal(a["intercept"], b["intercept"])
    assert 0.0 <= a["p_kill"] <= 1.0
    assert a["miss_distance"].dtype == np.float64
    assert a["intercept"].dtype == np.bool_
