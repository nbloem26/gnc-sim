"""Shared pytest fixtures + path setup for the gncpost test suite."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
# Make both the postproc package and the sensors pipeline importable.
sys.path.insert(0, str(REPO_ROOT / "postproc"))
sys.path.insert(0, str(REPO_ROOT / "sensors"))

CLI_PATH = REPO_ROOT / "build-native" / "apps" / "cli" / "gncsim"


@pytest.fixture(scope="session")
def repo_root() -> Path:
    return REPO_ROOT


@pytest.fixture(scope="session")
def cli_available() -> bool:
    return CLI_PATH.exists()


def requires_cli():
    """Skip-marker for tests that invoke the native CLI."""
    import pytest as _pytest

    return _pytest.mark.skipif(not CLI_PATH.exists(), reason="native CLI not built")
