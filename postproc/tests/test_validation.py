"""Analytic validation + loop-closure (these invoke the native CLI).

Skipped automatically if the build-native binary is absent.
"""

from __future__ import annotations

import tempfile
from pathlib import Path

from conftest import requires_cli

from gncpost.validate import (
    check_atmosphere,
    check_ballistic,
    check_pronav_intercept,
    check_terminal_velocity,
)


def test_atmosphere_check_no_cli():
    # USSA76 table check is pure Python — always runs.
    r = check_atmosphere()
    assert r.passed, r.detail


@requires_cli()
def test_ballistic_parabola():
    with tempfile.TemporaryDirectory() as td:
        r = check_ballistic(Path(td))
    assert r.passed, r.detail
    assert r.metric < 0.05  # < 5 cm over the whole trajectory


@requires_cli()
def test_terminal_velocity():
    with tempfile.TemporaryDirectory() as td:
        r = check_terminal_velocity(Path(td))
    assert r.passed, r.detail
    assert r.metric < 0.02  # within 2% of closed form


@requires_cli()
def test_pronav_intercept():
    with tempfile.TemporaryDirectory() as td:
        r = check_pronav_intercept(Path(td))
    assert r.passed, r.detail
    assert r.metric < 1.0  # miss < 1 m


@requires_cli()
def test_loop_closure_recovers_white():
    """The in-sim IMU reproduces the configured white noise within tolerance."""
    from gncpost.loop_closure import run_loop_closure

    res = run_loop_closure(make_figure=False)
    assert res.passed, res.summary_line()
    assert res.white_err_frac < 0.15
