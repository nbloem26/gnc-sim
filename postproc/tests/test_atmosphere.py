"""USSA76 twin: reference table values + terminal-velocity formula."""

from __future__ import annotations

import math

import pytest
from gncpost.atmosphere import REFERENCE_TABLE, terminal_velocity, ussa76


def test_sea_level_reference():
    s = ussa76(0.0)
    assert s.temperature == pytest.approx(288.15, rel=1e-4)
    assert s.pressure == pytest.approx(101325.0, rel=1e-4)
    assert s.density == pytest.approx(1.2250, rel=1e-3)
    # speed of sound at sea level ~ 340.29 m/s
    assert s.sound_speed == pytest.approx(340.29, rel=1e-3)


@pytest.mark.parametrize("alt", list(REFERENCE_TABLE.keys()))
def test_reference_table(alt):
    T_ref, rho_ref = REFERENCE_TABLE[alt]
    s = ussa76(alt)
    assert s.temperature == pytest.approx(T_ref, rel=0.01)
    assert s.density == pytest.approx(rho_ref, rel=0.05)


def test_density_monotonic_decrease():
    densities = [ussa76(z).density for z in range(0, 80000, 2000)]
    assert all(b < a for a, b in zip(densities, densities[1:], strict=False))


def test_terminal_velocity_formula():
    # Hand-computed: v = sqrt(2 m g / (rho Cd A)) at sea level (rho = 1.225).
    m, cd, area, g = 5.0, 1.0, 0.05, 9.80665
    rho = ussa76(0.0).density
    expected = math.sqrt(2 * m * g / (rho * cd * area))
    assert terminal_velocity(m, cd, area, 0.0, g) == pytest.approx(expected, rel=1e-9)
    assert terminal_velocity(m, cd, area, 0.0, g) == pytest.approx(40.0, abs=0.5)


def test_terminal_velocity_increases_with_altitude():
    # Thinner air aloft -> higher terminal velocity.
    v_low = terminal_velocity(5.0, 1.0, 0.05, 0.0)
    v_high = terminal_velocity(5.0, 1.0, 0.05, 10000.0)
    assert v_high > v_low
