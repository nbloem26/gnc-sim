"""US Standard Atmosphere 1976 — Python twin of ``core/src/env/Environment.cpp``.

Reimplements the exact 7-layer USSA76 model (same constants, same geometric->geopotential
conversion) so the validation harness can (a) check the sim's implicit density via the
terminal-velocity test and (b) assert the model reproduces published USSA76 table values.

Layers (geopotential base altitude [m], lapse rate [K/m]):
    0      troposphere      -0.0065
    11000  tropopause        0.0     (isothermal)
    20000  lower strat       0.0010
    32000  upper strat       0.0028
    47000  stratopause       0.0     (isothermal)
    51000  lower meso       -0.0028
    71000  upper meso       -0.0020
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# --- Physical constants (match Environment.cpp exactly) ---
G0 = 9.80665          # standard gravity [m/s^2]
R_GAS = 287.05287     # specific gas constant for air [J/(kg*K)]
GAMMA = 1.4           # ratio of specific heats [-]
RE_USSA = 6356766.0   # USSA76 effective Earth radius [m]
T0 = 288.15           # sea-level temperature [K]
P0 = 101325.0         # sea-level pressure [Pa]
TOP_H = 84852.0       # geopotential top of modeled region [m]

# (geopotential base altitude [m], lapse rate [K/m])
_LAYERS = [
    (0.0, -0.0065),
    (11000.0, 0.0),
    (20000.0, 0.0010),
    (32000.0, 0.0028),
    (47000.0, 0.0),
    (51000.0, -0.0028),
    (71000.0, -0.0020),
]


def _base_conditions():
    """Precompute base temperature/pressure at each layer bottom by integrating upward."""
    base_t = [T0]
    base_p = [P0]
    for i in range(1, len(_LAYERS)):
        h_prev, lapse_prev = _LAYERS[i - 1]
        dh = _LAYERS[i][0] - h_prev
        t_top = base_t[i - 1] + lapse_prev * dh
        if lapse_prev == 0.0:
            p = base_p[i - 1] * np.exp(-G0 * dh / (R_GAS * base_t[i - 1]))
        else:
            p = base_p[i - 1] * (t_top / base_t[i - 1]) ** (-G0 / (lapse_prev * R_GAS))
        base_t.append(t_top)
        base_p.append(p)
    return base_t, base_p


_BASE_T, _BASE_P = _base_conditions()


@dataclass(frozen=True)
class AtmSample:
    """Atmospheric state at an altitude."""

    temperature: float  # [K]
    pressure: float     # [Pa]
    density: float      # [kg/m^3]
    sound_speed: float  # [m/s]


def ussa76(altitude_m: float) -> AtmSample:
    """Return the USSA76 atmospheric sample at a geometric altitude [m] (0..86 km)."""
    z = float(np.clip(altitude_m, 0.0, 86000.0))
    h = min(RE_USSA * z / (RE_USSA + z), TOP_H)  # geopotential altitude

    idx = 0
    for i, (base_h, _l) in enumerate(_LAYERS):
        if h >= base_h:
            idx = i
        else:
            break

    base_h, lapse = _LAYERS[idx]
    tb, pb = _BASE_T[idx], _BASE_P[idx]
    dh = h - base_h

    if lapse == 0.0:
        temperature = tb
        pressure = pb * np.exp(-G0 * dh / (R_GAS * tb))
    else:
        temperature = tb + lapse * dh
        pressure = pb * (temperature / tb) ** (-G0 / (lapse * R_GAS))

    density = pressure / (R_GAS * temperature)
    sound_speed = np.sqrt(GAMMA * R_GAS * temperature)
    return AtmSample(temperature, pressure, density, sound_speed)


def terminal_velocity(mass: float, cd: float, ref_area: float, altitude_m: float, g: float = G0) -> float:
    """Closed-form terminal velocity v = sqrt(2 m g / (rho Cd A)) using USSA76 density."""
    rho = ussa76(altitude_m).density
    return float(np.sqrt(2.0 * mass * g / (rho * cd * ref_area)))


# Published USSA76 reference values (NASA TM-X-74335 tables), for validation asserts.
# altitude_m -> (temperature_K, density_kg_m3) with loose tolerances accounting for the
# geometric/geopotential rounding in published tables.
REFERENCE_TABLE = {
    0: (288.15, 1.2250),
    11000: (216.65, 0.36392),
    20000: (216.65, 0.088035),
    32000: (228.65, 0.013225),
    47000: (270.65, 0.0014275),
}
