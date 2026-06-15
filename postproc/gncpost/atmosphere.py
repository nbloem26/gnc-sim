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
G0 = 9.80665  # standard gravity [m/s^2]
R_GAS = 287.05287  # specific gas constant for air [J/(kg*K)]
GAMMA = 1.4  # ratio of specific heats [-]
RE_USSA = 6356766.0  # USSA76 effective Earth radius [m]
T0 = 288.15  # sea-level temperature [K]
P0 = 101325.0  # sea-level pressure [Pa]
TOP_H = 84852.0  # geopotential top of modeled region [m]

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
        h_prev_m, lapse_prev = _LAYERS[i - 1]
        dh_m = _LAYERS[i][0] - h_prev_m
        t_top_k = base_t[i - 1] + lapse_prev * dh_m
        if lapse_prev == 0.0:
            p_pa = base_p[i - 1] * np.exp(-G0 * dh_m / (R_GAS * base_t[i - 1]))
        else:
            p_pa = base_p[i - 1] * (t_top_k / base_t[i - 1]) ** (-G0 / (lapse_prev * R_GAS))
        base_t.append(t_top_k)
        base_p.append(p_pa)
    return base_t, base_p


_BASE_T, _BASE_P = _base_conditions()


@dataclass(frozen=True)
class AtmSample:
    """Atmospheric state at an altitude."""

    temperature: float  # [K]
    pressure: float  # [Pa]
    density: float  # [kg/m^3]
    sound_speed: float  # [m/s]


def ussa76(altitude_m: float) -> AtmSample:
    """Return the USSA76 atmospheric sample at a geometric altitude [m] (0..86 km)."""
    z_m = float(np.clip(altitude_m, 0.0, 86000.0))
    h_m = min(RE_USSA * z_m / (RE_USSA + z_m), TOP_H)  # geopotential altitude

    idx = 0
    for i, (base_h_m, _l) in enumerate(_LAYERS):
        if h_m >= base_h_m:
            idx = i
        else:
            break

    base_h_m, lapse = _LAYERS[idx]
    tb_k, pb_pa = _BASE_T[idx], _BASE_P[idx]
    dh_m = h_m - base_h_m

    if lapse == 0.0:
        temperature_k = tb_k
        pressure_pa = pb_pa * np.exp(-G0 * dh_m / (R_GAS * tb_k))
    else:
        temperature_k = tb_k + lapse * dh_m
        pressure_pa = pb_pa * (temperature_k / tb_k) ** (-G0 / (lapse * R_GAS))

    density_kgpm3 = pressure_pa / (R_GAS * temperature_k)
    sound_speed_mps = np.sqrt(GAMMA * R_GAS * temperature_k)
    return AtmSample(temperature_k, pressure_pa, density_kgpm3, sound_speed_mps)


def terminal_velocity(
    mass: float, cd: float, ref_area: float, altitude_m: float, g: float = G0
) -> float:
    """Closed-form terminal velocity v = sqrt(2 m g / (rho Cd A)) using USSA76 density."""
    rho_kgpm3 = ussa76(altitude_m).density
    return float(np.sqrt(2.0 * mass * g / (rho_kgpm3 * cd * ref_area)))


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
