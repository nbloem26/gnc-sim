// gnc-sim — environment models: local-vertical gravity + US Standard Atmosphere 1976.
//
// Both implementations declared in gncsim/env/Environment.hpp are defined here. Pure compute,
// no I/O, deterministic. Frame convention is ENU (z = Up); gravity points -Up. See
// docs/DATA_CONTRACT.md for units/frames.
#include "gncsim/env/Environment.hpp"

#include <algorithm>
#include <cmath>

namespace gncsim {

namespace {

// --- Shared physical constants -------------------------------------------------------------
constexpr double kG0 = 9.80665;      // standard gravity [m/s^2]
constexpr double kRgas = 287.05287;  // specific gas constant for air [J/(kg*K)]
constexpr double kGamma = 1.4;       // ratio of specific heats for air [-]

// Gravity model uses the mean Earth radius for the inverse-square falloff.
constexpr double kReGravity = 6371000.0;  // [m]

// USSA76 uses its own effective Earth radius for the geometric->geopotential conversion.
constexpr double kReUssa = 6356766.0;  // [m]

}  // namespace

// =============================================================================================
// GravityModel
// =============================================================================================
//
// Returns gravitational acceleration in the ENU frame, always pointing down (-z / -Up).
//   - altitude_dependent_g == false: constant magnitude g0.
//   - altitude_dependent_g == true:  inverse-square falloff g = g0 * (Re/(Re+alt))^2,
//                                     with alt = pos_enu.z clamped to >= 0.
Vector3 GravityModel::acceleration(const Vector3& pos_enu) const {
  if (!cfg_.altitude_dependent_g) {
    return {0.0, 0.0, -cfg_.g0};
  }

  // Clamp altitude to non-negative; below the reference surface gravity is treated as surface g.
  const double alt = std::max(0.0, pos_enu.z);
  const double ratio = kReGravity / (kReGravity + alt);
  const double g = cfg_.g0 * ratio * ratio;
  return {0.0, 0.0, -g};
}

// =============================================================================================
// US Standard Atmosphere 1976 (USSA76), lower layers (0..86 km geometric)
// =============================================================================================
//
// The model is a stack of 7 layers, each defined at a *geopotential* base altitude with a base
// temperature, base pressure, and a constant lapse rate (dT/dh). Within a layer:
//   - lapse != 0 (linear-lapse layer): barometric formula
//         T = Tb + L*(h - hb)
//         P = Pb * (T/Tb)^( -g0 / (L*R) )
//   - lapse == 0 (isothermal layer):
//         T = Tb
//         P = Pb * exp( -g0*(h - hb) / (R*Tb) )
//
// Base pressures for layers above the first are precomputed by walking the formulas up from the
// sea-level reference (T0=288.15 K, P0=101325 Pa). Geometric altitude z is converted to
// geopotential altitude h via h = Re*z/(Re+z) with the USSA76 effective radius.
AtmSample atmosphereUSSA76(double altitude_m) {
  // USSA76 layer breakpoints, defined at geopotential base altitudes.
  // {base geopotential altitude [m], lapse rate dT/dh [K/m]}.
  struct Layer {
    double base_h;  // geopotential base altitude [m]
    double lapse;   // lapse rate [K/m]
  };
  // Covers 0..84852 m geopotential (~86 km geometric). The 7th layer top (71 km base) extends to
  // 84852 m geopotential, which corresponds to 86 km geometric — the documented upper bound.
  static constexpr Layer kLayers[] = {
      {0.0, -0.0065},      // 0  troposphere
      {11000.0, 0.0},      // 1  tropopause (isothermal)
      {20000.0, 0.0010},   // 2  lower stratosphere
      {32000.0, 0.0028},   // 3  upper stratosphere
      {47000.0, 0.0},      // 4  stratopause (isothermal)
      {51000.0, -0.0028},  // 5  lower mesosphere
      {71000.0, -0.0020},  // 6  upper mesosphere
  };
  static constexpr int kNumLayers = static_cast<int>(sizeof(kLayers) / sizeof(kLayers[0]));
  static constexpr double kTopH = 84852.0;  // geopotential top of modeled region [m]

  // Sea-level reference conditions (USSA76).
  static constexpr double kT0 = 288.15;    // [K]
  static constexpr double kP0 = 101325.0;  // [Pa]

  // Precompute base temperature and base pressure at the bottom of each layer by integrating the
  // formulas upward from sea level. Done once on first call (function-local statics).
  static double base_temp[kNumLayers];
  static double base_pres[kNumLayers];
  static bool initialized = false;
  if (!initialized) {
    base_temp[0] = kT0;
    base_pres[0] = kP0;
    for (int i = 1; i < kNumLayers; ++i) {
      const Layer& prev = kLayers[i - 1];
      const double dh = kLayers[i].base_h - prev.base_h;
      const double t_top = base_temp[i - 1] + prev.lapse * dh;  // temp at top of previous layer
      if (prev.lapse == 0.0) {
        // Isothermal layer.
        base_pres[i] = base_pres[i - 1] * std::exp(-kG0 * dh / (kRgas * base_temp[i - 1]));
      } else {
        // Linear-lapse layer.
        base_pres[i] =
            base_pres[i - 1] * std::pow(t_top / base_temp[i - 1], -kG0 / (prev.lapse * kRgas));
      }
      base_temp[i] = t_top;
    }
    initialized = true;
  }

  // Clamp geometric altitude to the valid range, then convert to geopotential altitude.
  const double z = std::clamp(altitude_m, 0.0, 86000.0);
  double h = kReUssa * z / (kReUssa + z);
  // Guard against the converted geopotential altitude slightly exceeding the modeled top.
  h = std::min(h, kTopH);

  // Find the layer whose base is at or below h (layers are ascending in altitude).
  int idx = 0;
  for (int i = 0; i < kNumLayers; ++i) {
    if (h >= kLayers[i].base_h) {
      idx = i;
    } else {
      break;
    }
  }

  const Layer& layer = kLayers[idx];
  const double Tb = base_temp[idx];
  const double Pb = base_pres[idx];
  const double dh = h - layer.base_h;

  double temperature;
  double pressure;
  if (layer.lapse == 0.0) {
    // Isothermal layer.
    temperature = Tb;
    pressure = Pb * std::exp(-kG0 * dh / (kRgas * Tb));
  } else {
    // Linear-lapse layer.
    temperature = Tb + layer.lapse * dh;
    pressure = Pb * std::pow(temperature / Tb, -kG0 / (layer.lapse * kRgas));
  }

  AtmSample s;
  s.temperature = temperature;
  s.pressure = pressure;
  s.density = pressure / (kRgas * temperature);
  s.speed_of_sound = std::sqrt(kGamma * kRgas * temperature);
  return s;
}

}  // namespace gncsim
