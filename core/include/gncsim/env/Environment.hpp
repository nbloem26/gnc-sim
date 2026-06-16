/// @file Environment.hpp
/// @brief Environment models — gravity + US Standard Atmosphere 1976.
///
/// Phase 1 (env) owns the implementations in core/src/env/. See docs/THEORY.md §3 for the
/// gravity falloff and the USSA76 layer equations; round-Earth / hi-fi gravity live in
/// Frames.hpp / EnvFidelity.hpp.
#pragma once

#include "gncsim/core/Config.hpp"
#include "gncsim/math/Vector3.hpp"

namespace gncsim {

/// @brief Local-vertical gravity model (flat-Earth).
///
/// `acceleration()` returns the gravitational acceleration [m/s²] (negative-Up). Optionally
/// falls off with altitude (inverse-square about the Earth radius) when
/// `EnvConfig.altitude_dependent_g` is set.
class GravityModel {
 public:
  explicit GravityModel(const EnvConfig& cfg) : cfg_(cfg) {}
  Vector3 acceleration(const Vector3& pos_enu) const;

 private:
  EnvConfig cfg_;
};

/// @brief A US Standard Atmosphere 1976 sample (density, pressure, temperature, speed of sound).
struct AtmSample {
  double density = 0.0;         // [kg/m^3]
  double pressure = 0.0;        // [Pa]
  double temperature = 0.0;     // [K]
  double speed_of_sound = 0.0;  // [m/s]
};

/// @brief US Standard Atmosphere 1976 at geometric altitude (valid 0..86 km; clamps outside).
/// @param altitude_m Geometric altitude [m].
AtmSample atmosphereUSSA76(double altitude_m);

}  // namespace gncsim
