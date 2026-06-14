// gnc-sim — environment models: gravity + US Standard Atmosphere 1976.
// Phase 1 (env) owns the implementations in core/src/env/.
#pragma once

#include "gncsim/core/Config.hpp"
#include "gncsim/math/Vector3.hpp"

namespace gncsim {

// Local-vertical gravity. Returns acceleration [m/s^2] (negative-Up). Optionally falls off with
// altitude (inverse-square about Earth radius) when EnvConfig.altitude_dependent_g is set.
class GravityModel {
 public:
  explicit GravityModel(const EnvConfig& cfg) : cfg_(cfg) {}
  Vector3 acceleration(const Vector3& pos_enu) const;

 private:
  EnvConfig cfg_;
};

// US Standard Atmosphere 1976 sample at geometric altitude (valid 0..86 km; clamps outside).
struct AtmSample {
  double density = 0.0;        // [kg/m^3]
  double pressure = 0.0;       // [Pa]
  double temperature = 0.0;    // [K]
  double speed_of_sound = 0.0; // [m/s]
};

AtmSample atmosphereUSSA76(double altitude_m);

}  // namespace gncsim
