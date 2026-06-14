// gnc-sim — Phase 1 environment tests: gravity model + US Standard Atmosphere 1976.
// Reference values are taken from the published USSA76 tables and standard-gravity definitions.
#include <cmath>

#include <gtest/gtest.h>

#include "gncsim/core/Config.hpp"
#include "gncsim/env/Environment.hpp"
#include "gncsim/math/Vector3.hpp"

using namespace gncsim;

// ---------------------------------------------------------------------------------------------
// GravityModel
// ---------------------------------------------------------------------------------------------

TEST(GravityModel, ConstantGravityPointsDown) {
  EnvConfig cfg;
  cfg.g0 = 9.80665;
  cfg.altitude_dependent_g = false;
  GravityModel g(cfg);

  // Magnitude is g0 and direction is -Up regardless of position.
  Vector3 a = g.acceleration({0.0, 0.0, 0.0});
  EXPECT_DOUBLE_EQ(a.x, 0.0);
  EXPECT_DOUBLE_EQ(a.y, 0.0);
  EXPECT_NEAR(a.z, -9.80665, 1e-9);

  // Altitude has no effect when the falloff is disabled.
  Vector3 high = g.acceleration({1234.0, -567.0, 50000.0});
  EXPECT_DOUBLE_EQ(high.x, 0.0);
  EXPECT_DOUBLE_EQ(high.y, 0.0);
  EXPECT_NEAR(high.z, -9.80665, 1e-9);
}

TEST(GravityModel, SurfaceMagnitudeWithFalloff) {
  EnvConfig cfg;
  cfg.g0 = 9.80665;
  cfg.altitude_dependent_g = true;
  GravityModel g(cfg);

  // At the surface (alt=0) the inverse-square term is unity, so magnitude == g0.
  Vector3 a = g.acceleration({0.0, 0.0, 0.0});
  EXPECT_NEAR(a.z, -9.80665, 1e-9);
}

TEST(GravityModel, InverseSquareFalloffAtAltitude) {
  EnvConfig cfg;
  cfg.g0 = 9.80665;
  cfg.altitude_dependent_g = true;
  GravityModel g(cfg);

  // At 100 km gravity is meaningfully smaller (~9.5 m/s^2).
  const double mag = -g.acceleration({0.0, 0.0, 100000.0}).z;
  EXPECT_LT(mag, 9.80665);
  EXPECT_NEAR(mag, 9.5, 0.05);

  // Cross-check against the closed-form inverse-square value.
  constexpr double Re = 6371000.0;
  const double expected = 9.80665 * (Re / (Re + 100000.0)) * (Re / (Re + 100000.0));
  EXPECT_NEAR(mag, expected, 1e-9);
}

TEST(GravityModel, NegativeAltitudeClampsToSurface) {
  EnvConfig cfg;
  cfg.g0 = 9.80665;
  cfg.altitude_dependent_g = true;
  GravityModel g(cfg);

  // Below the surface, altitude clamps to 0 so magnitude stays at g0 (no super-surface boost).
  const double mag = -g.acceleration({0.0, 0.0, -500.0}).z;
  EXPECT_NEAR(mag, 9.80665, 1e-9);
}

// ---------------------------------------------------------------------------------------------
// US Standard Atmosphere 1976
// ---------------------------------------------------------------------------------------------

TEST(AtmosphereUSSA76, SeaLevelReference) {
  AtmSample s = atmosphereUSSA76(0.0);
  EXPECT_NEAR(s.temperature, 288.15, 0.05);
  EXPECT_NEAR(s.pressure, 101325.0, 50.0);
  EXPECT_NEAR(s.density, 1.225, 0.005);
  EXPECT_NEAR(s.speed_of_sound, 340.29, 0.5);
}

TEST(AtmosphereUSSA76, Tropopause11km) {
  AtmSample s = atmosphereUSSA76(11000.0);
  EXPECT_NEAR(s.temperature, 216.65, 0.2);
  EXPECT_NEAR(s.density, 0.3639, 0.005);
}

TEST(AtmosphereUSSA76, LowerStratosphere20km) {
  AtmSample s = atmosphereUSSA76(20000.0);
  EXPECT_NEAR(s.density, 0.0880, 0.002);
}

TEST(AtmosphereUSSA76, DerivedQuantitiesAreConsistent) {
  // density and speed_of_sound must be self-consistent with T and P via the ideal-gas /
  // adiabatic relations, at every sampled altitude.
  constexpr double R = 287.05287;
  constexpr double gamma = 1.4;
  for (double alt : {0.0, 5000.0, 11000.0, 20000.0, 32000.0, 47000.0, 60000.0, 80000.0}) {
    AtmSample s = atmosphereUSSA76(alt);
    EXPECT_GT(s.temperature, 0.0);
    EXPECT_GT(s.pressure, 0.0);
    EXPECT_NEAR(s.density, s.pressure / (R * s.temperature), 1e-9);
    EXPECT_NEAR(s.speed_of_sound, std::sqrt(gamma * R * s.temperature), 1e-9);
  }
}

TEST(AtmosphereUSSA76, MonotonicallyDecreasingPressureAndDensity) {
  // Pressure and density decrease with altitude across the modeled region.
  double prev_p = 1e30;
  double prev_rho = 1e30;
  for (double alt = 0.0; alt <= 86000.0; alt += 1000.0) {
    AtmSample s = atmosphereUSSA76(alt);
    EXPECT_LT(s.pressure, prev_p) << "pressure at alt=" << alt;
    EXPECT_LT(s.density, prev_rho) << "density at alt=" << alt;
    prev_p = s.pressure;
    prev_rho = s.density;
  }
}

TEST(AtmosphereUSSA76, ClampsOutsideValidRange) {
  // Below 0 m clamps to the sea-level sample.
  AtmSample below = atmosphereUSSA76(-1000.0);
  AtmSample surface = atmosphereUSSA76(0.0);
  EXPECT_DOUBLE_EQ(below.temperature, surface.temperature);
  EXPECT_DOUBLE_EQ(below.pressure, surface.pressure);

  // Above 86 km clamps to the 86 km sample (no extrapolation / NaNs).
  AtmSample high = atmosphereUSSA76(120000.0);
  AtmSample top = atmosphereUSSA76(86000.0);
  EXPECT_DOUBLE_EQ(high.temperature, top.temperature);
  EXPECT_DOUBLE_EQ(high.pressure, top.pressure);
  EXPECT_TRUE(std::isfinite(high.density));
  EXPECT_TRUE(std::isfinite(high.speed_of_sound));
}
