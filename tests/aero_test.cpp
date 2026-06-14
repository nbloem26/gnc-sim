// gnc-sim — Phase 1 aerodynamics tests: Cd interpolation, drag direction/magnitude scaling,
// and 6DOF normal force from angle of attack. World frame is ENU, SI units.
#include <cmath>

#include <gtest/gtest.h>

#include "gncsim/aero/Aero.hpp"
#include "gncsim/core/Config.hpp"
#include "gncsim/env/Environment.hpp"
#include "gncsim/math/Quaternion.hpp"
#include "gncsim/math/Vector3.hpp"

using namespace gncsim;

namespace {

// Atmosphere sample with a fixed speed of sound so Mach = speed / sos is controllable, and a
// configurable density for scaling tests.
AtmSample makeAtm(double density, double sos = 340.0) {
  AtmSample a;
  a.density = density;
  a.pressure = 101325.0;
  a.temperature = 288.15;
  a.speed_of_sound = sos;
  return a;
}

// Triangular Cd table used by the required interpolation assertions.
AeroConfig tableConfig() {
  AeroConfig c;
  c.ref_area = 0.05;
  c.cd0 = 0.3;
  c.cn_alpha = 12.0;
  c.cd_mach = {{0.0, 0.2}, {1.0, 0.6}, {2.0, 0.4}};
  return c;
}

}  // namespace

// ── A) Cd interpolation ─────────────────────────────────────────────────────────────────────
TEST(AeroCd, InterpolatesAndClamps) {
  AeroModel aero(tableConfig());

  // Interior interpolation.
  EXPECT_NEAR(aero.dragCoefficient(0.5), 0.4, 1e-12);  // midway 0.2 -> 0.6
  EXPECT_NEAR(aero.dragCoefficient(1.0), 0.6, 1e-12);  // exact breakpoint
  EXPECT_NEAR(aero.dragCoefficient(1.5), 0.5, 1e-12);  // midway 0.6 -> 0.4

  // Clamp below first and above last breakpoint to the endpoint value.
  EXPECT_NEAR(aero.dragCoefficient(-1.0), 0.2, 1e-12);
  EXPECT_NEAR(aero.dragCoefficient(3.0), 0.4, 1e-12);
}

TEST(AeroCd, EmptyTableFallsBackToCd0) {
  AeroConfig c;
  c.cd0 = 0.42;
  c.cd_mach.clear();
  AeroModel aero(c);
  EXPECT_NEAR(aero.dragCoefficient(0.0), 0.42, 1e-12);
  EXPECT_NEAR(aero.dragCoefficient(5.0), 0.42, 1e-12);
}

// ── B) Drag force ───────────────────────────────────────────────────────────────────────────
TEST(AeroDrag, OpposesVelocity) {
  AeroModel aero(tableConfig());
  const Vector3 vel{100.0, 50.0, -20.0};
  const Vector3 f = aero.dragForce(vel, makeAtm(1.225));
  // Drag must point against velocity.
  EXPECT_LT(f.dot(vel), 0.0);
}

TEST(AeroDrag, ZeroAtZeroSpeed) {
  AeroModel aero(tableConfig());
  const Vector3 f = aero.dragForce(Vector3{0.0, 0.0, 0.0}, makeAtm(1.225));
  EXPECT_DOUBLE_EQ(f.x, 0.0);
  EXPECT_DOUBLE_EQ(f.y, 0.0);
  EXPECT_DOUBLE_EQ(f.z, 0.0);
}

TEST(AeroDrag, HigherDensityGivesLargerDrag) {
  AeroModel aero(tableConfig());
  const Vector3 vel{200.0, 0.0, 0.0};  // Mach ~0.588 -> same Cd for both samples.
  const double low_alt = aero.dragForce(vel, makeAtm(1.225)).norm();  // sea level
  const double high_alt = aero.dragForce(vel, makeAtm(0.4135)).norm();  // ~10 km
  EXPECT_GT(low_alt, high_alt);
}

TEST(AeroDrag, QuadraticSpeedScaling) {
  // Isolate the v^2 dependence by keeping Cd flat. Use a single-point table so Cd is constant
  // across all Mach numbers (clamped to the lone breakpoint on both ends).
  AeroConfig c;
  c.ref_area = 0.05;
  c.cd0 = 0.3;
  c.cn_alpha = 12.0;
  c.cd_mach = {{0.0, 0.3}};  // constant Cd regardless of Mach
  AeroModel aero(c);

  const AtmSample atm = makeAtm(1.225);
  const double d1 = aero.dragForce(Vector3{50.0, 0.0, 0.0}, atm).norm();
  const double d2 = aero.dragForce(Vector3{100.0, 0.0, 0.0}, atm).norm();
  // Doubling speed quadruples drag when Cd is constant.
  EXPECT_NEAR(d2 / d1, 4.0, 1e-9);
}

// ── C) 6DOF force: drag + normal force from angle of attack ─────────────────────────────────
TEST(Aero6dof, ZeroAoaIsPureDrag) {
  AeroModel aero(tableConfig());
  const Vector3 vel{300.0, 0.0, 0.0};  // velocity along +East
  // Nose aligned with velocity (+x body -> +East world): identity attitude.
  const Quaternion att;  // {1,0,0,0}
  const AtmSample atm = makeAtm(1.225);

  const Vector3 f6 = aero.force6dof(vel, att, atm);
  const Vector3 fd = aero.dragForce(vel, atm);

  // With zero AoA the normal force vanishes, so force6dof ~ pure drag.
  EXPECT_NEAR(f6.x, fd.x, 1e-9);
  EXPECT_NEAR(f6.y, fd.y, 1e-9);
  EXPECT_NEAR(f6.z, fd.z, 1e-9);
}

TEST(Aero6dof, PitchedAttitudeProducesNormalForce) {
  AeroModel aero(tableConfig());
  const Vector3 vel{300.0, 0.0, 0.0};  // velocity along +East (world x)
  const AtmSample atm = makeAtm(1.225);

  // Pitch the nose up 10 deg: rotating about -y takes the body x-axis from +x toward +z (Up).
  // (Rotation about +y would tilt the nose toward -z; see the right-hand rule.)
  const double aoa = 10.0 * M_PI / 180.0;
  const Quaternion att = Quaternion::fromAxisAngle(Vector3{0.0, -1.0, 0.0}, aoa);

  const Vector3 f6 = aero.force6dof(vel, att, atm);
  const Vector3 fd = aero.dragForce(vel, atm);
  const Vector3 normal = f6 - fd;  // isolate the lift contribution

  // A normal force must exist (non-negligible) and act perpendicular to velocity.
  const Vector3 v_hat = vel.normalized();
  EXPECT_GT(normal.norm(), 1.0);
  EXPECT_NEAR(normal.dot(v_hat), 0.0, 1e-6);
  // Nose pitched up -> lift points toward the Up side (+z).
  EXPECT_GT(normal.z, 0.0);
}

TEST(Aero6dof, ZeroSpeedGivesZeroForce) {
  AeroModel aero(tableConfig());
  const Quaternion att = Quaternion::fromAxisAngle(Vector3{0.0, 1.0, 0.0}, 0.2);
  const Vector3 f = aero.force6dof(Vector3{0.0, 0.0, 0.0}, att, makeAtm(1.225));
  EXPECT_DOUBLE_EQ(f.norm(), 0.0);
}
