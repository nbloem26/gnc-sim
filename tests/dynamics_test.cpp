// gnc-sim — Phase 1 dynamics tests: integrator/EOM correctness for step3dof / step6dof.
// These mirror the Python validation suite's analytic checks (ballistic, free-fall apex,
// constant-torque spin-up, attitude unit-norm preservation).
#include <cmath>

#include <gtest/gtest.h>

#include "gncsim/core/Config.hpp"
#include "gncsim/core/Types.hpp"
#include "gncsim/dynamics/Dynamics.hpp"
#include "gncsim/math/Quaternion.hpp"
#include "gncsim/math/Vector3.hpp"

using namespace gncsim;

namespace {
constexpr double kG = 9.81;  // test gravity magnitude [m/s^2]
}

// ---------------------------------------------------------------------------
// Drag-free ballistic: zero applied force, only gravity. With constant accel,
// RK4 reproduces the analytic kinematic solution to machine precision.
//   pos(t) = pos0 + v0*t + 0.5*g*t^2
//   vel(t) = v0 + g*t
// ---------------------------------------------------------------------------
TEST(Dynamics3dof, DragFreeBallisticRK4MatchesAnalytic) {
  EntityState s;
  s.t = 0.0;
  s.pos = {0, 0, 0};
  s.vel = {100, 0, 100};
  s.mass = 22.0;  // arbitrary; force is zero so mass is irrelevant

  const Vector3 gravity{0, 0, -kG};
  const Vector3 zeroForce{0, 0, 0};
  const double dt = 0.001;
  const int steps = 2000;  // -> t = 2 s

  for (int i = 0; i < steps; ++i) {
    s = step3dof(s, zeroForce, gravity, dt, Integrator::RK4);
  }

  const double t = dt * steps;
  const Vector3 v0{100, 0, 100};
  const Vector3 pos_exact = v0 * t + gravity * (0.5 * t * t);
  const Vector3 vel_exact = v0 + gravity * t;

  EXPECT_NEAR(s.t, t, 1e-9);
  EXPECT_LT((s.pos - pos_exact).norm(), 1e-6);
  EXPECT_LT((s.vel - vel_exact).norm(), 1e-9);
}

// ---------------------------------------------------------------------------
// Free fall apex: thrown straight up, the time to apex is t* = v0/g and the
// apex height is h* = v0^2/(2g). Integrate to t* and compare with RK4.
// ---------------------------------------------------------------------------
TEST(Dynamics3dof, FreeFallApexRK4MatchesAnalytic) {
  const double v0 = 50.0;
  EntityState s;
  s.pos = {0, 0, 0};
  s.vel = {0, 0, v0};
  s.mass = 5.0;

  const Vector3 gravity{0, 0, -kG};
  const Vector3 zeroForce{0, 0, 0};

  const double t_apex = v0 / kG;            // analytic time to apex
  const double h_apex = v0 * v0 / (2 * kG);  // analytic apex height

  const double dt = 1e-4;
  const int steps = static_cast<int>(std::lround(t_apex / dt));

  for (int i = 0; i < steps; ++i) {
    s = step3dof(s, zeroForce, gravity, dt, Integrator::RK4);
  }

  // The apex time is not an exact multiple of dt, so the last sampled step lands within dt/2 of
  // the true apex: vertical velocity is therefore bounded by ~g*dt (RK4 velocity is otherwise
  // exact for constant accel). Height near the apex is flat, so it matches tightly.
  EXPECT_NEAR(s.vel.z, 0.0, kG * dt);
  EXPECT_NEAR(s.pos.z, h_apex, 1e-4);
}

// ---------------------------------------------------------------------------
// RK4 is exact for constant accel but Euler is not — sanity check that the
// integrator selection actually changes the result (Euler accumulates error).
// ---------------------------------------------------------------------------
TEST(Dynamics3dof, EulerLessAccurateThanRK4) {
  const auto run = [](Integrator integ) {
    EntityState s;
    s.vel = {0, 0, 0};
    s.mass = 1.0;
    const Vector3 g{0, 0, -kG};
    const Vector3 f{0, 0, 0};
    const double dt = 0.01;
    for (int i = 0; i < 100; ++i) s = step3dof(s, f, g, dt, integ);  // t = 1 s
    return s.pos.z;
  };
  const double exact = -0.5 * kG * 1.0 * 1.0;  // -4.905 m
  const double zEuler = run(Integrator::Euler);
  const double zRK4 = run(Integrator::RK4);
  EXPECT_GT(std::abs(zEuler - exact), std::abs(zRK4 - exact));
  EXPECT_NEAR(zRK4, exact, 1e-9);
}

// ---------------------------------------------------------------------------
// 6DOF translational must reduce to the 3DOF result with zero moment.
// ---------------------------------------------------------------------------
TEST(Dynamics6dof, TranslationMatches3dof) {
  EntityState s;
  s.pos = {0, 0, 0};
  s.vel = {30, -10, 80};
  s.mass = 12.0;
  s.att = Quaternion{};  // identity

  const Vector3 force{50, 0, 0};
  const Vector3 gravity{0, 0, -kG};
  const Vector3 zeroMoment{0, 0, 0};
  const double inertia = 1.2;
  const double dt = 0.002;

  EntityState s3 = s, s6 = s;
  for (int i = 0; i < 500; ++i) {
    s3 = step3dof(s3, force, gravity, dt, Integrator::RK4);
    s6 = step6dof(s6, force, zeroMoment, inertia, gravity, dt, Integrator::RK4);
  }
  EXPECT_LT((s3.pos - s6.pos).norm(), 1e-12);
  EXPECT_LT((s3.vel - s6.vel).norm(), 1e-12);
}

// ---------------------------------------------------------------------------
// Constant moment about z, zero initial omega -> omega_z = (M/I)*t exactly for
// RK4 (constant angular accel). Also verify the quaternion stays unit-norm over
// many steps.
// ---------------------------------------------------------------------------
TEST(Dynamics6dof, ConstantTorqueSpinUpAndUnitNorm) {
  const double inertia = 0.75;
  const double Mz = 0.3;  // [N*m]

  EntityState s;
  s.pos = {0, 0, 0};
  s.vel = {0, 0, 0};
  s.att = Quaternion{};   // identity
  s.angVel = {0, 0, 0};
  s.mass = 4.0;

  const Vector3 force{0, 0, 0};
  const Vector3 moment{0, 0, Mz};
  const Vector3 gravity{0, 0, 0};  // isolate rotational dynamics
  const double dt = 0.001;
  const int steps = 1500;  // t = 1.5 s

  double maxNormErr = 0.0;
  for (int i = 0; i < steps; ++i) {
    s = step6dof(s, force, moment, inertia, gravity, dt, Integrator::RK4);
    maxNormErr = std::max(maxNormErr, std::abs(s.att.norm() - 1.0));
  }

  const double t = dt * steps;
  const double omega_exact = (Mz / inertia) * t;
  EXPECT_NEAR(s.angVel.z, omega_exact, 1e-9);
  EXPECT_NEAR(s.angVel.x, 0.0, 1e-12);
  EXPECT_NEAR(s.angVel.y, 0.0, 1e-12);

  // Quaternion remains a unit quaternion throughout (renormalized each step).
  EXPECT_LT(maxNormErr, 1e-9);

  // Rotation about z by total angle theta = 0.5*(M/I)*t^2 -> yaw should match.
  const double theta = 0.5 * (Mz / inertia) * t * t;
  const Vector3 euler = s.att.toEuler();
  EXPECT_NEAR(euler.z, std::atan2(std::sin(theta), std::cos(theta)), 1e-6);
}

// ---------------------------------------------------------------------------
// Zero moment & zero initial omega -> attitude unchanged.
// ---------------------------------------------------------------------------
TEST(Dynamics6dof, ZeroMomentLeavesAttitudeUnchanged) {
  EntityState s;
  s.att = Quaternion::fromEuler(0.1, -0.2, 0.3);  // arbitrary non-identity attitude
  s.angVel = {0, 0, 0};
  s.mass = 3.0;
  const Quaternion att0 = s.att.normalized();

  const Vector3 force{0, 0, 0};
  const Vector3 moment{0, 0, 0};
  const Vector3 gravity{0, 0, -kG};
  const double inertia = 1.0;
  const double dt = 0.005;

  for (int i = 0; i < 400; ++i) {
    s = step6dof(s, force, moment, inertia, gravity, dt, Integrator::RK4);
  }

  EXPECT_NEAR(s.att.w, att0.w, 1e-12);
  EXPECT_NEAR(s.att.x, att0.x, 1e-12);
  EXPECT_NEAR(s.att.y, att0.y, 1e-12);
  EXPECT_NEAR(s.att.z, att0.z, 1e-12);
  EXPECT_NEAR(s.angVel.norm(), 0.0, 1e-15);
}
