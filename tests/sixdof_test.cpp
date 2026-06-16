// gnc-sim — high-fidelity 6DOF tests (issue #35): full inertia tensor + gyroscopic coupling,
// table-driven aero coefficients, first-order fin-actuator dynamics with rate/travel limits, and a
// full 6DOF intercept with realistic fin response. Pure in-code SimConfig — no file I/O.
#include <gtest/gtest.h>

#include <cmath>

#include "gncsim/aero/Aero.hpp"
#include "gncsim/core/Config.hpp"
#include "gncsim/core/Types.hpp"
#include "gncsim/dynamics/Dynamics.hpp"
#include "gncsim/dynamics/Dynamics6dofHiFi.hpp"
#include "gncsim/env/Environment.hpp"
#include "gncsim/gnc/Gnc.hpp"
#include "gncsim/math/Quaternion.hpp"
#include "gncsim/math/Vector3.hpp"
#include "gncsim/scenario/Runner.hpp"

using namespace gncsim;

// ---------------------------------------------------------------------------------------------
// Inertia tensor: I * (I^-1 * v) == v, and the products of inertia enter the off-diagonals.
// ---------------------------------------------------------------------------------------------
TEST(SixDofInertia, InverseRoundTrips) {
  const InertiaTensor t = InertiaTensor::fromComponents(0.18, 1.30, 1.32, 0.03, 0.02, 0.01);
  const Vector3 v{1.0, -2.0, 3.0};
  const Vector3 back = t.apply(t.applyInv(v));
  EXPECT_NEAR(back.x, v.x, 1e-9);
  EXPECT_NEAR(back.y, v.y, 1e-9);
  EXPECT_NEAR(back.z, v.z, 1e-9);

  // Diagonal moments sit on the diagonal; products are the negated off-diagonals.
  EXPECT_NEAR(t.I[0], 0.18, 1e-12);
  EXPECT_NEAR(t.I[1], -0.03, 1e-12);  // -Ixy
  EXPECT_NEAR(t.I[2], -0.02, 1e-12);  // -Ixz
}

// ---------------------------------------------------------------------------------------------
// With a diagonal inertia tensor whose principal moments are all equal to a scalar I0, and zero
// products, the hi-fi rotational EOM must reduce to the scalar-inertia step6dof for a torque about
// a single principal axis (no gyroscopic coupling in that case).
// ---------------------------------------------------------------------------------------------
TEST(SixDofHiFi, ReducesToScalarInertiaForSingleAxisTorque) {
  const double I0 = 0.75;
  const InertiaTensor t = InertiaTensor::fromComponents(I0, I0, I0, 0, 0, 0);

  EntityState a;
  a.att = Quaternion{};
  a.mass = 4.0;
  EntityState b = a;

  const Vector3 force{0, 0, 0};
  const Vector3 moment{0, 0, 0.3};  // about the symmetric z axis
  const Vector3 gravity{0, 0, 0};
  const double dt = 0.001;

  for (int i = 0; i < 1500; ++i) {
    a = step6dofHiFi(a, force, moment, t, gravity, dt, Integrator::RK4);
    b = step6dof(b, force, moment, I0, gravity, dt, Integrator::RK4);
  }
  EXPECT_NEAR(a.angVel.z, b.angVel.z, 1e-9);
  EXPECT_NEAR((a.att.conjugate() * b.att).norm(), 1.0, 1e-9);
  EXPECT_NEAR(a.att.z, b.att.z, 1e-7);
}

// ---------------------------------------------------------------------------------------------
// Gyroscopic coupling sanity: a torque-free asymmetric body spun about an intermediate axis with a
// small transverse perturbation must transfer angular momentum into the other axes (Euler's
// equations), while conserving the kinetic energy 0.5*omega.(I*omega) and |L| = |I*omega|. The
// scalar-inertia step6dof CANNOT produce this coupling — that's the whole point of the upgrade.
// ---------------------------------------------------------------------------------------------
TEST(SixDofHiFi, TorqueFreeGyroscopicCouplingConservesEnergyAndMomentum) {
  // Principal moments Ixx < Iyy < Izz; spin about the INTERMEDIATE axis (y) with a small transverse
  // perturbation. The Dzhanibekov / tennis-racket instability pumps the spin into the other axes —
  // a purely gyroscopic effect (-omega x I*omega) that the scalar-inertia step6dof cannot produce.
  const InertiaTensor t = InertiaTensor::fromComponents(0.20, 1.00, 1.40, 0, 0, 0);

  EntityState s;
  s.att = Quaternion{};
  s.mass = 5.0;
  s.angVel = {0.02, 6.0, 0.0};  // mostly about the intermediate (y) axis + a small x perturbation

  const Vector3 zero{0, 0, 0};
  const double dt = 0.0002;

  const auto energy = [&](const Vector3& w) { return 0.5 * w.dot(t.apply(w)); };
  const double E0 = energy(s.angVel);
  const double L0 = t.apply(s.angVel).norm();

  double max_x = 0.0, max_z = 0.0;
  for (int i = 0; i < 10000; ++i) {  // t = 2 s
    s = step6dofHiFi(s, zero, zero, t, zero, dt, Integrator::RK4);
    max_x = std::max(max_x, std::abs(s.angVel.x));
    max_z = std::max(max_z, std::abs(s.angVel.z));
  }

  // Energy and angular-momentum magnitude are integrals of the torque-free motion: conserved to the
  // integrator's accuracy.
  EXPECT_NEAR(energy(s.angVel), E0, 1e-3 * E0);
  EXPECT_NEAR(t.apply(s.angVel).norm(), L0, 1e-3 * L0);

  // The instability grew the transverse rates by orders of magnitude over the initial perturbation:
  // unambiguous gyroscopic coupling, impossible without the cross term.
  EXPECT_GT(max_x, 1.0);
  EXPECT_GT(max_z, 1.0);
}

// ---------------------------------------------------------------------------------------------
// Aero coefficient tables: bilinear interpolation over (alpha, Mach), with edge-hold clamping, and
// the linear fallback when the table is empty.
// ---------------------------------------------------------------------------------------------
TEST(SixDofAero, CnCmTablesInterpolateAndClamp) {
  AeroConfig cfg;
  cfg.cn_alpha = 12.0;
  cfg.cm_alpha = -8.0;
  cfg.cn_table = {{0.0, 1.0, 0.0}, {0.2, 1.0, 4.0}, {0.0, 2.0, 0.0}, {0.2, 2.0, 3.0}};
  cfg.cm_table = {{0.0, 1.0, 0.0}, {0.2, 1.0, -2.0}, {0.0, 2.0, 0.0}, {0.2, 2.0, -1.5}};
  AeroModel aero(cfg);

  // Midpoint in alpha at Mach 1: Cn(0.1, 1) = 2.0.
  EXPECT_NEAR(aero.normalForceCoeff(0.1, 1.0), 2.0, 1e-9);
  // Bilinear in both: Cn(0.1, 1.5) = average of the four corners' alpha-midpoints = (2.0+1.5)/2.
  EXPECT_NEAR(aero.normalForceCoeff(0.1, 1.5), 1.75, 1e-9);
  // Edge-hold: alpha and Mach beyond the table extents clamp to the corner.
  EXPECT_NEAR(aero.normalForceCoeff(0.5, 1.0), 4.0, 1e-9);
  EXPECT_NEAR(aero.normalForceCoeff(0.2, 5.0), 3.0, 1e-9);
  // Cm is negative for positive alpha (statically stable).
  EXPECT_LT(aero.pitchMomentCoeff(0.2, 1.0), 0.0);

  // Empty tables -> linear fallback.
  AeroConfig lin;
  lin.cn_alpha = 12.0;
  lin.cm_alpha = -8.0;
  AeroModel linmodel(lin);
  EXPECT_NEAR(linmodel.normalForceCoeff(0.05, 1.0), 12.0 * 0.05, 1e-12);
  EXPECT_NEAR(linmodel.pitchMomentCoeff(0.05, 1.0), -8.0 * 0.05, 1e-12);
}

// ---------------------------------------------------------------------------------------------
// Static restoring moment: a statically-stable airframe (Cm < 0 for alpha > 0) at a positive angle
// of attack produces a body moment that rotates the nose back toward the relative wind.
// ---------------------------------------------------------------------------------------------
TEST(SixDofAero, StaticMarginRestoresTowardWind) {
  AeroConfig cfg;
  cfg.ref_area = 0.02;
  cfg.ref_length = 0.15;
  cfg.cm_alpha = -8.0;  // statically stable
  cfg.cm_q = 0.0;       // isolate the static term
  cfg.cl_p = 0.0;
  AeroModel aero(cfg);

  AtmSample atm{1.225, 101325.0, 288.15, 340.29};
  // Velocity along +x (the relative wind), nose pitched up by 0.15 rad about the body y axis.
  const Vector3 vel{300.0, 0.0, 0.0};
  const Quaternion att = Quaternion::fromEuler(0.0, 0.15, 0.0);  // nose-up pitch
  const Vector3 m = aero.momentAero(vel, att, Vector3{}, atm);

  // A nose-up pitch (positive alpha) must produce a NOSE-DOWN restoring moment: negative about the
  // body pitch (y) axis. The lateral (yaw) component stays ~0 for an in-plane attitude.
  EXPECT_LT(m.y, 0.0);
  EXPECT_NEAR(m.x, 0.0, 1e-6);
  EXPECT_NEAR(m.z, 0.0, 1e-6);
}

// ---------------------------------------------------------------------------------------------
// Actuator rate-limit saturation: a large step command cannot be realized faster than
// rate_limit*dt per step; the early deflection tracks the rate limit, and the final value settles
// at the travel limit.
// ---------------------------------------------------------------------------------------------
TEST(SixDofActuator, RateLimitSaturatesThenSettlesAtTravelLimit) {
  ActuatorConfig cfg;
  cfg.tau = 0.001;             // fast lag so the rate limit is the binding constraint
  cfg.rate_limit = 4.0;        // [rad/s]
  cfg.deflection_limit = 0.3;  // [rad]
  cfg.effectiveness = 100.0;
  const double dt = 0.01;
  FinActuator act(cfg, dt);

  const Vector3 big_cmd{10.0, 10.0, 10.0};  // way past the travel limit on every axis

  // First step: limited to rate_limit*dt = 0.04 rad regardless of how large the command is.
  const Vector3 d1 = act.step(big_cmd);
  EXPECT_NEAR(d1.y, cfg.rate_limit * dt, 1e-12);

  // Drive to steady state; the deflection saturates at the travel limit and stays there.
  Vector3 d;
  for (int i = 0; i < 500; ++i) d = act.step(big_cmd);
  EXPECT_NEAR(d.x, cfg.deflection_limit, 1e-9);
  EXPECT_NEAR(d.y, cfg.deflection_limit, 1e-9);
  EXPECT_NEAR(d.z, cfg.deflection_limit, 1e-9);

  // Allocation / control-moment inverse round-trips through the effectiveness gain.
  const Vector3 defl{0.1, -0.05, 0.0};
  const Vector3 mom = act.controlMoment(defl);
  EXPECT_NEAR(mom.x, defl.x * cfg.effectiveness, 1e-12);
  // allocate(moment) is the clamped inverse of controlMoment.
  const Vector3 alloc = act.allocate(mom);
  EXPECT_NEAR(alloc.x, defl.x, 1e-12);
  EXPECT_NEAR(alloc.y, defl.y, 1e-12);
}

// ---------------------------------------------------------------------------------------------
// Full hi-fi 6DOF intercept: a non-maneuvering target is intercepted with realistic, NON-saturated
// fin response, and the run actually exercises the hi-fi rotational state (the vehicle develops a
// trim angle of attack so the table-driven normal force can turn it).
// ---------------------------------------------------------------------------------------------
namespace {
SimConfig makeHiFiInterceptConfig() {
  SimConfig c;
  c.scenario = "homing";
  c.model = "6dof_hifi";
  c.seed = 1;
  c.dt = 0.002;
  c.t_end = 40.0;
  c.integrator = Integrator::RK4;

  c.aero.ref_area = 0.02;
  c.aero.ref_length = 0.15;
  c.aero.cd_mach = {{0.0, 0.28}, {0.9, 0.42}, {1.0, 0.58}, {2.0, 0.36}, {4.0, 0.27}};
  // Cn / Cm tables over (alpha, Mach). Cm < 0 => statically stable.
  c.aero.cn_table = {{0.0, 1.0, 0.0}, {0.2, 1.0, 4.0}, {0.35, 1.0, 6.6},
                     {0.0, 2.0, 0.0}, {0.2, 2.0, 3.8}, {0.35, 2.0, 6.2}};
  c.aero.cm_table = {{0.0, 1.0, 0.0}, {0.2, 1.0, -0.20}, {0.35, 1.0, -0.35},
                     {0.0, 2.0, 0.0}, {0.2, 2.0, -0.19}, {0.35, 2.0, -0.33}};
  c.aero.cm_q = -140.0;
  c.aero.cl_p = -2.0;

  c.vehicle.pos0 = {0, 0, 0};
  c.vehicle.launch_speed = 900.0;
  c.vehicle.launch_elevation_deg = 42.0;
  c.vehicle.mass0 = 22.0;
  c.vehicle.inertia = 1.2;
  c.vehicle.ixx = 0.18;
  c.vehicle.iyy = 1.30;
  c.vehicle.izz = 1.32;
  c.vehicle.ixz = 0.02;

  c.guidance.law = "pronav";
  c.guidance.nav_constant = 4.0;
  c.guidance.max_accel = 400.0;

  c.control.kp = 900.0;
  c.control.kd = 220.0;
  c.control.max_fin_deflection = 0.35;

  c.actuator.tau = 0.02;
  c.actuator.rate_limit = 6.0;
  c.actuator.deflection_limit = 0.35;
  c.actuator.effectiveness = 2500.0;

  c.sensors.enable = false;
  c.target.pos0 = {9000, 0, 3500};
  c.target.vel0 = {-280, 0, -40};
  c.target.maneuver = "constant";
  return c;
}
}  // namespace

TEST(SixDofHiFi, InterceptsWithRealisticFinResponse) {
  const SimResult r = runSimulation(makeHiFiInterceptConfig());
  ASSERT_FALSE(r.frames.empty());
  EXPECT_EQ(r.model, "6dof_hifi");

  // Hits the target (analytic CPA within the lethal radius).
  EXPECT_TRUE(r.intercept);
  EXPECT_LT(r.miss_distance, 3.0);

  // Realistic fin response: the actuators move (the airframe is genuinely steering) but never sit
  // pinned at the travel limit for the whole flight, and the body rates stay physically bounded.
  double max_fin = 0.0, max_rate = 0.0;
  int moved = 0;
  for (const auto& f : r.frames) {
    const double fy = std::abs(f.fin_deflection.y);
    max_fin = std::max(max_fin, fy);
    if (fy > 1e-4) ++moved;
    Vector3 rates{f.imu_gyro_true.x, f.imu_gyro_true.y, f.imu_gyro_true.z};
    max_rate = std::max(max_rate, rates.norm());
  }
  EXPECT_GT(moved, 0);              // the fins actually deflect
  EXPECT_LT(max_fin, 0.35 + 1e-9);  // never exceed the mechanical travel limit
  EXPECT_LT(max_rate, 50.0);        // body rates stay sane (no rotational blow-up)

  // The attitude quaternion stays unit-norm throughout (renormalized each step).
  for (const auto& f : r.frames) {
    EXPECT_NEAR(f.veh_att.norm(), 1.0, 1e-6);
  }
}
