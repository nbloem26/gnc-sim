// gnc-sim — rigid-body equations of motion + integration (Phase 1 dynamics).
//
// step3dof  : translational EOM only (point mass).
// step6dof  : translational EOM + scalar-inertia rotational EOM with quaternion attitude.
//
// Both reuse the generic fixed-step integrators in Integrators.hpp. To do so we define tiny
// internal "state" types that expose operator+ and operator*(double) — the only vector-space
// ops the integrators require — and supply a derivative functor f(t, y) -> dy/dt.
//
// All accelerations are constant across a step (forces/moments are evaluated once by the Runner
// and passed in), so for constant-acceleration motion RK2/RK4 reproduce the exact analytic
// solution; Euler does not (first-order). The rotational case is genuinely nonlinear in the
// quaternion, so the integrator order matters there.

#include "gncsim/dynamics/Dynamics.hpp"

#include "gncsim/math/Integrators.hpp"
#include "gncsim/math/Quaternion.hpp"
#include "gncsim/math/Vector3.hpp"

namespace gncsim {

namespace {

// ---------------------------------------------------------------------------
// Translational 6-state: {pos, vel}. Derivative is {vel, accel} with accel
// held constant over the step.
// ---------------------------------------------------------------------------
struct TransState {
  Vector3 pos;
  Vector3 vel;

  // Vector-space ops required by the generic integrators.
  TransState operator+(const TransState& o) const { return {pos + o.pos, vel + o.vel}; }
  TransState operator*(double s) const { return {pos * s, vel * s}; }
};

// Integrate {pos, vel} one step under a constant world-frame acceleration `accel`.
TransState integrateTranslation(const TransState& y0, const Vector3& accel, double dt,
                                Integrator integ) {
  // Derivative: d/dt {pos, vel} = {vel, accel}. Time/state-independent accel -> ignore both args.
  const auto deriv = [&accel](double /*t*/, const TransState& y) -> TransState {
    return {y.vel, accel};
  };
  switch (integ) {
    case Integrator::Euler:
      return eulerStep(y0, 0.0, dt, deriv);
    case Integrator::RK2:
      return rk2Step(y0, 0.0, dt, deriv);
    case Integrator::RK4:
    default:
      return rk4Step(y0, 0.0, dt, deriv);
  }
}

// ---------------------------------------------------------------------------
// Quaternion 4-state wrapper so the generic integrators can advance attitude.
// The derivative is qdot = 0.5 * q ⊗ [0, omega]. We treat omega as constant
// across the substeps of a single integrator step (omega itself is integrated
// separately by the translational/rotational rate update).
// ---------------------------------------------------------------------------
struct QuatState {
  Quaternion q;

  QuatState operator+(const QuatState& o) const {
    return {{q.w + o.q.w, q.x + o.q.x, q.y + o.q.y, q.z + o.q.z}};
  }
  QuatState operator*(double s) const { return {{q.w * s, q.x * s, q.y * s, q.z * s}}; }
};

// Integrate the (unnormalized) quaternion one step for a fixed body rate `omega`.
// Result is NOT renormalized here — caller renormalizes once after the step.
QuatState integrateAttitude(const QuatState& y0, const Vector3& omega, double dt,
                            Integrator integ) {
  const auto deriv = [&omega](double /*t*/, const QuatState& y) -> QuatState {
    return {y.q.derivative(omega)};
  };
  switch (integ) {
    case Integrator::Euler:
      return eulerStep(y0, 0.0, dt, deriv);
    case Integrator::RK2:
      return rk2Step(y0, 0.0, dt, deriv);
    case Integrator::RK4:
    default:
      return rk4Step(y0, 0.0, dt, deriv);
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// 3DOF: translational motion of a point mass.
// Total acceleration a = force_world / mass + gravity  (gravity already an accel vector).
// ---------------------------------------------------------------------------
EntityState step3dof(const EntityState& s, const Vector3& force_world, const Vector3& gravity,
                     double dt, Integrator integ) {
  const Vector3 accel_mps2 = force_world / s.mass + gravity;

  const TransState y1 = integrateTranslation({s.pos, s.vel}, accel_mps2, dt, integ);

  EntityState out = s;  // copy through att, angVel, mass, mach
  out.t = s.t + dt;
  out.pos = y1.pos;
  out.vel = y1.vel;
  return out;
}

// ---------------------------------------------------------------------------
// 6DOF: translational (same as 3DOF) + rotational with scalar inertia proxy.
//   angular accel alpha = moment_body / inertia   (gyroscopic cross terms ignored)
//   body rate          omega += alpha * dt        (integrated alongside translation)
//   attitude qdot       = 0.5 * q ⊗ [0, omega]    (integrated, then renormalized)
//
// We advance the body rate with the same integrator (constant alpha -> exact for RK2/RK4),
// then propagate the quaternion holding omega fixed across the step. Using the pre-step omega
// keeps the two updates consistent to the integrator's order for this constant-torque case.
// ---------------------------------------------------------------------------
EntityState step6dof(const EntityState& s, const Vector3& force_world, const Vector3& moment_body,
                     double inertia, const Vector3& gravity, double dt, Integrator integ) {
  // --- Translational (identical to 3DOF). ---
  const Vector3 accel_mps2 = force_world / s.mass + gravity;
  const TransState yt = integrateTranslation({s.pos, s.vel}, accel_mps2, dt, integ);

  // --- Rotational rate: alpha constant over the step. ---
  // Reuse the translational integrator on a {dummy, omega} pair so omega advances with the
  // chosen scheme. Here the omega derivative is the constant alpha, so this is exact for RK2/RK4.
  const Vector3 alpha = moment_body / inertia;
  const TransState yr = integrateTranslation({Vector3{}, s.angVel}, alpha, dt, integ);
  const Vector3 omega_new_radps = yr.vel;

  // --- Attitude: propagate quaternion with the MIDPOINT body rate, then renormalize. ---
  // omega varies linearly across the step (alpha constant), so the step-averaged rate is the
  // midpoint value omega + alpha*dt/2. Using it makes the integrated angle exact for constant
  // torque (the pre-step rate would leave an O(alpha*dt^2)-per-step bias that accumulates).
  const Vector3 omega_mid_radps = s.angVel + alpha * (0.5 * dt);
  const QuatState yq = integrateAttitude({s.att}, omega_mid_radps, dt, integ);

  EntityState out = s;  // copy through mass, mach
  out.t = s.t + dt;
  out.pos = yt.pos;
  out.vel = yt.vel;
  out.angVel = omega_new_radps;
  out.att = yq.q.normalized();  // keep attitude a unit quaternion
  return out;
}

}  // namespace gncsim
