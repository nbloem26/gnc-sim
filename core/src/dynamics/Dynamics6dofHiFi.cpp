// gnc-sim — high-fidelity 6DOF EOM with the full inertia tensor + gyroscopic coupling (issue #35).
//
// Rotational EOM (Euler's equations, body frame):
//     I * omega_dot = M_body - omega x (I * omega)
//  => omega_dot = I^-1 * ( M_body - omega x (I*omega) )
// Together with translation (constant world-frame accel over the step) and the attitude kinematics
// q_dot = 0.5 * q ⊗ [0, omega], the combined 13-state {pos, vel, q, omega} is advanced by the
// chosen fixed-step integrator. Unlike step6dof's scalar-inertia case the angular acceleration is
// state-dependent (the gyroscopic term couples the rates), so a real RK integration of the coupled
// state is required — Euler/RK2/RK4 differ here in fidelity, not just round-off.
//
// The 3x3 inverse is a hand-rolled cofactor/adjugate (same approach as core/src/gnc/Ekf.cpp).
// Pure arithmetic, fixed evaluation order -> deterministic and native<->WASM parity-safe.
#include "gncsim/dynamics/Dynamics6dofHiFi.hpp"

#include <cmath>

#include "gncsim/math/Quaternion.hpp"

namespace gncsim {

namespace {

// 3x3 matrix inverse via cofactors / adjugate (row-major). Returns false if (near-)singular.
bool invert3x3(const std::array<double, 9>& m, std::array<double, 9>& out) {
  const double a = m[0], b = m[1], c = m[2];
  const double d = m[3], e = m[4], f = m[5];
  const double g = m[6], h = m[7], i = m[8];

  const double A = e * i - f * h;
  const double B = -(d * i - f * g);
  const double C = d * h - e * g;
  const double det = a * A + b * B + c * C;
  if (std::fabs(det) < 1e-300) return false;
  const double inv_det = 1.0 / det;

  out[0] = A * inv_det;
  out[1] = (c * h - b * i) * inv_det;
  out[2] = (b * f - c * e) * inv_det;
  out[3] = B * inv_det;
  out[4] = (a * i - c * g) * inv_det;
  out[5] = (c * d - a * f) * inv_det;
  out[6] = C * inv_det;
  out[7] = (b * g - a * h) * inv_det;
  out[8] = (a * e - b * d) * inv_det;
  return true;
}

Vector3 matVec(const std::array<double, 9>& m, const Vector3& v) {
  return {m[0] * v.x + m[1] * v.y + m[2] * v.z, m[3] * v.x + m[4] * v.y + m[5] * v.z,
          m[6] * v.x + m[7] * v.y + m[8] * v.z};
}

// Combined 13-state for the coupled rigid-body integration. Provides only the vector-space ops the
// RK substeps need; the quaternion is carried unnormalized between substeps and normalized once at
// the end (exactly as step6dof does).
struct RbState {
  Vector3 pos;
  Vector3 vel;
  Quaternion q;
  Vector3 w;

  RbState operator+(const RbState& o) const {
    return {
        pos + o.pos, vel + o.vel, {q.w + o.q.w, q.x + o.q.x, q.y + o.q.y, q.z + o.q.z}, w + o.w};
  }
  RbState operator*(double s) const {
    return {pos * s, vel * s, {q.w * s, q.x * s, q.y * s, q.z * s}, w * s};
  }
};

}  // namespace

InertiaTensor InertiaTensor::fromComponents(double ixx, double iyy, double izz, double ixy,
                                            double ixz, double iyz) {
  InertiaTensor t;
  // Convention: products of inertia enter the off-diagonals as -ixy etc. (the standard inertia
  // tensor I = [[Ixx,-Ixy,-Ixz],[-Ixy,Iyy,-Iyz],[-Ixz,-Iyz,Izz]]).
  t.I = {ixx, -ixy, -ixz, -ixy, iyy, -iyz, -ixz, -iyz, izz};
  if (!invert3x3(t.I, t.Iinv)) {
    // Non-invertible: fall back to a diagonal proxy from the principal moments so the EOM stays
    // finite. Guard each axis against a zero/negative moment.
    const double gx = ixx > 0.0 ? ixx : 1.0;
    const double gy = iyy > 0.0 ? iyy : 1.0;
    const double gz = izz > 0.0 ? izz : 1.0;
    t.Iinv = {1.0 / gx, 0.0, 0.0, 0.0, 1.0 / gy, 0.0, 0.0, 0.0, 1.0 / gz};
  }
  return t;
}

Vector3 InertiaTensor::apply(const Vector3& w) const { return matVec(I, w); }
Vector3 InertiaTensor::applyInv(const Vector3& w) const { return matVec(Iinv, w); }

InertiaTensor inertiaFromVehicle(const VehicleConfig& v) {
  // A principal moment <= 0 means "use the scalar inertia proxy" (uniform body); products default
  // to zero. This keeps the default hi-fi config a uniform-inertia body and makes the full tensor
  // opt-in by setting ixx/iyy/izz (and optionally the products) explicitly.
  const double ixx = v.ixx > 0.0 ? v.ixx : v.inertia;
  const double iyy = v.iyy > 0.0 ? v.iyy : v.inertia;
  const double izz = v.izz > 0.0 ? v.izz : v.inertia;
  return InertiaTensor::fromComponents(ixx, iyy, izz, v.ixy, v.ixz, v.iyz);
}

EntityState step6dofHiFi(const EntityState& s, const Vector3& force_world,
                         const Vector3& moment_body, const InertiaTensor& inertia,
                         const Vector3& gravity, double dt, Integrator integ) {
  // Translational acceleration (constant over the step, world frame).
  const Vector3 accel = force_world / s.mass + gravity;

  // Derivative of the coupled state. accel and moment_body are held constant across the step (the
  // Runner evaluates them once); the gyroscopic term -omega x (I omega) is recomputed per substep.
  const auto deriv = [&](const RbState& y) -> RbState {
    const Vector3 Iw = inertia.apply(y.w);
    const Vector3 gyro = y.w.cross(Iw);
    const Vector3 wdot = inertia.applyInv(moment_body - gyro);
    return {y.vel, accel, y.q.derivative(y.w), wdot};
  };

  const RbState y0{s.pos, s.vel, s.att, s.angVel};
  RbState y1;
  switch (integ) {
    case Integrator::Euler: {
      const RbState k1 = deriv(y0);
      y1 = y0 + k1 * dt;
      break;
    }
    case Integrator::RK2: {
      const RbState k1 = deriv(y0);
      const RbState k2 = deriv(y0 + k1 * (0.5 * dt));
      y1 = y0 + k2 * dt;
      break;
    }
    case Integrator::RK4:
    default: {
      const RbState k1 = deriv(y0);
      const RbState k2 = deriv(y0 + k1 * (0.5 * dt));
      const RbState k3 = deriv(y0 + k2 * (0.5 * dt));
      const RbState k4 = deriv(y0 + k3 * dt);
      y1 = y0 + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);
      break;
    }
  }

  EntityState out = s;  // copy through mass, mach
  out.t = s.t + dt;
  out.pos = y1.pos;
  out.vel = y1.vel;
  out.angVel = y1.w;
  out.att = y1.q.normalized();
  return out;
}

}  // namespace gncsim
