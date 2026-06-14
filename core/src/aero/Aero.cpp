// gnc-sim — aerodynamics implementation: Mach-dependent drag and 6DOF normal force.
// World frame is local ENU (see docs/DATA_CONTRACT.md). SI units throughout.
#include "gncsim/aero/Aero.hpp"

#include <algorithm>
#include <cmath>

namespace gncsim {

namespace {
// Below this speed [m/s] aerodynamic forces are treated as zero to avoid dividing by a
// vanishingly small velocity magnitude when forming the unit velocity direction.
constexpr double kMinSpeed = 1e-9;
// Below this angle-of-attack [rad] the normal (lift) force is treated as zero; also guards
// the perpendicular-direction construction from a near-singular projection.
constexpr double kMinAlpha = 1e-9;
}  // namespace

// Linear interpolation over the (mach, Cd) breakpoint table.
// - Empty table: fall back to the constant cd0.
// - mach at or below the first breakpoint: clamp to that breakpoint's Cd.
// - mach at or above the last breakpoint: clamp to that breakpoint's Cd.
// - otherwise: linearly interpolate between the bracketing breakpoints.
double AeroModel::dragCoefficient(double mach) const {
  const auto& table = cfg_.cd_mach;
  if (table.empty()) return cfg_.cd0;

  // Clamp to endpoints (table is ascending in mach).
  if (mach <= table.front()[0]) return table.front()[1];
  if (mach >= table.back()[0]) return table.back()[1];

  // Find the first breakpoint whose mach is strictly greater than the query, then interpolate
  // between it and its predecessor.
  for (std::size_t i = 1; i < table.size(); ++i) {
    const double m1 = table[i][0];
    if (mach <= m1) {
      const double m0 = table[i - 1][0];
      const double c0 = table[i - 1][1];
      const double c1 = table[i][1];
      const double span = m1 - m0;
      // Degenerate (duplicate mach) breakpoints: avoid a zero-divide, return the lower value.
      if (span <= 0.0) return c0;
      const double frac = (mach - m0) / span;
      return c0 + frac * (c1 - c0);
    }
  }
  // Unreachable given the endpoint clamps above, but keep a safe fallback.
  return table.back()[1];
}

// 3DOF drag: F = -0.5 * rho * speed^2 * Cd(mach) * S * v_hat. Drag opposes velocity.
Vector3 AeroModel::dragForce(const Vector3& vel_world, const AtmSample& atm) const {
  const double speed = vel_world.norm();
  if (speed <= kMinSpeed) return Vector3{};

  // Mach requires a valid speed of sound; if unavailable, treat as Mach 0 (table low end / cd0).
  const double mach = (atm.speed_of_sound > 0.0) ? speed / atm.speed_of_sound : 0.0;
  const double cd = dragCoefficient(mach);

  const Vector3 v_hat = vel_world / speed;
  const double drag_mag = 0.5 * atm.density * speed * speed * cd * cfg_.ref_area;
  return v_hat * (-drag_mag);
}

// 6DOF aerodynamic force: drag (anti-velocity) + normal force from angle of attack between the
// body x-axis ("nose") and the velocity vector. The normal force acts perpendicular to the
// velocity, in the plane spanned by velocity and nose, toward the nose side.
Vector3 AeroModel::force6dof(const Vector3& vel_world, const Quaternion& att,
                             const AtmSample& atm) const {
  const Vector3 drag = dragForce(vel_world, atm);

  const double speed = vel_world.norm();
  if (speed <= kMinSpeed) return drag;  // No relative wind -> no normal force.

  const Vector3 v_hat = vel_world / speed;
  const Vector3 nose = att.rotate(Vector3{1.0, 0.0, 0.0}).normalized();

  // Angle of attack: angle between nose and velocity. Clamp the dot to [-1,1] for numerical
  // safety before acos.
  double cos_alpha = nose.dot(v_hat);
  cos_alpha = std::max(-1.0, std::min(1.0, cos_alpha));
  const double alpha = std::acos(cos_alpha);
  if (alpha <= kMinAlpha) return drag;  // Essentially aligned -> negligible lift.

  // Component of the nose direction perpendicular to velocity (the lift direction). Normalize it;
  // if it collapses (nose nearly (anti)parallel to v), there is no well-defined lift plane.
  const Vector3 perp = nose - v_hat * nose.dot(v_hat);
  const double perp_norm = perp.norm();
  if (perp_norm <= kMinAlpha) return drag;
  const Vector3 lift_dir = perp / perp_norm;

  const double normal_mag = 0.5 * atm.density * speed * speed * cfg_.ref_area * cfg_.cn_alpha * alpha;
  const Vector3 normal = lift_dir * normal_mag;

  return drag + normal;
}

}  // namespace gncsim
