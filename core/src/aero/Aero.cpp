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

// Bilinear interpolation of a coefficient over an (alpha, mach, coeff) scatter table (issue #35).
// The table is treated as samples on the (alpha, mach) plane: we find the bracketing alpha
// breakpoints (the table is grouped/ascending in alpha) and, within each, interpolate over mach,
// then interpolate the two alpha slices. Inputs are clamped to the table's alpha/mach extents
// (edge-hold), matching dragCoefficient's endpoint-clamp behaviour. Empty table -> caller's
// fallback. Deterministic, fixed evaluation order.
double interpAlphaMach(const std::vector<std::array<double, 3>>& table, double alpha, double mach) {
  if (table.empty()) return 0.0;

  // Collect the distinct, ascending alpha breakpoints. Tables are authored grouped by alpha, but we
  // do not rely on order: scan for the bracketing alpha values around the query.
  double a_lo = table.front()[0], a_hi = table.front()[0];
  for (const auto& row : table) {
    if (row[0] < a_lo) a_lo = row[0];
    if (row[0] > a_hi) a_hi = row[0];
  }
  const double a_clamped = std::max(a_lo, std::min(a_hi, alpha));

  // Nearest alpha breakpoint below (a0) and above (a1) the query.
  double a0 = a_lo, a1 = a_hi;
  for (const auto& row : table) {
    if (row[0] <= a_clamped && row[0] > a0) a0 = row[0];
    if (row[0] >= a_clamped && row[0] < a1) a1 = row[0];
  }
  if (a0 > a1) a0 = a1;  // query at/under the lowest breakpoint

  // Interpolate the coefficient over mach within the rows at a given alpha breakpoint.
  const auto machSlice = [&](double a_bp) -> double {
    double m_lo = 0.0, m_hi = 0.0, c_lo = 0.0, c_hi = 0.0;
    bool have_lo = false, have_hi = false;
    for (const auto& row : table) {
      if (row[0] != a_bp) continue;
      const double m = row[1], c = row[2];
      if (m <= mach && (!have_lo || m > m_lo)) {
        m_lo = m;
        c_lo = c;
        have_lo = true;
      }
      if (m >= mach && (!have_hi || m < m_hi)) {
        m_hi = m;
        c_hi = c;
        have_hi = true;
      }
    }
    if (have_lo && have_hi) {
      const double span = m_hi - m_lo;
      if (span <= 0.0) return c_lo;
      return c_lo + (mach - m_lo) / span * (c_hi - c_lo);
    }
    if (have_lo) return c_lo;  // mach above all samples at this alpha -> clamp high
    if (have_hi) return c_hi;  // mach below all samples -> clamp low
    return 0.0;
  };

  const double c0 = machSlice(a0);
  const double c1 = machSlice(a1);
  const double da = a1 - a0;
  if (da <= 0.0) return c0;
  return c0 + (a_clamped - a0) / da * (c1 - c0);
}
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

  const double normal_mag =
      0.5 * atm.density * speed * speed * cfg_.ref_area * cfg_.cn_alpha * alpha;
  const Vector3 normal = lift_dir * normal_mag;

  return drag + normal;
}

// ── High-fidelity 6DOF aero (issue #35) ─────────────────────────────────────────────────────

double AeroModel::normalForceCoeff(double alpha, double mach) const {
  if (!cfg_.cn_table.empty()) return interpAlphaMach(cfg_.cn_table, alpha, mach);
  return cfg_.cn_alpha * alpha;  // linear fallback
}

double AeroModel::pitchMomentCoeff(double alpha, double mach) const {
  if (!cfg_.cm_table.empty()) return interpAlphaMach(cfg_.cm_table, alpha, mach);
  return cfg_.cm_alpha * alpha;  // linear fallback (cm_alpha < 0 => statically stable)
}

Vector3 AeroModel::force6dofHiFi(const Vector3& vel_world, const Quaternion& att,
                                 const AtmSample& atm) const {
  const Vector3 drag = dragForce(vel_world, atm);

  const double speed = vel_world.norm();
  if (speed <= kMinSpeed) return drag;

  const Vector3 v_hat = vel_world / speed;
  const Vector3 nose = att.rotate(Vector3{1.0, 0.0, 0.0}).normalized();

  double cos_alpha = std::max(-1.0, std::min(1.0, nose.dot(v_hat)));
  const double alpha = std::acos(cos_alpha);
  if (alpha <= kMinAlpha) return drag;

  const Vector3 perp = nose - v_hat * nose.dot(v_hat);
  const double perp_norm = perp.norm();
  if (perp_norm <= kMinAlpha) return drag;
  const Vector3 lift_dir = perp / perp_norm;

  const double mach = atm.speed_of_sound > 0.0 ? speed / atm.speed_of_sound : 0.0;
  const double cn = normalForceCoeff(alpha, mach);
  const double q_bar = 0.5 * atm.density * speed * speed;
  const Vector3 normal = lift_dir * (q_bar * cfg_.ref_area * cn);
  return drag + normal;
}

Vector3 AeroModel::momentAero(const Vector3& vel_world, const Quaternion& att,
                              const Vector3& ang_vel_body, const AtmSample& atm) const {
  const double speed = vel_world.norm();
  if (speed <= kMinSpeed) return Vector3{};

  const Vector3 v_hat = vel_world / speed;
  const Vector3 nose = att.rotate(Vector3{1.0, 0.0, 0.0}).normalized();
  double cos_alpha = std::max(-1.0, std::min(1.0, nose.dot(v_hat)));
  const double alpha = std::acos(cos_alpha);

  const double mach = atm.speed_of_sound > 0.0 ? speed / atm.speed_of_sound : 0.0;
  const double q_bar = 0.5 * atm.density * speed * speed;
  const double qSd = q_bar * cfg_.ref_area * cfg_.ref_length;  // dynamic-pressure * S * d

  // --- Static restoring moment: Cm(alpha) about the body axis perpendicular to the alpha plane.
  // --- The relative-wind direction in the body frame; alpha is the tilt of the nose (+x) off it.
  // The restoring moment acts to rotate the nose back toward the wind, about the axis nose x wind.
  Vector3 moment_body{};
  if (alpha > kMinAlpha) {
    const Vector3 wind_body = att.conjugate().rotate(v_hat);  // relative-wind unit, body frame
    Vector3 axis = Vector3{1.0, 0.0, 0.0}.cross(wind_body);   // pitch/yaw axis in body frame
    const double axis_norm = axis.norm();
    if (axis_norm > kMinAlpha) {
      axis = axis / axis_norm;
      // `axis` (= x_body x wind) is the right-hand axis that rotates the nose back toward the wind,
      // i.e. the restoring direction. Cm(alpha) < 0 encodes static stability, so the restoring
      // moment magnitude is qSd*(-Cm) along that axis (a stable airframe pushes the nose to wind).
      const double cm = pitchMomentCoeff(alpha, mach);
      moment_body += axis * (qSd * (-cm));
    }
  }

  // --- Rate damping: pitch/yaw (cm_q) on the lateral body rates, roll (cl_p) on the roll rate. ---
  // Nondimensional rate = omega * d / (2 V). The damping moment opposes the body rate.
  const double rate_scale = qSd * cfg_.ref_length / (2.0 * speed);
  moment_body.x += rate_scale * cfg_.cl_p * ang_vel_body.x;  // roll damping
  moment_body.y += rate_scale * cfg_.cm_q * ang_vel_body.y;  // pitch damping
  moment_body.z += rate_scale * cfg_.cm_q * ang_vel_body.z;  // yaw damping (same Cmq by symmetry)

  return moment_body;
}

}  // namespace gncsim
