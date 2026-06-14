// gnc-sim — Guidance, Navigation & Control implementation. World frame is ENU, SI units.
//   Guidance:   computeEngagement() builds the relative geometry; proNavCommand() turns it into
//               a commanded acceleration using True Proportional Navigation.
//   Navigation: Navigator is an alpha-beta tracker producing a smoothed relative state.
//   Control:    Autopilot is a 6DOF acceleration autopilot -> bounded body moment + fin telemetry.
#include "gncsim/gnc/Gnc.hpp"

#include <algorithm>
#include <cmath>

namespace gncsim {

namespace {

// Clamp each component of v to [-limit, +limit]. Used for bounded fin-deflection telemetry.
Vector3 clampVec(const Vector3& v, double limit) {
  return {std::clamp(v.x, -limit, limit), std::clamp(v.y, -limit, limit),
          std::clamp(v.z, -limit, limit)};
}

// Scale a vector down so its magnitude never exceeds limit; direction is preserved at saturation.
Vector3 limitMagnitude(const Vector3& v, double limit) {
  const double mag = v.norm();
  if (mag > limit && mag > 0.0) {
    return v * (limit / mag);
  }
  return v;
}

// The raw (un-limited) True PN acceleration term: N * Vc * (omega x LOS_unit).
Vector3 proNavTerm(const Engagement& e, const GuidanceConfig& cfg) {
  const double N = cfg.nav_constant;
  return N * e.v_closing * e.los_rate_vec.cross(e.los_unit);
}

}  // namespace

// ── Guidance: relative engagement geometry ──────────────────────────────────────────────────
Engagement computeEngagement(const EntityState& vehicle, const EntityState& target) {
  Engagement e;
  e.rel_pos = target.pos - vehicle.pos;  // vehicle -> target
  e.rel_vel = target.vel - vehicle.vel;
  e.range = e.rel_pos.norm();

  // Guard a degenerate (coincident) geometry: leave LOS quantities at their zero defaults.
  if (e.range > 0.0) {
    e.los_unit = e.rel_pos / e.range;

    // Closing speed: positive when range is decreasing. v_closing = -d(range)/dt projected on LOS.
    e.v_closing = -e.rel_vel.dot(e.los_unit);

    // LOS rotation-rate vector: omega = (r x v) / |r|^2. Its magnitude is the LOS angular rate;
    // zero omega means the target stays on a constant bearing (collision course).
    e.los_rate_vec = e.rel_pos.cross(e.rel_vel) / (e.range * e.range);
    e.los_rate = e.los_rate_vec.norm();

    // Elevation of the LOS in the vertical plane (angle above the local horizontal, ENU z = Up).
    const double horizontal = std::sqrt(e.rel_pos.x * e.rel_pos.x + e.rel_pos.y * e.rel_pos.y);
    e.los_angle = std::atan2(e.rel_pos.z, horizontal);
  }
  return e;
}

// ── Guidance: True Proportional Navigation ──────────────────────────────────────────────────
Vector3 proNavCommand(const Engagement& e, const GuidanceConfig& cfg) {
  // Only the "pronav" law produces a command. Don't guide when receding (v_closing <= 0):
  // PN is a terminal-homing law and a diverging engagement has no intercept to steer toward.
  if (cfg.law != "pronav" || e.v_closing <= 0.0) {
    return Vector3{};
  }

  // True PN: a_cmd = N * Vc * (omega x LOS_unit). This is automatically perpendicular to the LOS
  // and drives the LOS rate to zero (the parallel-navigation / constant-bearing intercept).
  // Magnitude limit: scale the whole vector down so its direction is preserved at saturation.
  return limitMagnitude(proNavTerm(e, cfg), cfg.max_accel);
}

// ── Guidance: Augmented Proportional Navigation ─────────────────────────────────────────────
Vector3 augmentedProNavCommand(const Engagement& e, const GuidanceConfig& cfg,
                               const Vector3& a_target_est) {
  // Only the "apn" law produces a command, and only while closing (terminal-homing geometry).
  if (cfg.law != "apn" || e.v_closing <= 0.0) {
    return Vector3{};
  }

  // Target-acceleration feedforward: project the estimated target accel onto the plane ⟂ to the
  // LOS (the along-LOS component does not change the intercept geometry, so it is removed).
  const Vector3 a_t_perp = a_target_est - e.los_unit * a_target_est.dot(e.los_unit);

  // APN: PN term + (N/2) * a_T_perp. The feedforward anticipates the target's lateral maneuver,
  // cancelling the steady-state miss that pure PN leaves against an accelerating target.
  const Vector3 a_cmd = proNavTerm(e, cfg) + (cfg.nav_constant * 0.5) * a_t_perp;
  return limitMagnitude(a_cmd, cfg.max_accel);
}

// ── Navigation: alpha-beta tracker ──────────────────────────────────────────────────────────
void Navigator::update(const Vector3& measured_rel_pos) {
  if (!initialized_) {
    // Bootstrap on the first measurement: position = measurement, velocity unknown (zero).
    rel_pos_ = measured_rel_pos;
    rel_vel_ = Vector3{};
    initialized_ = true;
    return;
  }

  // Predict forward one step using the current velocity estimate.
  const Vector3 p_pred = rel_pos_ + rel_vel_ * dt_;
  // Innovation: how far the measurement landed from the prediction.
  const Vector3 residual = measured_rel_pos - p_pred;

  // Correct position by alpha*residual and velocity by (beta/dt)*residual.
  rel_pos_ = p_pred + alpha_ * residual;
  rel_vel_ = rel_vel_ + (beta_ / dt_) * residual;
}

// ── Control: 6DOF acceleration autopilot ────────────────────────────────────────────────────
Vector3 Autopilot::moment(const EntityState& s, const Vector3& accel_cmd_world,
                          Vector3& fin_out) const {
  // Current nose direction: rotate the body +x axis into the world frame.
  const Vector3 nose = s.att.rotate(Vector3{1.0, 0.0, 0.0});

  // Desired nose direction: bias the velocity vector toward the commanded acceleration so the
  // airframe pitches/yaws to generate the lift that realizes accel_cmd. Falling back to the
  // current nose when both velocity and command vanish keeps the error (and moment) at zero.
  Vector3 desired = s.vel + accel_cmd_world;
  if (desired.norm() <= 0.0) {
    desired = nose;
  }
  const Vector3 desired_hat = desired.normalized();

  // Small-angle attitude error: the rotation that takes the current nose onto the desired nose
  // is, to first order, the cross product of the two unit vectors (axis * sin(angle)). Express it
  // in the body frame so the moment acts on the body axes.
  const Vector3 err_world = nose.cross(desired_hat);
  const Vector3 err_body = s.att.conjugate().rotate(err_world);

  // PD law: proportional on the attitude error, derivative (rate damping) on the body angular
  // rate. kd opposes angVel to keep the response stable and bounded.
  const Vector3 moment = cfg_.kp * err_body - cfg_.kd * s.angVel;

  // Representative fin-deflection telemetry: a small-gain image of the moment, clamped to the
  // mechanical deflection limit so the reported surface command is always physically plausible.
  constexpr double kFinGain = 0.05;
  fin_out = clampVec(moment * kFinGain, cfg_.max_fin_deflection);

  return moment;
}

// ── Control: first-order fin actuator with rate / deflection limits (issue #35) ──────────────
Vector3 FinActuator::allocate(const Vector3& moment_cmd) const {
  const double k = cfg_.effectiveness;
  if (k <= 0.0) return Vector3{};
  // Commanded deflection = moment / effectiveness, clamped to the mechanical travel limit.
  return clampVec(moment_cmd / k, cfg_.deflection_limit);
}

Vector3 FinActuator::controlMoment(const Vector3& deflection) const {
  return deflection * cfg_.effectiveness;
}

Vector3 FinActuator::step(const Vector3& deflection_cmd) {
  const Vector3 cmd = clampVec(deflection_cmd, cfg_.deflection_limit);

  // First-order lag toward the command: blend = dt/tau (clamped to 1 for tau <= dt).
  const double blend = cfg_.tau > 0.0 ? std::min(dt_ / cfg_.tau, 1.0) : 1.0;
  Vector3 target = defl_ + (cmd - defl_) * blend;

  // Per-axis change after the lag, then rate-limited to rate_limit*dt and travel-clamped.
  const double max_step = cfg_.rate_limit * dt_;
  const auto advance = [&](double cur, double tgt) -> double {
    double d = tgt - cur;
    if (d > max_step) d = max_step;
    if (d < -max_step) d = -max_step;
    double next = cur + d;
    if (next > cfg_.deflection_limit) next = cfg_.deflection_limit;
    if (next < -cfg_.deflection_limit) next = -cfg_.deflection_limit;
    return next;
  };
  defl_ = {advance(defl_.x, target.x), advance(defl_.y, target.y), advance(defl_.z, target.z)};
  return defl_;
}

}  // namespace gncsim
