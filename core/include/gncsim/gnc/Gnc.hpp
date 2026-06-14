// gnc-sim — Guidance, Navigation & Control. Phase 1 (gnc) owns core/src/gnc/.
//   Guidance:  Proportional Navigation -> commanded acceleration.
//   Navigation: alpha-beta estimator turning noisy seeker/IMU into a relative-state estimate.
//   Control:   6DOF acceleration autopilot -> body moment / fin deflection.
#pragma once

#include "gncsim/core/Config.hpp"
#include "gncsim/core/Types.hpp"
#include "gncsim/math/Vector3.hpp"

namespace gncsim {

// Relative engagement geometry between vehicle and target (world frame).
struct Engagement {
  Vector3 rel_pos;         // target - vehicle [m]
  Vector3 rel_vel;         // target_vel - vehicle_vel [m/s]
  Vector3 los_unit;        // unit line-of-sight (vehicle->target)
  Vector3 los_rate_vec;    // LOS rotation rate vector [rad/s]
  double range = 0.0;      // |rel_pos| [m]
  double v_closing = 0.0;  // closing speed [m/s] (positive = closing)
  double los_angle = 0.0;  // LOS elevation angle in the vertical plane [rad]
  double los_rate = 0.0;   // |los_rate_vec| [rad/s]
};

Engagement computeEngagement(const EntityState& vehicle, const EntityState& target);

// Proportional Navigation: a_cmd = N * Vc * (LOS_unit x los_rate_vec), magnitude-limited.
Vector3 proNavCommand(const Engagement& e, const GuidanceConfig& cfg);

// Augmented Proportional Navigation: PN plus a target-acceleration feedforward.
//   a_cmd = N * Vc * (omega x LOS_unit) + (N/2) * a_T_perp
// where a_T_perp is the estimated target acceleration projected perpendicular to the LOS. The
// feedforward cancels the steady-state miss PN alone leaves against an accelerating target. The
// result is magnitude-limited to cfg.max_accel. a_target_est is supplied by the caller (estimated
// at the runner level from the navigation relative-velocity derivative).
Vector3 augmentedProNavCommand(const Engagement& e, const GuidanceConfig& cfg,
                               const Vector3& a_target_est);

// Time-to-go estimate [s] for the predictive (ZEM/ZEV) laws: range / closing speed, floored at
// cfg.zemzev.tgo_floor_s so the 1/tgo^2 law never blows up. Returns the floor when not closing.
double timeToGo(const Engagement& e, const GuidanceConfig& cfg);

// Optimal ZEM/ZEV guidance (issue #40). Energy-optimal terminal/midcourse law:
//   a_cmd = (N_zem/tgo^2) * ZEM  +  weight(range) * (N_zev/tgo) * ZEV
// ZEM (zero-effort miss) = the predicted relative position at intercept if neither side
// accelerates further, including the estimated target acceleration `a_target_est`:
//   ZEM = rel_pos + rel_vel*tgo + 0.5*a_target_est*tgo^2.
// ZEV (zero-effort velocity error) = predicted relative velocity at intercept minus the desired
// closing velocity (drives the midcourse geometry); faded out near the handover range so the
// command is continuous across the midcourse->terminal switch. The result is magnitude-limited to
// cfg.max_accel. Only the "zemzev" law produces a command, and only while closing. `tgo_s` may be
// supplied by the caller (runner-level filtered estimate); 0 means "compute from the geometry".
Vector3 zemZevCommand(const Engagement& e, const GuidanceConfig& cfg, const Vector3& a_target_est,
                      double tgo_s = 0.0);

// Alpha-beta tracker for the relative position/velocity from noisy LOS + range. Construct once
// per run; call update() each step. Produces nav_pos_est / nav_vel_est used by guidance.
class Navigator {
 public:
  explicit Navigator(double dt, double alpha = 0.5, double beta = 0.1)
      : dt_(dt), alpha_(alpha), beta_(beta) {}

  // Feed a measured relative position (e.g. reconstructed from seeker LOS + range estimate).
  void update(const Vector3& measured_rel_pos);
  Vector3 relPos() const { return rel_pos_; }
  Vector3 relVel() const { return rel_vel_; }

 private:
  double dt_, alpha_, beta_;
  bool initialized_ = false;
  Vector3 rel_pos_, rel_vel_;
};

// 6DOF autopilot: convert a world-frame acceleration command into a body moment / fin deflection.
class Autopilot {
 public:
  explicit Autopilot(const ControlConfig& cfg) : cfg_(cfg) {}

  // Returns body moment [N*m]; writes the commanded fin deflection [rad] to fin_out for telemetry.
  Vector3 moment(const EntityState& s, const Vector3& accel_cmd_world, Vector3& fin_out) const;

 private:
  ControlConfig cfg_;
};

// First-order fin actuator with rate + deflection limits (issue #35). Models the lag between a
// commanded body-axis deflection and the realized one. Per axis the deflection follows the command
// through a first-order lag (time constant tau); the per-step change is rate-limited to
// rate_limit*dt and the absolute deflection clamped to deflection_limit. State is the current
// realized deflection [rad] per body axis (x=roll, y=pitch, z=yaw).
//
// Control allocation: the autopilot's commanded body MOMENT is converted to a commanded deflection
// by dividing by the control effectiveness (moment per unit deflection); the realized deflection,
// after the lag/limits, is converted back to the realized control moment. This makes the actuator
// dynamics the dominant lag in the terminal loop, as on a real airframe.
class FinActuator {
 public:
  explicit FinActuator(const ActuatorConfig& cfg, double dt) : cfg_(cfg), dt_(dt) {}

  // Advance the actuators one step toward `deflection_cmd` [rad]; returns the realized deflection.
  Vector3 step(const Vector3& deflection_cmd);

  // Map a commanded body moment [N*m] to a commanded deflection [rad] via the effectiveness gain.
  Vector3 allocate(const Vector3& moment_cmd) const;

  // The control moment [N*m] produced by a realized deflection (the allocation inverse).
  Vector3 controlMoment(const Vector3& deflection) const;

  Vector3 deflection() const { return defl_; }

 private:
  ActuatorConfig cfg_;
  double dt_;
  Vector3 defl_;  // current realized deflection per body axis [rad]
};

}  // namespace gncsim
