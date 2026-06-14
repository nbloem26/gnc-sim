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
  Vector3 rel_pos;        // target - vehicle [m]
  Vector3 rel_vel;        // target_vel - vehicle_vel [m/s]
  Vector3 los_unit;       // unit line-of-sight (vehicle->target)
  Vector3 los_rate_vec;   // LOS rotation rate vector [rad/s]
  double range = 0.0;     // |rel_pos| [m]
  double v_closing = 0.0; // closing speed [m/s] (positive = closing)
  double los_angle = 0.0; // LOS elevation angle in the vertical plane [rad]
  double los_rate = 0.0;  // |los_rate_vec| [rad/s]
};

Engagement computeEngagement(const EntityState& vehicle, const EntityState& target);

// Proportional Navigation: a_cmd = N * Vc * (LOS_unit x los_rate_vec), magnitude-limited.
Vector3 proNavCommand(const Engagement& e, const GuidanceConfig& cfg);

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

}  // namespace gncsim
