// gnc-sim — relative-state Extended Kalman Filter for navigation. Estimates the target-relative
// position and velocity (target - vehicle) in the ENU world frame from nonlinear seeker
// measurements (azimuth, elevation, range). Selectable alternative to the alpha-beta Navigator.
//
// State x (6): [rel_pos(3), rel_vel(3)].
// Predict (nearly-constant-velocity) with the interceptor's own achieved acceleration as a known
// control input — gravity is common-mode between target and vehicle in the relative frame so it
// cancels; u = -a_vehicle. Update with a nonlinear az/el/range measurement (Joseph-form
// covariance).
//
// Fixed-size hand-rolled linear algebra (std::array) — no Eigen / external dependency, so the EKF
// stays bit-for-bit identical between native (libstdc++) and WASM (libc++).
#pragma once

#include <array>

#include "gncsim/math/Vector3.hpp"

namespace gncsim {

class Ekf {
 public:
  // dt                : fixed integration step [s]
  // process_accel_psd : target-acceleration PSD q [m^2/s^3] per axis (nearly-constant-velocity Q)
  // sigma_az/sigma_el : angular measurement noise std [rad]
  // sigma_range       : range measurement noise std [m]
  Ekf(double dt, double process_accel_psd, double sigma_az, double sigma_el, double sigma_range);

  // Time update over dt using the vehicle's achieved acceleration (control input u = -a_vehicle).
  // No-op until the filter has been bootstrapped by the first update().
  void predict(const Vector3& a_vehicle);

  // Measurement update from a seeker observation of the relative position.
  //   az    = atan2(rel_y, rel_x)            [rad]
  //   el    = atan2(rel_z, hypot(rel_x,y))   [rad]
  //   range = |rel_pos|                       [m]
  // The first call bootstraps the state from the measurement (pos from az/el/range, vel = 0,
  // large initial covariance).
  void update(double az, double el, double range);

  Vector3 relPos() const { return {x_[0], x_[1], x_[2]}; }
  Vector3 relVel() const { return {x_[3], x_[4], x_[5]}; }
  double nis() const { return nis_; }  // last normalized innovation squared (dof = 3)
  bool initialized() const { return initialized_; }

 private:
  double dt_;
  double q_;                      // process-accel PSD
  std::array<double, 3> r_diag_;  // measurement noise variances [az^2, el^2, range^2]

  bool initialized_ = false;
  std::array<double, 6> x_{};   // state estimate
  std::array<double, 36> p_{};  // 6x6 covariance (row-major)
  double nis_ = 0.0;
};

}  // namespace gncsim
