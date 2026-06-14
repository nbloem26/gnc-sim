// gnc-sim — multi-sensor target-track Extended Kalman Filter. Estimates the TARGET's ABSOLUTE
// state (position + velocity) in the ENU world frame, fusing measurements from several fixed ground
// / space sensors (radar, IR) into one track file. This is distinct from the interceptor-relative
// seeker EKF (Ekf.hpp): there the state is (target - vehicle) from the seeker; here the state is
// the target's own absolute position/velocity, observed from external sensors at known fixed
// locations.
//
// State x (6): [pos(3), vel(3)] of the target, ENU, SI units.
// Predict: nearly-constant-velocity, F = [[I, dt*I],[0, I]], continuous-white-noise-acceleration Q
//          with a configurable per-axis process-noise PSD. No control input (the sensors are
//          passive and the target's maneuvers are unknown — absorbed by Q).
// Update : SEQUENTIAL, one sensor at a time, each with its own nonlinear h(x), Jacobian H, and
// noise
//          R. Joseph-form covariance update preserves symmetry / PSD; NIS reported per update.
//
// Sensor models (target measured FROM a fixed sensor at p_s; rel = target_pos - p_s):
//   radar: [az, el, range, range_rate], range_rate = (rel · target_vel)/|rel|  (sensor static)
//   ir   : [az, el]                      (angles-only — no range observability)
//
// Fixed-size hand-rolled linear algebra (std::array) — no Eigen / external dependency, so the
// filter is bit-for-bit identical between native (libstdc++) and WASM (libc++).
#pragma once

#include <array>
#include <vector>

#include "gncsim/math/Vector3.hpp"

namespace gncsim {

// Sensor kind for the measurement model dispatch.
enum class TrackSensorType { Radar, Ir };

// One fixed sensor: its kind, ENU position, and characterized measurement noise.
struct TrackSensor {
  TrackSensorType type = TrackSensorType::Radar;
  Vector3 pos;                    // sensor location, ENU [m]
  double sigma_az = 1.0e-3;       // azimuth noise std [rad]   (radar + ir)
  double sigma_el = 1.0e-3;       // elevation noise std [rad] (radar + ir)
  double sigma_range = 10.0;      // range noise std [m]       (radar only)
  double sigma_range_rate = 1.0;  // range-rate noise std [m/s] (radar only)

  // Number of scalar measurements this sensor produces (radar=4, ir=2).
  int dim() const { return type == TrackSensorType::Radar ? 4 : 2; }
};

class TargetTrackEkf {
 public:
  // dt          : fixed integration step [s]
  // process_psd : target-acceleration PSD q [m^2/s^3] per axis (nearly-constant-velocity Q)
  TargetTrackEkf(double dt, double process_psd);

  // Bootstrap the absolute target state directly (e.g. from an initial truth/cue), with a large
  // diagonal covariance. Until bootstrapped, predict() is a no-op and update() bootstraps position
  // from the first radar measurement if available.
  void bootstrap(const Vector3& pos, const Vector3& vel);

  // Time update over dt (nearly-constant-velocity). No-op until bootstrapped.
  void predict();

  // Sequential measurement update from one sensor. `z` holds the sensor's measurement vector
  // (radar: az,el,range,range_rate; ir: az,el). Returns the NIS of this update (0 if skipped).
  // If not yet bootstrapped and the sensor is a radar, the first call bootstraps position from the
  // az/el/range and zero velocity.
  double update(const TrackSensor& sensor, const std::vector<double>& z);

  Vector3 pos() const { return {x_[0], x_[1], x_[2]}; }
  Vector3 vel() const { return {x_[3], x_[4], x_[5]}; }
  double nis() const { return nis_; }  // NIS of the last update (dof = sensor.dim())
  bool initialized() const { return initialized_; }

  // Covariance accessors (for tests / diagnostics).
  double covTrace() const;  // trace of the full 6x6 P
  double cov(int i, int j) const { return p_[i * 6 + j]; }

  // Predicted measurement h(x) and Jacobian H (row-major, dim x 6) for a sensor at the current
  // state. Exposed for tests; also used internally by update().
  void measurementModel(const TrackSensor& sensor, std::array<double, 4>& h,
                        std::array<double, 24>& H, int& dim) const;

 private:
  double dt_;
  double q_;  // process-accel PSD

  bool initialized_ = false;
  std::array<double, 6> x_{};   // state estimate [pos, vel]
  std::array<double, 36> p_{};  // 6x6 covariance (row-major)
  double nis_ = 0.0;
};

}  // namespace gncsim
