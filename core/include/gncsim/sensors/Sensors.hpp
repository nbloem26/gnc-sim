// gnc-sim — sensor error models. Reproduce, in-sim, the error characteristics recovered from the
// Allan-variance analysis (configs/sensor_params.json): white noise + Gauss-Markov bias
// instability + rate random walk + scale factor. Phase 1 (sensors) owns core/src/sensors/.
#pragma once

#include "gncsim/core/Config.hpp"
#include "gncsim/core/Rng.hpp"
#include "gncsim/math/Vector3.hpp"

namespace gncsim {

// Strapdown IMU: corrupts true specific force and angular rate. Holds the slowly-varying bias
// state between calls (first-order Gauss-Markov + random walk), so construct once per run.
class Imu {
 public:
  Imu(const ImuNoise& cfg, double dt, Rng& rng) : cfg_(cfg), dt_(dt), rng_(rng) {}

  void measure(const Vector3& accel_true, const Vector3& gyro_true,
               Vector3& accel_meas, Vector3& gyro_meas);

 private:
  ImuNoise cfg_;
  double dt_;
  Rng& rng_;
  Vector3 accel_bias_;  // running accelerometer bias
  Vector3 gyro_bias_;   // running gyro bias
};

// Seeker: line-of-sight angle measurement with boresight bias, white noise, and range-dependent
// glint (noise grows as the target is approached).
class Seeker {
 public:
  Seeker(const SeekerNoise& cfg, Rng& rng) : cfg_(cfg), rng_(rng) {}

  double measureLos(double los_true, double range);

 private:
  SeekerNoise cfg_;
  Rng& rng_;
};

}  // namespace gncsim
