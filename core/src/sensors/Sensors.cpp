// gnc-sim — sensor error models (implementation).
//
// This file reproduces, in-sim, the error characteristics recovered by the Python
// Allan-variance pipeline (sensors/fit_noise_params.py -> configs/sensor_params.json).
// The math here IS the contract: the Allan-variance loop-closure test re-derives these
// same parameters from the *_meas_* columns, so the per-sample formulas below must match
// what that pipeline assumes. See docs/DATA_CONTRACT.md §5.
//
// ============================================================================
// NOISE CONVENTION (units + per-sample formulas)  -- the Python agent must match this.
// ============================================================================
// Sampling: Imu::measure() is called exactly once per integration step at the fixed rate
// dt_ [s] passed to the Imu constructor. All discretizations below assume that fixed dt_.
//
// For each axis of accelerometer and gyro (cfg fields accel_* / gyro_*):
//
//   1. White noise (velocity/angle random walk density):
//        cfg.white is a CONTINUOUS-TIME density:
//          accel: (m/s^2)/sqrt(Hz)  i.e. VRW expressed as (m/s)/sqrt(s) per axis
//          gyro:  (rad/s)/sqrt(Hz)  i.e. ARW expressed as rad/sqrt(s)   per axis
//        Discrete per-sample std = white / sqrt(dt_).
//        Add gaussian(0, white/sqrt(dt_)) independently per axis, every sample.
//
//   2. Bias instability as a first-order Gauss-Markov (GM) process with correlation
//      time tau [s] and steady-state std = bias_instability (same units as the signal):
//        bias <- bias * exp(-dt_/tau)
//                + gaussian(0, bias_instability * sqrt(1 - exp(-2*dt_/tau)))   [per axis]
//        The driving-noise std sqrt(1 - exp(-2 dt/tau)) makes the stationary variance of
//        the GM process equal bias_instability^2 regardless of dt_/tau.
//        If tau <= 0 the GM contribution is treated as zero (constant 0), i.e. disabled.
//      State (accel_bias_, gyro_bias_) is held across calls — that is why the Imu owns it.
//
//   3. Rate random walk (RRW): an unbounded random walk added on top of the GM bias:
//        bias += gaussian(0, rrw * sqrt(dt_))   [per axis]
//        cfg.rrw units: accel (m/s^2)/sqrt(s), gyro (rad/s)/sqrt(s) per axis.
//
//   4. Scale-factor error (fractional, dimensionless): the true signal is scaled by
//        (1 + scale_factor).
//
//   Measured = true * (1 + scale_factor) + bias + white       (per axis)
//   where `bias` is the post-update GM+RRW state and `white` is the fresh per-sample draw.
//
// Seeker (LOS angle, scalar):
//   measured = los_true + los_bias
//                       + gaussian(0, los_white)              [white, rad]
//                       + gaussian(0, glint / max(range, 1.0))[range-dependent glint]
//   The glint std grows as range -> 0 (clamped at range = 1.0 m to stay finite). If
//   glint == 0 it contributes nothing.
// ============================================================================

#include "gncsim/sensors/Sensors.hpp"

#include <cmath>

namespace gncsim {

namespace {

// Advance one axis of a Gauss-Markov + rate-random-walk bias state by one step of dt,
// then return the additive per-axis noise to apply this sample (scaled true + bias + white
// is assembled by the caller). `bias` is updated in place (held across calls).
//
//   white_std  = white / sqrt(dt)                       (per-sample white std)
//   GM update  = bias*exp(-dt/tau) + N(0, q_gm)         q_gm = bias_inst*sqrt(1-exp(-2dt/tau))
//   RRW update = bias += N(0, rrw*sqrt(dt))
//
// Returns the white-noise draw for this sample; `bias` carries the (correlated) low-freq term.
inline double stepAxisBias(double& bias, double white, double bias_instability, double tau,
                           double rrw, double dt, Rng& rng) {
  // --- Gauss-Markov bias instability ---
  if (tau > 0.0 && bias_instability > 0.0) {
    const double phi = std::exp(-dt / tau);                             // GM decay factor
    const double q_gm = bias_instability * std::sqrt(1.0 - phi * phi);  // driving-noise std
    bias = bias * phi + rng.gaussian(0.0, q_gm);
  } else {
    // tau <= 0 disables the GM contribution. We intentionally do NOT decay an existing
    // GM bias here; with bias_instability/tau disabled there is simply no GM driving term.
    // (RRW below may still walk the bias if rrw > 0.)
  }

  // --- Rate random walk (unbounded walk on top of the GM bias) ---
  if (rrw > 0.0) {
    bias += rng.gaussian(0.0, rrw * std::sqrt(dt));
  }

  // --- White noise (returned to caller; not part of the held state) ---
  if (white > 0.0) {
    return rng.gaussian(0.0, white / std::sqrt(dt));
  }
  return 0.0;
}

}  // namespace

void Imu::measure(const Vector3& accel_true, const Vector3& gyro_true, Vector3& accel_meas,
                  Vector3& gyro_meas) {
  // --- Accelerometer (per axis) ---
  const Vector3 accel_white_mps2{
      stepAxisBias(accel_bias_.x, cfg_.accel_white, cfg_.accel_bias_instability,
                   cfg_.accel_bias_tau, cfg_.accel_rrw, dt_, rng_),
      stepAxisBias(accel_bias_.y, cfg_.accel_white, cfg_.accel_bias_instability,
                   cfg_.accel_bias_tau, cfg_.accel_rrw, dt_, rng_),
      stepAxisBias(accel_bias_.z, cfg_.accel_white, cfg_.accel_bias_instability,
                   cfg_.accel_bias_tau, cfg_.accel_rrw, dt_, rng_)};
  accel_meas = accel_true * (1.0 + cfg_.accel_scale_factor) + accel_bias_ + accel_white_mps2;

  // --- Gyro (per axis) ---
  const Vector3 gyro_white_radps{
      stepAxisBias(gyro_bias_.x, cfg_.gyro_white, cfg_.gyro_bias_instability, cfg_.gyro_bias_tau,
                   cfg_.gyro_rrw, dt_, rng_),
      stepAxisBias(gyro_bias_.y, cfg_.gyro_white, cfg_.gyro_bias_instability, cfg_.gyro_bias_tau,
                   cfg_.gyro_rrw, dt_, rng_),
      stepAxisBias(gyro_bias_.z, cfg_.gyro_white, cfg_.gyro_bias_instability, cfg_.gyro_bias_tau,
                   cfg_.gyro_rrw, dt_, rng_)};
  gyro_meas = gyro_true * (1.0 + cfg_.gyro_scale_factor) + gyro_bias_ + gyro_white_radps;
}

double Seeker::measureLos(double los_true, double range) {
  double meas_rad = los_true + cfg_.los_bias;

  if (cfg_.los_white > 0.0) {
    meas_rad += rng_.gaussian(0.0, cfg_.los_white);
  }

  // Range-dependent glint: std grows as range -> 0, clamped at 1.0 m to stay finite.
  if (cfg_.glint > 0.0) {
    const double r_m = range > 1.0 ? range : 1.0;
    meas_rad += rng_.gaussian(0.0, cfg_.glint / r_m);
  }

  return meas_rad;
}

}  // namespace gncsim
