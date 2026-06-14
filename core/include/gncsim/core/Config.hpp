// gnc-sim — simulation configuration schema. Loaded from JSON (configs/*.json) by both the
// native CLI and the WASM entry. The parser lives in Config.cpp so modules that only read
// SimConfig fields don't pull in nlohmann/json. See docs/DATA_CONTRACT.md for the JSON shape.
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "gncsim/core/Types.hpp"
#include "gncsim/math/Vector3.hpp"

namespace gncsim {

enum class Integrator { Euler, RK2, RK4 };

struct EnvConfig {
  double g0 = 9.80665;                 // surface gravity [m/s^2]
  bool altitude_dependent_g = false;   // inverse-square falloff if true
  bool atmosphere = true;              // apply USSA76 drag if true
};

struct AeroConfig {
  double ref_area = 0.05;              // aerodynamic reference area [m^2]
  double cd0 = 0.3;                    // fallback drag coeff if table empty
  std::vector<std::array<double, 2>> cd_mach;  // (mach, Cd) breakpoints, ascending mach
  double cn_alpha = 12.0;              // normal-force coeff slope [1/rad] (6DOF)
};

struct VehicleConfig {
  Vector3 pos0{0.0, 0.0, 0.0};         // initial ENU position [m]
  double launch_speed = 600.0;         // [m/s]
  double launch_elevation_deg = 45.0;  // above horizon
  double launch_azimuth_deg = 0.0;     // from East toward North
  double mass0 = 22.0;                 // [kg]
  double inertia = 1.2;                // scalar moment of inertia proxy [kg*m^2] (6DOF)
};

struct GuidanceConfig {
  std::string law = "pronav";          // "pronav" | "none"
  double nav_constant = 3.0;           // PN gain N
  double max_accel = 300.0;            // accel command limit [m/s^2]
  double time_constant = 0.0;          // guidance/autopilot lag [s]; 0 = ideal instantaneous
};

struct ControlConfig {                 // 6DOF acceleration autopilot
  double kp = 8.0;
  double kd = 2.5;
  double max_fin_deflection = 0.35;    // [rad]
};

struct ImuNoise {
  double accel_white = 0.0;            // velocity random walk -> per-sample accel std [m/s^2]
  double accel_bias_instability = 0.0; // [m/s^2]
  double accel_bias_tau = 100.0;       // Gauss-Markov correlation time [s]
  double accel_rrw = 0.0;              // rate random walk driving accel bias
  double accel_scale_factor = 0.0;     // fractional
  double gyro_white = 0.0;             // angle random walk -> per-sample rate std [rad/s]
  double gyro_bias_instability = 0.0;  // [rad/s]
  double gyro_bias_tau = 100.0;        // [s]
  double gyro_rrw = 0.0;
  double gyro_scale_factor = 0.0;
};

struct SeekerNoise {
  double los_white = 0.0;              // LOS angle measurement noise std [rad]
  double los_bias = 0.0;               // boresight bias [rad]
  double glint = 0.0;                  // range-dependent glint coefficient
};

struct SensorConfig {
  bool enable = false;                 // if false, navigation uses truth (noise-free)
  ImuNoise imu;
  SeekerNoise seeker;
};

struct NavConfig {
  std::string filter = "alpha_beta";   // "alpha_beta" | "ekf"
  double process_accel_psd = 50.0;     // EKF target-accel PSD q [m^2/s^3] per axis
  double range_white = 5.0;            // range measurement noise std [m] (EKF range channel)
};

struct TargetConfig {
  Vector3 pos0{8000.0, 0.0, 3000.0};
  Vector3 vel0{-250.0, 0.0, 0.0};
  std::string maneuver = "constant";   // "constant" | "weave"
  double maneuver_g = 3.0;             // lateral accel for weave [g]
  double maneuver_freq = 0.4;          // [Hz]
  double maneuver_phase_deg = 0.0;     // weave phase offset [deg] (Monte Carlo randomizes this)
};

struct MonteCarloConfig {
  int num_cases = 0;                   // 0 => single deterministic run
  double launch_speed_sigma = 0.0;
  double launch_elevation_sigma_deg = 0.0;
  double target_pos_sigma = 0.0;
};

struct SimConfig {
  std::string scenario = "homing";
  std::string model = "3dof";          // "3dof" | "6dof"
  std::uint64_t seed = 1;
  double dt = 0.005;                   // integration step [s]
  double t_end = 60.0;                 // max sim time [s]
  Integrator integrator = Integrator::RK4;
  GeodeticOrigin origin;

  EnvConfig env;
  AeroConfig aero;
  VehicleConfig vehicle;
  GuidanceConfig guidance;
  ControlConfig control;
  SensorConfig sensors;
  NavConfig nav;
  TargetConfig target;
  MonteCarloConfig monte_carlo;
};

// Parse a JSON document (tolerant: missing keys fall back to struct defaults).
// Throws std::runtime_error on malformed JSON.
SimConfig loadConfigFromString(const std::string& json_text);

// Compute initial ENU velocity from launch speed/elevation/azimuth.
Vector3 launchVelocity(const VehicleConfig& v);

}  // namespace gncsim
