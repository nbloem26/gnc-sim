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
  double g0 = 9.80665;                // surface gravity [m/s^2]
  bool altitude_dependent_g = false;  // inverse-square falloff if true (flat mode)
  bool atmosphere = true;             // apply USSA76 drag if true
  std::string frame = "flat";         // "flat" (default, flat-Earth ENU) | "round" (WGS-84 ECI)
  bool j2 = false;                    // add J2 oblateness term to round-mode central gravity
};

struct AeroConfig {
  double ref_area = 0.05;                      // aerodynamic reference area [m^2]
  double cd0 = 0.3;                            // fallback drag coeff if table empty
  std::vector<std::array<double, 2>> cd_mach;  // (mach, Cd) breakpoints, ascending mach
  double cn_alpha = 12.0;                      // normal-force coeff slope [1/rad] (6DOF)
};

struct VehicleConfig {
  Vector3 pos0{0.0, 0.0, 0.0};         // initial ENU position [m]
  double launch_speed = 600.0;         // [m/s]
  double launch_elevation_deg = 45.0;  // above horizon
  double launch_azimuth_deg = 0.0;     // from East toward North
  double mass0 = 22.0;                 // [kg]
  double inertia = 1.2;                // scalar moment of inertia proxy [kg*m^2] (6DOF)
};

struct PropulsionConfig {
  double thrust = 0.0;           // boost thrust [N] (0 = unpowered default)
  double burn_time = 0.0;        // [s]
  double propellant_mass = 0.0;  // [kg], burns linearly over burn_time
  double boost_ref_area = 0.0;   // booster drag area during boost [m^2] (0 = use aero.ref_area)
  double stage_time = 0.0;       // booster jettison time [s] (0 = no staging)
  double stage_mass_drop = 0.0;  // mass dropped at staging [kg]
};

struct GuidanceConfig {
  std::string law = "pronav";   // "pronav" | "apn" | "none"
  double nav_constant = 3.0;    // PN gain N
  double max_accel = 300.0;     // accel command limit [m/s^2]
  double time_constant = 0.0;   // guidance/autopilot lag [s]; 0 = ideal instantaneous
  double apn_filter_tau = 0.1;  // APN target-accel estimator low-pass time constant [s]
};

struct ControlConfig {  // 6DOF acceleration autopilot
  double kp = 8.0;
  double kd = 2.5;
  double max_fin_deflection = 0.35;  // [rad]
};

struct ImuNoise {
  double accel_white = 0.0;             // velocity random walk -> per-sample accel std [m/s^2]
  double accel_bias_instability = 0.0;  // [m/s^2]
  double accel_bias_tau = 100.0;        // Gauss-Markov correlation time [s]
  double accel_rrw = 0.0;               // rate random walk driving accel bias
  double accel_scale_factor = 0.0;      // fractional
  double gyro_white = 0.0;              // angle random walk -> per-sample rate std [rad/s]
  double gyro_bias_instability = 0.0;   // [rad/s]
  double gyro_bias_tau = 100.0;         // [s]
  double gyro_rrw = 0.0;
  double gyro_scale_factor = 0.0;
};

struct SeekerNoise {
  double los_white = 0.0;  // LOS angle measurement noise std [rad]
  double los_bias = 0.0;   // boresight bias [rad]
  double glint = 0.0;      // range-dependent glint coefficient
};

struct SensorConfig {
  bool enable = false;  // if false, navigation uses truth (noise-free)
  ImuNoise imu;
  SeekerNoise seeker;
};

struct NavConfig {
  std::string filter = "alpha_beta";  // "alpha_beta" | "ekf"
  double process_accel_psd = 50.0;    // EKF target-accel PSD q [m^2/s^3] per axis
  double range_white = 5.0;           // range measurement noise std [m] (EKF range channel)
};

struct TargetConfig {
  Vector3 pos0{8000.0, 0.0, 3000.0};
  Vector3 vel0{-250.0, 0.0, 0.0};
  std::string maneuver = "constant";  // "constant" | "weave"
  double maneuver_g = 3.0;            // lateral accel for weave [g]
  double maneuver_freq = 0.4;         // [Hz]
  double maneuver_phase_deg = 0.0;    // weave phase offset [deg] (Monte Carlo randomizes this)
};

// One fixed external sensor in the multi-tracker fusion path (issue #5). Type "radar" yields
// [az, el, range, range_rate]; type "ir" is angles-only [az, el]. Position is fixed in ENU [m].
struct TrackerSensorConfig {
  std::string type = "radar";  // "radar" | "ir"
  Vector3 pos;                 // sensor location, ENU [m] (e.g. ground site at origin, space high)
  double sigma_az = 1.0e-3;    // azimuth noise std [rad]   (radar + ir)
  double sigma_el = 1.0e-3;    // elevation noise std [rad] (radar + ir)
  double sigma_range = 10.0;   // range noise std [m]        (radar only)
  double sigma_range_rate = 1.0;  // range-rate noise std [m/s] (radar only)
};

// Multi-sensor target-track fusion (issue #5). Opt-in: when disabled (default) nothing changes and
// the default single-seeker navigation path is byte-identical. When enabled, the Runner builds a
// TargetTrackEkf, synthesizes a noisy measurement from each sensor each step, fuses them
// sequentially into one absolute target-state estimate, and guidance uses (track_est - vehicle).
struct TrackersConfig {
  bool enabled = false;
  double process_psd = 50.0;  // target-accel PSD q [m^2/s^3] per axis (nearly-constant-velocity Q)
  std::vector<TrackerSensorConfig> sensors;
};

// Seeker target-discrimination against decoys / closely-spaced objects (issue #6). Opt-in: when
// disabled (default) nothing changes and the single-true-target homing path is byte-identical. When
// enabled, the Runner places `count` decoys in a small cluster around the true target, gives every
// object a feature SIGNATURE (IR intensity, apparent size, ballistic deceleration), the
// Discriminator scores each object's NOISILY measured features against the expected lethal-target
// signature each step, integrates the scores over time, and guidance homes on the highest-scoring
// (selected) object rather than necessarily the true target. The `separability` knob (0..1)
// controls how distinct the decoy feature distributions are from the true target: 1 = decoys
// obviously different (easy), 0 = decoys statistically indistinguishable (hard, selection
// degrades).
struct DecoysConfig {
  bool enabled = false;
  int count = 0;              // number of decoys around the true target
  double separation = 50.0;   // characteristic cluster spread of decoys about the target [m]
  double separability = 1.0;  // 0..1; how distinct decoy features are from the true target

  // True (lethal) target's characteristic feature signature, as measured by the seeker.
  double target_intensity = 1.0;  // IR brightness (dimensionless, relative)
  double target_size = 1.0;       // apparent size (dimensionless, relative)
  double target_decel = 1.0;      // ballistic deceleration proxy (heavier target sheds speed less)

  // Decoy feature MEANS at separability = 1 (fully distinct). The actual decoy means are blended
  // toward the target signature as separability -> 0 (decoys then look like the target).
  double decoy_intensity = 0.3;  // decoys are dimmer flares / lighter objects
  double decoy_size = 0.4;       // and smaller
  double decoy_decel = 3.0;      // and decelerate faster (lighter -> lower ballistic coeff)

  // Per-object feature SPREAD (std dev of the static per-object signature draw) and the seeker's
  // per-step measurement NOISE std on each feature. Larger -> harder to discriminate.
  double feature_spread = 0.10;     // static object-to-object signature variation
  double measurement_noise = 0.08;  // seeker per-step feature measurement noise std
  double score_filter_tau = 0.5;    // temporal score integration low-pass time constant [s]
};

// Interceptor cueing / launch-on-track (issue #8). Opt-in: when disabled (default) nothing changes
// and the interceptor launches at t=0 exactly as before (byte-identical). When enabled (requires
// trackers.enabled — the fused multi-sensor track that does the cueing), the engagement runs in two
// phases within one run: PHASE 1 the interceptor is held stationary at its launch site while the
// TargetTrackEkf fuses sensor measurements on the incoming threat; once the launch criterion fires
// the cue/launch time is recorded and the interceptor is LAUNCHED with launch_speed aimed via a
// constant-velocity lead/intercept solution from the current track estimate (loft_deg added to the
// elevation). PHASE 2 is normal terminal homing (PN/APN) off the track. The launch criterion is
// either the fused-track position-covariance trace dropping below cov_trace_threshold, or a hard
// timeout at max_cue_time, whichever comes first.
struct CueingConfig {
  bool enabled = false;
  std::string launch_criterion = "track_cov";  // "track_cov" | "fixed_delay"
  double cov_trace_threshold = 1.0e4;          // launch once track position-cov trace < this [m^2]
  double max_cue_time = 10.0;                  // launch by this time regardless [s]
  double loft_deg = 0.0;  // elevation bias added to the lead-solution aim [deg]
};

struct MonteCarloConfig {
  int num_cases = 0;  // 0 => single deterministic run
  double launch_speed_sigma = 0.0;
  double launch_elevation_sigma_deg = 0.0;
  double target_pos_sigma = 0.0;
};

struct SimConfig {
  std::string scenario = "homing";
  std::string model = "3dof";  // "3dof" | "6dof"
  std::uint64_t seed = 1;
  double dt = 0.005;    // integration step [s]
  double t_end = 60.0;  // max sim time [s]
  Integrator integrator = Integrator::RK4;
  GeodeticOrigin origin;

  EnvConfig env;
  AeroConfig aero;
  VehicleConfig vehicle;
  PropulsionConfig propulsion;
  GuidanceConfig guidance;
  ControlConfig control;
  SensorConfig sensors;
  NavConfig nav;
  TargetConfig target;
  TrackersConfig trackers;
  DecoysConfig decoys;
  CueingConfig cueing;
  MonteCarloConfig monte_carlo;
};

// Parse a JSON document (tolerant: missing keys fall back to struct defaults).
// Throws std::runtime_error on malformed JSON.
SimConfig loadConfigFromString(const std::string& json_text);

// Compute initial ENU velocity from launch speed/elevation/azimuth.
Vector3 launchVelocity(const VehicleConfig& v);

}  // namespace gncsim
