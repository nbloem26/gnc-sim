// gnc-sim — shared simulation types. The `Frame` struct IS the telemetry data contract in code;
// see docs/DATA_CONTRACT.md. Field names here must match the CSV columns and JSON keys exactly.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "gncsim/math/Quaternion.hpp"
#include "gncsim/math/Vector3.hpp"

namespace gncsim {

// Origin of the local flat-earth ENU frame, used by post-processing to project to geodetic.
struct GeodeticOrigin {
  double lat0_deg = 28.4889;  // default: Cape Canaveral-ish
  double lon0_deg = -80.5778;
  double alt0_m = 0.0;
};

// Truth state of a rigid body (vehicle or target) in the ENU world frame.
struct EntityState {
  double t = 0.0;
  Vector3 pos;        // ENU position [m]
  Vector3 vel;        // ENU velocity [m/s]
  Quaternion att;     // body->world attitude (6DOF; identity for 3DOF)
  Vector3 angVel;     // body angular rate [rad/s] (6DOF)
  double mass = 1.0;  // [kg]
  double mach = 0.0;  // current Mach number
};

// One telemetry row. Populated by the Runner each step from module outputs.
// CSV/JSON serializers iterate these; keep field names == contract names.
struct Frame {
  double t = 0.0;

  // Vehicle truth
  Vector3 veh_pos, veh_vel;
  Quaternion veh_att;
  double mass = 0.0;
  double mach = 0.0;
  double thrust = 0.0;  // current boost thrust magnitude [N]; 0 on the unpowered default path

  // Target truth
  Vector3 tgt_pos, tgt_vel;

  // GNC
  Vector3 accel_cmd;       // commanded acceleration [m/s^2] (guidance)
  Vector3 fin_deflection;  // control surface / body-rate command (6DOF) [rad]
  Vector3 nav_pos_est;     // navigation position estimate [m]
  Vector3 nav_vel_est;     // navigation velocity estimate [m/s]
  double los_angle = 0.0;  // line-of-sight angle [rad]
  double los_rate = 0.0;   // line-of-sight rate [rad/s]
  double v_closing = 0.0;  // closing velocity [m/s]
  double range = 0.0;      // vehicle-to-target range [m]
  double nav_nis = 0.0;    // EKF normalized innovation squared (dof=3); 0 on the alpha-beta path

  // Multi-sensor target track (issue #5). Zero on every non-tracker path (default). When the
  // trackers fusion path is enabled, track_pos_est is the fused absolute target-position estimate
  // [m] (ENU) and track_nis is the last sensor update's NIS.
  Vector3 track_pos_est;   // fused absolute target-position estimate [m]; (0,0,0) when disabled
  double track_nis = 0.0;  // last fused sensor-update NIS; 0 when disabled

  // Sensors (true vs measured)
  Vector3 imu_accel_true, imu_accel_meas;               // specific force [m/s^2]
  Vector3 imu_gyro_true, imu_gyro_meas;                 // angular rate [rad/s]
  double seeker_los_true = 0.0, seeker_los_meas = 0.0;  // [rad]
};

// Complete result of one simulation run: metadata + time series. Pure in-memory; no file I/O.
struct SimResult {
  std::string scenario;
  std::string model;  // "3dof" | "6dof"
  std::uint64_t seed = 0;
  double dt = 0.0;
  double t_end = 0.0;
  GeodeticOrigin origin;

  bool intercept = false;
  double miss_distance = 0.0;   // closest approach [m]
  double intercept_time = 0.0;  // time of closest approach [s]
  std::string git_sha;          // filled by entry points if available

  std::vector<Frame> frames;
};

}  // namespace gncsim
