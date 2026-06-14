// gnc-sim — runSimulation(): the full GNC loop tying every module together.
//
// Per step (fixed dt):
//   environment  -> gravity + USSA76 atmosphere
//   aerodynamics -> Mach-dependent drag (+ 6DOF normal force from attitude)
//   sensors      -> seeker LOS + IMU corrupted with the characterized noise (when enabled)
//   navigation   -> alpha-beta tracker turns the (noisy) measurement into a relative-state estimate
//   guidance     -> Proportional Navigation acceleration command off the estimate
//   control      -> 3DOF applies the command directly; 6DOF autopilot -> body moment -> attitude
//   dynamics     -> RK4 step of translational (and 6DOF rotational) state
// Pure: no file I/O, so native and WASM produce identical results.
#include "gncsim/scenario/Runner.hpp"

#include <algorithm>
#include <cmath>

#include "gncsim/aero/Aero.hpp"
#include "gncsim/core/Rng.hpp"
#include "gncsim/dynamics/Dynamics.hpp"
#include "gncsim/env/Environment.hpp"
#include "gncsim/gnc/Ekf.hpp"
#include "gncsim/gnc/Gnc.hpp"
#include "gncsim/sensors/Sensors.hpp"

namespace gncsim {

namespace {

constexpr double kLethalRadius = 3.0;  // intercept if closest approach is within this [m]

// Smallest rotation taking unit vector `from` onto unit vector `to`.
Quaternion quatFromTo(const Vector3& from, const Vector3& to) {
  const Vector3 a = from.normalized();
  const Vector3 b = to.normalized();
  const double d = a.dot(b);
  if (d > 0.999999) return Quaternion{};  // already aligned
  if (d < -0.999999) {                    // opposite: 180° about any ⟂ axis
    Vector3 axis = Vector3{1, 0, 0}.cross(a);
    if (axis.norm() < 1e-6) axis = Vector3{0, 1, 0}.cross(a);
    return Quaternion::fromAxisAngle(axis, M_PI);
  }
  return Quaternion::fromAxisAngle(a.cross(b), std::acos(d));
}

// Target acceleration for the configured maneuver (world frame).
Vector3 targetAccel(const TargetConfig& cfg, const EntityState& tgt, double t) {
  if (cfg.maneuver != "weave") return Vector3{};
  // Sinusoidal lateral accel, horizontal and perpendicular to the target's ground track.
  Vector3 vh = tgt.vel;
  vh.z = 0.0;
  const double s = vh.norm();
  if (s < 1e-6) return Vector3{};
  const Vector3 perp{-vh.y / s, vh.x / s, 0.0};
  const double phase = cfg.maneuver_phase_deg * M_PI / 180.0;
  return perp * (cfg.maneuver_g * 9.80665 * std::sin(2.0 * M_PI * cfg.maneuver_freq * t + phase));
}

}  // namespace

SimResult runSimulation(const SimConfig& cfg) {
  SimResult r;
  r.scenario = cfg.scenario;
  r.model = cfg.model;
  r.seed = cfg.seed;
  r.dt = cfg.dt;
  r.t_end = cfg.t_end;
  r.origin = cfg.origin;

  const bool is6dof = (cfg.model == "6dof");
  Rng rng(cfg.seed);

  // --- Vehicle initial state ---
  EntityState veh;
  veh.pos = cfg.vehicle.pos0;
  veh.vel = launchVelocity(cfg.vehicle);
  veh.mass = cfg.vehicle.mass0;
  if (is6dof && veh.vel.norm() > 1e-6) veh.att = quatFromTo({1, 0, 0}, veh.vel);

  // --- Target initial state ---
  EntityState tgt;
  tgt.pos = cfg.target.pos0;
  tgt.vel = cfg.target.vel0;

  // --- Modules ---
  GravityModel gravity(cfg.env);
  AeroModel aero(cfg.aero);
  // Booster aero: same model on a copy of the aero config with the larger boost reference area,
  // selected only inside the boost window. Built unconditionally; only used when boost_ref_area>0.
  AeroConfig boost_aero_cfg = cfg.aero;
  boost_aero_cfg.ref_area = cfg.propulsion.boost_ref_area;
  AeroModel boost_aero(boost_aero_cfg);
  Imu imu(cfg.sensors.imu, cfg.dt, rng);
  Seeker seeker(cfg.sensors.seeker, rng);
  Navigator nav(cfg.dt);

  // EKF (relative-state) navigation filter: opt-in via cfg.nav.filter == "ekf"; default stays
  // alpha-beta. az/el measurement sigma = seeker.los_white [rad]; range sigma = nav.range_white
  // [m].
  const bool use_ekf = (cfg.nav.filter == "ekf");
  Ekf ekf(cfg.dt, cfg.nav.process_accel_psd, cfg.sensors.seeker.los_white,
          cfg.sensors.seeker.los_white, cfg.nav.range_white);

  Autopilot autopilot(cfg.control);

  double best_range = (tgt.pos - veh.pos).norm();
  double best_t = 0.0;
  Vector3
      accel_achieved;  // realized guidance accel, lagged toward the command (finite autopilot τ)

  // --- Boost phase bookkeeping (default off: thrust==0 => everything below is a no-op) ---
  const auto& prop = cfg.propulsion;
  // Linear propellant burn rate [kg/s].
  const double m_dot = (prop.burn_time > 0.0 && prop.propellant_mass > 0.0)
                           ? prop.propellant_mass / prop.burn_time
                           : 0.0;
  // Dry-mass floor so mass never drops to/through zero from burning propellant.
  const double dry_mass_floor = std::max(cfg.vehicle.mass0 - prop.propellant_mass, 1e-6);
  // Booster drag applies for t < stage_time, or t < burn_time when no staging is configured.
  const double boost_drag_end = (prop.stage_time > 0.0) ? prop.stage_time : prop.burn_time;
  bool staged = false;  // ensures the staging mass drop happens exactly once

  const int steps = static_cast<int>(cfg.t_end / cfg.dt);
  for (int i = 0; i <= steps; ++i) {
    const double t = i * cfg.dt;

    // --- Staging: drop the spent booster mass exactly once at t >= stage_time ---
    if (!staged && prop.stage_time > 0.0 && t >= prop.stage_time) {
      veh.mass = std::max(veh.mass - prop.stage_mass_drop, 1e-6);
      staged = true;
    }

    // --- Environment / aero state ---
    const AtmSample atm =
        cfg.env.atmosphere ? atmosphereUSSA76(veh.pos.z) : AtmSample{0.0, 0.0, 288.15, 340.29};
    const double speed = veh.vel.norm();
    const double mach = atm.speed_of_sound > 0.0 ? speed / atm.speed_of_sound : 0.0;
    veh.mach = mach;

    // --- Truth engagement geometry ---
    const Engagement truth = computeEngagement(veh, tgt);

    // --- Navigation: estimate the relative state (noisy seeker when sensors enabled) ---
    Engagement est = truth;
    double seeker_los_meas = truth.los_angle;
    double nav_nis = 0.0;

    if (use_ekf) {
      // EKF path: az/el/range measurement of the relative position (truth when sensors disabled,
      // Gaussian-corrupted via the run's Rng when enabled). Native and WASM run identical code.
      const double horiz =
          std::sqrt(truth.rel_pos.x * truth.rel_pos.x + truth.rel_pos.y * truth.rel_pos.y);
      double az = std::atan2(truth.rel_pos.y, truth.rel_pos.x);
      double el = std::atan2(truth.rel_pos.z, horiz);
      double rng_meas = truth.range;
      if (cfg.sensors.enable) {
        az += rng.gaussian(0.0, cfg.sensors.seeker.los_white);
        el += rng.gaussian(0.0, cfg.sensors.seeker.los_white);
        rng_meas += rng.gaussian(0.0, cfg.nav.range_white);
      }
      seeker_los_meas = el;  // report the (measured) LOS elevation for telemetry parity

      ekf.predict(accel_achieved);  // achieved guidance accel from the previous step (0 on step 0)
      ekf.update(az, el, rng_meas);
      nav_nis = ekf.nis();

      EntityState veh_est = veh, tgt_est = tgt;
      tgt_est.pos = veh.pos + ekf.relPos();
      tgt_est.vel = veh.vel + ekf.relVel();
      est = computeEngagement(veh_est, tgt_est);
    } else {
      // Alpha-beta path (default). Draw order is unchanged from the original implementation.
      Vector3 measured_rel_pos = truth.rel_pos;
      if (cfg.sensors.enable) {
        // Seeker angular noise perturbs the LOS direction (cross-range error grows with range).
        seeker_los_meas = seeker.measureLos(truth.los_angle, truth.range);
        const double ang_err = seeker_los_meas - truth.los_angle;
        // Apply the angular error about a horizontal axis perpendicular to the LOS.
        Vector3 axis = truth.los_unit.cross(Vector3{0, 0, 1});
        if (axis.norm() < 1e-6) axis = Vector3{0, 1, 0};
        measured_rel_pos = Quaternion::fromAxisAngle(axis, ang_err).rotate(truth.rel_pos);
      }
      nav.update(measured_rel_pos);

      // Relative state used by guidance (filtered estimate, or truth if noise-free).
      if (cfg.sensors.enable) {
        EntityState veh_est = veh, tgt_est = tgt;
        tgt_est.pos = veh.pos + nav.relPos();
        tgt_est.vel = veh.vel + nav.relVel();
        est = computeEngagement(veh_est, tgt_est);
      }
    }

    // --- Guidance: Proportional Navigation ---
    const Vector3 accel_cmd = proNavCommand(est, cfg.guidance);

    // First-order autopilot lag: the interceptor cannot realize the command instantly. Against a
    // maneuvering target this finite time constant is the dominant miss-distance driver.
    if (cfg.guidance.time_constant > 0.0) {
      const double blend = cfg.dt / cfg.guidance.time_constant;
      accel_achieved += (accel_cmd - accel_achieved) * (blend < 1.0 ? blend : 1.0);
    } else {
      accel_achieved = accel_cmd;
    }

    // --- Boost: thrust along the velocity/nose, propellant burn (default off when thrust==0) ---
    const bool boosting = prop.thrust > 0.0 && t < prop.burn_time;
    const double thrust_mag = boosting ? prop.thrust : 0.0;
    // Booster drag uses the larger reference area while inside the boost window.
    const bool use_boost_drag = prop.boost_ref_area > 0.0 && t < boost_drag_end;
    const AeroModel& active_aero = use_boost_drag ? boost_aero : aero;

    // --- Forces / moments ---
    Vector3 force_world;
    Vector3 moment_body;
    Vector3 fin_deflection;
    Vector3 specific_force;  // non-gravitational accel, for the IMU
    if (is6dof) {
      force_world = active_aero.force6dof(veh.vel, veh.att, atm);
      // Thrust acts along the body nose (body x-axis rotated into the world frame).
      const Vector3 thrust_force = veh.att.rotate(Vector3{1.0, 0.0, 0.0}) * thrust_mag;
      force_world += thrust_force;
      moment_body = autopilot.moment(veh, accel_achieved, fin_deflection);
      specific_force = force_world / veh.mass;
    } else {
      // 3DOF point-mass: autopilot realizes the (lagged) PN command as lateral accel.
      const Vector3 drag = active_aero.dragForce(veh.vel, atm);
      // Thrust acts along the velocity direction; if (near-)stationary, along +x to get moving.
      const double speed_for_thrust = veh.vel.norm();
      const Vector3 thrust_dir =
          (speed_for_thrust > 1e-9) ? veh.vel / speed_for_thrust : Vector3{1.0, 0.0, 0.0};
      const Vector3 thrust_force = thrust_dir * thrust_mag;
      force_world = drag + thrust_force + accel_achieved * veh.mass;
      specific_force = (drag + thrust_force) / veh.mass + accel_achieved;
    }

    // --- Sensors (telemetry; gyro truth = body rate) ---
    Vector3 imu_accel_meas = specific_force, imu_gyro_meas = veh.angVel;
    if (cfg.sensors.enable) {
      imu.measure(specific_force, veh.angVel, imu_accel_meas, imu_gyro_meas);
    }

    // --- Record telemetry frame ---
    Frame f;
    f.t = t;
    f.veh_pos = veh.pos;
    f.veh_vel = veh.vel;
    f.veh_att = veh.att;
    f.mass = veh.mass;
    f.mach = mach;
    f.thrust = thrust_mag;
    f.tgt_pos = tgt.pos;
    f.tgt_vel = tgt.vel;
    f.accel_cmd = accel_cmd;
    f.fin_deflection = fin_deflection;
    if (use_ekf) {
      f.nav_pos_est = veh.pos + ekf.relPos();
      f.nav_vel_est = veh.vel + ekf.relVel();
    } else {
      f.nav_pos_est = veh.pos + (cfg.sensors.enable ? nav.relPos() : truth.rel_pos);
      f.nav_vel_est = veh.vel + (cfg.sensors.enable ? nav.relVel() : truth.rel_vel);
    }
    f.los_angle = truth.los_angle;
    f.los_rate = est.los_rate;
    f.v_closing = truth.v_closing;
    f.range = truth.range;
    f.nav_nis = nav_nis;
    f.imu_accel_true = specific_force;
    f.imu_accel_meas = imu_accel_meas;
    f.imu_gyro_true = veh.angVel;
    f.imu_gyro_meas = imu_gyro_meas;
    f.seeker_los_true = truth.los_angle;
    f.seeker_los_meas = seeker_los_meas;
    r.frames.push_back(f);

    // --- Closest-approach bookkeeping ---
    // Analytic CPA over the next dt assuming constant relative velocity. This makes the miss
    // distance continuous and independent of dt (sampling alone would quantize it to ~dt*Vc).
    const double denom = truth.rel_vel.normSq();
    double s_star = denom > 0.0 ? -(truth.rel_pos.dot(truth.rel_vel)) / denom : 0.0;
    if (s_star < 0.0) s_star = 0.0;
    if (s_star > cfg.dt) s_star = cfg.dt;
    const double cpa = (truth.rel_pos + truth.rel_vel * s_star).norm();
    if (cpa < best_range) {
      best_range = cpa;
      best_t = t + s_star;
    }

    // --- Termination ---
    if (veh.pos.z < 0.0 && t > 0.0) break;                     // hit the ground
    if (truth.v_closing < 0.0 && truth.range < 3000.0) break;  // passed CPA in the terminal phase

    // --- Propagate target then vehicle ---
    const Vector3 ta = targetAccel(cfg.target, tgt, t);
    tgt.vel += ta * cfg.dt;
    tgt.pos += tgt.vel * cfg.dt;

    const Vector3 g = gravity.acceleration(veh.pos);
    veh = is6dof ? step6dof(veh, force_world, moment_body, cfg.vehicle.inertia, g, cfg.dt,
                            cfg.integrator)
                 : step3dof(veh, force_world, g, cfg.dt, cfg.integrator);

    // Burn propellant down to the dry-mass floor while thrusting (after the step uses this
    // step's mass for accel, so force_world and accel stay mass-consistent within the step).
    if (boosting && m_dot > 0.0) {
      veh.mass = std::max(veh.mass - m_dot * cfg.dt, dry_mass_floor);
    }
  }

  r.miss_distance = best_range;
  r.intercept_time = best_t;
  r.intercept = best_range < kLethalRadius;
  return r;
}

}  // namespace gncsim
