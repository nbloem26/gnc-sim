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
#include "gncsim/env/Frames.hpp"
#include "gncsim/gnc/Ekf.hpp"
#include "gncsim/gnc/Gnc.hpp"
#include "gncsim/gnc/TargetTrackEkf.hpp"
#include "gncsim/sensors/Sensors.hpp"

namespace gncsim {

namespace {

constexpr double kLethalRadius = 3.0;  // intercept if closest approach is within this [m]

// APN target-acceleration estimator ceiling [m/s^2] ≈ 6 g. A real maneuvering target cannot pull
// much more than this; the cap rejects the large spurious accelerations that differentiating a
// noisy navigation velocity produces, so noise spikes can't saturate the feedforward term.
constexpr double kMaxTargetManeuver = 60.0;

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

// Build the fixed TrackSensor list from the trackers config (issue #5).
std::vector<TrackSensor> buildTrackSensors(const TrackersConfig& cfg) {
  std::vector<TrackSensor> sensors;
  sensors.reserve(cfg.sensors.size());
  for (const auto& sc : cfg.sensors) {
    TrackSensor s;
    s.type = (sc.type == "ir") ? TrackSensorType::Ir : TrackSensorType::Radar;
    s.pos = sc.pos;
    s.sigma_az = sc.sigma_az;
    s.sigma_el = sc.sigma_el;
    s.sigma_range = sc.sigma_range;
    s.sigma_range_rate = sc.sigma_range_rate;
    sensors.push_back(s);
  }
  return sensors;
}

// Synthesize one sensor's noisy measurement of the target's true absolute state, drawing Gaussian
// noise from the run RNG (Box-Muller — parity-preserving). Radar -> [az,el,range,range_rate];
// IR -> [az,el]. The measurement is taken from the sensor's fixed ENU position.
std::vector<double> synthSensorMeasurement(const TrackSensor& s, const Vector3& tgt_pos,
                                           const Vector3& tgt_vel, Rng& rng) {
  const Vector3 rel = tgt_pos - s.pos;
  const double horiz = std::sqrt(rel.x * rel.x + rel.y * rel.y);
  const double r = rel.norm();
  double az = std::atan2(rel.y, rel.x);
  double el = std::atan2(rel.z, horiz);
  az += rng.gaussian(0.0, s.sigma_az);
  el += rng.gaussian(0.0, s.sigma_el);
  if (s.type == TrackSensorType::Ir) {
    return {az, el};
  }
  double range = r + rng.gaussian(0.0, s.sigma_range);
  double range_rate =
      (r > 1e-9 ? rel.dot(tgt_vel) / r : 0.0) + rng.gaussian(0.0, s.sigma_range_rate);
  return {az, el, range, range_rate};
}

// =============================================================================================
// Round-Earth propagation (cfg.env.frame == "round")
// =============================================================================================
//
// Propagation frame: ECI (Earth-Centred Inertial), coincident with ECEF at t = 0 (GMST = 0). An
// inertial integration frame means the equations of motion carry NO Coriolis/centrifugal term —
// only true forces (central gravity + drag). Earth's rotation is applied purely as a kinematic
// ECI->ECEF rotation when sampling the atmosphere and emitting telemetry. This keeps the existing
// fixed-acceleration RK4 integrator (step3dof) reusable and the scope bounded.
//
// Frame flow each step:
//   ECI state  --(omega*t rotation)-->  ECEF  --(about origin)-->  ENU  (telemetry + engagement)
//   drag: computed from the airspeed relative to the co-rotating atmosphere (= ECEF velocity),
//         built in ECEF, rotated back to ECI to integrate.
//
// Gravity: central point-mass -GM/r^2 * r_hat in ECI, with an optional J2 term (cfg.env.j2).
// Guidance/sensors/EKF are intentionally out of scope here: this path targets the unguided
// ballistic launch-engagement comparison case. Relative engagement geometry is still computed in
// ENU (frame-agnostic) and recorded for the contract, but no guidance command is applied.
//
// Uses NO RNG draws (deterministic, parity-preserving).
SimResult runRoundEarth(const SimConfig& cfg) {
  SimResult r;
  r.scenario = cfg.scenario;
  r.model = cfg.model;
  r.seed = cfg.seed;
  r.dt = cfg.dt;
  r.t_end = cfg.t_end;
  r.origin = cfg.origin;

  const GeodeticOrigin& origin = cfg.origin;
  const bool with_j2 = cfg.env.j2;

  // --- Initial states: ENU (about origin) -> ECEF -> ECI. ---
  // Position: ENU point -> ECEF point -> ECI (coincident at t=0, so ECI == ECEF here).
  const Vector3 veh_enu0 = cfg.vehicle.pos0;
  const Vector3 veh_ecef0 = enuToEcef(veh_enu0, origin);
  Vector3 veh_r = ecefToEci(veh_ecef0, 0.0);
  // Velocity: ENU launch vel -> ECEF (rotation) -> ECI inertial (add Earth rotation omega x r).
  const Vector3 veh_vel_ecef = enuVecToEcef(launchVelocity(cfg.vehicle), origin);
  Vector3 veh_v = ecefVelToEci(veh_ecef0, veh_vel_ecef, 0.0);
  double veh_mass = cfg.vehicle.mass0;

  const Vector3 tgt_ecef0 = enuToEcef(cfg.target.pos0, origin);
  Vector3 tgt_r = ecefToEci(tgt_ecef0, 0.0);
  const Vector3 tgt_vel_ecef = enuVecToEcef(cfg.target.vel0, origin);
  Vector3 tgt_v = ecefVelToEci(tgt_ecef0, tgt_vel_ecef, 0.0);

  AeroModel aero(cfg.aero);

  // Helper: ECI state -> ENU telemetry (position + velocity) about the origin at time t.
  auto eciToEnuState = [&](const Vector3& r_eci, const Vector3& v_eci, double t, Vector3& pos_enu,
                           Vector3& vel_enu) {
    const Vector3 r_ecef = eciToEcef(r_eci, t);
    const Vector3 v_ecef = eciVelToEcef(r_eci, v_eci, t);
    pos_enu = ecefToEnu(r_ecef, origin);
    vel_enu = ecefVecToEnu(v_ecef, origin);
  };

  double best_range = (tgt_r - veh_r).norm();
  double best_t = 0.0;

  const int steps = static_cast<int>(cfg.t_end / cfg.dt);
  for (int i = 0; i <= steps; ++i) {
    const double t = i * cfg.dt;

    // --- ECEF / geodetic state for atmosphere + termination. ---
    const Vector3 veh_ecef = eciToEcef(veh_r, t);
    const Vector3 veh_vel_ecef_now = eciVelToEcef(veh_r, veh_v, t);
    double lat, lon, alt;
    ecefToGeodetic(veh_ecef, lat, lon, alt);

    const AtmSample atm =
        cfg.env.atmosphere ? atmosphereUSSA76(alt) : AtmSample{0.0, 0.0, 288.15, 340.29};
    const double airspeed = veh_vel_ecef_now.norm();
    const double mach = atm.speed_of_sound > 0.0 ? airspeed / atm.speed_of_sound : 0.0;

    // --- ENU telemetry + engagement geometry (frame-agnostic relative state). ---
    Vector3 veh_pos_enu, veh_vel_enu, tgt_pos_enu, tgt_vel_enu;
    eciToEnuState(veh_r, veh_v, t, veh_pos_enu, veh_vel_enu);
    eciToEnuState(tgt_r, tgt_v, t, tgt_pos_enu, tgt_vel_enu);

    EntityState veh_e, tgt_e;
    veh_e.pos = veh_pos_enu;
    veh_e.vel = veh_vel_enu;
    tgt_e.pos = tgt_pos_enu;
    tgt_e.vel = tgt_vel_enu;
    const Engagement truth = computeEngagement(veh_e, tgt_e);

    // --- Record telemetry frame. ---
    Frame f;
    f.t = t;
    f.veh_pos = veh_pos_enu;
    f.veh_vel = veh_vel_enu;
    f.mass = veh_mass;
    f.mach = mach;
    f.tgt_pos = tgt_pos_enu;
    f.tgt_vel = tgt_vel_enu;
    f.nav_pos_est = veh_pos_enu + truth.rel_pos;
    f.nav_vel_est = veh_vel_enu + truth.rel_vel;
    f.los_angle = truth.los_angle;
    f.los_rate = truth.los_rate;
    f.v_closing = truth.v_closing;
    f.range = truth.range;
    f.seeker_los_true = truth.los_angle;
    f.seeker_los_meas = truth.los_angle;
    r.frames.push_back(f);

    // --- Closest approach (analytic CPA over the next dt) in ECI. ---
    const Vector3 rel_pos = tgt_r - veh_r;
    const Vector3 rel_vel = tgt_v - veh_v;
    const double denom = rel_vel.normSq();
    double s_star = denom > 0.0 ? -(rel_pos.dot(rel_vel)) / denom : 0.0;
    if (s_star < 0.0) s_star = 0.0;
    if (s_star > cfg.dt) s_star = cfg.dt;
    const double cpa = (rel_pos + rel_vel * s_star).norm();
    if (cpa < best_range) {
      best_range = cpa;
      best_t = t + s_star;
    }

    // --- Termination: vehicle returns to (below) the ellipsoid surface. ---
    if (alt < 0.0 && t > 0.0) break;

    // --- Forces in ECI: central gravity + drag (drag built in ECEF, rotated into ECI). ---
    const Vector3 g_eci = centralGravity(veh_r, with_j2);
    Vector3 drag_eci{};
    if (cfg.env.atmosphere) {
      const Vector3 drag_ecef = aero.dragForce(veh_vel_ecef_now, atm);
      // Drag is a force vector in ECEF; rotate (no transport term — it's an instantaneous force).
      drag_eci = ecefToEci(drag_ecef, t);
    }
    // step3dof treats `force_world` as everything except gravity; here that's drag only.
    EntityState veh_state;
    veh_state.pos = veh_r;
    veh_state.vel = veh_v;
    veh_state.mass = veh_mass;
    const EntityState veh_next = step3dof(veh_state, drag_eci, g_eci, cfg.dt, cfg.integrator);
    veh_r = veh_next.pos;
    veh_v = veh_next.vel;

    // --- Target: ballistic under the same central gravity (no thrust, no drag for simplicity). ---
    const Vector3 tgt_g = centralGravity(tgt_r, with_j2);
    EntityState tgt_state;
    tgt_state.pos = tgt_r;
    tgt_state.vel = tgt_v;
    tgt_state.mass = 1.0;
    const EntityState tgt_next = step3dof(tgt_state, Vector3{}, tgt_g, cfg.dt, cfg.integrator);
    tgt_r = tgt_next.pos;
    tgt_v = tgt_next.vel;
  }

  r.miss_distance = best_range;
  r.intercept_time = best_t;
  r.intercept = best_range < kLethalRadius;
  return r;
}

}  // namespace

SimResult runSimulation(const SimConfig& cfg) {
  // Round-Earth opt-in: dispatch to the WGS-84 / ECI propagation path. The flat-Earth code below
  // is never reached in round mode, so the default (flat) trajectory stays byte-identical.
  if (cfg.env.frame == "round") {
    return runRoundEarth(cfg);
  }

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

  // --- Multi-sensor target track (issue #5): opt-in fusion of fixed ground/space sensors ---
  // When enabled, guidance uses the FUSED absolute target estimate (track_est - vehicle) instead of
  // the seeker. The default path (trackers disabled) never enters any tracker code below, so its
  // RNG draw order and trajectory stay byte-identical.
  const bool use_trackers = cfg.trackers.enabled && !cfg.trackers.sensors.empty();
  const std::vector<TrackSensor> track_sensors =
      use_trackers ? buildTrackSensors(cfg.trackers) : std::vector<TrackSensor>{};
  TargetTrackEkf track_ekf(cfg.dt, cfg.trackers.process_psd);
  if (use_trackers) {
    // Cue the track from a coarse initial fix at the true target state with a wide covariance, so
    // angles-only (IR-only) configs still have a position to refine. This consumes no RNG.
    track_ekf.bootstrap(tgt.pos, tgt.vel);
  }

  double best_range = (tgt.pos - veh.pos).norm();
  double best_t = 0.0;
  Vector3
      accel_achieved;  // realized guidance accel, lagged toward the command (finite autopilot τ)

  // --- APN target-acceleration estimator state (only used when guidance.law == "apn") ---
  // d(rel_vel)/dt = a_target - a_vehicle, and a_vehicle is the known achieved guidance accel, so
  //   a_target_est = lowpass( (rel_vel_est[k] - rel_vel_est[k-1]) / dt ) + accel_achieved_prev.
  // Works with either nav filter (alpha-beta or EKF) since it consumes the guidance estimate's
  // rel_vel. No new RNG draws — pure deterministic arithmetic, so parity is preserved.
  const bool use_apn = (cfg.guidance.law == "apn");
  Vector3 rel_vel_prev;         // est rel_vel from the previous step
  Vector3 a_target_est;         // low-pass-filtered target accel estimate
  bool apn_have_prev = false;   // guard the first step (no derivative yet)
  Vector3 accel_achieved_prev;  // a_vehicle from the previous step

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
    Vector3 track_pos_est;  // fused absolute target-position estimate (issue #5); 0 when disabled
    double track_nis = 0.0;

    if (use_trackers) {
      // Multi-sensor fusion path. Predict the target track, then sequentially fuse a synthesized
      // noisy measurement from each fixed sensor (radar [az,el,range,range_rate] / IR [az,el]).
      // Guidance below uses the fused absolute target estimate relative to the vehicle. RNG is
      // drawn only here (this whole branch is unreachable on the default path).
      track_ekf.predict();
      for (const auto& s : track_sensors) {
        const std::vector<double> z = synthSensorMeasurement(s, tgt.pos, tgt.vel, rng);
        track_nis = track_ekf.update(s, z);
      }
      track_pos_est = track_ekf.pos();
      nav_nis = track_nis;

      EntityState veh_est = veh, tgt_est = tgt;
      tgt_est.pos = track_ekf.pos();
      tgt_est.vel = track_ekf.vel();
      est = computeEngagement(veh_est, tgt_est);
    } else if (use_ekf) {
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

    // --- Guidance: Proportional Navigation (PN) or Augmented PN (APN) ---
    Vector3 accel_cmd;
    if (use_apn) {
      // Estimate the target's acceleration from the navigation relative-velocity derivative.
      // accel_achieved here is still the previous step's realized accel (updated below), i.e. the
      // a_vehicle that acted over the interval that produced this step's rel_vel change.
      if (apn_have_prev) {
        const Vector3 d_rel_vel = (est.rel_vel - rel_vel_prev) / cfg.dt;
        Vector3 a_target_raw = d_rel_vel + accel_achieved_prev;
        // Clamp the raw estimate to a physically plausible target-maneuver ceiling before
        // filtering. Differentiating a noisy navigation velocity blows up into huge spurious
        // accelerations; a real maneuvering target cannot pull more than a handful of g, so capping
        // at ~6 g rejects the differentiated-noise spikes that would otherwise saturate (and wreck)
        // the feedforward.
        const double raw_mag = a_target_raw.norm();
        if (raw_mag > kMaxTargetManeuver && raw_mag > 0.0) {
          a_target_raw = a_target_raw * (kMaxTargetManeuver / raw_mag);
        }
        // First-order low-pass to tame the residual noise in the numerical derivative.
        const double tau = cfg.guidance.apn_filter_tau;
        const double blend = tau > 0.0 ? std::min(cfg.dt / tau, 1.0) : 1.0;
        a_target_est += (a_target_raw - a_target_est) * blend;
      }
      rel_vel_prev = est.rel_vel;
      apn_have_prev = true;
      accel_cmd = augmentedProNavCommand(est, cfg.guidance, a_target_est);
    } else {
      accel_cmd = proNavCommand(est, cfg.guidance);
    }

    // First-order autopilot lag: the interceptor cannot realize the command instantly. Against a
    // maneuvering target this finite time constant is the dominant miss-distance driver.
    if (cfg.guidance.time_constant > 0.0) {
      const double blend = cfg.dt / cfg.guidance.time_constant;
      accel_achieved += (accel_cmd - accel_achieved) * (blend < 1.0 ? blend : 1.0);
    } else {
      accel_achieved = accel_cmd;
    }
    // Remember this step's realized accel for the next step's APN target-accel estimate.
    accel_achieved_prev = accel_achieved;

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
    f.track_pos_est = track_pos_est;
    f.track_nis = track_nis;
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
