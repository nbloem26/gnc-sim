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
#include <memory>

#include "gncsim/aero/Aero.hpp"
#include "gncsim/core/Rng.hpp"
#include "gncsim/dynamics/Dynamics.hpp"
#include "gncsim/dynamics/Dynamics6dofHiFi.hpp"
#include "gncsim/env/EnvFidelity.hpp"
#include "gncsim/env/Environment.hpp"
#include "gncsim/env/Frames.hpp"
#include "gncsim/gnc/Discriminator.hpp"
#include "gncsim/gnc/Ekf.hpp"
#include "gncsim/gnc/Gnc.hpp"
#include "gncsim/gnc/TargetTrackEkf.hpp"
#include "gncsim/model/Interfaces.hpp"
#include "gncsim/model/Registry.hpp"
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

// Constant-velocity lead / intercept aim for launch-on-track (issue #8).
//
// Given the launch site `launch_pos`, the interceptor's fixed launch speed `vi`, and the threat's
// estimated current position `tgt_pos` / velocity `tgt_vel` (from the fused track), solve for the
// interceptor VELOCITY that flies a straight constant-speed line to the point where it meets the
// threat assuming the threat continues at constant velocity. With r0 = tgt_pos - launch_pos, the
// intercept time t solves |r0 + tgt_vel*t| = vi*t, i.e. the quadratic
//   (|tgt_vel|^2 - vi^2) t^2 + 2 (r0·tgt_vel) t + |r0|^2 = 0.
// Take the smallest positive root; the aim direction is then (r0 + tgt_vel*t) normalized, scaled to
// vi. `loft_deg` biases the elevation of that aim upward (a lofted launch). If no positive root
// exists (threat outrunning the interceptor) fall back to pointing straight at the current
// estimate. Pure arithmetic — no RNG.
Vector3 leadIntercept(const Vector3& launch_pos, double vi, const Vector3& tgt_pos,
                      const Vector3& tgt_vel, double loft_deg) {
  const Vector3 r0 = tgt_pos - launch_pos;
  const double a = tgt_vel.normSq() - vi * vi;
  const double b = 2.0 * r0.dot(tgt_vel);
  const double c = r0.normSq();

  double t_int = -1.0;
  if (std::fabs(a) < 1e-9) {
    // Degenerate (threat speed ~ interceptor speed): linear equation b t + c = 0.
    if (std::fabs(b) > 1e-12) t_int = -c / b;
  } else {
    const double disc = b * b - 4.0 * a * c;
    if (disc >= 0.0) {
      const double sq = std::sqrt(disc);
      const double t1 = (-b - sq) / (2.0 * a);
      const double t2 = (-b + sq) / (2.0 * a);
      // Smallest strictly-positive root.
      for (double cand : {t1, t2}) {
        if (cand > 1e-6 && (t_int < 0.0 || cand < t_int)) t_int = cand;
      }
    }
  }

  // Aim point: predicted intercept position, or the current estimate if no valid lead solution.
  Vector3 aim_dir = (t_int > 0.0) ? (r0 + tgt_vel * t_int) : r0;
  if (aim_dir.norm() < 1e-9) aim_dir = Vector3{1.0, 0.0, 0.0};
  aim_dir = aim_dir.normalized();

  // Apply the loft (elevation bias) about the horizontal axis perpendicular to the aim azimuth.
  if (std::fabs(loft_deg) > 1e-12) {
    const double az = std::atan2(aim_dir.y, aim_dir.x);
    const Vector3 horiz_axis{-std::sin(az), std::cos(az), 0.0};  // points "left" of the aim
    aim_dir = Quaternion::fromAxisAngle(horiz_axis, loft_deg * M_PI / 180.0).rotate(aim_dir);
    aim_dir = aim_dir.normalized();
  }

  return aim_dir * vi;
}

// =============================================================================================
// Decoy / closely-spaced-object scene (cfg.decoys.enabled — issue #6)
// =============================================================================================
//
// One object in the multi-object scene. Index 0 is, by convention, the true (lethal) target; its
// truth state mirrors the existing single `tgt`. Decoys are placed in a small cluster around the
// true target and propagate ballistically with an EXTRA along-velocity deceleration set by their
// `decel` feature (lighter decoy -> lower ballistic coefficient -> sheds speed faster). Each object
// also carries a static 3-feature signature [intensity, size, decel] the seeker measures noisily.
struct SceneObject {
  EntityState state;     // truth position/velocity (ENU)
  FeatureVec signature;  // static [intensity, size, decel]
  double decel_accel;    // extra deceleration magnitude [m/s^2] applied opposite velocity
};

// Build the decoy scene from the config and the true target's initial state. Index 0 is the true
// target (signature = the configured lethal-target signature, no extra decel). Decoys are scattered
// in a Gaussian cluster of std `separation` about the true target with feature signatures drawn
// from distributions whose MEAN is blended between the distinct decoy means (separability=1) and
// the target signature (separability=0). Draws RNG only here (decoy path is opt-in, so the default
// RNG draw order is untouched).
std::vector<SceneObject> buildDecoyScene(const DecoysConfig& cfg, const EntityState& tgt,
                                         Rng& rng) {
  std::vector<SceneObject> objs;
  const int n = std::max(cfg.count, 0);
  objs.reserve(static_cast<std::size_t>(n) + 1);

  // Index 0: the true lethal target. Its measured signature is its expected signature plus a static
  // per-object draw (aspect/manufacturing variation): the seeker does NOT see the warhead sitting
  // exactly on the textbook signature, so at low separability a decoy's draw can out-score it. This
  // is what makes discrimination genuinely fail as the distributions overlap.
  SceneObject truth_obj;
  truth_obj.state = tgt;
  const double ti = cfg.target_intensity + rng.gaussian(0.0, cfg.feature_spread);
  const double ts = cfg.target_size + rng.gaussian(0.0, cfg.feature_spread);
  const double td = cfg.target_decel + rng.gaussian(0.0, cfg.feature_spread);
  truth_obj.signature = {ti, ts, td};
  // The kinematic feature drives the heavy warhead's (small) extra deceleration baseline.
  truth_obj.decel_accel = std::max(td, 0.0);
  objs.push_back(truth_obj);

  const double sep = std::clamp(cfg.separability, 0.0, 1.0);
  // Decoy feature means blended toward the target signature as separability -> 0.
  const double mu_int = cfg.decoy_intensity * sep + cfg.target_intensity * (1.0 - sep);
  const double mu_size = cfg.decoy_size * sep + cfg.target_size * (1.0 - sep);
  const double mu_decel = cfg.decoy_decel * sep + cfg.target_decel * (1.0 - sep);

  for (int i = 0; i < n; ++i) {
    SceneObject d;
    d.state = tgt;
    // Cluster placement: Gaussian offset about the true target (position + a small velocity
    // spread).
    d.state.pos = tgt.pos + rng.gaussianVec(cfg.separation);
    d.state.vel = tgt.vel + rng.gaussianVec(cfg.separation * 0.02);
    // Static feature signature: per-object draw about the (separability-blended) decoy means.
    const double fi = mu_int + rng.gaussian(0.0, cfg.feature_spread);
    const double fs = mu_size + rng.gaussian(0.0, cfg.feature_spread);
    const double fd = mu_decel + rng.gaussian(0.0, cfg.feature_spread);
    d.signature = {fi, fs, fd};
    d.decel_accel = std::max(fd, 0.0);  // the kinematic feature drives the extra deceleration
    objs.push_back(d);
  }
  return objs;
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

  // --- High-fidelity environment selectors (issue #41). At defaults these reproduce the legacy
  // round path exactly: gravFn == centralGravity(r, with_j2), atmFn == atmosphereUSSA76, no wind.
  const bool use_egm = (cfg.env.gravity_model == "egm");
  GravityFidelityConfig grav_cfg = cfg.env.gravity;
  // Honour the legacy env.j2 flag as the J2 toggle for the EGM model too (so "egm" with no explicit
  // zonal block but env.j2=true still includes J2).
  grav_cfg.include_j2 = grav_cfg.include_j2 || with_j2;
  auto gravFn = [&](const Vector3& r) -> Vector3 {
    return use_egm ? egmGravity(r, grav_cfg) : centralGravity(r, with_j2);
  };
  const bool use_ext_atm = (cfg.env.atmosphere_model == "extended");
  auto atmFn = [&](double alt) -> AtmSample {
    return use_ext_atm ? atmosphereExtended(alt) : atmosphereUSSA76(alt);
  };
  const WindConfig& wind = cfg.env.wind;

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

  // Round-Earth uses 3DOF point-mass integration (gravity supplied per step as ECI central
  // gravity). Resolve it through the registry so even this path runs the same IDynamics seam.
  ModelRegistry registry;
  const std::unique_ptr<IDynamics> dyn = registry.makeDynamics("3dof", cfg.vehicle, cfg.integrator);

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

    const AtmSample atm = cfg.env.atmosphere ? atmFn(alt) : AtmSample{0.0, 0.0, 288.15, 340.29};
    // Air-relative velocity in ECEF: the atmosphere co-rotates with Earth (= ECEF velocity), with
    // an optional parameterized wind in local ENU rotated into ECEF and subtracted off.
    Vector3 air_vel_ecef = veh_vel_ecef_now;
    if (wind.enabled) {
      const Vector3 wind_ecef = enuVecToEcef(windEnu(alt, wind), origin);
      air_vel_ecef = veh_vel_ecef_now - wind_ecef;
    }
    const double airspeed = air_vel_ecef.norm();
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
    const Vector3 g_eci = gravFn(veh_r);
    Vector3 drag_eci{};
    if (cfg.env.atmosphere) {
      // Drag opposes the AIR-relative velocity (co-rotating atmosphere + optional wind).
      const Vector3 drag_ecef = aero.dragForce(air_vel_ecef, atm);
      // Drag is a force vector in ECEF; rotate (no transport term — it's an instantaneous force).
      drag_eci = ecefToEci(drag_ecef, t);
    }
    // step3dof treats `force_world` as everything except gravity; here that's drag only.
    EntityState veh_state;
    veh_state.pos = veh_r;
    veh_state.vel = veh_v;
    veh_state.mass = veh_mass;
    const EntityState veh_next = dyn->step(veh_state, drag_eci, Vector3{}, g_eci, cfg.dt);
    veh_r = veh_next.pos;
    veh_v = veh_next.vel;

    // --- Target: ballistic under the same central gravity (no thrust, no drag for simplicity). ---
    const Vector3 tgt_g = gravFn(tgt_r);
    EntityState tgt_state;
    tgt_state.pos = tgt_r;
    tgt_state.vel = tgt_v;
    tgt_state.mass = 1.0;
    const EntityState tgt_next = dyn->step(tgt_state, Vector3{}, Vector3{}, tgt_g, cfg.dt);
    tgt_r = tgt_next.pos;
    tgt_v = tgt_next.vel;
  }

  r.miss_distance = best_range;
  r.intercept_time = best_t;
  r.intercept = best_range < kLethalRadius;
  return r;
}

// =============================================================================================
// Rotating-ECEF propagation (cfg.env.frame == "round" && cfg.env.rotating_ecef)
// =============================================================================================
//
// Alternative formulation to runRoundEarth's ECI integration. Here the translational state is
// integrated directly in the rotating ECEF frame, so the equations of motion carry the fictitious
// Coriolis (-2 omega x v) and centrifugal (-omega x (omega x r)) accelerations explicitly:
//
//   a_ecef = g(r) - 2 omega x v_ecef - omega x (omega x r) + F_drag/m,   omega = [0,0,kOmega].
//
// Physically this is EQUIVALENT to the ECI path: a special case with omega = 0 (non-rotating Earth)
// makes ECEF == ECI and the two paths agree to integrator precision (asserted in the tests).
// Because the fictitious accelerations are state-dependent, this path uses its own RK4 (the shared
// step3dof holds force constant across a step), evaluating gravity + fictitious + drag at every RK
// stage. Uses NO RNG draws (deterministic, parity-preserving).
SimResult runRoundEarthEcef(const SimConfig& cfg) {
  SimResult r;
  r.scenario = cfg.scenario;
  r.model = cfg.model;
  r.seed = cfg.seed;
  r.dt = cfg.dt;
  r.t_end = cfg.t_end;
  r.origin = cfg.origin;

  const GeodeticOrigin& origin = cfg.origin;
  const Vector3 omega{0.0, 0.0, wgs84::kOmega};

  // Environment selectors (same opt-in semantics as the ECI path).
  const bool use_egm = (cfg.env.gravity_model == "egm");
  GravityFidelityConfig grav_cfg = cfg.env.gravity;
  grav_cfg.include_j2 = grav_cfg.include_j2 || cfg.env.j2;
  auto gravFn = [&](const Vector3& r_ecef) -> Vector3 {
    return use_egm ? egmGravity(r_ecef, grav_cfg) : centralGravity(r_ecef, cfg.env.j2);
  };
  const bool use_ext_atm = (cfg.env.atmosphere_model == "extended");
  auto atmFn = [&](double alt) -> AtmSample {
    return use_ext_atm ? atmosphereExtended(alt) : atmosphereUSSA76(alt);
  };
  const WindConfig& wind = cfg.env.wind;

  AeroModel aero(cfg.aero);

  // ECEF state = (pos, vel). At t = 0 ECEF == ECI, so initial ECEF velocity is the launch velocity
  // expressed in ECEF (co-rotating); NO omega x r transport term is added here — the rotating-frame
  // velocity IS the ENU-derived ECEF velocity.
  struct State2 {
    Vector3 pos, vel;
    State2 operator+(const State2& o) const { return {pos + o.pos, vel + o.vel}; }
    State2 operator*(double s) const { return {pos * s, vel * s}; }
  };

  State2 veh{enuToEcef(cfg.vehicle.pos0, origin),
             enuVecToEcef(launchVelocity(cfg.vehicle), origin)};
  State2 tgt{enuToEcef(cfg.target.pos0, origin), enuVecToEcef(cfg.target.vel0, origin)};
  double veh_mass = cfg.vehicle.mass0;

  // ECEF derivative: gravitation + fictitious forces (+ drag for the vehicle).
  auto deriv = [&](const State2& y, bool with_drag) -> State2 {
    const Vector3 coriolis = omega.cross(y.vel) * (-2.0);
    const Vector3 centrifugal = omega.cross(omega.cross(y.pos)) * (-1.0);
    Vector3 a = gravFn(y.pos) + coriolis + centrifugal;
    if (with_drag && cfg.env.atmosphere) {
      double lat, lon, alt;
      ecefToGeodetic(y.pos, lat, lon, alt);
      const AtmSample atm = atmFn(alt);
      Vector3 air_vel = y.vel;  // atmosphere is stationary in ECEF (co-rotating)
      if (wind.enabled) air_vel = y.vel - enuVecToEcef(windEnu(alt, wind), origin);
      a += aero.dragForce(air_vel, atm) / veh_mass;
    }
    return {y.vel, a};
  };
  auto rk4 = [&](const State2& y, bool with_drag) -> State2 {
    const double dt = cfg.dt;
    const State2 k1 = deriv(y, with_drag);
    const State2 k2 = deriv(y + k1 * (0.5 * dt), with_drag);
    const State2 k3 = deriv(y + k2 * (0.5 * dt), with_drag);
    const State2 k4 = deriv(y + k3 * dt, with_drag);
    return y + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);
  };

  double best_range = (tgt.pos - veh.pos).norm();
  double best_t = 0.0;

  const int steps = static_cast<int>(cfg.t_end / cfg.dt);
  for (int i = 0; i <= steps; ++i) {
    const double t = i * cfg.dt;

    // ECEF -> ENU telemetry (no time rotation: ECEF is already Earth-fixed).
    const Vector3 veh_pos_enu = ecefToEnu(veh.pos, origin);
    const Vector3 veh_vel_enu = ecefVecToEnu(veh.vel, origin);
    const Vector3 tgt_pos_enu = ecefToEnu(tgt.pos, origin);
    const Vector3 tgt_vel_enu = ecefVecToEnu(tgt.vel, origin);

    double lat, lon, alt;
    ecefToGeodetic(veh.pos, lat, lon, alt);
    const AtmSample atm = cfg.env.atmosphere ? atmFn(alt) : AtmSample{0.0, 0.0, 288.15, 340.29};
    Vector3 air_vel = veh.vel;
    if (wind.enabled) air_vel = veh.vel - enuVecToEcef(windEnu(alt, wind), origin);
    const double mach = atm.speed_of_sound > 0.0 ? air_vel.norm() / atm.speed_of_sound : 0.0;

    EntityState veh_e, tgt_e;
    veh_e.pos = veh_pos_enu;
    veh_e.vel = veh_vel_enu;
    tgt_e.pos = tgt_pos_enu;
    tgt_e.vel = tgt_vel_enu;
    const Engagement truth = computeEngagement(veh_e, tgt_e);

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

    // CPA over the next dt, in ECEF (relative geometry is frame-agnostic).
    const Vector3 rel_pos = tgt.pos - veh.pos;
    const Vector3 rel_vel = tgt.vel - veh.vel;
    const double denom = rel_vel.normSq();
    double s_star = denom > 0.0 ? -(rel_pos.dot(rel_vel)) / denom : 0.0;
    if (s_star < 0.0) s_star = 0.0;
    if (s_star > cfg.dt) s_star = cfg.dt;
    const double cpa = (rel_pos + rel_vel * s_star).norm();
    if (cpa < best_range) {
      best_range = cpa;
      best_t = t + s_star;
    }

    if (alt < 0.0 && t > 0.0) break;

    veh = rk4(veh, /*with_drag=*/true);
    tgt = rk4(tgt, /*with_drag=*/false);
  }

  r.miss_distance = best_range;
  r.intercept_time = best_t;
  r.intercept = best_range < kLethalRadius;
  return r;
}

}  // namespace

SimResult runSimulation(const SimConfig& cfg) {
  // Round-Earth opt-in: dispatch to the WGS-84 propagation path. The flat-Earth code below is never
  // reached in round mode, so the default (flat) trajectory stays byte-identical. Within round
  // mode, env.rotating_ecef selects the rotating-ECEF formulation (Coriolis+centrifugal) over the
  // default ECI integration; both are physically equivalent.
  if (cfg.env.frame == "round") {
    return cfg.env.rotating_ecef ? runRoundEarthEcef(cfg) : runRoundEarth(cfg);
  }

  SimResult r;
  r.scenario = cfg.scenario;
  r.model = cfg.model;
  r.seed = cfg.seed;
  r.dt = cfg.dt;
  r.t_end = cfg.t_end;
  r.origin = cfg.origin;

  // Hi-fi 6DOF (issue #35) is a 6DOF model (attitude/rotational state active) with the full inertia
  // tensor, table-driven aero moments, and first-order fin-actuator dynamics layered on top.
  const bool is6dof_hifi = (cfg.model == "6dof_hifi");
  const bool is6dof = (cfg.model == "6dof") || is6dof_hifi;
  Rng rng(cfg.seed);

  // --- Interceptor cueing / launch-on-track (issue #8): opt-in, requires trackers ---
  // When enabled the interceptor is held at its launch site during PHASE 1 (the fused track builds
  // up) and only launches once the criterion fires. The flag below decides whether the vehicle
  // starts stationary or with its usual launch velocity. The whole branch is inert by default, so
  // the legacy launch-at-t=0 trajectory and RNG draw order stay byte-identical.
  const bool use_cueing =
      cfg.cueing.enabled && cfg.trackers.enabled && !cfg.trackers.sensors.empty();
  bool launched = !use_cueing;  // default path is "already launched" at t=0
  double launch_time = 0.0;

  // --- Vehicle initial state ---
  EntityState veh;
  veh.pos = cfg.vehicle.pos0;
  // Under cueing the interceptor is stationary at the launch site until launch; otherwise it gets
  // its configured launch velocity immediately (the legacy behaviour).
  veh.vel = use_cueing ? Vector3{} : launchVelocity(cfg.vehicle);
  veh.mass = cfg.vehicle.mass0;
  if (is6dof && veh.vel.norm() > 1e-6) veh.att = quatFromTo({1, 0, 0}, veh.vel);

  // --- Target initial state ---
  EntityState tgt;
  tgt.pos = cfg.target.pos0;
  tgt.vel = cfg.target.vel0;

  // --- Models (resolved by config string through the registry, issue #31) ---
  // The registry maps the contract's config strings to concrete models behind the interfaces in
  // model/Interfaces.hpp. Each model is an adapter delegating to the existing numerics, so the
  // per-step arithmetic, RNG draw order, and trajectory are byte-identical. Default config strings
  // resolve to exactly today's models.
  ModelRegistry registry;
  const std::unique_ptr<IEnvironment> environment = registry.makeEnvironment(cfg.env);
  const std::unique_ptr<IGuidance> guidance = registry.makeGuidance(cfg.guidance.law, cfg.guidance);
  const std::unique_ptr<INavigator> navigator =
      registry.makeNavigator(cfg.nav.filter, cfg.nav, cfg.sensors.seeker, cfg.dt);
  const std::unique_ptr<IDynamics> dyn =
      registry.makeDynamics(cfg.model, cfg.vehicle, cfg.integrator);
  const std::unique_ptr<IThreat> threat = registry.makeThreat(cfg.target, cfg.env.g0);

  AeroModel aero(cfg.aero);
  // Booster aero: same model on a copy of the aero config with the larger boost reference area,
  // selected only inside the boost window. Built unconditionally; only used when boost_ref_area>0.
  AeroConfig boost_aero_cfg = cfg.aero;
  boost_aero_cfg.ref_area = cfg.propulsion.boost_ref_area;
  AeroModel boost_aero(boost_aero_cfg);
  Imu imu(cfg.sensors.imu, cfg.dt, rng);
  Seeker seeker(cfg.sensors.seeker, rng);

  // Which nav filter is active drives the measurement-building branch + nav telemetry below. The
  // navigator itself is resolved above; this flag only selects how the Runner feeds it. Both the
  // EKF and the IMM (issue #36) consume the nonlinear az/el/range measurement, so they share the
  // same measurement-building branch — the navigator object differs, the feed does not.
  const bool use_ekf = (cfg.nav.filter == "ekf" || cfg.nav.filter == "imm");

  Autopilot autopilot(cfg.control);
  // Fin-actuator dynamics (issue #35). Constructed unconditionally (cheap); only stepped on the
  // hi-fi 6DOF path, so the legacy 6DOF/3DOF paths are untouched.
  FinActuator actuator(cfg.actuator, cfg.dt);

  // --- Multi-sensor target track (issue #5): opt-in fusion of fixed ground/space sensors ---
  // When enabled, guidance uses the FUSED absolute target estimate (track_est - vehicle) instead of
  // the seeker. The default path (trackers disabled) never enters any tracker code below, so its
  // RNG draw order and trajectory stay byte-identical. Each sensor is an ISensor resolved by type.
  const bool use_trackers = cfg.trackers.enabled && !cfg.trackers.sensors.empty();
  std::vector<std::unique_ptr<ISensor>> track_sensors;
  if (use_trackers) {
    track_sensors.reserve(cfg.trackers.sensors.size());
    for (const auto& sc : cfg.trackers.sensors) track_sensors.push_back(registry.makeSensor(sc));
  }
  TargetTrackEkf track_ekf(cfg.dt, cfg.trackers.process_psd);
  if (use_trackers) {
    // Cue the track from a coarse initial fix at the true target state with a wide covariance, so
    // angles-only (IR-only) configs still have a position to refine. This consumes no RNG.
    track_ekf.bootstrap(tgt.pos, tgt.vel);
  }

  // --- Decoy / closely-spaced-object discrimination (issue #6): opt-in ---
  // When enabled, build a multi-object scene (true target at index 0 + N decoys) and a
  // feature-based discriminator. Each step the seeker measures every object's features (noisily),
  // the discriminator selects the most target-like object, and guidance homes on the SELECTED
  // object's position instead of the true target. CPA / miss is always scored against the true
  // target, so a wrong selection produces a large miss. The default path (decoys disabled) never
  // enters any code below, so its RNG draw order and trajectory stay byte-identical.
  const bool use_decoys = cfg.decoys.enabled && cfg.decoys.count > 0;
  std::vector<SceneObject> scene =
      use_decoys ? buildDecoyScene(cfg.decoys, tgt, rng) : std::vector<SceneObject>{};
  const int kTargetIndex = 0;  // index of the true target within the scene
  Discriminator discriminator(cfg.decoys, use_decoys ? static_cast<int>(scene.size()) : 1,
                              kTargetIndex);

  double best_range = (tgt.pos - veh.pos).norm();
  double best_t = 0.0;
  Vector3
      accel_achieved;  // realized guidance accel, lagged toward the command (finite autopilot τ)

  // --- APN target-acceleration estimator state (only used when guidance.law == "apn") ---
  // d(rel_vel)/dt = a_target - a_vehicle, and a_vehicle is the known achieved guidance accel, so
  //   a_target_est = lowpass( (rel_vel_est[k] - rel_vel_est[k-1]) / dt ) + accel_achieved_prev.
  // Works with either nav filter (alpha-beta or EKF) since it consumes the guidance estimate's
  // rel_vel. No new RNG draws — pure deterministic arithmetic, so parity is preserved.
  // ZEM/ZEV (issue #40) consumes the same estimated-target-acceleration feedforward (folded into
  // the zero-effort miss), so the estimator below runs for both laws.
  const bool use_zemzev = (cfg.guidance.law == "zemzev");
  const bool use_apn = (cfg.guidance.law == "apn");
  const bool estimate_target_accel = use_apn || use_zemzev;
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
    const AtmSample atm = cfg.env.atmosphere ? environment->atmosphere(veh.pos.z)
                                             : AtmSample{0.0, 0.0, 288.15, 340.29};
    const double speed = veh.vel.norm();
    const double mach = atm.speed_of_sound > 0.0 ? speed / atm.speed_of_sound : 0.0;
    veh.mach = mach;

    // --- Truth engagement geometry (against the true target) ---
    const Engagement truth = computeEngagement(veh, tgt);

    // --- Decoy discrimination: pick which object guidance homes on (issue #6) ---
    // `aim` is the object the guidance estimate chases. Default (no decoys) == the true target, so
    // the engagement below is unchanged. With decoys, the seeker measures every object's features
    // noisily (RNG drawn only here), the discriminator integrates scores and selects the most
    // target-like object, and `aim` becomes that selected object's truth state.
    EntityState aim = tgt;
    int selected_obj = kTargetIndex;
    bool discrim_correct = true;
    double discrim_margin = 0.0;
    if (use_decoys) {
      std::vector<FeatureVec> z;
      z.reserve(scene.size());
      for (const auto& obj : scene) {
        z.push_back({obj.signature[0] + rng.gaussian(0.0, cfg.decoys.measurement_noise),
                     obj.signature[1] + rng.gaussian(0.0, cfg.decoys.measurement_noise),
                     obj.signature[2] + rng.gaussian(0.0, cfg.decoys.measurement_noise)});
      }
      discriminator.observe(z);
      selected_obj = discriminator.selected();
      discrim_correct = discriminator.correct();
      discrim_margin = discriminator.margin();
      aim = scene[static_cast<std::size_t>(selected_obj)].state;
    }

    // --- Navigation: estimate the relative state (noisy seeker when sensors enabled) ---
    // Guidance chases `aim` (== the true target on the default path; the selected object with
    // decoys). The closest-approach / miss bookkeeping below always uses the true-target `truth`.
    const Engagement aim_truth = use_decoys ? computeEngagement(veh, aim) : truth;
    Engagement est = aim_truth;
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
        // detect() gates the measurement on a CFAR detection for phenomenology sensors; for the
        // plain radar/ir sensors it always detects and delegates to measure() with the same RNG
        // draws, so the existing fusion path stays byte-identical. On a missed look the tracker
        // coasts (no update this step for that sensor).
        const SensorDetection det = s->detect(tgt.pos, tgt.vel, rng);
        if (det.detected) track_nis = track_ekf.update(s->spec(), det.z);
      }
      track_pos_est = track_ekf.pos();
      nav_nis = track_nis;

      // --- Launch-on-track (issue #8): fire the launch once the criterion is met ---
      // Criterion: fused-track position-covariance trace below the threshold (a confident track),
      // or a hard timeout at max_cue_time, whichever comes first. On firing, record the launch time
      // and aim the interceptor via a constant-velocity lead solution from the current track
      // estimate (loft added). Only reachable on the opt-in cueing path.
      if (use_cueing && !launched) {
        const bool cov_ready = cfg.cueing.launch_criterion == "track_cov" &&
                               track_ekf.covTrace() < cfg.cueing.cov_trace_threshold;
        const bool timed_out = t >= cfg.cueing.max_cue_time;
        if (cov_ready || timed_out) {
          launched = true;
          launch_time = t;
          veh.vel = leadIntercept(veh.pos, cfg.vehicle.launch_speed, track_ekf.pos(),
                                  track_ekf.vel(), cfg.cueing.loft_deg);
          if (is6dof && veh.vel.norm() > 1e-6) veh.att = quatFromTo({1, 0, 0}, veh.vel);
        }
      }

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

      NavMeasurement z;
      z.az = az;
      z.el = el;
      z.range = rng_meas;
      navigator->predict(
          accel_achieved);  // achieved guidance accel from the previous step (0 @ k0)
      navigator->update(z);
      nav_nis = navigator->nis();

      EntityState veh_est = veh, tgt_est = tgt;
      tgt_est.pos = veh.pos + navigator->relPos();
      tgt_est.vel = veh.vel + navigator->relVel();
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
      NavMeasurement z;
      z.measured_rel_pos = measured_rel_pos;
      navigator->update(z);

      // Relative state used by guidance (filtered estimate, or truth if noise-free).
      if (cfg.sensors.enable) {
        EntityState veh_est = veh, tgt_est = tgt;
        tgt_est.pos = veh.pos + navigator->relPos();
        tgt_est.vel = veh.vel + navigator->relVel();
        est = computeEngagement(veh_est, tgt_est);
      }
    }

    // --- Guidance: Proportional Navigation (PN) or Augmented PN (APN) ---
    // During the cueing PHASE 1 (pre-launch) the interceptor is parked at its launch site: no
    // guidance command, no realized accel. Skip straight past the guidance/autopilot math so the
    // APN derivative state isn't seeded from a stationary vehicle either.
    Vector3 accel_cmd;
    if (!launched) {
      accel_achieved = Vector3{};
      accel_achieved_prev = Vector3{};
    } else if (estimate_target_accel) {
      // Estimate the target's acceleration from the navigation relative-velocity derivative. Used
      // by both the APN feedforward and the ZEM/ZEV zero-effort miss (issue #40).
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
      GuidanceState gs;
      gs.a_target_est = a_target_est;
      // ZEM/ZEV needs time-to-go; compute it from the (estimated) engagement geometry. APN ignores
      // it. Leaving it 0 for APN keeps that path byte-identical.
      if (use_zemzev) gs.tgo_s = timeToGo(est, cfg.guidance);
      accel_cmd = guidance->command(est, gs);
    } else {
      accel_cmd = guidance->command(est, GuidanceState{});
    }

    // --- Reaction-control / divert actuation (issue #40) ---
    // Exo-atmospheric divert/ACS: the commanded acceleration is realized by RCS thrusters with a
    // finite divert authority, hard-limited to divert_limit_mps2 (distinct from the aero lift cap).
    // Opt-in: disabled by default, so the legacy aero-steered command is byte-identical.
    if (cfg.guidance.divert.enabled && launched) {
      const double divert_mag = accel_cmd.norm();
      const double divert_limit = cfg.guidance.divert.divert_limit_mps2;
      if (divert_mag > divert_limit && divert_mag > 0.0) {
        accel_cmd = accel_cmd * (divert_limit / divert_mag);
      }
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
    if (is6dof_hifi) {
      // --- High-fidelity 6DOF (issue #35): table-driven aero force/moment + fin actuators ---
      // Force: table-driven normal force (Cn(alpha,Mach)) + drag, plus body-nose thrust.
      force_world = active_aero.force6dofHiFi(veh.vel, veh.att, atm);
      const Vector3 thrust_force = veh.att.rotate(Vector3{1.0, 0.0, 0.0}) * thrust_mag;
      force_world += thrust_force;

      // Moment: the autopilot commands a body moment; control allocation converts it to a fin
      // deflection command, the actuators follow it through their first-order lag + rate/travel
      // limits, and the realized deflection produces the control moment. Aero adds the static
      // restoring moment (Cm(alpha)) and pitch/yaw/roll rate damping. The realized fin deflection
      // is the reported fin_deflection telemetry.
      Vector3 autopilot_fins;  // unused image from the autopilot; actuators own the realized fins
      const Vector3 moment_cmd = autopilot.moment(veh, accel_achieved, autopilot_fins);
      const Vector3 defl_cmd = actuator.allocate(moment_cmd);
      fin_deflection = actuator.step(defl_cmd);
      const Vector3 control_moment = actuator.controlMoment(fin_deflection);
      const Vector3 aero_moment = active_aero.momentAero(veh.vel, veh.att, veh.angVel, atm);
      moment_body = control_moment + aero_moment;
      specific_force = force_world / veh.mass;
    } else if (is6dof) {
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
      f.nav_pos_est = veh.pos + navigator->relPos();
      f.nav_vel_est = veh.vel + navigator->relVel();
    } else {
      f.nav_pos_est = veh.pos + (cfg.sensors.enable ? navigator->relPos() : truth.rel_pos);
      f.nav_vel_est = veh.vel + (cfg.sensors.enable ? navigator->relVel() : truth.rel_vel);
    }
    f.los_angle = truth.los_angle;
    f.los_rate = est.los_rate;
    f.v_closing = truth.v_closing;
    f.range = truth.range;
    f.nav_nis = nav_nis;
    f.track_pos_est = track_pos_est;
    f.track_nis = track_nis;
    f.selected_obj = static_cast<double>(selected_obj);
    f.discrim_correct = discrim_correct ? 1.0 : 0.0;
    f.discrim_margin = discrim_margin;
    f.pre_launch = !launched;  // issue #8: true while parked at the launch site (cued path only)
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
    // Skipped entirely during the cueing PHASE 1: the interceptor is parked at the launch site, so
    // its (huge, static) range to the threat must not pollute the closest-approach metric.
    if (launched) {
      const double denom = truth.rel_vel.normSq();
      double s_star = denom > 0.0 ? -(truth.rel_pos.dot(truth.rel_vel)) / denom : 0.0;
      if (s_star < 0.0) s_star = 0.0;
      if (s_star > cfg.dt) s_star = cfg.dt;
      const double cpa = (truth.rel_pos + truth.rel_vel * s_star).norm();
      if (cpa < best_range) {
        best_range = cpa;
        best_t = t + s_star;
      }

      // --- Termination (only after launch) ---
      if (veh.pos.z < 0.0 && t > 0.0) break;                     // hit the ground
      if (truth.v_closing < 0.0 && truth.range < 3000.0) break;  // passed CPA in the terminal phase
    }

    // --- Propagate target then vehicle ---
    const Vector3 ta = threat->accel(tgt, t);
    tgt.vel += ta * cfg.dt;
    tgt.pos += tgt.vel * cfg.dt;

    // --- Propagate the decoy scene (issue #6) ---
    // Index 0 mirrors the true target. Decoys fly ballistically (gravity, no maneuver) with an
    // extra along-velocity deceleration set by their kinematic feature: a light decoy sheds speed
    // faster than the heavy warhead, which is itself a discriminable kinematic cue. No RNG drawn
    // here.
    if (use_decoys) {
      scene[static_cast<std::size_t>(kTargetIndex)].state = tgt;
      const Vector3 g_obj = environment->gravity(tgt.pos);
      for (std::size_t oi = 0; oi < scene.size(); ++oi) {
        if (static_cast<int>(oi) == kTargetIndex) continue;
        EntityState& s = scene[oi].state;
        Vector3 a = g_obj;
        const double spd = s.vel.norm();
        if (spd > 1e-9) a -= s.vel * (scene[oi].decel_accel / spd);  // deceleration along -velocity
        s.vel += a * cfg.dt;
        s.pos += s.vel * cfg.dt;
      }
    }

    // Propagate the vehicle only after launch — during cueing PHASE 1 it stays parked at the launch
    // site (zero velocity, no forces integrated). The target/track/decoy scene above keep advancing
    // so the threat closes in while the launch criterion is being evaluated.
    if (launched) {
      const Vector3 g = environment->gravity(veh.pos);
      veh = dyn->step(veh, force_world, moment_body, g, cfg.dt);

      // Burn propellant down to the dry-mass floor while thrusting (after the step uses this
      // step's mass for accel, so force_world and accel stay mass-consistent within the step).
      if (boosting && m_dot > 0.0) {
        veh.mass = std::max(veh.mass - m_dot * cfg.dt, dry_mass_floor);
      }
    }
  }

  r.miss_distance = best_range;
  r.intercept_time = best_t;
  r.launch_time = launch_time;
  r.intercept = best_range < kLethalRadius;
  return r;
}

}  // namespace gncsim
