// gnc-sim — ModelRegistry + concrete model adapters (issue #31).
//
// Each adapter is a thin wrapper that DELEGATES to the existing, validated numerics (proNavCommand,
// the alpha-beta Navigator, the Ekf, TargetTrackEkf measurement synthesis, step3dof/step6dof, the
// GravityModel + USSA76, the weave target maneuver). No numerics are reimplemented here, so the
// per-step arithmetic — and therefore native<->WASM parity and the byte-identical default run — is
// unchanged. The adapters only forward calls in the same order the original hard-wired loop did.
#include "gncsim/model/Registry.hpp"

#include <cmath>
#include <stdexcept>

#include "gncsim/dynamics/Dynamics.hpp"
#include "gncsim/env/Environment.hpp"
#include "gncsim/gnc/Ekf.hpp"
#include "gncsim/gnc/Gnc.hpp"

namespace gncsim {

namespace {

// =============================================================================================
// Guidance adapters
// =============================================================================================

// Proportional Navigation. command() ignores the APN feedforward.
class ProNavGuidance final : public IGuidance {
 public:
  explicit ProNavGuidance(const GuidanceConfig& cfg) : cfg_(cfg) {}
  Vector3 command(const Engagement& e, const GuidanceState&) const override {
    return proNavCommand(e, cfg_);
  }

 private:
  GuidanceConfig cfg_;
};

// Augmented Proportional Navigation: PN plus the estimated-target-acceleration feedforward.
class AugmentedProNavGuidance final : public IGuidance {
 public:
  explicit AugmentedProNavGuidance(const GuidanceConfig& cfg) : cfg_(cfg) {}
  Vector3 command(const Engagement& e, const GuidanceState& gs) const override {
    return augmentedProNavCommand(e, cfg_, gs.a_target_est);
  }

 private:
  GuidanceConfig cfg_;
};

// "none": no guidance command (unguided ballistic). proNavCommand with law=="none" already returns
// zero, but model it explicitly so the law key is total over the contract's values.
class NoGuidance final : public IGuidance {
 public:
  Vector3 command(const Engagement&, const GuidanceState&) const override { return Vector3{}; }
};

// =============================================================================================
// Navigation adapters
// =============================================================================================

// Alpha-beta tracker: consumes the reconstructed relative position; no predict/innovation.
class AlphaBetaNavigator final : public INavigator {
 public:
  explicit AlphaBetaNavigator(double dt) : nav_(dt) {}
  void predict(const Vector3&) override {}  // alpha-beta has no time-update step
  void update(const NavMeasurement& z) override { nav_.update(z.measured_rel_pos); }
  Vector3 relPos() const override { return nav_.relPos(); }
  Vector3 relVel() const override { return nav_.relVel(); }
  double nis() const override { return 0.0; }

 private:
  Navigator nav_;
};

// Relative-state EKF: predict with the vehicle accel, update with the nonlinear az/el/range triple.
class EkfNavigator final : public INavigator {
 public:
  EkfNavigator(double dt, double process_accel_psd, double sigma_az, double sigma_el,
               double sigma_range)
      : ekf_(dt, process_accel_psd, sigma_az, sigma_el, sigma_range) {}
  void predict(const Vector3& a_vehicle) override { ekf_.predict(a_vehicle); }
  void update(const NavMeasurement& z) override { ekf_.update(z.az, z.el, z.range); }
  Vector3 relPos() const override { return ekf_.relPos(); }
  Vector3 relVel() const override { return ekf_.relVel(); }
  double nis() const override { return ekf_.nis(); }

 private:
  Ekf ekf_;
};

// =============================================================================================
// Sensor adapter (radar / IR track sensor)
// =============================================================================================

// Wraps one fixed TrackSensor. measure() synthesizes the noisy absolute-state measurement exactly
// as the original Runner did (Box-Muller draws from the run RNG, in az,el[,range,range_rate]
// order).
class TrackSensorModel final : public ISensor {
 public:
  explicit TrackSensorModel(const TrackSensor& s) : s_(s) {}

  std::vector<double> measure(const Vector3& tgt_pos, const Vector3& tgt_vel,
                              Rng& rng) const override {
    const Vector3 rel = tgt_pos - s_.pos;
    const double horiz = std::sqrt(rel.x * rel.x + rel.y * rel.y);
    const double r = rel.norm();
    double az = std::atan2(rel.y, rel.x);
    double el = std::atan2(rel.z, horiz);
    az += rng.gaussian(0.0, s_.sigma_az);
    el += rng.gaussian(0.0, s_.sigma_el);
    if (s_.type == TrackSensorType::Ir) {
      return {az, el};
    }
    double range = r + rng.gaussian(0.0, s_.sigma_range);
    double range_rate =
        (r > 1e-9 ? rel.dot(tgt_vel) / r : 0.0) + rng.gaussian(0.0, s_.sigma_range_rate);
    return {az, el, range, range_rate};
  }

  const TrackSensor& spec() const override { return s_; }

 private:
  TrackSensor s_;
};

// =============================================================================================
// Dynamics adapters
// =============================================================================================

class Dynamics3dof final : public IDynamics {
 public:
  explicit Dynamics3dof(Integrator integ) : integ_(integ) {}
  EntityState step(const EntityState& s, const Vector3& force_world, const Vector3& /*moment*/,
                   const Vector3& gravity, double dt) const override {
    return step3dof(s, force_world, gravity, dt, integ_);
  }
  bool is6dof() const override { return false; }

 private:
  Integrator integ_;
};

class Dynamics6dof final : public IDynamics {
 public:
  Dynamics6dof(double inertia, Integrator integ) : inertia_(inertia), integ_(integ) {}
  EntityState step(const EntityState& s, const Vector3& force_world, const Vector3& moment_body,
                   const Vector3& gravity, double dt) const override {
    return step6dof(s, force_world, moment_body, inertia_, gravity, dt, integ_);
  }
  bool is6dof() const override { return true; }

 private:
  double inertia_;
  Integrator integ_;
};

// =============================================================================================
// Environment adapter
// =============================================================================================

// Flat-Earth gravity (GravityModel) + USSA76 atmosphere. The round-Earth path uses ECI central
// gravity directly and is dispatched separately in the Runner (out of this adapter's scope).
class FlatEnvironment final : public IEnvironment {
 public:
  explicit FlatEnvironment(const EnvConfig& cfg) : gravity_(cfg) {}
  Vector3 gravity(const Vector3& pos) const override { return gravity_.acceleration(pos); }
  AtmSample atmosphere(double altitude_m) const override { return atmosphereUSSA76(altitude_m); }

 private:
  GravityModel gravity_;
};

// =============================================================================================
// Threat adapters
// =============================================================================================

class ConstantThreat final : public IThreat {
 public:
  Vector3 accel(const EntityState&, double) const override { return Vector3{}; }
};

// Sinusoidal lateral weave, horizontal and perpendicular to the target's ground track. Mirrors the
// original Runner targetAccel().
class WeaveThreat final : public IThreat {
 public:
  explicit WeaveThreat(const TargetConfig& cfg) : cfg_(cfg) {}
  Vector3 accel(const EntityState& tgt, double t) const override {
    Vector3 vh = tgt.vel;
    vh.z = 0.0;
    const double s = vh.norm();
    if (s < 1e-6) return Vector3{};
    const Vector3 perp{-vh.y / s, vh.x / s, 0.0};
    const double phase = cfg_.maneuver_phase_deg * M_PI / 180.0;
    return perp *
           (cfg_.maneuver_g * 9.80665 * std::sin(2.0 * M_PI * cfg_.maneuver_freq * t + phase));
  }

 private:
  TargetConfig cfg_;
};

}  // namespace

// =============================================================================================
// ModelRegistry resolution
// =============================================================================================

std::unique_ptr<IGuidance> ModelRegistry::makeGuidance(const std::string& law,
                                                       const GuidanceConfig& cfg) const {
  if (law == "pronav") return std::make_unique<ProNavGuidance>(cfg);
  if (law == "apn") return std::make_unique<AugmentedProNavGuidance>(cfg);
  if (law == "none") return std::make_unique<NoGuidance>();
  throw std::invalid_argument("ModelRegistry: unknown guidance.law '" + law + "'");
}

std::unique_ptr<INavigator> ModelRegistry::makeNavigator(const std::string& filter,
                                                         const NavConfig& nav,
                                                         const SeekerNoise& seeker,
                                                         double dt) const {
  if (filter == "alpha_beta") return std::make_unique<AlphaBetaNavigator>(dt);
  if (filter == "ekf") {
    return std::make_unique<EkfNavigator>(dt, nav.process_accel_psd, seeker.los_white,
                                          seeker.los_white, nav.range_white);
  }
  throw std::invalid_argument("ModelRegistry: unknown nav.filter '" + filter + "'");
}

std::unique_ptr<ISensor> ModelRegistry::makeSensor(const TrackerSensorConfig& sc) const {
  if (sc.type != "radar" && sc.type != "ir") {
    throw std::invalid_argument("ModelRegistry: unknown sensor type '" + sc.type + "'");
  }
  TrackSensor s;
  s.type = (sc.type == "ir") ? TrackSensorType::Ir : TrackSensorType::Radar;
  s.pos = sc.pos;
  s.sigma_az = sc.sigma_az;
  s.sigma_el = sc.sigma_el;
  s.sigma_range = sc.sigma_range;
  s.sigma_range_rate = sc.sigma_range_rate;
  return std::make_unique<TrackSensorModel>(s);
}

std::unique_ptr<IDynamics> ModelRegistry::makeDynamics(const std::string& model,
                                                       const VehicleConfig& vehicle,
                                                       Integrator integ) const {
  if (model == "3dof") return std::make_unique<Dynamics3dof>(integ);
  if (model == "6dof") return std::make_unique<Dynamics6dof>(vehicle.inertia, integ);
  throw std::invalid_argument("ModelRegistry: unknown model '" + model + "'");
}

std::unique_ptr<IEnvironment> ModelRegistry::makeEnvironment(const EnvConfig& env) const {
  if (env.frame == "flat" || env.frame == "round") {
    // The round-Earth path is dispatched separately in the Runner; the flat-Earth
    // gravity/atmosphere adapter is the one used on the per-step interface seam.
    return std::make_unique<FlatEnvironment>(env);
  }
  throw std::invalid_argument("ModelRegistry: unknown env.frame '" + env.frame + "'");
}

std::unique_ptr<IThreat> ModelRegistry::makeThreat(const TargetConfig& target) const {
  if (target.maneuver == "constant") return std::make_unique<ConstantThreat>();
  if (target.maneuver == "weave") return std::make_unique<WeaveThreat>(target);
  throw std::invalid_argument("ModelRegistry: unknown target.maneuver '" + target.maneuver + "'");
}

}  // namespace gncsim
