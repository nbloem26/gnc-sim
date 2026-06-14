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
#include "gncsim/dynamics/Dynamics6dofHiFi.hpp"
#include "gncsim/env/Environment.hpp"
#include "gncsim/gnc/Ekf.hpp"
#include "gncsim/gnc/Gnc.hpp"
#include "gncsim/gnc/Imm.hpp"
#include "gncsim/model/Threats.hpp"
#include "gncsim/sensors/Phenomenology.hpp"

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

// Optimal ZEM/ZEV guidance (issue #40). Uses the estimated target acceleration (ZEM feedforward)
// and the runner-supplied time-to-go (gs.tgo_s); midcourse ZEV term + handover fade live in
// zemZevCommand.
class ZemZevGuidance final : public IGuidance {
 public:
  explicit ZemZevGuidance(const GuidanceConfig& cfg) : cfg_(cfg) {}
  Vector3 command(const Engagement& e, const GuidanceState& gs) const override {
    return zemZevCommand(e, cfg_, gs.a_target_est, gs.tgo_s);
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

// Interacting Multiple Model navigator (issue #36): a bank of constant-velocity + maneuver
// nearly-constant-velocity filters mixed by mode probability. Consumes the same az/el/range
// channel as the EKF; exposes the mode-probability-weighted combined NIS. Selected by
// nav.filter == "imm".
class ImmNavigator final : public INavigator {
 public:
  ImmNavigator(double dt, double q_cv, double q_man, double sigma_az, double sigma_el,
               double sigma_range, double p_stay)
      : imm_(dt, q_cv, q_man, sigma_az, sigma_el, sigma_range, p_stay) {}
  void predict(const Vector3& a_vehicle) override { imm_.predict(a_vehicle); }
  void update(const NavMeasurement& z) override { imm_.update(z.az, z.el, z.range); }
  Vector3 relPos() const override { return imm_.relPos(); }
  Vector3 relVel() const override { return imm_.relVel(); }
  double nis() const override { return imm_.nis(); }

 private:
  Imm imm_;
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

// Phenomenology sensor (issue #39): a signal->detection front-end over the plain measurement model.
// Each look it (1) models the SNR from geometry — radar range-equation + Swerling RCS +
// clutter/ECM, or IR NETD + atmospheric transmission — (2) runs CA-CFAR to decide detected/missed
// at the configured Pfa, and (3) on a detection synthesizes the same az/el[/range/range_rate]
// measurement the TargetTrackEkf fuses. RNG draw order is fixed (signal draws, then the CFAR
// Bernoulli, then the measurement-noise draws only on a hit) so native<->WASM parity holds.
class PhenomenologySensorModel final : public ISensor {
 public:
  PhenomenologySensorModel(const TrackSensor& s, const TrackerSensorConfig& cfg)
      : s_(s),
        cfar_(cfg.cfar),
        radar_(cfg.radar),
        ir_(cfg.ir_pheno),
        is_radar_(s.type == TrackSensorType::Radar),
        swerling_(swerlingFromInt(cfg.radar.swerling)) {}

  // measure() is the plain noisy-measurement synthesis (used by the default ISensor::detect only if
  // a caller bypasses detect(); the Runner always calls detect()). Kept consistent with detect().
  std::vector<double> measure(const Vector3& tgt_pos, const Vector3& tgt_vel,
                              Rng& rng) const override {
    const Vector3 rel = tgt_pos - s_.pos;
    const double horiz = std::sqrt(rel.x * rel.x + rel.y * rel.y);
    const double r = rel.norm();
    double az = std::atan2(rel.y, rel.x) + rng.gaussian(0.0, s_.sigma_az);
    double el = std::atan2(rel.z, horiz) + rng.gaussian(0.0, s_.sigma_el);
    if (!is_radar_) return {az, el};
    const double range = r + rng.gaussian(0.0, s_.sigma_range);
    const double range_rate =
        (r > 1e-9 ? rel.dot(tgt_vel) / r : 0.0) + rng.gaussian(0.0, s_.sigma_range_rate);
    return {az, el, range, range_rate};
  }

  SensorDetection detect(const Vector3& tgt_pos, const Vector3& tgt_vel, Rng& rng) const override {
    const Vector3 rel = tgt_pos - s_.pos;
    const double r = rel.norm();

    // (1) Signal model -> linear SNR. Radar draws the instantaneous Swerling RCS first.
    double snr_linear = 0.0;
    double ir_sigma_angle = s_.sigma_az;
    if (is_radar_) {
      const double rcs_m2 = swerlingRcsSample(swerling_, radar_.rcs_mean_m2, rng);
      snr_linear = radarSnrLinear(radar_, r, rcs_m2);
    } else {
      snr_linear = irSnrLinear(ir_, r);
      ir_sigma_angle = irAngleSigmaRad(ir_, snr_linear);
    }

    // (2) CA-CFAR detection decision (one Bernoulli uniform).
    const CfarResult cfar = cfarDetect(cfar_, snr_linear, rng);
    if (!cfar.detected) return SensorDetection{false, {}, cfar.snr_db};

    // (3) On a detection, synthesize the measurement. IR centroid noise scales with the SNR.
    const double horiz = std::sqrt(rel.x * rel.x + rel.y * rel.y);
    if (is_radar_) {
      double az = std::atan2(rel.y, rel.x) + rng.gaussian(0.0, s_.sigma_az);
      double el = std::atan2(rel.z, horiz) + rng.gaussian(0.0, s_.sigma_el);
      const double range = r + rng.gaussian(0.0, s_.sigma_range);
      const double range_rate =
          (r > 1e-9 ? rel.dot(tgt_vel) / r : 0.0) + rng.gaussian(0.0, s_.sigma_range_rate);
      return SensorDetection{true, {az, el, range, range_rate}, cfar.snr_db};
    }
    double az = std::atan2(rel.y, rel.x) + rng.gaussian(0.0, ir_sigma_angle);
    double el = std::atan2(rel.z, horiz) + rng.gaussian(0.0, ir_sigma_angle);
    return SensorDetection{true, {az, el}, cfar.snr_db};
  }

  const TrackSensor& spec() const override { return s_; }

 private:
  TrackSensor s_;
  CfarConfig cfar_;
  RadarPhenomenologyConfig radar_;
  IrPhenomenologyConfig ir_;
  bool is_radar_;
  SwerlingCase swerling_;
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

// High-fidelity 6DOF (issue #35): full inertia tensor + gyroscopic coupling in the rotational EOM.
// The aero-moment tables and actuator dynamics live in the Runner's force/moment block (they need
// per-step atmosphere + autopilot state); this adapter owns the inertia tensor and the coupled
// integration. moment_body is the total body torque (aero restoring/damping + control) assembled by
// the Runner.
class Dynamics6dofHiFi final : public IDynamics {
 public:
  Dynamics6dofHiFi(const InertiaTensor& inertia, Integrator integ)
      : inertia_(inertia), integ_(integ) {}
  EntityState step(const EntityState& s, const Vector3& force_world, const Vector3& moment_body,
                   const Vector3& gravity, double dt) const override {
    return step6dofHiFi(s, force_world, moment_body, inertia_, gravity, dt, integ_);
  }
  bool is6dof() const override { return true; }

 private:
  InertiaTensor inertia_;
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
  if (law == "zemzev") return std::make_unique<ZemZevGuidance>(cfg);
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
  if (filter == "imm") {
    return std::make_unique<ImmNavigator>(dt, nav.imm_q_cv, nav.imm_q_man, seeker.los_white,
                                          seeker.los_white, nav.range_white, nav.imm_p_stay);
  }
  throw std::invalid_argument("ModelRegistry: unknown nav.filter '" + filter + "'");
}

std::unique_ptr<ISensor> ModelRegistry::makeSensor(const TrackerSensorConfig& sc) const {
  if (sc.type != "radar" && sc.type != "ir" && sc.type != "radar_pheno" && sc.type != "ir_pheno") {
    throw std::invalid_argument("ModelRegistry: unknown sensor type '" + sc.type + "'");
  }
  TrackSensor s;
  const bool is_ir = (sc.type == "ir" || sc.type == "ir_pheno");
  s.type = is_ir ? TrackSensorType::Ir : TrackSensorType::Radar;
  s.pos = sc.pos;
  s.sigma_az = sc.sigma_az;
  s.sigma_el = sc.sigma_el;
  s.sigma_range = sc.sigma_range;
  s.sigma_range_rate = sc.sigma_range_rate;
  if (sc.type == "radar_pheno" || sc.type == "ir_pheno") {
    return std::make_unique<PhenomenologySensorModel>(s, sc);
  }
  return std::make_unique<TrackSensorModel>(s);
}

std::unique_ptr<IDynamics> ModelRegistry::makeDynamics(const std::string& model,
                                                       const VehicleConfig& vehicle,
                                                       Integrator integ) const {
  if (model == "3dof") return std::make_unique<Dynamics3dof>(integ);
  if (model == "6dof") return std::make_unique<Dynamics6dof>(vehicle.inertia, integ);
  if (model == "6dof_hifi") {
    return std::make_unique<Dynamics6dofHiFi>(inertiaFromVehicle(vehicle), integ);
  }
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

std::unique_ptr<IThreat> ModelRegistry::makeThreat(const TargetConfig& target,
                                                   double g0_mps2) const {
  if (target.maneuver == "constant") return std::make_unique<ConstantThreat>();
  if (target.maneuver == "weave") return std::make_unique<WeaveThreat>(target);
  // Threat suite (issue #42): gravitating multi-stage ICBM / hypersonic glide / RV+penaids.
  if (target.maneuver == "icbm") return std::make_unique<IcbmThreat>(target.icbm, g0_mps2);
  if (target.maneuver == "hgv") return std::make_unique<HgvThreat>(target.hgv, g0_mps2);
  if (target.maneuver == "rv_penaids") return std::make_unique<RvPenaidsThreat>(target.rv, g0_mps2);
  throw std::invalid_argument("ModelRegistry: unknown target.maneuver '" + target.maneuver + "'");
}

}  // namespace gncsim
