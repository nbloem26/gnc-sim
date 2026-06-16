// gnc-sim — threat-suite model implementations (issue #42). See model/Threats.hpp for the design.
// Pure arithmetic, no RNG, no I/O: deterministic and native<->WASM bit-identical.
#include "gncsim/model/Threats.hpp"

#include <algorithm>
#include <cmath>

namespace gncsim {

namespace {

// Unit velocity direction, or +x if (near-)stationary, so thrust/drag always have a defined axis.
Vector3 velDir(const Vector3& v) {
  const double s_mps = v.norm();
  return (s_mps > 1e-9) ? v / s_mps : Vector3{1.0, 0.0, 0.0};
}

}  // namespace

// =============================================================================================
// Multi-stage boosting ICBM
// =============================================================================================

IcbmThreat::IcbmThreat(const IcbmConfig& cfg, double g0_mps2)
    : cfg_(cfg), g0_mps2_(g0_mps2), total_burn_time_s_(0.0) {
  // Full stacked launch mass = payload + every stage's (dry + propellant). Cumulative burn-out
  // times mark each staging event.
  initial_mass_kg_ = cfg_.payload_mass_kg;
  double t_cum_s = 0.0;
  burnout_time_s_.reserve(cfg_.stages.size());
  for (const auto& s : cfg_.stages) {
    initial_mass_kg_ += s.dry_mass_kg + s.propellant_mass_kg;
    t_cum_s += s.burn_time_s;
    burnout_time_s_.push_back(t_cum_s);
  }
  total_burn_time_s_ = t_cum_s;
  // Guard against a zero/empty stack so massAt never divides by zero downstream.
  if (initial_mass_kg_ < 1e-9) initial_mass_kg_ = 1e-9;
}

double IcbmThreat::stagingTimeS(std::size_t stage) const {
  if (cfg_.stages.empty()) return 0.0;
  const std::size_t k = std::min(stage, burnout_time_s_.size() - 1);
  return burnout_time_s_[k];
}

double IcbmThreat::massAt(double t_s) const {
  // Start from the full stack; subtract the dry mass of every fully-staged-away stage, and the
  // propellant already burned (linear within the burning stage).
  double mass_kg = initial_mass_kg_;
  double t_stage_start_s = 0.0;
  for (std::size_t k = 0; k < cfg_.stages.size(); ++k) {
    const IcbmStage& s = cfg_.stages[k];
    const double t_burnout_s = burnout_time_s_[k];
    if (t_s >= t_burnout_s) {
      // Stage fully burned and jettisoned: remove its propellant and its dry mass.
      mass_kg -= s.propellant_mass_kg + s.dry_mass_kg;
    } else if (t_s > t_stage_start_s) {
      // Currently burning this stage: remove the propellant fraction consumed so far.
      const double frac = s.burn_time_s > 0.0 ? (t_s - t_stage_start_s) / s.burn_time_s : 1.0;
      mass_kg -= s.propellant_mass_kg * std::clamp(frac, 0.0, 1.0);
      break;  // later stages haven't ignited
    } else {
      break;  // before this stage starts
    }
    t_stage_start_s = t_burnout_s;
  }
  return std::max(mass_kg, 1e-9);
}

Vector3 IcbmThreat::accel(const EntityState& tgt, double t) const {
  Vector3 a{0.0, 0.0, -g0_mps2_};  // gravity always acts
  // Which stage (if any) is burning at time t?
  double t_stage_start_s = 0.0;
  for (std::size_t k = 0; k < cfg_.stages.size(); ++k) {
    const IcbmStage& s = cfg_.stages[k];
    const double t_burnout_s = burnout_time_s_[k];
    if (t >= t_stage_start_s && t < t_burnout_s && s.thrust_n > 0.0) {
      const double m_kg = massAt(t);
      a += velDir(tgt.vel) * (s.thrust_n / m_kg);
      break;
    }
    t_stage_start_s = t_burnout_s;
  }
  return a;
}

// =============================================================================================
// Hypersonic glide vehicle (skip-glide)
// =============================================================================================

HgvThreat::HgvThreat(const HgvConfig& cfg, double g0_mps2) : cfg_(cfg), g0_mps2_(g0_mps2) {}

double HgvThreat::densityAt(double alt_m) const {
  const double a_m = std::max(alt_m, 0.0);
  return cfg_.rho0_kgpm3 * std::exp(-a_m / cfg_.scale_height_m);
}

Vector3 HgvThreat::accel(const EntityState& tgt, double t) const {
  (void)t;
  Vector3 a{0.0, 0.0, -g0_mps2_};  // gravity

  const double alt_m = tgt.pos.z;
  const double speed_mps = tgt.vel.norm();
  if (speed_mps < 1e-6) return a;

  // Drag deceleration magnitude: q / beta = 0.5 * rho * V^2 / ballistic_coeff  [m/s^2].
  const double rho = densityAt(alt_m);
  const double q = 0.5 * rho * speed_mps * speed_mps;
  const double drag_decel_mps2 = (cfg_.ballistic_coeff > 1e-9) ? q / cfg_.ballistic_coeff : 0.0;

  const Vector3 vhat = tgt.vel / speed_mps;
  a -= vhat * drag_decel_mps2;  // drag opposes velocity

  // Lift: only in the glide regime (below the pull-up altitude, where there is air to work with).
  // It acts perpendicular to velocity, in the vertical plane, pointing "up" (away from the planet),
  // with magnitude (L/D) * drag. This is the skip-inducing force.
  if (alt_m < cfg_.pull_up_alt_m) {
    // In-plane perpendicular: project +z (up) off the velocity direction and normalize.
    Vector3 up{0.0, 0.0, 1.0};
    Vector3 lift_dir = up - vhat * up.dot(vhat);
    const double ln = lift_dir.norm();
    if (ln > 1e-9) {
      lift_dir = lift_dir / ln;
      const double lift_accel_mps2 = cfg_.ld_ratio * drag_decel_mps2;
      a += lift_dir * lift_accel_mps2;
    }
  }
  return a;
}

// =============================================================================================
// Reentry vehicle + penaids
// =============================================================================================

RvPenaidsThreat::RvPenaidsThreat(const RvPenaidsConfig& cfg, double g0_mps2)
    : cfg_(cfg), g0_mps2_(g0_mps2) {}

Vector3 RvPenaidsThreat::accel(const EntityState& /*tgt*/, double /*t*/) const {
  return Vector3{0.0, 0.0, -g0_mps2_};  // ballistic RV
}

RvPenaids::RvPenaids(const RvPenaidsConfig& cfg, double g0_mps2) : cfg_(cfg), g0_mps2_(g0_mps2) {}

std::vector<RvObject> RvPenaids::deploy(const EntityState& rv) const {
  std::vector<RvObject> scene;
  const int n = std::max(cfg_.penaid_count, 0);
  scene.reserve(static_cast<std::size_t>(n) + 1);

  // Index 0: the true RV (heavy, no extra deceleration).
  RvObject true_rv;
  true_rv.state = rv;
  true_rv.is_true_rv = true;
  true_rv.decel_mps2 = 0.0;
  scene.push_back(true_rv);

  // Penaids dispensed in a deterministic radial fan in the plane perpendicular to the RV velocity,
  // each with the configured dispense delta-v. No RNG: a fixed angular pattern keeps the threat
  // deterministic and WASM-parity-safe.
  const Vector3 vhat = velDir(rv.vel);
  // Two in-plane axes perpendicular to vhat.
  Vector3 e1 = Vector3{0.0, 0.0, 1.0} - vhat * vhat.z;
  if (e1.norm() < 1e-6) e1 = Vector3{0.0, 1.0, 0.0} - vhat * vhat.y;
  e1 = e1.normalized();
  const Vector3 e2 = vhat.cross(e1).normalized();

  for (int i = 0; i < n; ++i) {
    const double ang_rad = (n > 0) ? (2.0 * M_PI * i) / n : 0.0;
    const Vector3 dv_mps = (e1 * std::cos(ang_rad) + e2 * std::sin(ang_rad)) * cfg_.deploy_dv_mps;
    RvObject p;
    p.state = rv;
    p.state.vel = rv.vel + dv_mps;
    p.is_true_rv = false;
    p.decel_mps2 = std::max(cfg_.penaid_decel_mps2, 0.0);
    scene.push_back(p);
  }
  return scene;
}

void RvPenaids::propagate(std::vector<RvObject>& scene, double dt_s) const {
  const Vector3 g{0.0, 0.0, -g0_mps2_};
  for (auto& obj : scene) {
    Vector3 a = g;
    const double spd_mps = obj.state.vel.norm();
    if (obj.decel_mps2 > 0.0 && spd_mps > 1e-9) {
      a -= obj.state.vel * (obj.decel_mps2 / spd_mps);  // deceleration along -velocity
    }
    obj.state.vel += a * dt_s;
    obj.state.pos += obj.state.vel * dt_s;
  }
}

double RvPenaids::scoreAgainstTrueRv(const std::vector<RvObject>& scene) const {
  // The true RV's deceleration feature is the discrimination threshold: a penaid is correctly
  // judged when its deceleration exceeds the RV's (lighter -> sheds speed faster). Fraction of
  // penaids correctly classified.
  if (scene.empty()) return 0.0;
  double rv_decel_mps2 = 0.0;
  for (const auto& o : scene) {
    if (o.is_true_rv) {
      rv_decel_mps2 = o.decel_mps2;
      break;
    }
  }
  int penaids = 0;
  int correct = 0;
  for (const auto& o : scene) {
    if (o.is_true_rv) continue;
    ++penaids;
    if (o.decel_mps2 > rv_decel_mps2) ++correct;
  }
  return (penaids > 0) ? static_cast<double>(correct) / penaids : 0.0;
}

}  // namespace gncsim
