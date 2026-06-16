// gnc-sim — Interacting Multiple Model (IMM) relative-state estimator for maneuvering targets
// (issue #36). A bank of constant-velocity / higher-process-noise (maneuver) Kalman filters run in
// parallel over the SAME relative state [rel_pos(3), rel_vel(3)] and the SAME nonlinear seeker
// measurement (az, el, range); each step does interaction/mixing of the per-filter estimates,
// independent per-filter measurement updates, and a likelihood-weighted recombination. The mode
// probabilities track which dynamics best explain the data, so the fused estimate stays consistent
// across both quiescent flight and hard maneuvers — where a single nearly-constant-velocity EKF
// lags and goes NIS-inconsistent.
//
// Selectable as an INavigator alternative to "alpha_beta" / "ekf" via nav.filter == "imm". The
// default nav path is untouched.
//
// Construction of the bank: the models differ ONLY in their target-acceleration process-noise PSD
// (q_cv small, q_man large). This is the standard CV/CA IMM specialization — a constant-velocity
// model and a maneuver (white-noise-acceleration) model — and keeps every filter on the identical
// 6-state, az/el/range measurement, so all the linear algebra is shared and small.
//
// Fixed-size hand-rolled linear algebra (std::array) — no Eigen / external dependency, no std
// distributions, deterministic fixed FP order — so the estimator is bit-for-bit identical between
// native (libstdc++) and WASM (libc++), exactly like Ekf.
#pragma once

#include <array>
#include <vector>

#include "gncsim/math/Vector3.hpp"

namespace gncsim {

// One model in the IMM bank: a 6-state nearly-constant-velocity Kalman filter whose
// target-acceleration process-noise PSD `q` distinguishes it from its siblings (small q => CV /
// quiescent model, large q => maneuver model). All models share the az/el/range measurement.
class ImmModel {
 public:
  ImmModel(double dt, double q, double sigma_az, double sigma_el, double sigma_range);

  // Time update over dt with the vehicle's achieved acceleration as the known control input
  // (u = -a_vehicle; gravity cancels as a common-mode term in the relative frame). No-op until
  // bootstrapped.
  void predict(const Vector3& a_vehicle);

  // Measurement update from a relative-position observation (az, el, range). Returns the Gaussian
  // measurement likelihood Λ = N(y; 0, S) of this model's innovation, used by the IMM to update the
  // mode probabilities. The first call bootstraps the state. NIS of the innovation is stored.
  double update(double az, double el, double range);

  // Bootstrap directly from a known mean/covariance (used by the IMM mixing step to seed each model
  // with the mixed estimate before its own update).
  void setState(const std::array<double, 6>& x, const std::array<double, 36>& p);

  const std::array<double, 6>& state() const { return x_; }
  const std::array<double, 36>& cov() const { return p_; }
  bool initialized() const { return initialized_; }
  double nis() const { return nis_; }

  Vector3 relPos() const { return {x_[0], x_[1], x_[2]}; }
  Vector3 relVel() const { return {x_[3], x_[4], x_[5]}; }

 private:
  double dt_;
  double q_;
  std::array<double, 3> r_diag_;

  bool initialized_ = false;
  std::array<double, 6> x_{};
  std::array<double, 36> p_{};
  double nis_ = 0.0;
};

// Interacting Multiple Model estimator over a bank of ImmModel filters. The bank, the Markov
// mode-transition matrix, and the initial mode probabilities are set at construction. Each step:
//   (1) interaction/mixing  — mix each model's prior with its siblings' weighted by the predicted
//       mode probabilities and the transition matrix;
//   (2) predict + update    — each model runs its own time + measurement update, yielding a
//       likelihood;
//   (3) mode update         — combine the predicted mode probabilities with the likelihoods;
//   (4) combination         — output the probability-weighted fused mean (and a combined NIS).
class Imm {
 public:
  // dt                 : fixed integration step [s]
  // q_cv / q_man       : process-accel PSD of the constant-velocity and maneuver models [m^2/s^3]
  // sigma_az/el/range  : measurement noise std (rad, rad, m)
  // p_stay             : Markov probability of remaining in the same mode each step (off-diagonal
  //                      mass is split evenly across the other modes). Higher => stickier modes.
  Imm(double dt, double q_cv, double q_man, double sigma_az, double sigma_el, double sigma_range,
      double p_stay = 0.999);

  void predict(const Vector3& a_vehicle);
  void update(double az, double el, double range);

  Vector3 relPos() const { return {x_[0], x_[1], x_[2]}; }
  Vector3 relVel() const { return {x_[3], x_[4], x_[5]}; }
  double nis() const { return nis_; }  // combined innovation NIS (dof = 3)

  // Mode probabilities (sum to 1). Index 0 is the constant-velocity model; the last is the maneuver
  // model. Exposed for telemetry / tests (mode switching during a maneuver).
  const std::vector<double>& modeProbabilities() const { return mu_; }
  double maneuverProbability() const { return mu_.empty() ? 0.0 : mu_.back(); }
  bool initialized() const { return initialized_; }

 private:
  void recombine();  // fuse model means/covs into x_/p_ by mode probability

  double dt_;
  bool initialized_ = false;

  std::vector<ImmModel> models_;
  std::vector<double> mu_;                  // mode probabilities
  std::vector<std::vector<double>> trans_;  // Markov transition matrix [from][to]

  std::array<double, 6> x_{};   // fused state
  std::array<double, 36> p_{};  // fused covariance
  double nis_ = 0.0;
};

}  // namespace gncsim
