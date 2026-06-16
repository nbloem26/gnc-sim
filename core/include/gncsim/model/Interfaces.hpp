// gnc-sim — model interface layer (issue #31). Abstract, pure-virtual interfaces for each stage of
// the per-step GNC pipeline. Concrete models (PN/APN guidance, alpha-beta/EKF navigation, radar/IR
// track sensors, 3DOF/6DOF dynamics, flat/round environment, ballistic/weave threats) implement
// these and are resolved by config string through the ModelRegistry (see Registry.hpp).
//
// This is a thin abstraction over the existing numerics: every concrete implementation is an
// adapter that delegates to the already-validated code in gnc/, sensors/, dynamics/, env/, aero/.
// No numerics live here. The interfaces exist so the Runner orchestrates resolved models rather
// than hard-wiring them, and so trade studies become config sweeps.
//
// Pure: no file I/O, no globals. Determinism is preserved because the adapters call the same
// functions, in the same order, with the same RNG, as the original hard-wired loop.
#pragma once

#include <vector>

#include "gncsim/core/Config.hpp"
#include "gncsim/core/Rng.hpp"
#include "gncsim/core/Types.hpp"
#include "gncsim/env/Environment.hpp"
#include "gncsim/gnc/Gnc.hpp"
#include "gncsim/gnc/TargetTrackEkf.hpp"
#include "gncsim/math/Vector3.hpp"

namespace gncsim {

// ---------------------------------------------------------------------------------------------
// Guidance
// ---------------------------------------------------------------------------------------------

// Per-step inputs the guidance law needs beyond the engagement geometry. `a_target_est` is the
// runner-level estimated target acceleration (used only by the augmented law; ignored by PN).
struct GuidanceState {
  Vector3 a_target_est;  // estimated target acceleration [m/s^2] (APN feedforward / ZEM)
  double tgo_s = 0.0;    // estimated time-to-go to intercept [s] (ZEM/ZEV); 0 = compute from geom
};

// Maps engagement geometry to a commanded acceleration [m/s^2] (magnitude-limited internally).
struct IGuidance {
  virtual ~IGuidance() = default;
  virtual Vector3 command(const Engagement& e, const GuidanceState& gs) const = 0;
};

// ---------------------------------------------------------------------------------------------
// Navigation
// ---------------------------------------------------------------------------------------------

// One navigation measurement of the relative (target - vehicle) state. Both representations are
// supplied so each filter consumes what it needs without the Runner branching on filter type:
//   - alpha-beta uses `measured_rel_pos` directly;
//   - the EKF uses the nonlinear (az, el, range) triple.
// The Runner builds these (drawing any sensor noise from the run RNG, in the original order) and
// hands them to the navigator, keeping the byte-identical RNG draw order.
struct NavMeasurement {
  Vector3 measured_rel_pos;  // reconstructed relative position [m] (alpha-beta channel)
  double az = 0.0;           // azimuth   = atan2(rel_y, rel_x)          [rad] (EKF channel)
  double el = 0.0;           // elevation = atan2(rel_z, hypot(rel_x,y)) [rad] (EKF channel)
  double range = 0.0;        // |rel_pos|                                 [m]  (EKF channel)
};

// Relative-state navigation filter (target - vehicle, world frame).
struct INavigator {
  virtual ~INavigator() = default;

  // Time update with the vehicle's achieved acceleration as a known control input. The alpha-beta
  // tracker has no predict step (no-op); the EKF advances its nearly-constant-velocity model.
  virtual void predict(const Vector3& a_vehicle) = 0;

  // Measurement update.
  virtual void update(const NavMeasurement& z) = 0;

  virtual Vector3 relPos() const = 0;
  virtual Vector3 relVel() const = 0;
  virtual double nis() const = 0;  // 0 for filters without an innovation (alpha-beta)
};

// ---------------------------------------------------------------------------------------------
// Sensor (external track sensor: radar / IR)
// ---------------------------------------------------------------------------------------------

// One detection look from a sensor (issue #39). A plain measurement sensor (radar/ir) always
// "detects" and fills `z` from measure(); a phenomenology sensor (radar_pheno/ir_pheno) runs a
// CA-CFAR front-end and may report detected=false (missed look — the tracker then coasts). snr_db
// is a diagnostic (NaN-free) of the cell-under-test signal strength; 0 for non-phenomenology
// sensors.
struct SensorDetection {
  bool detected = true;
  std::vector<double> z;  // measurement vector (radar [az,el,range,range_rate] / ir [az,el])
  double snr_db = 0.0;
};

// One fixed external sensor (ground radar / space IR) in the multi-tracker fusion path. Synthesizes
// a noisy measurement of the target's absolute state from the sensor's fixed position, drawing
// Gaussian noise from the run RNG (Box-Muller — parity-preserving). Radar -> [az,el,range,
// range_rate]; IR -> [az,el].
struct ISensor {
  virtual ~ISensor() = default;
  virtual std::vector<double> measure(const Vector3& tgt_pos, const Vector3& tgt_vel,
                                      Rng& rng) const = 0;
  virtual const TrackSensor& spec() const = 0;  // sensor parameters (type, position, noise sigmas)

  // Signal->detection look. The default (plain measurement sensors) always detects and delegates to
  // measure(), so its RNG draw order is unchanged. Phenomenology sensors override this to gate the
  // measurement on a CA-CFAR detection. Returning detected=false means "no measurement this look".
  virtual SensorDetection detect(const Vector3& tgt_pos, const Vector3& tgt_vel, Rng& rng) const {
    return SensorDetection{true, measure(tgt_pos, tgt_vel, rng), 0.0};
  }
};

// ---------------------------------------------------------------------------------------------
// Dynamics
// ---------------------------------------------------------------------------------------------

// Advance an entity's state one fixed step under the given world-frame force (excludes gravity),
// body moment (6DOF only), and gravity acceleration.
struct IDynamics {
  virtual ~IDynamics() = default;
  virtual EntityState step(const EntityState& s, const Vector3& force_world,
                           const Vector3& moment_body, const Vector3& gravity, double dt) const = 0;
  virtual bool is6dof() const = 0;
};

// ---------------------------------------------------------------------------------------------
// Environment
// ---------------------------------------------------------------------------------------------

// Gravity + atmosphere for the flat-Earth path.
struct IEnvironment {
  virtual ~IEnvironment() = default;
  virtual Vector3 gravity(const Vector3& pos) const = 0;
  virtual AtmSample atmosphere(double altitude_m) const = 0;
};

// ---------------------------------------------------------------------------------------------
// Threat (target propagation)
// ---------------------------------------------------------------------------------------------

// Target maneuver acceleration at time t (world frame). The Runner integrates the threat with this
// acceleration, preserving the original Euler step. "constant" -> zero; "weave" -> sinusoidal.
struct IThreat {
  virtual ~IThreat() = default;
  virtual Vector3 accel(const EntityState& tgt, double t) const = 0;
};

}  // namespace gncsim
