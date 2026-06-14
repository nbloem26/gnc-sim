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

// Truncated zonal spherical-harmonic ("EGM") gravity selection (issue #41). Opt-in on the round
// path. The `j2` flag below (kept for the legacy round path) maps onto include_j2; when no higher
// zonals are requested the field reduces EXACTLY to the existing central+J2 model. Coefficients are
// cited in core/src/env/EnvFidelity.cpp.
struct GravityFidelityConfig {
  bool include_j2 = false;  // J2 oblateness term
  bool include_j3 = false;  // J3 pear-shape (odd zonal)
  bool include_j4 = false;  // J4 zonal
};

// Parameterized wind profile (issue #41). Opt-in on the round path. Disabled => no wind, drag is
// computed against the co-rotating atmosphere exactly as before.
struct WindConfig {
  bool enabled = false;
  double surface_mps = 0.0;       // wind speed at the ground [m/s]
  double jet_mps = 0.0;           // peak wind speed at the jet altitude [m/s]
  double jet_alt_m = 10000.0;     // altitude of the wind maximum [m]
  double decay_scale_m = 8000.0;  // exponential decay scale above the jet [m]
  double dir_deg = 0.0;           // wind heading, from East toward North (ENU convention) [deg]
};

struct EnvConfig {
  double g0 = 9.80665;                // surface gravity [m/s^2]
  bool altitude_dependent_g = false;  // inverse-square falloff if true (flat mode)
  bool atmosphere = true;             // apply USSA76 drag if true
  std::string frame = "flat";         // "flat" (default, flat-Earth ENU) | "round" (WGS-84 ECI)
  bool j2 = false;                    // add J2 oblateness term to round-mode central gravity

  // --- High-fidelity environment (issue #41). Opt-in, additive; all default to the existing
  // round-mode behaviour so flat + current round runs are byte-identical. ---
  std::string gravity_model = "central";  // "central" (central+optional J2) | "egm" (J2..J4 zonals)
  std::string atmosphere_model = "ussa76";  // "ussa76" (<=86 km) | "extended" (USSA76 + >86 km ext)
  bool rotating_ecef = false;     // round path: integrate in rotating ECEF (Coriolis+centrifugal)
                                  // instead of ECI. Physically equivalent; alternative formulation.
  GravityFidelityConfig gravity;  // zonal selection when gravity_model == "egm"
  WindConfig wind;                // wind profile (round path drag)
};

struct AeroConfig {
  double ref_area = 0.05;                      // aerodynamic reference area [m^2]
  double cd0 = 0.3;                            // fallback drag coeff if table empty
  std::vector<std::array<double, 2>> cd_mach;  // (mach, Cd) breakpoints, ascending mach
  double cn_alpha = 12.0;                      // normal-force coeff slope [1/rad] (6DOF)

  // --- High-fidelity 6DOF aero (model == "6dof_hifi", issue #35). Opt-in: these are consulted
  // only on the hi-fi path. Tables are interpolated over angle-of-attack [rad] and Mach exactly
  // like cd_mach. Empty tables fall back to the linear slope/scalar fields below. ---
  double ref_length = 0.15;  // aerodynamic reference length (diameter) [m]; moment normalization

  // Normal-force coefficient Cn(alpha, mach) and pitch/yaw moment coefficient Cm(alpha, mach).
  // Each table is a list of (alpha_rad, mach, coeff) rows. When non-empty it supersedes the linear
  // cn_alpha / cm_alpha slopes. Bilinear-interpolated, clamped at the table edges.
  std::vector<std::array<double, 3>> cn_table;  // (alpha, mach, Cn)
  std::vector<std::array<double, 3>> cm_table;  // (alpha, mach, Cm) about the reference (CP-CG)

  double cm_alpha = -8.0;  // static pitch-moment slope [1/rad] (negative => statically stable)
  double cm_q = -120.0;    // pitch/yaw rate damping coefficient Cmq [1/rad] (negative => damping)
  double cl_p = -1.0;      // roll rate damping coefficient Clp [1/rad] (negative => damping)
};

struct VehicleConfig {
  Vector3 pos0{0.0, 0.0, 0.0};         // initial ENU position [m]
  double launch_speed = 600.0;         // [m/s]
  double launch_elevation_deg = 45.0;  // above horizon
  double launch_azimuth_deg = 0.0;     // from East toward North
  double mass0 = 22.0;                 // [kg]
  double inertia = 1.2;                // scalar moment of inertia proxy [kg*m^2] (6DOF)

  // --- Full inertia tensor (model == "6dof_hifi", issue #35). Opt-in: the legacy "6dof" path uses
  // the scalar `inertia` above unchanged. When any product of inertia is set, or all three
  // principal moments are given, the hi-fi rotational EOM uses the full 3x3 tensor with the
  // gyroscopic coupling -omega x (I omega). Defaults below reproduce the scalar proxy: all axes
  // equal to `inertia`, no products. A value <= 0 for a principal moment means "use `inertia`". ---
  double ixx = -1.0;  // body-x (roll) moment of inertia [kg*m^2]; <=0 => fall back to `inertia`
  double iyy = -1.0;  // body-y (pitch) moment of inertia [kg*m^2]
  double izz = -1.0;  // body-z (yaw) moment of inertia [kg*m^2]
  double ixy = 0.0;   // products of inertia [kg*m^2]
  double ixz = 0.0;
  double iyz = 0.0;
};

// First-order fin-actuator dynamics + limits (model == "6dof_hifi", issue #35). Opt-in. The
// autopilot's commanded body moment is allocated to a deflection command; each actuator channel
// follows it through a first-order lag (time constant `tau`), with the deflection rate and travel
// hard-limited. The realized deflection generates the control moment fed to the rotational EOM.
struct ActuatorConfig {
  double tau = 0.02;               // first-order actuator time constant [s]
  double rate_limit = 6.0;         // max deflection rate [rad/s]
  double deflection_limit = 0.35;  // max travel per channel [rad]
  double effectiveness = 60.0;     // control moment per unit deflection [N*m/rad] (allocation gain)
};

struct PropulsionConfig {
  double thrust = 0.0;           // boost thrust [N] (0 = unpowered default)
  double burn_time = 0.0;        // [s]
  double propellant_mass = 0.0;  // [kg], burns linearly over burn_time
  double boost_ref_area = 0.0;   // booster drag area during boost [m^2] (0 = use aero.ref_area)
  double stage_time = 0.0;       // booster jettison time [s] (0 = no staging)
  double stage_mass_drop = 0.0;  // mass dropped at staging [kg]
};

// Optimal / predictive guidance (guidance.law == "zemzev", issue #40). Opt-in, additive; the
// existing pronav/apn/none laws are untouched. ZEM/ZEV is the energy-optimal terminal law:
//   a_cmd = (N_zem / tgo^2) * ZEM  +  (N_zev / tgo) * ZEV
// where ZEM (zero-effort miss) is the predicted relative MISS at intercept if neither side
// accelerates further, and ZEV (zero-effort velocity error) is the predicted relative VELOCITY
// error at intercept versus a desired closing velocity. With N_zem = 3 and the ZEV term off
// (midcourse only), the law reduces to the classical optimal-PN form (Zarchan ch. 8). The ZEM
// uses the estimated target acceleration, so it intercepts a constant-accel target with zero
// steady-state miss (like APN, derived from the optimal-control solution rather than bolted on).
//
// Midcourse -> terminal handover: while range > handover_range_m the law runs in MIDCOURSE mode
// (the ZEV term is active, steering toward a desired intercept geometry); at/within
// handover_range_m it switches to pure TERMINAL ZEM homing. The switch is continuous: the ZEV
// weight is faded to zero over a handover_blend_m band ABOVE the switch range, so there is no
// command discontinuity at the boundary (verified in optimal_guidance_test.cpp).
struct ZemZevConfig {
  double n_zem = 3.0;                // ZEM optimal-guidance gain (3 = energy-optimal)
  double n_zev = 0.0;                // ZEV (velocity-shaping) gain; 0 disables the midcourse term
  double desired_closing_mps = 0.0;  // desired closing speed for the ZEV term [m/s]; 0 = current
  double tgo_floor_s = 0.05;      // smallest time-to-go used in the 1/tgo^2 law [s] (avoids blowup)
  double handover_range_m = 0.0;  // midcourse->terminal switch range [m]; 0 = always terminal
  double handover_blend_m = 500.0;  // ZEV-fade band above the switch range [m] (continuity)
};

// Reaction-control / divert actuation (guidance.divert.enabled, issue #40). Exo-atmospheric
// interceptors steer with discrete reaction-control thrusters (a divert/ACS stage) rather than
// aero fins. Opt-in: when enabled the realized guidance acceleration is the RCS divert command,
// hard magnitude-limited to divert_limit_mps2 (the thruster authority). Distinct from the aero
// max_accel cap, which is the airframe lift limit. Disabled by default -> the legacy aero path is
// byte-identical.
struct DivertConfig {
  bool enabled = false;
  double divert_limit_mps2 = 100.0;  // RCS divert thruster authority [m/s^2]
};

struct GuidanceConfig {
  std::string law = "pronav";   // "pronav" | "apn" | "none" | "zemzev"
  double nav_constant = 3.0;    // PN gain N
  double max_accel = 300.0;     // accel command limit [m/s^2]
  double time_constant = 0.0;   // guidance/autopilot lag [s]; 0 = ideal instantaneous
  double apn_filter_tau = 0.1;  // APN target-accel estimator low-pass time constant [s]
  ZemZevConfig zemzev;          // optimal ZEM/ZEV law parameters (law == "zemzev")
  DivertConfig divert;          // reaction-control divert actuation (issue #40)
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
  std::string filter = "alpha_beta";  // "alpha_beta" | "ekf" | "imm"
  double process_accel_psd = 50.0;    // EKF target-accel PSD q [m^2/s^3] per axis
  double range_white = 5.0;           // range measurement noise std [m] (EKF range channel)
  // IMM (nav.filter == "imm") model bank: a constant-velocity model (low PSD) and a maneuver model
  // (high PSD), mixed by mode probability. Defaults give a quiescent vs ~hard-maneuver pair.
  double imm_q_cv = 0.5;      // constant-velocity model process-accel PSD q [m^2/s^3]
  double imm_q_man = 3000.0;  // maneuver model process-accel PSD q [m^2/s^3]
  double imm_p_stay = 0.999;  // Markov mode self-transition probability (mode stickiness)
};

// One boosting rocket stage of a multi-stage ICBM threat (issue #42). Stages burn in order; each
// adds thrust along the body/velocity axis for burn_time_s, drops its spent dry mass at the end of
// its burn (staging event), then the next stage ignites. After the final stage the threat coasts
// ballistically (gravity only) through midcourse — the characteristic high-apogee lofted arc.
struct IcbmStage {
  double thrust_n = 0.0;            // stage thrust [N]
  double burn_time_s = 0.0;         // stage burn duration [s]
  double propellant_mass_kg = 0.0;  // propellant burned linearly over burn_time_s [kg]
  double dry_mass_kg = 0.0;         // stage structural (dry) mass dropped at staging [kg]
};

// Multi-stage boosting ICBM threat (target.maneuver == "icbm", issue #42). The threat thrusts
// through its stage stack, drops mass at each staging event, then flies a ballistic midcourse arc
// under gravity. Gravity is supplied by the threat itself (the flat-Earth target propagation does
// not otherwise gravitate the target). Defaults are empty so an unconfigured icbm threat is inert.
struct IcbmConfig {
  std::vector<IcbmStage> stages;  // ordered boosting stages
  double payload_mass_kg = 0.0;   // RV/payload mass carried through midcourse [kg]
};

// Hypersonic glide vehicle threat (target.maneuver == "hgv", issue #42). After a ballistic entry
// the vehicle develops aerodynamic lift (lift-to-drag ratio ld_ratio) in the vertical plane,
// producing the characteristic skip-glide oscillation: it pulls up out of the dense atmosphere,
// arcs over, re-enters, and skips again. Lift acts perpendicular to velocity in the vertical plane;
// drag opposes velocity. A crude exponential atmosphere drives the aero so the threat is fully
// self-contained (no dependence on the interceptor-side aero/atmosphere blocks).
struct HgvConfig {
  double ld_ratio = 2.5;            // lift-to-drag ratio L/D (dimensionless)
  double ballistic_coeff = 4000.0;  // ballistic coefficient m/(Cd*A) [kg/m^2] (drag scaling)
  double rho0_kgpm3 = 1.225;        // sea-level air density [kg/m^3]
  double scale_height_m = 7200.0;   // exponential-atmosphere scale height [m]
  double pull_up_alt_m = 40000.0;   // altitude below which lift switches on (glide regime) [m]
};

// Reentry-vehicle-with-penaids threat (target.maneuver == "rv_penaids", issue #42). The true RV is
// ballistic (gravity only). At deploy_time_s a bus dispenses penaids/decoys as a kinematic spread
// about the RV; the penaids are scored against the true RV by the discrimination stack. The threat
// itself only governs the RV's acceleration (ballistic); penaid kinematics + scoring live in the
// RvPenaids helper so they feed issue #6's discriminator. Defaults: no penaids (inert beyond the
// ballistic RV).
struct RvPenaidsConfig {
  int penaid_count = 0;            // number of penaids/decoys dispensed
  double deploy_time_s = 0.0;      // bus dispense time [s]
  double deploy_dv_mps = 5.0;      // characteristic dispense delta-v spread [m/s]
  double penaid_decel_mps2 = 0.5;  // extra atmospheric deceleration penaids shed (lighter) [m/s^2]
};

struct TargetConfig {
  Vector3 pos0{8000.0, 0.0, 3000.0};
  Vector3 vel0{-250.0, 0.0, 0.0};
  // "constant" | "weave" | "icbm" | "hgv" | "rv_penaids"
  std::string maneuver = "constant";
  double maneuver_g = 3.0;          // lateral accel for weave [g]
  double maneuver_freq = 0.4;       // [Hz]
  double maneuver_phase_deg = 0.0;  // weave phase offset [deg] (Monte Carlo randomizes this)

  // --- Threat suite (issue #42). Opt-in: consulted only by the matching maneuver string, so the
  // constant/weave paths are untouched. ---
  IcbmConfig icbm;     // target.maneuver == "icbm"
  HgvConfig hgv;       // target.maneuver == "hgv"
  RvPenaidsConfig rv;  // target.maneuver == "rv_penaids"
};

// CA-CFAR detector parameters (issue #39). Shared by radar/IR phenomenology sensors. The
// false-alarm rate is held constant by setting the threshold from `num_ref_cells` reference cells:
//   alpha = N * (pfa^(-1/N) - 1).  Larger N -> lower CFAR loss (alpha -> -ln(pfa)).
struct CfarConfig {
  double pfa = 1.0e-4;     // design probability of false alarm (per CFAR cell-under-test)
  int num_ref_cells = 24;  // number of reference (training) cells averaged for the noise estimate
};

// Radar phenomenology (issue #39): turns geometry into an SNR via the range equation, with Swerling
// RCS fluctuation, clutter, and barrage-noise ECM. SNR is anchored: at `range_ref_m` a target of
// `rcs_ref_m2` yields `snr_ref_db`; SNR then scales as rcs / R^4. Used only by type "radar_pheno".
struct RadarPhenomenologyConfig {
  double rcs_mean_m2 = 1.0;    // mean target radar cross section [m^2]
  int swerling = 1;            // Swerling case: 0 (non-fluctuating) | 1 | 2 | 3 | 4
  double range_ref_m = 1.0e4;  // reference range at which snr_ref_db is anchored [m]
  double rcs_ref_m2 = 1.0;     // reference RCS for the SNR anchor [m^2]
  double snr_ref_db = 20.0;    // single-pulse SNR of an rcs_ref target at range_ref [dB]
  double clutter_cnr_db =
      -100.0;                     // clutter-to-noise ratio raising the noise floor [dB] (off: <-90)
  double jammer_jnr_db = -100.0;  // barrage-noise jammer-to-noise ratio [dB] (off when <-90)
};

// IR phenomenology (issue #39): contrast SNR from NETD and atmospheric transmission. The target's
// apparent intensity falls as 1/R^2 and is attenuated by Beer-Lambert exp(-beta R); NETD sets the
// noise-equivalent contrast. Detection SNR also drives the angular measurement noise. Type
// "ir_pheno".
struct IrPhenomenologyConfig {
  double netd_k = 0.05;            // noise-equivalent temperature difference [K]
  double target_contrast_k = 2.0;  // target-vs-background apparent ΔT contrast at range_ref [K]
  double range_ref_m = 1.0e4;      // reference range for the contrast anchor [m]
  double atm_extinction_per_m = 5.0e-5;  // Beer-Lambert extinction coefficient beta [1/m]
  double theta_resolution_rad = 1.0e-3;  // pixel/IFOV angular resolution [rad]
  double centroid_gain = 4.0;            // centroiding gain k in sigma = theta_res/(k*sqrt(SNR))
};

// One fixed external sensor in the multi-tracker fusion path (issue #5). Type "radar" yields
// [az, el, range, range_rate]; type "ir" is angles-only [az, el]. Position is fixed in ENU [m].
// Types "radar_pheno" / "ir_pheno" (issue #39) add the signal->detection front-end: the same az/el
// [/range/range_rate] measurement, but gated by a CA-CFAR detection (Pd from the modelled SNR), so
// the tracker consumes detections — not perfect-with-noise truth — and coasts on a missed look.
struct TrackerSensorConfig {
  std::string type = "radar";  // "radar" | "ir" | "radar_pheno" | "ir_pheno"
  Vector3 pos;                 // sensor location, ENU [m] (e.g. ground site at origin, space high)
  double sigma_az = 1.0e-3;    // azimuth noise std [rad]   (radar + ir)
  double sigma_el = 1.0e-3;    // elevation noise std [rad] (radar + ir)
  double sigma_range = 10.0;   // range noise std [m]        (radar only)
  double sigma_range_rate = 1.0;  // range-rate noise std [m/s] (radar only)

  // --- Phenomenology (issue #39); consulted only by the *_pheno types, additive/opt-in. ---
  CfarConfig cfar;                 // CA-CFAR detector (Pfa, reference cells)
  RadarPhenomenologyConfig radar;  // radar SNR / RCS / clutter / ECM (type "radar_pheno")
  IrPhenomenologyConfig ir_pheno;  // IR NETD / atmospheric transmission (type "ir_pheno")
};

// Multi-target data association (issue #38). Opt-in second-generation tracking mode, selected by
// trackers.association.mode == "jpda". When the default mode ("none") is in effect the Runner uses
// the byte-identical single-target fusion path of issue #5. With "jpda", each sensor look produces
// a SET of detections — the true-target return (when CFAR-detected), one return per decoy /
// closely- spaced object, and Poisson clutter false alarms — and the Runner runs JPDA over the
// confirmed track: gate the detections, weight them by likelihood + a no-detection/clutter
// hypothesis, and fold the probabilistic combination into the TargetTrackEkf (PDA update). A
// track-lifecycle state machine (M-of-N confirmation, miss-run deletion) governs the track, and
// per-sensor tracks are fused track-to-track by Covariance Intersection.
struct TrackAssociationConfig {
  std::string mode = "none";            // "none" (issue #5 single-target fusion) | "jpda"
  double prob_detect = 0.9;             // P_D used by the JPDA association weights (dimensionless)
  double gate_chi2 = 16.0;              // validation-gate threshold on measurement NIS (chi-square)
  double clutter_density = 1.0e-4;      // clutter spatial density in measurement space (lambda)
  double clutter_rate = 2.0;            // mean Poisson clutter false alarms per sensor per look
  double clutter_az_spread_rad = 0.02;  // angular spread of clutter about the track (az/el) [rad]
  double clutter_range_spread_m = 300.0;  // range spread of clutter about the track [m]
  int confirm_m = 3;                      // M of the M-of-N confirmation rule
  int confirm_n = 5;                      // N of the M-of-N confirmation rule
  int delete_misses = 6;  // delete the track after this many consecutive missed looks

  // Initial track 1-sigma uncertainty for the JPDA bootstrap. A TIGHT, accuracy-consistent gate
  // from the first look is what lets the associator reject the surrounding decoys/clutter instead
  // of being pulled to the cluster centroid; the issue-#5 single-target path keeps its wide cue.
  double init_pos_sigma_m = 30.0;    // initial position 1-sigma [m]
  double init_vel_sigma_mps = 30.0;  // initial velocity 1-sigma [m/s]

  // Closely-spaced-object scene (the decoys the associator must reject). Reuses the decoy kinematic
  // spread; these are pure geometry (no feature signature — the associator is kinematic, not a
  // feature discriminator). Opt-in: 0 closely-spaced objects -> only true target + clutter.
  int num_cso = 3;  // number of closely-spaced objects (decoys) around the true target
  double cso_separation_m = 80.0;  // characteristic cluster spread of the CSOs about the target [m]
};

// Multi-sensor target-track fusion (issue #5). Opt-in: when disabled (default) nothing changes and
// the default single-seeker navigation path is byte-identical. When enabled, the Runner builds a
// TargetTrackEkf, synthesizes a noisy measurement from each sensor each step, fuses them
// sequentially into one absolute target-state estimate, and guidance uses (track_est - vehicle).
struct TrackersConfig {
  bool enabled = false;
  double process_psd = 50.0;  // target-accel PSD q [m^2/s^3] per axis (nearly-constant-velocity Q)
  std::vector<TrackerSensorConfig> sensors;
  TrackAssociationConfig association;  // multi-target data association mode (issue #38)
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

// One threat (raider) in a many-on-many engagement (issue #45). Each is a full target spec:
// its launch state + maneuver, plus a release time so a raid can arrive staggered. The campaign
// runs each interceptor-vs-threat pairing through the SAME per-engagement physics (runSimulation),
// so a threat carries everything TargetConfig does — only the fields most useful per-raider are
// surfaced here; everything else inherits the base `target` block.
struct ManyThreatSpec {
  Vector3 pos0_m{12000.0, 0.0, 4000.0};  // threat initial ENU position [m]
  Vector3 vel0_mps{-300.0, 0.0, -40.0};  // threat initial ENU velocity [m/s]
  std::string maneuver = "constant";     // reuse TargetConfig maneuver strings
  double maneuver_g = 0.0;               // lateral accel for weave [g]
  double maneuver_freq_hz = 0.4;         // weave frequency [Hz]
  double release_time_s = 0.0;  // raid stagger: when this raider "arrives" [s] (bookkeeping)
  double value = 1.0;           // target value weight for WTA (defended-asset lethality)
};

// One interceptor (weapon) in a many-on-many engagement (issue #45). Each is a launch-site +
// launch-kinematics spec; the rest of the airframe / guidance / aero inherits the base config so a
// salvo of identical interceptors is just N entries with the same numbers. An interceptor is a
// single round: once expended (assigned + fired) it is gone.
struct ManyInterceptorSpec {
  Vector3 pos0_m{0.0, 0.0, 0.0};       // launch-site ENU position [m]
  double launch_speed_mps = 1100.0;    // [m/s]
  double launch_elevation_deg = 42.0;  // above horizon
  double launch_azimuth_deg = 0.0;     // from East toward North
};

// Many-on-many engagement campaign (issue #45). Opt-in, additive: disabled by default so the
// single-engagement runSimulation path is byte-identical and never touched. When enabled, the
// campaign driver (core/src/scenario/ManyOnMany.cpp) forms an interceptor-vs-threat pairing matrix,
// scores each pairing with the SAME per-engagement physics (runSimulation -> miss distance -> a
// lethality P(kill) model), solves a deterministic weapon-target assignment (WTA) maximizing
// expected kills, and plays out a doctrine:
//   - "salvo"             : shots_per_threat interceptors committed to each threat at once.
//   - "shoot_look_shoot"  : engage a wave, ASSESS outcomes, RE-ENGAGE only the survivors, up to
//                           max_waves waves (each wave is one shot per surviving threat).
//   - "raid"              : many threats (possibly staggered by release_time_s) defended against a
//                           finite interceptor inventory via WTA; leakage = threats that survive.
// The driver rolls up campaign metrics: leakage (threats surviving), interceptors expended, and the
// probability of raid annihilation (all threats killed). Deterministic given seed (project Rng; the
// shoot-look-shoot kill assessment draws from a seeded Rng, NO std distributions).
struct ManyOnManyConfig {
  bool enabled = false;
  std::string doctrine = "salvo";     // "salvo" | "shoot_look_shoot" | "raid"
  std::string wta_method = "greedy";  // "greedy" | "auction" (both maximize expected kills)

  std::vector<ManyInterceptorSpec> interceptors;  // weapon inventory
  std::vector<ManyThreatSpec> threats;            // raiders

  int shots_per_threat = 1;  // "salvo": interceptors committed per threat
  int max_waves = 3;         // "shoot_look_shoot": max re-engagement waves

  // Lethality model: a single-shot P(kill) from the pairing's analytic CPA miss distance,
  //   Pssk = pk_max * exp(-0.5 * (miss / pk_sigma_m)^2),
  // a standard Gaussian (Carleton) damage function. A larger pk_sigma_m models a bigger lethal
  // radius / warhead. The salvo cumulative kill probability against one threat with k independent
  // shots is 1 - prod(1 - Pssk_i).
  double pk_sigma_m = 5.0;  // lethality 1-sigma radius [m]
  double pk_max = 0.95;     // ceiling single-shot P(kill) at zero miss (reliability < 1)

  // Monte Carlo campaign trials: P(raid annihilation) and mean leakage are estimated over
  // num_trials stochastic realizations (each pairing's kill is a Bernoulli draw at its Pssk). 0 or
  // 1
  // => a single deterministic expected-value rollup (no kill sampling). Deterministic given seed.
  int num_trials = 0;
};

struct SimConfig {
  std::string scenario = "homing";
  std::string model = "3dof";  // "3dof" | "6dof" | "6dof_hifi" (issue #35)
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
  ActuatorConfig actuator;
  SensorConfig sensors;
  NavConfig nav;
  TargetConfig target;
  TrackersConfig trackers;
  DecoysConfig decoys;
  CueingConfig cueing;
  MonteCarloConfig monte_carlo;
  ManyOnManyConfig many_on_many;
};

// Parse a JSON document (tolerant: missing keys fall back to struct defaults).
// Throws std::runtime_error on malformed JSON.
SimConfig loadConfigFromString(const std::string& json_text);

// Compute initial ENU velocity from launch speed/elevation/azimuth.
Vector3 launchVelocity(const VehicleConfig& v);

}  // namespace gncsim
