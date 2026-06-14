# Model credibility — per-model documentation

Every **shipped model** in gnc-sim (the set resolved by
[`core/src/model/Registry.cpp`](../core/src/model/Registry.cpp)) gets a short page here:
its assumptions, governing equations, validity limits, and references. This is the
"what does this model actually claim, and where does it stop being valid" reference.

Each model's **evidence** (the GoogleTest benchmark, the golden-run baseline entry, and/or
the analytic check in `postproc/`) is catalogued in the companion
[**V&V matrix**](VNV_MATRIX.md). The matrix is cross-checked against this set of shipped
models by a pytest (`postproc/tests/test_vnv_matrix.py`) so the two cannot silently drift.

Models are selected from the config (see [DATA_CONTRACT.md](DATA_CONTRACT.md)):

| Family | Config key | Shipped values |
|---|---|---|
| Guidance | `guidance.law` | `pronav`, `apn`, `zemzev`, `none` |
| Navigation | `nav.filter` | `alpha_beta`, `ekf`, `imm` |
| Dynamics | `vehicle.model` | `3dof`, `6dof`, `6dof_hifi` |
| Sensor | `trackers[].type` | `radar`, `ir` |
| Environment | `env.frame` | `flat`, `round` |
| Threat | `target.maneuver` | `constant`, `weave`, `icbm`, `hgv`, `rv_penaids` |

> **Adding a model?** You must also add its page here **and** a row to the
> [V&V matrix](VNV_MATRIX.md) (see `AGENTS.md` → *Adding a model*). The consistency test
> fails the build if a shipped model has no matrix row, or a matrix row names a missing
> golden key.

---

## Guidance

### `pronav` — Proportional Navigation

- **Assumptions.** Point-mass kinematics; the seeker/navigator supplies the inertial
  line-of-sight (LOS) unit vector `û_LOS`, its rotation rate `Ω`, and the closing speed
  `V_c`. Command is issued perpendicular to the LOS. No lag in the guidance loop itself
  (actuator/airframe lag is modelled downstream in 6DOF).
- **Governing equation.** `a_cmd = N · V_c · (Ω × û_LOS)`, magnitude-limited to the
  vehicle's `max_accel`. `N` is the dimensionless navigation constant (`guidance.nav_constant`,
  typically 3–5).
- **Validity limits.** Optimal for a **non-maneuvering** target on a near-collision course.
  Degrades against a maneuvering target (no target-accel feedforward — use `apn`). The
  command saturates at `max_accel`; once saturated, miss distance grows. Assumes a usable
  closing geometry (`V_c > 0`); a receding target yields zero command.
- **References.** Zarchan, *Tactical and Strategic Missile Guidance*, 7th ed., ch. 2
  (PN derivation and the `N·V_c·Ω` form).

### `apn` — Augmented Proportional Navigation

- **Assumptions.** As `pronav`, plus an **estimate of target acceleration**
  `a_target_est` from the navigator (the EKF/IMM relative-state filter). The feedforward is
  applied only under the `apn` law.
- **Governing equation.** `a_cmd = N · V_c · (Ω × û_LOS) + (N/2) · a_target_est_perp`, with
  the augmentation term projected perpendicular to the LOS and the total again
  magnitude-limited.
- **Validity limits.** The augmentation only helps when the target-accel estimate is good;
  with a poor estimate it can be worse than plain PN. Same saturation and closing-geometry
  caveats as `pronav`.
- **References.** Zarchan, ch. 4 (Augmented PN); the `N/2` augmentation gain.

### `zemzev` — Optimal ZEM/ZEV (predictive) guidance

- **Assumptions.** Point-mass kinematics; the navigator supplies the relative position
  `r`, relative velocity `v` (target − vehicle), and an estimate of the target acceleration
  `a_T` (the same runner-level estimator the `apn` law uses). Time-to-go `t_go` is estimated
  as `range / V_c` (floored at `tgo_floor_s` so the `1/t_go²` term never blows up). The law is
  the closed-form solution of the linear-quadratic optimal-control intercept problem (minimum
  control energy to null the predicted miss), not a heuristic.
- **Governing equation.**
  `a_cmd = (N_zem / t_go²)·ZEM  +  w(range)·(N_zev / t_go)·ZEV`, magnitude-limited to
  `max_accel`, where
  - **ZEM** (zero-effort miss) = predicted relative position at intercept if neither side
    accelerates further: `ZEM = r + v·t_go + ½·a_T·t_go²`. With `N_zem = 3` this is the
    energy-optimal terminal law; the `½·a_T·t_go²` term folds in the target's maneuver, so a
    **constant-acceleration** target is intercepted with zero steady-state miss (the optimal
    analogue of the `apn` feedforward).
  - **ZEV** (zero-effort velocity error) = predicted relative velocity at intercept minus a
    desired closing velocity: `ZEV = (v + a_T·t_go) − v_des`. The `N_zev` term shapes the
    **midcourse** trajectory toward a desired intercept geometry (e.g. a desired closing
    speed); set `N_zev = 0` for a pure terminal-homing law.
  - **Midcourse → terminal handover.** `w(range)` is the handover weight: `1` while
    `range` is well above `handover_range_m` (midcourse, ZEV active), ramping **linearly to 0**
    across the `handover_blend_m` band just above the switch range, and `0` at/inside it (pure
    terminal ZEM). Because `w` is continuous in range, the command has **no discontinuity** at
    the switch (verified in `optimal_guidance_test::ZemZev.HandoverIsContinuous` /
    `TerminalPhaseIsPureZem`).
- **Divert / ACS actuation (`guidance.divert`).** Exo-atmospheric interceptors steer with
  reaction-control thrusters rather than aero fins. When `guidance.divert.enabled`, the
  realized guidance acceleration is the RCS divert command, **hard magnitude-limited to
  `divert_limit_mps2`** (the thruster authority) — distinct from the aero `max_accel` lift cap.
  Disabled by default; usable with any law but exercised here with `zemzev` for the
  exo-atmospheric case.
- **Validity limits.** Optimal under the linearized constant-`a_T`, known-`t_go` assumptions;
  a poor `t_go` or `a_T` estimate degrades it (the `tgo_floor_s` guard bounds the end-game
  gain). Like `pronav`/`apn` it commands only while closing (`V_c > 0`) and saturates at
  `max_accel` (or `divert_limit_mps2` under divert).
- **References.** Zarchan, *Tactical and Strategic Missile Guidance*, 7th ed., ch. 8
  (optimal guidance, the ZEM `N=3` form); Ben-Asher & Yaesh, *Advances in Missile Guidance
  Theory* (ZEM/ZEV and predictive/optimal laws).

### `none` — Unguided / ballistic

- **Assumptions.** No guidance command is ever issued (`a_cmd ≡ 0`). Used for ballistic
  rounds and projectiles whose trajectory is set purely by launch state + environment + aero.
- **Governing equation.** `a_cmd = 0`.
- **Validity limits.** Not a guidance law — a deliberate null. Trajectory accuracy is then
  entirely the dynamics/env/aero models' responsibility.
- **References.** n/a (null model).

---

## Navigation

### `alpha_beta` — Alpha-beta tracker

- **Assumptions.** A fixed-gain (α, β) filter on a **reconstructed relative position**
  measurement. No predict/innovation step; a constant-velocity target between updates.
- **Governing equation.** Position update `x̂ ← x̂ + α(z − x̂)`; velocity update
  `v̂ ← v̂ + (β/Δt)(z − x̂)`. Fixed gains, no covariance.
- **Validity limits.** Cheap and robust but **suboptimal**: fixed gains aren't tuned to the
  actual noise, and there is no statistical consistency (NIS is reported as 0). Lags a
  maneuvering target. Adequate for the default noise-free homing case; prefer `ekf`/`imm`
  with real sensor noise.
- **References.** Bar-Shalom, *Estimation with Applications to Tracking and Navigation*,
  §6 (steady-state / α-β filters).

### `ekf` — Relative-state Extended Kalman Filter

- **Assumptions.** A continuous-white-noise-acceleration (CWNA) process model; the vehicle
  acceleration is the known control input in the predict step. Measurements are the
  **nonlinear** azimuth/elevation/range triple, linearised about the current estimate.
- **Governing equations.** Predict `x̂⁻ = F x̂ + G a_vehicle`, `P⁻ = F P Fᵀ + Q(q_psd)`;
  update with measurement Jacobian `H`, gain `K = P⁻ Hᵀ S⁻¹`, `S = H P⁻ Hᵀ + R`. Reports the
  normalised innovation squared (NIS) for consistency checking.
- **Validity limits.** Single-model: assumes one motion model, so it lags a hard maneuver
  (the IMM addresses this). EKF linearisation assumes small per-step measurement nonlinearity.
  Range observability is weak for angles-only (IR) sensors.
- **References.** Bar-Shalom, §10 (EKF); the CWNA process model, §6.

### `imm` — Interacting Multiple Model filter

- **Assumptions.** A bank of two filters — a constant-velocity (CV) model and a maneuver
  (higher process-noise nearly-constant-velocity) model — mixed by mode probability with a
  Markov transition matrix (`imm_p_stay` self-transition). Same az/el/range measurement
  channel as the EKF.
- **Governing equations.** Mode mixing → per-mode EKF predict/update → likelihood-weighted
  mode-probability update → combined estimate as the probability-weighted mixture. Reports a
  mode-probability-weighted combined NIS.
- **Validity limits.** Best when the target alternates between benign and maneuvering flight
  (it picks up the maneuver mode); the extra filter costs compute. Tuning of `q_cv`, `q_man`,
  and `p_stay` matters. Two modes only.
- **References.** Bar-Shalom, §11 (IMM estimator); Blom & Bar-Shalom (1988), IMM algorithm.

---

## Dynamics

### `3dof` — Point-mass translational dynamics

- **Assumptions.** Three translational DOF; attitude is not modelled (the guidance accel is
  applied directly to the point mass). Fixed-step RK4 integration.
- **Governing equation.** `ẋ = v`, `v̇ = (F_world + m·g)/m`, where `F_world` carries thrust +
  aero drag + the applied guidance accel. Integrated with RK4 at fixed `dt`.
- **Validity limits.** No airframe lag, no angle-of-attack, no body rates — instantaneous
  acceleration response. Good for trajectory/guidance studies; not for fin/autopilot fidelity.
- **References.** Standard flat-/round-Earth point-mass EOM; Zarchan ch. 2.

### `6dof` — Rigid-body dynamics (scalar inertia)

- **Assumptions.** Six DOF: translation + attitude as a unit quaternion, with a **scalar**
  inertia (symmetric body). Body moment from the autopilot→fin chain drives attitude.
- **Governing equations.** Translational as 3DOF; rotational `ω̇ = M_body / I`,
  `q̇ = ½ q ⊗ ω`, with the quaternion renormalised each step. RK4.
- **Validity limits.** Scalar inertia ignores cross-axis (gyroscopic) coupling — fine for a
  roughly axisymmetric body but not for a body with distinct principal inertias. Aero moment
  tables are the autopilot's, not a full coupled tensor.
- **References.** Stevens & Lewis, *Aircraft Control and Simulation*, ch. 1–2 (rigid-body EOM);
  quaternion kinematics.

### `6dof_hifi` — High-fidelity rigid-body (full inertia tensor)

- **Assumptions.** Full **inertia tensor** with gyroscopic coupling in the rotational EOM
  (issue #35). Aero-moment tables (Cn/Cm) and actuator (fin rate/travel) dynamics are
  assembled by the Runner per step; this model owns the inertia tensor and the coupled
  integration.
- **Governing equations.** Euler's rigid-body equation
  `I ω̇ = M_body − ω × (I ω)`, integrated with RK4; quaternion attitude as in `6dof`. Conserves
  rotational energy and angular momentum in the torque-free limit (a verified benchmark).
- **Validity limits.** Highest-fidelity shipped airframe model; correspondingly the most
  config to specify (inertia tensor, Cn/Cm tables, actuator limits). Static-margin and
  actuator-rate assumptions in the moment block still apply.
- **References.** Stevens & Lewis ch. 1–2; Euler's equations for a rigid body with full
  inertia tensor.

---

## Sensors

### `radar` — Ground/track radar

- **Assumptions.** A fixed-position track sensor measuring **azimuth, elevation, range, and
  range-rate** to the target, each corrupted by independent zero-mean Gaussian noise
  (`sigma_az`, `sigma_el`, `sigma_range`, `sigma_range_rate`). Noise drawn from the run RNG
  via Box-Muller, in az,el,range,range-rate order (determinism-preserving).
- **Governing equations.** `az = atan2(Δy, Δx) + n_az`, `el = atan2(Δz, √(Δx²+Δy²)) + n_el`,
  `range = |Δ| + n_r`, `range_rate = (Δ·v)/|Δ| + n_ṙ`, with `Δ = tgt_pos − sensor_pos`.
- **Validity limits.** Full-state (range-resolved) → strong observability when fused. Gaussian
  white-noise model only — no clutter, multipath, glint, or detection dropouts. Fixed mounting.
- **References.** Skolnik, *Introduction to Radar Systems* (measurement model); Bar-Shalom §3
  (Cartesian↔polar measurement geometry).

### `ir` — Infrared (angles-only) seeker/track sensor

- **Assumptions.** As `radar` but **angles-only**: it reports azimuth + elevation, with no
  range or range-rate. Same Gaussian-noise model on the two angle channels.
- **Governing equations.** `az = atan2(Δy, Δx) + n_az`, `el = atan2(Δz, √(Δx²+Δy²)) + n_el`.
- **Validity limits.** **Weak range observability** — range is only recoverable through motion
  (fusion with a range-bearing sensor, or own-ship maneuver). Passive: no range/range-rate. The
  seeker noise model (white + range-dependent glint) is characterised separately for the homing
  seeker; this track-sensor channel is the white-noise az/el model.
- **References.** Bar-Shalom §3 (angles-only / bearings-only observability); IR seeker glint
  models.

---

## Environment

### `flat` — Flat-Earth gravity + USSA76 atmosphere

- **Assumptions.** Flat-Earth gravity (constant or inverse-square falloff with altitude per
  `GravityModel`) plus the **U.S. Standard Atmosphere 1976** for density/pressure/temperature.
- **Governing equations.** `g(h)` from `GravityModel`; `(ρ, p, T, a_sound)` from the piecewise
  USSA76 layers (the same model the Python `atmosphere.py` twin validates against).
- **Validity limits.** Local/short-range engagements where Earth curvature and rotation are
  negligible. USSA76 is valid ~0–86 km; clamps outside. No winds, no day/night variation.
- **References.** *U.S. Standard Atmosphere, 1976* (NOAA/NASA/USAF); flat-Earth point-mass
  gravity.

### `round` — Round-Earth ECI/ECEF + WGS-84

- **Assumptions.** Round, rotating Earth (issue #4): central + optional J2 gravity in an ECI
  frame, WGS-84 ellipsoid for geodetic↔ECEF, ECI↔ECEF rotation by Earth rate. Ground impact is
  the WGS-84 ellipsoid (`alt < 0`).
- **Governing equations.** `g = −μ r̂/|r|²` (+ J2 perturbation); WGS-84 geodetic↔ECEF closed
  form; ECI↔ECEF rotation `R(ω_⊕ t)`. Vacuum two-body energy is conserved (a verified benchmark),
  and the flat mode reproduces the prior trajectory (regression guard).
- **Validity limits.** For long-range / exo-atmospheric engagements where curvature and rotation
  matter. The atmosphere model on the round path is dispatched in the Runner; this entry covers the
  frame/gravity geometry. J2 only (no higher-order geopotential, no third-body).
- **References.** Vallado, *Fundamentals of Astrodynamics and Applications* (ECI/ECEF, J2, WGS-84).

### `round` (hi-fi) — EGM gravity + extended atmosphere/winds + rotating ECEF

- **Assumptions.** Opt-in fidelity layer on the round path (issue #41): truncated zonal
  spherical-harmonic gravity (J2/J3/J4), an extended atmosphere above the USSA76 ceiling plus a
  parameterized wind profile, and a full rotating-ECEF propagation option with explicit
  Coriolis/centrifugal terms.
- **Governing equations.** Zonal geopotential through J4; exponential/layered atmosphere
  extension; ECEF EOM with `-2 omega x v - omega x (omega x r)`.
- **Validity limits.** Reduced upper atmosphere (no F10.7/Ap/diurnal/seasonal terms — full
  NRLMSISE-00 is a follow-up); zonal-only geopotential (no tesseral/sectoral, no third body). The
  hi-fi round path is the unguided ballistic case, matching the base round path.
- **Evidence.** `env_fidelity_test` (J2-only reduces to central+J2 bit-for-bit; WGS-84 surface
  gravity pole>equator; vacuum energy drift 2.7e-7/orbit; atmosphere handover continuous; ECEF<->ECI
  agree <1 m / <5 cm/s). Config `configs/ballistic_round_hifi.json`.
- **References.** Vallado (geopotential, ECEF); *U.S. Standard Atmosphere 1976* (base atmosphere).

---

## Threats (target maneuver)

### `constant` — Non-maneuvering (ballistic / constant-velocity) target

- **Assumptions.** The target applies **zero** commanded lateral acceleration; its path is set
  by its launch state + environment (ballistic) only.
- **Governing equation.** `a_target = 0`.
- **Validity limits.** The benign baseline. PN is optimal against it; not representative of an
  evasive threat.
- **References.** n/a (null maneuver).

### `weave` — Sinusoidal lateral weave

- **Assumptions.** A constant-amplitude sinusoidal lateral acceleration, **horizontal and
  perpendicular** to the target's ground track. Drives the augmented-PN / IMM benchmarks.
- **Governing equation.**
  `a_target = perp · (g_man · 9.80665 · sin(2π f t + φ))`, where `perp` is the in-plane unit
  vector perpendicular to the horizontal velocity, `g_man = target.maneuver_g`,
  `f = target.maneuver_freq`, `φ = target.maneuver_phase_deg`.
- **Validity limits.** A canonical evasive maneuver for stressing guidance/estimation; a single
  sinusoid, horizontal plane only. Degenerates to no maneuver if the target's horizontal speed
  is ~0.
- **References.** Zarchan ch. 4 (weaving-target case for Augmented PN).

### `icbm` — Multi-stage boosting ICBM

- **Assumptions.** A serial stack of boosting stages (`target.icbm.stages`) plus a payload
  (`payload_mass_kg`). Each stage thrusts along the velocity vector for its `burn_time_s`, burning
  `propellant_mass_kg` linearly; at burn-out the spent stage's `dry_mass_kg` is jettisoned (the
  staging event) and the next stage ignites. After the last stage the threat coasts ballistically.
  Gravity is constant `g0` (the threat supplies its own gravity; the flat-Earth target propagation
  does not otherwise gravitate the target).
- **Governing equations.** During stage *k*'s burn,
  `a = ĝ·thrust_n_k / m(t)  +  (0,0,−g0)`, where `ĝ = v̂` (velocity direction) and the mass
  schedule is `m(t) = M0 − Σ(jettisoned dry+propellant) − (propellant burned in the active stage)`,
  `M0 = payload + Σ_k (dry_mass_kg_k + propellant_mass_kg_k)`. After the final burn-out
  `a = (0,0,−g0)` (ballistic midcourse).
- **Validity limits.** A point-mass boost model: thrust along velocity, linear burn, flat-Earth
  constant gravity, no atmospheric drag on the threat. It reproduces the characteristic lofted
  ICBM arc (apogee of order 10³ km, downrange of order 10³ km for a near-vertical loft) and the
  discrete staging mass drops, but it is not a round-Earth great-circle range model.
- **References.** Boost-phase rocket staging (Tsiolkovsky/serial staging); minimum-energy vs
  lofted ballistic trajectories (Wertz, *Orbit & Constellation Design*).

### `hgv` — Hypersonic glide vehicle (skip-glide)

- **Assumptions.** A lifting reentry body. Below `pull_up_alt_m` it develops aerodynamic lift
  perpendicular to its velocity in the vertical plane, magnitude `(L/D)·drag`, directed "up";
  drag opposes velocity. The atmosphere is a self-contained exponential profile
  (`ρ = rho0·exp(−h/scale_height_m)`) so the threat does not depend on the interceptor-side aero.
  Drag deceleration is `q/β = ½ρV²/ballistic_coeff`. Gravity is constant `g0`.
- **Governing equations.**
  `a = (0,0,−g0) − v̂·(½ρV²/β) + n̂·(L/D)·(½ρV²/β)`, where `v̂` is the velocity direction,
  `n̂` the in-plane unit vector perpendicular to `v̂` toward `+z`, `β = ballistic_coeff`, and the
  lift term is gated off above `pull_up_alt_m`.
- **Validity limits.** Reproduces the qualitative **skip-glide oscillation** (repeated altitude
  pull-ups) and the monotone L/D→downrange dependence (higher L/D glides farther). It is a
  vertical-plane point-mass model: no banking/cross-range, no thermodynamics, an exponential
  (not USSA76) atmosphere, constant flat-Earth gravity.
- **References.** Sänger–Bredt skip-glide trajectory; Eggers boost-glide analysis;
  Allen–Eggers reentry.

### `rv_penaids` — Reentry vehicle + penetration aids

- **Assumptions.** The lethal reentry vehicle (RV) is a heavy ballistic body: `accel` is gravity
  only. A dispenser releases `penaid_count` lighter penaids/decoys about the RV at `deploy_time_s`
  with a deterministic radial dispense pattern (`deploy_dv_mps`); penaids carry an extra
  atmospheric deceleration (`penaid_decel_mps2`) because their lower ballistic coefficient sheds
  speed faster than the heavy RV. That deceleration difference is the kinematic discrimination cue
  consumed by the discrimination stack (issue #6).
- **Governing equations.** RV: `a = (0,0,−g0)`. Penaid *i* (after deploy): velocity offset
  `Δv_i = (ê₁cos θ_i + ê₂ sin θ_i)·deploy_dv_mps` with `θ_i = 2πi/N` in the plane ⟂ to the RV
  velocity; propagation `a = (0,0,−g0) − v̂·penaid_decel_mps2`. Scoring: a penaid is correctly
  classified when its deceleration feature exceeds the RV's (always true for `penaid_decel_mps2 > 0`).
- **Validity limits.** Deterministic kinematic deployment (no RNG) and a single scalar
  deceleration feature; the richer noisy multi-feature signature model lives in the `decoys` block
  (issue #6). The RV itself does not maneuver. Penaids separate from the RV over tens of seconds.
- **References.** Reentry-vehicle penetration aids / decoy discrimination (ballistic-coefficient
  separation); see also the `decoys` discrimination model.
</content>
</invoke>
