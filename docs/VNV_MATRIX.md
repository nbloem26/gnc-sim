# V&V matrix

This is the **verification & validation matrix**: every shipped model (and its headline
claim) mapped to the **evidence** that backs it — the GoogleTest benchmark case, the
golden-run baseline entry, and/or the analytic check in `postproc/`. It is the audit trail
behind [`docs/MODELS.md`](MODELS.md).

**This table is machine-checked.** `postproc/tests/test_vnv_matrix.py` parses it and asserts:

1. every shipped model (resolved by [`core/src/model/Registry.cpp`](../core/src/model/Registry.cpp))
   has **≥1 row**;
2. every non-`—` **Golden key** names an actual case in
   [`postproc/golden/golden.json`](../postproc/golden/golden.json);
3. every **Benchmark** names a real `tests/*_test.cpp` file (the GoogleTest suite, our
   primary benchmark — built and run via CTest);
4. every row links to **at least one** form of evidence (benchmark, golden, or analytic).

If you add a model, add a row here too, or CI fails (see `AGENTS.md` → *Adding a model*).

## Evidence columns

- **Benchmark** — the C++ GoogleTest case(s) in `tests/<file>_test.cpp` (run via CTest). The
  primary, deterministic correctness benchmark. Format `file::TestSuite.TestName` (suite/name
  abbreviated when several cases back one claim).
- **Golden** — the case key in `postproc/golden/golden.json` whose headline metrics are frozen
  as the regression baseline (the *drift* net). `—` if no end-to-end golden run exercises this
  model in isolation.
- **Analytic** — the closed-form / statistical check in `postproc/gncpost/` (validation harness,
  fusion/discrimination/pkill sweeps, loop closure, Allan, USSA76 twin). `—` if none.

| Family | Model (`config key`) | Claim under test | Benchmark (`tests/…`) | Golden | Analytic (`postproc/…`) |
|---|---|---|---|---|---|
| Guidance | `pronav` (`guidance.law`) | PN issues `N·V_c·(Ω×û_LOS)` ⟂ LOS, limited; intercepts a non-maneuvering target | `gnc_test::ProNav.*`, `runner_test::Runner.NoiseFreeProNavIntercepts` | `homing_3dof` | `validate.py::check_pronav_intercept` |
| Guidance | `apn` (`guidance.law`) | Target-accel feedforward applied only under `apn`; beats PN vs a weaving target | `apn_test::Apn.*` | `montecarlo` | `pkill.py` campaign |
| Guidance | `zemzev` (`guidance.law`) | Optimal ZEM/ZEV drives miss→0 vs non-maneuvering & constant-accel targets; continuous midcourse→terminal handover; divert/ACS respects its authority limit | `optimal_guidance_test::ZemZev.*`, `registry_test::Registry.GuidanceKeysResolve` | — | — |
| Guidance | `none` (`guidance.law`) | No command issued; trajectory is purely ballistic | `gnc_test::ProNav.NoneLawGivesZeroCommand`, `registry_test::Registry.NoneGuidanceCommandsZero` | `projectile_3dof` | `validate.py::check_ballistic` |
| Navigation | `alpha_beta` (`nav.filter`) | Fixed-gain tracker recovers constant-velocity relative motion | `gnc_test::Navigator.TracksConstantVelocityRelativeMotion`, `registry_test::Registry.NavigatorKeysResolve` | `homing_3dof` | — |
| Navigation | `ekf` (`nav.filter`) | EKF converges on a CV track; NIS mean ≈ DOF (statistically consistent) | `ekf_test::Ekf.*` | — | — |
| Navigation | `imm` (`nav.filter`) | Mode probability switches on maneuver; outperforms a single EKF under maneuver; NIS consistent | `imm_test::Imm.*` | — | — |
| Dynamics | `3dof` (`vehicle.model`) | RK4 point-mass matches the analytic drag-free ballistic / free-fall apex solution | `dynamics_test::Dynamics3dof.*` | `homing_3dof` | `validate.py::check_ballistic`, `check_terminal_velocity` |
| Dynamics | `6dof` (`vehicle.model`) | Translation matches 3DOF; constant-torque spin-up; quaternion stays unit-norm | `dynamics_test::Dynamics6dof.*`, `sixdof_test::SixDofInertia.InverseRoundTrips` | — | — |
| Dynamics | `6dof_hifi` (`vehicle.model`) | Full inertia tensor: torque-free gyroscopic coupling conserves energy/momentum; intercepts with realistic fin response | `sixdof_test::SixDofHiFi.*` | — | — |
| Sensor | `radar` (`trackers[].type`) | Az/el/range/range-rate measurement; white-noise std matches param; fusion lowers covariance | `sensors_test::SeekerTest.*`, `target_track_test::TargetTrack*` | `track_fused` | `fusion.py` sweep |
| Sensor | `ir` (`trackers[].type`) | Angles-only az/el; weak range observability; fusion beats a single sensor's RMS | `target_track_test::TargetTrackEkf.AnglesOnlyHasWeakRangeObservability`, `target_track_test::TargetTrackRunner.FusionBeatsSingleSensorRms` | `track_fused` | `fusion.py` sweep |
| Environment | `flat` (`env.frame`) | Flat-Earth gravity + USSA76 atmosphere (sea-level/tropopause/monotonic) | `env_test::GravityModel.*`, `env_test::AtmosphereUSSA76.*` | `homing_3dof` | `atmosphere.py` twin, `validate.py::check_atmosphere` |
| Environment | `round` (`env.frame`) | Round-Earth ECI/ECEF + WGS-84; vacuum two-body energy conserved; flat mode reproduces prior trajectory | `frames_test::Frames.*`, `frames_test::RoundEarth.*` | `ballistic_round` | — |
| Environment | `round` hi-fi (`env.gravity`/`env.atmosphere`/`env.wind`) | EGM J2/J3/J4 zonal gravity (J2-only reduces to central+J2 bit-for-bit); extended atmosphere + winds; rotating-ECEF Coriolis/centrifugal; vacuum energy drift 2.7e-7/orbit | `env_fidelity_test::*` | — | in-test gravity-vs-reference + energy-conservation checks |
| Threat | `constant` (`target.maneuver`) | Non-maneuvering target → zero applied acceleration | `runner_test::Runner.UnguidedProjectileFallsBack`, `registry_test::Registry.ThreatKeysResolve` | `projectile_3dof` | `validate.py::check_ballistic` |
| Threat | `weave` (`target.maneuver`) | Sinusoidal perpendicular weave; APN/IMM benchmarks rely on it | `apn_test::Apn.BeatsPnAgainstWeavingTarget`, `imm_test::Imm.OutperformsSingleEkfUnderManeuver` | `montecarlo` | `pkill.py` campaign |

## How to read a row

Take `pronav`: the **claim** is that the law produces the textbook PN command and intercepts a
benign target. That claim is benchmarked deterministically by the C++ `ProNav.*` unit cases and
the `Runner.NoiseFreeProNavIntercepts` integration test; its end-to-end miss distance is frozen
in the `homing_3dof` golden entry (so a regression trips `python -m gncpost.golden`); and it's
independently re-derived against a closed-form intercept in `validate.py::check_pronav_intercept`.
Three independent legs — unit, regression, analytic — back the one claim.

A `—` in **Golden** or **Analytic** is allowed: not every model is exercised end-to-end in
isolation by a golden config, and not every model has a closed-form check. But **every** row has a
**Benchmark** (the consistency test enforces at-least-one evidence leg per row, and every shipped
model having at least one row).
</content>
