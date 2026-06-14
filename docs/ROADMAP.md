# Roadmap

The guiding principle (and the reason for the heavy validation up front): **establish a valid,
repeatable environment, then add fidelity in layers** — each layer landing with its own tests and
validation so the baseline never silently drifts. The eventual target is a launch-engagement
simulation: a boost/glide-phase threat, multiple ground- and space-based trackers, multi-sensor
fusion, and a coordinated interceptor.

## Status — Phase 0 baseline (done)

A repeatable single-interceptor / single-target terminal-homing engagement, validated end to end:
deterministic C++ core (native + WASM parity), PN guidance, seeker + IMU with Allan-characterized
noise, analytical validation suite, Monte Carlo CEP, CI, and an interactive 3D web demo.

## Follow-on issues (proposed, roughly ordered)

**Navigation / estimation**
- [ ] Replace the alpha-beta tracker with a proper **Extended Kalman Filter** for relative state;
      log NEES/NIS consistency and innovation whiteness as validation artifacts.
- [ ] **Multi-sensor fusion**: combine a ground-based radar (range + range-rate + angles) with a
      space-based IR tracker (angles-only); model each sensor's measurement covariance from its own
      Allan/PSD characterization.
- [ ] **Seeker discrimination** stub: closely-spaced-object / decoy handling, feature-based scoring.

**Threat & environment fidelity**
- [ ] Round-Earth (ECEF/ECI) dynamics + WGS-84; gravity harmonics. Retire the flat-Earth assumption
      behind a frame-abstraction layer so existing tests still pin the flat-Earth limit.
- [ ] **Boost-phase / glide threat model**: powered ICBM ascent, staging events, then a hypersonic
      glide trajectory (a hypersonic glide-vehicle engagement).
- [ ] Atmosphere perturbations (winds, density dispersions) as Monte Carlo factors.

**Guidance & control**
- [ ] Predictive / optimal midcourse guidance to a glide-phase intercept window; hand-off to terminal PN.
- [ ] Higher-fidelity 6-DOF: full inertia tensor, fin aerodynamics + actuator dynamics, thrust/mass burn.

**Coordination & scale**
- [ ] Multiple trackers feeding a central track file; interceptor cueing and **launch-on-track**.
- [ ] Engagement-level Monte Carlo: P(kill) vs threat dispersions and sensor coverage geometry.

**Tooling**
- [ ] MATLAB analysis scripts alongside the Python suite (the data contract is already language-neutral).
- [ ] Scenario schema versioning + a regression harness that diffs key metrics against golden runs.
- [ ] Parameterized batch runner on the native binary for large (10⁴+) Monte Carlo campaigns.

## Non-negotiables for every layer

1. New physics ships with a closed-form or reference-data validation check.
2. Determinism preserved — native↔WASM parity stays green (or WASM scoped out explicitly per feature).
3. CI stays green: build → unit tests → validation → parity → deploy.
