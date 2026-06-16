// gnc-sim — high-fidelity environment models (issue #41). Opt-in, additive extensions to the
// baseline gravity + USSA76 atmosphere used only on the round-Earth path:
//
//   * EGM-style truncated spherical-harmonic gravity (zonal J2..J4), hand-rolled, coefficients
//     cited inline. Reduces EXACTLY to central + J2 when the higher zonals are zero, so the
//     existing `env.j2` round path is unchanged.
//   * An exponential/layered extension of USSA76 above its 86 km ceiling, with winds. The
//     low-altitude (0..86 km) sample is the unchanged USSA76; the extension is C0-continuous at
//     the 86 km handover. This is a CREDIBLE REDUCED model, NOT a full NRLMSISE-00 (no
//     solar/geomagnetic/diurnal terms) — see the limits note in EnvFidelity.cpp and the
//     follow-up for a full NRLMSISE-00 port.
//   * A parameterized altitude-dependent wind profile (ENU), used to form the air-relative
//     velocity for drag.
//
// Pure compute, no I/O, deterministic. All formulae use libm sin/cos/pow; on the NEW hi-fi config
// this can introduce a ~1e-13 native-vs-WASM floating-point floor (acceptable; the default and the
// existing flat/round paths never call these and stay byte-identical).
#pragma once

#include "gncsim/core/Config.hpp"
#include "gncsim/env/Environment.hpp"
#include "gncsim/math/Vector3.hpp"

namespace gncsim {

// --- High-fidelity gravity ------------------------------------------------------------------
// Truncated zonal spherical-harmonic gravitational acceleration at Earth-fixed/inertial position
// `r` (m, +Z = spin axis). Includes the central point-mass term plus the zonal harmonics J2, J3,
// J4 selected by `cfg`. With all higher zonals disabled this is bit-for-bit the existing
// centralGravity(r, with_j2) result (same J2 expansion), so the legacy round path is unchanged.
Vector3 egmGravity(const Vector3& r, const GravityFidelityConfig& cfg);

// --- High-fidelity atmosphere ---------------------------------------------------------------
// USSA76 below 86 km (delegates to atmosphereUSSA76, so the lower atmosphere is identical), then an
// exponential-layer extension up to ~1000 km. C0-continuous in density/temperature at 86 km.
// Above the modeled top the topmost layer's exponential is held (no NaNs).
AtmSample atmosphereExtended(double altitude_m);

// --- Winds ----------------------------------------------------------------------------------
// Parameterized horizontal wind in the LOCAL ENU frame [m/s] at geometric altitude `altitude_m`.
// A simple sheared profile: the wind grows linearly from `surface_mps` at the ground to a jet
// maximum at `jet_alt_m`, then decays exponentially above it, blowing toward `dir_deg` (compass-
// style, measured from East toward North to match the ENU launch-azimuth convention). Returns the
// zero vector when winds are disabled.
Vector3 windEnu(double altitude_m, const WindConfig& cfg);

}  // namespace gncsim
