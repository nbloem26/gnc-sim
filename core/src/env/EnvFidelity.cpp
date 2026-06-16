// gnc-sim — high-fidelity environment models (issue #41): truncated zonal spherical-harmonic
// gravity (J2..J4), an exponential extension of USSA76 above 86 km, and a parameterized wind
// profile. Pure compute, deterministic, no external dependencies (hand-rolled). See
// gncsim/env/EnvFidelity.hpp for the contract and the documented limits.
#include "gncsim/env/EnvFidelity.hpp"

#include <algorithm>
#include <cmath>

#include "gncsim/env/Frames.hpp"

namespace gncsim {

namespace {

// --- Zonal harmonic coefficients ------------------------------------------------------------
// Normalized-form-free (unnormalized) zonal coefficients J_n of Earth's geopotential. J2 matches
// wgs84::kJ2 exactly so "egm" with only J2 reduces to the legacy centralGravity J2 expansion.
//   Source: WGS-84 / EGM-96 unnormalized zonals (NIMA TR8350.2, Table 3.4 & EGM96 model):
//     J2 = +1.082626683e-3
//     J3 = -2.532435346e-6
//     J4 = -1.619331205e-6
// These three zonals capture the dominant non-spherical (oblateness, pear-shape) gravity field.
constexpr double kJ2 = 1.082626683e-3;
constexpr double kJ3 = -2.532435346e-6;
constexpr double kJ4 = -1.619331205e-6;

// --- USSA76-extension anchor nodes (geometric altitude > 86 km) ------------------------------
// Above 86 km the full USSA76 transitions to species-dependent diffusive separation; a faithful
// port is large. We instead interpolate the US Standard Atmosphere 1976 UPPER tables between a few
// anchor nodes: density is LOG-LINEAR in altitude between nodes (i.e. each interval has its own
// effective exponential scale height that makes density continuous at every node), and temperature
// is linear. This is strictly monotone, C0-continuous across 86..1000 km, and matches the published
// curve at the nodes. This is a REDUCED model: no solar-activity (F10.7), geomagnetic (Ap), or
// diurnal/seasonal variation. Follow-up: full NRLMSISE-00 port.
//   Source: US Standard Atmosphere 1976 (mass density & temperature tables), 86-1000 km.
struct ExtNode {
  double alt_m;   // geometric altitude [m]
  double rho;     // tabulated density [kg/m^3]
  double temp_k;  // tabulated temperature [K]
};

constexpr double kHandoverAltM = 86000.0;  // USSA76 ceiling; the first node's rho is pinned to the
                                           // live USSA76 value here for a continuous handover.

const ExtNode kExtNodes[] = {
    {86000.0, 6.958e-6, 186.87},      //
    {100000.0, 5.604e-7, 195.08},     //
    {150000.0, 2.076e-9, 634.39},     //
    {200000.0, 2.541e-10, 854.56},    //
    {300000.0, 1.916e-11, 976.01},    //
    {500000.0, 6.967e-13, 999.24},    //
    {1000000.0, 3.561e-15, 1000.00},  // top anchor (held above)
};
constexpr int kNumExtNodes = static_cast<int>(sizeof(kExtNodes) / sizeof(kExtNodes[0]));

constexpr double kRgas = 287.05287;  // specific gas constant for air [J/(kg*K)]
constexpr double kGamma = 1.4;       // ratio of specific heats [-]

constexpr double kDeg2Rad = M_PI / 180.0;

}  // namespace

// =============================================================================================
// EGM-style zonal gravity (central + J2..J4)
// =============================================================================================
//
// Acceleration of the zonal geopotential, expressed in the same Earth-fixed/inertial frame as the
// position (spin axis = +Z). For zonal harmonic n the standard closed form (e.g. Vallado,
// "Fundamentals of Astrodynamics and Applications") gives the perturbing acceleration in terms of
// the ratio re/r and the latitude term phi = z/r. We accumulate the central term plus each selected
// zonal. The J2 block is algebraically identical to centralGravity()'s J2 term.
Vector3 egmGravity(const Vector3& r, const GravityFidelityConfig& cfg) {
  const double rmag_m = r.norm();
  if (rmag_m < 1.0) return Vector3{};  // guard the singularity at the centre

  const double mu = wgs84::kGM;
  const double re_m = wgs84::kA;
  const double inv_r = 1.0 / rmag_m;
  const double inv_r2 = inv_r * inv_r;

  // Central point-mass term: -mu/r^3 * r.
  Vector3 g = r * (-mu * inv_r2 * inv_r);

  const double x_m = r.x, y_m = r.y, z_m = r.z;
  const double zr = z_m * inv_r;  // sin(geocentric latitude)
  const double zr2 = zr * zr;
  const double re_r = re_m * inv_r;  // (Re/r)
  const double re_r2 = re_r * re_r;
  const double mu_r2 = mu * inv_r2;  // mu/r^2

  // --- J2 ---  (matches centralGravity's expansion exactly)
  if (cfg.include_j2) {
    const double factor = 1.5 * kJ2 * mu * re_m * re_m * inv_r2 * inv_r2;  // 1.5 J2 mu Re^2 / r^4
    const double zterm = 5.0 * zr2;
    g.x += factor * (zterm - 1.0) * x_m * inv_r;
    g.y += factor * (zterm - 1.0) * y_m * inv_r;
    g.z += factor * (zterm - 3.0) * z_m * inv_r;
  }

  // --- J3 ---  (odd zonal; "pear-shape"). Vallado form:
  //   a_{x,y} = -(5/2) J3 (mu/r^2)(Re/r)^3 [ 7 (z/r)^3 - 3 (z/r) ] (x or y)/r
  //   a_z     = -(1/2) J3 (mu/r^2)(Re/r)^3 [ 35 (z/r)^4 - 30 (z/r)^2 + 3 ]
  if (cfg.include_j3) {
    const double re_r3 = re_r2 * re_r;
    const double common = mu_r2 * re_r3;
    const double horiz = -2.5 * kJ3 * common * (7.0 * zr2 * zr - 3.0 * zr) * inv_r;
    g.x += horiz * x_m;
    g.y += horiz * y_m;
    g.z += -0.5 * kJ3 * common * (35.0 * zr2 * zr2 - 30.0 * zr2 + 3.0);
  }

  // --- J4 ---  Vallado form:
  //   a_{x,y} = (5/8) J4 (mu/r^2)(Re/r)^4 [ 63 (z/r)^4 - 42 (z/r)^2 + 3 ] (x or y)/r
  //   a_z     = (5/8) J4 (mu/r^2)(Re/r)^4 [ 33 (z/r)^4 - 30 (z/r)^2 + 5 ] (z/r)
  if (cfg.include_j4) {
    const double re_r4 = re_r2 * re_r2;
    const double common = mu_r2 * re_r4;
    const double horiz = 0.625 * kJ4 * common * (63.0 * zr2 * zr2 - 42.0 * zr2 + 3.0) * inv_r;
    g.x += horiz * x_m;
    g.y += horiz * y_m;
    g.z += 0.625 * kJ4 * common * (33.0 * zr2 * zr2 - 30.0 * zr2 + 5.0) * zr;
  }

  return g;
}

// =============================================================================================
// Extended atmosphere (USSA76 below 86 km, log-linear node interpolation above)
// =============================================================================================
AtmSample atmosphereExtended(double altitude_m) {
  // Below the USSA76 ceiling, delegate verbatim — the lower atmosphere is identical.
  if (altitude_m <= kHandoverAltM) {
    return atmosphereUSSA76(altitude_m);
  }

  // Pin the bottom node's density to the live USSA76 value at exactly 86 km so the handover is
  // continuous to machine precision regardless of the tabulated node constant.
  const double rho0_kgpm3 = atmosphereUSSA76(kHandoverAltM).density;

  // Above the top anchor, hold the top node (finite, tiny, no NaN).
  if (altitude_m >= kExtNodes[kNumExtNodes - 1].alt_m) {
    const ExtNode& top = kExtNodes[kNumExtNodes - 1];
    AtmSample s;
    s.temperature = top.temp_k;
    s.density = top.rho;
    s.pressure = top.rho * kRgas * top.temp_k;
    s.speed_of_sound = std::sqrt(kGamma * kRgas * top.temp_k);
    return s;
  }

  // Find the bracketing interval [lo, hi] with kExtNodes[lo].alt <= altitude_m < kExtNodes[hi].alt.
  int lo = 0;
  for (int i = 0; i + 1 < kNumExtNodes; ++i) {
    if (altitude_m >= kExtNodes[i].alt_m) lo = i;
  }
  const ExtNode& a = kExtNodes[lo];
  const ExtNode& b = kExtNodes[lo + 1];
  const double rho_a_kgpm3 = (lo == 0) ? rho0_kgpm3 : a.rho;
  const double rho_b_kgpm3 = b.rho;

  // Log-linear (constant effective scale height) density interpolation within the interval, so the
  // curve is continuous and strictly monotone between nodes.
  const double frac = (altitude_m - a.alt_m) / (b.alt_m - a.alt_m);
  const double density_kgpm3 = rho_a_kgpm3 * std::pow(rho_b_kgpm3 / rho_a_kgpm3, frac);

  // Linear temperature interpolation within the interval.
  const double temperature_k = a.temp_k + frac * (b.temp_k - a.temp_k);

  AtmSample s;
  s.temperature = temperature_k;
  s.density = density_kgpm3;
  s.pressure = density_kgpm3 * kRgas * temperature_k;  // ideal-gas closure (continuum proxy)
  s.speed_of_sound = std::sqrt(kGamma * kRgas * temperature_k);
  return s;
}

// =============================================================================================
// Winds (parameterized ENU profile)
// =============================================================================================
Vector3 windEnu(double altitude_m, const WindConfig& cfg) {
  if (!cfg.enabled) return Vector3{};

  const double z_m = std::max(0.0, altitude_m);
  double speed_mps;
  if (z_m <= cfg.jet_alt_m) {
    // Linear shear from the surface value up to the jet maximum.
    const double frac = (cfg.jet_alt_m > 0.0) ? (z_m / cfg.jet_alt_m) : 1.0;
    speed_mps = cfg.surface_mps + frac * (cfg.jet_mps - cfg.surface_mps);
  } else {
    // Exponential decay above the jet back toward (and below) the surface value.
    const double decay = std::exp(-(z_m - cfg.jet_alt_m) / std::max(1.0, cfg.decay_scale_m));
    speed_mps = cfg.jet_mps * decay;
  }

  // Heading measured from East toward North (matches launch_azimuth convention). Horizontal only.
  const double dir_rad = cfg.dir_deg * kDeg2Rad;
  return {speed_mps * std::cos(dir_rad), speed_mps * std::sin(dir_rad), 0.0};
}

}  // namespace gncsim
