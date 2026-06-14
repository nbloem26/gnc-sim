// gnc-sim — high-fidelity environment tests (issue #41): EGM zonal gravity (J2..J4), the extended
// atmosphere above 86 km + winds, and rotating-ECEF vs ECI propagation consistency.
//
// Reference accelerations are cross-checked against the closed-form central + J2 expansion in
// Frames.cpp (which is independently validated in frames_test) and against published WGS-84 surface
// gravity. The orbit/energy test integrates a vacuum trajectory under the harmonic field and
// asserts specific-energy conservation. The ECEF/ECI test runs both round-mode propagation paths
// and asserts they agree (they are physically equivalent formulations).
#include <gtest/gtest.h>

#include <cmath>

#include "gncsim/core/Config.hpp"
#include "gncsim/env/EnvFidelity.hpp"
#include "gncsim/env/Environment.hpp"
#include "gncsim/env/Frames.hpp"
#include "gncsim/math/Vector3.hpp"
#include "gncsim/scenario/Runner.hpp"

using namespace gncsim;

namespace {

// A point ~700 km up at a representative latitude (Cape Canaveral-ish), in ECEF/ECI.
Vector3 samplePos() {
  return geodeticToEcef(28.4889 * M_PI / 180.0, -80.5778 * M_PI / 180.0, 700000.0);
}

}  // namespace

// =============================================================================================
// EGM gravity — reduction and reference accelerations
// =============================================================================================

TEST(EgmGravity, ReducesToCentralWhenNoZonals) {
  GravityFidelityConfig cfg;  // all zonals off
  for (const Vector3& r : {samplePos(), Vector3{7.0e6, 0.0, 0.0}, Vector3{0.0, 0.0, 6.8e6}}) {
    const Vector3 egm = egmGravity(r, cfg);
    const Vector3 central = centralGravity(r, /*with_j2=*/false);
    EXPECT_NEAR(egm.x, central.x, 1e-9 * std::max(1.0, std::fabs(central.x)));
    EXPECT_NEAR(egm.y, central.y, 1e-9 * std::max(1.0, std::fabs(central.y)));
    EXPECT_NEAR(egm.z, central.z, 1e-9 * std::max(1.0, std::fabs(central.z)));
  }
}

TEST(EgmGravity, J2MatchesClosedFormCentralJ2) {
  GravityFidelityConfig cfg;
  cfg.include_j2 = true;
  // The EGM J2 term must reproduce the closed-form central+J2 expansion (Frames.cpp) bit-for-bit
  // wherever it is exercised — this is what keeps the legacy round path unchanged.
  for (const Vector3& r : {samplePos(), Vector3{7.0e6, 1.0e6, 2.0e6}, Vector3{0.0, 0.0, 6.9e6}}) {
    const Vector3 egm = egmGravity(r, cfg);
    const Vector3 ref = centralGravity(r, /*with_j2=*/true);
    EXPECT_NEAR(egm.x, ref.x, 1e-9 * std::max(1.0, std::fabs(ref.x)));
    EXPECT_NEAR(egm.y, ref.y, 1e-9 * std::max(1.0, std::fabs(ref.y)));
    EXPECT_NEAR(egm.z, ref.z, 1e-9 * std::max(1.0, std::fabs(ref.z)));
  }
}

TEST(EgmGravity, SurfaceMagnitudeMatchesWgs84) {
  // Equatorial surface point. Total gravitational acceleration (no centrifugal) should be close to
  // the WGS-84 GM/Re^2 ~ 9.80 m/s^2; J2 makes the equatorial value slightly smaller than the poles.
  const Vector3 eq{wgs84::kA, 0.0, 0.0};
  GravityFidelityConfig cfg;
  cfg.include_j2 = true;
  cfg.include_j3 = true;
  cfg.include_j4 = true;
  const double g_eq = egmGravity(eq, cfg).norm();
  EXPECT_NEAR(g_eq, wgs84::kGM / (wgs84::kA * wgs84::kA), 0.05);  // ~9.80 m/s^2 within 0.05

  // Polar point (along +Z at the semi-minor axis): J2 makes polar gravity LARGER than equatorial.
  const Vector3 pole{0.0, 0.0, wgs84::kB};
  const double g_pole = egmGravity(pole, cfg).norm();
  EXPECT_GT(g_pole, g_eq);
}

TEST(EgmGravity, ZonalTermsAreSmallCorrections) {
  // J3/J4 are 3+ orders of magnitude smaller than the J2 correction, which is itself ~1e-3 of the
  // central term. Sanity-check the magnitudes so a sign/coefficient blunder is caught.
  const Vector3 r = samplePos();
  const double central = centralGravity(r, false).norm();

  GravityFidelityConfig j2;
  j2.include_j2 = true;
  GravityFidelityConfig j234 = j2;
  j234.include_j3 = true;
  j234.include_j4 = true;

  const double dj2 = (egmGravity(r, j2) - centralGravity(r, false)).norm();
  const double dj34 = (egmGravity(r, j234) - egmGravity(r, j2)).norm();
  EXPECT_GT(dj2, 1e-4 * central);  // J2 correction is ~1e-3 of central
  EXPECT_LT(dj2, 1e-2 * central);
  EXPECT_GT(dj34, 0.0);         // J3+J4 are nonzero
  EXPECT_LT(dj34, 0.05 * dj2);  // but much smaller than the J2 correction
}

// =============================================================================================
// Vacuum orbit / energy conservation under the harmonic field
// =============================================================================================

TEST(EgmGravity, CircularOrbitConservesEnergy) {
  // Integrate a near-circular LEO under central + J2 + J3 + J4 with a fixed-step RK4 and check that
  // the specific orbital energy E = v^2/2 - GM/r drifts only at integrator-truncation level over
  // one orbit. (J-perturbations change the orbit shape but conserve energy in a static field.)
  GravityFidelityConfig cfg;
  cfg.include_j2 = true;
  cfg.include_j3 = true;
  cfg.include_j4 = true;

  const double r0 = wgs84::kA + 500000.0;  // 500 km altitude
  const double v0 = std::sqrt(wgs84::kGM / r0);
  Vector3 r{r0, 0.0, 0.0};
  Vector3 v{0.0, v0 * std::cos(0.6), v0 * std::sin(0.6)};  // inclined so J-terms are exercised

  auto energy = [&](const Vector3& p, const Vector3& vel) {
    return 0.5 * vel.normSq() - wgs84::kGM / p.norm();
  };
  const double e0 = energy(r, v);

  const double dt = 0.5;
  const int steps = static_cast<int>(2.0 * M_PI * std::sqrt(r0 * r0 * r0 / wgs84::kGM) / dt);
  for (int i = 0; i < steps; ++i) {
    // RK4 on (r, v) with a = egmGravity(r).
    auto a = [&](const Vector3& p) { return egmGravity(p, cfg); };
    const Vector3 k1r = v, k1v = a(r);
    const Vector3 k2r = v + k1v * (0.5 * dt), k2v = a(r + k1r * (0.5 * dt));
    const Vector3 k3r = v + k2v * (0.5 * dt), k3v = a(r + k2r * (0.5 * dt));
    const Vector3 k4r = v + k3v * dt, k4v = a(r + k3r * dt);
    r = r + (k1r + k2r * 2.0 + k3r * 2.0 + k4r) * (dt / 6.0);
    v = v + (k1v + k2v * 2.0 + k3v * 2.0 + k4v) * (dt / 6.0);
  }
  const double e1 = energy(r, v);
  EXPECT_NEAR(e1, e0, 1e-6 * std::fabs(e0));  // relative energy drift < 1e-6 over one orbit
}

// =============================================================================================
// Extended atmosphere — continuity at the USSA76 handover and behaviour above
// =============================================================================================

TEST(AtmosphereExtended, IdenticalToUssa76BelowHandover) {
  for (double alt : {0.0, 5000.0, 11000.0, 32000.0, 60000.0, 80000.0, 86000.0}) {
    AtmSample ext = atmosphereExtended(alt);
    AtmSample base = atmosphereUSSA76(alt);
    EXPECT_DOUBLE_EQ(ext.density, base.density) << "alt=" << alt;
    EXPECT_DOUBLE_EQ(ext.temperature, base.temperature) << "alt=" << alt;
  }
}

TEST(AtmosphereExtended, ContinuousDensityAcrossHandover) {
  // Density must be continuous across the 86 km handover (the extension is pinned to the live
  // USSA76 value at 86 km).
  AtmSample below = atmosphereExtended(85999.0);
  AtmSample at = atmosphereExtended(86000.0);
  AtmSample above = atmosphereExtended(86001.0);
  EXPECT_NEAR(above.density, at.density, 1e-3 * at.density);
  EXPECT_NEAR(below.density, at.density, 1e-3 * at.density);
  // No discontinuous jump: the relative step across the seam is tiny.
  EXPECT_LT(std::fabs(above.density - below.density), 1e-2 * at.density);
}

TEST(AtmosphereExtended, DecaysMonotonicallyAndStaysFinite) {
  double prev = 1e30;
  for (double alt = 86000.0; alt <= 1000000.0; alt += 5000.0) {
    AtmSample s = atmosphereExtended(alt);
    EXPECT_GT(s.density, 0.0) << "alt=" << alt;
    EXPECT_LT(s.density, prev) << "alt=" << alt;  // strictly thinning
    EXPECT_TRUE(std::isfinite(s.pressure));
    EXPECT_TRUE(std::isfinite(s.speed_of_sound));
    EXPECT_GT(s.temperature, 0.0);
    prev = s.density;
  }
  // Above the modeled ceiling the top layer is held (finite, tiny, no NaN).
  AtmSample top = atmosphereExtended(2.0e6);
  EXPECT_TRUE(std::isfinite(top.density));
  EXPECT_GT(top.density, 0.0);
}

// =============================================================================================
// Winds — parameterized profile
// =============================================================================================

TEST(WindProfile, DisabledIsZero) {
  WindConfig cfg;  // enabled defaults false
  EXPECT_EQ(windEnu(0.0, cfg).norm(), 0.0);
  EXPECT_EQ(windEnu(50000.0, cfg).norm(), 0.0);
}

TEST(WindProfile, ShearPeaksAtJetAndDecaysAbove) {
  WindConfig cfg;
  cfg.enabled = true;
  cfg.surface_mps = 5.0;
  cfg.jet_mps = 40.0;
  cfg.jet_alt_m = 10000.0;
  cfg.decay_scale_m = 8000.0;
  cfg.dir_deg = 0.0;  // due East

  EXPECT_NEAR(windEnu(0.0, cfg).norm(), 5.0, 1e-9);       // surface value
  EXPECT_NEAR(windEnu(10000.0, cfg).norm(), 40.0, 1e-9);  // peak at jet altitude
  EXPECT_LT(windEnu(30000.0, cfg).norm(), 40.0);          // decays above
  EXPECT_GT(windEnu(5000.0, cfg).norm(), 5.0);            // sheared between surface and jet
  // Direction: dir_deg = 0 => +East, horizontal only.
  const Vector3 w = windEnu(10000.0, cfg);
  EXPECT_NEAR(w.x, 40.0, 1e-9);
  EXPECT_NEAR(w.y, 0.0, 1e-9);
  EXPECT_EQ(w.z, 0.0);
}

// =============================================================================================
// Rotating-ECEF vs ECI propagation consistency
// =============================================================================================

namespace {

SimConfig roundBallisticBase() {
  SimConfig c;
  c.scenario = "ballistic";
  c.model = "3dof";
  c.dt = 0.02;
  c.t_end = 120.0;
  c.integrator = Integrator::RK4;
  c.env.frame = "round";
  c.env.atmosphere = false;  // vacuum: isolate the gravity + frame physics
  c.env.j2 = false;
  c.guidance.law = "none";
  c.vehicle.pos0 = {0.0, 0.0, 0.0};
  c.vehicle.launch_speed = 2800.0;
  c.vehicle.launch_elevation_deg = 45.0;
  c.vehicle.launch_azimuth_deg = 90.0;
  c.vehicle.mass0 = 100.0;
  c.target.pos0 = {0.0, 700000.0, 0.0};
  c.target.vel0 = {0.0, 0.0, 0.0};
  return c;
}

}  // namespace

TEST(RoundPropagation, EcefAndEciAgree) {
  // The rotating-ECEF formulation (Coriolis + centrifugal) and the ECI formulation are physically
  // equivalent. Over a 120 s vacuum lob they must agree to integrator precision in the reported ENU
  // state (a small finite-step difference is expected because one applies Earth's rotation
  // kinematically and the other dynamically).
  SimConfig eci = roundBallisticBase();
  SimConfig ecef = roundBallisticBase();
  ecef.env.rotating_ecef = true;

  SimResult r_eci = runSimulation(eci);
  SimResult r_ecef = runSimulation(ecef);

  ASSERT_EQ(r_eci.frames.size(), r_ecef.frames.size());
  const Frame& a = r_eci.frames.back();
  const Frame& b = r_ecef.frames.back();
  const double pos_diff = (a.veh_pos - b.veh_pos).norm();
  const double vel_diff = (a.veh_vel - b.veh_vel).norm();
  EXPECT_LT(pos_diff, 1.0);   // < 1 m over 120 s of flight (~240 km downrange)
  EXPECT_LT(vel_diff, 0.05);  // < 5 cm/s
}

TEST(RoundPropagation, EcefPathRunsWithHiFiEnvironment) {
  // Smoke test: the rotating-ECEF path with EGM gravity, the extended atmosphere, and winds enabled
  // produces a finite, sane trajectory.
  SimConfig c = roundBallisticBase();
  c.env.rotating_ecef = true;
  c.env.atmosphere = true;
  c.env.gravity_model = "egm";
  c.env.gravity.include_j2 = true;
  c.env.gravity.include_j3 = true;
  c.env.gravity.include_j4 = true;
  c.env.atmosphere_model = "extended";
  c.env.wind.enabled = true;
  c.env.wind.surface_mps = 5.0;
  c.env.wind.jet_mps = 35.0;
  c.aero.ref_area = 0.02;

  SimResult r = runSimulation(c);
  ASSERT_FALSE(r.frames.empty());
  for (const Frame& f : r.frames) {
    EXPECT_TRUE(std::isfinite(f.veh_pos.x));
    EXPECT_TRUE(std::isfinite(f.veh_pos.y));
    EXPECT_TRUE(std::isfinite(f.veh_pos.z));
    EXPECT_TRUE(std::isfinite(f.mach));
  }
}
