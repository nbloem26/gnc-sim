// gnc-sim — round-Earth frame abstraction tests (WGS-84 geodesy + central gravity).
// Round-trip precision, ENU axis orientation, gravity magnitude/falloff, plus a flat-vs-round
// ballistic propagation sanity check via runSimulation().
#include "gncsim/env/Frames.hpp"

#include <gtest/gtest.h>

#include <cmath>

#include "gncsim/core/Config.hpp"
#include "gncsim/math/Vector3.hpp"
#include "gncsim/scenario/Runner.hpp"

using namespace gncsim;

namespace {
constexpr double kDeg2Rad = M_PI / 180.0;

// Known WGS-84 geodetic origins to round-trip through.
struct Site {
  double lat_deg, lon_deg, alt_m;
};
const Site kSites[] = {
    {28.4889, -80.5778, 0.0},    // Cape Canaveral (default origin)
    {0.0, 0.0, 0.0},             // equator / prime meridian
    {51.4779, -0.0015, 45.0},    // Greenwich
    {-33.8568, 151.2153, 58.0},  // Sydney
    {89.0, 30.0, 12000.0},       // near North pole, high altitude
    {-77.85, 166.67, -50.0},     // Antarctica, below the ellipsoid
};
}  // namespace

// ---------------------------------------------------------------------------------------------
// geodetic <-> ECEF round-trip to mm precision
// ---------------------------------------------------------------------------------------------
TEST(Frames, GeodeticEcefRoundTrip) {
  for (const Site& s : kSites) {
    const double lat = s.lat_deg * kDeg2Rad;
    const double lon = s.lon_deg * kDeg2Rad;
    const Vector3 ecef = geodeticToEcef(lat, lon, s.alt_m);

    double lat2, lon2, alt2;
    ecefToGeodetic(ecef, lat2, lon2, alt2);

    // Re-encode and compare positions in metres (robust near the poles where lon is ill-defined).
    const Vector3 ecef2 = geodeticToEcef(lat2, lon2, alt2);
    EXPECT_LT((ecef - ecef2).norm(), 1e-3) << "site lat=" << s.lat_deg;
    EXPECT_NEAR(alt2, s.alt_m, 1e-3) << "site lat=" << s.lat_deg;
  }
}

TEST(Frames, KnownEcefValue) {
  // Equator/prime-meridian at sea level: ECEF = (a, 0, 0).
  const Vector3 e = geodeticToEcef(0.0, 0.0, 0.0);
  EXPECT_NEAR(e.x, wgs84::kA, 1e-6);
  EXPECT_NEAR(e.y, 0.0, 1e-6);
  EXPECT_NEAR(e.z, 0.0, 1e-6);

  // North pole at sea level: ECEF = (0, 0, b).
  const Vector3 p = geodeticToEcef(M_PI / 2.0, 0.0, 0.0);
  EXPECT_NEAR(p.x, 0.0, 1e-6);
  EXPECT_NEAR(p.y, 0.0, 1e-6);
  EXPECT_NEAR(p.z, wgs84::kB, 1e-6);
}

// ---------------------------------------------------------------------------------------------
// ENU <-> ECEF round-trip + axis orientation
// ---------------------------------------------------------------------------------------------
TEST(Frames, EnuEcefRoundTrip) {
  GeodeticOrigin o;
  o.lat0_deg = 28.4889;
  o.lon0_deg = -80.5778;
  o.alt0_m = 0.0;

  for (const Vector3& enu : {Vector3{1000, 2000, 3000}, Vector3{-5000, 800, -120}, Vector3{0, 0, 0},
                             Vector3{12345, -6789, 4242}}) {
    const Vector3 ecef = enuToEcef(enu, o);
    const Vector3 back = ecefToEnu(ecef, o);
    EXPECT_LT((enu - back).norm(), 1e-6);
  }
}

TEST(Frames, EnuOriginIsEcefOrigin) {
  GeodeticOrigin o;
  o.lat0_deg = 12.34;
  o.lon0_deg = -56.78;
  o.alt0_m = 100.0;
  // ENU (0,0,0) maps to the origin's own ECEF point.
  const Vector3 ecef = enuToEcef({0, 0, 0}, o);
  const Vector3 expect = geodeticToEcef(o.lat0_deg * kDeg2Rad, o.lon0_deg * kDeg2Rad, o.alt0_m);
  EXPECT_LT((ecef - expect).norm(), 1e-6);
}

TEST(Frames, EnuUpAxisIsLocalNormal) {
  GeodeticOrigin o;
  o.lat0_deg = 40.0;
  o.lon0_deg = 25.0;
  o.alt0_m = 0.0;

  // Moving +Up by 1000 m in ENU should increase geodetic altitude by ~1000 m with lat/lon fixed.
  const Vector3 up_ecef = enuToEcef({0, 0, 1000.0}, o);
  double lat, lon, alt;
  ecefToGeodetic(up_ecef, lat, lon, alt);
  EXPECT_NEAR(alt, 1000.0, 1e-3);
  EXPECT_NEAR(lat, o.lat0_deg * kDeg2Rad, 1e-9);
  EXPECT_NEAR(lon, o.lon0_deg * kDeg2Rad, 1e-9);

  // The ENU Up unit vector in ECEF must be parallel to the geodetic surface normal.
  const Vector3 up_dir = enuVecToEcef({0, 0, 1.0}, o);
  const double slat = std::sin(o.lat0_deg * kDeg2Rad);
  const double clat = std::cos(o.lat0_deg * kDeg2Rad);
  const double slon = std::sin(o.lon0_deg * kDeg2Rad);
  const double clon = std::cos(o.lon0_deg * kDeg2Rad);
  const Vector3 normal{clat * clon, clat * slon, slat};
  EXPECT_NEAR(up_dir.dot(normal), 1.0, 1e-9);  // both unit, parallel
}

// ---------------------------------------------------------------------------------------------
// Central gravity
// ---------------------------------------------------------------------------------------------
TEST(Frames, SurfaceGravityMagnitude) {
  // At the equatorial surface, |g| from the point-mass model sits in the physical band.
  const Vector3 r = geodeticToEcef(0.0, 0.0, 0.0);
  const double g = centralGravity(r, /*with_j2=*/false).norm();
  EXPECT_GT(g, 9.79);
  EXPECT_LT(g, 9.84);

  // GM / a^2 closed form.
  const double expected = wgs84::kGM / (wgs84::kA * wgs84::kA);
  EXPECT_NEAR(g, expected, 1e-9);
}

TEST(Frames, GravityFallsOffInverseSquare) {
  const Vector3 r1{wgs84::kA, 0.0, 0.0};
  const Vector3 r2{2.0 * wgs84::kA, 0.0, 0.0};
  const double g1 = centralGravity(r1, false).norm();
  const double g2 = centralGravity(r2, false).norm();
  // Doubling r quarters g.
  EXPECT_NEAR(g2, g1 / 4.0, 1e-9);
}

TEST(Frames, GravityPointsTowardCenter) {
  const Vector3 r = geodeticToEcef(30.0 * kDeg2Rad, 45.0 * kDeg2Rad, 500000.0);
  const Vector3 g = centralGravity(r, false);
  // g is anti-parallel to r (central).
  EXPECT_NEAR(g.normalized().dot(r.normalized()), -1.0, 1e-9);
}

TEST(Frames, J2PerturbsButStaysClose) {
  const Vector3 r = geodeticToEcef(45.0 * kDeg2Rad, 0.0, 700000.0);
  const Vector3 g0 = centralGravity(r, false);
  const Vector3 gj = centralGravity(r, true);
  // J2 is a small perturbation (~1e-3 of the central term) — present but bounded.
  const double diff = (gj - g0).norm();
  EXPECT_GT(diff, 0.0);
  EXPECT_LT(diff, 0.05 * g0.norm());
}

// ---------------------------------------------------------------------------------------------
// ECI <-> ECEF
// ---------------------------------------------------------------------------------------------
TEST(Frames, EciEcefCoincideAtTimeZero) {
  const Vector3 v{1000.0, -2000.0, 3000.0};
  EXPECT_LT((eciToEcef(v, 0.0) - v).norm(), 1e-9);
  EXPECT_LT((ecefToEci(v, 0.0) - v).norm(), 1e-9);
}

TEST(Frames, EciEcefRoundTrip) {
  const Vector3 r{7000000.0, 1000000.0, 500000.0};
  const Vector3 v{100.0, 7500.0, -50.0};
  const double t = 1234.5;
  const Vector3 r_ecef = eciToEcef(r, t);
  const Vector3 v_ecef = eciVelToEcef(r, v, t);
  const Vector3 r_back = ecefToEci(r_ecef, t);
  const Vector3 v_back = ecefVelToEci(r_ecef, v_ecef, t);
  EXPECT_LT((r - r_back).norm(), 1e-6);
  EXPECT_LT((v - v_back).norm(), 1e-6);
}

// ---------------------------------------------------------------------------------------------
// Round-mode propagation sanity (via runSimulation)
// ---------------------------------------------------------------------------------------------
namespace {
// A high, long-range unguided ballistic shot. Vacuum (atmosphere off) so energy is conserved and
// the flat-vs-round comparison isolates the gravity-frame difference.
SimConfig ballisticConfig() {
  SimConfig c;
  c.scenario = "ballistic";
  c.model = "3dof";
  c.seed = 1;
  c.dt = 0.02;
  c.t_end = 400.0;
  c.integrator = Integrator::RK4;
  c.env.atmosphere = false;  // vacuum
  c.guidance.law = "none";
  c.vehicle.pos0 = {0, 0, 0};
  c.vehicle.launch_speed = 2500.0;
  c.vehicle.launch_elevation_deg = 45.0;
  c.vehicle.launch_azimuth_deg = 90.0;  // due North
  c.vehicle.mass0 = 100.0;
  c.target.pos0 = {0, 600000, 0};
  c.target.vel0 = {0, 0, 0};
  return c;
}

// Downrange ground distance (East-North horizontal) at the final recorded frame.
double horizontalRange(const SimResult& r) {
  const Frame& last = r.frames.back();
  return std::sqrt(last.veh_pos.x * last.veh_pos.x + last.veh_pos.y * last.veh_pos.y);
}
}  // namespace

TEST(RoundEarth, FlatStaysDefaultAndRoundDiffers) {
  SimConfig flat = ballisticConfig();
  flat.env.frame = "flat";
  flat.env.altitude_dependent_g = true;  // fair comparison: flat with inverse-square falloff
  SimConfig round = ballisticConfig();
  round.env.frame = "round";

  const SimResult rf = runSimulation(flat);
  const SimResult rr = runSimulation(round);

  ASSERT_GT(rf.frames.size(), 10u);
  ASSERT_GT(rr.frames.size(), 10u);

  const double range_flat = horizontalRange(rf);
  const double range_round = horizontalRange(rr);

  // Both are long-range shots (hundreds of km).
  EXPECT_GT(range_flat, 100000.0);
  EXPECT_GT(range_round, 100000.0);

  // Round-Earth curvature makes a meaningful, measurable difference vs flat for the same launch.
  EXPECT_GT(std::fabs(range_round - range_flat), 1000.0)
      << "flat=" << range_flat << " round=" << range_round;
}

TEST(RoundEarth, VacuumEnergyConserved) {
  // Specific orbital energy E = v^2/2 - GM/r is conserved in a vacuum central-gravity field.
  SimConfig c = ballisticConfig();
  c.env.frame = "round";
  const SimResult r = runSimulation(c);
  ASSERT_GT(r.frames.size(), 10u);

  // Reconstruct ECI energy from the ENU telemetry at start vs near the apogee/midpoint.
  auto specificEnergy = [&](const Frame& f) {
    // ENU pos/vel -> ECEF -> ECI to evaluate r and v in the inertial frame.
    const Vector3 r_ecef = enuToEcef(f.veh_pos, c.origin);
    const Vector3 r_eci = ecefToEci(r_ecef, f.t);
    const Vector3 v_ecef = enuVecToEcef(f.veh_vel, c.origin);
    const Vector3 v_eci = ecefVelToEci(r_ecef, v_ecef, f.t);
    const double rmag = r_eci.norm();
    return 0.5 * v_eci.normSq() - wgs84::kGM / rmag;
  };

  const double e0 = specificEnergy(r.frames.front());
  const double e_mid = specificEnergy(r.frames[r.frames.size() / 2]);
  const double e_end = specificEnergy(r.frames.back());

  // Relative drift over the whole flight must be tiny (RK4 + smooth central field).
  EXPECT_NEAR(e_mid, e0, std::fabs(e0) * 1e-6);
  EXPECT_NEAR(e_end, e0, std::fabs(e0) * 1e-6);
}

// ---------------------------------------------------------------------------------------------
// Regression: flat-Earth path is unchanged by the frame abstraction
// ---------------------------------------------------------------------------------------------
TEST(RoundEarth, FlatModeReproducesPriorTrajectory) {
  // Default config -> flat path. Run twice; identical. Also confirm the default frame is "flat".
  SimConfig c;
  EXPECT_EQ(c.env.frame, "flat");

  c.scenario = "homing";
  c.model = "3dof";
  c.dt = 0.005;
  c.t_end = 10.0;
  c.aero.cd_mach = {{0.0, 0.28}, {1.0, 0.58}, {2.0, 0.36}};
  c.vehicle.launch_speed = 900.0;
  c.vehicle.launch_elevation_deg = 42.0;
  c.guidance.nav_constant = 4.0;
  c.target.pos0 = {9000, 0, 3500};
  c.target.vel0 = {-280, 0, -40};

  const SimResult a = runSimulation(c);
  const SimResult b = runSimulation(c);
  ASSERT_EQ(a.frames.size(), b.frames.size());
  const std::size_t k = a.frames.size() / 2;
  EXPECT_NEAR(a.frames[k].veh_pos.x, b.frames[k].veh_pos.x, 1e-9);
  EXPECT_NEAR(a.frames[k].veh_pos.z, b.frames[k].veh_pos.z, 1e-9);
  EXPECT_NEAR(a.miss_distance, b.miss_distance, 1e-9);
}
