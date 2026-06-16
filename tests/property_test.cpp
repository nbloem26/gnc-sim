// gnc-sim — property-based / invariant tests (issue #51).
//
// These assert *properties* that must hold across a swept space of seeded inputs, rather than a
// single hand-picked case. Randomness is driven by the project's own `gncsim::Rng` (a standardized
// mt19937_64 + Box-Muller) with FIXED seeds, NOT std distributions, so every case is reproducible
// and bit-identical across libstdc++/libc++ — the same determinism guarantee the rest of the suite
// relies on. No I/O, no clocks, no nondeterminism: nothing here can flake.
//
// Properties covered:
//   * Frame round-trips      — geodetic<->ECEF<->ENU and ECI<->ECEF are exact inverses.
//   * Vacuum energy           — specific orbital energy is conserved under central gravity.
//   * Run determinism         — same (config, seed) => byte-identical telemetry + miss.
//   * CPA monotonicity        — reported miss (analytic CPA) <= every sampled range, and the CPA
//                               is sub-dt (<=) the nearest recorded sample.
//   * Seed independence       — different seeds with sensor noise generally diverge (RNG is live).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "gncsim/core/Config.hpp"
#include "gncsim/core/Rng.hpp"
#include "gncsim/env/Frames.hpp"
#include "gncsim/math/Vector3.hpp"
#include "gncsim/scenario/Runner.hpp"

using namespace gncsim;

namespace {

constexpr double kDeg2Rad = M_PI / 180.0;
constexpr int kNumCases = 64;  // swept cases per property

// A deterministic homing config the property sweeps perturb. Built in-code (no file/cwd).
SimConfig baseHomingConfig() {
  SimConfig c;
  c.scenario = "homing";
  c.model = "3dof";
  c.seed = 1;
  c.dt = 0.005;
  c.t_end = 40.0;
  c.aero.ref_area = 0.02;
  c.aero.cd_mach = {{0.0, 0.28}, {0.9, 0.42}, {1.0, 0.58}, {2.0, 0.36}, {4.0, 0.27}};
  c.vehicle.launch_speed = 900.0;
  c.vehicle.launch_elevation_deg = 42.0;
  c.vehicle.mass0 = 22.0;
  c.guidance.nav_constant = 4.0;
  c.guidance.max_accel = 400.0;
  c.target.pos0 = {9000, 0, 3500};
  c.target.vel0 = {-280, 0, -40};
  return c;
}

// Minimum vehicle->target range over all recorded frames [m].
double minSampledRange_m(const SimResult& r) {
  double m = std::numeric_limits<double>::infinity();
  for (const Frame& f : r.frames) {
    const double range_m = (f.tgt_pos - f.veh_pos).norm();
    if (range_m < m) m = range_m;
  }
  return m;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// Property: frame conversions round-trip exactly, over an Rng-swept space of points/sites.
// ---------------------------------------------------------------------------------------------
TEST(Property, GeodeticEcefEnuRoundTrip) {
  Rng rng(0xC0FFEE);  // fixed seed -> reproducible sweep
  for (int i = 0; i < kNumCases; ++i) {
    GeodeticOrigin origin;
    origin.lat0_deg = rng.uniform(-89.0, 89.0);
    origin.lon0_deg = rng.uniform(-179.0, 179.0);
    origin.alt0_m = rng.uniform(-200.0, 12000.0);

    const Vector3 enu{rng.uniform(-5e4, 5e4), rng.uniform(-5e4, 5e4), rng.uniform(-5e3, 5e4)};

    // ENU -> ECEF -> ENU is an exact inverse.
    const Vector3 ecef = enuToEcef(enu, origin);
    const Vector3 enu_back = ecefToEnu(ecef, origin);
    EXPECT_LT((enu - enu_back).norm(), 1e-4)
        << "case " << i << " lat=" << origin.lat0_deg << " lon=" << origin.lon0_deg;

    // The same ECEF point round-trips through geodetic to mm precision (compared in metres,
    // robust near the poles where longitude is ill-conditioned).
    double lat_rad, lon_rad, alt_m;
    ecefToGeodetic(ecef, lat_rad, lon_rad, alt_m);
    const Vector3 ecef_back = geodeticToEcef(lat_rad, lon_rad, alt_m);
    EXPECT_LT((ecef - ecef_back).norm(), 1e-3) << "case " << i;
  }
}

TEST(Property, EciEcefRoundTrip) {
  Rng rng(0xBADF00D);
  for (int i = 0; i < kNumCases; ++i) {
    const Vector3 r_eci{rng.uniform(-8e6, 8e6), rng.uniform(-8e6, 8e6), rng.uniform(-8e6, 8e6)};
    const Vector3 v_eci{rng.uniform(-8e3, 8e3), rng.uniform(-8e3, 8e3), rng.uniform(-8e3, 8e3)};
    const double t_s = rng.uniform(0.0, 6000.0);

    const Vector3 r_ecef = eciToEcef(r_eci, t_s);
    const Vector3 v_ecef = eciVelToEcef(r_eci, v_eci, t_s);
    const Vector3 r_back = ecefToEci(r_ecef, t_s);
    const Vector3 v_back = ecefVelToEci(r_ecef, v_ecef, t_s);

    EXPECT_LT((r_eci - r_back).norm(), 1e-4) << "case " << i;
    EXPECT_LT((v_eci - v_back).norm(), 1e-6) << "case " << i;
  }
}

// ---------------------------------------------------------------------------------------------
// Property: in a vacuum central-gravity field, specific orbital energy E = v^2/2 - GM/r is
// invariant along the trajectory. Swept over launch speed/elevation/azimuth.
// ---------------------------------------------------------------------------------------------
TEST(Property, VacuumEnergyConserved) {
  Rng rng(0x5EED5);
  for (int i = 0; i < 16;
       ++i) {  // each case integrates a full ballistic arc — keep the count modest
    SimConfig c;
    c.scenario = "ballistic";
    c.model = "3dof";
    c.seed = 1;
    c.dt = 0.02;
    c.t_end = 300.0;
    c.integrator = Integrator::RK4;
    c.env.frame = "round";
    c.env.atmosphere = false;  // vacuum
    c.guidance.law = "none";
    c.vehicle.pos0 = {0, 0, 0};
    c.vehicle.launch_speed = rng.uniform(1500.0, 3000.0);
    c.vehicle.launch_elevation_deg = rng.uniform(30.0, 70.0);
    c.vehicle.launch_azimuth_deg = rng.uniform(0.0, 360.0);
    c.vehicle.mass0 = 100.0;
    c.target.pos0 = {0, 600000, 0};
    c.target.vel0 = {0, 0, 0};

    const SimResult r = runSimulation(c);
    ASSERT_GT(r.frames.size(), 10u) << "case " << i;

    auto specificEnergy = [&](const Frame& f) {
      const Vector3 r_ecef = enuToEcef(f.veh_pos, c.origin);
      const Vector3 r_eci = ecefToEci(r_ecef, f.t);
      const Vector3 v_ecef = enuVecToEcef(f.veh_vel, c.origin);
      const Vector3 v_eci = ecefVelToEci(r_ecef, v_ecef, f.t);
      return 0.5 * v_eci.normSq() - wgs84::kGM / r_eci.norm();
    };

    const double e0 = specificEnergy(r.frames.front());
    const double e_mid = specificEnergy(r.frames[r.frames.size() / 2]);
    const double e_end = specificEnergy(r.frames.back());
    EXPECT_NEAR(e_mid, e0, std::fabs(e0) * 1e-6) << "case " << i;
    EXPECT_NEAR(e_end, e0, std::fabs(e0) * 1e-6) << "case " << i;
  }
}

// ---------------------------------------------------------------------------------------------
// Property: a run is a pure function of (config, seed). Repeating any swept config yields
// byte-identical telemetry and miss. This is the native half of the native<->WASM parity contract.
// ---------------------------------------------------------------------------------------------
TEST(Property, RunIsDeterministic) {
  Rng rng(0xD37E12);
  for (int i = 0; i < kNumCases; ++i) {
    SimConfig c = baseHomingConfig();
    c.seed = static_cast<std::uint64_t>(rng.uniform(1.0, 1e6));
    c.sensors.enable = true;  // exercise the live RNG path
    c.sensors.seeker.los_white = rng.uniform(1e-3, 5e-3);
    c.sensors.imu.gyro_white = rng.uniform(5e-5, 2e-4);
    c.vehicle.launch_elevation_deg = rng.uniform(35.0, 50.0);
    c.guidance.nav_constant = rng.uniform(3.0, 5.0);

    const SimResult a = runSimulation(c);
    const SimResult b = runSimulation(c);
    ASSERT_EQ(a.frames.size(), b.frames.size()) << "case " << i;
    EXPECT_DOUBLE_EQ(a.miss_distance, b.miss_distance) << "case " << i;
    const std::size_t k = a.frames.size() / 2;
    EXPECT_DOUBLE_EQ(a.frames[k].veh_pos.x, b.frames[k].veh_pos.x) << "case " << i;
    EXPECT_DOUBLE_EQ(a.frames[k].veh_pos.z, b.frames[k].veh_pos.z) << "case " << i;
    EXPECT_DOUBLE_EQ(a.frames[k].seeker_los_meas, b.frames[k].seeker_los_meas) << "case " << i;
  }
}

// ---------------------------------------------------------------------------------------------
// Property: the reported miss distance is the analytic closest-point-of-approach, so it must be
// <= every sampled range (CPA is sub-dt accurate and can only be tighter than the nearest sample).
// ---------------------------------------------------------------------------------------------
TEST(Property, CpaNeverExceedsSampledRange) {
  Rng rng(0xCDA001);
  for (int i = 0; i < kNumCases; ++i) {
    SimConfig c = baseHomingConfig();
    c.seed = static_cast<std::uint64_t>(rng.uniform(1.0, 1e6));
    c.vehicle.launch_speed = rng.uniform(700.0, 1100.0);
    c.vehicle.launch_elevation_deg = rng.uniform(35.0, 50.0);
    c.guidance.nav_constant = rng.uniform(3.0, 5.0);
    c.target.pos0 = {rng.uniform(7000.0, 11000.0), rng.uniform(-1500.0, 1500.0),
                     rng.uniform(2500.0, 4500.0)};

    const SimResult r = runSimulation(c);
    ASSERT_GT(r.frames.size(), 2u) << "case " << i;

    const double min_sampled_m = minSampledRange_m(r);
    // miss (analytic CPA) is the running minimum of sub-dt CPAs; it can only be <= the best sample
    // (allow a tiny epsilon for floating-point comparison of two independently computed minima).
    EXPECT_LE(r.miss_distance, min_sampled_m + 1e-6)
        << "case " << i << " miss=" << r.miss_distance << " min_sample=" << min_sampled_m;
    EXPECT_GE(r.miss_distance, 0.0) << "case " << i;
  }
}

// ---------------------------------------------------------------------------------------------
// Property: with sensor noise live, different seeds generally produce different outcomes — the
// RNG is genuinely wired in (a guard against an accidentally dead noise path that would make the
// determinism test vacuously true).
// ---------------------------------------------------------------------------------------------
TEST(Property, DistinctSeedsGenerallyDiverge) {
  SimConfig base = baseHomingConfig();
  base.sensors.enable = true;
  base.sensors.seeker.los_white = 4e-3;
  base.sensors.imu.gyro_white = 1.5e-4;

  Rng rng(0x53EED0);
  int diverged = 0;
  for (int i = 0; i < kNumCases; ++i) {
    SimConfig a = base;
    SimConfig b = base;
    a.seed = static_cast<std::uint64_t>(rng.uniform(1.0, 1e9));
    b.seed = static_cast<std::uint64_t>(rng.uniform(1.0, 1e9));
    if (a.seed == b.seed) continue;
    const SimResult ra = runSimulation(a);
    const SimResult rb = runSimulation(b);
    if (std::fabs(ra.miss_distance - rb.miss_distance) > 1e-9) ++diverged;
  }
  // The vast majority of distinct-seed pairs must differ. (Not all: an easy intercept can drive
  // both misses to ~0 regardless of noise.)
  EXPECT_GT(diverged, kNumCases / 2);
}
