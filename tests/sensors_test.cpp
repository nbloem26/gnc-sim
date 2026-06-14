// gnc-sim — Phase 1 sensor error-model tests: IMU (white / Gauss-Markov bias / RRW / scale)
// and seeker (bias / white / range-dependent glint). The noise convention under test is the
// contract documented in core/src/sensors/Sensors.cpp; the Allan-variance loop-closure relies
// on it. All draws are seeded for determinism. World units are SI (accel m/s^2, gyro rad/s,
// LOS rad).
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "gncsim/core/Config.hpp"
#include "gncsim/core/Rng.hpp"
#include "gncsim/math/Vector3.hpp"
#include "gncsim/sensors/Sensors.hpp"

using namespace gncsim;

namespace {

constexpr double kDt = 0.005;  // matches SimConfig::dt default

// A fully zeroed IMU noise config (every error source disabled).
ImuNoise zeroImu() {
  ImuNoise c;  // struct defaults are already all-zero except the tau fields
  c.accel_white = 0.0;
  c.accel_bias_instability = 0.0;
  c.accel_bias_tau = 100.0;
  c.accel_rrw = 0.0;
  c.accel_scale_factor = 0.0;
  c.gyro_white = 0.0;
  c.gyro_bias_instability = 0.0;
  c.gyro_bias_tau = 100.0;
  c.gyro_rrw = 0.0;
  c.gyro_scale_factor = 0.0;
  return c;
}

SeekerNoise zeroSeeker() {
  SeekerNoise c;
  c.los_white = 0.0;
  c.los_bias = 0.0;
  c.glint = 0.0;
  return c;
}

// Sample standard deviation (population/unbiased N-1) of a scalar series.
double sampleStd(const std::vector<double>& v) {
  double mean = 0.0;
  for (double x : v) mean += x;
  mean /= static_cast<double>(v.size());
  double acc = 0.0;
  for (double x : v) acc += (x - mean) * (x - mean);
  return std::sqrt(acc / static_cast<double>(v.size() - 1));
}

}  // namespace

// ── A) Zero-noise config: measured == true exactly ──────────────────────────────────────────

TEST(ImuTest, ZeroNoiseIsExactPassthrough) {
  Rng rng(42);
  Imu imu(zeroImu(), kDt, rng);

  const Vector3 a_true{1.0, -2.0, 9.81};
  const Vector3 g_true{0.1, 0.2, -0.3};
  Vector3 a_meas, g_meas;

  // Run several steps — with all params zero the held bias must stay exactly zero.
  for (int i = 0; i < 100; ++i) {
    imu.measure(a_true, g_true, a_meas, g_meas);
    EXPECT_DOUBLE_EQ(a_meas.x, a_true.x);
    EXPECT_DOUBLE_EQ(a_meas.y, a_true.y);
    EXPECT_DOUBLE_EQ(a_meas.z, a_true.z);
    EXPECT_DOUBLE_EQ(g_meas.x, g_true.x);
    EXPECT_DOUBLE_EQ(g_meas.y, g_true.y);
    EXPECT_DOUBLE_EQ(g_meas.z, g_true.z);
  }
}

TEST(SeekerTest, ZeroNoiseIsExactPassthrough) {
  Rng rng(42);
  Seeker seeker(zeroSeeker(), rng);

  // Exact at any range (including small range where glint would otherwise grow).
  EXPECT_DOUBLE_EQ(seeker.measureLos(0.123, 5000.0), 0.123);
  EXPECT_DOUBLE_EQ(seeker.measureLos(-0.05, 1e6), -0.05);
  EXPECT_DOUBLE_EQ(seeker.measureLos(0.0, 0.5), 0.0);
}

// ── B) Bias-only (GM): nonzero, bounded, not diverging ──────────────────────────────────────

TEST(ImuTest, BiasInstabilityIsBoundedAndNonzero) {
  // white=0, rrw=0, scale=0; only the Gauss-Markov bias instability is active with a large tau.
  ImuNoise c = zeroImu();
  c.accel_bias_instability = 5.0e-4;  // [m/s^2] steady-state std
  c.accel_bias_tau = 100.0;           // large relative to dt -> slow correlated walk

  Rng rng(7);
  Imu imu(c, kDt, rng);

  const Vector3 a_true{2.0, 0.0, 0.0};
  const Vector3 g_true{0.0, 0.0, 0.0};
  Vector3 a_meas, g_meas;

  const int N = 50000;
  double running_mean_x = 0.0;
  bool saw_nonzero = false;
  for (int i = 0; i < N; ++i) {
    imu.measure(a_true, g_true, a_meas, g_meas);
    const double err = a_meas.x - a_true.x;  // pure bias contribution (no white/scale)
    running_mean_x += err;
    if (std::fabs(err) > 1e-12) saw_nonzero = true;
  }
  running_mean_x /= static_cast<double>(N);

  // The GM process actually perturbs the measurement.
  EXPECT_TRUE(saw_nonzero);

  // Sanity bound: a stationary GM process stays O(bias_instability); the time-average is even
  // smaller. A few-sigma bound confirms it is not diverging (which a pure RRW would).
  EXPECT_LT(std::fabs(running_mean_x), 5.0 * c.accel_bias_instability);
}

// ── C) Scale-factor only: exact deterministic scaling ───────────────────────────────────────

TEST(ImuTest, ScaleFactorOnlyIsExactScaling) {
  ImuNoise c = zeroImu();
  c.accel_scale_factor = 1.0e-3;
  c.gyro_scale_factor = 5.0e-4;

  Rng rng(99);
  Imu imu(c, kDt, rng);

  const Vector3 a_true{3.0, -4.0, 5.0};
  const Vector3 g_true{0.6, -0.7, 0.8};
  Vector3 a_meas, g_meas;

  for (int i = 0; i < 10; ++i) {
    imu.measure(a_true, g_true, a_meas, g_meas);
    EXPECT_DOUBLE_EQ(a_meas.x, a_true.x * (1.0 + c.accel_scale_factor));
    EXPECT_DOUBLE_EQ(a_meas.y, a_true.y * (1.0 + c.accel_scale_factor));
    EXPECT_DOUBLE_EQ(a_meas.z, a_true.z * (1.0 + c.accel_scale_factor));
    EXPECT_DOUBLE_EQ(g_meas.x, g_true.x * (1.0 + c.gyro_scale_factor));
    EXPECT_DOUBLE_EQ(g_meas.y, g_true.y * (1.0 + c.gyro_scale_factor));
    EXPECT_DOUBLE_EQ(g_meas.z, g_true.z * (1.0 + c.gyro_scale_factor));
  }
}

// ── D) White-only: sample std ≈ white/sqrt(dt) within 5% ─────────────────────────────────────

TEST(ImuTest, WhiteNoiseSampleStdMatchesDensity) {
  ImuNoise c = zeroImu();
  c.accel_white = 8.0e-4;  // (m/s^2)/sqrt(Hz)
  c.accel_bias_tau = -1.0;  // disable GM explicitly (bias_instability is already 0 anyway)

  Rng rng(2024);
  Imu imu(c, kDt, rng);

  const Vector3 a_true{1.5, 0.0, 0.0};  // constant input
  const Vector3 g_true{0.0, 0.0, 0.0};
  Vector3 a_meas, g_meas;

  const int N = 20000;
  std::vector<double> err;
  err.reserve(N);
  for (int i = 0; i < N; ++i) {
    imu.measure(a_true, g_true, a_meas, g_meas);
    err.push_back(a_meas.x - a_true.x);  // pure white draw (no bias, no scale)
  }

  const double expected = c.accel_white / std::sqrt(kDt);
  const double measured = sampleStd(err);
  EXPECT_NEAR(measured, expected, 0.05 * expected);
}

TEST(SeekerTest, WhiteNoiseSampleStdMatchesParam) {
  SeekerNoise c = zeroSeeker();
  c.los_white = 1.0e-3;  // [rad]

  Rng rng(31337);
  Seeker seeker(c, rng);

  const int N = 20000;
  std::vector<double> err;
  err.reserve(N);
  const double los_true = 0.02;
  for (int i = 0; i < N; ++i) {
    // Large range so glint (which is zero here anyway) cannot contaminate the white estimate.
    err.push_back(seeker.measureLos(los_true, 1.0e6) - los_true);
  }
  EXPECT_NEAR(sampleStd(err), c.los_white, 0.05 * c.los_white);
}

// ── E) Determinism: same seed + inputs -> identical measurements ─────────────────────────────

TEST(ImuTest, DeterministicForSameSeed) {
  // Use the full placeholder param set so every noise path is exercised.
  ImuNoise c = zeroImu();
  c.accel_white = 8.0e-4;
  c.accel_bias_instability = 5.0e-4;
  c.accel_bias_tau = 100.0;
  c.accel_rrw = 1.0e-5;
  c.accel_scale_factor = 1.0e-3;
  c.gyro_white = 9.0e-5;
  c.gyro_bias_instability = 1.5e-5;
  c.gyro_bias_tau = 100.0;
  c.gyro_rrw = 2.0e-7;
  c.gyro_scale_factor = 5.0e-4;

  Rng rng_a(12345);
  Rng rng_b(12345);
  Imu imu_a(c, kDt, rng_a);
  Imu imu_b(c, kDt, rng_b);

  const Vector3 a_true{1.0, 2.0, 3.0};
  const Vector3 g_true{0.01, 0.02, 0.03};
  Vector3 a_a, g_a, a_b, g_b;

  for (int i = 0; i < 1000; ++i) {
    imu_a.measure(a_true, g_true, a_a, g_a);
    imu_b.measure(a_true, g_true, a_b, g_b);
    EXPECT_DOUBLE_EQ(a_a.x, a_b.x);
    EXPECT_DOUBLE_EQ(a_a.y, a_b.y);
    EXPECT_DOUBLE_EQ(a_a.z, a_b.z);
    EXPECT_DOUBLE_EQ(g_a.x, g_b.x);
    EXPECT_DOUBLE_EQ(g_a.y, g_b.y);
    EXPECT_DOUBLE_EQ(g_a.z, g_b.z);
  }
}

TEST(SeekerTest, DeterministicForSameSeed) {
  SeekerNoise c = zeroSeeker();
  c.los_white = 1.0e-3;
  c.los_bias = 5.0e-4;
  c.glint = 0.2;

  Rng rng_a(555);
  Rng rng_b(555);
  Seeker seeker_a(c, rng_a);
  Seeker seeker_b(c, rng_b);

  for (int i = 0; i < 1000; ++i) {
    const double range = 100.0 + i;  // sweep range so glint scaling varies
    EXPECT_DOUBLE_EQ(seeker_a.measureLos(0.02, range), seeker_b.measureLos(0.02, range));
  }
}

// ── F) Seeker glint grows as range -> 0 (supporting the contract's range dependence) ─────────

TEST(SeekerTest, GlintGrowsAsRangeShrinks) {
  SeekerNoise c = zeroSeeker();
  c.glint = 0.2;  // only glint active

  const int N = 20000;
  const double los_true = 0.0;

  auto glintStd = [&](double range) {
    Rng rng(808);
    Seeker seeker(c, rng);
    std::vector<double> err;
    err.reserve(N);
    for (int i = 0; i < N; ++i) err.push_back(seeker.measureLos(los_true, range) - los_true);
    return sampleStd(err);
  };

  const double std_far = glintStd(1000.0);   // expected ~ glint/1000
  const double std_near = glintStd(10.0);    // expected ~ glint/10
  EXPECT_GT(std_near, std_far);

  // Match the contract std = glint/max(range,1) within 5%.
  EXPECT_NEAR(std_far, c.glint / 1000.0, 0.05 * (c.glint / 1000.0));
  EXPECT_NEAR(std_near, c.glint / 10.0, 0.05 * (c.glint / 10.0));
}
