// gnc-sim — sensor phenomenology tests (issue #39). Statistical benchmarks over a seeded ensemble:
//   * CA-CFAR holds the configured Pfa on noise-only (SNR = 0) and achieves the closed-form Pd at a
//     given SNR;
//   * Swerling RCS samples reproduce the case mean and (for the exponential cases) variance;
//   * IR NETD/atmosphere maps range to SNR to angular noise as configured;
//   * the detector decision is deterministic for a fixed seed.
#include "gncsim/sensors/Phenomenology.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "gncsim/core/Rng.hpp"

namespace gncsim {
namespace {

// Ensemble fraction of CFAR detections over N independent looks at a fixed SNR.
double detectionRate(const CfarConfig& cfg, double snr_linear, int n, std::uint64_t seed) {
  Rng rng(seed);
  int hits = 0;
  for (int i = 0; i < n; ++i) {
    if (cfarDetect(cfg, snr_linear, rng).detected) ++hits;
  }
  return static_cast<double>(hits) / static_cast<double>(n);
}

TEST(Cfar, AlphaMatchesGandhiKassam) {
  CfarConfig cfg;
  cfg.pfa = 1.0e-4;
  cfg.num_ref_cells = 16;
  // alpha = N (Pfa^(-1/N) - 1).
  const double expected = 16.0 * (std::pow(1.0e-4, -1.0 / 16.0) - 1.0);
  EXPECT_NEAR(cfarAlpha(cfg), expected, 1e-9);
  // As N -> large, the CFAR threshold approaches the ideal -ln(Pfa).
  CfarConfig big = cfg;
  big.num_ref_cells = 4000;
  EXPECT_NEAR(cfarAlpha(big), -std::log(1.0e-4), 0.2);
}

TEST(Cfar, AchievesConfiguredPfaOnNoiseOnly) {
  // Noise-only cell-under-test: SNR = 0 -> the closed-form Pd collapses to exactly Pfa.
  CfarConfig cfg;
  cfg.pfa = 1.0e-2;  // a few false alarms per hundred looks, measurable in a finite ensemble
  cfg.num_ref_cells = 24;
  EXPECT_NEAR(cfarPdSwerling(cfg, 0.0), cfg.pfa, 1e-9);
  const double rate = detectionRate(cfg, 0.0, 200000, 12345);
  // Binomial std over 2e5 looks at p=1e-2 is ~2.2e-4; allow a comfortable band.
  EXPECT_NEAR(rate, cfg.pfa, 2.0e-3);
}

TEST(Cfar, AchievesExpectedPdAtSnr) {
  CfarConfig cfg;
  cfg.pfa = 1.0e-4;
  cfg.num_ref_cells = 24;
  // 13 dB single-pulse SNR -> a high but sub-unity detection probability.
  const double snr = std::pow(10.0, 1.3);
  const double pd = cfarPdSwerling(cfg, snr);
  EXPECT_GT(pd, 0.5);
  EXPECT_LT(pd, 1.0);
  const double rate = detectionRate(cfg, snr, 200000, 999);
  EXPECT_NEAR(rate, pd, 5.0e-3);
}

TEST(Cfar, PdIncreasesMonotonicallyWithSnr) {
  CfarConfig cfg;
  double prev = -1.0;
  for (double snr_db = -10.0; snr_db <= 25.0; snr_db += 1.0) {
    const double pd = cfarPdSwerling(cfg, std::pow(10.0, 0.1 * snr_db));
    EXPECT_GE(pd, prev - 1e-12);
    prev = pd;
  }
}

TEST(Swerling, ExponentialCasesMatchMeanAndVariance) {
  Rng rng(7);
  const double mean_rcs = 2.5;
  const int n = 400000;
  double sum = 0.0;
  double sum_sq = 0.0;
  for (int i = 0; i < n; ++i) {
    const double s = swerlingRcsSample(SwerlingCase::One, mean_rcs, rng);
    sum += s;
    sum_sq += s * s;
  }
  const double mean = sum / n;
  const double var = sum_sq / n - mean * mean;
  EXPECT_NEAR(mean, mean_rcs, 0.02);
  // Exponential: variance = mean^2.
  EXPECT_NEAR(var, mean_rcs * mean_rcs, 0.1);
}

TEST(Swerling, ChiSquare4CasesHaveHalfTheRelativeVariance) {
  Rng rng(11);
  const double mean_rcs = 1.0;
  const int n = 400000;
  double sum = 0.0;
  double sum_sq = 0.0;
  for (int i = 0; i < n; ++i) {
    const double s = swerlingRcsSample(SwerlingCase::Three, mean_rcs, rng);
    sum += s;
    sum_sq += s * s;
  }
  const double mean = sum / n;
  const double var = sum_sq / n - mean * mean;
  EXPECT_NEAR(mean, mean_rcs, 0.01);
  // chi-square 4 dof: variance = mean^2 / 2.
  EXPECT_NEAR(var, mean_rcs * mean_rcs * 0.5, 0.05);
}

TEST(Swerling, ZeroCaseIsNonFluctuating) {
  Rng rng(3);
  for (int i = 0; i < 100; ++i) {
    EXPECT_DOUBLE_EQ(swerlingRcsSample(SwerlingCase::Zero, 4.0, rng), 4.0);
  }
}

TEST(RadarSnr, ScalesAsInverseFourthPower) {
  RadarPhenomenologyConfig cfg;
  cfg.range_ref_m = 1.0e4;
  cfg.rcs_ref_m2 = 1.0;
  cfg.snr_ref_db = 20.0;
  // At the reference range with reference RCS, SNR == snr_ref.
  EXPECT_NEAR(radarSnrLinear(cfg, 1.0e4, 1.0), std::pow(10.0, 2.0), 1e-6);
  // Doubling the range cuts SNR by 2^4 = 16.
  const double s_far = radarSnrLinear(cfg, 2.0e4, 1.0);
  EXPECT_NEAR(s_far, std::pow(10.0, 2.0) / 16.0, 1e-6);
  // Doubling the RCS doubles SNR.
  EXPECT_NEAR(radarSnrLinear(cfg, 1.0e4, 2.0), std::pow(10.0, 2.0) * 2.0, 1e-6);
}

TEST(RadarSnr, JammingAndClutterRaiseTheNoiseFloor) {
  RadarPhenomenologyConfig clean;
  RadarPhenomenologyConfig jammed = clean;
  jammed.jammer_jnr_db = 10.0;  // 10x noise power
  const double s_clean = radarSnrLinear(clean, 1.0e4, 1.0);
  const double s_jam = radarSnrLinear(jammed, 1.0e4, 1.0);
  // Interference factor 1 + 10 = 11.
  EXPECT_NEAR(s_clean / s_jam, 11.0, 1e-6);
}

TEST(IrSnr, NetdAndAtmosphereMapToAngularNoise) {
  IrPhenomenologyConfig cfg;
  cfg.netd_k = 0.05;
  cfg.target_contrast_k = 2.0;
  cfg.range_ref_m = 1.0e4;
  cfg.atm_extinction_per_m = 0.0;  // isolate the inverse-square geometry first
  // At the reference range SNR = contrast / NETD = 2.0/0.05 = 40.
  EXPECT_NEAR(irSnrLinear(cfg, 1.0e4), 40.0, 1e-6);
  // Halving the range quadruples SNR (inverse-square).
  EXPECT_NEAR(irSnrLinear(cfg, 5.0e3), 160.0, 1e-6);
  // Atmospheric extinction attenuates farther ranges.
  cfg.atm_extinction_per_m = 1.0e-4;
  EXPECT_LT(irSnrLinear(cfg, 2.0e4), 10.0);  // 2x range: 1/4 geom * exp(-1) atmosphere
  // Stronger signal -> tighter angular centroid noise.
  const double sigma_near = irAngleSigmaRad(cfg, 160.0);
  const double sigma_far = irAngleSigmaRad(cfg, 10.0);
  EXPECT_LT(sigma_near, sigma_far);
  EXPECT_GT(sigma_near, 0.0);
}

TEST(Phenomenology, DetectionIsDeterministicForFixedSeed) {
  CfarConfig cfg;
  cfg.pfa = 1.0e-3;
  cfg.num_ref_cells = 24;
  const double snr = std::pow(10.0, 1.0);
  std::vector<bool> first;
  for (int run = 0; run < 2; ++run) {
    Rng rng(424242);
    std::vector<bool> seq;
    seq.reserve(500);
    for (int i = 0; i < 500; ++i) seq.push_back(cfarDetect(cfg, snr, rng).detected);
    if (run == 0) {
      first = seq;
    } else {
      EXPECT_EQ(first, seq);
    }
  }
}

}  // namespace
}  // namespace gncsim
