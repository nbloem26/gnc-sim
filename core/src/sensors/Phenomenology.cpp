// gnc-sim — sensor phenomenology implementation (issue #39). See Phenomenology.hpp for the model
// rationale. Pure arithmetic on the project Rng + libm; no std distributions, fixed FP order, so
// native and WASM stay bit-identical.
#include "gncsim/sensors/Phenomenology.hpp"

#include <algorithm>
#include <cmath>

namespace gncsim {

namespace {

// dB (power ratio) -> linear.
double dbToLinear(double db) { return std::pow(10.0, 0.1 * db); }

// One uniform in (0,1], clamped away from 0 so log() is finite.
double safeUniform(Rng& rng) {
  double u = rng.uniform(0.0, 1.0);
  if (u < 1e-300) u = 1e-300;
  return u;
}

// One unit-mean exponential draw, -ln(U). (chi-square with 2 dof, scaled.)
double exponentialUnitMean(Rng& rng) { return -std::log(safeUniform(rng)); }

}  // namespace

// ---------------------------------------------------------------------------------------------
// Swerling RCS
// ---------------------------------------------------------------------------------------------

SwerlingCase swerlingFromInt(int n) {
  switch (n) {
    case 0:
      return SwerlingCase::Zero;
    case 2:
      return SwerlingCase::Two;
    case 3:
      return SwerlingCase::Three;
    case 4:
      return SwerlingCase::Four;
    default:
      return SwerlingCase::One;  // 1 (and any out-of-range value) -> Swerling I
  }
}

double swerlingRcsSample(SwerlingCase sw, double rcs_mean_m2, Rng& rng) {
  if (rcs_mean_m2 <= 0.0) return 0.0;
  switch (sw) {
    case SwerlingCase::Zero:
      // Non-fluctuating: constant cross section.
      return rcs_mean_m2;
    case SwerlingCase::One:
    case SwerlingCase::Two:
      // Exponential (chi-square, 2 dof). Single-look amplitude distribution is identical for I/II.
      return rcs_mean_m2 * exponentialUnitMean(rng);
    case SwerlingCase::Three:
    case SwerlingCase::Four:
      // chi-square with 4 dof = sum of two unit-mean exponentials, mean preserved by /2.
      return rcs_mean_m2 * 0.5 * (exponentialUnitMean(rng) + exponentialUnitMean(rng));
  }
  return rcs_mean_m2;
}

// ---------------------------------------------------------------------------------------------
// Radar SNR
// ---------------------------------------------------------------------------------------------

double radarSnrLinear(const RadarPhenomenologyConfig& cfg, double range_m, double rcs_m2) {
  const double r = std::max(range_m, 1.0);
  const double r_ref = std::max(cfg.range_ref_m, 1.0);
  const double rcs_ref = std::max(cfg.rcs_ref_m2, 1e-12);
  const double snr_ref = dbToLinear(cfg.snr_ref_db);
  // Range equation scaling about the anchor: SNR ∝ sigma / R^4.
  const double r_ratio = r_ref / r;
  const double r4 = r_ratio * r_ratio * r_ratio * r_ratio;
  double snr = snr_ref * (rcs_m2 / rcs_ref) * r4;
  // Clutter + barrage-noise jamming raise the effective noise floor: divide SNR by (1 + CNR + JNR).
  double interference = 1.0;
  if (cfg.clutter_cnr_db > -90.0) interference += dbToLinear(cfg.clutter_cnr_db);
  if (cfg.jammer_jnr_db > -90.0) interference += dbToLinear(cfg.jammer_jnr_db);
  return snr / interference;
}

// ---------------------------------------------------------------------------------------------
// IR SNR
// ---------------------------------------------------------------------------------------------

double irSnrLinear(const IrPhenomenologyConfig& cfg, double range_m) {
  const double r = std::max(range_m, 1.0);
  const double r_ref = std::max(cfg.range_ref_m, 1.0);
  const double netd = std::max(cfg.netd_k, 1e-9);
  // Inverse-square apparent-intensity falloff relative to the reference range.
  const double r_ratio = r_ref / r;
  const double geom = r_ratio * r_ratio;
  // Beer-Lambert atmospheric transmission relative to the reference range (so the contrast anchor
  // is the transmitted contrast at range_ref).
  const double tau = std::exp(-cfg.atm_extinction_per_m * (r - r_ref));
  const double contrast_k = cfg.target_contrast_k * geom * tau;
  return contrast_k / netd;
}

double irAngleSigmaRad(const IrPhenomenologyConfig& cfg, double snr_linear) {
  const double snr = std::max(snr_linear, 1e-6);
  const double k = std::max(cfg.centroid_gain, 1e-6);
  const double sigma = cfg.theta_resolution_rad / (k * std::sqrt(snr));
  // Floor at a small fraction of a pixel so a very strong signal still has nonzero noise.
  return std::max(sigma, 1e-3 * cfg.theta_resolution_rad);
}

// ---------------------------------------------------------------------------------------------
// CA-CFAR
// ---------------------------------------------------------------------------------------------

double cfarAlpha(const CfarConfig& cfg) {
  const int n = std::max(cfg.num_ref_cells, 1);
  double pfa = cfg.pfa;
  if (pfa <= 0.0) pfa = 1e-12;
  if (pfa >= 1.0) pfa = 1.0 - 1e-12;
  // Gandhi & Kassam CA-CFAR threshold multiplier: alpha = N (Pfa^(-1/N) - 1).
  return static_cast<double>(n) * (std::pow(pfa, -1.0 / static_cast<double>(n)) - 1.0);
}

double cfarPdSwerling(const CfarConfig& cfg, double snr_linear) {
  const int n = std::max(cfg.num_ref_cells, 1);
  const double alpha = cfarAlpha(cfg);
  const double snr = std::max(snr_linear, 0.0);
  // Gandhi & Kassam square-law CA-CFAR closed form (Swerling I/II): with the cell-average noise
  // estimate normalized by N, Pfa = (1 + alpha/N)^(-N) and Pd = (1 + alpha/(N(1+SNR)))^(-N). At
  // SNR = 0 this reduces exactly to Pfa (the cell-under-test is statistically a reference cell).
  const double base = 1.0 + alpha / (static_cast<double>(n) * (1.0 + snr));
  return std::pow(base, -static_cast<double>(n));
}

CfarResult cfarDetect(const CfarConfig& cfg, double snr_linear, Rng& rng) {
  CfarResult out;
  out.pfa = std::clamp(cfg.pfa, 0.0, 1.0);
  out.pd = cfarPdSwerling(cfg, snr_linear);
  out.snr_db = 10.0 * std::log10(std::max(snr_linear, 1e-12));
  // Bernoulli draw from one uniform: detected iff U < Pd.
  out.detected = rng.uniform(0.0, 1.0) < out.pd;
  return out;
}

}  // namespace gncsim
