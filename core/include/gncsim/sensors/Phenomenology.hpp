// gnc-sim — sensor phenomenology: signal -> detection (issue #39).
//
// The legacy track sensors (TargetTrackEkf / TrackSensorModel) synthesize a measurement as
// "truth + Gaussian noise" and always hand it to the tracker. That is a *measurement* model, not a
// *detection* model: it has no concept of signal strength, false alarms, or missed detections.
//
// This module adds the missing front-end: it turns the engagement geometry into a SIGNAL (radar
// SNR from the range equation with Swerling RCS fluctuation + clutter + noise jamming; IR
// contrast SNR from NETD and atmospheric transmission), runs a Cell-Averaging CFAR detector over a
// noise floor to decide DETECTED / NOT-DETECTED at a controlled false-alarm rate, and — only on a
// detection — produces the az/el[/range/range-rate] measurement the existing TargetTrackEkf already
// knows how to fuse. On a miss the tracker simply coasts (predict-only) that step.
//
// Everything here is pure arithmetic on the project Rng (no std distributions, fixed FP order, libm
// only) so native and WASM stay bit-identical, and it is entirely opt-in: it is reached only when a
// trackers[].type of "radar_pheno" / "ir_pheno" is configured, so every existing config is
// byte-identical.
//
// Units carry SI suffixes per AGENTS.md (rcs_m2, range_m, snr_db, netd_k, doppler_hz, ...).
#pragma once

#include "gncsim/core/Config.hpp"
#include "gncsim/core/Rng.hpp"
#include "gncsim/math/Vector3.hpp"

namespace gncsim {

// ---------------------------------------------------------------------------------------------
// RCS fluctuation — Swerling cases I-IV
// ---------------------------------------------------------------------------------------------
//
// The instantaneous radar cross section fluctuates pulse-to-pulse / scan-to-scan. The four
// classical Swerling cases are exponential (chi-square, 2 dof, cases I/II — many comparable
// scatterers) or chi-square with 4 dof (cases III/IV — one dominant plus many small scatterers),
// each with a configured MEAN rcs_mean_m2. (I vs II and III vs IV differ only by scan-to-scan vs
// pulse-to-pulse decorrelation; the single-look amplitude distribution is the same, which is what
// we sample here.) Swerling 0/V is the non-fluctuating (constant) case.
enum class SwerlingCase { Zero, One, Two, Three, Four };

SwerlingCase swerlingFromInt(int n);

// Draw one instantaneous RCS [m^2] for the given case and mean, using the run Rng (one or two
// uniforms; no std distributions). Mean is preserved in expectation.
double swerlingRcsSample(SwerlingCase sw, double rcs_mean_m2, Rng& rng);

// ---------------------------------------------------------------------------------------------
// Radar signal model — range equation, clutter, ECM
// ---------------------------------------------------------------------------------------------

// Single-pulse signal-to-noise ratio (LINEAR, not dB) from the monostatic radar range equation
//   SNR = (Pt G^2 lambda^2 sigma) / ((4 pi)^3 R^4 k Ts B L)
// folded into a calibrated constant: at the configured reference range_ref_m a target of the
// configured reference RCS yields snr_ref_db. From that anchor, SNR scales as sigma / R^4. Clutter
// and barrage-noise jamming raise the effective noise floor (interference-to-noise ratio), reducing
// SNR. `rcs_m2` is the instantaneous (already Swerling-sampled) cross section.
double radarSnrLinear(const RadarPhenomenologyConfig& cfg, double range_m, double rcs_m2);

// ---------------------------------------------------------------------------------------------
// IR signal model — NETD + atmospheric transmission
// ---------------------------------------------------------------------------------------------

// IR contrast signal-to-noise ratio (LINEAR). The target's apparent radiant intensity falls as
// 1/R^2 (inverse-square) and is attenuated by Beer-Lambert atmospheric transmission
// exp(-beta * R). NETD sets the noise-equivalent contrast; SNR = (contrast at range) / NETD.
double irSnrLinear(const IrPhenomenologyConfig& cfg, double range_m);

// Map an IR detection SNR to the 1-sigma angular measurement noise [rad]: stronger signal ->
// tighter centroid. sigma_angle = theta_resolution / (k * sqrt(SNR)), floored so it never collapses
// to zero.
double irAngleSigmaRad(const IrPhenomenologyConfig& cfg, double snr_linear);

// ---------------------------------------------------------------------------------------------
// CA-CFAR detector
// ---------------------------------------------------------------------------------------------

// Cell-Averaging Constant-False-Alarm-Rate detector. The detector estimates the noise power from
// `num_ref_cells` reference cells and sets the threshold at alpha * noise_estimate, where
//   alpha = N * (Pfa^(-1/N) - 1)              (Gandhi & Kassam; N = number of reference cells)
// so the design false-alarm probability Pfa is held constant regardless of the (unknown) noise
// level. For a square-law detector in exponential (Rayleigh-amplitude) noise the closed-form
// single- pulse detection probability is
//   Pd = (1 + alpha/(N(1+SNR)))^(-N)          (Swerling I/II, CA-CFAR; -> Pfa at SNR = 0)
// which we use both as the analytic prediction and to draw a Bernoulli detection outcome from the
// run Rng (one uniform). For non-fluctuating (Swerling 0) signals the Pd uses the corresponding
// average over the reference-cell statistics.
struct CfarResult {
  bool detected = false;  // did the cell under test exceed the CFAR threshold this look?
  double pd = 0.0;        // closed-form detection probability for this SNR (diagnostic)
  double pfa = 0.0;       // design false-alarm probability (diagnostic)
  double snr_db = 0.0;    // signal-to-noise ratio of the cell under test [dB]
};

// CFAR threshold multiplier alpha for the configured Pfa and reference-cell count.
double cfarAlpha(const CfarConfig& cfg);

// Closed-form CA-CFAR single-look detection probability for a Swerling-I/II (exponential) target at
// the given linear SNR.
double cfarPdSwerling(const CfarConfig& cfg, double snr_linear);

// Run the CFAR detector for a cell-under-test at the given linear SNR: compute Pd, then draw a
// Bernoulli detection from the Rng. The returned struct also reports the diagnostic Pd/Pfa/SNR.
CfarResult cfarDetect(const CfarConfig& cfg, double snr_linear, Rng& rng);

}  // namespace gncsim
