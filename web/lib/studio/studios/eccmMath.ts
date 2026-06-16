/**
 * eccmMath — closed-form electronic countermeasures / counter-countermeasures
 * phenomenology for the Countermeasures / ECCM Studio (issue #119).
 *
 * Adversarial sensing: how countermeasures degrade the victim radar's CFAR
 * detection chain and the IR seeker's discrimination, and where the radar
 * "burns through". This module is **pure client-side** and deliberately
 * **reuses** the validated radar phenomenology in `radarMath.ts` (range-equation
 * SNR ∝ σ/R⁴, CA-CFAR Gandhi–Kassam threshold α, Swerling Pd) rather than
 * duplicating those closed forms. It adds only the ECM/CM-specific models:
 *
 *  - **Noise / barrage jamming.** A stand-off jammer radiates broadband noise
 *    into the victim's receiver, raising the noise floor. Unlike a target echo
 *    (two-way, ∝ 1/R⁴), the jammer signal is one-way (∝ 1/R_j²), so the
 *    jammer-to-noise ratio J/N grows as the radar looks further out. The
 *    signal-to-jammer ratio J/S therefore *falls* as range closes — the radar
 *    **burns through** inside the range where the (1/R⁴) skin echo overtakes the
 *    (1/R²) jamming. We compute J/S(R) and solve the burn-through range.
 *  - **DRFM / repeater.** A digital-RF-memory repeater samples the victim's own
 *    waveform and replays delayed copies, injecting coherent **false targets**
 *    that pass the matched filter. Count scales with repeater power margin over
 *    the CFAR threshold (number of plausible false-target cells it can light up).
 *  - **Chaff.** A dispensed cloud of resonant dipoles blooms into a large, slowly
 *    decorrelating RCS (Swerling-I-like). It both *masks* (raises the local noise
 *    /clutter floor, an extra CNR term) and triggers **CFAR false alarms** in the
 *    cells it occupies. Expected false-alarm count = occupied_cells · Pfa(chaff).
 *  - **IR flares / decoys.** Against the feature-based seeker discriminator
 *    (`core/src/gnc/Discriminator`), flares and decoys are extra objects whose
 *    measured signatures sit near the true warhead's. Discrimination **margin**
 *    is the weighted (Mahalanobis) separation of the true target from the
 *    best-scoring decoy; adding decoys/flares (denser, brighter) erodes it. We
 *    model the margin vs decoy density and flare intensity in the same
 *    inverse-variance-weighted form the C++ discriminator uses.
 *
 * Units follow the repo convention (units-in-names): range_m, rcs_m2, *_db,
 * *_dimensionless probabilities. "Linear" ratios are power ratios, not dB.
 */

import {
  dbToLinear,
  linearToDb,
  radarSnrLinear,
  cfarAlpha,
  cfarPdSwerling,
  type SwerlingCase,
} from './radarMath';

export type JammerType = 'off' | 'noise' | 'drfm';

// ---------------------------------------------------------------------------
// Noise / barrage jamming — J/N, J/S, burn-through
// ---------------------------------------------------------------------------

/**
 * Jammer-to-noise ratio J/N [linear] at the victim receiver for a stand-off
 * noise jammer. The jammer link is one-way, so received jammer power ∝ 1/R_j².
 * We anchor it to a user-facing J/S figure-of-merit, `js_ref_db`, specified at
 * the reference range against the reference-RCS target: at R = range_ref_m the
 * jammer-to-signal ratio equals js_ref_db. From the anchor, J/N(R) follows the
 * one-way law J/N ∝ 1/R² (relative to the same anchor), independent of the
 * target echo.
 *
 *   J/S(R_ref) = js_ref_db   (definition of the anchor)
 *   S(R_ref)   = snr_ref (linear, the clean skin-echo SNR at the anchor)
 *   => J/N(R_ref) = J/S(R_ref) * S(R_ref)
 *   => J/N(R)     = J/N(R_ref) * (R_ref / R)^2     (one-way jammer link)
 */
export function jammerJnrLinear(args: {
  js_ref_db: number;
  snr_ref_db: number;
  range_ref_m: number;
  range_m: number;
}): number {
  const r = Math.max(args.range_m, 1.0);
  const r_ref = Math.max(args.range_ref_m, 1.0);
  const js_ref = dbToLinear(args.js_ref_db);
  const snr_ref = dbToLinear(args.snr_ref_db);
  const jnr_ref = js_ref * snr_ref; // J/N at the anchor
  const r_ratio = r_ref / r;
  return jnr_ref * r_ratio * r_ratio; // one-way: 1/R²
}

/**
 * Signal-to-jammer ratio S/J [linear] vs range. The skin echo is two-way
 * (∝ σ/R⁴, from the radar range equation) and the jammer is one-way (∝ 1/R²),
 * so S/J ∝ σ R⁻⁴ / R⁻² = σ / R². As range *closes*, S/J climbs — burn-through.
 * Returned as S/J so that S/J ≥ 1 (0 dB) is the burn-through condition.
 */
export function signalToJammerLinear(args: {
  snr_ref_db: number;
  js_ref_db: number;
  range_ref_m: number;
  rcs_ref_m2: number;
  range_m: number;
  rcs_m2: number;
}): number {
  // Clean two-way echo SNR (no interference folded in — that's the "S").
  const s = radarSnrLinear({
    snr_ref_db: args.snr_ref_db,
    range_ref_m: args.range_ref_m,
    rcs_ref_m2: args.rcs_ref_m2,
    range_m: args.range_m,
    rcs_m2: args.rcs_m2,
  });
  const j = jammerJnrLinear({
    js_ref_db: args.js_ref_db,
    snr_ref_db: args.snr_ref_db,
    range_ref_m: args.range_ref_m,
    range_m: args.range_m,
  });
  return s / Math.max(j, 1e-300);
}

/**
 * Burn-through range [m]: the range at which the skin echo overtakes the
 * jamming, S/J = 1 (0 dB). Since S/J ∝ σ / R², it is closed-form, but we solve
 * it by bisection on the same anchored expressions to stay consistent with the
 * SNR model (and robust to the clamps). S/J is monotonically *decreasing* in
 * range, so inside the burn-through range the target beats the jammer.
 * Returns 0 if the jammer never wins (always burned through) within the window,
 * or range_max_m if it is never burned through.
 */
export function burnThroughRangeM(args: {
  snr_ref_db: number;
  js_ref_db: number;
  range_ref_m: number;
  rcs_ref_m2: number;
  rcs_m2: number;
  range_min_m: number;
  range_max_m: number;
}): number {
  const sjAt = (range_m: number): number =>
    signalToJammerLinear({
      snr_ref_db: args.snr_ref_db,
      js_ref_db: args.js_ref_db,
      range_ref_m: args.range_ref_m,
      rcs_ref_m2: args.rcs_ref_m2,
      range_m,
      rcs_m2: args.rcs_m2,
    });

  const lo0 = Math.max(args.range_min_m, 1.0);
  const hi0 = Math.max(args.range_max_m, lo0 + 1.0);

  // At the near edge S/J is highest. If even there the jammer wins, no burn-through.
  if (sjAt(lo0) < 1.0) return 0.0;
  // At the far edge S/J is lowest. If even there the target wins, always burned through.
  if (sjAt(hi0) >= 1.0) return hi0;

  let lo = lo0;
  let hi = hi0;
  for (let i = 0; i < 60; i++) {
    const mid = 0.5 * (lo + hi);
    if (sjAt(mid) >= 1.0) lo = mid; // still burned through nearer -> push out
    else hi = mid;
  }
  return 0.5 * (lo + hi);
}

// ---------------------------------------------------------------------------
// DRFM / repeater — coherent false targets
// ---------------------------------------------------------------------------

/**
 * Expected number of DRFM coherent false targets. A repeater replays the
 * victim's matched waveform, so each replayed copy lands at processing gain
 * (full SNR) — the count is set by how many distinct delay/Doppler cells the
 * repeater has the power budget to fill above the CFAR detection threshold.
 *
 * We model it as: power margin in dB of the repeater output over the CFAR
 * detection-threshold SNR, converted to a cell count by a per-dB fill rate. The
 * stronger the repeater (higher J/S), the more false-target cells it can light.
 *
 *   margin_db = repeater_js_db - thresh_snr_db
 *   count     = round( max(0, margin_db) * cells_per_db ), capped at max_cells
 */
export function drfmFalseTargetCount(args: {
  repeater_js_db: number;
  thresh_snr_db: number;
  cells_per_db: number;
  max_cells: number;
}): number {
  const margin_db = args.repeater_js_db - args.thresh_snr_db;
  if (margin_db <= 0) return 0;
  const raw = margin_db * Math.max(args.cells_per_db, 0);
  return Math.min(Math.round(raw), Math.max(0, Math.round(args.max_cells)));
}

// ---------------------------------------------------------------------------
// Chaff — masking + CFAR false alarms
// ---------------------------------------------------------------------------

/**
 * Effective clutter-to-noise ratio [dB] contributed by a blooming chaff cloud.
 * The cloud presents a large resonant-dipole RCS that, distributed over the
 * cells it occupies, raises the local noise/clutter floor. We anchor its CNR to
 * the radar range-equation SNR of an equivalent point target of the chaff RCS at
 * the current range (so it scales 1/R⁴ like any echo), reduced by a `bloom`
 * factor that spreads the energy over more cells as the cloud expands (diluting
 * the per-cell return but occupying more cells — see occupiedCells).
 */
export function chaffCnrDb(args: {
  snr_ref_db: number;
  range_ref_m: number;
  rcs_ref_m2: number;
  range_m: number;
  chaff_rcs_m2: number;
  bloom: number;
}): number {
  const bloom = Math.max(args.bloom, 1.0);
  // Per-cell effective RCS = total cloud RCS spread over ~bloom cells.
  const per_cell_rcs_m2 = Math.max(args.chaff_rcs_m2 / bloom, 1e-9);
  const cnr_linear = radarSnrLinear({
    snr_ref_db: args.snr_ref_db,
    range_ref_m: args.range_ref_m,
    rcs_ref_m2: args.rcs_ref_m2,
    range_m: args.range_m,
    rcs_m2: per_cell_rcs_m2,
  });
  return linearToDb(cnr_linear);
}

/**
 * Number of range/Doppler cells the chaff cloud occupies — grows with bloom.
 * A simple linear-in-bloom occupancy (one cell per unit bloom, floored at 1).
 */
export function chaffOccupiedCells(bloom: number): number {
  return Math.max(1, Math.round(bloom));
}

/**
 * Expected CFAR false-alarm count produced by chaff. Within the cloud the local
 * mean rises but the CA-CFAR reference window is also contaminated, so the
 * *realised* Pfa departs from the design Pfa. We model the realised per-cell
 * false-alarm probability as the CA-CFAR Pd of a Swerling-I "target" whose SNR
 * equals the chaff per-cell CNR (the cloud return *is* the thing crossing the
 * threshold), then multiply by the number of occupied cells.
 *
 *   E[false alarms] = occupied_cells · Pfa_realised
 *   Pfa_realised    = CFAR-Pd( SNR = chaff per-cell CNR )
 */
export function chaffFalseAlarmCount(args: {
  num_ref_cells: number;
  pfa: number;
  chaff_cnr_db: number;
  occupied_cells: number;
}): number {
  const cnr_linear = dbToLinear(args.chaff_cnr_db);
  const pfaRealised = cfarPdSwerling(1, args.num_ref_cells, args.pfa, cnr_linear);
  return args.occupied_cells * pfaRealised;
}

// ---------------------------------------------------------------------------
// IR flares / decoys — discrimination margin
// ---------------------------------------------------------------------------

/**
 * Feature-based IR discrimination margin (dimensionless score units), modelled
 * on the C++ `Discriminator` (issue #6): each object carries a 3-feature
 * signature [intensity, size, decel] and is scored by the inverse-variance-
 * weighted squared distance from the true-target signature, negated so higher =
 * more target-like. The **margin** is (true-target score) − (best decoy score):
 * positive margin = the seeker prefers the warhead; ≤ 0 = it is spoofed.
 *
 * The true target sits at the target signature (instant score ≈ 0, minus
 * measurement-noise variance). A decoy/flare sits `delta` away in feature space.
 * Brighter flares (higher relative intensity) sit *closer* in the intensity
 * feature, shrinking delta and the margin; denser decoy/CSO fields raise the
 * chance that the *best-scoring* decoy happens to fall near the target (an
 * order-statistic effect: the closest of N decoys is nearer than a single one),
 * also shrinking the margin.
 *
 *   var          = feature_spread² + measurement_noise²   (per-feature)
 *   w            = 1 / var
 *   delta_eff    = base_separation · (1 − flare_intensity_frac) /
 *                  (1 + k_density·(decoy_density − 1))      (order-stat shrink)
 *   margin       = w · delta_eff²   −   w · measurement_noise²   (noise floor)
 *
 * Margin is returned in the same arbitrary score units the C++ module uses; only
 * its sign and relative magnitude are meaningful (positive ⇒ discriminable).
 */
export function irDiscriminationMargin(args: {
  base_separation: number; // nominal feature-space separation of a lone decoy
  feature_spread: number; // static per-object signature spread (1σ)
  measurement_noise: number; // per-step seeker measurement noise (1σ)
  flare_intensity_frac: number; // 0..1: how close the flare mimics target intensity
  decoy_density: number; // number of decoys/CSOs competing (>=1)
  density_shrink_k: number; // order-statistic shrink rate with density
}): number {
  const var_feat =
    args.feature_spread * args.feature_spread +
    args.measurement_noise * args.measurement_noise;
  const w = 1.0 / Math.max(var_feat, 1e-9);

  const density = Math.max(args.decoy_density, 1.0);
  const intensityShrink = Math.max(0.0, 1.0 - clamp01(args.flare_intensity_frac));
  const densityDenom = 1.0 + Math.max(args.density_shrink_k, 0.0) * (density - 1.0);
  const delta_eff = (args.base_separation * intensityShrink) / densityDenom;

  // Weighted separation minus the irreducible measurement-noise floor.
  const noiseFloor = w * args.measurement_noise * args.measurement_noise;
  return w * delta_eff * delta_eff - noiseFloor;
}

function clamp01(x: number): number {
  return x < 0 ? 0 : x > 1 ? 1 : x;
}

// ---------------------------------------------------------------------------
// Small re-exports so the studio imports from one place
// ---------------------------------------------------------------------------

export {
  dbToLinear,
  linearToDb,
  radarSnrLinear,
  cfarAlpha,
  cfarPdSwerling,
  type SwerlingCase,
};
