/**
 * radarMath — closed-form radar / sensor phenomenology math for the Radar Studio.
 *
 * These are pure, dependency-free TypeScript ports of the already-validated C++
 * model in `core/src/sensors/Phenomenology.{hpp,cpp}` (issue #39). The Studio is
 * a client-side exploration tool, so it does not call the WASM core — instead it
 * re-implements the same closed-form expressions here. Each function is
 * unit-commented against its C++ counterpart so the two stay in lockstep.
 *
 * Units follow the repo convention (units-in-names): range_m, rcs_m2, snr_db,
 * pfa (dimensionless probability), etc. "Linear" SNR/INR are power ratios, not dB.
 */

/** Swerling fluctuation case. Mirrors `enum class SwerlingCase` in Phenomenology.hpp. */
export type SwerlingCase = 0 | 1 | 2 | 3 | 4;

/** dB (power ratio) -> linear. Mirrors `dbToLinear` in Phenomenology.cpp. */
export function dbToLinear(db: number): number {
  return Math.pow(10.0, 0.1 * db);
}

/** linear (power ratio) -> dB. */
export function linearToDb(linear: number): number {
  return 10.0 * Math.log10(Math.max(linear, 1e-300));
}

/**
 * Anchored monostatic radar-range-equation SNR (LINEAR), mirroring
 * `radarSnrLinear` in Phenomenology.cpp.
 *
 *   SNR = (Pt G^2 lambda^2 sigma) / ((4 pi)^3 R^4 k Ts B L)
 *
 * is folded into a calibrated constant: at `range_ref_m` a target of
 * `rcs_ref_m2` yields `snr_ref_db`. From that anchor SNR scales as sigma / R^4.
 * Clutter and barrage-noise jamming raise the effective noise floor, dividing
 * SNR by (1 + CNR + JNR).
 */
export function radarSnrLinear(args: {
  snr_ref_db: number;
  range_ref_m: number;
  rcs_ref_m2: number;
  range_m: number;
  rcs_m2: number;
  clutter_cnr_db?: number; // <= -90 disables (matches C++ sentinel)
  jammer_jnr_db?: number;
}): number {
  const r = Math.max(args.range_m, 1.0);
  const r_ref = Math.max(args.range_ref_m, 1.0);
  const rcs_ref = Math.max(args.rcs_ref_m2, 1e-12);
  const snr_ref = dbToLinear(args.snr_ref_db);
  // Range equation scaling about the anchor: SNR ∝ sigma / R^4.
  const r_ratio = r_ref / r;
  const r4 = r_ratio * r_ratio * r_ratio * r_ratio;
  let snr = snr_ref * (args.rcs_m2 / rcs_ref) * r4;
  // Clutter + barrage-noise jamming raise the noise floor: divide by (1 + CNR + JNR).
  let interference = 1.0;
  const cnr = args.clutter_cnr_db ?? -120;
  const jnr = args.jammer_jnr_db ?? -120;
  if (cnr > -90.0) interference += dbToLinear(cnr);
  if (jnr > -90.0) interference += dbToLinear(jnr);
  return snr / interference;
}

/**
 * CA-CFAR threshold multiplier alpha. Mirrors `cfarAlpha` in Phenomenology.cpp.
 * Gandhi & Kassam:  alpha = N * (Pfa^(-1/N) - 1).
 */
export function cfarAlpha(num_ref_cells: number, pfa: number): number {
  const n = Math.max(Math.round(num_ref_cells), 1);
  let p = pfa;
  if (p <= 0.0) p = 1e-12;
  if (p >= 1.0) p = 1.0 - 1e-12;
  return n * (Math.pow(p, -1.0 / n) - 1.0);
}

/**
 * Closed-form CA-CFAR single-look detection probability for a Swerling-I/II
 * (exponential) target at linear SNR. Mirrors `cfarPdSwerling`:
 *   Pd = (1 + alpha / (N (1 + SNR)))^(-N)        (-> Pfa at SNR = 0)
 */
export function cfarPdSwerlingI(
  num_ref_cells: number,
  pfa: number,
  snr_linear: number,
): number {
  const n = Math.max(Math.round(num_ref_cells), 1);
  const alpha = cfarAlpha(n, pfa);
  const snr = Math.max(snr_linear, 0.0);
  const base = 1.0 + alpha / (n * (1.0 + snr));
  return Math.pow(base, -n);
}

/**
 * CA-CFAR single-look Pd generalised over Swerling case at linear SNR.
 *
 * The C++ core ships only the Swerling-I/II closed form (`cfarPdSwerling`); the
 * Studio extends the *same* CA-CFAR threshold (alpha from Gandhi & Kassam) to
 * the other classical cases so the ROC/operating-point plots can compare them:
 *
 *  - Swerling 0/V (non-fluctuating): the exact CA-CFAR Pd for a deterministic
 *    target has no simple product form. To stay faithful to the validated core
 *    and avoid shipping an unvalidated closed form, case 0 falls back to the
 *    Swerling-I/II expression — a conservative (pessimistic) lower bound on the
 *    true non-fluctuating Pd (documented caveat).
 *  - Swerling I/II: exact, the form validated in the C++ core (`cfarPdSwerlingI`).
 *  - Swerling III/IV (chi-square, 4 dof — one dominant + many small scatterers):
 *    the matched CA-CFAR closed form, a two-term correction on the I/II base.
 */
export function cfarPdSwerling(
  swerling: SwerlingCase,
  num_ref_cells: number,
  pfa: number,
  snr_linear: number,
): number {
  const n = Math.max(Math.round(num_ref_cells), 1);
  const alpha = cfarAlpha(n, pfa);
  const snr = Math.max(snr_linear, 0.0);

  if (swerling === 3 || swerling === 4) {
    // chi-square 4-dof (one dominant + many small scatterers). CA-CFAR closed
    // form: base^(-N) * [1 + (N/(N+1)) * (alpha/N) * SNR / (1+SNR/2)^2 ... ]
    // We use the standard two-term Swerling-III/IV CFAR result:
    //   y = alpha / (N (1 + SNR/2))
    //   Pd = (1 + y)^(-N) * (1 + N*y*(SNR/2)/(1+SNR/2)/(1+y))
    const half = 1.0 + snr / 2.0;
    const y = alpha / (n * half);
    const baseN = Math.pow(1.0 + y, -n);
    const corr = 1.0 + (n * y * (snr / 2.0)) / (half * (1.0 + y));
    return Math.min(1.0, Math.max(0.0, baseN * corr));
  }

  // Swerling 0/V and I/II share the validated exponential CA-CFAR form here.
  // (Case 0 is treated as a conservative I/II bound — see doc comment / caveat.)
  const base = 1.0 + alpha / (n * (1.0 + snr));
  return Math.pow(base, -n);
}

/**
 * Mean-RCS scale for a given Swerling case used only for *display* of the
 * fluctuation: returns the multiplicative factor on mean RCS at the median of
 * the amplitude distribution (informational; the Pd math above already folds in
 * fluctuation analytically). Kept simple and unit-clean.
 */
export function swerlingLabel(sw: SwerlingCase): string {
  switch (sw) {
    case 0:
      return 'Swerling 0/V (non-fluctuating)';
    case 1:
      return 'Swerling I (scan-to-scan, χ² 2-dof)';
    case 2:
      return 'Swerling II (pulse-to-pulse, χ² 2-dof)';
    case 3:
      return 'Swerling III (scan-to-scan, χ² 4-dof)';
    case 4:
      return 'Swerling IV (pulse-to-pulse, χ² 4-dof)';
  }
}

/** Linearly spaced grid, inclusive of both endpoints. */
export function linspace(start: number, end: number, count: number): number[] {
  if (count < 2) return [start];
  const out = new Array<number>(count);
  const span = (end - start) / (count - 1);
  for (let i = 0; i < count; i++) out[i] = start + i * span;
  return out;
}

/** Logarithmically spaced grid (base-10), inclusive of both endpoints. */
export function logspace(
  startExp: number,
  endExp: number,
  count: number,
): number[] {
  return linspace(startExp, endExp, count).map((e) => Math.pow(10, e));
}

/**
 * Solve for the range [m] at which Pd >= pdTarget for a fixed RCS, given the
 * anchored range equation. Pd is monotonically decreasing in range (SNR ∝ 1/R^4),
 * so a bisection on range is exact. Returns the crossing range, or 0 if the
 * target Pd is unreachable even at the minimum range.
 */
export function detectionRangeForPd(args: {
  swerling: SwerlingCase;
  snr_ref_db: number;
  range_ref_m: number;
  rcs_ref_m2: number;
  rcs_m2: number;
  num_ref_cells: number;
  pfa: number;
  pdTarget: number;
  range_min_m: number;
  range_max_m: number;
}): number {
  const pdAt = (range_m: number): number => {
    const snr = radarSnrLinear({
      snr_ref_db: args.snr_ref_db,
      range_ref_m: args.range_ref_m,
      rcs_ref_m2: args.rcs_ref_m2,
      range_m,
      rcs_m2: args.rcs_m2,
    });
    return cfarPdSwerling(args.swerling, args.num_ref_cells, args.pfa, snr);
  };

  let lo = Math.max(args.range_min_m, 1.0);
  let hi = Math.max(args.range_max_m, lo + 1.0);

  // If even at the far edge Pd is already above target, the envelope extends
  // beyond our search window — report the window edge.
  if (pdAt(hi) >= args.pdTarget) return hi;
  // If even at the near edge Pd is below target, it is undetectable.
  if (pdAt(lo) < args.pdTarget) return 0.0;

  // Bisection: pdAt is monotonically decreasing in range.
  for (let i = 0; i < 60; i++) {
    const mid = 0.5 * (lo + hi);
    if (pdAt(mid) >= args.pdTarget) lo = mid;
    else hi = mid;
  }
  return 0.5 * (lo + hi);
}
