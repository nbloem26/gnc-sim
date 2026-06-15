/**
 * UQ math for the UQ / Credibility Studio — a TypeScript port of the deterministic,
 * seeded uncertainty-quantification tools in `postproc/gncpost/uq.py` (issue #33),
 * trimmed for responsive in-browser use:
 *
 *   1. **Bootstrap confidence interval** (`bootstrapCi`) — a seeded nonparametric
 *      (percentile) bootstrap CI for any scalar statistic (CEP = median miss,
 *      P(hit) = intercept fraction). The half-width quantifies the sampling noise
 *      on a headline number.
 *   2. **Convergence diagnostic** (`convergence`) — the running statistic vs the
 *      number of cases N with a bootstrap CI band, so the "enough cases?" question
 *      is visible: the band tightens ~1/sqrt(N) and the estimate stops wandering.
 *   3. **Global sensitivity** (`sobolIndices`) — a hand-rolled Saltelli sampler +
 *      first/total-order Sobol estimator (Saltelli 2010 / Jansen 1999), ranking the
 *      dispersion inputs by how much each drives the output variance — i.e. *what
 *      drives the miss*.
 *
 * Everything is seeded through `@/lib/prng` (mulberry32 + Box-Muller, the same
 * generator the web Monte Carlo batch uses), so the same seed reproduces the same
 * CIs / Sobol indices bit-for-bit. No `Math.random`.
 *
 * Units follow the repo convention (units in names): miss distances are `_m`.
 */

import { Prng } from '@/lib/prng';

// ----------------------------------------------------------------------------
// Statistics + percentile helpers (local copies so this module is self-contained)
// ----------------------------------------------------------------------------

/** Empirical percentile p∈[0,100] via linear interpolation (matches lib/stats). */
export function percentile(xs: number[], p: number): number {
  if (xs.length === 0) return 0;
  const sorted = [...xs].sort((a, b) => a - b);
  const idx = (p / 100) * (sorted.length - 1);
  const lo = Math.floor(idx);
  const hi = Math.ceil(idx);
  if (lo === hi) return sorted[lo];
  const frac = idx - lo;
  return sorted[lo] * (1 - frac) + sorted[hi] * frac;
}

/** CEP = median miss distance (the 50% radius). */
export function cepStat(miss_m: number[]): number {
  return percentile(miss_m, 50);
}

/** P(hit) = mean of the 0/1 intercept flags. */
export function hitFractionStat(flags: number[]): number {
  if (flags.length === 0) return 0;
  return flags.reduce((a, b) => a + b, 0) / flags.length;
}

/** A scalar statistic over a sample. */
export type Statistic = (xs: number[]) => number;

// ----------------------------------------------------------------------------
// Bootstrap confidence interval
// ----------------------------------------------------------------------------

export interface ConfidenceInterval {
  /** Point estimate on the full sample. */
  estimate: number;
  /** Lower confidence bound. */
  low: number;
  /** Upper confidence bound. */
  high: number;
  /** Nominal coverage, e.g. 0.95. */
  level: number;
  /** Half the CI width — the ± reported alongside the estimate. */
  halfWidth: number;
}

/**
 * Seeded nonparametric (percentile) bootstrap CI for `statistic`.
 *
 * Resamples `data` with replacement `nResamples` times, recomputes the statistic
 * on each resample, and takes the empirical [alpha/2, 1-alpha/2] percentiles as the
 * interval. Distribution-free, so it works for the median (CEP) and the mean of a
 * 0/1 vector (P(hit)) alike. Determinism: resampling indices come from the supplied
 * `Prng`, so a fixed seed reproduces the interval exactly.
 */
export function bootstrapCi(
  data: number[],
  statistic: Statistic = cepStat,
  opts: { level?: number; nResamples?: number; rng?: Prng; seed?: number } = {},
): ConfidenceInterval {
  const level = opts.level ?? 0.95;
  const nResamples = opts.nResamples ?? 1000;
  const rng = opts.rng ?? new Prng(opts.seed ?? 0);

  const arr = data.filter((x) => Number.isFinite(x));
  const n = arr.length;
  const estimate = n > 0 ? statistic(arr) : NaN;
  if (n === 0) {
    return { estimate: NaN, low: NaN, high: NaN, level, halfWidth: NaN };
  }

  const stats = new Array<number>(nResamples);
  const resample = new Array<number>(n);
  for (let r = 0; r < nResamples; r++) {
    for (let i = 0; i < n; i++) {
      resample[i] = arr[Math.floor(rng.uniform(0, n))];
    }
    stats[r] = statistic(resample);
  }

  const alpha = 1 - level;
  const low = percentile(stats, 100 * (alpha / 2));
  const high = percentile(stats, 100 * (1 - alpha / 2));
  return { estimate, low, high, level, halfWidth: 0.5 * (high - low) };
}

// ----------------------------------------------------------------------------
// Convergence diagnostic
// ----------------------------------------------------------------------------

export interface ConvergencePoint {
  n: number;
  estimate: number;
  low: number;
  high: number;
}

/** Unique, ascending, log-spaced integers in [min, max] inclusive. */
function geomIntSteps(min: number, max: number, count: number): number[] {
  if (max <= min) return [Math.max(1, max)];
  const out: number[] = [];
  const lnLo = Math.log(min);
  const lnHi = Math.log(max);
  for (let i = 0; i < count; i++) {
    const f = count > 1 ? i / (count - 1) : 1;
    const v = Math.round(Math.exp(lnLo + f * (lnHi - lnLo)));
    if (out.length === 0 || v > out[out.length - 1]) out.push(v);
  }
  if (out[out.length - 1] !== max) out.push(max);
  return out;
}

/**
 * Running `statistic` vs sample size N with a bootstrap CI band.
 *
 * Evaluates the statistic (and a bootstrap CI) on the first N cases for up to
 * `nPoints` log-spaced values of N between `minN` and the full sample size. The CI
 * band should tighten ~1/sqrt(N) and the estimate should stop wandering once enough
 * cases have run — the visual sufficiency check. Each sub-sample's CI is drawn from a
 * distinct child seed so the whole curve is reproducible from one master seed.
 */
export function convergence(
  data: number[],
  statistic: Statistic = cepStat,
  opts: {
    level?: number;
    nPoints?: number;
    minN?: number;
    nResamples?: number;
    seed?: number;
  } = {},
): ConvergencePoint[] {
  const level = opts.level ?? 0.95;
  const nPoints = opts.nPoints ?? 16;
  const minN = opts.minN ?? 5;
  const nResamples = opts.nResamples ?? 600;
  const master = new Prng(opts.seed ?? 0);

  const arr = data.filter((x) => Number.isFinite(x));
  const total = arr.length;
  if (total < minN) return [];

  const ns = geomIntSteps(minN, total, nPoints);
  return ns.map((n) => {
    const child = new Prng(master.nextUint32());
    const ci = bootstrapCi(arr.slice(0, n), statistic, { level, nResamples, rng: child });
    return { n, estimate: ci.estimate, low: ci.low, high: ci.high };
  });
}

// ----------------------------------------------------------------------------
// Global sensitivity: Saltelli sampling + first/total-order Sobol indices
// ----------------------------------------------------------------------------

export interface SobolResult {
  names: string[];
  /** S_i — fraction of output variance from input i alone. */
  firstOrder: number[];
  /** S_Ti — fraction including all interactions involving i. */
  totalOrder: number[];
}

/** One Saltelli design: matrices A, B (n×d) and the d cross matrices AB_i. */
interface SaltelliSample {
  a: number[][];
  b: number[][];
  ab: number[][][];
}

/**
 * Saltelli A, B and cross matrices AB_i for `bounds` (one [lo,hi] per input).
 *
 * A single (n×2d) seeded uniform block is split into two (n×d) matrices A and B
 * (drawing both halves from one stream keeps them jointly well-distributed); AB_i is
 * A with column i replaced by B's column i. Evaluating the model on A, B and every
 * AB_i gives the pieces the first/total-order estimators need. Total evaluations:
 * nBase·(d+2). (The Python port prefers a scrambled Sobol low-discrepancy sequence;
 * here we use plain seeded uniforms to keep the TS port small and dependency-free —
 * we compensate with the same estimators and keep nBase small for responsiveness.)
 */
function saltelliSample(
  nBase: number,
  bounds: [number, number][],
  rng: Prng,
): SaltelliSample {
  const d = bounds.length;
  const a: number[][] = [];
  const b: number[][] = [];
  for (let r = 0; r < nBase; r++) {
    const rowA = new Array<number>(d);
    const rowB = new Array<number>(d);
    // Draw all of A's row first, then B's, mirroring [:, :d] / [:, d:] of one block.
    for (let i = 0; i < d; i++) {
      const [lo, hi] = bounds[i];
      rowA[i] = lo + rng.uniform(0, 1) * (hi - lo);
    }
    for (let i = 0; i < d; i++) {
      const [lo, hi] = bounds[i];
      rowB[i] = lo + rng.uniform(0, 1) * (hi - lo);
    }
    a.push(rowA);
    b.push(rowB);
  }

  const ab: number[][][] = [];
  for (let i = 0; i < d; i++) {
    const abi: number[][] = [];
    for (let r = 0; r < nBase; r++) {
      const row = a[r].slice();
      row[i] = b[r][i];
      abi.push(row);
    }
    ab.push(abi);
  }
  return { a, b, ab };
}

/** Sample variance (ddof=1). */
function variance(xs: number[]): number {
  const n = xs.length;
  if (n < 2) return 0;
  const m = xs.reduce((acc, v) => acc + v, 0) / n;
  let s = 0;
  for (const v of xs) s += (v - m) * (v - m);
  return s / (n - 1);
}

const clamp01 = (x: number): number => (x < 0 ? 0 : x > 1 ? 1 : x);

/**
 * Estimate first- and total-order Sobol indices via the Saltelli design.
 *
 * `model` maps an (m×d) array of input rows to a length-m output vector. Estimators
 * (Saltelli 2010 / Jansen 1999), both standard:
 *
 *     S_i  = mean(yB · (yAB_i − yA)) / Var          (first order)
 *     S_Ti = mean((yA − yAB_i)^2) / (2·Var)         (total order)
 *
 * Indices are clipped to [0, 1]. A degenerate (constant) output yields all-zero
 * indices. Determinism follows from the seeded sampler.
 */
export function sobolIndices(
  model: (rows: number[][]) => number[],
  bounds: [number, number][],
  names: string[],
  opts: { nBase?: number; seed?: number; rng?: Prng } = {},
): SobolResult {
  if (bounds.length !== names.length) {
    throw new Error('sobolIndices: bounds and names must have the same length');
  }
  const nBase = opts.nBase ?? 64;
  const rng = opts.rng ?? new Prng(opts.seed ?? 0);
  const d = bounds.length;

  const { a, b, ab } = saltelliSample(nBase, bounds, rng);
  const ya = model(a);
  const yb = model(b);
  const yab = ab.map((abi) => model(abi));

  const varTotal = variance([...ya, ...yb]);
  const first = new Array<number>(d).fill(0);
  const total = new Array<number>(d).fill(0);
  if (varTotal <= 0) {
    return { names: [...names], firstOrder: first, totalOrder: total };
  }

  for (let i = 0; i < d; i++) {
    let sFirst = 0;
    let sTotal = 0;
    for (let r = 0; r < nBase; r++) {
      sFirst += yb[r] * (yab[i][r] - ya[r]);
      const diff = ya[r] - yab[i][r];
      sTotal += diff * diff;
    }
    first[i] = clamp01(sFirst / nBase / varTotal);
    total[i] = clamp01(sTotal / nBase / (2 * varTotal));
  }
  return { names: [...names], firstOrder: first, totalOrder: total };
}

/**
 * Async Sobol estimate when the model is an `async` per-row evaluator (e.g. each
 * row is one WASM `runSim` case). Builds the Saltelli design, evaluates every row
 * through `modelAsync`, then runs the same first/total-order estimators as
 * {@link sobolIndices}. Keep `nBase` small for responsiveness — total engine runs
 * are nBase·(d+2).
 */
export async function sobolIndicesAsync(
  modelAsync: (rows: number[][]) => Promise<number[]>,
  bounds: [number, number][],
  names: string[],
  opts: { nBase?: number; seed?: number; rng?: Prng } = {},
): Promise<SobolResult> {
  if (bounds.length !== names.length) {
    throw new Error('sobolIndicesAsync: bounds and names must have the same length');
  }
  const nBase = opts.nBase ?? 8;
  const rng = opts.rng ?? new Prng(opts.seed ?? 0);
  const d = bounds.length;

  const { a, b, ab } = saltelliSample(nBase, bounds, rng);
  const ya = await modelAsync(a);
  const yb = await modelAsync(b);
  const yab: number[][] = [];
  for (const abi of ab) yab.push(await modelAsync(abi));

  const varTotal = variance([...ya, ...yb]);
  const first = new Array<number>(d).fill(0);
  const total = new Array<number>(d).fill(0);
  if (varTotal <= 0) {
    return { names: [...names], firstOrder: first, totalOrder: total };
  }
  for (let i = 0; i < d; i++) {
    let sFirst = 0;
    let sTotal = 0;
    for (let r = 0; r < nBase; r++) {
      sFirst += yb[r] * (yab[i][r] - ya[r]);
      const diff = ya[r] - yab[i][r];
      sTotal += diff * diff;
    }
    first[i] = clamp01(sFirst / nBase / varTotal);
    total[i] = clamp01(sTotal / nBase / (2 * varTotal));
  }
  return { names: [...names], firstOrder: first, totalOrder: total };
}

/** Inputs sorted by total-order index, descending (what drives the output). */
export function sobolRanking(result: SobolResult): { name: string; total: number }[] {
  return result.names
    .map((name, i) => ({ name, total: result.totalOrder[i] }))
    .sort((p, q) => q.total - p.total);
}
