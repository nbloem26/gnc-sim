/**
 * Optimizer math for the Optimizer Studio (issue #121) — a small, dependency-free
 * 1-D continuous optimizer used to auto-tune a single decision variable against a
 * (noisy, engine-backed) scalar objective.
 *
 * The objective is expensive (each evaluation is a seeded Monte Carlo through the
 * WASM core), so the strategy is deliberately frugal and bounded:
 *
 *   1. **Coarse grid** over the bound builds the *objective surface* we plot, and
 *      seeds a global-ish best (robust to the mild multimodality / sampling noise a
 *      small MC introduces — golden-section alone can fall into a local dip).
 *   2. **Local pattern search** (compass / Hooke-Jeeves style, contracting step)
 *      refines around the grid best to squeeze out the local optimum.
 *
 * Every evaluated point is recorded so the studio can draw the surface and the
 * convergence trace (best-so-far vs cumulative evaluation count). Determinism comes
 * entirely from the caller: the objective fn is seeded, this module holds no RNG.
 *
 * Units live in the caller (the decision variable carries its own unit). This module
 * is unit-agnostic: it minimizes a scalar `objective(x)` over `x ∈ [lo, hi]`.
 */

/** One recorded objective evaluation. */
export interface Evaluation {
  /** Decision-variable value evaluated. */
  x: number;
  /** Objective value (lower is better). NaN if the evaluation failed. */
  y: number;
  /** 'grid' (surface sweep) or 'refine' (pattern-search probe). */
  phase: 'grid' | 'refine';
}

/** A point on the best-so-far convergence trace. */
export interface ConvergencePoint {
  /** Cumulative number of objective evaluations spent so far. */
  evals: number;
  /** Best (lowest) objective seen up to and including this evaluation. */
  bestY: number;
  /** Decision-variable value achieving `bestY`. */
  bestX: number;
}

export interface OptimizeResult {
  /** Argmin found over the bound. */
  bestX: number;
  /** Objective at `bestX`. */
  bestY: number;
  /** Every evaluation, in order (grid sweep then refinement probes). */
  evaluations: Evaluation[];
  /** Best-so-far trace, one point per evaluation. */
  convergence: ConvergencePoint[];
}

export interface OptimizeOptions {
  /** Number of coarse grid points across [lo, hi] (>= 2). */
  gridPoints: number;
  /** Pattern-search refinement iterations (step halves each iteration). */
  refineIters: number;
}

/** Lower of two evaluations, treating NaN as +∞ (a failed run never "wins"). */
function better(aY: number, bY: number): boolean {
  if (!Number.isFinite(aY)) return false;
  if (!Number.isFinite(bY)) return true;
  return aY < bY;
}

/**
 * Minimize `objective(x)` over `x ∈ [lo, hi]` with a coarse grid + local pattern
 * search. `objective` is async (one engine-backed MC per call) and may return NaN
 * for a failed evaluation; such points are recorded but never selected.
 *
 * The total evaluation budget is bounded: `gridPoints` for the sweep plus up to
 * `2·refineIters` probes for the refinement (two compass directions per iteration),
 * so the caller can size the cost up front.
 */
export async function optimize1d(
  objective: (x: number) => Promise<number>,
  lo: number,
  hi: number,
  opts: OptimizeOptions,
): Promise<OptimizeResult> {
  const gridPoints = Math.max(2, Math.floor(opts.gridPoints));
  const refineIters = Math.max(0, Math.floor(opts.refineIters));
  const span = hi - lo;

  const evaluations: Evaluation[] = [];
  const convergence: ConvergencePoint[] = [];
  let bestX = lo;
  let bestY = Number.POSITIVE_INFINITY;

  // Memoize so refinement probes that land on an already-evaluated x don't re-run
  // the (expensive) engine MC. Keyed by a rounded x to absorb FP noise.
  const cache = new Map<string, number>();
  const keyOf = (x: number): string => x.toFixed(6);

  const record = async (x: number, phase: 'grid' | 'refine'): Promise<number> => {
    const k = keyOf(x);
    let y = cache.get(k);
    if (y === undefined) {
      y = await objective(x);
      cache.set(k, y);
    }
    evaluations.push({ x, y, phase });
    if (better(y, bestY)) {
      bestY = y;
      bestX = x;
    }
    convergence.push({ evals: evaluations.length, bestY, bestX });
    return y;
  };

  // 1) Coarse grid sweep → the plotted objective surface + a global-ish seed.
  for (let i = 0; i < gridPoints; i++) {
    const f = gridPoints > 1 ? i / (gridPoints - 1) : 0;
    const x = lo + f * span;
    // eslint-disable-next-line no-await-in-loop
    await record(x, 'grid');
  }

  // 2) Local pattern search around the grid best. Start the step at one grid cell
  //    and contract by half each iteration; probe both neighbours, recentre on the
  //    best, then contract. `record` keeps `bestX`/`bestY` the running incumbent.
  let step = gridPoints > 1 ? span / (gridPoints - 1) : span;
  for (let it = 0; it < refineIters && step > 0; it++) {
    for (const dir of [-1, 1]) {
      const x = bestX + dir * step;
      if (x < lo || x > hi) continue;
      // eslint-disable-next-line no-await-in-loop
      await record(x, 'refine');
    }
    step *= 0.5;
  }

  if (!Number.isFinite(bestY)) {
    // Every evaluation failed; report the lower bound as a defined-but-degenerate best.
    bestX = lo;
    bestY = NaN;
  }
  return { bestX, bestY, evaluations, convergence };
}
