/**
 * Optimizer Studio (issue #121) — closed-loop auto-tune over the C++ engine.
 *
 * This studio drives the WASM core in a closed optimization loop: it picks an
 * **objective** and a single **decision variable**, sweeps the decision variable to
 * build the *objective surface*, then runs a small 1-D optimizer (coarse grid → local
 * pattern search, lib/studio/studios/optimizerMath.ts) to find the value that
 * minimizes the objective.
 *
 * Each objective evaluation is a **small seeded Monte Carlo** through the engine — the
 * same dispersed-IC approach the UQ Studio (#112) and MonteCarloPanel use: draw seeded
 * Gaussian dispersions on launch speed / launch elevation / target position, run
 * `await runSim(cfg)` per case, and reduce the ensemble to one scalar:
 *
 *   - **Minimize CEP**            → median miss distance [m] (the 50% radius).
 *   - **Minimize mean miss**      → mean miss distance [m].
 *   - **Min control effort @ Pk** → mean integrated commanded-accel effort [m/s], with
 *     a soft penalty when the ensemble hit fraction P(kill) drops below a floor (so the
 *     optimizer trades effort *at fixed kill probability*, not by missing on purpose).
 *
 * Decision variables (each with a search bound, units in names):
 *   - **nav_constant N**          [-]   pro-nav gain.
 *   - **launch_elevation_deg**    [deg] launch elevation.
 *   - **guidance max_accel**      [m/s²] commanded-accel saturation.
 *
 * Plots: (1) objective vs decision variable (the surface) with the optimum starred and
 * the default marked; (2) optimizer convergence (best-so-far objective vs cumulative
 * evaluations); (3) a before/after table comparing the objective at the default vs the
 * optimum, plus the optimal value readout.
 *
 * Determinism: every dispersion draw flows through the seeded Prng (lib/prng.ts) keyed
 * off the studio seed *and the candidate value*, so the whole study reproduces from one
 * integer. Responsiveness: the evaluation budget is capped — `gridPoints` surface
 * evaluations + `2·refineIters` refinement probes, each a `mcCases`-case MC — so the
 * worst-case engine-run count is bounded and shown in the readout.
 *
 * MOCK mode (no WASM artifact): every `runSim` returns the same committed sample, so the
 * objective is flat (constant) over the bound and the optimum is degenerate. We detect
 * this (warmUp + isMockMode) and label every plot, never crash (mirrors uqStudio /
 * MonteCarloPanel).
 */

import type { StudioDef, ParamValues, PlotSpec, EnumOption } from '../types';
import type { SimConfig, SimResult } from '@/lib/types';
import { runSim, isMockMode, isResolved, warmUp } from '@/lib/wasmRunner';
import { Prng } from '@/lib/prng';
import { cepStat, hitFractionStat } from './uqMath';
import { optimize1d } from './optimizerMath';

const BASE_PRESET_URL = '/scenarios/homing_3dof.json';

// Evaluation-budget caps for in-browser responsiveness.
const MAX_MC_CASES = 40;
const MAX_GRID_POINTS = 11;
const REFINE_ITERS = 4; // → up to 2·4 = 8 refinement probes.

// Pk floor for the control-effort objective (fraction of cases that must intercept).
const PK_FLOOR = 0.8;
// Penalty weight [m/s of effort per unit Pk shortfall] — large so Pk dominates effort.
const PK_PENALTY = 5000;

const TEAL = '#4fd1c5';
const CORAL = '#fc8181';
const BLUE = '#90cdf4';
const AMBER = '#f6ad55';

// ----------------------------------------------------------------------------
// Decision variables + objectives (enum options + their config/metric plumbing)
// ----------------------------------------------------------------------------

type DecisionKey = 'nav_constant' | 'launch_elevation_deg' | 'max_accel';

interface DecisionSpec {
  key: DecisionKey;
  label: string;
  unit: string;
  /** Default value (read from the baseline config when available). */
  defaultValue: number;
  /** Search bound the optimizer sweeps. */
  bound: [number, number];
  /** Apply a candidate value onto a (dispersed) config. */
  apply: (cfg: SimConfig, value: number) => SimConfig;
}

const DECISIONS: Record<DecisionKey, DecisionSpec> = {
  nav_constant: {
    key: 'nav_constant',
    label: 'nav constant N',
    unit: '',
    defaultValue: 4,
    bound: [2, 6],
    apply: (cfg, v) => ({ ...cfg, guidance: { ...cfg.guidance, nav_constant: v } }),
  },
  launch_elevation_deg: {
    key: 'launch_elevation_deg',
    label: 'launch elevation',
    unit: 'deg',
    defaultValue: 42,
    bound: [25, 60],
    apply: (cfg, v) => ({
      ...cfg,
      vehicle: { ...cfg.vehicle, launch_elevation_deg: v },
    }),
  },
  max_accel: {
    key: 'max_accel',
    label: 'guidance max accel',
    unit: 'm/s²',
    defaultValue: 400,
    bound: [150, 600],
    apply: (cfg, v) => ({ ...cfg, guidance: { ...cfg.guidance, max_accel: v } }),
  },
};

const DECISION_OPTIONS: EnumOption[] = [
  { value: 'nav_constant', label: 'Nav constant N [-]' },
  { value: 'launch_elevation_deg', label: 'Launch elevation [deg]' },
  { value: 'max_accel', label: 'Guidance max accel [m/s²]' },
];

type ObjectiveKey = 'min_cep' | 'min_mean_miss' | 'min_effort_at_pk';

interface ObjectiveSpec {
  key: ObjectiveKey;
  label: string;
  unit: string;
}

const OBJECTIVES: Record<ObjectiveKey, ObjectiveSpec> = {
  min_cep: { key: 'min_cep', label: 'Minimize CEP', unit: 'm' },
  min_mean_miss: { key: 'min_mean_miss', label: 'Minimize mean miss', unit: 'm' },
  min_effort_at_pk: {
    key: 'min_effort_at_pk',
    label: `Minimize control effort @ P(kill) ≥ ${PK_FLOOR}`,
    unit: 'm/s',
  },
};

const OBJECTIVE_OPTIONS: EnumOption[] = [
  { value: 'min_cep', label: 'Minimize CEP [m]' },
  { value: 'min_mean_miss', label: 'Minimize mean miss [m]' },
  { value: 'min_effort_at_pk', label: `Min control effort @ P(kill) ≥ ${PK_FLOOR}` },
];

// ----------------------------------------------------------------------------
// Baseline config (memoized)
// ----------------------------------------------------------------------------

let basePromise: Promise<SimConfig> | null = null;

function loadBaseConfig(): Promise<SimConfig> {
  if (!basePromise) {
    basePromise = fetch(BASE_PRESET_URL)
      .then((res) => {
        if (!res.ok) throw new Error(`base preset ${res.status}`);
        return res.json() as Promise<SimConfig>;
      })
      .catch((e) => {
        basePromise = null;
        throw e;
      });
  }
  return basePromise;
}

// ----------------------------------------------------------------------------
// Per-case dispersion (mirrors uqStudio / MonteCarloPanel) + metric extraction
// ----------------------------------------------------------------------------

interface Dispersion {
  launchSpeedSigmaMps: number;
  launchElevationSigmaDeg: number;
  targetPosSigmaM: number;
}

/** Apply seeded Gaussian IC dispersions to a config (units carried in names). */
function disperseConfig(
  base: SimConfig,
  seed: number,
  rng: Prng,
  sigma: Dispersion,
): SimConfig {
  const [tx, ty, tz] = base.target.pos0;
  return {
    ...base,
    seed,
    vehicle: {
      ...base.vehicle,
      launch_speed: base.vehicle.launch_speed + rng.gaussian(0, sigma.launchSpeedSigmaMps),
      launch_elevation_deg:
        base.vehicle.launch_elevation_deg + rng.gaussian(0, sigma.launchElevationSigmaDeg),
    },
    target: {
      ...base.target,
      pos0: [
        tx + rng.gaussian(0, sigma.targetPosSigmaM),
        ty,
        tz + rng.gaussian(0, sigma.targetPosSigmaM),
      ],
    },
  };
}

/** Integrated commanded-accel "effort" [m/s] = ∫|a_cmd| dt (trapezoid over the run). */
function controlEffortMps(r: SimResult): number {
  const s = r.series;
  const t = s.t;
  const n = t.length;
  if (n < 2) return 0;
  let effort = 0;
  let prevMag = Math.hypot(s.accel_cmd_x[0], s.accel_cmd_y[0], s.accel_cmd_z[0]);
  for (let i = 1; i < n; i++) {
    const mag = Math.hypot(s.accel_cmd_x[i], s.accel_cmd_y[i], s.accel_cmd_z[i]);
    effort += 0.5 * (mag + prevMag) * (t[i] - t[i - 1]);
    prevMag = mag;
  }
  return effort;
}

/** One MC ensemble's raw per-case channels. */
interface Ensemble {
  miss_m: number[];
  hitFlags: number[];
  effort_mps: number[];
}

/**
 * Evaluate one `mcCases`-case dispersed Monte Carlo for a fixed decision value, all
 * runs sequential (keeps the browser responsive). The ensemble RNG is seeded off the
 * master seed *and the candidate value index* so each candidate is reproducible yet
 * distinct, and re-evaluating the same candidate replays identically.
 */
async function runEnsemble(
  base: SimConfig,
  decision: DecisionSpec,
  value: number,
  mcCases: number,
  sigma: Dispersion,
  ensembleSeed: number,
): Promise<Ensemble> {
  const rng = new Prng(ensembleSeed);
  const miss_m: number[] = [];
  const hitFlags: number[] = [];
  const effort_mps: number[] = [];
  for (let i = 0; i < mcCases; i++) {
    const caseSeed = rng.nextUint32();
    const dispersed = disperseConfig(base, caseSeed, rng, sigma);
    const cfg = decision.apply(dispersed, value);
    // eslint-disable-next-line no-await-in-loop
    const r = await runSim(cfg).catch(() => null);
    if (r) {
      miss_m.push(r.miss_distance);
      hitFlags.push(r.intercept ? 1 : 0);
      effort_mps.push(controlEffortMps(r));
    }
  }
  return { miss_m, hitFlags, effort_mps };
}

/** Reduce an ensemble to the scalar objective (lower is better). NaN if empty. */
function objectiveValue(ens: Ensemble, objective: ObjectiveKey): number {
  const finiteMiss = ens.miss_m.filter((m) => Number.isFinite(m));
  if (finiteMiss.length === 0) return NaN;
  switch (objective) {
    case 'min_cep':
      return cepStat(finiteMiss);
    case 'min_mean_miss':
      return finiteMiss.reduce((a, b) => a + b, 0) / finiteMiss.length;
    case 'min_effort_at_pk': {
      const finiteEffort = ens.effort_mps.filter((e) => Number.isFinite(e));
      const meanEffort =
        finiteEffort.length > 0
          ? finiteEffort.reduce((a, b) => a + b, 0) / finiteEffort.length
          : NaN;
      const pk = hitFractionStat(ens.hitFlags);
      const shortfall = Math.max(0, PK_FLOOR - pk);
      return meanEffort + PK_PENALTY * shortfall;
    }
    default:
      return NaN;
  }
}

/** A stable child seed for a candidate value (so the same x always replays). */
function seedForValue(masterSeed: number, value: number): number {
  // Mix the candidate (rounded to absorb FP noise) into the master seed.
  const q = Math.round(value * 1000);
  return (Math.imul(masterSeed ^ q, 0x9e3779b1) ^ (q << 7)) >>> 0;
}

function titleWithMock(title: string, mock: boolean): string {
  return mock ? `${title} — MOCK MODE (degenerate: no WASM artifact)` : title;
}

// ----------------------------------------------------------------------------
// Studio definition
// ----------------------------------------------------------------------------

export const optimizerStudio: StudioDef = {
  id: 'optimizer',
  label: 'Optimizer',
  description:
    'Closed-loop auto-tune over the C++ engine: pick an objective (min CEP / min mean ' +
    'miss / min control effort at fixed P(kill)) and a decision variable (nav constant ' +
    'N / launch elevation / guidance max accel), and the studio sweeps the bound to ' +
    'build the objective surface, then runs a 1-D optimizer (coarse grid → local ' +
    'pattern search) to find the optimum — each evaluation a small seeded Monte Carlo ' +
    'via runSim. Seeded and reproducible. In mock mode (no WASM) every run returns the ' +
    'same sample, so the objective is flat and the optimum degenerate — plots are labelled.',
  params: [
    {
      kind: 'enum',
      key: 'objective',
      label: 'Objective',
      options: OBJECTIVE_OPTIONS,
      default_value: 'min_cep',
      help: 'The scalar (lower is better) each Monte Carlo evaluation reduces to.',
    },
    {
      kind: 'enum',
      key: 'decision',
      label: 'Decision variable',
      options: DECISION_OPTIONS,
      default_value: 'nav_constant',
      help: 'The single variable the optimizer tunes over its search bound.',
    },
    {
      kind: 'number',
      key: 'mc_cases',
      label: 'MC cases / evaluation',
      min_value: 6,
      max_value: MAX_MC_CASES,
      step: 2,
      default_value: 16,
      help: `Dispersed engine runs per objective evaluation (capped at ${MAX_MC_CASES}).`,
    },
    {
      kind: 'number',
      key: 'grid_points',
      label: 'Surface grid points',
      min_value: 4,
      max_value: MAX_GRID_POINTS,
      step: 1,
      default_value: 7,
      help: `Coarse-sweep points across the bound (capped at ${MAX_GRID_POINTS}); seeds the optimizer.`,
    },
    {
      kind: 'number',
      key: 'launch_speed_sigma_mps',
      label: 'Launch speed σ',
      unit: 'm/s',
      min_value: 0,
      max_value: 50,
      step: 1,
      default_value: 12,
      help: '1σ Gaussian dispersion on launch speed (per case).',
    },
    {
      kind: 'number',
      key: 'launch_elevation_sigma_deg',
      label: 'Launch elevation σ',
      unit: 'deg',
      min_value: 0,
      max_value: 3,
      step: 0.1,
      default_value: 0.4,
      help: '1σ Gaussian dispersion on launch elevation (per case).',
    },
    {
      kind: 'number',
      key: 'target_pos_sigma_m',
      label: 'Target position σ',
      unit: 'm',
      min_value: 0,
      max_value: 200,
      step: 5,
      default_value: 40,
      help: '1σ Gaussian dispersion on the target initial X/Z position (per case).',
    },
    {
      kind: 'number',
      key: 'seed',
      label: 'Master seed',
      min_value: 1,
      max_value: 100000,
      step: 1,
      default_value: 1,
      help: 'Seeds the whole study (dispersions per candidate) — reproducible.',
    },
  ],
  presets: [
    {
      id: 'cep-vs-n',
      label: 'CEP vs nav constant N',
      values: {
        objective: 'min_cep',
        decision: 'nav_constant',
        mc_cases: 16,
        grid_points: 7,
        launch_speed_sigma_mps: 12,
        launch_elevation_sigma_deg: 0.4,
        target_pos_sigma_m: 40,
        seed: 1,
      },
    },
    {
      id: 'miss-vs-elevation',
      label: 'Mean miss vs launch elevation',
      values: {
        objective: 'min_mean_miss',
        decision: 'launch_elevation_deg',
        mc_cases: 16,
        grid_points: 8,
        launch_speed_sigma_mps: 12,
        launch_elevation_sigma_deg: 0.4,
        target_pos_sigma_m: 40,
        seed: 1,
      },
    },
    {
      id: 'effort-vs-accel',
      label: 'Control effort @ Pk vs max accel',
      values: {
        objective: 'min_effort_at_pk',
        decision: 'max_accel',
        mc_cases: 20,
        grid_points: 7,
        launch_speed_sigma_mps: 12,
        launch_elevation_sigma_deg: 0.4,
        target_pos_sigma_m: 40,
        seed: 3,
      },
    },
  ],

  async compute(params: ParamValues): Promise<PlotSpec[]> {
    const objKey = String(params.objective) as ObjectiveKey;
    const decKey = String(params.decision) as DecisionKey;
    const objective = OBJECTIVES[objKey] ?? OBJECTIVES.min_cep;
    const baseDecision = DECISIONS[decKey] ?? DECISIONS.nav_constant;

    const mcCases = Math.max(6, Math.min(MAX_MC_CASES, Math.round(Number(params.mc_cases))));
    const gridPoints = Math.max(4, Math.min(MAX_GRID_POINTS, Math.round(Number(params.grid_points))));
    const sigma: Dispersion = {
      launchSpeedSigmaMps: Number(params.launch_speed_sigma_mps),
      launchElevationSigmaDeg: Number(params.launch_elevation_sigma_deg),
      targetPosSigmaM: Number(params.target_pos_sigma_m),
    };
    const seed = Math.max(1, Math.round(Number(params.seed)));

    await warmUp();
    const base = await loadBaseConfig();
    const mock = isResolved() && isMockMode();

    // Pull the live default for the chosen decision variable off the baseline config.
    let defaultValue = baseDecision.defaultValue;
    if (decKey === 'nav_constant') defaultValue = base.guidance.nav_constant;
    else if (decKey === 'launch_elevation_deg') defaultValue = base.vehicle.launch_elevation_deg;
    else if (decKey === 'max_accel') defaultValue = base.guidance.max_accel;
    const decision: DecisionSpec = { ...baseDecision, defaultValue };

    const [lo, hi] = decision.bound;
    const unitSuffix = objective.unit ? ` [${objective.unit}]` : '';
    const decUnit = decision.unit ? ` [${decision.unit}]` : '';

    // The closed-loop objective: one seeded MC ensemble → one scalar.
    const evalObjective = async (value: number): Promise<number> => {
      const ens = await runEnsemble(
        base,
        decision,
        value,
        mcCases,
        sigma,
        seedForValue(seed, value),
      );
      return objectiveValue(ens, objKey);
    };

    // Run the optimizer (coarse grid surface + local pattern-search refinement).
    const result = await optimize1d(evalObjective, lo, hi, {
      gridPoints,
      refineIters: REFINE_ITERS,
    });

    // Default-vs-optimum: evaluate the objective at the live default (one more MC).
    const defaultObjective = await evalObjective(defaultValue);
    const optX = result.bestX;
    const optY = result.bestY;

    // -------------------------------------------------------------------------
    // Plot 1: objective surface vs decision variable, optimum starred + default.
    // -------------------------------------------------------------------------
    const gridEvals = result.evaluations.filter((e) => e.phase === 'grid');
    const refineEvals = result.evaluations.filter((e) => e.phase === 'refine');
    const surfacePlot: PlotSpec = {
      id: 'surface',
      title: titleWithMock(`${objective.label}: objective surface vs ${decision.label}`, mock),
      data: [
        {
          x: gridEvals.map((e) => e.x),
          y: gridEvals.map((e) => e.y),
          type: 'scatter',
          mode: 'lines+markers',
          name: 'objective surface (grid)',
          line: { color: TEAL, width: 2 },
          marker: { color: TEAL, size: 6 },
        },
        {
          x: refineEvals.map((e) => e.x),
          y: refineEvals.map((e) => e.y),
          type: 'scatter',
          mode: 'markers',
          name: 'refinement probes',
          marker: { color: BLUE, size: 7, symbol: 'circle-open' },
        },
        {
          x: [defaultValue],
          y: [defaultObjective],
          type: 'scatter',
          mode: 'markers',
          name: 'default',
          marker: { color: AMBER, size: 12, symbol: 'diamond' },
        },
        {
          x: [optX],
          y: [optY],
          type: 'scatter',
          mode: 'markers',
          name: 'optimum',
          marker: { color: CORAL, size: 16, symbol: 'star' },
        },
      ],
      layout: {
        xaxis: { title: { text: `${decision.label}${decUnit}` } },
        yaxis: { title: { text: `${objective.label}${unitSuffix}` } },
        annotations: Number.isFinite(optY)
          ? [
              {
                x: optX,
                y: optY,
                text: `optimum: ${optX.toFixed(2)} → ${optY.toFixed(1)}`,
                showarrow: true,
                arrowhead: 2,
                ax: 0,
                ay: -32,
                font: { color: CORAL },
              },
            ]
          : [],
      },
    };

    // -------------------------------------------------------------------------
    // Plot 2: optimizer convergence — best-so-far objective vs evaluations.
    // -------------------------------------------------------------------------
    const conv = result.convergence;
    const convPlot: PlotSpec = {
      id: 'convergence',
      title: titleWithMock('Optimizer convergence — best objective vs evaluations', mock),
      data: [
        {
          x: conv.map((p) => p.evals),
          y: conv.map((p) => p.bestY),
          type: 'scatter',
          mode: 'lines+markers',
          name: 'best-so-far',
          line: { color: TEAL, width: 2, shape: 'hv' },
          marker: { color: TEAL, size: 5 },
        },
        {
          x: [gridPoints],
          y: [conv[gridPoints - 1]?.bestY ?? optY],
          type: 'scatter',
          mode: 'markers',
          name: 'grid → refine',
          marker: { color: AMBER, size: 10, symbol: 'square' },
        },
      ],
      layout: {
        xaxis: { title: { text: 'Cumulative objective evaluations' } },
        yaxis: { title: { text: `Best ${objective.label}${unitSuffix}` } },
      },
    };

    // -------------------------------------------------------------------------
    // Plot 3: before/after table + optimal value readout + eval budget.
    // -------------------------------------------------------------------------
    const improveAbs = defaultObjective - optY;
    const improvePct =
      Number.isFinite(defaultObjective) && defaultObjective !== 0
        ? (improveAbs / defaultObjective) * 100
        : NaN;
    const maxBudget = gridPoints + 2 * REFINE_ITERS; // surface + refinement probes
    const fmt = (v: number): string => (Number.isFinite(v) ? v.toFixed(2) : '—');

    const rows: [string, string][] = [
      ['Objective', objective.label],
      ['Decision variable', `${decision.label}${decUnit}`],
      ['Search bound', `[${lo}, ${hi}]`],
      [`Default ${decision.label}`, `${fmt(defaultValue)}${decision.unit ? ' ' + decision.unit : ''}`],
      [`Optimal ${decision.label}`, `${fmt(optX)}${decision.unit ? ' ' + decision.unit : ''}`],
      [`Objective @ default${unitSuffix}`, fmt(defaultObjective)],
      [`Objective @ optimum${unitSuffix}`, fmt(optY)],
      [
        'Improvement',
        Number.isFinite(improvePct)
          ? `${improveAbs >= 0 ? '−' : '+'}${Math.abs(improveAbs).toFixed(1)}${unitSuffix} (${improvePct.toFixed(0)}% better)`
          : '—',
      ],
      ['MC cases / evaluation', `${mcCases}`],
      ['Objective evaluations', `${result.evaluations.length} (≤ ${maxBudget})`],
      [
        'Engine runs (≈)',
        `${result.evaluations.length * mcCases} (≤ ${(maxBudget + 1) * mcCases})`,
      ],
    ];
    const readoutPlot: PlotSpec = {
      id: 'readout',
      title: titleWithMock('Before / after + optimal value readout', mock),
      data: [
        {
          type: 'table',
          header: {
            values: ['<b>Quantity</b>', '<b>Value</b>'],
            align: 'left',
            fill: { color: '#1f2a36' },
            font: { color: '#e2e8f0' },
          },
          cells: {
            values: [rows.map((r) => r[0]), rows.map((r) => r[1])],
            align: 'left',
            fill: { color: ['#0f1620'] },
            font: { color: ['#e2e8f0', AMBER] },
            height: 26,
          },
          // The `table` trace isn't in this @types/plotly.js Data union, but the
          // runtime Plotly bundle renders it; cast through Data.
        } as unknown as PlotSpec['data'][number],
      ],
      layout: { margin: { t: 10, b: 10, l: 10, r: 10 } },
    };

    return [surfacePlot, convPlot, readoutPlot];
  },
};
