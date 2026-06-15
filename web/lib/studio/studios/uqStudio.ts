/**
 * UQ / Credibility Studio (issue #112) — an engine-backed, client-side
 * uncertainty-quantification studio plugged into the generic <StudioShell>.
 *
 * It mirrors the existing Monte Carlo panel (web/components/MonteCarloPanel.tsx):
 * it loads the baseline homing config, draws seeded Gaussian dispersions on launch
 * speed / launch elevation / target position (the same IC dispersion the C++ driver
 * `core/src/scenario/MonteCarlo.cpp` applies), and `await runSim(cfg)` per case to
 * collect the miss distance. On top of that ensemble it layers the UQ tools ported
 * from `postproc/gncpost/uq.py` (lib/studio/studios/uqMath.ts):
 *
 *   1. **CEP with a bootstrap confidence band** + a miss-distance histogram.
 *   2. **Convergence** — running CEP vs N with a CI band (the "enough cases?" view).
 *   3. **Sobol first/total-order ranking** — what drives the miss over the dispersion
 *      inputs, via a small hand-rolled Saltelli design evaluated through the engine.
 *   4. A **credibility scorecard** mini-table derived cheaply from the above.
 *
 * Determinism: every random draw flows through the seeded `Prng` (lib/prng.ts) keyed
 * off the studio's `seed` param, so the whole studio reproduces from one integer.
 *
 * MOCK mode (no WASM artifact): every `runSim` returns the same committed sample, so
 * the dispersion collapses to a single repeated miss — CEP has zero spread, the
 * histogram is one spike and Sobol is degenerate (all-zero). We detect this and add a
 * clear note to every plot title instead of crashing (mirrors MonteCarloPanel's mock
 * handling).
 *
 * Units follow the repo convention (units in names): `*_mps`, `*_deg`, `*_m`.
 */

import type { StudioDef, ParamValues, PlotSpec } from '../types';
import type { SimConfig, SimResult } from '@/lib/types';
import { runSim, isMockMode, isResolved, warmUp } from '@/lib/wasmRunner';
import { Prng } from '@/lib/prng';
import {
  cepStat,
  hitFractionStat,
  bootstrapCi,
  convergence,
  sobolIndicesAsync,
  sobolRanking,
  percentile,
} from './uqMath';

const BASE_PRESET_URL = '/scenarios/homing_3dof.json';
const MAX_CASES = 200;

// Sobol design size (nBase). Total engine runs for the Sobol leg = nBase·(d+2) with
// d = 4 dispersion inputs → nBase·6. Kept small for in-browser responsiveness.
const SOBOL_NBASE = 8;

// Dispersion-input names (and the order the Sobol bounds follow).
const SOBOL_INPUT_NAMES = [
  'launch speed',
  'launch elevation',
  'target pos X',
  'target pos Z',
];

let basePromise: Promise<SimConfig> | null = null;

/** Fetch (once, memoized) the baseline homing config the studio disperses. */
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

interface Dispersion {
  launchSpeedSigmaMps: number;
  launchElevationSigmaDeg: number;
  targetPosSigmaM: number;
}

/** One dispersed config from explicit per-input offsets (units carried in names). */
function disperseConfigWith(
  base: SimConfig,
  seed: number,
  dLaunchSpeedMps: number,
  dLaunchElevationDeg: number,
  dTargetXM: number,
  dTargetZM: number,
): SimConfig {
  const [tx, ty, tz] = base.target.pos0;
  return {
    ...base,
    seed,
    vehicle: {
      ...base.vehicle,
      launch_speed: base.vehicle.launch_speed + dLaunchSpeedMps,
      launch_elevation_deg: base.vehicle.launch_elevation_deg + dLaunchElevationDeg,
    },
    target: { ...base.target, pos0: [tx + dTargetXM, ty, tz + dTargetZM] },
  };
}

/** Run one case and return its miss distance [m] (NaN if the run failed). */
async function runMiss(cfg: SimConfig): Promise<number> {
  try {
    const r: SimResult = await runSim(cfg);
    return r.miss_distance;
  } catch {
    return NaN;
  }
}

/** A monochromatic, theme-friendly palette. */
const TEAL = '#4fd1c5';
const TEAL_FILL = 'rgba(79, 209, 197, 0.18)';
const CORAL = '#fc8181';
const BLUE = '#90cdf4';
const AMBER = '#f6ad55';

/** Append a mock-mode caveat to a plot title when WASM is absent. */
function titleWithMock(title: string, mock: boolean): string {
  return mock ? `${title} — MOCK MODE (degenerate: no WASM artifact)` : title;
}

export const uqStudio: StudioDef = {
  id: 'uq-credibility',
  label: 'UQ / Credibility',
  description:
    'Engine-backed client-side Monte Carlo with uncertainty quantification: CEP with a ' +
    'bootstrap confidence band + miss histogram, convergence (running CEP vs N with a CI ' +
    'band), and a Sobol first/total-order ranking of what drives the miss. Mirrors ' +
    'postproc/gncpost/uq.py; seeded and reproducible. In mock mode (no WASM) every run ' +
    'returns the same sample, so the distributions are degenerate — plots are labelled.',
  params: [
    {
      kind: 'number',
      key: 'num_cases',
      label: 'Monte Carlo cases',
      min_value: 10,
      max_value: MAX_CASES,
      step: 10,
      default_value: 60,
      help: `Number of dispersed engine runs (capped at ${MAX_CASES} for responsiveness).`,
    },
    {
      kind: 'number',
      key: 'launch_speed_sigma_mps',
      label: 'Launch speed σ',
      unit: 'm/s',
      min_value: 0,
      max_value: 50,
      step: 1,
      default_value: 15,
      help: '1σ Gaussian dispersion on launch speed.',
    },
    {
      kind: 'number',
      key: 'launch_elevation_sigma_deg',
      label: 'Launch elevation σ',
      unit: 'deg',
      min_value: 0,
      max_value: 3,
      step: 0.1,
      default_value: 0.5,
      help: '1σ Gaussian dispersion on launch elevation.',
    },
    {
      kind: 'number',
      key: 'target_pos_sigma_m',
      label: 'Target position σ',
      unit: 'm',
      min_value: 0,
      max_value: 200,
      step: 5,
      default_value: 50,
      help: '1σ Gaussian dispersion on the target initial X/Z position.',
    },
    {
      kind: 'number',
      key: 'seed',
      label: 'Master seed',
      min_value: 1,
      max_value: 100000,
      step: 1,
      default_value: 1,
      help: 'Seeds the whole studio (dispersions, bootstrap, Sobol) — reproducible.',
    },
  ],
  presets: [
    {
      id: 'quick',
      label: 'Quick (30 cases)',
      values: {
        num_cases: 30,
        launch_speed_sigma_mps: 15,
        launch_elevation_sigma_deg: 0.5,
        target_pos_sigma_m: 50,
        seed: 1,
      },
    },
    {
      id: 'tight',
      label: 'Tight dispersion',
      values: {
        num_cases: 100,
        launch_speed_sigma_mps: 5,
        launch_elevation_sigma_deg: 0.2,
        target_pos_sigma_m: 20,
        seed: 1,
      },
    },
    {
      id: 'wide',
      label: 'Wide dispersion',
      values: {
        num_cases: 120,
        launch_speed_sigma_mps: 30,
        launch_elevation_sigma_deg: 1.5,
        target_pos_sigma_m: 120,
        seed: 7,
      },
    },
  ],

  async compute(params: ParamValues): Promise<PlotSpec[]> {
    const numCases = Math.max(10, Math.min(MAX_CASES, Math.round(Number(params.num_cases))));
    const sigma: Dispersion = {
      launchSpeedSigmaMps: Number(params.launch_speed_sigma_mps),
      launchElevationSigmaDeg: Number(params.launch_elevation_sigma_deg),
      targetPosSigmaM: Number(params.target_pos_sigma_m),
    };
    const seed = Math.max(1, Math.round(Number(params.seed)));

    // Make sure the WASM module (or the mock fallback) is resolved so isMockMode()
    // reports correctly, then load the baseline config we disperse.
    await warmUp();
    const base = await loadBaseConfig();
    const mock = isResolved() && isMockMode();

    // -------------------------------------------------------------------------
    // 1) The dispersed Monte Carlo ensemble (the same approach as MonteCarloPanel).
    // -------------------------------------------------------------------------
    const ensembleRng = new Prng(seed);
    const miss_m: number[] = [];
    const hitFlags: number[] = [];
    for (let i = 0; i < numCases; i++) {
      const caseSeed = ensembleRng.nextUint32();
      const cfg = disperseConfigWith(
        base,
        caseSeed,
        ensembleRng.gaussian(0, sigma.launchSpeedSigmaMps),
        ensembleRng.gaussian(0, sigma.launchElevationSigmaDeg),
        ensembleRng.gaussian(0, sigma.targetPosSigmaM),
        ensembleRng.gaussian(0, sigma.targetPosSigmaM),
      );
      // eslint-disable-next-line no-await-in-loop
      const r = await runSim(cfg).catch(() => null);
      if (r) {
        miss_m.push(r.miss_distance);
        hitFlags.push(r.intercept ? 1 : 0);
      }
    }

    const finite = miss_m.filter((m) => Number.isFinite(m));
    const n = finite.length;

    // Bootstrap CIs on CEP and P(hit) (seeded children of the master seed).
    const cepCi =
      n > 0
        ? bootstrapCi(finite, cepStat, { level: 0.95, nResamples: 1000, seed: seed ^ 0x9e3779b9 })
        : { estimate: NaN, low: NaN, high: NaN, level: 0.95, halfWidth: NaN };
    const pHitCi =
      n > 0
        ? bootstrapCi(hitFlags, hitFractionStat, {
            level: 0.95,
            nResamples: 1000,
            seed: seed ^ 0x85ebca6b,
          })
        : { estimate: NaN, low: NaN, high: NaN, level: 0.95, halfWidth: NaN };

    // -------------------------------------------------------------------------
    // Plot 1: CEP + bootstrap band, with a miss-distance histogram overlay.
    // -------------------------------------------------------------------------
    const cep = cepCi.estimate;
    const p95 = n > 0 ? percentile(finite, 95) : NaN;
    const cepPlot: PlotSpec = {
      id: 'cep',
      title: titleWithMock('CEP with 95% bootstrap band + miss histogram', mock),
      data: [
        {
          x: finite,
          type: 'histogram',
          name: 'Miss distance',
          marker: { color: BLUE },
          opacity: 0.85,
          // `nbinsx` is valid for histograms but absent from this @types/plotly.js
          // PlotData; the runtime Plotly supports it, so cast through Data.
          nbinsx: 24,
        } as unknown as PlotSpec['data'][number],
      ],
      layout: {
        xaxis: { title: { text: 'Miss distance [m]' } },
        yaxis: { title: { text: 'Count' } },
        shapes: Number.isFinite(cep)
          ? [
              // 95% bootstrap CI band on CEP (shaded vertical region).
              {
                type: 'rect',
                xref: 'x',
                yref: 'paper',
                x0: cepCi.low,
                x1: cepCi.high,
                y0: 0,
                y1: 1,
                fillcolor: TEAL_FILL,
                line: { width: 0 },
                layer: 'below',
              },
              // CEP line.
              {
                type: 'line',
                xref: 'x',
                yref: 'paper',
                x0: cep,
                x1: cep,
                y0: 0,
                y1: 1,
                line: { color: TEAL, width: 2 },
              },
            ]
          : [],
        annotations: Number.isFinite(cep)
          ? [
              {
                x: cep,
                yref: 'paper',
                y: 1,
                text: `CEP = ${cep.toFixed(1)} m (95% CI ±${cepCi.halfWidth.toFixed(1)})`,
                showarrow: false,
                font: { color: TEAL },
                yanchor: 'bottom',
              },
            ]
          : [],
      },
    };

    // -------------------------------------------------------------------------
    // Plot 2: Convergence — running CEP vs N with a bootstrap CI band.
    // -------------------------------------------------------------------------
    const conv = convergence(finite, cepStat, {
      level: 0.95,
      nPoints: 16,
      minN: Math.min(10, Math.max(5, Math.floor(n / 4))),
      nResamples: 600,
      seed: seed ^ 0xc2b2ae35,
    });
    const convNs = conv.map((p) => p.n);
    const convPlot: PlotSpec = {
      id: 'convergence',
      title: titleWithMock('Convergence — running CEP vs N (95% CI band)', mock),
      data:
        conv.length > 0
          ? [
              // Upper band edge, then lower with fill='tonexty' for a shaded ribbon.
              {
                x: convNs,
                y: conv.map((p) => p.high),
                type: 'scatter',
                mode: 'lines',
                line: { width: 0 },
                showlegend: false,
                hoverinfo: 'skip',
              },
              {
                x: convNs,
                y: conv.map((p) => p.low),
                type: 'scatter',
                mode: 'lines',
                line: { width: 0 },
                fill: 'tonexty',
                fillcolor: TEAL_FILL,
                name: '95% CI band',
                hoverinfo: 'skip',
              },
              {
                x: convNs,
                y: conv.map((p) => p.estimate),
                type: 'scatter',
                mode: 'lines+markers',
                name: 'running CEP',
                line: { color: TEAL, width: 2 },
                marker: { color: TEAL, size: 5 },
              },
            ]
          : [],
      layout: {
        xaxis: { title: { text: 'Number of cases N' }, type: 'log' },
        yaxis: { title: { text: 'CEP [m]' } },
      },
    };

    // -------------------------------------------------------------------------
    // Plot 3: Sobol first/total-order ranking — what drives the miss.
    //
    // Each Saltelli row is one dispersion-input vector (offsets in σ-units mapped to
    // physical offsets) → one engine run → miss. We sweep each input over ±3σ.
    // -------------------------------------------------------------------------
    const sobolRng = new Prng(seed ^ 0x27d4eb2f);
    const bounds: [number, number][] = [
      [-3 * sigma.launchSpeedSigmaMps, 3 * sigma.launchSpeedSigmaMps],
      [-3 * sigma.launchElevationSigmaDeg, 3 * sigma.launchElevationSigmaDeg],
      [-3 * sigma.targetPosSigmaM, 3 * sigma.targetPosSigmaM],
      [-3 * sigma.targetPosSigmaM, 3 * sigma.targetPosSigmaM],
    ];
    // Evaluate a batch of Saltelli rows through the engine (sequential to keep the
    // browser responsive; nBase·(d+2) total runs).
    const sobolModel = async (rows: number[][]): Promise<number[]> => {
      const out: number[] = [];
      for (const row of rows) {
        const caseSeed = sobolRng.nextUint32();
        const cfg = disperseConfigWith(base, caseSeed, row[0], row[1], row[2], row[3]);
        // eslint-disable-next-line no-await-in-loop
        out.push(await runMiss(cfg));
      }
      return out;
    };

    const sobol = await sobolIndicesAsync(sobolModel, bounds, SOBOL_INPUT_NAMES, {
      nBase: SOBOL_NBASE,
      rng: sobolRng,
    });
    const ranked = sobolRanking(sobol);
    // Order bars ascending by total so the biggest driver sits at the top (barh).
    const order = sobol.names
      .map((_, i) => i)
      .sort((a, b) => sobol.totalOrder[a] - sobol.totalOrder[b]);
    const sobolPlot: PlotSpec = {
      id: 'sobol',
      title: titleWithMock('Sobol sensitivity — what drives the miss', mock),
      data: [
        {
          x: order.map((i) => sobol.firstOrder[i]),
          y: order.map((i) => sobol.names[i]),
          type: 'bar',
          orientation: 'h',
          name: 'first-order Sᵢ',
          marker: { color: BLUE },
        },
        {
          x: order.map((i) => sobol.totalOrder[i]),
          y: order.map((i) => sobol.names[i]),
          type: 'bar',
          orientation: 'h',
          name: 'total-order S_Tᵢ',
          marker: { color: CORAL },
        },
      ],
      layout: {
        barmode: 'group',
        xaxis: { title: { text: 'Sobol index (fraction of miss variance)' }, range: [0, 1] },
        yaxis: { title: { text: '' }, automargin: true },
      },
    };

    // -------------------------------------------------------------------------
    // Plot 4: Credibility scorecard — a cheap derived mini-table.
    // -------------------------------------------------------------------------
    const pHit = pHitCi.estimate;
    // Relative CI half-width as a convergence proxy: <10% ≈ "converged".
    const cepRelHalf = Number.isFinite(cep) && cep > 0 ? cepCi.halfWidth / cep : NaN;
    const topDriver = ranked.length > 0 ? ranked[0] : { name: '—', total: NaN };
    const verdict = (ok: boolean, warn: boolean): string =>
      mock ? 'n/a (mock)' : ok ? 'good' : warn ? 'marginal' : 'thin';

    const rows: [string, string, string][] = [
      ['Cases run (finite)', `${n} / ${numCases}`, verdict(n >= 50, n >= 20)],
      [
        'CEP',
        Number.isFinite(cep)
          ? `${cep.toFixed(1)} m  (95% CI ±${cepCi.halfWidth.toFixed(1)})`
          : '—',
        '—',
      ],
      [
        'CEP CI half-width / CEP',
        Number.isFinite(cepRelHalf) ? `${(cepRelHalf * 100).toFixed(1)} %` : '—',
        verdict(cepRelHalf < 0.1, cepRelHalf < 0.25),
      ],
      [
        'P(hit)',
        Number.isFinite(pHit)
          ? `${(pHit * 100).toFixed(0)} %  (95% CI ±${(pHitCi.halfWidth * 100).toFixed(0)}%)`
          : '—',
        '—',
      ],
      ['95th-pct miss', Number.isFinite(p95) ? `${p95.toFixed(1)} m` : '—', '—'],
      [
        'Top miss driver (Sobol)',
        Number.isFinite(topDriver.total)
          ? `${topDriver.name} (S_T=${topDriver.total.toFixed(2)})`
          : '—',
        '—',
      ],
    ];
    const scorecardPlot: PlotSpec = {
      id: 'scorecard',
      title: titleWithMock('Credibility scorecard', mock),
      data: [
        {
          type: 'table',
          header: {
            values: ['<b>Metric</b>', '<b>Value</b>', '<b>Credibility</b>'],
            align: 'left',
            fill: { color: '#1f2a36' },
            font: { color: '#e2e8f0' },
          },
          cells: {
            values: [
              rows.map((r) => r[0]),
              rows.map((r) => r[1]),
              rows.map((r) => r[2]),
            ],
            align: 'left',
            fill: { color: ['#0f1620'] },
            font: { color: ['#e2e8f0', '#e2e8f0', AMBER] },
            height: 26,
          },
          // The `table` trace isn't in this @types/plotly.js Data union, but the
          // runtime Plotly bundle renders it; cast through Data.
        } as unknown as PlotSpec['data'][number],
      ],
      layout: { margin: { t: 10, b: 10, l: 10, r: 10 } },
    };

    return [cepPlot, convPlot, sobolPlot, scorecardPlot];
  },
};
