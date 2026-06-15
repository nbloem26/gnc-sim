/**
 * Campaign / BMC2 Studio (issue #111) — a many-on-many planner.
 *
 * The many-on-many engagement engine (`core/src/scenario/ManyOnMany.cpp`, issue
 * #45) is CLI/SDK-only — it is NOT exposed through the browser WASM `run_sim`,
 * which only runs a single engagement. So unlike an engine-backed studio, this
 * one CANNOT `await runSim(...)`; instead it mirrors the campaign math client-side
 * in pure TS (`campaignMath.ts`), faithful to the C++ logic, and overlays the BMC2
 * datalink latency/dropout degradation idea (issue #46).
 *
 * The model (all in campaignMath.ts, annotated against ManyOnMany.cpp):
 *   - per-pairing single-shot P(kill) via the Gaussian/Carleton lethality model
 *       Pssk = pk_max * exp(-0.5 * (miss / sigma_m)^2),
 *   - greedy weapon-target assignment (WTA): commit highest-Pk pairings first,
 *     one weapon per threat per wave,
 *   - doctrines salvo / shoot-look-shoot / raid playing out atop the WTA,
 *   - deterministic (expected-value) + seeded Monte-Carlo campaign rollups,
 *   - datalink degradation pk_eff = pk·(1−dropout)·exp(−latency/τ).
 *
 * Four plots:
 *   1. Leakage vs interceptor inventory (monotone → 0 once inventory ≥ threats).
 *   2. P(raid annihilation) vs shots-per-threat, for a few pk_max values.
 *   3. WTA assignment matrix heatmap (pairing Pk; committed cells marked).
 *   4. Latency/dropout sensitivity (campaign leakage vs latency for dropout slices).
 *
 * Units follow the repo convention (unit suffixes on physical params).
 */

import type { StudioDef, ParamValues, PlotSpec } from '../types';
import {
  scorePairings,
  degradeMatrix,
  runCampaign,
  type Doctrine,
  type CampaignParams,
  type PairingMatrix,
} from './campaignMath';

const ACCENT = '#4fd1c5';
const WARN = 'rgba(246, 173, 85, 0.95)';

function asDoctrine(v: unknown): Doctrine {
  const s = String(v);
  return s === 'salvo' || s === 'shoot_look_shoot' || s === 'raid'
    ? (s as Doctrine)
    : 'raid';
}

/** Build a degraded pairing matrix for a given interceptor count and link state. */
function buildMatrix(
  ni: number,
  nt: number,
  sigma_m: number,
  pk_max: number,
  seed: number,
  latency_s: number,
  dropout_prob: number,
): PairingMatrix {
  const base = scorePairings(ni, nt, sigma_m, pk_max, seed);
  return degradeMatrix(base, latency_s, dropout_prob);
}

export const campaignStudio: StudioDef = {
  id: 'campaign-bmc2',
  label: 'Campaign / BMC2 planner',
  description:
    'Many-on-many engagement planner: Gaussian-lethality P(kill), greedy weapon-target ' +
    'assignment, salvo / shoot-look-shoot / raid doctrines, and BMC2 datalink latency/dropout ' +
    'degradation. A client-side TS mirror of core/src/scenario/ManyOnMany.cpp (the engine is ' +
    'CLI/SDK-only, not in the browser WASM).',
  params: [
    {
      kind: 'number',
      key: 'num_interceptors',
      label: 'Interceptors (inventory)',
      min_value: 1,
      max_value: 30,
      step: 1,
      default_value: 8,
      help: 'Weapons available to the defender.',
    },
    {
      kind: 'number',
      key: 'num_threats',
      label: 'Threats (raid size)',
      min_value: 1,
      max_value: 30,
      step: 1,
      default_value: 8,
    },
    {
      kind: 'enum',
      key: 'doctrine',
      label: 'Doctrine',
      options: [
        { value: 'salvo', label: 'Salvo (N shots / threat)' },
        { value: 'shoot_look_shoot', label: 'Shoot-look-shoot' },
        { value: 'raid', label: 'Raid (exhaust inventory)' },
      ],
      default_value: 'raid',
    },
    {
      kind: 'number',
      key: 'shots_per_threat',
      label: 'Shots per threat (salvo)',
      min_value: 1,
      max_value: 6,
      step: 1,
      default_value: 2,
      help: 'Salvo doctrine: interceptors committed to each threat.',
    },
    {
      kind: 'number',
      key: 'pk_max',
      label: 'Single-shot Pk (max)',
      min_value: 0.1,
      max_value: 1,
      step: 0.05,
      default_value: 0.8,
      help: 'Peak single-shot P(kill) for a perfect (zero-miss) intercept.',
    },
    {
      kind: 'number',
      key: 'sigma_m',
      label: 'Lethality σ',
      unit: 'm',
      min_value: 1,
      max_value: 50,
      step: 1,
      default_value: 10,
      help: 'Effective lethal radius / sensor sigma in the Carleton Pk model.',
    },
    {
      kind: 'number',
      key: 'latency_s',
      label: 'Datalink latency',
      unit: 's',
      min_value: 0,
      max_value: 5,
      step: 0.1,
      default_value: 0,
      help: 'BMC2 command-link staleness; degrades Pk as exp(−latency/τ).',
    },
    {
      kind: 'number',
      key: 'dropout_prob',
      label: 'Datalink dropout',
      min_value: 0,
      max_value: 0.9,
      step: 0.05,
      default_value: 0,
      help: 'Probability a command-link cue is lost (the shot is wasted).',
    },
    {
      kind: 'number',
      key: 'num_trials',
      label: 'Monte-Carlo trials',
      min_value: 1,
      max_value: 2000,
      step: 1,
      default_value: 400,
      help: '1 = deterministic expected-value rollup only; >1 = seeded MC.',
    },
    {
      kind: 'number',
      key: 'seed',
      label: 'Seed',
      min_value: 0,
      max_value: 9999,
      step: 1,
      default_value: 1234,
    },
  ],
  presets: [
    {
      id: 'balanced-raid',
      label: 'Balanced raid (8v8)',
      values: {
        num_interceptors: 8,
        num_threats: 8,
        doctrine: 'raid',
        shots_per_threat: 2,
        pk_max: 0.8,
        sigma_m: 10,
        latency_s: 0,
        dropout_prob: 0,
        num_trials: 400,
        seed: 1234,
      },
    },
    {
      id: 'degraded-link',
      label: 'Degraded BMC2 link',
      values: {
        num_interceptors: 12,
        num_threats: 8,
        doctrine: 'shoot_look_shoot',
        shots_per_threat: 2,
        pk_max: 0.85,
        sigma_m: 12,
        latency_s: 2.5,
        dropout_prob: 0.3,
        num_trials: 600,
        seed: 1234,
      },
    },
  ],
  compute(params: ParamValues): PlotSpec[] {
    const ni = Math.round(Number(params.num_interceptors));
    const nt = Math.round(Number(params.num_threats));
    const doctrine = asDoctrine(params.doctrine);
    const shots_per_threat = Math.round(Number(params.shots_per_threat));
    const pk_max = Number(params.pk_max);
    const sigma_m = Number(params.sigma_m);
    const latency_s = Number(params.latency_s);
    const dropout_prob = Number(params.dropout_prob);
    const num_trials = Math.round(Number(params.num_trials));
    const seed = Math.round(Number(params.seed));

    const campaignFor = (
      matrix: PairingMatrix,
      override: Partial<CampaignParams> = {},
    ) =>
      runCampaign(matrix, {
        ni: matrix.ni,
        nt: matrix.nt,
        doctrine,
        shots_per_threat,
        max_waves: 4,
        num_trials,
        seed,
        ...override,
      });

    // ---- Plot 1: Leakage vs interceptor inventory ----------------------------
    // Sweep inventory 0..(2·nt), re-scoring the matrix for each inventory size
    // (a fresh greedy WTA + doctrine play-out). Uses the deterministic
    // expected-value leakage (sum of per-threat survival probability) so the
    // curve is monotone-decreasing and reaches ~0 once inventory ≥ threats with
    // adequate Pk, independent of the Monte-Carlo trial count.
    const invMax = Math.max(2 * nt, ni);
    const invX: number[] = [];
    const invLeak: number[] = [];
    for (let inv = 0; inv <= invMax; inv++) {
      invX.push(inv);
      if (inv === 0) {
        invLeak.push(nt); // no interceptors → every threat leaks
        continue;
      }
      const m = buildMatrix(inv, nt, sigma_m, pk_max, seed, latency_s, dropout_prob);
      invLeak.push(campaignFor(m, { num_trials: 1 }).expected_leakage);
    }

    // ---- Plot 2: P(raid annihilation) vs shots-per-threat --------------------
    // Salvo doctrine with a generous inventory (shots·nt), swept over
    // shots_per_threat for a few pk_max values. P(annihilation) rises toward 1.
    const matrixForShots = (shots: number, pk: number) =>
      buildMatrix(shots * nt, nt, sigma_m, pk, seed, latency_s, dropout_prob);
    const shotsX = [1, 2, 3, 4, 5, 6];
    const pkLevels = [0.5, 0.7, Math.max(0.5, Math.min(0.99, pk_max))];
    const annihTraces = pkLevels.map((pk, idx) => {
      const y = shotsX.map((shots) => {
        const m = matrixForShots(shots, pk);
        // Deterministic expected-value annihilation probability (product of
        // per-threat kill probabilities) → a clean monotone-in-shots curve.
        return runCampaign(m, {
          ni: m.ni,
          nt: m.nt,
          doctrine: 'salvo',
          shots_per_threat: shots,
          max_waves: 4,
          num_trials: 1,
          seed,
        }).p_raid_annihilation;
      });
      const isCurrent = Math.abs(pk - pk_max) < 1e-9;
      return {
        x: shotsX,
        y,
        type: 'scatter' as const,
        mode: 'lines+markers' as const,
        name: `pk_max=${pk.toFixed(2)}${isCurrent ? ' (current)' : ''}`,
        line: { width: isCurrent ? 3 : 1.5 },
      };
    });

    // ---- Plot 3: WTA assignment matrix heatmap -------------------------------
    // The current-config pairing Pk matrix + the committed assignments overlaid
    // as markers (one weapon per threat from the greedy WTA / doctrine).
    const matrix = buildMatrix(ni, nt, sigma_m, pk_max, seed, latency_s, dropout_prob);
    const campaign = campaignFor(matrix);
    const heat = matrix.pk; // [weapon][threat]
    const assignX = campaign.assignments.map((a) => a.threat_index);
    const assignY = campaign.assignments.map((a) => a.interceptor_index);

    // ---- Plot 4: Latency / dropout sensitivity -------------------------------
    // Campaign mean leakage vs datalink latency, one trace per dropout slice.
    const latX: number[] = [];
    for (let l = 0; l <= 5.0 + 1e-9; l += 0.25) latX.push(Number(l.toFixed(2)));
    const dropoutSlices = [0, 0.2, 0.4];
    const latTraces = dropoutSlices.map((dp) => {
      const y = latX.map((lat) => {
        const m = buildMatrix(ni, nt, sigma_m, pk_max, seed, lat, dp);
        return campaignFor(m, { num_trials: 1 }).expected_leakage;
      });
      return {
        x: latX,
        y,
        type: 'scatter' as const,
        mode: 'lines' as const,
        name: `dropout=${dp.toFixed(2)}`,
      };
    });

    return [
      {
        id: 'leakage-vs-inventory',
        title: 'Leakage vs interceptor inventory',
        data: [
          {
            x: invX,
            y: invLeak,
            type: 'scatter',
            mode: 'lines+markers',
            name: 'mean leakage',
            line: { color: ACCENT, width: 2 },
          },
          {
            x: [nt, nt],
            y: [0, Math.max(nt, 1)],
            type: 'scatter',
            mode: 'lines',
            name: 'inventory = threats',
            line: { color: WARN, width: 1, dash: 'dot' },
          },
        ],
        layout: {
          xaxis: { title: { text: 'interceptor inventory [#]' } },
          yaxis: { title: { text: 'surviving threats (mean leakage) [#]' }, rangemode: 'tozero' },
        },
      },
      {
        id: 'annihilation-vs-shots',
        title: 'P(raid annihilation) vs shots per threat (salvo)',
        data: annihTraces,
        layout: {
          xaxis: { title: { text: 'shots per threat [#]' }, dtick: 1 },
          yaxis: { title: { text: 'P(all threats killed)' }, range: [0, 1] },
        },
      },
      {
        id: 'wta-matrix',
        title: 'WTA assignment matrix — pairing P(kill) (× = committed shot)',
        data: [
          {
            z: heat,
            type: 'heatmap',
            colorscale: 'Viridis',
            colorbar: { title: { text: 'Pk' } },
            zmin: 0,
            zmax: 1,
          },
          {
            x: assignX,
            y: assignY,
            type: 'scatter',
            mode: 'markers',
            name: 'committed',
            marker: { symbol: 'x', size: 10, color: '#ffffff', line: { width: 1 } },
          },
        ],
        layout: {
          xaxis: { title: { text: 'threat index' }, dtick: 1 },
          yaxis: { title: { text: 'interceptor index' }, dtick: 1, autorange: 'reversed' },
        },
      },
      {
        id: 'latency-dropout-sensitivity',
        title: 'BMC2 datalink sensitivity — leakage vs latency / dropout',
        data: latTraces,
        layout: {
          xaxis: { title: { text: 'datalink latency [s]' } },
          yaxis: { title: { text: 'surviving threats (mean leakage) [#]' }, rangemode: 'tozero' },
        },
      },
    ];
  },
};
