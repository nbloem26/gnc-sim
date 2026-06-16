/**
 * Filter / Estimator Studio (issue #108) — engine-backed tuning & consistency.
 *
 * Drives the C++ core (WASM) navigation filter and reads back the EKF NIS
 * consistency channel (`series.nav_nis`) so you can see, interactively, whether
 * the estimator is *consistent* (NIS statistics match the chi-square the filter
 * assumes) or over-/under-confident, and how that trades against tracking error
 * as the target maneuvers.
 *
 * It is the engine-backed sibling of the pure-math exampleStudio: it builds a
 * base config from the `homing_3dof_ekf` preset (sensors on, EKF nav, emits
 * nav_nis), overrides the nav filter + Q/R-ish knobs + target maneuver, then
 * `await runSim(cfg)`.
 *
 * --- What "NIS" is (see core/src/gnc/Ekf.cpp) -------------------------------
 * Each filter update forms the innovation y = z − h(x̂) and its covariance S,
 * and reports NIS = yᵀS⁻¹y. The measurement is (azimuth, elevation, range), so
 * NIS is chi-square distributed with **DOF = 3** for a consistent filter. Its
 * mean should be ≈ 3; a NIS that rides high means the filter is over-confident
 * (S too small → Q or R underestimated), riding low means under-confident.
 *
 * --- How Q and R map onto the config (no exposed "Q"/"R" scalar) ------------
 * The core has no single Q or R knob; it has the physical noise parameters the
 * EKF is built from (see core/src/model/Registry.cpp makeNavigator). So the
 * studio's Q / R sliders are *multiplicative scales* on those:
 *   Q scale → nav.process_accel_psd  (EKF target-accel PSD q, base 50)
 *             and, in IMM mode, nav.imm_q_cv / nav.imm_q_man together.
 *   R scale → sensors.seeker.los_white (az & el measurement σ, base 0.003)
 *             and nav.range_white       (range measurement σ, base 5).
 * Those are exactly the sigmas fed into S, so scaling them scales the filter's
 * assumed measurement noise R (and process noise Q) directly.
 *
 * --- IMM mode probabilities -------------------------------------------------
 * The data contract's SimResult.series does NOT emit an IMM mode-probability
 * channel (no `mode_prob*` key in core/src/core/Serialize.cpp), so the
 * mode-probability-vs-time plot cannot be drawn; the studio renders a note
 * instead. IMM is still fully exercised (filter='imm' runs the real Imm core
 * and emits nav_nis), so consistency + RMS plots work for it.
 */

import type { StudioDef, ParamValues, PlotSpec } from '../types';
import type { SimConfig, SimResult } from '@/lib/types';
import { loadPresetConfig } from '@/lib/presets';
import { runSim, isMockMode, isResolved } from '@/lib/wasmRunner';

// ----------------------------------------------------------------------------
// Constants
// ----------------------------------------------------------------------------

/** EKF measurement DOF = (az, el, range) → NIS ~ chi-square(3). */
const NIS_DOF = 3;

/**
 * Two-sided 95% consistency band for chi-square(3): the 2.5% and 97.5%
 * quantiles. A per-sample NIS for a consistent filter falls inside this band
 * ~95% of the time. (χ²₃ inverse-CDF: 0.025 → 0.2158, 0.975 → 9.3484.)
 */
const CHI2_3_LO = 0.2158;
const CHI2_3_HI = 9.3484;

const ACCENT = '#4fd1c5'; // teal — primary series
const WARN = '#f6ad55'; // orange — band / expected
const VIOLET = '#9f7aea'; // third filter / overlay

/** The EKF/IMM-bearing base preset (sensors on, emits nav_nis). */
const BASE_PRESET_FILE = 'homing_3dof_ekf.json';

// Base config values the Q/R scales multiply (kept in sync with the preset /
// core defaults; see file header for provenance).
const BASE_PROCESS_ACCEL_PSD = 50.0; // nav.process_accel_psd
const BASE_LOS_WHITE = 0.003; // sensors.seeker.los_white
const BASE_RANGE_WHITE = 5.0; // nav.range_white
const BASE_IMM_Q_CV = 0.5; // nav.imm_q_cv
const BASE_IMM_Q_MAN = 3000.0; // nav.imm_q_man

// ----------------------------------------------------------------------------
// Small numeric helpers
// ----------------------------------------------------------------------------

/** Γ(3/2) = √π / 2, used by the chi-square(3) pdf below. */
const GAMMA_3_2 = Math.sqrt(Math.PI) / 2;

/** chi-square pdf with k=3 DOF: f(x)= x^{1/2} e^{-x/2} / (2^{3/2} Γ(3/2)). */
function chi2Pdf3(x: number): number {
  if (x <= 0) return 0;
  return (Math.sqrt(x) * Math.exp(-x / 2)) / (Math.pow(2, 1.5) * GAMMA_3_2);
}

function mean(xs: number[]): number {
  if (xs.length === 0) return NaN;
  let s = 0;
  for (const v of xs) s += v;
  return s / xs.length;
}

function linspace(a: number, b: number, n: number): number[] {
  if (n < 2) return [a];
  const out = new Array<number>(n);
  const d = (b - a) / (n - 1);
  for (let i = 0; i < n; i++) out[i] = a + i * d;
  return out;
}

/** RMS of the fused/estimated target-track position error vs truth [m]. */
function trackRmsError_m(s: SimResult['series']): number {
  // Prefer the nav state estimate (nav_x/y/z) vs the true target. nav_* is the
  // navigator's relative-state→target estimate and is always populated.
  const nx = s.nav_x;
  const ny = s.nav_y;
  const nz = s.nav_z;
  if (!nx || !ny || !nz) return NaN;
  const n = Math.min(s.t.length, nx.length, s.tgt_x.length);
  if (n === 0) return NaN;
  let acc = 0;
  let count = 0;
  // Skip the first 10% (filter warm-up transient) so RMS reflects steady track.
  const start = Math.floor(n * 0.1);
  for (let i = start; i < n; i++) {
    const dx = nx[i] - s.tgt_x[i];
    const dy = ny[i] - s.tgt_y[i];
    const dz = nz[i] - s.tgt_z[i];
    acc += dx * dx + dy * dy + dz * dz;
    count++;
  }
  return count > 0 ? Math.sqrt(acc / count) : NaN;
}

/** Pull a clean, finite, non-trivial NIS series (the EKF emits 0 on skipped updates). */
function nisSeries(r: SimResult): { t_s: number[]; nis: number[] } {
  const s = r.series;
  const raw = s.nav_nis;
  if (!Array.isArray(raw)) return { t_s: [], nis: [] };
  const t_s: number[] = [];
  const nis: number[] = [];
  const n = Math.min(s.t.length, raw.length);
  for (let i = 0; i < n; i++) {
    const v = raw[i];
    // 0.0 = no measurement update that step (warm-up / singular S); drop it so
    // the histogram/consistency stats aren't polluted by structural zeros.
    if (Number.isFinite(v) && v > 0) {
      t_s.push(s.t[i]);
      nis.push(v);
    }
  }
  return { t_s, nis };
}

// ----------------------------------------------------------------------------
// Config builder
// ----------------------------------------------------------------------------

interface FilterParams {
  filter: string;
  qScale: number;
  rScale: number;
  maneuver_g: number;
  maneuver_freq_hz: number;
}

/** Apply the studio params onto a fresh copy of the base preset config. */
function buildConfig(base: SimConfig, p: FilterParams): SimConfig {
  // Deep-ish clone the blocks we mutate (the rest can share references; runSim
  // serializes the whole thing).
  const cfg: SimConfig = {
    ...base,
    sensors: {
      ...base.sensors,
      enable: true,
      seeker: { ...(base.sensors.seeker ?? { los_white: BASE_LOS_WHITE, los_bias: 0, glint: 1 }) },
    },
    nav: { ...(base.nav ?? {}) },
    target: { ...base.target },
  };

  // --- Filter selection -----------------------------------------------------
  (cfg.nav as Record<string, unknown>).filter = p.filter;

  // --- R scale → measurement-noise sigmas (az/el via los_white, range) ------
  if (cfg.sensors.seeker) {
    cfg.sensors.seeker.los_white = BASE_LOS_WHITE * p.rScale;
  }
  (cfg.nav as Record<string, unknown>).range_white = BASE_RANGE_WHITE * p.rScale;

  // --- Q scale → process-noise PSDs (EKF and both IMM modes) ----------------
  (cfg.nav as Record<string, unknown>).process_accel_psd = BASE_PROCESS_ACCEL_PSD * p.qScale;
  (cfg.nav as Record<string, unknown>).imm_q_cv = BASE_IMM_Q_CV * p.qScale;
  (cfg.nav as Record<string, unknown>).imm_q_man = BASE_IMM_Q_MAN * p.qScale;

  // --- Target maneuver ------------------------------------------------------
  // A weaving target is what stresses the filter's consistency; constant-g if
  // freq is ~0, weave otherwise. maneuver_freq is in Hz (contract key).
  cfg.target.maneuver = p.maneuver_freq_hz > 1e-6 && p.maneuver_g > 1e-6 ? 'weave' : 'constant';
  cfg.target.maneuver_g = p.maneuver_g;
  cfg.target.maneuver_freq = p.maneuver_freq_hz;

  return cfg;
}

// ----------------------------------------------------------------------------
// Plot builders
// ----------------------------------------------------------------------------

function mockNote(): PlotSpec {
  return {
    id: 'mock-note',
    title: 'Engine unavailable — mock mode',
    data: [],
    layout: {
      xaxis: { visible: false },
      yaxis: { visible: false },
      annotations: [
        {
          text:
            'WASM core not built — running in MOCK mode.<br>' +
            'Every run returns the committed sample, so live filter sweeps and<br>' +
            'NIS consistency are disabled. Build the WASM artifact ' +
            '(scripts/build-wasm.sh) for real client-side runs.',
          showarrow: false,
          font: { size: 13, color: WARN },
          x: 0.5,
          y: 0.5,
          xref: 'paper',
          yref: 'paper',
        },
      ],
    },
  };
}

/** Plot 1 — NIS vs time with the 95% chi-square(3) consistency band. */
function nisVsTimePlot(t_s: number[], nis: number[], filterLabel: string): PlotSpec {
  const tLo = t_s.length ? t_s[0] : 0;
  const tHi = t_s.length ? t_s[t_s.length - 1] : 1;
  const m = mean(nis);
  return {
    id: 'nis-time',
    title: `NIS vs time — ${filterLabel} (95% χ²(${NIS_DOF}) band)`,
    data: [
      // Shaded band as an upper trace filled down to the lower trace.
      {
        x: [tLo, tHi],
        y: [CHI2_3_LO, CHI2_3_LO],
        type: 'scatter',
        mode: 'lines',
        line: { width: 0 },
        hoverinfo: 'skip',
        showlegend: false,
      },
      {
        x: [tLo, tHi],
        y: [CHI2_3_HI, CHI2_3_HI],
        type: 'scatter',
        mode: 'lines',
        fill: 'tonexty',
        fillcolor: 'rgba(246, 173, 85, 0.12)',
        line: { width: 0 },
        name: `95% band [${CHI2_3_LO.toFixed(2)}, ${CHI2_3_HI.toFixed(2)}]`,
      },
      {
        x: t_s,
        y: nis,
        type: 'scatter',
        mode: 'lines',
        name: 'NIS',
        line: { color: ACCENT, width: 1 },
      },
      {
        x: [tLo, tHi],
        y: [NIS_DOF, NIS_DOF],
        type: 'scatter',
        mode: 'lines',
        name: `expected mean = DOF = ${NIS_DOF}`,
        line: { color: WARN, width: 1.5, dash: 'dash' },
      },
    ],
    layout: {
      xaxis: { title: { text: 'Time [s]' } },
      yaxis: { title: { text: 'NIS [—]' }, rangemode: 'tozero' },
      annotations: [
        {
          text: `sample mean NIS = ${Number.isFinite(m) ? m.toFixed(2) : '—'} (ideal ≈ ${NIS_DOF})`,
          showarrow: false,
          x: 0.02,
          y: 1.06,
          xref: 'paper',
          yref: 'paper',
          xanchor: 'left',
          font: { size: 11, color: '#c8d2dc' },
        },
      ],
    },
  };
}

/** Plot 2 — NIS histogram (density) vs the chi-square(3) pdf. */
function nisHistogramPlot(nis: number[], filterLabel: string): PlotSpec {
  const hiData = nis.length ? Math.max(...nis) : CHI2_3_HI;
  // Cap the x range at a sensible multiple of the DOF so a few huge outliers
  // don't flatten the whole histogram; note them in the title if clipped.
  const xMax = Math.min(Math.max(hiData, 4 * NIS_DOF), 6 * NIS_DOF);
  const xs = linspace(0.01, xMax, 200);
  const pdf = xs.map(chi2Pdf3);
  const m = mean(nis);
  return {
    id: 'nis-hist',
    title: `NIS histogram vs χ²(${NIS_DOF}) pdf — ${filterLabel}`,
    data: [
      {
        x: nis,
        type: 'histogram',
        histnorm: 'probability density',
        name: 'NIS (density)',
        marker: { color: 'rgba(79, 209, 197, 0.55)' },
        xbins: { start: 0, end: xMax, size: xMax / 30 },
      },
      {
        x: xs,
        y: pdf,
        type: 'scatter',
        mode: 'lines',
        name: `χ²(${NIS_DOF}) pdf (mean ${NIS_DOF})`,
        line: { color: WARN, width: 2 },
      },
    ],
    layout: {
      xaxis: { title: { text: 'NIS [—]' }, range: [0, xMax] },
      yaxis: { title: { text: 'density' } },
      bargap: 0.04,
      annotations: [
        {
          text: `sample mean = ${Number.isFinite(m) ? m.toFixed(2) : '—'}  (ideal ${NIS_DOF})`,
          showarrow: false,
          x: 0.98,
          y: 1.06,
          xref: 'paper',
          yref: 'paper',
          xanchor: 'right',
          font: { size: 11, color: '#c8d2dc' },
        },
      ],
    },
  };
}

/** Plot 3 — Miss distance & track RMS vs maneuver-g sweep, per filter. */
function sweepPlot(
  manG: number[],
  perFilter: { filter: string; label: string; miss_m: number[]; rms_m: number[] }[],
): PlotSpec {
  const colors = [ACCENT, WARN, VIOLET];
  const data: PlotSpec['data'] = [];
  perFilter.forEach((f, i) => {
    const c = colors[i % colors.length];
    data.push({
      x: manG,
      y: f.miss_m,
      type: 'scatter',
      mode: 'lines+markers',
      name: `${f.label}: miss [m]`,
      line: { color: c, width: 2 },
      marker: { color: c, size: 6 },
      yaxis: 'y',
    });
    data.push({
      x: manG,
      y: f.rms_m,
      type: 'scatter',
      mode: 'lines+markers',
      name: `${f.label}: track RMS [m]`,
      line: { color: c, width: 1.5, dash: 'dot' },
      marker: { color: c, size: 5, symbol: 'diamond' },
      yaxis: 'y2',
    });
  });
  return {
    id: 'sweep',
    title: 'Miss & track RMS vs target maneuver-g (per filter)',
    data,
    layout: {
      xaxis: { title: { text: 'Target maneuver [g]' } },
      yaxis: { title: { text: 'Miss distance [m]' }, rangemode: 'tozero' },
      yaxis2: {
        title: { text: 'Track RMS [m]' },
        overlaying: 'y',
        side: 'right',
        rangemode: 'tozero',
        gridcolor: 'rgba(0,0,0,0)',
      },
    },
  };
}

/** Plot 4 — note that IMM mode probabilities are not emitted by the contract. */
function modeProbNote(): PlotSpec {
  return {
    id: 'mode-prob-note',
    title: 'IMM mode probabilities',
    data: [],
    layout: {
      xaxis: { visible: false },
      yaxis: { visible: false },
      annotations: [
        {
          text:
            'The data contract (SimResult.series) does not emit an IMM<br>' +
            'mode-probability channel, so mode-probability-vs-time cannot be<br>' +
            'plotted. IMM still runs the real core filter and reports nav_nis,<br>' +
            'so the consistency and RMS plots above are valid for it.',
          showarrow: false,
          font: { size: 12, color: '#8aa0b4' },
          x: 0.5,
          y: 0.5,
          xref: 'paper',
          yref: 'paper',
        },
      ],
    },
  };
}

// ----------------------------------------------------------------------------
// Studio definition
// ----------------------------------------------------------------------------

const FILTER_OPTIONS = [
  { value: 'alpha_beta', label: 'α–β tracker' },
  { value: 'ekf', label: 'EKF' },
  { value: 'imm', label: 'IMM' },
];

/** Maneuver-g grid for the comparison sweep (kept small — each point is a run). */
const SWEEP_MAN_G = [0, 2, 4, 6, 8];
/** Filters compared in the sweep. */
const SWEEP_FILTERS = [
  { filter: 'alpha_beta', label: 'α–β' },
  { filter: 'ekf', label: 'EKF' },
  { filter: 'imm', label: 'IMM' },
];

let basePromise: Promise<SimConfig> | null = null;
function getBase(): Promise<SimConfig> {
  if (!basePromise) basePromise = loadPresetConfig(BASE_PRESET_FILE);
  return basePromise;
}

export const filterStudio: StudioDef = {
  id: 'filter-estimator',
  label: 'Filter / Estimator Studio',
  description:
    'Engine-backed estimator tuning & consistency: drive the EKF/IMM/α–β nav ' +
    'filter on a maneuvering homing engagement, read back the NIS, and check it ' +
    'against the χ²(3) the filter assumes. Q scales the process-noise PSD, R the ' +
    'measurement-noise σ. Over-/under-confidence shows as NIS riding above/below ' +
    'the 95% band; the sweep trades tracking RMS against maneuver hardness.',
  params: [
    {
      kind: 'enum',
      key: 'filter',
      label: 'Nav filter',
      options: FILTER_OPTIONS,
      default_value: 'ekf',
      help: 'α–β does not emit NIS; EKF/IMM do (DOF = 3).',
    },
    {
      kind: 'number',
      key: 'q_scale',
      label: 'Process-noise scale Q',
      min_value: 0.05,
      max_value: 20,
      step: 0.05,
      default_value: 1,
      help: 'Multiplies nav.process_accel_psd (EKF) / imm_q_cv & imm_q_man (IMM).',
    },
    {
      kind: 'number',
      key: 'r_scale',
      label: 'Measurement-noise scale R',
      min_value: 0.1,
      max_value: 10,
      step: 0.1,
      default_value: 1,
      help: 'Multiplies seeker los_white (az/el σ) and nav.range_white (range σ).',
    },
    {
      kind: 'number',
      key: 'maneuver_g',
      label: 'Target maneuver',
      unit: 'g',
      min_value: 0,
      max_value: 9,
      step: 0.5,
      default_value: 4,
      help: 'Weave amplitude. 0 g → non-maneuvering (constant-velocity) target.',
    },
    {
      kind: 'number',
      key: 'maneuver_freq_hz',
      label: 'Maneuver frequency',
      unit: 'Hz',
      min_value: 0,
      max_value: 2,
      step: 0.05,
      default_value: 0.4,
      help: 'Weave frequency. 0 Hz → constant maneuver (no weave).',
    },
  ],
  presets: [
    {
      id: 'tuned-ekf',
      label: 'Tuned EKF',
      values: { filter: 'ekf', q_scale: 1, r_scale: 1, maneuver_g: 4, maneuver_freq_hz: 0.4 },
    },
    {
      id: 'overconfident',
      label: 'Over-confident (low Q)',
      values: { filter: 'ekf', q_scale: 0.1, r_scale: 1, maneuver_g: 6, maneuver_freq_hz: 0.6 },
    },
    {
      id: 'underconfident',
      label: 'Under-confident (high Q)',
      values: { filter: 'ekf', q_scale: 12, r_scale: 1, maneuver_g: 2, maneuver_freq_hz: 0.3 },
    },
    {
      id: 'imm-hard',
      label: 'IMM vs hard weave',
      values: { filter: 'imm', q_scale: 1, r_scale: 1, maneuver_g: 7, maneuver_freq_hz: 0.5 },
    },
  ],

  async compute(values: ParamValues): Promise<PlotSpec[]> {
    const p: FilterParams = {
      filter: String(values.filter ?? 'ekf'),
      qScale: Number(values.q_scale ?? 1),
      rScale: Number(values.r_scale ?? 1),
      maneuver_g: Number(values.maneuver_g ?? 4),
      maneuver_freq_hz: Number(values.maneuver_freq_hz ?? 0.4),
    };

    const base = await getBase();

    // --- Primary run (current params) ---------------------------------------
    const mainResult = await runSim(buildConfig(base, p));

    // Resolve mock state only AFTER the first runSim (so isResolved() is true).
    // In mock mode every run returns the same committed sample, so a live sweep
    // is meaningless — guard like the other engine-backed surfaces.
    if (isResolved() && isMockMode()) {
      return [mockNote()];
    }

    const filterLabel = FILTER_OPTIONS.find((o) => o.value === p.filter)?.label ?? p.filter;
    const { t_s, nis } = nisSeries(mainResult);

    const plots: PlotSpec[] = [];

    if (nis.length === 0) {
      // α–β (no NIS) — still useful for the sweep, but explain the empty band.
      plots.push({
        id: 'no-nis',
        title: `No NIS channel — ${filterLabel}`,
        data: [],
        layout: {
          xaxis: { visible: false },
          yaxis: { visible: false },
          annotations: [
            {
              text:
                `The ${filterLabel} filter does not emit a NIS channel<br>` +
                '(only the EKF/IMM filters report nav_nis). Switch to EKF or IMM<br>' +
                'for the consistency plots; the maneuver sweep below still applies.',
              showarrow: false,
              font: { size: 12, color: '#8aa0b4' },
              x: 0.5,
              y: 0.5,
              xref: 'paper',
              yref: 'paper',
            },
          ],
        },
      });
    } else {
      plots.push(nisVsTimePlot(t_s, nis, filterLabel));
      plots.push(nisHistogramPlot(nis, filterLabel));
    }

    // --- Maneuver sweep across filters --------------------------------------
    // Run every (filter × maneuver-g) at the current Q/R and weave frequency.
    const perFilter: { filter: string; label: string; miss_m: number[]; rms_m: number[] }[] = [];
    for (const f of SWEEP_FILTERS) {
      const miss_m: number[] = [];
      const rms_m: number[] = [];
      for (const g of SWEEP_MAN_G) {
        const cfg = buildConfig(base, {
          filter: f.filter,
          qScale: p.qScale,
          rScale: p.rScale,
          maneuver_g: g,
          maneuver_freq_hz: p.maneuver_freq_hz,
        });
        const r = await runSim(cfg);
        miss_m.push(r.miss_distance);
        rms_m.push(trackRmsError_m(r.series));
      }
      perFilter.push({ filter: f.filter, label: f.label, miss_m, rms_m });
    }
    plots.push(sweepPlot(SWEEP_MAN_G, perFilter));

    // --- IMM mode probabilities: not emitted by the contract ----------------
    if (p.filter === 'imm') {
      plots.push(modeProbNote());
    }

    return plots;
  },
};
