/**
 * Radar / sensor phenomenology Studio (issue #105).
 *
 * A pure client-side exploration of the monostatic radar detection chain:
 * range-equation SNR (1/R^4) with Swerling RCS fluctuation, clutter + barrage
 * jamming, and a Cell-Averaging CFAR detector (Gandhi & Kassam threshold, with
 * the closed-form Pd/Pfa). It re-implements — rather than calls — the validated
 * C++ model in `core/src/sensors/Phenomenology.{hpp,cpp}`; the math lives in
 * `radarMath.ts`, unit-commented against that core.
 *
 * Plots:
 *   1. SNR vs range (range equation) with the CFAR detection threshold.
 *   2. ROC — Pd vs Pfa swept, for the current operating SNR + Swerling case.
 *   3. Detection-range envelope — range at which Pd >= 0.9 vs target RCS.
 *   4. CA-CFAR operating point — Pd & Pfa vs SNR at the design Pfa, marked.
 *
 * Naming follows the repo convention (units-in-names): range_m, rcs_m2, snr_db,
 * freq_hz, pfa (dimensionless probability), etc.
 */

import type { StudioDef, ParamValues, PlotSpec } from '../types';
import type { SwerlingCase } from './radarMath';
import {
  radarSnrLinear,
  cfarAlpha,
  cfarPdSwerling,
  linearToDb,
  linspace,
  logspace,
  detectionRangeForPd,
  swerlingLabel,
} from './radarMath';

// Speed of light, for the (informational) wavelength readout from frequency.
const C_M_PER_S = 299_792_458;

const COLORS = {
  signal: '#4fd1c5',
  threshold: '#f6ad55',
  pd: '#63b3ed',
  pfa: '#fc8181',
  envelope: '#9f7aea',
  marker: '#f6e05e',
};

/**
 * Pull the common physical params out of the live values object once. The
 * design Pfa is exposed to the user as its base-10 exponent (a log slider), so
 * pfa = 10^pfa_log10.
 */
function readParams(values: ParamValues) {
  const snr_ref_db = Number(values.snr_ref_db);
  const range_ref_m = Number(values.range_ref_m);
  const rcs_ref_m2 = Number(values.rcs_ref_m2);
  const freq_hz = Number(values.freq_hz);
  const rcs_m2 = Number(values.rcs_m2);
  const range_m = Number(values.target_range_m);
  const num_ref_cells = Math.round(Number(values.cfar_num_ref_cells));
  const pfa = Math.pow(10, Number(values.pfa_log10));
  const jammer_jnr_db = Number(values.jammer_jnr_db);
  const swerling = ((): SwerlingCase => {
    const n = Number(values.swerling_case);
    return n === 0 || n === 2 || n === 3 || n === 4 ? n : 1;
  })();
  // Jammer slider at its floor disables the jammer (matches the C++ sentinel).
  const jammerActive = jammer_jnr_db > -89.0;
  return {
    snr_ref_db,
    range_ref_m,
    rcs_ref_m2,
    freq_hz,
    rcs_m2,
    range_m,
    num_ref_cells,
    pfa,
    jammer_jnr_db: jammerActive ? jammer_jnr_db : -120,
    swerling,
  };
}

/** Detection-threshold SNR [linear]: the SNR at which Pd reaches 0.5. */
function thresholdSnrLinearForPd(
  swerling: SwerlingCase,
  num_ref_cells: number,
  pfa: number,
  pdTarget: number,
): number {
  // Pd is monotonically increasing in SNR; bisect on linear SNR.
  let lo = 0.0;
  let hi = 1e6;
  for (let i = 0; i < 80; i++) {
    const mid = 0.5 * (lo + hi);
    const pd = cfarPdSwerling(swerling, num_ref_cells, pfa, mid);
    if (pd >= pdTarget) hi = mid;
    else lo = mid;
  }
  return 0.5 * (lo + hi);
}

export const radarStudio: StudioDef = {
  id: 'radar-phenomenology',
  label: 'Radar / sensor phenomenology',
  description:
    'Monostatic radar detection chain: range-equation SNR (1/R⁴) with Swerling RCS ' +
    'fluctuation, clutter + barrage jamming, and a CA-CFAR detector (Gandhi–Kassam ' +
    'threshold, closed-form Pd/Pfa). Pure client-side; mirrors core/src/sensors/Phenomenology.',
  params: [
    {
      kind: 'number',
      key: 'snr_ref_db',
      label: 'Reference SNR (anchor)',
      unit: 'dB',
      min_value: -10,
      max_value: 60,
      step: 1,
      default_value: 30,
      help: 'Single-pulse SNR of a reference-RCS target at the reference range. Anchors the range equation.',
    },
    {
      kind: 'number',
      key: 'range_ref_m',
      label: 'Reference range (anchor)',
      unit: 'm',
      min_value: 1_000,
      max_value: 200_000,
      step: 1_000,
      default_value: 50_000,
    },
    {
      kind: 'number',
      key: 'rcs_ref_m2',
      label: 'Reference RCS (anchor)',
      unit: 'm²',
      min_value: 0.01,
      max_value: 100,
      step: 0.01,
      default_value: 1,
    },
    {
      kind: 'number',
      key: 'freq_hz',
      label: 'Carrier frequency',
      unit: 'Hz',
      min_value: 1e9,
      max_value: 40e9,
      step: 1e8,
      default_value: 1e10,
      help: 'Informational (wavelength readout). The anchored range equation absorbs λ into the SNR anchor.',
    },
    {
      kind: 'number',
      key: 'rcs_m2',
      label: 'Target RCS (mean)',
      unit: 'm²',
      min_value: 0.001,
      max_value: 50,
      step: 0.001,
      default_value: 1,
    },
    {
      kind: 'enum',
      key: 'swerling_case',
      label: 'Swerling case',
      options: [
        { value: '0', label: '0 / V — non-fluctuating' },
        { value: '1', label: 'I — scan-to-scan (χ² 2-dof)' },
        { value: '2', label: 'II — pulse-to-pulse (χ² 2-dof)' },
        { value: '3', label: 'III — scan-to-scan (χ² 4-dof)' },
        { value: '4', label: 'IV — pulse-to-pulse (χ² 4-dof)' },
      ],
      default_value: '1',
      help: 'RCS fluctuation model. I/II are exact; 0 is a conservative I/II bound (see notes).',
    },
    {
      kind: 'number',
      key: 'target_range_m',
      label: 'Target range',
      unit: 'm',
      min_value: 1_000,
      max_value: 300_000,
      step: 1_000,
      default_value: 80_000,
    },
    {
      kind: 'number',
      key: 'cfar_num_ref_cells',
      label: 'CFAR reference cells N',
      min_value: 4,
      max_value: 128,
      step: 1,
      default_value: 24,
      help: 'Cell-averaging window size. Larger N → tighter threshold (less CFAR loss).',
    },
    {
      kind: 'number',
      key: 'pfa_log10',
      label: 'Design Pfa (log₁₀)',
      min_value: -12,
      max_value: -2,
      step: 0.1,
      default_value: -6,
      help: 'Design false-alarm probability as its base-10 exponent: Pfa = 10^x.',
    },
    {
      kind: 'number',
      key: 'jammer_jnr_db',
      label: 'Barrage jammer J/N',
      unit: 'dB',
      min_value: -90,
      max_value: 40,
      step: 1,
      default_value: -90,
      help: 'Jammer-to-noise ratio raising the noise floor. At −90 dB the jammer is off.',
    },
  ],
  presets: [
    {
      id: 'x-band-short',
      label: 'X-band short range',
      values: {
        snr_ref_db: 35,
        range_ref_m: 20_000,
        rcs_ref_m2: 1,
        freq_hz: 1e10,
        rcs_m2: 1,
        swerling_case: '1',
        target_range_m: 20_000,
        cfar_num_ref_cells: 16,
        pfa_log10: -6,
        jammer_jnr_db: -90,
      },
    },
    {
      id: 'long-range-low-pfa',
      label: 'Long range / low Pfa',
      values: {
        snr_ref_db: 40,
        range_ref_m: 100_000,
        rcs_ref_m2: 1,
        freq_hz: 3e9,
        rcs_m2: 2,
        swerling_case: '3',
        target_range_m: 150_000,
        cfar_num_ref_cells: 64,
        pfa_log10: -10,
        jammer_jnr_db: -90,
      },
    },
    {
      id: 'jammed',
      label: 'Stand-off jammed',
      values: {
        snr_ref_db: 35,
        range_ref_m: 50_000,
        rcs_ref_m2: 1,
        freq_hz: 1e10,
        rcs_m2: 1,
        swerling_case: '1',
        target_range_m: 60_000,
        cfar_num_ref_cells: 24,
        pfa_log10: -6,
        jammer_jnr_db: 15,
      },
    },
  ],
  compute(values: ParamValues): PlotSpec[] {
    const p = readParams(values);
    const swLabel = swerlingLabel(p.swerling);
    const wavelength_m = C_M_PER_S / Math.max(p.freq_hz, 1);

    // --- Operating point ----------------------------------------------------
    const opSnrLinear = radarSnrLinear({
      snr_ref_db: p.snr_ref_db,
      range_ref_m: p.range_ref_m,
      rcs_ref_m2: p.rcs_ref_m2,
      range_m: p.range_m,
      rcs_m2: p.rcs_m2,
      jammer_jnr_db: p.jammer_jnr_db,
    });
    const opSnrDb = linearToDb(opSnrLinear);
    const opPd = cfarPdSwerling(p.swerling, p.num_ref_cells, p.pfa, opSnrLinear);

    // Threshold SNR (Pd = 0.5) for the CFAR config — drawn on the SNR-vs-range plot.
    const threshSnrLinear = thresholdSnrLinearForPd(
      p.swerling,
      p.num_ref_cells,
      p.pfa,
      0.5,
    );
    const threshSnrDb = linearToDb(threshSnrLinear);

    // === Plot 1: SNR vs range ===============================================
    const ranges_m = linspace(
      Math.max(1_000, p.range_ref_m * 0.05),
      Math.max(p.range_ref_m * 2.5, p.range_m * 1.3),
      400,
    );
    const snr_db_curve = ranges_m.map((range_m) =>
      linearToDb(
        radarSnrLinear({
          snr_ref_db: p.snr_ref_db,
          range_ref_m: p.range_ref_m,
          rcs_ref_m2: p.rcs_ref_m2,
          range_m,
          rcs_m2: p.rcs_m2,
          jammer_jnr_db: p.jammer_jnr_db,
        }),
      ),
    );
    const snrVsRange: PlotSpec = {
      id: 'snr-vs-range',
      title: 'SNR vs range — range equation (1/R⁴) + CFAR threshold',
      data: [
        {
          x: ranges_m,
          y: snr_db_curve,
          type: 'scatter',
          mode: 'lines',
          name: 'SNR(R)',
          line: { color: COLORS.signal, width: 2 },
        },
        {
          x: [ranges_m[0], ranges_m[ranges_m.length - 1]],
          y: [threshSnrDb, threshSnrDb],
          type: 'scatter',
          mode: 'lines',
          name: `detection threshold (Pd=0.5, ${threshSnrDb.toFixed(1)} dB)`,
          line: { color: COLORS.threshold, width: 1.5, dash: 'dash' },
        },
        {
          x: [p.range_m],
          y: [opSnrDb],
          type: 'scatter',
          mode: 'markers',
          name: `operating point (${opSnrDb.toFixed(1)} dB)`,
          marker: { color: COLORS.marker, size: 10, symbol: 'diamond' },
        },
      ],
      layout: {
        xaxis: { title: { text: 'range [m]' } },
        yaxis: { title: { text: 'SNR [dB]' } },
      },
    };

    // === Plot 2: ROC — Pd vs Pfa at the operating SNR =======================
    const pfa_grid = logspace(-12, -1, 220);
    const roc_pd = pfa_grid.map((pfa) =>
      cfarPdSwerling(p.swerling, p.num_ref_cells, pfa, opSnrLinear),
    );
    const roc: PlotSpec = {
      id: 'roc',
      title: `ROC — Pd vs Pfa @ ${opSnrDb.toFixed(1)} dB SNR · ${swLabel}`,
      data: [
        {
          x: pfa_grid,
          y: roc_pd,
          type: 'scatter',
          mode: 'lines',
          name: 'Pd(Pfa)',
          line: { color: COLORS.pd, width: 2 },
        },
        {
          x: [p.pfa],
          y: [opPd],
          type: 'scatter',
          mode: 'markers',
          name: `design Pfa=${p.pfa.toExponential(1)} → Pd=${opPd.toFixed(3)}`,
          marker: { color: COLORS.marker, size: 10, symbol: 'diamond' },
        },
      ],
      layout: {
        xaxis: { title: { text: 'Pfa' }, type: 'log', autorange: 'reversed' },
        yaxis: { title: { text: 'Pd' }, range: [0, 1.02] },
      },
    };

    // === Plot 3: Detection-range envelope (Pd >= 0.9) vs RCS ================
    const rcs_grid_m2 = logspace(-3, Math.log10(50), 120);
    const env_range_m = rcs_grid_m2.map((rcs_m2) =>
      detectionRangeForPd({
        swerling: p.swerling,
        snr_ref_db: p.snr_ref_db,
        range_ref_m: p.range_ref_m,
        rcs_ref_m2: p.rcs_ref_m2,
        rcs_m2,
        num_ref_cells: p.num_ref_cells,
        pfa: p.pfa,
        pdTarget: 0.9,
        range_min_m: 500,
        range_max_m: 500_000,
      }),
    );
    const env: PlotSpec = {
      id: 'detection-envelope',
      title: 'Detection-range envelope — range at Pd ≥ 0.9 vs target RCS',
      data: [
        {
          x: rcs_grid_m2,
          y: env_range_m,
          type: 'scatter',
          mode: 'lines',
          name: 'R(Pd≥0.9)',
          line: { color: COLORS.envelope, width: 2 },
        },
        {
          x: [p.rcs_m2],
          y: [
            detectionRangeForPd({
              swerling: p.swerling,
              snr_ref_db: p.snr_ref_db,
              range_ref_m: p.range_ref_m,
              rcs_ref_m2: p.rcs_ref_m2,
              rcs_m2: p.rcs_m2,
              num_ref_cells: p.num_ref_cells,
              pfa: p.pfa,
              pdTarget: 0.9,
              range_min_m: 500,
              range_max_m: 500_000,
            }),
          ],
          type: 'scatter',
          mode: 'markers',
          name: `current RCS = ${p.rcs_m2} m²`,
          marker: { color: COLORS.marker, size: 10, symbol: 'diamond' },
        },
      ],
      layout: {
        xaxis: { title: { text: 'target RCS [m²]' }, type: 'log' },
        yaxis: { title: { text: 'detection range [m]' } },
      },
    };

    // === Plot 4: CA-CFAR operating point — Pd & Pfa vs SNR ==================
    const snr_db_grid = linspace(-10, 30, 240);
    const pd_vs_snr = snr_db_grid.map((snr_db) =>
      cfarPdSwerling(
        p.swerling,
        p.num_ref_cells,
        p.pfa,
        Math.pow(10, 0.1 * snr_db),
      ),
    );
    // Pfa is held constant by the CFAR threshold (Gandhi–Kassam): a flat line.
    const cfarPfaHeld = Math.pow(
      1 + cfarAlpha(p.num_ref_cells, p.pfa) / p.num_ref_cells,
      -p.num_ref_cells,
    );
    const opPoint: PlotSpec = {
      id: 'cfar-operating-point',
      title: 'CA-CFAR operating point — Pd & Pfa vs SNR @ design Pfa',
      data: [
        {
          x: snr_db_grid,
          y: pd_vs_snr,
          type: 'scatter',
          mode: 'lines',
          name: 'Pd(SNR)',
          line: { color: COLORS.pd, width: 2 },
        },
        {
          x: [snr_db_grid[0], snr_db_grid[snr_db_grid.length - 1]],
          y: [cfarPfaHeld, cfarPfaHeld],
          type: 'scatter',
          mode: 'lines',
          name: `Pfa held = ${cfarPfaHeld.toExponential(1)}`,
          line: { color: COLORS.pfa, width: 1.5, dash: 'dot' },
        },
        {
          x: [opSnrDb],
          y: [opPd],
          type: 'scatter',
          mode: 'markers',
          name: `operating point (${opSnrDb.toFixed(1)} dB, Pd=${opPd.toFixed(3)})`,
          marker: { color: COLORS.marker, size: 10, symbol: 'diamond' },
        },
      ],
      layout: {
        xaxis: { title: { text: 'SNR [dB]' } },
        yaxis: { title: { text: 'probability' }, range: [0, 1.02] },
        annotations: [
          {
            x: 0.02,
            y: 0.98,
            xref: 'paper',
            yref: 'paper',
            align: 'left',
            showarrow: false,
            text:
              `λ = ${(wavelength_m * 100).toFixed(2)} cm · N = ${p.num_ref_cells}` +
              ` · α = ${cfarAlpha(p.num_ref_cells, p.pfa).toFixed(2)}`,
            font: { size: 11, color: '#a0aec0' },
          },
        ],
      },
    };

    return [snrVsRange, roc, env, opPoint];
  },
};
