/**
 * Countermeasures / ECCM Studio (issue #119).
 *
 * A pure client-side exploration of *adversarial* sensing — how electronic
 * countermeasures (ECM) degrade the victim radar's CA-CFAR detection chain and
 * the IR seeker's discrimination, and where the radar counters (ECCM) by
 * "burning through". It **reuses** the validated radar phenomenology in
 * `radarMath.ts` (range-equation SNR ∝ σ/R⁴, Gandhi–Kassam CFAR threshold,
 * Swerling Pd) and adds the ECM/CM models in `eccmMath.ts` (noise/barrage
 * jamming + burn-through, DRFM false targets, chaff masking + CFAR false alarms,
 * IR flare/decoy discrimination margin). No core changes; no WASM call.
 *
 * Plots:
 *   1. Pd vs range — clean vs jammed, marking the burn-through range.
 *   2. False-alarm / false-target count vs jammer power (DRFM) and chaff bloom.
 *   3. IR discrimination margin vs decoy density (flares erode it).
 *   4. J/S vs range with the burn-through crossover (0 dB).
 *
 * Naming follows the repo convention (units-in-names): range_m, rcs_m2, *_db,
 * pfa (dimensionless probability), etc.
 */

import type { StudioDef, ParamValues, PlotSpec } from '../types';
import {
  radarSnrLinear,
  cfarPdSwerling,
  cfarAlpha,
  linearToDb,
  jammerJnrLinear,
  signalToJammerLinear,
  burnThroughRangeM,
  drfmFalseTargetCount,
  chaffCnrDb,
  chaffOccupiedCells,
  chaffFalseAlarmCount,
  irDiscriminationMargin,
  type JammerType,
} from './eccmMath';
import { linspace } from './radarMath';

const COLORS = {
  clean: '#4fd1c5',
  jammed: '#fc8181',
  burnthrough: '#f6e05e',
  drfm: '#63b3ed',
  chaff: '#9f7aea',
  margin: '#68d391',
  zero: '#a0aec0',
  marker: '#f6e05e',
};

/** Detection-threshold SNR [linear]: SNR at which Swerling-I CFAR Pd reaches a target. */
function thresholdSnrLinear(
  num_ref_cells: number,
  pfa: number,
  pdTarget: number,
): number {
  let lo = 0.0;
  let hi = 1e6;
  for (let i = 0; i < 80; i++) {
    const mid = 0.5 * (lo + hi);
    const pd = cfarPdSwerling(1, num_ref_cells, pfa, mid);
    if (pd >= pdTarget) hi = mid;
    else lo = mid;
  }
  return 0.5 * (lo + hi);
}

function readParams(values: ParamValues) {
  const jammer_type = String(values.jammer_type) as JammerType;
  const js_db = Number(values.jammer_js_db);
  return {
    jammer_type,
    js_db,
    snr_ref_db: Number(values.snr_ref_db),
    range_ref_m: Number(values.range_ref_m),
    rcs_ref_m2: Number(values.rcs_ref_m2),
    rcs_m2: Number(values.target_rcs_m2),
    range_m: Number(values.target_range_m),
    num_ref_cells: Math.round(Number(values.cfar_num_ref_cells)),
    pfa: Math.pow(10, Number(values.pfa_log10)),
    chaff_rcs_m2: Number(values.chaff_rcs_m2),
    chaff_bloom: Number(values.chaff_bloom),
    flare_count: Math.round(Number(values.flare_count)),
    flare_intensity_frac: Number(values.flare_intensity_frac),
    decoy_density: Number(values.decoy_density),
  };
}

export const eccmStudio: StudioDef = {
  id: 'countermeasures-eccm',
  label: 'Countermeasures / ECCM',
  description:
    'Adversarial sensing: noise/barrage jamming (burn-through), DRFM false targets, chaff masking ' +
    '+ CFAR false alarms, and IR flare/decoy discrimination margin. Reuses the radar CFAR/SNR/Swerling ' +
    'phenomenology (radarMath) and the feature-based seeker discriminator concept. Pure client-side.',
  params: [
    {
      kind: 'enum',
      key: 'jammer_type',
      label: 'Jammer type',
      options: [
        { value: 'off', label: 'Off' },
        { value: 'noise', label: 'Noise / barrage (stand-off)' },
        { value: 'drfm', label: 'DRFM / repeater (false targets)' },
      ],
      default_value: 'noise',
      help: 'Noise raises the receiver floor (J/S, burn-through). DRFM injects coherent false targets.',
    },
    {
      kind: 'number',
      key: 'jammer_js_db',
      label: 'Jammer power J/S (at anchor)',
      unit: 'dB',
      min_value: -20,
      max_value: 60,
      step: 1,
      default_value: 25,
      help: 'Jammer-to-signal ratio at the reference range. Anchors J/N and the burn-through range.',
    },
    {
      kind: 'number',
      key: 'snr_ref_db',
      label: 'Reference SNR (anchor)',
      unit: 'dB',
      min_value: -10,
      max_value: 60,
      step: 1,
      default_value: 35,
      help: 'Clean single-pulse SNR of a reference-RCS target at the reference range.',
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
      key: 'target_rcs_m2',
      label: 'Target RCS (mean)',
      unit: 'm²',
      min_value: 0.001,
      max_value: 50,
      step: 0.001,
      default_value: 1,
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
      help: 'CA-CFAR window size. Larger N → tighter threshold (less CFAR loss).',
    },
    {
      kind: 'number',
      key: 'pfa_log10',
      label: 'CFAR design Pfa (log₁₀)',
      min_value: -12,
      max_value: -2,
      step: 0.1,
      default_value: -6,
      help: 'Design false-alarm probability as its base-10 exponent: Pfa = 10^x.',
    },
    {
      kind: 'number',
      key: 'chaff_rcs_m2',
      label: 'Chaff cloud RCS',
      unit: 'm²',
      min_value: 0,
      max_value: 5_000,
      step: 10,
      default_value: 500,
      help: 'Total blooming resonant-dipole RCS of the chaff cloud. 0 disables chaff.',
    },
    {
      kind: 'number',
      key: 'chaff_bloom',
      label: 'Chaff bloom (cells)',
      min_value: 1,
      max_value: 200,
      step: 1,
      default_value: 30,
      help: 'How many range/Doppler cells the cloud spreads over as it blooms (dilutes per-cell return, masks more cells).',
    },
    {
      kind: 'number',
      key: 'flare_count',
      label: 'IR flare count',
      min_value: 0,
      max_value: 32,
      step: 1,
      default_value: 4,
      help: 'Number of dispensed IR flares competing with the warhead in the seeker FOV.',
    },
    {
      kind: 'number',
      key: 'flare_intensity_frac',
      label: 'Flare relative intensity',
      min_value: 0,
      max_value: 1,
      step: 0.01,
      default_value: 0.6,
      help: 'How closely a flare mimics the warhead IR intensity feature (1 = identical → no separation).',
    },
    {
      kind: 'number',
      key: 'decoy_density',
      label: 'Decoy / CSO density',
      min_value: 1,
      max_value: 64,
      step: 1,
      default_value: 6,
      help: 'Number of decoys / closely-spaced objects. Denser fields erode the discrimination margin (order statistics).',
    },
  ],
  presets: [
    {
      id: 'noise-standoff',
      label: 'Noise jammer stand-off',
      values: {
        jammer_type: 'noise',
        jammer_js_db: 30,
        snr_ref_db: 35,
        range_ref_m: 50_000,
        rcs_ref_m2: 1,
        target_rcs_m2: 1,
        target_range_m: 90_000,
        cfar_num_ref_cells: 24,
        pfa_log10: -6,
        chaff_rcs_m2: 0,
        chaff_bloom: 1,
        flare_count: 0,
        flare_intensity_frac: 0.6,
        decoy_density: 1,
      },
    },
    {
      id: 'chaff-flare-selfprotect',
      label: 'Chaff + flare self-protection',
      values: {
        jammer_type: 'off',
        jammer_js_db: 0,
        snr_ref_db: 35,
        range_ref_m: 30_000,
        rcs_ref_m2: 1,
        target_rcs_m2: 2,
        target_range_m: 20_000,
        cfar_num_ref_cells: 24,
        pfa_log10: -6,
        chaff_rcs_m2: 1_500,
        chaff_bloom: 60,
        flare_count: 8,
        flare_intensity_frac: 0.8,
        decoy_density: 12,
      },
    },
  ],
  compute(values: ParamValues): PlotSpec[] {
    const p = readParams(values);
    const jammerOn = p.jammer_type !== 'off';
    const noiseOn = p.jammer_type === 'noise';
    const drfmOn = p.jammer_type === 'drfm';

    // Threshold SNR (Pd = 0.5) for the CFAR config — the detection knee.
    const threshSnrLinear = thresholdSnrLinear(p.num_ref_cells, p.pfa, 0.5);
    const threshSnrDb = linearToDb(threshSnrLinear);

    // Burn-through range (S/J = 0 dB). Only meaningful for a noise jammer.
    const burnThrough_m = noiseOn
      ? burnThroughRangeM({
          snr_ref_db: p.snr_ref_db,
          js_ref_db: p.js_db,
          range_ref_m: p.range_ref_m,
          rcs_ref_m2: p.rcs_ref_m2,
          rcs_m2: p.rcs_m2,
          range_min_m: 500,
          range_max_m: 400_000,
        })
      : 0;

    const ranges_m = linspace(
      Math.max(1_000, p.range_ref_m * 0.04),
      Math.max(p.range_ref_m * 3, p.range_m * 1.3),
      400,
    );

    // === Plot 1: Pd vs range — clean vs jammed ==============================
    const pdClean = ranges_m.map((range_m) => {
      const snr = radarSnrLinear({
        snr_ref_db: p.snr_ref_db,
        range_ref_m: p.range_ref_m,
        rcs_ref_m2: p.rcs_ref_m2,
        range_m,
        rcs_m2: p.rcs_m2,
      });
      return cfarPdSwerling(1, p.num_ref_cells, p.pfa, snr);
    });
    const pdJammed = ranges_m.map((range_m) => {
      // Noise jamming folds a range-dependent J/N into the SNR (raises floor).
      const jnr_db = noiseOn
        ? linearToDb(
            jammerJnrLinear({
              js_ref_db: p.js_db,
              snr_ref_db: p.snr_ref_db,
              range_ref_m: p.range_ref_m,
              range_m,
            }),
          )
        : -120;
      const snr = radarSnrLinear({
        snr_ref_db: p.snr_ref_db,
        range_ref_m: p.range_ref_m,
        rcs_ref_m2: p.rcs_ref_m2,
        range_m,
        rcs_m2: p.rcs_m2,
        jammer_jnr_db: jnr_db,
      });
      return cfarPdSwerling(1, p.num_ref_cells, p.pfa, snr);
    });

    const pdData: PlotSpec['data'] = [
      {
        x: ranges_m,
        y: pdClean,
        type: 'scatter',
        mode: 'lines',
        name: 'Pd clean',
        line: { color: COLORS.clean, width: 2 },
      },
    ];
    if (noiseOn) {
      pdData.push({
        x: ranges_m,
        y: pdJammed,
        type: 'scatter',
        mode: 'lines',
        name: `Pd jammed (J/S=${p.js_db} dB @ anchor)`,
        line: { color: COLORS.jammed, width: 2 },
      });
      if (burnThrough_m > 0) {
        pdData.push({
          x: [burnThrough_m, burnThrough_m],
          y: [0, 1.02],
          type: 'scatter',
          mode: 'lines',
          name: `burn-through R = ${(burnThrough_m / 1000).toFixed(1)} km`,
          line: { color: COLORS.burnthrough, width: 1.5, dash: 'dash' },
        });
      }
    }
    const pdVsRange: PlotSpec = {
      id: 'pd-vs-range',
      title: 'Pd vs range — clean vs jammed (burn-through marked)',
      data: pdData,
      layout: {
        xaxis: { title: { text: 'range [m]' } },
        yaxis: { title: { text: 'Pd' }, range: [0, 1.02] },
      },
    };

    // === Plot 2: false-alarm / false-target count ===========================
    // DRFM false targets vs repeater J/S; chaff false alarms vs chaff bloom.
    const js_grid_db = linspace(0, 50, 120);
    const drfmCount = js_grid_db.map((js_db) =>
      drfmFalseTargetCount({
        repeater_js_db: js_db,
        thresh_snr_db: threshSnrDb,
        cells_per_db: 1.5,
        max_cells: 64,
      }),
    );

    const bloom_grid = linspace(1, 200, 120);
    const chaffFa = bloom_grid.map((bloom) => {
      if (p.chaff_rcs_m2 <= 0) return 0;
      const cnr_db = chaffCnrDb({
        snr_ref_db: p.snr_ref_db,
        range_ref_m: p.range_ref_m,
        rcs_ref_m2: p.rcs_ref_m2,
        range_m: p.range_m,
        chaff_rcs_m2: p.chaff_rcs_m2,
        bloom,
      });
      return chaffFalseAlarmCount({
        num_ref_cells: p.num_ref_cells,
        pfa: p.pfa,
        chaff_cnr_db: cnr_db,
        occupied_cells: chaffOccupiedCells(bloom),
      });
    });

    // Current operating markers.
    const drfmNow = drfmOn
      ? drfmFalseTargetCount({
          repeater_js_db: p.js_db,
          thresh_snr_db: threshSnrDb,
          cells_per_db: 1.5,
          max_cells: 64,
        })
      : 0;
    const chaffCnrNow =
      p.chaff_rcs_m2 > 0
        ? chaffCnrDb({
            snr_ref_db: p.snr_ref_db,
            range_ref_m: p.range_ref_m,
            rcs_ref_m2: p.rcs_ref_m2,
            range_m: p.range_m,
            chaff_rcs_m2: p.chaff_rcs_m2,
            bloom: p.chaff_bloom,
          })
        : -120;
    const chaffFaNow =
      p.chaff_rcs_m2 > 0
        ? chaffFalseAlarmCount({
            num_ref_cells: p.num_ref_cells,
            pfa: p.pfa,
            chaff_cnr_db: chaffCnrNow,
            occupied_cells: chaffOccupiedCells(p.chaff_bloom),
          })
        : 0;

    const falseCount: PlotSpec = {
      id: 'false-count',
      title: 'False-alarm / false-target count — DRFM (J/S) & chaff (bloom)',
      data: [
        {
          x: js_grid_db,
          y: drfmCount,
          type: 'scatter',
          mode: 'lines',
          name: 'DRFM false targets vs J/S',
          line: { color: COLORS.drfm, width: 2 },
          xaxis: 'x',
          yaxis: 'y',
        },
        {
          x: bloom_grid,
          y: chaffFa,
          type: 'scatter',
          mode: 'lines',
          name: 'chaff CFAR false alarms vs bloom',
          line: { color: COLORS.chaff, width: 2 },
          xaxis: 'x2',
          yaxis: 'y',
        },
        {
          x: [p.js_db],
          y: [drfmNow],
          type: 'scatter',
          mode: 'markers',
          name: `DRFM now = ${drfmNow}${drfmOn ? '' : ' (off)'}`,
          marker: { color: COLORS.marker, size: 10, symbol: 'diamond' },
          xaxis: 'x',
          yaxis: 'y',
        },
        {
          x: [p.chaff_bloom],
          y: [chaffFaNow],
          type: 'scatter',
          mode: 'markers',
          name: `chaff FA now = ${chaffFaNow.toFixed(2)}`,
          marker: { color: COLORS.marker, size: 10, symbol: 'circle' },
          xaxis: 'x2',
          yaxis: 'y',
        },
      ],
      layout: {
        xaxis: {
          title: { text: 'DRFM repeater J/S [dB]' },
          domain: [0, 0.46],
        },
        xaxis2: {
          title: { text: 'chaff bloom [cells]' },
          domain: [0.54, 1],
        },
        yaxis: { title: { text: 'expected count' } },
      },
    };

    // === Plot 3: IR discrimination margin vs decoy density ==================
    // Flares erode the margin both by mimicking intensity and by raising density.
    const density_grid = linspace(1, 64, 128);
    const baseSep = 3.0; // nominal feature-space separation of a lone decoy
    const featureSpread = 0.4;
    const measNoise = 0.25;
    const densityShrinkK = 0.06;

    // Effective intensity-mimic fraction grows with flare count (more flares ~
    // brighter aggregate clutter near the target intensity feature), capped at 1.
    const flareMimic = Math.min(
      1.0,
      p.flare_intensity_frac * (1 + 0.05 * p.flare_count),
    );

    const marginVsDensity = density_grid.map((density) =>
      irDiscriminationMargin({
        base_separation: baseSep,
        feature_spread: featureSpread,
        measurement_noise: measNoise,
        flare_intensity_frac: flareMimic,
        decoy_density: density,
        density_shrink_k: densityShrinkK,
      }),
    );
    // A "no-flare" reference curve (flares off) to show the erosion.
    const marginNoFlare = density_grid.map((density) =>
      irDiscriminationMargin({
        base_separation: baseSep,
        feature_spread: featureSpread,
        measurement_noise: measNoise,
        flare_intensity_frac: 0,
        decoy_density: density,
        density_shrink_k: densityShrinkK,
      }),
    );
    const marginNow = irDiscriminationMargin({
      base_separation: baseSep,
      feature_spread: featureSpread,
      measurement_noise: measNoise,
      flare_intensity_frac: flareMimic,
      decoy_density: p.decoy_density,
      density_shrink_k: densityShrinkK,
    });

    const irMargin: PlotSpec = {
      id: 'ir-margin',
      title: 'IR discrimination margin vs decoy density (flares erode it)',
      data: [
        {
          x: density_grid,
          y: marginNoFlare,
          type: 'scatter',
          mode: 'lines',
          name: 'margin, no flares',
          line: { color: COLORS.clean, width: 1.5, dash: 'dot' },
        },
        {
          x: density_grid,
          y: marginVsDensity,
          type: 'scatter',
          mode: 'lines',
          name: `margin, ${p.flare_count} flares @ I=${p.flare_intensity_frac.toFixed(2)}`,
          line: { color: COLORS.margin, width: 2 },
        },
        {
          x: [density_grid[0], density_grid[density_grid.length - 1]],
          y: [0, 0],
          type: 'scatter',
          mode: 'lines',
          name: 'spoof threshold (margin = 0)',
          line: { color: COLORS.zero, width: 1, dash: 'dash' },
        },
        {
          x: [p.decoy_density],
          y: [marginNow],
          type: 'scatter',
          mode: 'markers',
          name: `current: margin = ${marginNow.toFixed(2)}${marginNow <= 0 ? ' (SPOOFED)' : ''}`,
          marker: { color: COLORS.marker, size: 10, symbol: 'diamond' },
        },
      ],
      layout: {
        xaxis: { title: { text: 'decoy / CSO density' } },
        yaxis: { title: { text: 'discrimination margin [score]' } },
      },
    };

    // === Plot 4: J/S vs range with burn-through crossover ===================
    const sjDb = ranges_m.map((range_m) =>
      linearToDb(
        1.0 /
          Math.max(
            signalToJammerLinear({
              snr_ref_db: p.snr_ref_db,
              js_ref_db: p.js_db,
              range_ref_m: p.range_ref_m,
              rcs_ref_m2: p.rcs_ref_m2,
              range_m,
              rcs_m2: p.rcs_m2,
            }),
            1e-300,
          ),
      ),
    );
    const jsData: PlotSpec['data'] = [
      {
        x: ranges_m,
        y: sjDb,
        type: 'scatter',
        mode: 'lines',
        name: 'J/S [dB]',
        line: { color: COLORS.jammed, width: 2 },
      },
      {
        x: [ranges_m[0], ranges_m[ranges_m.length - 1]],
        y: [0, 0],
        type: 'scatter',
        mode: 'lines',
        name: 'burn-through (J/S = 0 dB)',
        line: { color: COLORS.zero, width: 1.5, dash: 'dash' },
      },
    ];
    if (noiseOn && burnThrough_m > 0) {
      jsData.push({
        x: [burnThrough_m],
        y: [0],
        type: 'scatter',
        mode: 'markers',
        name: `crossover R = ${(burnThrough_m / 1000).toFixed(1)} km`,
        marker: { color: COLORS.marker, size: 12, symbol: 'x' },
      });
    }
    const jsVsRange: PlotSpec = {
      id: 'js-vs-range',
      title: 'J/S vs range — burn-through crossover (skin echo ∝1/R⁴ beats jammer ∝1/R²)',
      data: jsData,
      layout: {
        xaxis: { title: { text: 'range [m]' } },
        yaxis: { title: { text: 'J/S [dB]  (>0 = jammer wins)' } },
        annotations: [
          {
            x: 0.02,
            y: 0.04,
            xref: 'paper',
            yref: 'paper',
            align: 'left',
            showarrow: false,
            text:
              `jammer: ${p.jammer_type}${jammerOn ? `, J/S=${p.js_db} dB @ anchor` : ''}` +
              ` · CFAR N=${p.num_ref_cells} · α=${cfarAlpha(p.num_ref_cells, p.pfa).toFixed(2)}` +
              ` · thresh SNR=${threshSnrDb.toFixed(1)} dB`,
            font: { size: 11, color: '#a0aec0' },
          },
        ],
      },
    };

    return [pdVsRange, falseCount, irMargin, jsVsRange];
  },
};
