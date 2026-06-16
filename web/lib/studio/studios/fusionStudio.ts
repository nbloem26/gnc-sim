/**
 * Fusion / Tracking Studio (issue #106) — pure client-side TS math, no engine.
 *
 * Place 2–3 sensors (radar + IR) around a target and explore how
 * information-form fusion (Σ HᵀR⁻¹H, inverted) shrinks and reorients the
 * position-error covariance vs any single sensor. Four plots:
 *
 *   1. Covariance ellipses (1σ) — each single sensor vs the fused estimate.
 *   2. GDOP map — √trace(P) over a grid of target positions for this layout.
 *   3. Fusion gain — fused position-σ vs the best single sensor (bar chart).
 *   4. Track purity vs clutter/decoy density — PDA-style association model.
 *
 * The linear algebra and models live in `fusionMath.ts`. Units in names:
 * *_m metres, *_mrad milliradians (UI) → radians (math), *_perKm2 → /m² (math).
 */

import type { StudioDef, ParamValues, PlotSpec } from '../types';
import {
  type Sensor,
  covToEllipse,
  ellipsePolyline,
  sensorInformation,
  covFromInformation,
  fusedInformation,
  positionSigma_m,
  gdopAt,
  trackPurity,
} from './fusionMath';

const COLORS = ['#4fd1c5', '#f6ad55', '#9f7aea']; // s1, s2, s3
const FUSED_COLOR = '#f56565';

function linspace(a: number, b: number, n: number): number[] {
  if (n < 2) return [a];
  const out = new Array<number>(n);
  const d = (b - a) / (n - 1);
  for (let i = 0; i < n; i++) out[i] = a + i * d;
  return out;
}

/** Build the active sensor list from params (target is fixed at origin). */
function buildSensors(p: ParamValues): Sensor[] {
  const sensors: Sensor[] = [
    {
      id: 'S1 radar',
      kind: 'radar',
      x_m: Number(p.s1_x_m),
      y_m: Number(p.s1_y_m),
      sigmaAngle_rad: Number(p.s1_sigmaAngle_mrad) * 1e-3,
      sigmaRange_m: Number(p.s1_sigmaRange_m),
    },
    {
      id: 'S2 IR',
      kind: 'ir',
      x_m: Number(p.s2_x_m),
      y_m: Number(p.s2_y_m),
      sigmaAngle_rad: Number(p.s2_sigmaAngle_mrad) * 1e-3,
      // IR is angles-only: a deliberately huge range σ → its range row ≈ 0.
      sigmaRange_m: 1e6,
    },
  ];
  if (Boolean(p.s3_enabled)) {
    sensors.push({
      id: 'S3 radar',
      kind: 'radar',
      x_m: Number(p.s3_x_m),
      y_m: Number(p.s3_y_m),
      sigmaAngle_rad: Number(p.s3_sigmaAngle_mrad) * 1e-3,
      sigmaRange_m: Number(p.s3_sigmaRange_m),
    });
  }
  return sensors;
}

/** Covariance ellipses (1σ) — each single sensor vs the fused estimate. */
function ellipsesPlot(sensors: Sensor[], tx_m: number, ty_m: number): PlotSpec {
  const data: PlotSpec['data'] = [];

  // Target + sensor markers.
  data.push({
    x: [tx_m],
    y: [ty_m],
    type: 'scatter',
    mode: 'text+markers',
    name: 'target',
    text: ['target'],
    textposition: 'top center',
    marker: { color: '#e2e8f0', size: 10, symbol: 'x' },
  });
  data.push({
    x: sensors.map((s) => s.x_m),
    y: sensors.map((s) => s.y_m),
    type: 'scatter',
    mode: 'text+markers',
    name: 'sensors',
    text: sensors.map((s) => s.id),
    textposition: 'bottom center',
    marker: {
      color: sensors.map((_, i) => COLORS[i % COLORS.length]),
      size: 9,
      symbol: 'triangle-up',
    },
  });

  // Per-sensor 1σ ellipse (where observable). A lone bearing-only sensor is
  // singular → skip its (infinite) ellipse but still show the marker.
  sensors.forEach((s, i) => {
    const cov = covFromInformation(sensorInformation(s, tx_m, ty_m));
    if (!cov) return;
    const e = covToEllipse(cov, tx_m, ty_m);
    // Cap absurd single-sensor ellipses so the plot stays readable.
    if (e.a_m > 5e4) return;
    const poly = ellipsePolyline(e);
    data.push({
      x: poly.x_m,
      y: poly.y_m,
      type: 'scatter',
      mode: 'lines',
      name: `${s.id} 1σ`,
      line: { color: COLORS[i % COLORS.length], width: 1.5, dash: 'dot' },
    });
  });

  // Fused 1σ ellipse.
  const fusedCov = covFromInformation(fusedInformation(sensors, tx_m, ty_m));
  if (fusedCov) {
    const e = covToEllipse(fusedCov, tx_m, ty_m);
    const poly = ellipsePolyline(e);
    data.push({
      x: poly.x_m,
      y: poly.y_m,
      type: 'scatter',
      mode: 'lines',
      name: 'fused 1σ',
      line: { color: FUSED_COLOR, width: 2.5 },
    });
  }

  return {
    id: 'ellipses',
    title: '1σ position-error ellipses — single sensors vs fused',
    data,
    layout: {
      xaxis: { title: { text: 'x [m]' }, scaleanchor: 'y', scaleratio: 1 },
      yaxis: { title: { text: 'y [m]' } },
      legend: { orientation: 'h' },
    },
  };
}

/** GDOP map — √trace(P) over a grid of target positions. */
function gdopPlot(sensors: Sensor[], halfSpan_m: number): PlotSpec {
  const n = 70;
  const axis = linspace(-halfSpan_m, halfSpan_m, n);
  const z: number[][] = [];
  for (let j = 0; j < n; j++) {
    const row = new Array<number>(n);
    for (let i = 0; i < n; i++) {
      row[i] = gdopAt(sensors, axis[i], axis[j], 5e3);
    }
    z.push(row);
  }
  return {
    id: 'gdop',
    title: 'GDOP map — fused position σ = √trace(P) [m] over target position',
    data: [
      {
        x: axis,
        y: axis,
        z,
        type: 'heatmap',
        colorscale: 'Viridis',
        zsmooth: 'best',
        colorbar: { title: { text: 'σ_pos [m]' } },
      },
      {
        x: sensors.map((s) => s.x_m),
        y: sensors.map((s) => s.y_m),
        type: 'scatter',
        mode: 'markers',
        name: 'sensors',
        marker: { color: '#ffffff', size: 9, symbol: 'triangle-up' },
      },
    ],
    layout: {
      xaxis: { title: { text: 'x [m]' }, scaleanchor: 'y', scaleratio: 1 },
      yaxis: { title: { text: 'y [m]' } },
    },
  };
}

/** Fusion gain — fused σ vs each single sensor's σ (bar chart). */
function fusionGainPlot(sensors: Sensor[], tx_m: number, ty_m: number): PlotSpec {
  const labels: string[] = [];
  const sigmas_m: number[] = [];
  const colors: string[] = [];

  sensors.forEach((s, i) => {
    const cov = covFromInformation(sensorInformation(s, tx_m, ty_m));
    labels.push(s.id);
    sigmas_m.push(cov ? Math.min(positionSigma_m(cov), 5e3) : 5e3);
    colors.push(COLORS[i % COLORS.length]);
  });

  const fusedCov = covFromInformation(fusedInformation(sensors, tx_m, ty_m));
  const fusedSigma_m = fusedCov ? positionSigma_m(fusedCov) : Number.NaN;
  labels.push('fused');
  sigmas_m.push(fusedSigma_m);
  colors.push(FUSED_COLOR);

  // Best single sensor (finite) for the annotated gain factor.
  const singleSigmas = sigmas_m.slice(0, sensors.length).filter((v) => v < 5e3);
  const bestSingle_m = singleSigmas.length ? Math.min(...singleSigmas) : Number.NaN;
  const gain =
    Number.isFinite(bestSingle_m) && Number.isFinite(fusedSigma_m)
      ? bestSingle_m / fusedSigma_m
      : Number.NaN;

  return {
    id: 'fusion-gain',
    title: Number.isFinite(gain)
      ? `Fusion gain — fused σ is ${gain.toFixed(2)}× tighter than the best single sensor`
      : 'Fusion gain — position σ per sensor vs fused',
    data: [
      {
        x: labels,
        y: sigmas_m,
        type: 'bar',
        marker: { color: colors },
        text: sigmas_m.map((v) => (v >= 5e3 ? '∞ (unobservable)' : `${v.toFixed(1)} m`)),
        textposition: 'outside',
      },
    ],
    layout: {
      yaxis: { title: { text: 'position σ = √trace(P) [m]' } },
      xaxis: { title: { text: 'estimator' } },
    },
  };
}

/** Track purity vs clutter / decoy density — PDA-style association model. */
function purityPlot(
  sensors: Sensor[],
  tx_m: number,
  ty_m: number,
  maxClutter_perKm2: number,
  detectionProb: number,
  gateSize: number,
): PlotSpec {
  // Gate footprint scales with the fused innovation covariance: A_gate ≈
  // π · γ · σ_pos² (γ = gate χ² size, σ_pos² ≈ |S|). This ties purity to the
  // actual fusion geometry — tighter fusion ⇒ smaller gate ⇒ higher purity.
  const fusedCov = covFromInformation(fusedInformation(sensors, tx_m, ty_m));
  const sigPos_m = fusedCov ? positionSigma_m(fusedCov) : 50;
  const gateArea_m2 = Math.PI * gateSize * sigPos_m * sigPos_m;

  const densities_perKm2 = linspace(0, maxClutter_perKm2, 120);
  const purity = densities_perKm2.map((d) =>
    trackPurity(d / 1e6, gateArea_m2, detectionProb),
  );

  return {
    id: 'purity',
    title: 'Track purity vs clutter/decoy density (PDA association model)',
    data: [
      {
        x: densities_perKm2,
        y: purity,
        type: 'scatter',
        mode: 'lines',
        name: 'P(correct association)',
        line: { color: '#48bb78', width: 2.5 },
      },
    ],
    layout: {
      xaxis: { title: { text: 'clutter / decoy density [1/km²]' } },
      yaxis: { title: { text: 'track purity  P_D/(P_D+μ)' }, range: [0, 1] },
      annotations: [
        {
          x: maxClutter_perKm2,
          y: purity[purity.length - 1],
          xanchor: 'right',
          yanchor: 'bottom',
          text: `gate σ_pos ≈ ${sigPos_m.toFixed(1)} m`,
          showarrow: false,
          font: { color: '#a0aec0' },
        },
      ],
    },
  };
}

export const fusionStudio: StudioDef = {
  id: 'fusion-tracking',
  label: 'Fusion / Tracking',
  description:
    'Information-form multi-sensor fusion: place radar + IR sensors around a ' +
    'target and watch Σ HᵀR⁻¹H, inverted, shrink and reorient the 1σ position ' +
    'ellipse. Shows a GDOP map, the fusion gain vs the best single sensor, and a ' +
    'PDA-style track-purity-vs-clutter curve. Pure client-side math (no engine).',
  params: [
    // Sensor 1 — radar.
    { kind: 'number', key: 's1_x_m', label: 'S1 radar x', unit: 'm', min_value: -2000, max_value: 2000, step: 25, default_value: -800 },
    { kind: 'number', key: 's1_y_m', label: 'S1 radar y', unit: 'm', min_value: -2000, max_value: 2000, step: 25, default_value: -600 },
    { kind: 'number', key: 's1_sigmaAngle_mrad', label: 'S1 bearing σ', unit: 'mrad', min_value: 0.2, max_value: 30, step: 0.2, default_value: 5 },
    { kind: 'number', key: 's1_sigmaRange_m', label: 'S1 range σ', unit: 'm', min_value: 1, max_value: 200, step: 1, default_value: 15 },
    // Sensor 2 — IR (angles-only).
    { kind: 'number', key: 's2_x_m', label: 'S2 IR x', unit: 'm', min_value: -2000, max_value: 2000, step: 25, default_value: 900 },
    { kind: 'number', key: 's2_y_m', label: 'S2 IR y', unit: 'm', min_value: -2000, max_value: 2000, step: 25, default_value: -700 },
    { kind: 'number', key: 's2_sigmaAngle_mrad', label: 'S2 bearing σ', unit: 'mrad', min_value: 0.1, max_value: 20, step: 0.1, default_value: 1, help: 'IR is angles-only (range unobservable).' },
    // Sensor 3 — optional second radar.
    { kind: 'boolean', key: 's3_enabled', label: 'Enable S3 (radar)', default_value: false },
    { kind: 'number', key: 's3_x_m', label: 'S3 radar x', unit: 'm', min_value: -2000, max_value: 2000, step: 25, default_value: 0 },
    { kind: 'number', key: 's3_y_m', label: 'S3 radar y', unit: 'm', min_value: -2000, max_value: 2000, step: 25, default_value: 1000 },
    { kind: 'number', key: 's3_sigmaAngle_mrad', label: 'S3 bearing σ', unit: 'mrad', min_value: 0.2, max_value: 30, step: 0.2, default_value: 8 },
    { kind: 'number', key: 's3_sigmaRange_m', label: 'S3 range σ', unit: 'm', min_value: 1, max_value: 200, step: 1, default_value: 25 },
    // Map extent.
    { kind: 'number', key: 'mapHalfSpan_m', label: 'GDOP map half-span', unit: 'm', min_value: 500, max_value: 3000, step: 100, default_value: 1500 },
    // Purity / PDA model.
    { kind: 'number', key: 'maxClutter_perKm2', label: 'Max clutter density', unit: '1/km²', min_value: 1, max_value: 500, step: 1, default_value: 200 },
    { kind: 'number', key: 'detectionProb', label: 'Detection prob P_D', min_value: 0.3, max_value: 1, step: 0.01, default_value: 0.9 },
    { kind: 'number', key: 'gateSize', label: 'Gate size γ (χ²)', min_value: 4, max_value: 25, step: 0.5, default_value: 9, help: 'Validation-gate χ² threshold; larger γ ⇒ bigger gate ⇒ more clutter admitted.' },
  ],
  presets: [
    {
      id: 'radar-ir-crossfix',
      label: 'Radar + IR cross-fix',
      values: {
        s1_x_m: -800, s1_y_m: -600, s1_sigmaAngle_mrad: 5, s1_sigmaRange_m: 15,
        s2_x_m: 900, s2_y_m: -700, s2_sigmaAngle_mrad: 1,
        s3_enabled: false, s3_x_m: 0, s3_y_m: 1000, s3_sigmaAngle_mrad: 8, s3_sigmaRange_m: 25,
        mapHalfSpan_m: 1500, maxClutter_perKm2: 200, detectionProb: 0.9, gateSize: 9,
      },
    },
    {
      id: 'three-radar-triangulation',
      label: 'Three-radar triangulation',
      values: {
        s1_x_m: -1000, s1_y_m: -700, s1_sigmaAngle_mrad: 8, s1_sigmaRange_m: 30,
        s2_x_m: 1000, s2_y_m: -700, s2_sigmaAngle_mrad: 8,
        s3_enabled: true, s3_x_m: 0, s3_y_m: 1100, s3_sigmaAngle_mrad: 8, s3_sigmaRange_m: 30,
        mapHalfSpan_m: 1500, maxClutter_perKm2: 200, detectionProb: 0.85, gateSize: 9,
      },
    },
  ],
  compute(params: ParamValues): PlotSpec[] {
    const sensors = buildSensors(params);
    const tx_m = 0;
    const ty_m = 0;
    const halfSpan_m = Number(params.mapHalfSpan_m);

    return [
      ellipsesPlot(sensors, tx_m, ty_m),
      gdopPlot(sensors, halfSpan_m),
      fusionGainPlot(sensors, tx_m, ty_m),
      purityPlot(
        sensors,
        tx_m,
        ty_m,
        Number(params.maxClutter_perKm2),
        Number(params.detectionProb),
        Number(params.gateSize),
      ),
    ];
  },
};
