/**
 * Example studio — a pure-math demo that needs no engine.
 *
 * It plots a damped sinusoid  x(t) = A · e^(−ζ·t) · sin(2π·f·t + φ), driven by
 * editable params. This is the simplest end-to-end proof of the studio pattern
 * and the template later (engine-backed) studios copy:
 *
 *   1. declare a typed `params` schema (the shell renders the controls),
 *   2. implement `compute(params) -> PlotSpec[]` (here a pure function; a real
 *      studio would instead `await runSim(config)` from `@/lib/wasmRunner`),
 *   3. register it (see studios/index.ts).
 *
 * Naming follows the repo convention — physical params carry unit suffixes
 * (`freq_hz`, `damping_per_s`, `duration_s`).
 */

import type { StudioDef, ParamValues, PlotSpec } from '../types';

const TWO_PI = 2 * Math.PI;

/** Linearly spaced sample grid, inclusive of both endpoints. */
function linspace(start: number, end: number, count: number): number[] {
  if (count < 2) return [start];
  const out = new Array<number>(count);
  const span = (end - start) / (count - 1);
  for (let i = 0; i < count; i++) out[i] = start + i * span;
  return out;
}

export const exampleStudio: StudioDef = {
  id: 'damped-sinusoid',
  label: 'Damped sinusoid (demo)',
  description:
    'A pure-math reference studio: x(t) = A·e^(−ζt)·sin(2πft + φ). No engine — ' +
    'it validates the shell end-to-end and is the template engine-backed studios copy.',
  params: [
    {
      kind: 'number',
      key: 'amplitude',
      label: 'Amplitude A',
      min_value: 0.1,
      max_value: 5,
      step: 0.1,
      default_value: 1,
    },
    {
      kind: 'number',
      key: 'freq_hz',
      label: 'Frequency',
      unit: 'Hz',
      min_value: 0.1,
      max_value: 10,
      step: 0.1,
      default_value: 1.5,
    },
    {
      kind: 'number',
      key: 'damping_per_s',
      label: 'Damping ζ',
      unit: '1/s',
      min_value: 0,
      max_value: 5,
      step: 0.05,
      default_value: 0.6,
      help: 'Exponential decay rate of the envelope.',
    },
    {
      kind: 'number',
      key: 'phase_deg',
      label: 'Phase φ',
      unit: 'deg',
      min_value: -180,
      max_value: 180,
      step: 5,
      default_value: 0,
    },
    {
      kind: 'number',
      key: 'duration_s',
      label: 'Duration',
      unit: 's',
      min_value: 1,
      max_value: 20,
      step: 1,
      default_value: 6,
    },
    {
      kind: 'boolean',
      key: 'show_envelope',
      label: 'Show decay envelope',
      default_value: true,
    },
    {
      kind: 'enum',
      key: 'line_shape',
      label: 'Line shape',
      options: [
        { value: 'linear', label: 'Linear' },
        { value: 'spline', label: 'Spline' },
      ],
      default_value: 'spline',
    },
  ],
  presets: [
    {
      id: 'underdamped',
      label: 'Underdamped',
      values: { amplitude: 1, freq_hz: 2, damping_per_s: 0.3, phase_deg: 0, duration_s: 8, show_envelope: true, line_shape: 'spline' },
    },
    {
      id: 'heavy',
      label: 'Heavily damped',
      values: { amplitude: 2, freq_hz: 1, damping_per_s: 2.5, phase_deg: 0, duration_s: 4, show_envelope: true, line_shape: 'spline' },
    },
  ],
  compute(params: ParamValues): PlotSpec[] {
    const amplitude = Number(params.amplitude);
    const freq_hz = Number(params.freq_hz);
    const damping_per_s = Number(params.damping_per_s);
    const phase_rad = (Number(params.phase_deg) * Math.PI) / 180;
    const duration_s = Number(params.duration_s);
    const showEnvelope = Boolean(params.show_envelope);
    const lineShape = String(params.line_shape) === 'linear' ? 'linear' : 'spline';

    const sampleCount = 600;
    const t_s = linspace(0, duration_s, sampleCount);
    const envelope = t_s.map((t) => amplitude * Math.exp(-damping_per_s * t));
    const signal = t_s.map(
      (t, i) => envelope[i] * Math.sin(TWO_PI * freq_hz * t + phase_rad),
    );

    const data: PlotSpec['data'] = [
      {
        x: t_s,
        y: signal,
        type: 'scatter',
        mode: 'lines',
        name: 'x(t)',
        line: { color: '#4fd1c5', width: 2, shape: lineShape },
      },
    ];

    if (showEnvelope) {
      const envColor = 'rgba(246, 173, 85, 0.7)';
      data.push(
        {
          x: t_s,
          y: envelope,
          type: 'scatter',
          mode: 'lines',
          name: '±envelope',
          line: { color: envColor, width: 1, dash: 'dot' },
        },
        {
          x: t_s,
          y: envelope.map((v) => -v),
          type: 'scatter',
          mode: 'lines',
          name: '−envelope',
          line: { color: envColor, width: 1, dash: 'dot' },
          showlegend: false,
        },
      );
    }

    return [
      {
        id: 'time-series',
        title: 'Damped sinusoid — time response',
        data,
        layout: {
          xaxis: { title: { text: 't [s]' } },
          yaxis: { title: { text: 'x(t)' } },
        },
      },
    ];
  },
};
