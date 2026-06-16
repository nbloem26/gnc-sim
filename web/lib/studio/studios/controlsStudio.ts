/**
 * Controls / Autopilot Studio (issue #113) — the headline "C in GNC" workbench.
 *
 * An interactive loop-shaping bench for the pitch/yaw acceleration autopilot.
 * It models a reduced-order version of the loop the C++ core flies in full 6DOF:
 *
 *   AUTOPILOT lead  →  FIN ACTUATOR lag  →  AIRFRAME short-period
 *
 * and renders the four classic controls views: closed-loop step response,
 * open-loop Bode (with the gain/phase-margin crossovers), open-loop Nyquist
 * (with the −1 point + margins), and the nonlinear actuator-saturation trace.
 *
 * The reduced-order model and its mapping to the C++ (Aero::momentAero,
 * Gnc FinActuator + Autopilot) is documented in `controlsMath.ts`. The full
 * 6DOF-linearisation-via-WASM is a deliberate follow-up (see that file + the
 * studio description).
 *
 * Naming follows AGENTS.md — physical params carry unit suffixes
 * (`actuator_bandwidth_hz`, `actuator_rate_limit_degps`, `rate_gain_kr_ms`).
 */

import type { StudioDef, ParamValues, PlotSpec } from '../types';
import {
  type ControlsModel,
  airframeOmegaRadps,
  computeMargins,
  computeNyquist,
  computeStepResponse,
  computeActuatorSim,
} from './controlsMath';

// Shared palette (consistent with the dark Plotly theme).
const C_CMD = 'rgba(246, 173, 85, 0.9)'; // amber — command / reference
const C_OUT = '#4fd1c5'; // teal — response
const C_MARK = '#f56565'; // red — margins / −1 point
const C_DIM = 'rgba(200, 210, 220, 0.45)';

/** Assemble the SI/native model from the raw param values. */
function modelFromParams(params: ParamValues): ControlsModel {
  const bandwidth_hz = Number(params.actuator_bandwidth_hz);
  return {
    static_margin_caliber: Number(params.static_margin_caliber),
    control_effectiveness: Number(params.control_effectiveness),
    actuator_tau_s: 1 / (2 * Math.PI * Math.max(bandwidth_hz, 1e-3)),
    actuator_rate_limit_degps: Number(params.actuator_rate_limit_degps),
    actuator_travel_limit_deg: Number(params.actuator_travel_limit_deg),
    accel_gain_ka: Number(params.accel_gain_ka),
    // The rate-loop gain is entered in milliseconds (a lead time constant), → s.
    rate_gain_kr_s: Number(params.rate_gain_kr_ms) / 1000,
  };
}

function fmt(x: number, digits = 2): string {
  if (!Number.isFinite(x)) return '∞';
  return x.toFixed(digits);
}

export const controlsStudio: StudioDef = {
  id: 'controls-autopilot',
  label: 'Controls / Autopilot',
  description:
    'Loop-shape the pitch/yaw acceleration autopilot: airframe short-period (from static ' +
    'margin) → fin actuator lag (+ rate/travel limits) → autopilot accel+rate gains. ' +
    'Watch the step response, Bode/Nyquist stability margins, and actuator saturation respond ' +
    'to the gains. Reduced-order model of the C++ 6DOF loop (full WASM linearisation is a follow-up).',
  params: [
    {
      kind: 'number',
      key: 'static_margin_caliber',
      label: 'Static margin',
      unit: 'cal',
      min_value: 0.1,
      max_value: 3,
      step: 0.05,
      default_value: 1.0,
      help: 'CG–CP separation (calibers). Stiffer airframe ⇒ higher short-period ωn (∝ −Cm_alpha).',
    },
    {
      kind: 'number',
      key: 'control_effectiveness',
      label: 'Control effectiveness',
      min_value: 0.2,
      max_value: 4,
      step: 0.05,
      default_value: 1.0,
      help: 'Airframe accel-per-deflection DC gain (∝ Cm_delta / Cm_alpha).',
    },
    {
      kind: 'number',
      key: 'actuator_bandwidth_hz',
      label: 'Actuator bandwidth',
      unit: 'Hz',
      min_value: 2,
      max_value: 40,
      step: 0.5,
      default_value: 15,
      help: 'First-order fin lag: tau = 1/(2π·BW). The dominant terminal-loop lag.',
    },
    {
      kind: 'number',
      key: 'actuator_rate_limit_degps',
      label: 'Actuator rate limit',
      unit: 'deg/s',
      min_value: 20,
      max_value: 1500,
      step: 10,
      default_value: 500,
      help: 'Max fin slew rate (the FinActuator rate_limit). Bites under hard commands.',
    },
    {
      kind: 'number',
      key: 'actuator_travel_limit_deg',
      label: 'Actuator travel limit',
      unit: 'deg',
      min_value: 5,
      max_value: 40,
      step: 1,
      default_value: 25,
      help: 'Mechanical deflection limit (the FinActuator deflection_limit).',
    },
    {
      kind: 'number',
      key: 'accel_gain_ka',
      label: 'Autopilot accel gain Ka',
      min_value: 0.5,
      max_value: 30,
      step: 0.5,
      default_value: 8.5,
      help: 'Outer forward accel-loop gain (the kp analogue). Crank it ⇒ less phase margin ⇒ ringing → instability.',
    },
    {
      kind: 'number',
      key: 'rate_gain_kr_ms',
      label: 'Autopilot rate gain Kr',
      unit: 'ms',
      min_value: 0,
      max_value: 400,
      step: 5,
      default_value: 360,
      help: 'Inner rate-feedback gain (the kd analogue). Adds damping → more phase margin.',
    },
    {
      kind: 'number',
      key: 'fin_command_deg',
      label: 'Actuator step command',
      unit: 'deg',
      min_value: 1,
      max_value: 60,
      step: 1,
      default_value: 30,
      help: 'Size of the commanded fin step for the saturation plot.',
    },
  ],
  presets: [
    {
      id: 'well-tuned',
      label: 'Well-tuned',
      values: {
        static_margin_caliber: 1.0,
        control_effectiveness: 1.0,
        actuator_bandwidth_hz: 15,
        actuator_rate_limit_degps: 600,
        actuator_travel_limit_deg: 25,
        accel_gain_ka: 8.5,
        rate_gain_kr_ms: 360,
        fin_command_deg: 5,
      },
    },
    {
      id: 'marginal',
      label: 'Marginal (high gain)',
      values: {
        static_margin_caliber: 0.4,
        control_effectiveness: 1.4,
        actuator_bandwidth_hz: 8,
        actuator_rate_limit_degps: 400,
        actuator_travel_limit_deg: 25,
        accel_gain_ka: 10,
        rate_gain_kr_ms: 40,
        fin_command_deg: 25,
      },
    },
    {
      id: 'rate-limited',
      label: 'Actuator rate-limited',
      values: {
        static_margin_caliber: 1.0,
        control_effectiveness: 1.0,
        actuator_bandwidth_hz: 20,
        actuator_rate_limit_degps: 60,
        actuator_travel_limit_deg: 25,
        accel_gain_ka: 8.5,
        rate_gain_kr_ms: 300,
        fin_command_deg: 40,
      },
    },
  ],
  compute(params: ParamValues): PlotSpec[] {
    const model = modelFromParams(params);
    const wn = airframeOmegaRadps(model);
    const margins = computeMargins(model);
    const nyquist = computeNyquist(model);
    const step = computeStepResponse(model);
    const fin_command_deg = Number(params.fin_command_deg);
    const act = computeActuatorSim(model, fin_command_deg);

    // ── Plot 1: closed-loop step response ──────────────────────────────────
    const finalVal = step.accel_response[step.accel_response.length - 1];
    const stepPlot: PlotSpec = {
      id: 'step',
      title:
        `Step response — rise ${fmt(step.rise_time_s * 1000, 0)} ms, ` +
        `overshoot ${fmt(step.overshoot_pct, 0)}%, ` +
        `settling ${fmt(step.settling_time_s * 1000, 0)} ms`,
      data: [
        {
          x: [step.t_s[0], step.t_s[step.t_s.length - 1]],
          y: [1, 1],
          type: 'scatter',
          mode: 'lines',
          name: 'command',
          line: { color: C_CMD, width: 1.5, dash: 'dash' },
        },
        {
          x: step.t_s,
          y: step.accel_response,
          type: 'scatter',
          mode: 'lines',
          name: 'accel response',
          line: { color: C_OUT, width: 2 },
        },
      ],
      layout: {
        xaxis: { title: { text: 't [s]' } },
        yaxis: { title: { text: 'normalized accel A_z' } },
        annotations: [
          {
            x: step.t_s[step.t_s.length - 1],
            y: finalVal,
            xanchor: 'right',
            yanchor: 'bottom',
            text: margins.stable ? 'stable' : 'UNSTABLE',
            showarrow: false,
            font: { color: margins.stable ? C_OUT : C_MARK },
          },
        ],
      },
    };

    // ── Plot 2: open-loop Bode with margin crossovers ──────────────────────
    const omega = margins.bode.map((p) => p.omega_radps);
    const bodePlot: PlotSpec = {
      id: 'bode',
      title:
        `Open-loop Bode — gain margin ${fmt(margins.gain_margin_db, 1)} dB ` +
        `@ ${fmt(margins.phase_crossover_radps, 1)} rad/s, ` +
        `phase margin ${fmt(margins.phase_margin_deg, 1)}° @ ${fmt(margins.gain_crossover_radps, 1)} rad/s`,
      data: [
        {
          x: omega,
          y: margins.bode.map((p) => p.gain_db),
          type: 'scatter',
          mode: 'lines',
          name: 'gain [dB]',
          line: { color: C_OUT, width: 2 },
          xaxis: 'x',
          yaxis: 'y',
        },
        {
          x: omega,
          y: margins.bode.map((p) => p.phase_deg),
          type: 'scatter',
          mode: 'lines',
          name: 'phase [deg]',
          line: { color: C_CMD, width: 2 },
          xaxis: 'x',
          yaxis: 'y2',
        },
        // 0 dB and −180° reference lines.
        {
          x: [omega[0], omega[omega.length - 1]],
          y: [0, 0],
          type: 'scatter',
          mode: 'lines',
          name: '0 dB',
          line: { color: C_DIM, width: 1, dash: 'dot' },
          xaxis: 'x',
          yaxis: 'y',
          showlegend: false,
        },
        {
          x: [omega[0], omega[omega.length - 1]],
          y: [-180, -180],
          type: 'scatter',
          mode: 'lines',
          name: '−180°',
          line: { color: C_DIM, width: 1, dash: 'dot' },
          xaxis: 'x',
          yaxis: 'y2',
          showlegend: false,
        },
        // Crossover markers.
        ...(Number.isFinite(margins.gain_crossover_radps)
          ? [{
              x: [margins.gain_crossover_radps],
              y: [0],
              type: 'scatter' as const,
              mode: 'markers' as const,
              name: 'gain xover',
              marker: { color: C_MARK, size: 9, symbol: 'circle' as const },
              xaxis: 'x',
              yaxis: 'y',
            }]
          : []),
        ...(Number.isFinite(margins.phase_crossover_radps)
          ? [{
              x: [margins.phase_crossover_radps],
              y: [-180],
              type: 'scatter' as const,
              mode: 'markers' as const,
              name: 'phase xover',
              marker: { color: C_MARK, size: 9, symbol: 'x' as const },
              xaxis: 'x',
              yaxis: 'y2',
            }]
          : []),
      ],
      layout: {
        xaxis: { title: { text: 'ω [rad/s]' }, type: 'log', domain: [0, 1] },
        yaxis: { title: { text: 'gain [dB]' }, domain: [0.55, 1] },
        yaxis2: { title: { text: 'phase [deg]' }, domain: [0, 0.45], anchor: 'x' },
        grid: { rows: 2, columns: 1, pattern: 'independent' },
      },
    };

    // ── Plot 3: open-loop Nyquist with the −1 point ────────────────────────
    const nyquistPlot: PlotSpec = {
      id: 'nyquist',
      title:
        `Open-loop Nyquist — ${margins.stable ? 'stable' : 'UNSTABLE'} ` +
        `(GM ${fmt(margins.gain_margin_db, 1)} dB, PM ${fmt(margins.phase_margin_deg, 1)}°)`,
      data: [
        {
          x: nyquist.map((p) => p.re),
          y: nyquist.map((p) => p.im),
          type: 'scatter',
          mode: 'lines',
          name: 'L(jω)',
          line: { color: C_OUT, width: 2 },
        },
        {
          x: nyquist.map((p) => p.re),
          y: nyquist.map((p) => -p.im),
          type: 'scatter',
          mode: 'lines',
          name: 'L(−jω)',
          line: { color: C_OUT, width: 1, dash: 'dot' },
          showlegend: false,
        },
        {
          x: [-1],
          y: [0],
          type: 'scatter',
          mode: 'text+markers',
          name: '−1',
          text: ['−1'],
          textposition: 'top center',
          marker: { color: C_MARK, size: 11, symbol: 'x' },
        },
      ],
      layout: {
        xaxis: { title: { text: 'Re L(jω)' }, zeroline: true, scaleanchor: 'y', scaleratio: 1 },
        yaxis: { title: { text: 'Im L(jω)' }, zeroline: true },
      },
    };

    // ── Plot 4: actuator saturation (commanded vs achieved) ────────────────
    const satNote = act.rate_saturated
      ? 'RATE-limited'
      : act.travel_saturated
        ? 'TRAVEL-limited'
        : 'within limits';
    const actuatorPlot: PlotSpec = {
      id: 'actuator',
      title: `Actuator saturation — ${fmt(fin_command_deg, 0)}° step (${satNote})`,
      data: [
        {
          x: act.t_s,
          y: act.commanded_deg,
          type: 'scatter',
          mode: 'lines',
          name: 'commanded δ [deg]',
          line: { color: C_CMD, width: 1.5, dash: 'dash' },
          xaxis: 'x',
          yaxis: 'y',
        },
        {
          x: act.t_s,
          y: act.achieved_deg,
          type: 'scatter',
          mode: 'lines',
          name: 'achieved δ [deg]',
          line: { color: C_OUT, width: 2 },
          xaxis: 'x',
          yaxis: 'y',
        },
        {
          x: act.t_s,
          y: act.achieved_rate_degps,
          type: 'scatter',
          mode: 'lines',
          name: 'achieved rate [deg/s]',
          line: { color: '#9f7aea', width: 1.5 },
          xaxis: 'x',
          yaxis: 'y2',
        },
        {
          x: [act.t_s[0], act.t_s[act.t_s.length - 1]],
          y: [model.actuator_rate_limit_degps, model.actuator_rate_limit_degps],
          type: 'scatter',
          mode: 'lines',
          name: 'rate limit',
          line: { color: C_MARK, width: 1, dash: 'dot' },
          xaxis: 'x',
          yaxis: 'y2',
        },
      ],
      layout: {
        xaxis: { title: { text: 't [s]' }, domain: [0, 1] },
        yaxis: { title: { text: 'deflection [deg]' }, domain: [0.55, 1] },
        yaxis2: { title: { text: 'rate [deg/s]' }, domain: [0, 0.45], anchor: 'x' },
        grid: { rows: 2, columns: 1, pattern: 'independent' },
      },
    };

    return [stepPlot, bodePlot, nyquistPlot, actuatorPlot];
  },
};
