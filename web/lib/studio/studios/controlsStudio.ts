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

import type { SimConfig } from '@/lib/types';
import { loadPresetConfig } from '@/lib/presets';
import { linearizeAirframe, isMockMode, isResolved } from '@/lib/wasmRunner';
import type { StudioDef, ParamValues, PlotSpec } from '../types';
import {
  type ControlsModel,
  airframeOmegaRadps,
  airframeZeta,
  computeMargins,
  computeNyquist,
  computeStepResponse,
  computeActuatorSim,
} from './controlsMath';

/** Base airframe preset the real linearization runs against (the hi-fi 6DOF config). */
const AIRFRAME_PRESET_FILE = 'homing_6dof_hifi.json';

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

/**
 * Try to replace the reduced static-margin airframe with the REAL 6DOF airframe+actuator
 * linearization from the WASM core (issue #122): load the hi-fi preset, override its static
 * stability with the studio's `static_margin_caliber` knob (scaling Cm_alpha so the knob still
 * stiffens/softens the real airframe), and linearize about the chosen Mach/altitude/alpha. On
 * success it stamps the real ωn / ζ / control-effectiveness onto the model. Returns a short status
 * string for the plot captions; returns null (model untouched) in mock mode or if WASM lacks the
 * `linearize` export — the reduced model is then used as a graceful fallback.
 */
async function applyRealLinearization(
  model: ControlsModel,
  params: ParamValues,
): Promise<string | null> {
  // Mock mode (no WASM artifact) → keep the reduced model.
  if (isResolved() && isMockMode()) return null;

  let base: SimConfig;
  try {
    base = await loadPresetConfig(AIRFRAME_PRESET_FILE);
  } catch {
    return null; // preset unavailable in this build → reduced fallback
  }

  // Map the studio's static-margin knob onto the real Cm_alpha slope: 1 caliber == the preset's
  // nominal stability, scaled linearly (ωn ∝ sqrt(-Cm_alpha) ∝ sqrt(static margin)). The hi-fi
  // preset ships a cm_table; clearing it lets the linear cm_alpha slope drive the static stability
  // so the knob is honoured. Larger SM ⇒ more negative Cm_alpha ⇒ stiffer airframe.
  const sm = Math.max(Number(params.static_margin_caliber), 1e-3);
  const cfg: SimConfig = {
    ...base,
    aero: {
      ...base.aero,
      cm_table: [],
      cm_alpha: -8.0 * sm,
    },
  };

  const trim = {
    mach: Number(params.trim_mach),
    altitude_m: Number(params.trim_altitude_m),
    alpha_rad: (Number(params.trim_alpha_deg) * Math.PI) / 180,
  };

  let lin;
  try {
    lin = await linearizeAirframe(cfg, trim);
  } catch {
    return null; // linearization failed → reduced fallback
  }
  if (!lin) return null; // WASM present but no `linearize` export → reduced fallback

  // Stamp the real short-period mode onto the model.
  if (lin.omega_n_radps > 0) model.airframe_omega_radps = lin.omega_n_radps;
  if (lin.zeta > 0) model.airframe_zeta = lin.zeta;

  // Normalize the real accel-per-deflection DC gain to the studio's dimensionless K_af, preserving
  // the user's `control_effectiveness` knob as a multiplier on the real airframe gain. The
  // reference (1131 m/s²/rad) is the preset's nominal effectiveness so the default knob (=1) maps
  // to a unity normalized gain — matching the reduced model's scaling.
  const K_REF_MPS2_PER_RAD = 1131.25;
  const normalized = lin.control_effectiveness_mps2_per_rad / K_REF_MPS2_PER_RAD;
  if (Number.isFinite(normalized) && normalized !== 0) {
    model.control_effectiveness = Number(params.control_effectiveness) * normalized;
  }

  return (
    `REAL 6DOF airframe @ M${fmt(trim.mach, 1)}, ${fmt(trim.altitude_m / 1000, 1)} km, ` +
    `α=${fmt(params.trim_alpha_deg as number, 1)}° → ωn ${fmt(lin.omega_n_radps, 1)} rad/s, ` +
    `ζ ${fmt(lin.zeta, 3)}${lin.stable ? '' : ' (UNSTABLE airframe)'}`
  );
}

export const controlsStudio: StudioDef = {
  id: 'controls-autopilot',
  label: 'Controls / Autopilot',
  description:
    'Loop-shape the pitch/yaw acceleration autopilot: airframe short-period (from the REAL 6DOF ' +
    'airframe+actuator linearisation in the C++ core via WASM, at the chosen Mach/altitude/α) → ' +
    'fin actuator lag (+ rate/travel limits) → autopilot accel+rate gains. Watch the step response, ' +
    'Bode/Nyquist stability margins, and actuator saturation respond to the gains. Falls back to a ' +
    'reduced-order airframe in mock mode (no WASM artifact).',
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
      key: 'trim_mach',
      label: 'Flight condition — Mach',
      unit: 'M',
      min_value: 0.5,
      max_value: 5,
      step: 0.1,
      default_value: 2.5,
      help: 'Freestream Mach the real 6DOF airframe is linearized about (sets q̄ → ωn).',
    },
    {
      kind: 'number',
      key: 'trim_altitude_m',
      label: 'Flight condition — altitude',
      unit: 'm',
      min_value: 0,
      max_value: 30000,
      step: 500,
      default_value: 6000,
      help: 'Geometric altitude (USSA76 density). Higher ⇒ lower q̄ ⇒ softer airframe.',
    },
    {
      kind: 'number',
      key: 'trim_alpha_deg',
      label: 'Flight condition — trim α',
      unit: 'deg',
      min_value: 0.5,
      max_value: 15,
      step: 0.5,
      default_value: 3,
      help: 'Trim angle of attack the airframe Jacobian is taken about.',
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
        trim_mach: 2.5,
        trim_altitude_m: 6000,
        trim_alpha_deg: 3,
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
        trim_mach: 1.5,
        trim_altitude_m: 10000,
        trim_alpha_deg: 5,
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
        trim_mach: 3.0,
        trim_altitude_m: 4000,
        trim_alpha_deg: 4,
        actuator_bandwidth_hz: 20,
        actuator_rate_limit_degps: 60,
        actuator_travel_limit_deg: 25,
        accel_gain_ka: 8.5,
        rate_gain_kr_ms: 300,
        fin_command_deg: 40,
      },
    },
  ],
  async compute(params: ParamValues): Promise<PlotSpec[]> {
    const model = modelFromParams(params);

    // Drive the airframe from the REAL 6DOF linearization via WASM (issue #122). On success this
    // mutates `model` (ωn / ζ / control-effectiveness) and returns a status note; in mock mode or
    // without the WASM `linearize` export it returns null and we keep the reduced-order airframe.
    const linNote = await applyRealLinearization(model, params);
    const airframeNote = linNote ?? 'reduced-order airframe (mock mode — no WASM)';

    const wn = airframeOmegaRadps(model);
    const zeta = airframeZeta(model);
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
        `settling ${fmt(step.settling_time_s * 1000, 0)} ms` +
        `  ·  airframe ωn ${fmt(wn, 1)} rad/s, ζ ${fmt(zeta, 3)}\n${airframeNote}`,
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
