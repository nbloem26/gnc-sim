/**
 * Guidance Studio — engine-backed guidance-law trade study (issue #107).
 *
 * Unlike the pure-math example studio, this one drives the real C++ core (via
 * `runSim` from `@/lib/wasmRunner`). It loads a homing preset as a base config,
 * then sweeps the navigation gain N across a handful of points for each enabled
 * guidance law (PN / APN / ZEM-ZEV), running a full engagement per (law, N).
 *
 * For each run it records:
 *   - miss_distance [m]                       (analytic CPA from the core)
 *   - hit (miss < lethal radius)              → P(hit) aggregated per law over N
 *   - control effort [m/s · ?]  =  ∫|a_cmd| dt  (trapezoidal over the series,
 *     using the accel-command channel when present, else |vehicle accel|)
 *
 * Plots:
 *   1. Miss vs control-effort scatter — one series per law; the chosen N marked.
 *   2. Miss vs N line — one series per law.
 *   3. Trajectory overlay — vehicle path per law at the currently chosen N.
 *
 * The sweep is intentionally small (a fixed grid of N) so the shell stays
 * responsive; the shell already debounces param changes.
 *
 * Mock mode: when no WASM artifact is present (`isMockMode()`), every run returns
 * the same committed sample, so the trade study is meaningless. We mirror the
 * MonteCarloPanel / ComparePanel handling: detect it, render a clear note plot,
 * and never crash.
 *
 * Naming follows the repo convention (units in names): `maneuver_g`,
 * `maneuver_freq_hz`, `time_constant_s`, `launch_speed_mps`, `nav_constant`
 * (dimensionless). Contract keys passed to the core keep their schema spelling
 * (`guidance.nav_constant`, `guidance.time_constant`, `target.maneuver_g`,
 * `target.maneuver_freq`).
 */

import type { SimConfig, SimResult } from '@/lib/types';
import { loadPresetConfig } from '@/lib/presets';
import { runSim, isMockMode, isResolved } from '@/lib/wasmRunner';
import type { StudioDef, ParamValues, PlotSpec } from '../types';

/** Lethal radius [m] — matches the core's kLethalRadius used for `intercept`. */
const LETHAL_RADIUS_M = 3.0;

/** Base homing preset the trade study perturbs. */
const BASE_PRESET_FILE = 'homing_3dof.json';

/** A modest, fixed N sweep keeps every param change responsive. */
const N_SWEEP = [2.5, 3, 3.5, 4, 4.5, 5];

interface LawDef {
  /** Contract value for `guidance.law`. */
  key: string;
  /** Param key (boolean toggle) that enables this law. */
  paramKey: string;
  /** Human-facing label. */
  label: string;
  /** Plot colour. */
  color: string;
}

const LAWS: LawDef[] = [
  { key: 'pronav', paramKey: 'include_pn', label: 'PN', color: '#4fd1c5' },
  { key: 'apn', paramKey: 'include_apn', label: 'APN', color: '#f6ad55' },
  { key: 'zemzev', paramKey: 'include_zemzev', label: 'ZEM-ZEV', color: '#b794f4' },
];

/** Result of one engagement run, reduced to the trade-study metrics. */
interface RunMetrics {
  nav_constant: number;
  miss_distance_m: number;
  hit: boolean;
  control_effort: number;
  veh_x: number[];
  veh_z: number[];
}

/**
 * Trapezoidal integral of |accel command| over the series → control effort.
 * Prefers the dedicated accel-command channel (accel_cmd_{x,y,z}); falls back
 * to the numerically differentiated vehicle velocity (|dV/dt|) when the command
 * channel is absent or all-zero (e.g. unguided / older artifacts).
 */
function controlEffort(series: SimResult['series']): number {
  const t = series.t;
  if (!t || t.length < 2) return 0;

  const ax = series.accel_cmd_x;
  const ay = series.accel_cmd_y;
  const az = series.accel_cmd_z;
  const hasCmd =
    Array.isArray(ax) &&
    Array.isArray(ay) &&
    Array.isArray(az) &&
    (ax.some((v) => v !== 0) || ay.some((v) => v !== 0) || az.some((v) => v !== 0));

  const mag = new Array<number>(t.length);
  if (hasCmd && ax && ay && az) {
    for (let i = 0; i < t.length; i++) mag[i] = Math.hypot(ax[i], ay[i], az[i]);
  } else {
    // Fallback: |a| ≈ |dV/dt| from the vehicle velocity channels.
    const vx = series.veh_vx;
    const vy = series.veh_vy;
    const vz = series.veh_vz;
    if (!vx || !vy || !vz) return 0;
    for (let i = 0; i < t.length; i++) {
      const j0 = Math.max(0, i - 1);
      const j1 = Math.min(t.length - 1, i + 1);
      const dt = t[j1] - t[j0];
      if (dt <= 0) {
        mag[i] = 0;
        continue;
      }
      mag[i] = Math.hypot(
        (vx[j1] - vx[j0]) / dt,
        (vy[j1] - vy[j0]) / dt,
        (vz[j1] - vz[j0]) / dt,
      );
    }
  }

  let integral = 0;
  for (let i = 1; i < t.length; i++) {
    const dt = t[i] - t[i - 1];
    integral += 0.5 * (mag[i] + mag[i - 1]) * dt;
  }
  return integral;
}

/** Build a per-run SimConfig from the base preset + studio params. */
function buildConfig(
  base: SimConfig,
  law: string,
  nav_constant: number,
  values: ParamValues,
): SimConfig {
  const maneuver_g = Number(values.maneuver_g);
  const maneuver_freq_hz = Number(values.maneuver_freq_hz);
  const time_constant_s = Number(values.time_constant_s);
  const launch_speed_mps = Number(values.launch_speed_mps);

  // A maneuvering target uses the core's sinusoidal "weave"; otherwise leave it
  // ballistic ("constant"). maneuver_g / maneuver_freq are contract keys.
  const maneuvering = maneuver_g > 0 && maneuver_freq_hz > 0;

  return {
    ...base,
    guidance: {
      ...base.guidance,
      law,
      nav_constant,
      // `time_constant` is a contract key (seconds); pass it through verbatim.
      time_constant: time_constant_s,
    } as SimConfig['guidance'],
    vehicle: {
      ...base.vehicle,
      launch_speed: launch_speed_mps,
    },
    target: {
      ...base.target,
      maneuver: maneuvering ? 'weave' : 'constant',
      maneuver_g: maneuvering ? maneuver_g : 0,
      maneuver_freq: maneuvering ? maneuver_freq_hz : 0,
    },
  };
}

/** The mock-mode note, rendered as a single annotated (empty) plot. */
function mockNotePlots(): PlotSpec[] {
  return [
    {
      id: 'mock-note',
      title: 'Guidance trade study — needs the live WASM build',
      data: [],
      layout: {
        xaxis: { visible: false },
        yaxis: { visible: false },
        annotations: [
          {
            text:
              'Running in <b>mock mode</b> (no WASM artifact present).<br>' +
              'Every engagement returns the same committed sample, so the law /<br>' +
              'N sweep is meaningless. Build the WASM core (scripts/build-wasm.sh)<br>' +
              'for a live guidance trade study.',
            showarrow: false,
            font: { size: 14 },
            xref: 'paper',
            yref: 'paper',
            x: 0.5,
            y: 0.5,
            align: 'center',
          },
        ],
      },
    },
  ];
}

export const guidanceStudio: StudioDef = {
  id: 'guidance-trade',
  label: 'Guidance Studio (law trade study)',
  description:
    'Engine-backed guidance-law trade study. Sweeps the navigation gain N across ' +
    'PN / APN / ZEM-ZEV on a homing engagement and aggregates miss distance, P(hit), ' +
    'and control effort (∫|a_cmd| dt). Requires the live WASM build — mock mode shows a note.',
  params: [
    {
      kind: 'boolean',
      key: 'include_pn',
      label: 'Include PN (pronav)',
      default_value: true,
    },
    {
      kind: 'boolean',
      key: 'include_apn',
      label: 'Include APN',
      default_value: true,
    },
    {
      kind: 'boolean',
      key: 'include_zemzev',
      label: 'Include ZEM-ZEV',
      default_value: false,
    },
    {
      kind: 'number',
      key: 'nav_constant',
      label: 'Chosen nav constant N',
      min_value: 2.5,
      max_value: 5,
      step: 0.5,
      default_value: 4,
      help: 'The N highlighted on the scatter / used for the trajectory overlay (swept across the grid for the line plot).',
    },
    {
      kind: 'number',
      key: 'maneuver_g',
      label: 'Target maneuver',
      unit: 'g',
      min_value: 0,
      max_value: 9,
      step: 1,
      default_value: 0,
      help: 'Target weave amplitude in g. 0 = ballistic (constant-velocity) target.',
    },
    {
      kind: 'number',
      key: 'maneuver_freq_hz',
      label: 'Target maneuver frequency',
      unit: 'Hz',
      min_value: 0,
      max_value: 2,
      step: 0.1,
      default_value: 0.5,
      help: 'Weave frequency. Only active when maneuver g > 0.',
    },
    {
      kind: 'number',
      key: 'time_constant_s',
      label: 'Guidance time constant',
      unit: 's',
      min_value: 0,
      max_value: 1,
      step: 0.05,
      default_value: 0,
      help: 'First-order guidance lag (0 = instantaneous response).',
    },
    {
      kind: 'number',
      key: 'launch_speed_mps',
      label: 'Launch speed',
      unit: 'm/s',
      min_value: 400,
      max_value: 1400,
      step: 50,
      default_value: 900,
    },
  ],
  presets: [
    {
      id: 'ballistic-target',
      label: 'Ballistic target',
      values: {
        include_pn: true,
        include_apn: true,
        include_zemzev: false,
        nav_constant: 4,
        maneuver_g: 0,
        maneuver_freq_hz: 0.5,
        time_constant_s: 0,
        launch_speed_mps: 900,
      },
    },
    {
      id: 'hard-maneuver',
      label: 'Hard-maneuvering target',
      values: {
        include_pn: true,
        include_apn: true,
        include_zemzev: true,
        nav_constant: 4,
        maneuver_g: 7,
        maneuver_freq_hz: 0.5,
        time_constant_s: 0.1,
        launch_speed_mps: 1000,
      },
    },
  ],

  async compute(values: ParamValues): Promise<PlotSpec[]> {
    const enabledLaws = LAWS.filter((l) => Boolean(values[l.paramKey]));

    // Ensure the WASM module has resolved so isMockMode() is meaningful, then
    // short-circuit with a clear note if we're in mock mode (every run identical).
    if (!isResolved()) {
      // A no-op run resolves the module (and is cheap; result is discarded).
      try {
        const probe = await loadPresetConfig(BASE_PRESET_FILE);
        await runSim(probe);
      } catch {
        /* fall through — handled below */
      }
    }
    if (isMockMode()) {
      return mockNotePlots();
    }

    if (enabledLaws.length === 0) {
      return [
        {
          id: 'no-law',
          title: 'No guidance law selected',
          data: [],
          layout: {
            xaxis: { visible: false },
            yaxis: { visible: false },
            annotations: [
              {
                text: 'Enable at least one guidance law (PN / APN / ZEM-ZEV).',
                showarrow: false,
                font: { size: 14 },
                xref: 'paper',
                yref: 'paper',
                x: 0.5,
                y: 0.5,
              },
            ],
          },
        },
      ];
    }

    const base = await loadPresetConfig(BASE_PRESET_FILE);
    const chosenN = Number(values.nav_constant);

    // Run the sweep: one engagement per (law, N). Sequential to keep the WASM
    // module (single instance) from being hammered re-entrantly.
    const perLaw = new Map<string, RunMetrics[]>();
    for (const law of enabledLaws) {
      const runs: RunMetrics[] = [];
      for (const N of N_SWEEP) {
        const cfg = buildConfig(base, law.key, N, values);
        const result = await runSim(cfg);
        const s = result.series;
        runs.push({
          nav_constant: N,
          miss_distance_m: result.miss_distance,
          hit: result.miss_distance < LETHAL_RADIUS_M,
          control_effort: controlEffort(s),
          veh_x: s.veh_x ?? [],
          veh_z: s.veh_z ?? [],
        });
      }
      perLaw.set(law.key, runs);
    }

    // For the trajectory overlay, run each law once at the chosen N (or reuse a
    // sweep point if it coincides with the grid).
    const trajByLaw = new Map<string, { veh_x: number[]; veh_z: number[] }>();
    for (const law of enabledLaws) {
      const runs = perLaw.get(law.key)!;
      const coincident = runs.find((r) => Math.abs(r.nav_constant - chosenN) < 1e-9);
      if (coincident) {
        trajByLaw.set(law.key, { veh_x: coincident.veh_x, veh_z: coincident.veh_z });
      } else {
        const cfg = buildConfig(base, law.key, chosenN, values);
        const result = await runSim(cfg);
        trajByLaw.set(law.key, {
          veh_x: result.series.veh_x ?? [],
          veh_z: result.series.veh_z ?? [],
        });
      }
    }

    // ---- Plot 1: miss vs control-effort scatter (chosen N marked) ----------
    const scatterData: PlotSpec['data'] = [];
    for (const law of enabledLaws) {
      const runs = perLaw.get(law.key)!;
      const pHit =
        runs.reduce((acc, r) => acc + (r.hit ? 1 : 0), 0) / Math.max(1, runs.length);
      scatterData.push({
        x: runs.map((r) => r.control_effort),
        y: runs.map((r) => r.miss_distance_m),
        type: 'scatter',
        mode: 'lines+markers',
        name: `${law.label}  (P(hit)=${pHit.toFixed(2)})`,
        line: { color: law.color, width: 1 },
        marker: { color: law.color, size: 7 },
        text: runs.map((r) => `N=${r.nav_constant}`),
        hovertemplate: '%{text}<br>effort=%{x:.1f}<br>miss=%{y:.2f} m<extra></extra>',
      });
      // Highlight the chosen N.
      const chosen = runs.find((r) => Math.abs(r.nav_constant - chosenN) < 1e-9);
      if (chosen) {
        scatterData.push({
          x: [chosen.control_effort],
          y: [chosen.miss_distance_m],
          type: 'scatter',
          mode: 'markers',
          name: `${law.label} @ N=${chosenN}`,
          marker: { color: law.color, size: 14, symbol: 'star', line: { color: '#fff', width: 1 } },
          showlegend: false,
          hovertemplate: `${law.label} N=${chosenN}<br>effort=%{x:.1f}<br>miss=%{y:.2f} m<extra></extra>`,
        });
      }
    }

    // ---- Plot 2: miss vs N line per law ------------------------------------
    const missVsNData: PlotSpec['data'] = enabledLaws.map((law) => {
      const runs = perLaw.get(law.key)!;
      return {
        x: runs.map((r) => r.nav_constant),
        y: runs.map((r) => r.miss_distance_m),
        type: 'scatter',
        mode: 'lines+markers',
        name: law.label,
        line: { color: law.color, width: 2 },
        marker: { color: law.color, size: 6 },
      };
    });

    // ---- Plot 3: trajectory overlay (vehicle path per law @ chosen N) ------
    const trajData: PlotSpec['data'] = [];
    for (const law of enabledLaws) {
      const traj = trajByLaw.get(law.key);
      if (!traj || traj.veh_x.length === 0) continue;
      trajData.push({
        x: traj.veh_x,
        y: traj.veh_z,
        type: 'scatter',
        mode: 'lines',
        name: law.label,
        line: { color: law.color, width: 2 },
      });
    }
    // Target path (from the chosen-N PN run, or the first enabled law) for context.
    // Re-run isn't needed: overlay just the vehicle paths per the spec.

    return [
      {
        id: 'miss-vs-effort',
        title: 'Miss distance vs control effort (chosen N starred)',
        data: scatterData,
        layout: {
          xaxis: { title: { text: 'Control effort  ∫|a_cmd| dt  [m/s]' } },
          yaxis: { title: { text: 'Miss distance [m]' } },
        },
      },
      {
        id: 'miss-vs-n',
        title: 'Miss distance vs navigation gain N',
        data: missVsNData,
        layout: {
          xaxis: { title: { text: 'Navigation gain N [—]' } },
          yaxis: { title: { text: 'Miss distance [m]' } },
        },
      },
      {
        id: 'trajectory-overlay',
        title: `Vehicle trajectory overlay (N = ${chosenN})`,
        data: trajData,
        layout: {
          xaxis: { title: { text: 'East [m]' } },
          yaxis: { title: { text: 'Up [m]' }, scaleanchor: 'x', scaleratio: 1 },
        },
      },
    ];
  },
};
