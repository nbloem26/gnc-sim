/**
 * Threat Studio — trajectory & signature design (issue #109).
 *
 * Engine-backed: it builds a *threat* SimConfig (ICBM / HGV / RV+penaids),
 * `await runSim(cfg)`, and plots the resulting **tgt_*** series — the threat
 * trajectory the rest of the sim tracks/engages. (In a homing run the `tgt_*`
 * channels are the target; with the threat `maneuver` set, that target follows
 * the boosting-ICBM / skip-glide / RV-deploy dynamics in core's Threats.cpp.)
 *
 * Coordinates: the core stays in local ENU metres (DATA_CONTRACT §6),
 * x=East, y=North, z=Up. So:
 *   - altitude   = tgt_z
 *   - downrange  = horizontal great-circle-ish distance from origin = √(x²+y²)
 *   - ground track via ENU→geodetic (enuToLatLonAlt), or raw ENU if you prefer.
 *
 * The web contract does NOT expose per-object (penaid) channels or geodetic
 * tgt_* channels, so:
 *   - the ground track is derived by projecting ENU→geodetic on the client;
 *   - the RV "penaid fan" cannot be drawn from a single tgt_* trace — we mark
 *     the deploy event on the trajectory and note the limitation. (See caveats.)
 *
 * Params map onto the `target.{icbm,hgv,rv}` config blocks of the canonical
 * threat presets (configs/threat_*.json), which are embedded below so the
 * studio is self-contained (they are not in the /scenarios manifest). Where a
 * single knob has to stand in for a richer block we scale the closest field and
 * say so in the param `help`.
 */

import type { StudioDef, ParamValues, PlotSpec } from '../types';
import type { SimConfig, SimResult } from '@/lib/types';
import { runSim, isMockMode } from '@/lib/wasmRunner';
import { enuToLatLonAlt } from '@/lib/enuToGeodetic';

// ----------------------------------------------------------------------------
// Embedded base configs (byte-for-byte from configs/threat_*.json). Each
// compute() deep-clones the matching base, applies the params, then runSim().
// ----------------------------------------------------------------------------

const BASE_ICBM = {
  schema_version: 1,
  scenario: 'homing',
  model: '3dof',
  seed: 1,
  dt: 0.01,
  t_end: 40.0,
  integrator: 'rk4',
  origin: { lat0_deg: 28.4889, lon0_deg: -80.5778, alt0_m: 0.0 },
  env: { g0: 9.80665, altitude_dependent_g: false, atmosphere: false },
  aero: { ref_area: 0.02, cd0: 0.3 },
  vehicle: { pos0: [0, 0, 0], launch_speed: 2200, launch_elevation_deg: 68, launch_azimuth_deg: 0, mass0: 60.0 },
  guidance: { law: 'pronav', nav_constant: 4.0, max_accel: 1500.0 },
  sensors: { enable: false },
  target: {
    pos0: [70000, 0, 35000],
    vel0: [-1600, 0, 250],
    maneuver: 'icbm',
    icbm: {
      payload_mass_kg: 600.0,
      stages: [
        { thrust_n: 520000.0, burn_time_s: 60.0, propellant_mass_kg: 11000.0, dry_mass_kg: 1500.0 },
        { thrust_n: 130000.0, burn_time_s: 70.0, propellant_mass_kg: 3200.0, dry_mass_kg: 600.0 },
      ],
    },
  },
} as const;

const BASE_HGV = {
  schema_version: 1,
  scenario: 'homing',
  model: '3dof',
  seed: 1,
  dt: 0.01,
  t_end: 60.0,
  integrator: 'rk4',
  origin: { lat0_deg: 28.4889, lon0_deg: -80.5778, alt0_m: 0.0 },
  env: { g0: 9.80665, altitude_dependent_g: false, atmosphere: false },
  aero: { ref_area: 0.02, cd0: 0.3 },
  vehicle: { pos0: [200000, 0, 0], launch_speed: 1800, launch_elevation_deg: 75, launch_azimuth_deg: 180, mass0: 60.0 },
  guidance: { law: 'pronav', nav_constant: 4.0, max_accel: 1500.0 },
  sensors: { enable: false },
  target: {
    pos0: [0, 0, 55000],
    vel0: [6500, 0, -600],
    maneuver: 'hgv',
    hgv: {
      ld_ratio: 2.5,
      ballistic_coeff: 6000.0,
      rho0_kgpm3: 1.225,
      scale_height_m: 7200.0,
      pull_up_alt_m: 60000.0,
    },
  },
} as const;

const BASE_RV = {
  schema_version: 1,
  scenario: 'homing',
  model: '3dof',
  seed: 1,
  dt: 0.01,
  t_end: 40.0,
  integrator: 'rk4',
  origin: { lat0_deg: 28.4889, lon0_deg: -80.5778, alt0_m: 0.0 },
  env: { g0: 9.80665, altitude_dependent_g: false, atmosphere: false },
  aero: { ref_area: 0.02, cd0: 0.3 },
  vehicle: { pos0: [0, 0, 0], launch_speed: 2000, launch_elevation_deg: 72, launch_azimuth_deg: 0, mass0: 60.0 },
  guidance: { law: 'pronav', nav_constant: 4.0, max_accel: 1500.0 },
  sensors: { enable: false },
  target: {
    pos0: [55000, 0, 60000],
    vel0: [-1500, 0, -1200],
    maneuver: 'rv_penaids',
    rv: {
      penaid_count: 6,
      deploy_time_s: 2.0,
      deploy_dv_mps: 8.0,
      penaid_decel_mps2: 1.0,
    },
  },
} as const;

// ----------------------------------------------------------------------------
// Param keys (shared across the three threat types; only the relevant ones are
// honored per type — the others are inert).
// ----------------------------------------------------------------------------

const COLORS = {
  trace: '#4fd1c5',
  accent: '#f6ad55',
  marker: '#fc8181',
  muted: 'rgba(160,174,192,0.6)',
};

/** Structural-clone helper (configs are plain JSON, no Dates/functions). */
function clone<T>(obj: T): T {
  return JSON.parse(JSON.stringify(obj)) as T;
}

/** Magnitude of an ENU velocity vector at index i. */
function speedAt(s: SimResult['series'], i: number): number {
  return Math.hypot(s.tgt_vx[i], s.tgt_vy[i], s.tgt_vz[i]);
}

/**
 * Build the SimConfig for the chosen threat type from the embedded base preset,
 * applying the studio params onto the matching `target.*` block.
 */
function buildConfig(params: ParamValues): { cfg: SimConfig; threat: string } {
  const threat = String(params.threat_type);

  if (threat === 'icbm') {
    const cfg = clone(BASE_ICBM) as unknown as SimConfig;
    const tgt = (cfg.target as unknown as Record<string, unknown>);
    const icbm = tgt.icbm as {
      payload_mass_kg: number;
      stages: { thrust_n: number; burn_time_s: number; propellant_mass_kg: number; dry_mass_kg: number }[];
    };
    // Map: loft angle -> the threat's initial flight-path (we encode it on the
    // velocity vector vel0 so the loft profile actually changes). Speed kept.
    const loft_deg = Number(params.icbm_loft_deg);
    const v0 = Math.hypot(BASE_ICBM.target.vel0[0], BASE_ICBM.target.vel0[2]);
    // vel0 x is downrange-ish (negative = inbound), z is up. Preserve sign of x.
    const sign = BASE_ICBM.target.vel0[0] < 0 ? -1 : 1;
    tgt.vel0 = [sign * v0 * Math.cos((loft_deg * Math.PI) / 180), 0, v0 * Math.sin((loft_deg * Math.PI) / 180)];
    // Map: a single "boost scale" knob scales every stage's burn time AND
    // propellant together (more total impulse -> higher apogee / longer range).
    const boost = Number(params.icbm_boost_scale);
    icbm.stages = icbm.stages.map((s) => ({
      ...s,
      burn_time_s: s.burn_time_s * boost,
      propellant_mass_kg: s.propellant_mass_kg * boost,
    }));
    icbm.payload_mass_kg = Number(params.icbm_payload_mass_kg);
    cfg.t_end = Number(params.sim_duration_s);
    return { cfg, threat };
  }

  if (threat === 'hgv') {
    const cfg = clone(BASE_HGV) as unknown as SimConfig;
    const tgt = cfg.target as unknown as Record<string, unknown>;
    const hgv = tgt.hgv as { ld_ratio: number; pull_up_alt_m: number; ballistic_coeff: number };
    hgv.ld_ratio = Number(params.hgv_ld_ratio);
    hgv.pull_up_alt_m = Number(params.hgv_pull_up_alt_m);
    // Entry altitude maps onto the threat's initial Up position (tgt z).
    const entry_alt_m = Number(params.hgv_entry_alt_m);
    (tgt.pos0 as number[])[2] = entry_alt_m;
    cfg.t_end = Number(params.sim_duration_s);
    return { cfg, threat };
  }

  // RV + penaids.
  const cfg = clone(BASE_RV) as unknown as SimConfig;
  const tgt = cfg.target as unknown as Record<string, unknown>;
  const rv = tgt.rv as { penaid_count: number; deploy_time_s: number; deploy_dv_mps: number };
  rv.penaid_count = Math.round(Number(params.rv_penaid_count));
  rv.deploy_time_s = Number(params.rv_deploy_time_s);
  rv.deploy_dv_mps = Number(params.rv_deploy_dv_mps);
  cfg.t_end = Number(params.sim_duration_s);
  return { cfg, threat };
}

/** A small note plot used in mock mode / when a feature is unavailable. */
function notePlot(id: string, title: string, lines: string[]): PlotSpec {
  return {
    id,
    title,
    data: [{ x: [0], y: [0], type: 'scatter', mode: 'markers', marker: { opacity: 0 }, showlegend: false }],
    layout: {
      annotations: [
        {
          text: lines.join('<br>'),
          showarrow: false,
          font: { color: '#cbd5e0', size: 13 },
          x: 0.5,
          y: 0.5,
          xref: 'paper',
          yref: 'paper',
          align: 'left',
        },
      ],
      xaxis: { visible: false },
      yaxis: { visible: false },
    },
  };
}

export const threatStudio: StudioDef = {
  id: 'threat-studio',
  label: 'Threat Studio',
  description:
    'Design a threat trajectory & signature: pick ICBM / HGV / RV+penaids, tune the ' +
    'type-relevant knobs, and the engine runs the threat (tgt_* series) — altitude-vs-downrange ' +
    'loft / skip profile, 3-D ground track, and apogee/range (plus HGV skip count & RV deploy).',
  params: [
    {
      kind: 'enum',
      key: 'threat_type',
      label: 'Threat type',
      options: [
        { value: 'icbm', label: 'ICBM (boosting, lofted)' },
        { value: 'hgv', label: 'HGV (hypersonic skip-glide)' },
        { value: 'rv', label: 'RV + penaids' },
      ],
      default_value: 'icbm',
      help: 'Selects the target.maneuver block driven into the core.',
    },
    {
      kind: 'number',
      key: 'sim_duration_s',
      label: 'Sim duration',
      unit: 's',
      min_value: 10,
      max_value: 120,
      step: 5,
      default_value: 40,
      help: 'Flight time simulated (cfg.t_end).',
    },
    // --- ICBM knobs (target.icbm) ---
    {
      kind: 'number',
      key: 'icbm_loft_deg',
      label: 'ICBM loft angle',
      unit: 'deg',
      min_value: 30,
      max_value: 80,
      step: 1,
      default_value: 53,
      help: 'ICBM only — initial flight-path angle (encoded on target.vel0). Higher = steeper loft.',
    },
    {
      kind: 'number',
      key: 'icbm_boost_scale',
      label: 'ICBM boost scale',
      min_value: 0.5,
      max_value: 2.0,
      step: 0.05,
      default_value: 1.0,
      help: 'ICBM only — scales every stage burn_time_s + propellant_mass_kg together (total impulse).',
    },
    {
      kind: 'number',
      key: 'icbm_payload_mass_kg',
      label: 'ICBM payload mass',
      unit: 'kg',
      min_value: 200,
      max_value: 1500,
      step: 50,
      default_value: 600,
      help: 'ICBM only — target.icbm.payload_mass_kg (heavier = lower burnout speed).',
    },
    // --- HGV knobs (target.hgv) ---
    {
      kind: 'number',
      key: 'hgv_ld_ratio',
      label: 'HGV L/D ratio',
      min_value: 1.0,
      max_value: 4.0,
      step: 0.1,
      default_value: 2.5,
      help: 'HGV only — lift/drag of the glider. Higher L/D = more & higher skips.',
    },
    {
      kind: 'number',
      key: 'hgv_entry_alt_m',
      label: 'HGV entry altitude',
      unit: 'm',
      min_value: 30000,
      max_value: 90000,
      step: 1000,
      default_value: 55000,
      help: 'HGV only — initial Up position (target.pos0 z) where the glide begins.',
    },
    {
      kind: 'number',
      key: 'hgv_pull_up_alt_m',
      label: 'HGV pull-up altitude',
      unit: 'm',
      min_value: 30000,
      max_value: 80000,
      step: 1000,
      default_value: 60000,
      help: 'HGV only — altitude below which lift acts (target.hgv.pull_up_alt_m); sets the skip ceiling.',
    },
    // --- RV knobs (target.rv) ---
    {
      kind: 'number',
      key: 'rv_penaid_count',
      label: 'RV penaid count',
      min_value: 0,
      max_value: 20,
      step: 1,
      default_value: 6,
      help: 'RV only — number of penetration aids deployed (target.rv.penaid_count).',
    },
    {
      kind: 'number',
      key: 'rv_deploy_time_s',
      label: 'RV deploy time',
      unit: 's',
      min_value: 0,
      max_value: 20,
      step: 0.5,
      default_value: 2.0,
      help: 'RV only — when penaids deploy (target.rv.deploy_time_s). Marked on the trajectory.',
    },
    {
      kind: 'number',
      key: 'rv_deploy_dv_mps',
      label: 'RV deploy Δv',
      unit: 'm/s',
      min_value: 0,
      max_value: 30,
      step: 0.5,
      default_value: 8.0,
      help: 'RV only — separation Δv imparted to penaids (target.rv.deploy_dv_mps).',
    },
  ],
  presets: [
    {
      id: 'icbm_minimum_energy',
      label: 'ICBM — minimum energy',
      values: { threat_type: 'icbm', sim_duration_s: 40, icbm_loft_deg: 45, icbm_boost_scale: 1.0, icbm_payload_mass_kg: 600 },
    },
    {
      id: 'icbm_lofted',
      label: 'ICBM — lofted',
      values: { threat_type: 'icbm', sim_duration_s: 60, icbm_loft_deg: 70, icbm_boost_scale: 1.3, icbm_payload_mass_kg: 500 },
    },
    {
      id: 'hgv_aggressive',
      label: 'HGV — high L/D skip',
      values: { threat_type: 'hgv', sim_duration_s: 60, hgv_ld_ratio: 3.5, hgv_entry_alt_m: 60000, hgv_pull_up_alt_m: 65000 },
    },
    {
      id: 'rv_heavy_fan',
      label: 'RV — heavy penaid fan',
      values: { threat_type: 'rv', sim_duration_s: 40, rv_penaid_count: 12, rv_deploy_time_s: 3, rv_deploy_dv_mps: 15 },
    },
  ],
  async compute(params: ParamValues): Promise<PlotSpec[]> {
    const { cfg, threat } = buildConfig(params);
    const result = await runSim(cfg);
    const s = result.series;
    const t = s.t;
    const n = Math.min(t.length, s.tgt_x.length, s.tgt_y.length, s.tgt_z.length);

    // Threat trajectory in ENU.
    const east = s.tgt_x.slice(0, n);
    const north = s.tgt_y.slice(0, n);
    const up = s.tgt_z.slice(0, n);
    const tt = t.slice(0, n);

    // Downrange = horizontal distance from origin.
    const downrange_m = east.map((e, i) => Math.hypot(e, north[i]));
    const downrange_km = downrange_m.map((d) => d / 1000);
    const alt_km = up.map((z) => z / 1000);

    // Apogee & range readouts.
    let apogee_m = -Infinity;
    let apogeeIdx = 0;
    for (let i = 0; i < n; i++) {
      if (up[i] > apogee_m) {
        apogee_m = up[i];
        apogeeIdx = i;
      }
    }
    const totalRange_m = downrange_m[n - 1] ?? 0;
    const maxSpeed_mps = Math.max(...Array.from({ length: n }, (_, i) => speedAt(s, i)));

    const mock = isMockMode();
    const plots: PlotSpec[] = [];

    if (mock) {
      plots.push(
        notePlot('mock-note', 'Mock mode — no live threat trajectory', [
          '<b>WASM core not built — running in mock mode.</b>',
          'runSim() is serving the committed sample_result.json, so the',
          'tgt_* trajectory below does NOT reflect the chosen threat or params.',
          'Build the WASM artifact (scripts/build-wasm.sh) for live threat design.',
        ]),
      );
    }

    // ---- Plot 1: Altitude vs downrange (loft / skip profile) ----
    plots.push({
      id: 'alt-downrange',
      title: 'Altitude vs downrange — loft / skip profile' + (mock ? ' (mock data)' : ''),
      data: [
        {
          x: downrange_km,
          y: alt_km,
          type: 'scatter',
          mode: 'lines',
          name: 'threat',
          line: { color: COLORS.trace, width: 2 },
        },
        {
          x: [downrange_km[apogeeIdx]],
          y: [alt_km[apogeeIdx]],
          type: 'scatter',
          mode: 'text+markers',
          name: 'apogee',
          marker: { color: COLORS.accent, size: 9, symbol: 'diamond' },
          text: [`apogee ${(apogee_m / 1000).toFixed(1)} km`],
          textposition: 'top center',
          textfont: { color: COLORS.accent },
        },
      ],
      layout: {
        xaxis: { title: { text: 'downrange [km]' } },
        yaxis: { title: { text: 'altitude [km]' }, rangemode: 'tozero' },
      },
    });

    // ---- Plot 2: 3D trajectory (geodetic if available, else ENU) ----
    const origin = (cfg as unknown as { origin: { lat0_deg: number; lon0_deg: number; alt0_m: number } }).origin;
    const geo = east.map((e, i) =>
      enuToLatLonAlt(e, north[i], up[i], origin.lat0_deg, origin.lon0_deg, origin.alt0_m),
    );
    plots.push({
      id: 'ground-track-3d',
      title: '3-D ground track (ENU→geodetic projection)' + (mock ? ' (mock data)' : ''),
      data: [
        {
          x: geo.map((g) => g.lon_deg),
          y: geo.map((g) => g.lat_deg),
          z: alt_km,
          type: 'scatter3d',
          mode: 'lines',
          name: 'threat',
          line: { color: COLORS.trace, width: 4 },
        },
        {
          x: [geo[0].lon_deg, geo[n - 1].lon_deg],
          y: [geo[0].lat_deg, geo[n - 1].lat_deg],
          z: [alt_km[0], alt_km[n - 1]],
          type: 'scatter3d',
          mode: 'text+markers',
          name: 'endpoints',
          marker: { color: [COLORS.accent, COLORS.marker], size: 5 },
          text: ['start', 'end'],
          textposition: 'top center',
        },
      ],
      layout: {
        scene: {
          xaxis: { title: { text: 'lon [deg]' } },
          yaxis: { title: { text: 'lat [deg]' } },
          zaxis: { title: { text: 'altitude [km]' } },
        },
      },
    });

    // ---- Plot 3: readouts + per-type extra ----
    if (threat === 'hgv') {
      // Skip detection: count local maxima in altitude (apogees of each skip),
      // ignoring tiny ripples. Period = mean time between successive skip peaks.
      const peaks: number[] = [];
      const minProminence_m = 200;
      for (let i = 2; i < n - 2; i++) {
        if (up[i] > up[i - 1] && up[i] >= up[i + 1]) {
          // local max — require it to rise at least minProminence above the
          // preceding local trough to count as a real skip.
          let j = i - 1;
          while (j > 0 && up[j] <= up[j + 1]) j--;
          if (up[i] - up[j] > minProminence_m) peaks.push(i);
        }
      }
      const skipCount = peaks.length;
      let meanPeriod_s = NaN;
      if (peaks.length >= 2) {
        let acc = 0;
        for (let k = 1; k < peaks.length; k++) acc += tt[peaks[k]] - tt[peaks[k - 1]];
        meanPeriod_s = acc / (peaks.length - 1);
      }
      plots.push({
        id: 'hgv-skips',
        title: 'HGV altitude vs time — skip count & period',
        data: [
          {
            x: tt,
            y: alt_km,
            type: 'scatter',
            mode: 'lines',
            name: 'altitude',
            line: { color: COLORS.trace, width: 2 },
          },
          {
            x: peaks.map((i) => tt[i]),
            y: peaks.map((i) => alt_km[i]),
            type: 'scatter',
            mode: 'markers',
            name: 'skip apogees',
            marker: { color: COLORS.accent, size: 8, symbol: 'triangle-up' },
          },
        ],
        layout: {
          xaxis: { title: { text: 't [s]' } },
          yaxis: { title: { text: 'altitude [km]' }, rangemode: 'tozero' },
          annotations: [
            {
              text: `skips: <b>${skipCount}</b>` +
                (Number.isFinite(meanPeriod_s) ? `   ·   mean period: <b>${meanPeriod_s.toFixed(1)} s</b>` : ''),
              showarrow: false,
              x: 0.02,
              y: 0.98,
              xref: 'paper',
              yref: 'paper',
              align: 'left',
              font: { color: '#cbd5e0' },
            },
          ],
        },
      });
    } else if (threat === 'rv') {
      // The web data contract exposes only ONE tgt_* trace (the RV bus), not the
      // individual penaids, so we cannot draw the fan separation directly. We
      // mark the deploy event and note the limitation.
      const deploy_s = Number(params.rv_deploy_time_s);
      let deployIdx = 0;
      for (let i = 0; i < n; i++) {
        if (tt[i] >= deploy_s) {
          deployIdx = i;
          break;
        }
      }
      plots.push({
        id: 'rv-deploy',
        title: 'RV altitude vs time — penaid deploy event',
        data: [
          {
            x: tt,
            y: alt_km,
            type: 'scatter',
            mode: 'lines',
            name: 'RV bus',
            line: { color: COLORS.trace, width: 2 },
          },
          {
            x: [tt[deployIdx]],
            y: [alt_km[deployIdx]],
            type: 'scatter',
            mode: 'text+markers',
            name: 'penaid deploy',
            marker: { color: COLORS.marker, size: 10, symbol: 'x' },
            text: [`deploy ${Number(params.rv_penaid_count).toFixed(0)} penaids @ ${deploy_s.toFixed(1)} s`],
            textposition: 'top center',
            textfont: { color: COLORS.marker },
          },
        ],
        layout: {
          xaxis: { title: { text: 't [s]' } },
          yaxis: { title: { text: 'altitude [km]' }, rangemode: 'tozero' },
          annotations: [
            {
              text: 'Note: the web data contract exposes a single tgt_* trace ' +
                '(the RV bus),<br>not per-penaid channels — the fan separation ' +
                'cannot be plotted from this run.',
              showarrow: false,
              x: 0.5,
              y: 0.06,
              xref: 'paper',
              yref: 'paper',
              align: 'center',
              font: { color: COLORS.muted, size: 11 },
            },
          ],
        },
      });
    }

    // Readout summary (always last).
    plots.push(
      notePlot('readout', 'Apogee & range readout', [
        `<b>Threat:</b> ${threat.toUpperCase()}`,
        `<b>Apogee:</b> ${(apogee_m / 1000).toFixed(2)} km (at t = ${tt[apogeeIdx].toFixed(1)} s)`,
        `<b>Final downrange:</b> ${(totalRange_m / 1000).toFixed(2)} km`,
        `<b>Max speed:</b> ${maxSpeed_mps.toFixed(0)} m/s`,
        `<b>Flight time:</b> ${tt[n - 1].toFixed(1)} s`,
        mock ? '<i>(mock data — values are from sample_result.json)</i>' : '',
      ].filter(Boolean)),
    );

    return plots;
  },
};
