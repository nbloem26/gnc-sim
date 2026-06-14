'use client';

/**
 * A/B run-compare view. Pick two scenario presets, run both (client-side WASM,
 * or the committed sample in mock mode), and overlay their trajectories, a
 * selectable time-series channel, and key metrics (miss distance, intercept
 * time, P(hit-ish status), launch time) side-by-side.
 *
 * This lets a reviewer compare two guidance laws (e.g. ProNav vs APN) or two
 * sensor/track configurations in the browser and read both miss distances.
 *
 * Surfaces the newer channels where present: EKF NIS (nav_nis), fused-track
 * error magnitude (|track - tgt|), and decoy-discrimination margin.
 */

import { useEffect, useMemo, useState } from 'react';
import type { Data } from 'plotly.js-dist-min';
import type { ScenarioPreset, SimConfig, SimResult } from '@/lib/types';
import { loadPresetManifest, loadPresetConfig } from '@/lib/presets';
import { runSim } from '@/lib/wasmRunner';
import Plot from './Plot';

const A_COLOR = '#4fd1c5';
const B_COLOR = '#f6ad55';

interface Slot {
  preset: ScenarioPreset;
  config: SimConfig;
  result: SimResult;
}

function magnitude(x?: number[], y?: number[], z?: number[]): number[] {
  if (!x || !y || !z) return [];
  const n = Math.min(x.length, y.length, z.length);
  const out = new Array<number>(n);
  for (let i = 0; i < n; i++) out[i] = Math.hypot(x[i], y[i], z[i]);
  return out;
}

/** Fused-track error magnitude |track - tgt| each step (0/empty when no tracker). */
function trackError(s: SimResult['series']): number[] {
  if (!s.track_x || !s.track_y || !s.track_z) return [];
  const n = Math.min(s.t.length, s.track_x.length, s.tgt_x.length);
  const out = new Array<number>(n);
  for (let i = 0; i < n; i++) {
    out[i] = Math.hypot(
      s.track_x[i] - s.tgt_x[i],
      s.track_y[i] - s.tgt_y[i],
      s.track_z[i] - s.tgt_z[i],
    );
  }
  return out;
}

type Channel = {
  id: string;
  label: string;
  unit: string;
  /** Returns y-series for a result, or [] when the channel is inert for it. */
  get: (r: SimResult) => number[];
};

const CHANNELS: Channel[] = [
  { id: 'range', label: 'Range to target', unit: 'm', get: (r) => r.series.range },
  { id: 'los_rate', label: 'LOS rate', unit: 'rad/s', get: (r) => r.series.los_rate },
  { id: 'v_closing', label: 'Closing velocity', unit: 'm/s', get: (r) => r.series.v_closing },
  {
    id: 'accel_cmd',
    label: 'Accel command |a|',
    unit: 'm/s²',
    get: (r) =>
      magnitude(r.series.accel_cmd_x, r.series.accel_cmd_y, r.series.accel_cmd_z),
  },
  {
    id: 'nav_nis',
    label: 'EKF nav NIS',
    unit: '—',
    get: (r) =>
      Array.isArray(r.series.nav_nis) && r.series.nav_nis.some((v) => v !== 0)
        ? r.series.nav_nis
        : [],
  },
  {
    id: 'track_err',
    label: 'Fused track error',
    unit: 'm',
    get: (r) => {
      const e = trackError(r.series);
      return e.some((v) => v !== 0) ? e : [];
    },
  },
  {
    id: 'discrim_margin',
    label: 'Discrimination margin',
    unit: '—',
    get: (r) =>
      Array.isArray(r.series.discrim_margin) &&
      r.series.discrim_margin.some((v) => v !== 0)
        ? r.series.discrim_margin
        : [],
  },
];

export default function ComparePanel() {
  const [presets, setPresets] = useState<ScenarioPreset[]>([]);
  const [idA, setIdA] = useState('');
  const [idB, setIdB] = useState('');
  const [slotA, setSlotA] = useState<Slot | null>(null);
  const [slotB, setSlotB] = useState<Slot | null>(null);
  const [running, setRunning] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [channelId, setChannelId] = useState('range');

  useEffect(() => {
    let alive = true;
    loadPresetManifest()
      .then((p) => {
        if (!alive) return;
        setPresets(p);
        // Sensible default pairing: ProNav vs APN Monte-Carlo if available,
        // else the first two presets.
        const a = p.find((x) => x.id === 'montecarlo') ?? p[0];
        const b = p.find((x) => x.id === 'montecarlo_apn') ?? p[1] ?? p[0];
        if (a) setIdA(a.id);
        if (b) setIdB(b.id);
      })
      .catch(() => {
        if (alive) setError('Scenario presets are unavailable.');
      });
    return () => {
      alive = false;
    };
  }, []);

  async function runOne(id: string): Promise<Slot | null> {
    const preset = presets.find((p) => p.id === id);
    if (!preset) return null;
    const config = await loadPresetConfig(preset.file);
    const result = await runSim(config);
    return { preset, config, result };
  }

  async function compare() {
    if (!idA || !idB) return;
    setRunning(true);
    setError(null);
    try {
      const a = await runOne(idA);
      const b = await runOne(idB);
      setSlotA(a);
      setSlotB(b);
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    } finally {
      setRunning(false);
    }
  }

  const trajData = useMemo<Data[]>(() => {
    const traces: Data[] = [];
    const add = (slot: Slot | null, color: string, tag: 'A' | 'B') => {
      if (!slot) return;
      const s = slot.result.series;
      const last = s.t.length - 1;
      traces.push({
        x: s.veh_x,
        y: s.veh_z,
        type: 'scatter',
        mode: 'lines',
        name: `${tag}: vehicle`,
        line: { color, width: 2 },
      });
      traces.push({
        x: s.tgt_x,
        y: s.tgt_z,
        type: 'scatter',
        mode: 'lines',
        name: `${tag}: target`,
        line: { color, width: 1.5, dash: 'dot' },
        opacity: 0.7,
      });
      traces.push({
        x: [s.veh_x[last]],
        y: [s.veh_z[last]],
        type: 'scatter',
        mode: 'markers',
        name: `${tag}: ${slot.result.intercept ? 'intercept' : 'CPA'}`,
        marker: { color, size: 11, symbol: 'x' },
      });
    };
    add(slotA, A_COLOR, 'A');
    add(slotB, B_COLOR, 'B');
    return traces;
  }, [slotA, slotB]);

  const channel = CHANNELS.find((c) => c.id === channelId) ?? CHANNELS[0];

  const channelData = useMemo<Data[]>(() => {
    const traces: Data[] = [];
    const add = (slot: Slot | null, color: string, tag: 'A' | 'B') => {
      if (!slot) return;
      const y = channel.get(slot.result);
      if (y.length === 0) return;
      traces.push({
        x: slot.result.series.t.slice(0, y.length),
        y,
        type: 'scatter',
        mode: 'lines',
        name: `${tag}: ${slot.preset.label}`,
        line: { color, width: 2 },
      });
    };
    add(slotA, A_COLOR, 'A');
    add(slotB, B_COLOR, 'B');
    return traces;
  }, [slotA, slotB, channel]);

  const channelEmpty = channelData.length === 0;

  return (
    <div>
      <h3 style={{ margin: '0 0 4px' }}>Compare (A/B)</h3>
      <p className="muted" style={{ marginTop: 0 }}>
        Run two scenario presets and overlay their trajectories, metrics, and a
        time-series channel. Compare e.g. ProNav vs APN guidance, or fused vs
        single-sensor tracking.
      </p>

      {error ? (
        <div className="card" style={{ borderColor: 'var(--err)' }}>
          <strong style={{ color: 'var(--err)' }}>Compare error:</strong> {error}
        </div>
      ) : null}

      <div className="row2">
        <label className="field">
          <span>
            <span style={{ color: A_COLOR }}>A</span> — preset
          </span>
          <select value={idA} onChange={(e) => setIdA(e.target.value)} disabled={running}>
            {presets.map((p) => (
              <option key={p.id} value={p.id}>
                {p.label}
              </option>
            ))}
          </select>
        </label>
        <label className="field">
          <span>
            <span style={{ color: B_COLOR }}>B</span> — preset
          </span>
          <select value={idB} onChange={(e) => setIdB(e.target.value)} disabled={running}>
            {presets.map((p) => (
              <option key={p.id} value={p.id}>
                {p.label}
              </option>
            ))}
          </select>
        </label>
      </div>

      <div className="actions">
        <button
          type="button"
          className="primary"
          onClick={compare}
          disabled={running || !idA || !idB || presets.length === 0}
        >
          {running ? 'Running…' : 'Run A/B'}
        </button>
      </div>

      {slotA || slotB ? (
        <table className="cmpTable">
          <thead>
            <tr>
              <th>Metric</th>
              <th style={{ color: A_COLOR }}>A: {slotA?.preset.label ?? '—'}</th>
              <th style={{ color: B_COLOR }}>B: {slotB?.preset.label ?? '—'}</th>
            </tr>
          </thead>
          <tbody>
            <tr>
              <td>Status</td>
              <td>{slotA ? (slotA.result.intercept ? 'INTERCEPT' : 'MISS') : '—'}</td>
              <td>{slotB ? (slotB.result.intercept ? 'INTERCEPT' : 'MISS') : '—'}</td>
            </tr>
            <tr>
              <td>Miss distance [m]</td>
              <td>{slotA ? slotA.result.miss_distance.toFixed(2) : '—'}</td>
              <td>{slotB ? slotB.result.miss_distance.toFixed(2) : '—'}</td>
            </tr>
            <tr>
              <td>Intercept time [s]</td>
              <td>
                {slotA && Number.isFinite(slotA.result.intercept_time)
                  ? slotA.result.intercept_time.toFixed(2)
                  : '—'}
              </td>
              <td>
                {slotB && Number.isFinite(slotB.result.intercept_time)
                  ? slotB.result.intercept_time.toFixed(2)
                  : '—'}
              </td>
            </tr>
            <tr>
              <td>Launch time [s]</td>
              <td>{slotA ? (slotA.result.launch_time ?? 0).toFixed(2) : '—'}</td>
              <td>{slotB ? (slotB.result.launch_time ?? 0).toFixed(2) : '—'}</td>
            </tr>
          </tbody>
        </table>
      ) : null}

      {trajData.length > 0 ? (
        <Plot
          data={trajData}
          layout={{
            title: { text: 'Trajectories (East vs Up)', font: { size: 13 } },
            xaxis: { title: { text: 'East [m]' } },
            yaxis: { title: { text: 'Up [m]' }, scaleanchor: 'x', scaleratio: 1 },
          }}
        />
      ) : null}

      {slotA || slotB ? (
        <>
          <label className="field" style={{ maxWidth: 280 }}>
            <span>Time-series channel</span>
            <select value={channelId} onChange={(e) => setChannelId(e.target.value)}>
              {CHANNELS.map((c) => (
                <option key={c.id} value={c.id}>
                  {c.label}
                </option>
              ))}
            </select>
          </label>
          {channelEmpty ? (
            <div className="placeholder" style={{ marginTop: 8 }}>
              Neither run emits a non-zero <code>{channel.label}</code> channel.
            </div>
          ) : (
            <Plot
              data={channelData}
              layout={{
                title: { text: channel.label, font: { size: 13 } },
                xaxis: { title: { text: 'Time [s]' } },
                yaxis: { title: { text: channel.unit } },
              }}
            />
          )}
        </>
      ) : (
        <p className="muted">Pick two presets and run A/B to compare.</p>
      )}
    </div>
  );
}
