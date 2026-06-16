'use client';

/**
 * StudioShell — the generic, reusable layout every subsystem studio plugs into.
 *
 * Left: a parameter panel auto-rendered from the studio's typed `ParamSchema`
 * (numeric sliders + inputs with label/unit/range/step, boolean toggles, enum
 * selects), bound to a live params object. Right: a responsive grid of plots
 * produced by the studio's `compute(params)`, each rendered through the shared
 * <Plot> wrapper. A header shows the studio title + description.
 *
 * Re-compute is debounced on param change so dragging a slider doesn't fire a
 * storm of (possibly WASM-backed) computes. `compute` may be sync or async; the
 * shell always awaits it and ignores stale results.
 *
 * Lazy by construction: <Plot> imports Plotly only inside an effect, and the
 * shell itself is mounted lazily by the Studios area (which is a lazy
 * dynamic(ssr:false) import in page.tsx). So nothing heavy runs on first load.
 */

import { useEffect, useMemo, useRef, useState } from 'react';
import Plot from '@/components/Plot';
import type {
  StudioDef,
  ParamValues,
  ParamDescriptor,
  PlotSpec,
} from '@/lib/studio/types';
import { defaultParamValues } from '@/lib/studio/types';

/** Debounce window before a param change triggers a re-compute. */
const RECOMPUTE_DEBOUNCE_MS = 150;

export interface StudioShellProps {
  studio: StudioDef;
}

export default function StudioShell({ studio }: StudioShellProps) {
  // Fresh param values whenever the active studio changes.
  const [params, setParams] = useState<ParamValues>(() =>
    defaultParamValues(studio.params),
  );
  const [plots, setPlots] = useState<PlotSpec[]>([]);
  const [computing, setComputing] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Reset params when switching studios.
  useEffect(() => {
    setParams(defaultParamValues(studio.params));
    setPlots([]);
    setError(null);
  }, [studio]);

  // Debounced compute. A monotonically increasing token guards against
  // out-of-order async results (a slower earlier compute resolving last).
  const runToken = useRef(0);
  useEffect(() => {
    const token = ++runToken.current;
    const timer = setTimeout(async () => {
      setComputing(true);
      setError(null);
      try {
        const result = await studio.compute(params);
        if (runToken.current === token) setPlots(result);
      } catch (e) {
        if (runToken.current === token) {
          setError(e instanceof Error ? e.message : String(e));
        }
      } finally {
        if (runToken.current === token) setComputing(false);
      }
    }, RECOMPUTE_DEBOUNCE_MS);
    return () => clearTimeout(timer);
  }, [studio, params]);

  function setParam(key: string, value: number | boolean | string) {
    setParams((prev) => ({ ...prev, [key]: value }));
  }

  function applyPreset(values: ParamValues) {
    // Merge over current so a partial preset only overrides what it names.
    setParams((prev) => ({ ...prev, ...values }));
  }

  function resetParams() {
    setParams(defaultParamValues(studio.params));
  }

  return (
    <div className="studioShell">
      <form className="studioPanel" onSubmit={(e) => e.preventDefault()}>
        <h3 className="studioPanelTitle">Parameters</h3>

        {studio.presets && studio.presets.length > 0 ? (
          <div className="studioPresets">
            {studio.presets.map((p) => (
              <button
                key={p.id}
                type="button"
                className="studioPresetBtn"
                onClick={() => applyPreset(p.values)}
              >
                {p.label}
              </button>
            ))}
          </div>
        ) : null}

        {studio.params.map((descriptor) => (
          <ParamControl
            key={descriptor.key}
            descriptor={descriptor}
            value={params[descriptor.key]}
            onChange={(v) => setParam(descriptor.key, v)}
          />
        ))}

        <div className="actions">
          <button type="button" onClick={resetParams}>
            Reset defaults
          </button>
        </div>
      </form>

      <div className="studioMain">
        <div className="studioHeader">
          <h2 className="studioTitle">
            {studio.label}
            {computing ? <span className="studioComputing"> · computing…</span> : null}
          </h2>
          <p className="muted studioDesc">{studio.description}</p>
        </div>

        {error ? (
          <div className="card" style={{ borderColor: 'var(--err)' }}>
            <strong style={{ color: 'var(--err)' }}>Compute error:</strong> {error}
          </div>
        ) : null}

        <div className="studioGrid">
          {plots.length === 0 && !error ? (
            <div className="card placeholder">
              {computing ? 'Computing…' : 'Adjust parameters to generate plots.'}
            </div>
          ) : (
            plots.map((spec) => (
              <div className="card studioPlotCard" key={spec.id ?? spec.title}>
                <div className="studioPlotTitle">{spec.title}</div>
                <Plot data={spec.data} layout={spec.layout} />
              </div>
            ))
          )}
        </div>
      </div>
    </div>
  );
}

// ----------------------------------------------------------------------------
// One typed control, rendered from a descriptor.
// ----------------------------------------------------------------------------

interface ParamControlProps {
  descriptor: ParamDescriptor;
  value: number | boolean | string | undefined;
  onChange: (value: number | boolean | string) => void;
}

function ParamControl({ descriptor, value, onChange }: ParamControlProps) {
  if (descriptor.kind === 'boolean') {
    return (
      <label className="check">
        <input
          type="checkbox"
          checked={Boolean(value)}
          onChange={(e) => onChange(e.target.checked)}
        />
        <span>{descriptor.label}</span>
        {descriptor.help ? <ControlHelp text={descriptor.help} /> : null}
      </label>
    );
  }

  if (descriptor.kind === 'enum') {
    return (
      <label className="field">
        <span>{descriptor.label}</span>
        <select value={String(value)} onChange={(e) => onChange(e.target.value)}>
          {descriptor.options.map((o) => (
            <option key={o.value} value={o.value}>
              {o.label}
            </option>
          ))}
        </select>
        {descriptor.help ? <ControlHelp text={descriptor.help} /> : null}
      </label>
    );
  }

  // number → slider + numeric input kept in sync.
  const numValue = Number.isFinite(Number(value))
    ? Number(value)
    : descriptor.default_value;
  return (
    <label className="field">
      <span>
        {descriptor.label}
        {descriptor.unit ? <em className="unit"> {descriptor.unit}</em> : null}
        <span className="studioValue">{formatValue(numValue, descriptor.step)}</span>
      </span>
      <div className="studioNumberRow">
        <input
          type="range"
          className="studioSlider"
          min={descriptor.min_value}
          max={descriptor.max_value}
          step={descriptor.step}
          value={numValue}
          onChange={(e) => onChange(Number(e.target.value))}
        />
        <input
          type="number"
          className="studioNumberInput"
          min={descriptor.min_value}
          max={descriptor.max_value}
          step={descriptor.step}
          value={numValue}
          onChange={(e) => {
            const v = Number(e.target.value);
            if (Number.isFinite(v)) onChange(v);
          }}
        />
      </div>
      {descriptor.help ? <ControlHelp text={descriptor.help} /> : null}
    </label>
  );
}

function ControlHelp({ text }: { text: string }) {
  return <small className="studioHelp muted">{text}</small>;
}

/** Trim float noise from slider values for display, honoring the step. */
function formatValue(value: number, step: number): string {
  const decimals = step < 1 ? Math.min(4, Math.ceil(-Math.log10(step))) : 0;
  return value.toFixed(decimals);
}
