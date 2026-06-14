'use client';

/**
 * PlotJuggler-style telemetry explorer (issue #54).
 *
 * A free-form analysis surface over the same `SimResult.series` the curated tabs
 * use (LIVE wasm or mock fallback). It is data-driven: every numeric channel in
 * `series` becomes a selectable signal, grouped into a searchable tree. Click a
 * signal to add it to the active plot pane; stack multiple panes that share a
 * synchronized time axis and a single hover cursor (scrubbing one pane shows the
 * value at that time across every pane).
 *
 * Why a bespoke multi-pane renderer instead of the shared <Plot> wrapper: we need
 * imperative access to each pane's Plotly graph div to (a) cross-link zoom/pan via
 * `plotly_relayout`, and (b) drive a shared spike-line cursor + read each curve's
 * value at the hovered time. The whole component still mounts lazily — only the
 * active tab's panes are constructed (the <Tabs> host invokes render() lazily).
 */

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import type { Data, Layout, PlotMouseEvent, PlotRelayoutEvent } from 'plotly.js-dist-min';
import type { SimResult } from '@/lib/types';
import {
  buildChannels,
  curveColor,
  evalExpression,
  groupChannels,
  timeBase,
  type Channel,
} from '@/lib/signals';

// ---------------------------------------------------------------------------
// Model
// ---------------------------------------------------------------------------

interface Curve {
  id: string;
  /** Display label (channel key or expression). */
  label: string;
  /** Resolved per-sample values aligned to the time base. */
  values: number[];
  color: string;
  visible: boolean;
  /** True for derived/expression curves (vs. raw channels). */
  derived: boolean;
}

interface Pane {
  id: string;
  curves: Curve[];
}

let uid = 0;
const nextId = (p: string) => `${p}-${++uid}`;

// ---------------------------------------------------------------------------
// One synchronized plot pane (imperative Plotly).
// ---------------------------------------------------------------------------

interface PaneProps {
  pane: Pane;
  t: number[];
  /** Shared x-range; null = autoscale. */
  xRange: [number, number] | null;
  /** Hovered time across all panes (null = none). */
  cursorT: number | null;
  onRelayout: (range: [number, number] | null) => void;
  onHover: (t: number | null) => void;
  onRemovePane: () => void;
  onToggleCurve: (curveId: string) => void;
  onRemoveCurve: (curveId: string) => void;
  isActive: boolean;
  onActivate: () => void;
}

function SyncPlotPane({
  pane,
  t,
  xRange,
  cursorT,
  onRelayout,
  onHover,
  onRemovePane,
  onToggleCurve,
  onRemoveCurve,
  isActive,
  onActivate,
}: PaneProps) {
  const elRef = useRef<HTMLDivElement>(null);
  // Cache the dynamically-imported Plotly module so listeners/effects reuse it.
  // `@types/plotly.js` types the module as a namespace (the runtime CJS export is
  // reachable via `.default`), so we hold the resolved object as the namespace type.
  const plotlyRef = useRef<typeof import('plotly.js-dist-min') | null>(null);
  // Guards relayout feedback loops (programmatic range set must not re-emit).
  const suppressRelayout = useRef(false);

  const traces: Data[] = useMemo(() => {
    return pane.curves
      .filter((c) => c.visible)
      .map((c) => ({
        x: t,
        y: c.values,
        type: 'scatter',
        mode: 'lines',
        name: c.label,
        line: { color: c.color, width: 1.6 },
        hovertemplate: `${c.label}: %{y:.4g}<extra></extra>`,
      }));
  }, [pane.curves, t]);

  const layout: Partial<Layout> = useMemo(
    () => ({
      paper_bgcolor: '#0f141b',
      plot_bgcolor: '#0f141b',
      font: { color: '#c8d2dc', family: 'ui-monospace, monospace', size: 11 },
      margin: { l: 56, r: 12, t: 8, b: 28 },
      showlegend: false,
      hovermode: 'x',
      xaxis: {
        gridcolor: '#1f2a36',
        zerolinecolor: '#2a3744',
        showspikes: true,
        spikemode: 'across',
        spikecolor: '#5a6b7a',
        spikethickness: 1,
        spikedash: 'dot',
        ...(xRange ? { range: xRange, autorange: false } : { autorange: true }),
      },
      yaxis: { gridcolor: '#1f2a36', zerolinecolor: '#2a3744', autorange: true },
    }),
    [xRange],
  );

  // Mount / update the Plotly graph.
  useEffect(() => {
    let cancelled = false;
    const el = elRef.current;
    if (!el) return;
    (async () => {
      const mod = await import('plotly.js-dist-min');
      const Plotly = mod.default as unknown as typeof import('plotly.js-dist-min');
      if (cancelled || !elRef.current) return;
      plotlyRef.current = Plotly;
      suppressRelayout.current = true;
      await Plotly.react(elRef.current, traces, layout, {
        displayModeBar: true,
        displaylogo: false,
        responsive: true,
        modeBarButtonsToRemove: ['lasso2d', 'select2d'],
      });
      suppressRelayout.current = false;
    })();
    return () => {
      cancelled = true;
    };
  }, [traces, layout]);

  // Purge on unmount.
  useEffect(() => {
    const el = elRef.current;
    return () => {
      if (el) {
        import('plotly.js-dist-min')
          .then((m) => m.default.purge(el))
          .catch(() => {});
      }
    };
  }, []);

  // Wire relayout (zoom/pan) + hover events once per mount.
  useEffect(() => {
    const el = elRef.current as (HTMLDivElement & { on?: unknown }) | null;
    if (!el || typeof (el as { on?: unknown }).on !== 'function') return;
    const gd = el as unknown as {
      on: (ev: string, cb: (d: unknown) => void) => void;
      removeAllListeners?: (ev: string) => void;
    };

    const handleRelayout = (raw: unknown) => {
      if (suppressRelayout.current) return;
      const e = raw as PlotRelayoutEvent;
      if (e['xaxis.autorange']) {
        onRelayout(null);
      } else if (
        typeof e['xaxis.range[0]'] === 'number' &&
        typeof e['xaxis.range[1]'] === 'number'
      ) {
        onRelayout([e['xaxis.range[0]'] as number, e['xaxis.range[1]'] as number]);
      }
    };
    const handleHover = (raw: unknown) => {
      const e = raw as PlotMouseEvent;
      const pt = e.points?.[0];
      if (pt && typeof pt.x === 'number') onHover(pt.x);
    };
    const handleUnhover = () => onHover(null);

    gd.on('plotly_relayout', handleRelayout);
    gd.on('plotly_hover', handleHover);
    gd.on('plotly_unhover', handleUnhover);
    return () => {
      gd.removeAllListeners?.('plotly_relayout');
      gd.removeAllListeners?.('plotly_hover');
      gd.removeAllListeners?.('plotly_unhover');
    };
    // Re-bind when the curve set changes (Plotly recreates the gd internals).
  }, [onRelayout, onHover, traces.length]);

  // Push the shared cursor as a vertical line shape (imperative, no re-render).
  useEffect(() => {
    const el = elRef.current;
    const Plotly = plotlyRef.current;
    if (!el || !Plotly) return;
    const shapes =
      cursorT == null
        ? []
        : [
            {
              type: 'line' as const,
              x0: cursorT,
              x1: cursorT,
              y0: 0,
              y1: 1,
              xref: 'x' as const,
              yref: 'paper' as const,
              line: { color: '#e2b04a', width: 1, dash: 'dot' as const },
            },
          ];
    Plotly.relayout(el, { shapes }).catch(() => {});
  }, [cursorT]);

  // Value readout at the cursor time (nearest sample) for each visible curve.
  const readout = useMemo(() => {
    if (cursorT == null || t.length === 0) return null;
    // Nearest index by binary search on the monotonic time base.
    let lo = 0;
    let hi = t.length - 1;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (t[mid] < cursorT) lo = mid;
      else hi = mid;
    }
    const idx = Math.abs(t[lo] - cursorT) <= Math.abs(t[hi] - cursorT) ? lo : hi;
    return { time: t[idx], idx };
  }, [cursorT, t]);

  return (
    <div
      className={`tx-pane${isActive ? ' tx-pane-active' : ''}`}
      onClick={onActivate}
      role="group"
      aria-label="Plot pane"
    >
      <div className="tx-pane-head">
        <div className="tx-legend">
          {pane.curves.length === 0 ? (
            <span className="muted">
              {isActive ? 'Active pane — click a signal to add it' : 'Empty pane'}
            </span>
          ) : (
            pane.curves.map((c) => {
              const val =
                readout && c.visible ? c.values[readout.idx] : undefined;
              return (
                <span
                  key={c.id}
                  className={`tx-chip${c.visible ? '' : ' tx-chip-off'}`}
                  style={{ borderColor: c.color }}
                >
                  <button
                    type="button"
                    className="tx-chip-dot"
                    style={{ background: c.visible ? c.color : 'transparent', borderColor: c.color }}
                    title={c.visible ? 'Hide curve' : 'Show curve'}
                    onClick={(e) => {
                      e.stopPropagation();
                      onToggleCurve(c.id);
                    }}
                  />
                  <span className="tx-chip-label">
                    {c.label}
                    {val !== undefined && Number.isFinite(val) ? (
                      <span className="tx-chip-val"> {val.toPrecision(5)}</span>
                    ) : null}
                  </span>
                  <button
                    type="button"
                    className="tx-chip-x"
                    title="Remove curve"
                    onClick={(e) => {
                      e.stopPropagation();
                      onRemoveCurve(c.id);
                    }}
                  >
                    ×
                  </button>
                </span>
              );
            })
          )}
        </div>
        <button
          type="button"
          className="tx-pane-remove"
          title="Remove this pane"
          onClick={(e) => {
            e.stopPropagation();
            onRemovePane();
          }}
        >
          ✕ pane
        </button>
      </div>
      <div ref={elRef} className="tx-pane-plot" />
    </div>
  );
}

// ---------------------------------------------------------------------------
// Explorer
// ---------------------------------------------------------------------------

export default function TelemetryExplorer({ result }: { result: SimResult }) {
  const channels = useMemo(() => buildChannels(result), [result]);
  const t = useMemo(() => timeBase(result), [result]);
  const groups = useMemo(() => groupChannels(channels), [channels]);
  const channelByKey = useMemo(() => {
    const m = new Map<string, Channel>();
    for (const c of channels) m.set(c.key, c);
    return m;
  }, [channels]);

  const [filter, setFilter] = useState('');
  const [panes, setPanes] = useState<Pane[]>(() => [{ id: nextId('pane'), curves: [] }]);
  const [activePane, setActivePane] = useState<string>(() => panes[0]?.id ?? '');
  const [xRange, setXRange] = useState<[number, number] | null>(null);
  const [cursorT, setCursorT] = useState<number | null>(null);
  const [exprText, setExprText] = useState('');
  const [exprError, setExprError] = useState<string | null>(null);

  // Track which signals are on the active pane (for tree highlight).
  const activeKeys = useMemo(() => {
    const p = panes.find((x) => x.id === activePane);
    return new Set(p ? p.curves.map((c) => c.label) : []);
  }, [panes, activePane]);

  const filtered = useMemo(() => {
    const q = filter.trim().toLowerCase();
    if (!q) return groups;
    return groups
      .map((g) => ({
        name: g.name,
        channels: g.channels.filter(
          (c) => c.key.toLowerCase().includes(q) || g.name.includes(q),
        ),
      }))
      .filter((g) => g.channels.length > 0);
  }, [groups, filter]);

  const colorCounter = useRef(0);

  const addCurveToActive = useCallback(
    (label: string, values: number[], derived: boolean) => {
      setPanes((prev) =>
        prev.map((p) => {
          if (p.id !== activePane) return p;
          // Toggle: if a raw channel with this label is already present, remove it.
          if (!derived && p.curves.some((c) => c.label === label)) {
            return { ...p, curves: p.curves.filter((c) => c.label !== label) };
          }
          const color = curveColor(colorCounter.current++);
          return {
            ...p,
            curves: [
              ...p.curves,
              { id: nextId('curve'), label, values, color, visible: true, derived },
            ],
          };
        }),
      );
    },
    [activePane],
  );

  const addChannel = useCallback(
    (c: Channel) => addCurveToActive(c.key, c.values, false),
    [addCurveToActive],
  );

  const addPane = useCallback(() => {
    const id = nextId('pane');
    setPanes((prev) => [...prev, { id, curves: [] }]);
    setActivePane(id);
  }, []);

  const removePane = useCallback(
    (id: string) => {
      setPanes((prev) => {
        const next = prev.filter((p) => p.id !== id);
        return next.length ? next : [{ id: nextId('pane'), curves: [] }];
      });
    },
    [],
  );

  useEffect(() => {
    // Keep activePane valid if it was removed.
    if (!panes.some((p) => p.id === activePane) && panes[0]) {
      setActivePane(panes[0].id);
    }
  }, [panes, activePane]);

  const toggleCurve = useCallback((paneId: string, curveId: string) => {
    setPanes((prev) =>
      prev.map((p) =>
        p.id !== paneId
          ? p
          : {
              ...p,
              curves: p.curves.map((c) =>
                c.id === curveId ? { ...c, visible: !c.visible } : c,
              ),
            },
      ),
    );
  }, []);

  const removeCurve = useCallback((paneId: string, curveId: string) => {
    setPanes((prev) =>
      prev.map((p) =>
        p.id !== paneId ? p : { ...p, curves: p.curves.filter((c) => c.id !== curveId) },
      ),
    );
  }, []);

  const addDerived = useCallback(() => {
    const expr = exprText.trim();
    if (!expr) return;
    try {
      const values = evalExpression(
        expr,
        (name) => channelByKey.get(name)?.values,
        t.length,
      );
      setExprError(null);
      addCurveToActive(expr, values, true);
      setExprText('');
    } catch (err) {
      setExprError(err instanceof Error ? err.message : String(err));
    }
  }, [exprText, channelByKey, t.length, addCurveToActive]);

  // Stable per-pane callbacks aren't memoized individually (small N panes); the
  // imperative Plotly cost dominates and panes are few.
  return (
    <div className="tx-root">
      <aside className="tx-tree" aria-label="Signal tree">
        <div className="tx-tree-head">
          <h3 style={{ margin: 0, fontSize: 13 }}>Signals</h3>
          <span className="muted">{channels.length} channels</span>
        </div>
        <input
          className="tx-search"
          type="search"
          placeholder="Filter signals…"
          value={filter}
          onChange={(e) => setFilter(e.target.value)}
          aria-label="Filter signals"
        />
        <div className="tx-tree-body">
          {filtered.length === 0 ? (
            <p className="muted" style={{ padding: '8px 4px' }}>
              No signals match “{filter}”.
            </p>
          ) : (
            filtered.map((g) => (
              <details key={g.name} open className="tx-group">
                <summary className="tx-group-summary">
                  {g.name} <span className="muted">({g.channels.length})</span>
                </summary>
                <ul className="tx-sig-list">
                  {g.channels.map((c) => (
                    <li key={c.key}>
                      <button
                        type="button"
                        className={`tx-sig${activeKeys.has(c.key) ? ' tx-sig-on' : ''}`}
                        title={`Add ${c.key} to the active pane`}
                        onClick={() => addChannel(c)}
                      >
                        {c.key}
                      </button>
                    </li>
                  ))}
                </ul>
              </details>
            ))
          )}
        </div>
        <div className="tx-derived">
          <label className="tx-derived-label" htmlFor="tx-expr">
            Derived signal
          </label>
          <div className="tx-derived-row">
            <input
              id="tx-expr"
              className="tx-search"
              type="text"
              placeholder="sqrt(veh_vx^2+veh_vy^2+veh_vz^2)"
              value={exprText}
              onChange={(e) => setExprText(e.target.value)}
              onKeyDown={(e) => {
                if (e.key === 'Enter') addDerived();
              }}
              aria-label="Derived-signal expression"
            />
            <button type="button" className="tx-btn" onClick={addDerived}>
              + add
            </button>
          </div>
          {exprError ? (
            <p className="tx-expr-err" role="alert">
              {exprError}
            </p>
          ) : (
            <p className="muted" style={{ margin: '4px 0 0', fontSize: 11 }}>
              Element-wise over channel names; fns: sqrt, abs, hypot, sin, min, deg…
            </p>
          )}
        </div>
      </aside>

      <section className="tx-panes" aria-label="Plot panes">
        <div className="tx-panes-toolbar">
          <span className="muted">
            Click a signal to add it to the highlighted pane. Zoom/pan and the hover
            cursor are synchronized across panes.
          </span>
          <div className="tx-toolbar-actions">
            {xRange ? (
              <button type="button" className="tx-btn" onClick={() => setXRange(null)}>
                Reset zoom
              </button>
            ) : null}
            <button type="button" className="tx-btn tx-btn-accent" onClick={addPane}>
              + plot pane
            </button>
          </div>
        </div>
        <div className="tx-pane-stack">
          {panes.map((p) => (
            <SyncPlotPane
              key={p.id}
              pane={p}
              t={t}
              xRange={xRange}
              cursorT={cursorT}
              isActive={p.id === activePane}
              onActivate={() => setActivePane(p.id)}
              onRelayout={setXRange}
              onHover={setCursorT}
              onRemovePane={() => removePane(p.id)}
              onToggleCurve={(cid) => toggleCurve(p.id, cid)}
              onRemoveCurve={(cid) => removeCurve(p.id, cid)}
            />
          ))}
        </div>
      </section>
    </div>
  );
}
