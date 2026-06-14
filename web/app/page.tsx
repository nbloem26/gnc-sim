'use client';

import { useEffect, useMemo, useState } from 'react';
import dynamic from 'next/dynamic';
import type { SimConfig, SimResult } from '@/lib/types';
import { runSim, isMockMode, isResolved, warmUp } from '@/lib/wasmRunner';
import ParamForm from '@/components/ParamForm';
import Tabs, { type TabDef } from '@/components/Tabs';
import TrajectoryPlot from '@/components/TrajectoryPlot';
import GuidancePanel from '@/components/GuidancePanel';
import NavigationPanel from '@/components/NavigationPanel';
import SensorsPanel from '@/components/SensorsPanel';
import EnvironmentPanel from '@/components/EnvironmentPanel';
import MonteCarloPanel from '@/components/MonteCarloPanel';
import ValidationFigures from '@/components/ValidationFigures';

// Ground track pulls in Leaflet (window-bound) — load client-only, no SSR.
// It is composed inside the merged "Trajectory & Ground Track" tab, whose
// render() is invoked lazily by <Tabs> only while that tab is active — so the
// Leaflet map never instantiates on first load or under any other tab.
const GroundTrack = dynamic(() => import('@/components/GroundTrack'), {
  ssr: false,
  loading: () => <div className="placeholder">Loading map…</div>,
});

// Telemetry Explorer instantiates Plotly imperatively per pane — load client-only
// and only when its tab actually mounts (lazy via <Tabs>), so first load stays light.
const TelemetryExplorer = dynamic(() => import('@/components/TelemetryExplorer'), {
  ssr: false,
  loading: () => <div className="placeholder">Loading explorer…</div>,
});

export default function Home() {
  const [result, setResult] = useState<SimResult | null>(null);
  const [lastConfig, setLastConfig] = useState<SimConfig | null>(null);
  const [running, setRunning] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [mock, setMock] = useState(false);
  const [modeKnown, setModeKnown] = useState(false);
  const [activeTab, setActiveTab] = useState('trajectory');

  // Warm the WASM module and resolve real-vs-mock on mount.
  useEffect(() => {
    warmUp().then(() => {
      setMock(isMockMode());
      setModeKnown(isResolved());
    });
  }, []);

  async function handleRun(config: SimConfig) {
    setRunning(true);
    setError(null);
    try {
      const r = await runSim(config);
      setResult(r);
      setLastConfig(config);
      setMock(isMockMode());
      setModeKnown(true);
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    } finally {
      setRunning(false);
    }
  }

  // Tab definitions. Each `render()` is invoked lazily by <Tabs> — only the
  // active tab's Plotly/Leaflet components ever mount. This keeps the page from
  // instantiating every chart at once on load (which triggered the Canvas2D
  // readback storm + setTimeout-handler violation).
  const tabs = useMemo<TabDef[]>(() => {
    if (!result) return [];
    return [
      {
        id: 'trajectory',
        label: 'Trajectory & Ground Track',
        // Stack the trajectory plot above the Leaflet ground track. Because
        // this render() runs only while the tab is active, the lazy
        // <GroundTrack> (and its Leaflet map) still mounts only on activation.
        render: () => (
          <div className="trajGroundStack">
            <TrajectoryPlot result={result} />
            <GroundTrack result={result} />
          </div>
        ),
      },
      {
        id: 'guidance',
        label: 'Guidance',
        render: () => <GuidancePanel result={result} />,
      },
      {
        id: 'navigation',
        label: 'Navigation',
        render: () => <NavigationPanel result={result} />,
      },
      {
        id: 'sensors',
        label: 'Sensors',
        render: () => <SensorsPanel result={result} />,
      },
      {
        id: 'environment',
        label: 'Environment / Aero',
        render: () => <EnvironmentPanel result={result} />,
      },
      {
        id: 'montecarlo',
        label: 'Monte Carlo',
        render: () => <MonteCarloPanel baseConfig={lastConfig} mock={mock} />,
      },
      {
        id: 'explorer',
        label: 'Telemetry Explorer',
        render: () => <TelemetryExplorer result={result} />,
      },
      {
        id: 'validation',
        label: 'Validation',
        render: () => <ValidationFigures />,
      },
    ];
  }, [result, lastConfig, mock]);

  return (
    <>
      <div className="intro">
        <div
          style={{ display: 'flex', alignItems: 'center', gap: 12, flexWrap: 'wrap' }}
        >
          <h1 style={{ marginBottom: 0 }}>Interactive GNC Simulator</h1>
          {modeKnown ? (
            <span
              className={`badge ${mock ? 'mock' : 'real'}`}
              title={
                mock
                  ? 'WASM artifact not present — showing committed sample_result.json'
                  : 'Running the C++ core compiled to WebAssembly in your browser'
              }
            >
              {mock ? 'MOCK · sample data' : 'LIVE · WASM'}
            </span>
          ) : null}
        </div>
        <p className="muted">
          Adjust the scenario on the left and run a proportional-navigation homing
          engagement. The C++ flight-dynamics core executes entirely in your browser.
        </p>
      </div>

      <div className="shell">
        <ParamForm onRun={handleRun} running={running} />

        <div className="results">
          {error ? (
            <div className="card" style={{ borderColor: 'var(--err)' }}>
              <strong style={{ color: 'var(--err)' }}>Simulation error:</strong>{' '}
              {error}
            </div>
          ) : null}

          {!result ? (
            <div className="card placeholder">Run a simulation to see results.</div>
          ) : (
            <>
              <div className="card">
                <div className="headline">
                  <div className="hstat">
                    <span className="k">Status</span>
                    <span className={`v ${result.intercept ? 'hit' : 'nohit'}`}>
                      {result.intercept ? 'INTERCEPT' : 'MISS'}
                    </span>
                  </div>
                  <div className="hstat">
                    <span className="k">Miss distance</span>
                    <span className="v">{result.miss_distance.toFixed(2)} m</span>
                  </div>
                  <div className="hstat">
                    <span className="k">Intercept time</span>
                    <span className="v">
                      {Number.isFinite(result.intercept_time)
                        ? `${result.intercept_time.toFixed(2)} s`
                        : '—'}
                    </span>
                  </div>
                  <div className="hstat">
                    <span className="k">Model · seed</span>
                    <span className="v">
                      {result.model} · {result.seed}
                    </span>
                  </div>
                </div>
              </div>

              <div className="card">
                <Tabs tabs={tabs} active={activeTab} onChange={setActiveTab} />
              </div>
            </>
          )}
        </div>
      </div>
    </>
  );
}
