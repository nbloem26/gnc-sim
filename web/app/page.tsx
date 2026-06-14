'use client';

import { useEffect, useState } from 'react';
import dynamic from 'next/dynamic';
import type { SimConfig, SimResult } from '@/lib/types';
import { runSim, isMockMode, isResolved, warmUp } from '@/lib/wasmRunner';
import ParamForm from '@/components/ParamForm';
import TrajectoryPlot from '@/components/TrajectoryPlot';
import StatePlots from '@/components/StatePlots';
import MonteCarloPanel from '@/components/MonteCarloPanel';

// Ground track pulls in Leaflet (window-bound) — load client-only, no SSR.
const GroundTrack = dynamic(() => import('@/components/GroundTrack'), {
  ssr: false,
  loading: () => <div className="placeholder">Loading map…</div>,
});

export default function Home() {
  const [result, setResult] = useState<SimResult | null>(null);
  const [lastConfig, setLastConfig] = useState<SimConfig | null>(null);
  const [running, setRunning] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [mock, setMock] = useState(false);
  const [modeKnown, setModeKnown] = useState(false);

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
            <div className="card placeholder">
              Run a simulation to see results.
            </div>
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
                <TrajectoryPlot result={result} />
              </div>

              <div className="grid2">
                <div className="card">
                  <StatePlots result={result} />
                </div>
                <div className="card">
                  <GroundTrack result={result} />
                </div>
              </div>

              <div className="card">
                <MonteCarloPanel baseConfig={lastConfig} mock={mock} />
              </div>
            </>
          )}
        </div>
      </div>
    </>
  );
}
