'use client';

/**
 * Monte Carlo panel: runs N (<=200) client-side WASM runs varying the seed and
 * plots the miss-distance scatter + histogram with an empirical CEP circle.
 *
 * In MOCK mode (no WASM artifact) live MC is meaningless (every seed returns the
 * same sample), so we disable the run and show the committed montecarlo_cep.png.
 */

import { useMemo, useState } from 'react';
import type { Data } from 'plotly.js-dist-min';
import type { SimConfig, SimResult } from '@/lib/types';
import { runSim, isMockMode } from '@/lib/wasmRunner';
import { cep, mean } from '@/lib/stats';
import Plot from './Plot';

interface MCResult {
  seed: number;
  miss: number;
  impactX: number;
  impactZ: number;
  intercept: boolean;
}

const MAX_CASES = 200;
const CEP_FIGURE = '/figures/montecarlo_cep.png';

export default function MonteCarloPanel({
  baseConfig,
  mock,
}: {
  baseConfig: SimConfig | null;
  mock: boolean;
}) {
  const [numCases, setNumCases] = useState(50);
  const [running, setRunning] = useState(false);
  const [progress, setProgress] = useState(0);
  const [results, setResults] = useState<MCResult[]>([]);
  const [figureOk, setFigureOk] = useState(true);

  async function runBatch() {
    if (!baseConfig) return;
    const n = Math.max(1, Math.min(MAX_CASES, Math.floor(numCases)));
    setRunning(true);
    setProgress(0);
    setResults([]);
    const out: MCResult[] = [];
    for (let i = 0; i < n; i++) {
      const seed = (baseConfig.seed ?? 1) + i;
      const cfg: SimConfig = { ...baseConfig, seed };
      try {
        const r: SimResult = await runSim(cfg);
        const last = r.series.t.length - 1;
        out.push({
          seed,
          miss: r.miss_distance,
          impactX: r.series.veh_x[last] - r.series.tgt_x[last],
          impactZ: r.series.veh_z[last] - r.series.tgt_z[last],
          intercept: r.intercept,
        });
      } catch {
        // Skip failed cases; keep the batch going.
      }
      setProgress(i + 1);
      // Yield to the event loop so the progress bar can paint.
      // eslint-disable-next-line no-await-in-loop
      await new Promise((res) => setTimeout(res, 0));
    }
    setResults(out);
    setRunning(false);
  }

  const stats = useMemo(() => {
    if (results.length === 0) return null;
    const misses = results.map((r) => r.miss);
    return {
      n: results.length,
      cep: cep(misses),
      mean: mean(misses),
      pHit: results.filter((r) => r.intercept).length / results.length,
    };
  }, [results]);

  const scatterData = useMemo<Data[]>(() => {
    if (results.length === 0 || !stats) return [];
    const cepR = stats.cep;
    // CEP circle (parametric).
    const theta = Array.from({ length: 65 }, (_, i) => (i / 64) * 2 * Math.PI);
    return [
      {
        x: results.map((r) => r.impactX),
        y: results.map((r) => r.impactZ),
        type: 'scatter',
        mode: 'markers',
        name: 'Impact (E vs U err)',
        marker: { color: '#4fd1c5', size: 6, opacity: 0.7 },
      },
      {
        x: theta.map((t) => cepR * Math.cos(t)),
        y: theta.map((t) => cepR * Math.sin(t)),
        type: 'scatter',
        mode: 'lines',
        name: `CEP = ${cepR.toFixed(2)} m`,
        line: { color: '#fc8181', width: 2, dash: 'dash' },
      },
    ];
  }, [results, stats]);

  const histData = useMemo<Data[]>(() => {
    if (results.length === 0) return [];
    return [
      {
        x: results.map((r) => r.miss),
        type: 'histogram',
        name: 'Miss distance',
        marker: { color: '#90cdf4' },
        nbinsx: 24,
      },
    ];
  }, [results]);

  // ---- Mock mode: show committed figure -----------------------------------
  if (mock || isMockMode()) {
    return (
      <div>
        <h3 style={{ margin: '0 0 4px' }}>Monte Carlo</h3>
        <p className="muted">
          Live Monte Carlo is disabled in mock mode (no WASM artifact present —
          every seed returns the same committed sample). The figure below is the
          batch CEP produced by the native build.
        </p>
        {figureOk ? (
          <img
            src={CEP_FIGURE}
            alt="Monte Carlo CEP scatter"
            style={{ width: '100%', borderRadius: 8, border: '1px solid #1f2a36' }}
            onError={() => setFigureOk(false)}
          />
        ) : (
          <div className="placeholder">montecarlo_cep.png not built yet</div>
        )}
      </div>
    );
  }

  // ---- Real mode: live batch ----------------------------------------------
  return (
    <div>
      <h3 style={{ margin: '0 0 4px' }}>Monte Carlo</h3>
      <div className="mcControls">
        <label className="field inline">
          <span>Cases (≤ {MAX_CASES})</span>
          <input
            type="number"
            min={1}
            max={MAX_CASES}
            value={numCases}
            onChange={(e) => setNumCases(Number(e.target.value))}
            disabled={running}
          />
        </label>
        <button
          type="button"
          className="primary"
          onClick={runBatch}
          disabled={running || !baseConfig}
        >
          {running ? `Running ${progress}/${numCases}…` : 'Run batch'}
        </button>
      </div>

      {running ? (
        <div className="progress">
          <div
            className="bar"
            style={{ width: `${(progress / Math.max(1, numCases)) * 100}%` }}
          />
        </div>
      ) : null}

      {stats ? (
        <div className="statsRow">
          <div className="stat">
            <span className="k">N</span>
            <span className="v">{stats.n}</span>
          </div>
          <div className="stat">
            <span className="k">CEP</span>
            <span className="v">{stats.cep.toFixed(2)} m</span>
          </div>
          <div className="stat">
            <span className="k">Mean miss</span>
            <span className="v">{stats.mean.toFixed(2)} m</span>
          </div>
          <div className="stat">
            <span className="k">P(hit)</span>
            <span className="v">{(stats.pHit * 100).toFixed(0)}%</span>
          </div>
        </div>
      ) : null}

      {results.length > 0 ? (
        <>
          <Plot
            data={scatterData}
            layout={{
              title: { text: 'Impact dispersion', font: { size: 13 } },
              xaxis: { title: { text: 'East error [m]' }, scaleanchor: 'y', scaleratio: 1 },
              yaxis: { title: { text: 'Up error [m]' } },
            }}
          />
          <Plot
            data={histData}
            layout={{
              title: { text: 'Miss-distance distribution', font: { size: 13 } },
              xaxis: { title: { text: 'Miss distance [m]' } },
              yaxis: { title: { text: 'Count' } },
            }}
          />
        </>
      ) : (
        <p className="muted">Run a batch to see the CEP and dispersion.</p>
      )}
    </div>
  );
}
