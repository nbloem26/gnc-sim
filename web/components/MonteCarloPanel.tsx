'use client';

/**
 * Monte Carlo panel: runs N (<=200) client-side WASM runs, each with dispersed
 * initial conditions, and plots the miss-distance scatter + histogram with an
 * empirical CEP circle.
 *
 * IC dispersion mirrors the C++ driver (core/src/scenario/MonteCarlo.cpp): per
 * case we draw Gaussian perturbations on launch speed, launch elevation, and the
 * target's initial position (x/y/z), and randomize the weave maneuver phase, then
 * bump the seed so the core's sensor-noise stream also varies. Without this the
 * default scenario (sensors_enable may be off, and even on the seed only drives
 * noise) collapses to N identical runs. A seeded PRNG (lib/prng.ts) makes the web
 * batch reproducible from the master seed.
 *
 * In MOCK mode (no WASM artifact) live MC is meaningless (every run returns the
 * same committed sample regardless of config), so we disable the run and show the
 * committed montecarlo_cep.png.
 */

import { useMemo, useState } from 'react';
import type { Data } from 'plotly.js-dist-min';
import type { SimConfig, SimResult } from '@/lib/types';
import { runSim, isMockMode } from '@/lib/wasmRunner';
import { cep, mean } from '@/lib/stats';
import { Prng } from '@/lib/prng';
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

// 1-sigma IC dispersions. Defaults match the C++ driver's monte_carlo sigma block
// (configs/montecarlo.json): launch speed 15 m/s, launch elevation 0.5 deg, target
// position 50 m. Overridden by baseConfig.monte_carlo when the loaded scenario
// carries it, then editable in the panel.
const DEFAULT_LAUNCH_SPEED_SIGMA_MPS = 15.0;
const DEFAULT_LAUNCH_ELEVATION_SIGMA_DEG = 0.5;
const DEFAULT_TARGET_POS_SIGMA_M = 50.0;

interface Dispersion {
  launchSpeedSigmaMps: number;
  launchElevationSigmaDeg: number;
  targetPosSigmaM: number;
}

/** Apply one case's Gaussian IC perturbations + per-case seed, mirroring the C++ driver. */
function disperseConfig(base: SimConfig, rng: Prng, d: Dispersion): SimConfig {
  // Independent per-case seed: drives the core's bit-identical sensor-noise stream.
  const seed = rng.nextUint32();

  const launchSpeed = base.vehicle.launch_speed + rng.gaussian(0, d.launchSpeedSigmaMps);
  const launchElevationDeg =
    base.vehicle.launch_elevation_deg + rng.gaussian(0, d.launchElevationSigmaDeg);
  const [tx, ty, tz] = base.target.pos0;
  const targetPos0: [number, number, number] = [
    tx + rng.gaussian(0, d.targetPosSigmaM),
    ty + rng.gaussian(0, d.targetPosSigmaM),
    tz + rng.gaussian(0, d.targetPosSigmaM),
  ];

  const target: SimConfig['target'] = { ...base.target, pos0: targetPos0 };
  // A weaving target is caught at a random point in its maneuver cycle each run.
  if (base.target.maneuver === 'weave') {
    target.maneuver_phase_deg = rng.uniform(0, 360);
  }

  return {
    ...base,
    seed,
    vehicle: {
      ...base.vehicle,
      launch_speed: launchSpeed,
      launch_elevation_deg: launchElevationDeg,
    },
    target,
  };
}

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

  // Seed the editable dispersions from baseConfig.monte_carlo when present, else
  // from the C++ defaults. (One-time init: the user edits them in the panel.)
  const [launchSpeedSigmaMps, setLaunchSpeedSigmaMps] = useState(
    baseConfig?.monte_carlo?.launch_speed_sigma ?? DEFAULT_LAUNCH_SPEED_SIGMA_MPS,
  );
  const [launchElevationSigmaDeg, setLaunchElevationSigmaDeg] = useState(
    baseConfig?.monte_carlo?.launch_elevation_sigma_deg ?? DEFAULT_LAUNCH_ELEVATION_SIGMA_DEG,
  );
  const [targetPosSigmaM, setTargetPosSigmaM] = useState(
    baseConfig?.monte_carlo?.target_pos_sigma ?? DEFAULT_TARGET_POS_SIGMA_M,
  );

  const isWeave = baseConfig?.target.maneuver === 'weave';

  async function runBatch() {
    if (!baseConfig) return;
    const n = Math.max(1, Math.min(MAX_CASES, Math.floor(numCases)));
    setRunning(true);
    setProgress(0);
    setResults([]);
    const out: MCResult[] = [];
    // Master PRNG seeded from the scenario seed -> reproducible web batch.
    const master = new Prng(baseConfig.seed ?? 1);
    const dispersion: Dispersion = {
      launchSpeedSigmaMps,
      launchElevationSigmaDeg,
      targetPosSigmaM,
    };
    for (let i = 0; i < n; i++) {
      const cfg: SimConfig = disperseConfig(baseConfig, master, dispersion);
      const { seed } = cfg;
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

      <fieldset className="mcDispersion" disabled={running}>
        <legend>Initial-condition dispersion (1σ)</legend>
        <div className="mcControls">
          <label className="field inline">
            <span>Launch speed [m/s]</span>
            <input
              type="number"
              min={0}
              step={0.5}
              value={launchSpeedSigmaMps}
              onChange={(e) => setLaunchSpeedSigmaMps(Number(e.target.value))}
            />
          </label>
          <label className="field inline">
            <span>Launch elevation [deg]</span>
            <input
              type="number"
              min={0}
              step={0.1}
              value={launchElevationSigmaDeg}
              onChange={(e) => setLaunchElevationSigmaDeg(Number(e.target.value))}
            />
          </label>
          <label className="field inline">
            <span>Target position x/y/z [m]</span>
            <input
              type="number"
              min={0}
              step={5}
              value={targetPosSigmaM}
              onChange={(e) => setTargetPosSigmaM(Number(e.target.value))}
            />
          </label>
        </div>
        <p className="muted" style={{ margin: '4px 0 0' }}>
          Each case perturbs launch speed (σ {launchSpeedSigmaMps} m/s), launch elevation (σ{' '}
          {launchElevationSigmaDeg}°) and target position (σ {targetPosSigmaM} m), bumps the seed
          for sensor noise
          {isWeave ? ', and randomizes the weave maneuver phase ∈ [0,360°)' : ''}. Mirrors{' '}
          core/src/scenario/MonteCarlo.cpp; reproducible from the scenario seed.
        </p>
      </fieldset>

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
