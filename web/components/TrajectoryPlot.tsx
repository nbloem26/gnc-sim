'use client';

/**
 * Trajectory plot: vehicle vs target path with intercept marker.
 * Default 2D view is East (x) vs Up (z); a toggle switches to a 3D ENU view.
 */

import { useMemo, useState } from 'react';
import type { Data } from 'plotly.js-dist-min';
import type { SimResult } from '@/lib/types';
import Plot from './Plot';

const VEH_COLOR = '#4fd1c5';
const TGT_COLOR = '#f6ad55';
const HIT_COLOR = '#fc8181';

export default function TrajectoryPlot({ result }: { result: SimResult }) {
  const [view, setView] = useState<'2d' | '3d'>('2d');
  const s = result.series;

  const data2d = useMemo<Data[]>(() => {
    const last = s.t.length - 1;
    return [
      {
        x: s.veh_x,
        y: s.veh_z,
        type: 'scatter',
        mode: 'lines',
        name: 'Vehicle',
        line: { color: VEH_COLOR, width: 2 },
      },
      {
        x: s.tgt_x,
        y: s.tgt_z,
        type: 'scatter',
        mode: 'lines',
        name: 'Target',
        line: { color: TGT_COLOR, width: 2, dash: 'dot' },
      },
      {
        x: [s.veh_x[last]],
        y: [s.veh_z[last]],
        type: 'scatter',
        mode: 'markers',
        name: result.intercept ? 'Intercept' : 'Closest approach',
        marker: { color: HIT_COLOR, size: 12, symbol: 'x' },
      },
    ];
  }, [s, result.intercept]);

  const data3d = useMemo<Data[]>(() => {
    const last = s.t.length - 1;
    return [
      {
        x: s.veh_x,
        y: s.veh_y,
        z: s.veh_z,
        type: 'scatter3d',
        mode: 'lines',
        name: 'Vehicle',
        line: { color: VEH_COLOR, width: 4 },
      },
      {
        x: s.tgt_x,
        y: s.tgt_y,
        z: s.tgt_z,
        type: 'scatter3d',
        mode: 'lines',
        name: 'Target',
        line: { color: TGT_COLOR, width: 4 },
      },
      {
        x: [s.veh_x[last]],
        y: [s.veh_y[last]],
        z: [s.veh_z[last]],
        type: 'scatter3d',
        mode: 'markers',
        name: result.intercept ? 'Intercept' : 'Closest approach',
        marker: { color: HIT_COLOR, size: 5, symbol: 'x' },
      },
    ];
  }, [s, result.intercept]);

  return (
    <div>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
        <h3 style={{ margin: '0 0 4px' }}>Trajectory</h3>
        <div className="seg">
          <button
            className={view === '2d' ? 'segActive' : ''}
            onClick={() => setView('2d')}
            type="button"
          >
            2D
          </button>
          <button
            className={view === '3d' ? 'segActive' : ''}
            onClick={() => setView('3d')}
            type="button"
          >
            3D
          </button>
        </div>
      </div>
      {view === '2d' ? (
        <Plot
          data={data2d}
          layout={{
            xaxis: { title: { text: 'East [m]' } },
            yaxis: { title: { text: 'Up [m]' }, scaleanchor: 'x', scaleratio: 1 },
          }}
        />
      ) : (
        <Plot
          data={data3d}
          style={{ height: 460 }}
          layout={{
            scene: {
              xaxis: { title: { text: 'East [m]' }, gridcolor: '#1f2a36' },
              yaxis: { title: { text: 'North [m]' }, gridcolor: '#1f2a36' },
              zaxis: { title: { text: 'Up [m]' }, gridcolor: '#1f2a36' },
              bgcolor: '#0f141b',
            },
          }}
        />
      )}
    </div>
  );
}
