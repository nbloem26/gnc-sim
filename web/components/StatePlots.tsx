'use client';

/**
 * State time-series plots: speed & altitude, LOS rate, and accel-command magnitude.
 */

import { useMemo } from 'react';
import type { Data } from 'plotly.js-dist-min';
import type { SimResult } from '@/lib/types';
import Plot from './Plot';

function magnitude(x: number[], y: number[], z: number[]): number[] {
  const n = Math.min(x.length, y.length, z.length);
  const out = new Array<number>(n);
  for (let i = 0; i < n; i++) {
    out[i] = Math.hypot(x[i], y[i], z[i]);
  }
  return out;
}

export default function StatePlots({ result }: { result: SimResult }) {
  const s = result.series;

  const speed = useMemo(
    () => magnitude(s.veh_vx, s.veh_vy, s.veh_vz),
    [s.veh_vx, s.veh_vy, s.veh_vz],
  );
  const accelMag = useMemo(
    () => magnitude(s.accel_cmd_x, s.accel_cmd_y, s.accel_cmd_z),
    [s.accel_cmd_x, s.accel_cmd_y, s.accel_cmd_z],
  );

  const speedAlt: Data[] = [
    {
      x: s.t,
      y: speed,
      type: 'scatter',
      mode: 'lines',
      name: 'Speed [m/s]',
      line: { color: '#4fd1c5', width: 2 },
    },
    {
      x: s.t,
      y: s.veh_z,
      type: 'scatter',
      mode: 'lines',
      name: 'Altitude [m]',
      yaxis: 'y2',
      line: { color: '#90cdf4', width: 2 },
    },
  ];

  const losData: Data[] = [
    {
      x: s.t,
      y: s.los_rate,
      type: 'scatter',
      mode: 'lines',
      name: 'LOS rate [rad/s]',
      line: { color: '#f6ad55', width: 2 },
    },
  ];

  const accelData: Data[] = [
    {
      x: s.t,
      y: accelMag,
      type: 'scatter',
      mode: 'lines',
      name: '|a_cmd| [m/s²]',
      line: { color: '#fc8181', width: 2 },
      fill: 'tozeroy',
      fillcolor: 'rgba(252,129,129,0.12)',
    },
  ];

  return (
    <div>
      <h3 style={{ margin: '0 0 4px' }}>States</h3>
      <Plot
        data={speedAlt}
        layout={{
          title: { text: 'Speed & Altitude', font: { size: 13 } },
          xaxis: { title: { text: 'Time [s]' } },
          yaxis: { title: { text: 'Speed [m/s]' } },
          yaxis2: {
            title: { text: 'Altitude [m]' },
            overlaying: 'y',
            side: 'right',
            gridcolor: 'rgba(0,0,0,0)',
          },
        }}
      />
      <Plot
        data={losData}
        layout={{
          title: { text: 'Line-of-Sight Rate', font: { size: 13 } },
          xaxis: { title: { text: 'Time [s]' } },
          yaxis: { title: { text: 'rad/s' } },
        }}
      />
      <Plot
        data={accelData}
        layout={{
          title: { text: 'Acceleration Command Magnitude', font: { size: 13 } },
          xaxis: { title: { text: 'Time [s]' } },
          yaxis: { title: { text: 'm/s²' } },
        }}
      />
    </div>
  );
}
