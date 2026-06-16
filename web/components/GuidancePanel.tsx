'use client';

/**
 * Guidance analysis: the proportional-navigation signals over time —
 * line-of-sight rate, closing velocity, range-to-go, and commanded acceleration
 * magnitude. Together these show the homing loop nulling LOS rate to intercept.
 */

import { useMemo } from 'react';
import type { Data } from 'plotly.js-dist-min';
import type { SimResult } from '@/lib/types';
import Plot from './Plot';

function magnitude(x: number[], y: number[], z: number[]): number[] {
  const n = Math.min(x.length, y.length, z.length);
  const out = new Array<number>(n);
  for (let i = 0; i < n; i++) out[i] = Math.hypot(x[i], y[i], z[i]);
  return out;
}

export default function GuidancePanel({ result }: { result: SimResult }) {
  const s = result.series;

  const accelMag = useMemo(
    () => magnitude(s.accel_cmd_x, s.accel_cmd_y, s.accel_cmd_z),
    [s.accel_cmd_x, s.accel_cmd_y, s.accel_cmd_z],
  );

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

  const closingRange: Data[] = [
    {
      x: s.t,
      y: s.v_closing,
      type: 'scatter',
      mode: 'lines',
      name: 'Closing velocity [m/s]',
      line: { color: '#4fd1c5', width: 2 },
    },
    {
      x: s.t,
      y: s.range,
      type: 'scatter',
      mode: 'lines',
      name: 'Range [m]',
      yaxis: 'y2',
      line: { color: '#90cdf4', width: 2 },
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
      <h3 style={{ margin: '0 0 4px' }}>Guidance</h3>
      <p className="muted" style={{ marginTop: 0 }}>
        Homing-guidance signals: the law drives line-of-sight rate
        toward zero while closing range.
      </p>
      <Plot
        data={losData}
        layout={{
          title: { text: 'Line-of-Sight Rate', font: { size: 13 } },
          xaxis: { title: { text: 'Time [s]' } },
          yaxis: { title: { text: 'rad/s' } },
        }}
      />
      <Plot
        data={closingRange}
        layout={{
          title: { text: 'Closing Velocity & Range', font: { size: 13 } },
          xaxis: { title: { text: 'Time [s]' } },
          yaxis: { title: { text: 'Closing vel [m/s]' } },
          yaxis2: {
            title: { text: 'Range [m]' },
            overlaying: 'y',
            side: 'right',
            gridcolor: 'rgba(0,0,0,0)',
          },
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
