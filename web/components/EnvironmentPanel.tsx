'use client';

/**
 * Environment / Aero analysis: altitude (veh_z), Mach number, and speed
 * (|velocity|) over time — the flight envelope the vehicle traverses.
 */

import { useMemo } from 'react';
import type { Data } from 'plotly.js-dist-min';
import type { SimResult } from '@/lib/types';
import Plot from './Plot';

export default function EnvironmentPanel({ result }: { result: SimResult }) {
  const s = result.series;

  const speed = useMemo(() => {
    const n = Math.min(s.veh_vx.length, s.veh_vy.length, s.veh_vz.length);
    const out = new Array<number>(n);
    for (let i = 0; i < n; i++) {
      out[i] = Math.hypot(s.veh_vx[i], s.veh_vy[i], s.veh_vz[i]);
    }
    return out;
  }, [s.veh_vx, s.veh_vy, s.veh_vz]);

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

  const machData: Data[] = [
    {
      x: s.t,
      y: s.mach,
      type: 'scatter',
      mode: 'lines',
      name: 'Mach',
      line: { color: '#f6ad55', width: 2 },
      fill: 'tozeroy',
      fillcolor: 'rgba(246,173,85,0.10)',
    },
  ];

  return (
    <div>
      <h3 style={{ margin: '0 0 4px' }}>Environment / Aero</h3>
      <p className="muted" style={{ marginTop: 0 }}>
        Flight envelope: altitude, Mach number, and total speed over the
        engagement.
      </p>
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
        data={machData}
        layout={{
          title: { text: 'Mach Number', font: { size: 13 } },
          xaxis: { title: { text: 'Time [s]' } },
          yaxis: { title: { text: 'Mach' } },
        }}
      />
    </div>
  );
}
