'use client';

/**
 * Navigation analysis: compares the navigation position estimate (nav_x/y/z)
 * against the target truth (tgt_x/y/z) by plotting the position-error magnitude
 * over time. When the EKF filter is active and emits `nav_nis`, also plots the
 * normalized innovation squared with a chi-square reference line at the
 * 3-DOF mean (consistency check).
 */

import { useMemo } from 'react';
import type { Data } from 'plotly.js-dist-min';
import type { SimResult } from '@/lib/types';
import Plot from './Plot';

/** chi-square mean for a 3-DOF position innovation = degrees of freedom. */
const NIS_DOF = 3;

export default function NavigationPanel({ result }: { result: SimResult }) {
  const s = result.series;

  const posError = useMemo(() => {
    const n = Math.min(
      s.t.length,
      s.nav_x.length,
      s.nav_y.length,
      s.nav_z.length,
      s.tgt_x.length,
      s.tgt_y.length,
      s.tgt_z.length,
    );
    const out = new Array<number>(n);
    for (let i = 0; i < n; i++) {
      out[i] = Math.hypot(
        s.nav_x[i] - s.tgt_x[i],
        s.nav_y[i] - s.tgt_y[i],
        s.nav_z[i] - s.tgt_z[i],
      );
    }
    return out;
  }, [s]);

  // Guard: nav_nis is only emitted by the EKF filter; treat all-zero as absent.
  const hasNis = useMemo(
    () => Array.isArray(s.nav_nis) && s.nav_nis.some((v) => v !== 0),
    [s.nav_nis],
  );

  const errData: Data[] = [
    {
      x: s.t,
      y: posError,
      type: 'scatter',
      mode: 'lines',
      name: 'Position error [m]',
      line: { color: '#4fd1c5', width: 2 },
      fill: 'tozeroy',
      fillcolor: 'rgba(79,209,197,0.10)',
    },
  ];

  const nisData = useMemo<Data[]>(() => {
    if (!hasNis || !s.nav_nis) return [];
    return [
      {
        x: s.t,
        y: s.nav_nis,
        type: 'scatter',
        mode: 'lines',
        name: 'NIS',
        line: { color: '#f6ad55', width: 1.5 },
      },
      {
        x: [s.t[0], s.t[s.t.length - 1]],
        y: [NIS_DOF, NIS_DOF],
        type: 'scatter',
        mode: 'lines',
        name: `χ² mean (dof=${NIS_DOF})`,
        line: { color: '#fc8181', width: 1.5, dash: 'dash' },
      },
    ];
  }, [hasNis, s.nav_nis, s.t]);

  return (
    <div>
      <h3 style={{ margin: '0 0 4px' }}>Navigation</h3>
      <p className="muted" style={{ marginTop: 0 }}>
        Navigation position estimate vs. target truth — the magnitude of the
        estimate error over the engagement.
      </p>
      <Plot
        data={errData}
        layout={{
          title: { text: 'Estimate Error Magnitude', font: { size: 13 } },
          xaxis: { title: { text: 'Time [s]' } },
          yaxis: { title: { text: 'Error [m]' } },
        }}
      />
      {hasNis ? (
        <Plot
          data={nisData}
          layout={{
            title: { text: 'Filter Consistency (NIS)', font: { size: 13 } },
            xaxis: { title: { text: 'Time [s]' } },
            yaxis: { title: { text: 'NIS' } },
          }}
        />
      ) : (
        <div className="placeholder" style={{ marginTop: 12 }}>
          NIS (normalized innovation squared) appears here when the EKF filter is
          selected. The current run does not emit a <code>nav_nis</code> channel.
        </div>
      )}
    </div>
  );
}
