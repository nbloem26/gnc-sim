'use client';

/**
 * Sensors analysis: overlays true vs. measured signals for the IMU accelerometer
 * (x-axis), the IMU gyro (x-axis), and the seeker line-of-sight, with the
 * measurement residual (meas - true) plotted alongside each. Channels are
 * optional in the data contract, so each block is guarded and only rendered when
 * present.
 */

import { useMemo } from 'react';
import type { Data } from 'plotly.js-dist-min';
import type { SimResult } from '@/lib/types';
import Plot from './Plot';

const TRUE_COLOR = '#90cdf4';
const MEAS_COLOR = '#f6ad55';
const RES_COLOR = '#fc8181';

function residual(meas: number[], truth: number[]): number[] {
  const n = Math.min(meas.length, truth.length);
  const out = new Array<number>(n);
  for (let i = 0; i < n; i++) out[i] = meas[i] - truth[i];
  return out;
}

interface Block {
  key: string;
  title: string;
  unit: string;
  resUnit: string;
  truth: number[];
  meas: number[];
}

export default function SensorsPanel({ result }: { result: SimResult }) {
  const s = result.series;

  const blocks = useMemo<Block[]>(() => {
    const out: Block[] = [];
    if (s.imu_accel_true_x && s.imu_accel_meas_x) {
      out.push({
        key: 'accel',
        title: 'IMU Accelerometer (x)',
        unit: 'm/s²',
        resUnit: 'm/s²',
        truth: s.imu_accel_true_x,
        meas: s.imu_accel_meas_x,
      });
    }
    if (s.imu_gyro_true_x && s.imu_gyro_meas_x) {
      out.push({
        key: 'gyro',
        title: 'IMU Gyro (x)',
        unit: 'rad/s',
        resUnit: 'rad/s',
        truth: s.imu_gyro_true_x,
        meas: s.imu_gyro_meas_x,
      });
    }
    if (s.seeker_los_true && s.seeker_los_meas) {
      out.push({
        key: 'seeker',
        title: 'Seeker Line-of-Sight',
        unit: 'rad',
        resUnit: 'rad',
        truth: s.seeker_los_true,
        meas: s.seeker_los_meas,
      });
    }
    return out;
  }, [s]);

  if (blocks.length === 0) {
    return (
      <div>
        <h3 style={{ margin: '0 0 4px' }}>Sensors</h3>
        <div className="placeholder" style={{ marginTop: 12 }}>
          No sensor channels in this run — enable sensors in the scenario to see
          true vs. measured IMU and seeker signals.
        </div>
      </div>
    );
  }

  return (
    <div>
      <h3 style={{ margin: '0 0 4px' }}>Sensors</h3>
      <p className="muted" style={{ marginTop: 0 }}>
        True vs. measured sensor signals, with the residual (measured − true)
        showing the injected noise and bias.
      </p>
      {blocks.map((b) => {
        const res = residual(b.meas, b.truth);
        const overlay: Data[] = [
          {
            x: s.t,
            y: b.truth,
            type: 'scatter',
            mode: 'lines',
            name: 'True',
            line: { color: TRUE_COLOR, width: 2 },
          },
          {
            x: s.t,
            y: b.meas,
            type: 'scatter',
            mode: 'lines',
            name: 'Measured',
            line: { color: MEAS_COLOR, width: 1, dash: 'dot' },
          },
        ];
        const resData: Data[] = [
          {
            x: s.t,
            y: res,
            type: 'scatter',
            mode: 'lines',
            name: 'Residual',
            line: { color: RES_COLOR, width: 1 },
          },
        ];
        return (
          <div key={b.key}>
            <Plot
              data={overlay}
              layout={{
                title: { text: `${b.title} — True vs Measured`, font: { size: 13 } },
                xaxis: { title: { text: 'Time [s]' } },
                yaxis: { title: { text: b.unit } },
              }}
            />
            <Plot
              data={resData}
              style={{ height: 240 }}
              layout={{
                title: { text: `${b.title} — Residual`, font: { size: 13 } },
                xaxis: { title: { text: 'Time [s]' } },
                yaxis: { title: { text: b.resUnit } },
              }}
            />
          </div>
        );
      })}
    </div>
  );
}
