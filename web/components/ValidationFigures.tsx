'use client';

/**
 * Engineering validation figures produced by the Python rigor track. Rendered
 * both at the /validation route and inline as the dashboard's Validation tab.
 * Any figure not yet built renders as a placeholder.
 */

import { useState } from 'react';

interface Fig {
  src: string;
  title: string;
  caption: string;
}

export const VALIDATION_FIGURES: Fig[] = [
  {
    src: '/figures/allan_deviation.png',
    title: 'Allan Deviation — Sensor Noise Identification',
    caption:
      'Allan deviation of synthetic IMU data isolates each noise process by its slope: ' +
      'white noise (−½), bias instability (flat floor), and rate random walk (+½). ' +
      'These recovered parameters are written to sensor_params.json and drive the in-sim IMU model.',
  },
  {
    src: '/figures/loop_closure_allan.png',
    title: 'Loop-Closure Validation',
    caption:
      'Re-running Allan variance on the simulator’s own measured IMU output recovers the ' +
      'same noise parameters that were injected. This closes the sensor-fidelity loop: ' +
      'the C++ sensor model faithfully reproduces the characterised hardware error budget.',
  },
  {
    src: '/figures/montecarlo_cep.png',
    title: 'Monte Carlo — Circular Error Probable',
    caption:
      'Impact dispersion over a 200-case batch with randomized launch and target conditions. ' +
      'The dashed circle is the empirical CEP (50% radius), the headline accuracy metric for ' +
      'the guidance law under realistic uncertainty.',
  },
  {
    src: '/figures/trajectory_sample.png',
    title: 'Representative Engagement',
    caption:
      'A single proportional-navigation homing trajectory: the vehicle nulls the ' +
      'line-of-sight rate and converges to intercept. Used as a sanity reference for the ' +
      'in-browser trajectory rendering.',
  },
  {
    src: '/figures/validation_summary.png',
    title: 'Validation Summary',
    caption:
      'Consolidated analytical checks — energy/Mach consistency, integrator convergence, and ' +
      'guidance-law behaviour — confirming the dynamics core matches closed-form expectations.',
  },
];

function Figure({ fig }: { fig: Fig }) {
  const [ok, setOk] = useState(true);
  return (
    <div className="figure">
      <h3>{fig.title}</h3>
      {ok ? (
        // eslint-disable-next-line @next/next/no-img-element
        <img src={fig.src} alt={fig.title} onError={() => setOk(false)} />
      ) : (
        <div className="placeholder">
          {fig.src.split('/').pop()} not generated yet
        </div>
      )}
      <p>{fig.caption}</p>
    </div>
  );
}

export default function ValidationFigures() {
  return (
    <div className="valGrid valGridInline">
      {VALIDATION_FIGURES.map((f) => (
        <Figure key={f.src} fig={f} />
      ))}
    </div>
  );
}
