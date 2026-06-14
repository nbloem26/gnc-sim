'use client';

/**
 * Parameter form: builds a SimConfig from user inputs and fires onRun(config).
 * Defaults mirror configs/homing_3dof.json.
 */

import { useState } from 'react';
import type { SimConfig, ModelKind, Integrator } from '@/lib/types';

export interface ParamFormProps {
  onRun: (config: SimConfig) => void;
  running: boolean;
}

// Defaults from configs/homing_3dof.json.
interface FormState {
  model: ModelKind;
  integrator: Integrator;
  seed: number;
  dt: number;
  t_end: number;
  launch_speed: number;
  launch_elevation_deg: number;
  launch_azimuth_deg: number;
  mass0: number;
  nav_constant: number;
  max_accel: number;
  tgt_x: number;
  tgt_y: number;
  tgt_z: number;
  tgt_vx: number;
  tgt_vy: number;
  tgt_vz: number;
  sensors_enable: boolean;
  los_white: number;
  los_bias: number;
  glint: number;
}

const DEFAULTS: FormState = {
  model: '3dof',
  integrator: 'rk4',
  seed: 1,
  dt: 0.005,
  t_end: 40.0,
  launch_speed: 900,
  launch_elevation_deg: 42,
  launch_azimuth_deg: 0,
  mass0: 22.0,
  nav_constant: 4.0,
  max_accel: 400.0,
  tgt_x: 9000,
  tgt_y: 0,
  tgt_z: 3500,
  tgt_vx: -280,
  tgt_vy: 0,
  tgt_vz: -40,
  sensors_enable: false,
  los_white: 3.0e-3,
  los_bias: 5.0e-4,
  glint: 2.0,
};

const ORIGIN = { lat0_deg: 28.4889, lon0_deg: -80.5778, alt0_m: 0.0 };

function buildConfig(f: FormState): SimConfig {
  return {
    scenario: 'homing',
    model: f.model,
    seed: f.seed,
    dt: f.dt,
    t_end: f.t_end,
    integrator: f.integrator,
    origin: ORIGIN,
    env: { g0: 9.80665, altitude_dependent_g: false, atmosphere: true },
    aero: {
      ref_area: 0.02,
      cd0: 0.3,
      cd_mach: [
        [0.0, 0.28],
        [0.7, 0.3],
        [0.9, 0.42],
        [1.0, 0.58],
        [1.2, 0.52],
        [1.5, 0.43],
        [2.0, 0.36],
        [3.0, 0.3],
        [4.0, 0.27],
      ],
    },
    vehicle: {
      pos0: [0, 0, 0],
      launch_speed: f.launch_speed,
      launch_elevation_deg: f.launch_elevation_deg,
      launch_azimuth_deg: f.launch_azimuth_deg,
      mass0: f.mass0,
    },
    guidance: { law: 'pronav', nav_constant: f.nav_constant, max_accel: f.max_accel },
    sensors: f.sensors_enable
      ? {
          enable: true,
          seeker: { los_white: f.los_white, los_bias: f.los_bias, glint: f.glint },
        }
      : { enable: false },
    target: {
      pos0: [f.tgt_x, f.tgt_y, f.tgt_z],
      vel0: [f.tgt_vx, f.tgt_vy, f.tgt_vz],
      maneuver: 'constant',
    },
  };
}

type NumKeys = {
  [K in keyof FormState]: FormState[K] extends number ? K : never;
}[keyof FormState];

export default function ParamForm({ onRun, running }: ParamFormProps) {
  const [f, setF] = useState<FormState>(DEFAULTS);
  const [errors, setErrors] = useState<Partial<Record<keyof FormState, string>>>({});

  function setNum(key: NumKeys, raw: string) {
    const val = raw === '' ? NaN : Number(raw);
    setF((prev) => ({ ...prev, [key]: val }));
    setErrors((prev) => ({
      ...prev,
      [key]: Number.isFinite(val) ? undefined : 'must be a number',
    }));
  }

  function validate(): boolean {
    const errs: Partial<Record<keyof FormState, string>> = {};
    const numericKeys: NumKeys[] = [
      'seed', 'dt', 't_end', 'launch_speed', 'launch_elevation_deg',
      'launch_azimuth_deg', 'mass0', 'nav_constant', 'max_accel',
      'tgt_x', 'tgt_y', 'tgt_z', 'tgt_vx', 'tgt_vy', 'tgt_vz',
      'los_white', 'los_bias', 'glint',
    ];
    for (const k of numericKeys) {
      if (!Number.isFinite(f[k] as number)) errs[k] = 'must be a number';
    }
    if (Number.isFinite(f.dt) && f.dt <= 0) errs.dt = 'must be > 0';
    if (Number.isFinite(f.t_end) && f.t_end <= 0) errs.t_end = 'must be > 0';
    if (Number.isFinite(f.mass0) && f.mass0 <= 0) errs.mass0 = 'must be > 0';
    if (Number.isFinite(f.launch_speed) && f.launch_speed < 0)
      errs.launch_speed = 'must be ≥ 0';
    setErrors(errs);
    return Object.keys(errs).length === 0;
  }

  function handleRun() {
    if (!validate()) return;
    onRun(buildConfig(f));
  }

  function reset() {
    setF(DEFAULTS);
    setErrors({});
  }

  const numField = (
    label: string,
    key: NumKeys,
    step = 'any',
    unit?: string,
  ) => (
    <label className="field">
      <span>
        {label}
        {unit ? <em className="unit"> {unit}</em> : null}
      </span>
      <input
        type="number"
        step={step}
        value={Number.isFinite(f[key] as number) ? (f[key] as number) : ''}
        onChange={(e) => setNum(key, e.target.value)}
        className={errors[key] ? 'invalid' : ''}
      />
      {errors[key] ? <small className="err">{errors[key]}</small> : null}
    </label>
  );

  return (
    <form
      className="panel"
      onSubmit={(e) => {
        e.preventDefault();
        handleRun();
      }}
    >
      <h2>Scenario</h2>

      <fieldset>
        <legend>Model &amp; Integration</legend>
        <label className="field">
          <span>Model</span>
          <select
            value={f.model}
            onChange={(e) => setF({ ...f, model: e.target.value as ModelKind })}
          >
            <option value="3dof">3-DOF (point mass)</option>
            <option value="6dof">6-DOF (rigid body)</option>
          </select>
        </label>
        <label className="field">
          <span>Integrator</span>
          <select
            value={f.integrator}
            onChange={(e) =>
              setF({ ...f, integrator: e.target.value as Integrator })
            }
          >
            <option value="euler">Euler</option>
            <option value="rk2">RK2</option>
            <option value="rk4">RK4</option>
          </select>
        </label>
        <div className="row2">
          {numField('Seed', 'seed', '1')}
          {numField('dt', 'dt', '0.001', 's')}
        </div>
        {numField('t_end', 't_end', '1', 's')}
      </fieldset>

      <fieldset>
        <legend>Launch</legend>
        {numField('Launch speed', 'launch_speed', '10', 'm/s')}
        <div className="row2">
          {numField('Elevation', 'launch_elevation_deg', '1', 'deg')}
          {numField('Azimuth', 'launch_azimuth_deg', '1', 'deg')}
        </div>
        {numField('Mass', 'mass0', '0.5', 'kg')}
      </fieldset>

      <fieldset>
        <legend>Guidance (ProNav)</legend>
        {numField('Nav constant N', 'nav_constant', '0.1')}
        {numField('Max accel', 'max_accel', '10', 'm/s²')}
      </fieldset>

      <fieldset>
        <legend>Target — position [m]</legend>
        <div className="row3">
          {numField('x', 'tgt_x', '100')}
          {numField('y', 'tgt_y', '100')}
          {numField('z', 'tgt_z', '100')}
        </div>
        <legend style={{ marginTop: 8 }}>Target — velocity [m/s]</legend>
        <div className="row3">
          {numField('vx', 'tgt_vx', '10')}
          {numField('vy', 'tgt_vy', '10')}
          {numField('vz', 'tgt_vz', '10')}
        </div>
      </fieldset>

      <fieldset>
        <legend>Sensors</legend>
        <label className="check">
          <input
            type="checkbox"
            checked={f.sensors_enable}
            onChange={(e) => setF({ ...f, sensors_enable: e.target.checked })}
          />
          <span>Enable sensor models (IMU + seeker)</span>
        </label>
        {f.sensors_enable ? (
          <div className="nested">
            {numField('Seeker LOS white noise', 'los_white', '0.0001', 'rad')}
            {numField('Seeker LOS bias', 'los_bias', '0.0001', 'rad')}
            {numField('Glint', 'glint', '0.1')}
          </div>
        ) : null}
      </fieldset>

      <div className="actions">
        <button type="submit" className="primary" disabled={running}>
          {running ? 'Running…' : 'Run simulation'}
        </button>
        <button type="button" onClick={reset} disabled={running}>
          Reset
        </button>
      </div>
    </form>
  );
}
