/**
 * TypeScript types mirroring the GNC sim data contract.
 * Source of truth: docs/DATA_CONTRACT.md (§1 input config, §2 result JSON).
 */

// ----------------------------------------------------------------------------
// Input config (§1)
// ----------------------------------------------------------------------------

export type ModelKind = '3dof' | '6dof';
export type Integrator = 'euler' | 'rk2' | 'rk4';

export interface OriginConfig {
  lat0_deg: number;
  lon0_deg: number;
  alt0_m: number;
}

export interface EnvConfig {
  g0: number;
  altitude_dependent_g: boolean;
  atmosphere: boolean;
}

export interface SeekerConfig {
  los_white: number;
  los_bias: number;
  glint: number;
}

export interface ImuConfig {
  accel_white?: number;
  accel_bias_instability?: number;
  accel_bias_tau?: number;
  accel_rrw?: number;
  accel_scale_factor?: number;
  gyro_white?: number;
  gyro_bias_instability?: number;
  gyro_bias_tau?: number;
  gyro_rrw?: number;
  gyro_scale_factor?: number;
}

export interface SensorsConfig {
  enable: boolean;
  imu?: ImuConfig;
  seeker?: SeekerConfig;
}

export interface VehicleConfig {
  pos0: [number, number, number];
  launch_speed: number;
  launch_elevation_deg: number;
  launch_azimuth_deg: number;
  mass0: number;
  inertia?: number;
}

export interface GuidanceConfig {
  law: string;
  nav_constant: number;
  max_accel: number;
}

export interface ControlConfig {
  kp: number;
  kd: number;
  max_fin_deflection: number;
}

export interface TargetConfig {
  pos0: [number, number, number];
  vel0: [number, number, number];
  maneuver: string;
  maneuver_g?: number;
  maneuver_freq?: number;
  /** Phase offset [deg] into the weave maneuver cycle. Randomized per Monte Carlo case. */
  maneuver_phase_deg?: number;
}

export interface MonteCarloConfig {
  num_cases: number;
  launch_speed_sigma: number;
  launch_elevation_sigma_deg: number;
  target_pos_sigma: number;
}

export interface SimConfig {
  scenario: string;
  model: ModelKind;
  seed: number;
  dt: number;
  t_end: number;
  integrator: Integrator;
  origin: OriginConfig;
  env?: EnvConfig;
  aero?: Record<string, unknown>;
  vehicle: VehicleConfig;
  guidance: GuidanceConfig;
  control?: ControlConfig;
  sensors: SensorsConfig;
  target: TargetConfig;
  monte_carlo?: MonteCarloConfig;
  // Opt-in subsystem blocks carried verbatim from the canonical configs/*.json
  // presets (EKF nav, multi-sensor fusion, decoy discrimination, launch cueing).
  // The web form does not edit these; they pass straight through to the core.
  schema_version?: number;
  nav?: Record<string, unknown>;
  trackers?: Record<string, unknown>;
  decoys?: Record<string, unknown>;
  cueing?: Record<string, unknown>;
}

// ----------------------------------------------------------------------------
// Output result (§2)
// ----------------------------------------------------------------------------

/** Columnar time-series channels. Every array shares the same length as `t`. */
export interface SimSeries {
  t: number[];
  veh_x: number[];
  veh_y: number[];
  veh_z: number[];
  veh_vx: number[];
  veh_vy: number[];
  veh_vz: number[];
  roll: number[];
  pitch: number[];
  yaw: number[];
  mass: number[];
  mach: number[];
  /** Motor thrust [N]. Present on boosting configs; `0` on coast/no-motor paths. */
  thrust?: number[];
  tgt_x: number[];
  tgt_y: number[];
  tgt_z: number[];
  tgt_vx: number[];
  tgt_vy: number[];
  tgt_vz: number[];
  accel_cmd_x: number[];
  accel_cmd_y: number[];
  accel_cmd_z: number[];
  los_angle: number[];
  los_rate: number[];
  v_closing: number[];
  range: number[];
  nav_x: number[];
  nav_y: number[];
  nav_z: number[];
  imu_accel_true_x?: number[];
  imu_accel_meas_x?: number[];
  imu_gyro_true_x?: number[];
  imu_gyro_meas_x?: number[];
  seeker_los_true?: number[];
  seeker_los_meas?: number[];
  /** EKF normalized innovation squared. Present only when an EKF filter is run. */
  nav_nis?: number[];
  /**
   * Fused multi-sensor target-track position estimate [m] (ENU). Populated only
   * when `trackers.enabled`; `0` on every non-tracker path.
   */
  track_x?: number[];
  track_y?: number[];
  track_z?: number[];
  /** Last fused sensor update's NIS. `0` on non-tracker paths. */
  track_nis?: number[];
  /** Seeker decoy-discrimination channels. Meaningful only when `decoys.enabled`. */
  selected_obj?: number[];
  discrim_correct?: number[];
  discrim_margin?: number[];
}

export interface SimResult {
  scenario: string;
  model: ModelKind;
  seed: number;
  dt: number;
  t_end: number;
  intercept: boolean;
  miss_distance: number;
  intercept_time: number;
  /** Interceptor cue/launch time [s]. `0` on the default launch-at-t=0 path. */
  launch_time?: number;
  git_sha: string;
  origin: OriginConfig;
  series: SimSeries;
}

// ----------------------------------------------------------------------------
// Scenario presets (web/public/scenarios/index.json)
// ----------------------------------------------------------------------------

export interface ScenarioPreset {
  id: string;
  file: string;
  label: string;
  description: string;
}

export interface ScenarioManifest {
  presets: ScenarioPreset[];
}

/** Shape returned by the WASM entry on failure. */
export interface SimError {
  error: string;
}

export function isSimError(x: unknown): x is SimError {
  return typeof x === 'object' && x !== null && 'error' in x;
}

// ----------------------------------------------------------------------------
// Geodetic helpers
// ----------------------------------------------------------------------------

export interface LatLon {
  lat: number;
  lon: number;
}
