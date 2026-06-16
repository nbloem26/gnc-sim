/**
 * Scenario-authoring helpers (issue #120).
 *
 * Pure, framework-free functions that turn entities placed on the Cesium globe
 * into a runnable SimConfig. The geometry is the exact INVERSE of the web-map
 * ENU->geodetic projection in `enuToGeodetic.ts` (DATA_CONTRACT §6 small-area
 * equirectangular approximation), so a position authored on the globe and then
 * replayed by the globe round-trips to the same lat/lon.
 *
 *   forward (enuToGeodetic):  dLat = north / 111320
 *                             dLon = east  / (111320 * cos(lat0))
 *   inverse (here):           north = dLat * 111320
 *                             east  = dLon * 111320 * cos(lat0)
 *
 * ENU axes: x = East, y = North, z = Up (metres). The origin is the launch
 * site's geodetic {lat0_deg, lon0_deg, alt0_m}; the threat's pos0 is ENU metres
 * relative to that origin.
 */

import type { SimConfig, OriginConfig } from './types';

const M_PER_DEG = 111320; // metres per degree of latitude (mean) — matches enuToGeodetic

/** A geodetic point picked on the globe. */
export interface GeodeticPoint {
  lat_deg: number;
  lon_deg: number;
  alt_m: number;
}

/**
 * Inverse of `enuToLatLonAlt`: project a geodetic point to local-tangent-plane
 * ENU metres about a geodetic origin. Returns [east, north, up].
 */
export function geodeticToEnu(
  p: GeodeticPoint,
  origin: OriginConfig,
): [number, number, number] {
  const cosLat0 = Math.cos((origin.lat0_deg * Math.PI) / 180);
  const safeCos = Math.abs(cosLat0) < 1e-9 ? 1e-9 : cosLat0;
  const north = (p.lat_deg - origin.lat0_deg) * M_PER_DEG;
  const east = (p.lon_deg - origin.lon0_deg) * M_PER_DEG * safeCos;
  const up = p.alt_m - origin.alt0_m;
  return [east, north, up];
}

/** The visually-authorable knobs collected from the globe UI. */
export interface AuthoredScenario {
  /** Launch site -> geodetic origin. */
  launch: GeodeticPoint;
  /** Threat geodetic position -> target.pos0 (ENU about origin). */
  threat: GeodeticPoint;
  /** Optional sensor position with a coverage dome (visual only; not in core). */
  sensor?: GeodeticPoint;
  /** Sensor coverage range [m] (dome radius) — visual only. */
  sensorRange_m?: number;
  // Inline-editable numbers:
  launch_speed_m_s: number;
  launch_elevation_deg: number;
  launch_azimuth_deg: number;
  /** Threat speed [m/s] along a horizontal heading (bearing from North, CW). */
  threat_speed_m_s: number;
  /** Threat ground heading [deg], clockwise from North. */
  threat_heading_deg: number;
  /** Threat vertical velocity [m/s], +up. */
  threat_climb_m_s: number;
  /** Threat maneuver kind passed through to target.maneuver. */
  threat_maneuver: string;
}

/**
 * Convert an authored-threat horizontal speed/heading + climb rate into an ENU
 * velocity triple [vx_east, vy_north, vz_up], in m/s.
 */
export function threatVelocityEnu(
  speed_m_s: number,
  heading_deg: number,
  climb_m_s: number,
): [number, number, number] {
  const hdg = (heading_deg * Math.PI) / 180;
  // Heading is clockwise from North: East = sin, North = cos.
  const vx_east = speed_m_s * Math.sin(hdg);
  const vy_north = speed_m_s * Math.cos(hdg);
  return [vx_east, vy_north, climb_m_s];
}

/**
 * Build a runnable SimConfig from an authored scenario. `base` is an optional
 * preset SimConfig whose non-authored blocks (env/aero/guidance.law/sensors/
 * nav/trackers/... and other contract fields) pass through verbatim — exactly
 * like ParamForm.buildConfig. When `base` is null a minimal homing config is
 * synthesized so authoring works offline / in mock mode.
 */
export function buildAuthoredConfig(
  s: AuthoredScenario,
  base: SimConfig | null,
): SimConfig {
  const origin: OriginConfig = {
    lat0_deg: s.launch.lat_deg,
    lon0_deg: s.launch.lon_deg,
    alt0_m: s.launch.alt_m,
  };
  const pos0 = geodeticToEnu(s.threat, origin);
  const vel0 = threatVelocityEnu(
    s.threat_speed_m_s,
    s.threat_heading_deg,
    s.threat_climb_m_s,
  );

  const src: SimConfig = base ?? {
    scenario: 'homing',
    model: '3dof',
    seed: 1,
    dt: 0.005,
    t_end: 40.0,
    integrator: 'rk4',
    origin,
    env: { g0: 9.80665, altitude_dependent_g: false, atmosphere: true },
    vehicle: {
      pos0: [0, 0, 0],
      launch_speed: 900,
      launch_elevation_deg: 42,
      launch_azimuth_deg: 0,
      mass0: 22.0,
    },
    guidance: { law: 'pronav', nav_constant: 4.0, max_accel: 400.0 },
    sensors: { enable: false },
    target: { pos0, vel0, maneuver: 'constant' },
  };

  return {
    ...src,
    origin,
    vehicle: {
      ...src.vehicle,
      // The interceptor always launches from the origin.
      pos0: [0, 0, 0],
      launch_speed: s.launch_speed_m_s,
      launch_elevation_deg: s.launch_elevation_deg,
      launch_azimuth_deg: s.launch_azimuth_deg,
    },
    target: {
      ...src.target,
      pos0,
      vel0,
      maneuver: s.threat_maneuver,
    },
  };
}

/** Derive an AuthoredScenario seeded from a preset SimConfig (round-trip start). */
export function authoredFromConfig(cfg: SimConfig): AuthoredScenario {
  const origin = cfg.origin;
  // Place the threat on the globe at the preset's ENU pos0 about the origin.
  // (The component re-projects pos0 -> lat/lon for display.)
  const v = cfg.target?.vel0 ?? [0, 0, 0];
  const speed = Math.hypot(v[0], v[1]);
  // Inverse of threatVelocityEnu heading: atan2(east, north).
  const heading_deg =
    speed > 1e-6 ? (Math.atan2(v[0], v[1]) * 180) / Math.PI : 0;
  return {
    launch: {
      lat_deg: origin.lat0_deg,
      lon_deg: origin.lon0_deg,
      alt_m: origin.alt0_m,
    },
    // threat geodetic is filled in by the component from pos0; placeholder here.
    threat: {
      lat_deg: origin.lat0_deg,
      lon_deg: origin.lon0_deg,
      alt_m: origin.alt0_m,
    },
    launch_speed_m_s: cfg.vehicle?.launch_speed ?? 900,
    launch_elevation_deg: cfg.vehicle?.launch_elevation_deg ?? 42,
    launch_azimuth_deg: cfg.vehicle?.launch_azimuth_deg ?? 0,
    threat_speed_m_s: speed,
    threat_heading_deg: (heading_deg + 360) % 360,
    threat_climb_m_s: v[2] ?? 0,
    threat_maneuver: cfg.target?.maneuver ?? 'constant',
  };
}
