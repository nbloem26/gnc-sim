/**
 * Local-tangent-plane ENU -> geodetic projection (DATA_CONTRACT §6).
 *
 * The C++ core stays in local ENU metres. This is a web-map-only equirectangular
 * small-area approximation about a geodetic origin {lat0, lon0}:
 *
 *   dLat = northing / 111320
 *   dLon = easting  / (111320 * cos(lat0))
 *
 * ENU axes: x = East, y = North, z = Up.
 */

import type { LatLon } from './types';

const M_PER_DEG = 111320; // metres per degree of latitude (mean)

/**
 * Project a single ENU point (metres) to lat/lon (degrees) about an origin.
 * @param east   ENU x [m]
 * @param north  ENU y [m]
 * @param lat0   origin latitude [deg]
 * @param lon0   origin longitude [deg]
 */
export function enuToLatLon(
  east: number,
  north: number,
  lat0: number,
  lon0: number,
): LatLon {
  const cosLat0 = Math.cos((lat0 * Math.PI) / 180);
  const dLat = north / M_PER_DEG;
  const dLon = east / (M_PER_DEG * (Math.abs(cosLat0) < 1e-9 ? 1e-9 : cosLat0));
  return { lat: lat0 + dLat, lon: lon0 + dLon };
}

/**
 * Project parallel East/North arrays to an array of [lat, lon] tuples,
 * suitable for a Leaflet Polyline.
 */
export function enuPathToLatLngs(
  east: number[],
  north: number[],
  lat0: number,
  lon0: number,
): [number, number][] {
  const n = Math.min(east.length, north.length);
  const out: [number, number][] = new Array(n);
  for (let i = 0; i < n; i++) {
    const { lat, lon } = enuToLatLon(east[i], north[i], lat0, lon0);
    out[i] = [lat, lon];
  }
  return out;
}
