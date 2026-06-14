'use client';

/**
 * Ground track: projects vehicle & target ENU paths to lat/lon about the result
 * origin (DATA_CONTRACT §6) and draws both tracks + an intercept marker on a
 * Leaflet map.
 *
 * react-leaflet / leaflet touch `window`, so the whole map is dynamically
 * imported with ssr:false and rendered only on the client.
 */

import { useMemo } from 'react';
import dynamic from 'next/dynamic';
import 'leaflet/dist/leaflet.css';
import type { SimResult } from '@/lib/types';
import { enuPathToLatLngs, enuToLatLon } from '@/lib/enuToGeodetic';

// Dynamically import the react-leaflet primitives (no SSR).
const MapContainer = dynamic(
  () => import('react-leaflet').then((m) => m.MapContainer),
  { ssr: false },
);
const TileLayer = dynamic(() => import('react-leaflet').then((m) => m.TileLayer), {
  ssr: false,
});
const Polyline = dynamic(() => import('react-leaflet').then((m) => m.Polyline), {
  ssr: false,
});
const CircleMarker = dynamic(
  () => import('react-leaflet').then((m) => m.CircleMarker),
  { ssr: false },
);
const Tooltip = dynamic(() => import('react-leaflet').then((m) => m.Tooltip), {
  ssr: false,
});

function decimate(pts: [number, number][], maxPts = 1500): [number, number][] {
  if (pts.length <= maxPts) return pts;
  const step = Math.ceil(pts.length / maxPts);
  const out: [number, number][] = [];
  for (let i = 0; i < pts.length; i += step) out.push(pts[i]);
  out.push(pts[pts.length - 1]);
  return out;
}

export default function GroundTrack({ result }: { result: SimResult }) {
  const { lat0_deg, lon0_deg } = result.origin;
  const s = result.series;

  const vehTrack = useMemo(
    () => decimate(enuPathToLatLngs(s.veh_x, s.veh_y, lat0_deg, lon0_deg)),
    [s.veh_x, s.veh_y, lat0_deg, lon0_deg],
  );
  const tgtTrack = useMemo(
    () => decimate(enuPathToLatLngs(s.tgt_x, s.tgt_y, lat0_deg, lon0_deg)),
    [s.tgt_x, s.tgt_y, lat0_deg, lon0_deg],
  );

  const last = s.t.length - 1;
  const intercept = enuToLatLon(s.veh_x[last], s.veh_y[last], lat0_deg, lon0_deg);
  const center: [number, number] = [intercept.lat, intercept.lon];

  return (
    <div>
      <h3 style={{ margin: '0 0 4px' }}>Ground Track</h3>
      <div
        style={{
          height: 360,
          borderRadius: 8,
          overflow: 'hidden',
          border: '1px solid #1f2a36',
        }}
      >
        <MapContainer
          center={center}
          zoom={12}
          scrollWheelZoom
          style={{ height: '100%', width: '100%', background: '#0f141b' }}
        >
          <TileLayer
            attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> &amp; CARTO'
            url="https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png"
          />
          <Polyline positions={tgtTrack} pathOptions={{ color: '#f6ad55', weight: 3, dashArray: '6 6' }}>
            <Tooltip sticky>Target</Tooltip>
          </Polyline>
          <Polyline positions={vehTrack} pathOptions={{ color: '#4fd1c5', weight: 3 }}>
            <Tooltip sticky>Vehicle</Tooltip>
          </Polyline>
          <CircleMarker
            center={center}
            radius={7}
            pathOptions={{ color: '#fc8181', fillColor: '#fc8181', fillOpacity: 0.9 }}
          >
            <Tooltip permanent direction="top">
              {result.intercept ? 'Intercept' : 'Closest approach'}
            </Tooltip>
          </CircleMarker>
        </MapContainer>
      </div>
      <p className="muted" style={{ marginTop: 6 }}>
        ENU projected about origin {lat0_deg.toFixed(4)}°, {lon0_deg.toFixed(4)}°
        (local tangent plane).
      </p>
    </div>
  );
}
