'use client';

/**
 * 3D Globe (issue #48 / #27): a CesiumJS globe that replays the engagement.
 *
 * It consumes the same `SimResult.series` as every other panel (LIVE-WASM or the
 * committed mock sample), projects the interceptor & threat ENU tracks to
 * geodetic lat/lon/altitude about the result origin, and drives time-dynamic
 * entities via `SampledPositionProperty` over the Cesium clock — so the built-in
 * animation/timeline widgets give play / pause / scrub for free.
 *
 * Tokenless by design: imagery comes from OpenStreetMap tiles (no Cesium ion
 * token), terrain is the plain WGS-84 ellipsoid, and Cesium's runtime assets are
 * served from `/cesium` (copied out of node_modules at build time — see
 * scripts/copy-cesium-assets.mjs). Nothing here requires `Cesium.Ion.defaultAccessToken`.
 *
 * This whole module is heavy (~Cesium). It is mounted ONLY via the lazy
 * `dynamic(ssr:false)` import on the active "3D Globe" tab (app/page.tsx), so the
 * Cesium bundle never lands in the first-load JS or under any other tab.
 */

import { useEffect, useRef } from 'react';
import 'cesium/Build/Cesium/Widgets/widgets.css';
import type { SimResult, SimSeries } from '@/lib/types';
import { enuToLatLonAlt } from '@/lib/enuToGeodetic';

// Cesium needs to know where its Workers/Assets/Widgets/ThirdParty live before
// the module wires up its worker pool. Must be set before the Viewer is built.
function ensureCesiumBaseUrl() {
  if (typeof window === 'undefined') return;
  const w = window as unknown as { CESIUM_BASE_URL?: string };
  if (!w.CESIUM_BASE_URL) w.CESIUM_BASE_URL = '/cesium';
}

interface TrackStyle {
  /** Cesium CSS color name resolved via Color.fromCssColorString. */
  cssColor: string;
  label: string;
}

const VEH_STYLE: TrackStyle = { cssColor: '#4fd1c5', label: 'Interceptor' };
const TGT_STYLE: TrackStyle = { cssColor: '#f6ad55', label: 'Threat' };

/** Cap entity samples so very long runs stay responsive on the globe. */
function sampleStride(n: number, maxSamples = 1200): number {
  return n <= maxSamples ? 1 : Math.ceil(n / maxSamples);
}

export default function CesiumGlobe({ result }: { result: SimResult }) {
  const containerRef = useRef<HTMLDivElement | null>(null);
  // Hold the live Viewer so the effect cleanup can destroy it on unmount.
  const viewerRef = useRef<{ destroy: () => void; isDestroyed: () => boolean } | null>(
    null,
  );

  useEffect(() => {
    let cancelled = false;
    ensureCesiumBaseUrl();

    // Dynamic import keeps Cesium out of the synchronous module graph; combined
    // with the lazy tab mount it lands only in this code-split chunk.
    import('cesium')
      .then((Cesium) => {
        if (cancelled || !containerRef.current) return;

        const {
          Viewer,
          OpenStreetMapImageryProvider,
          ImageryLayer,
          Ellipsoid,
          JulianDate,
          ClockRange,
          SampledPositionProperty,
          Cartesian3,
          Color,
          PolylineGlowMaterialProperty,
          PathGraphics,
          TimeIntervalCollection,
          TimeInterval,
          HeightReference,
        } = Cesium;

        const s: SimSeries = result.series;
        const { lat0_deg, lon0_deg, alt0_m } = result.origin;
        const n = Math.min(s.t.length, s.veh_x.length, s.tgt_x.length);
        if (n === 0) return;

        // Tokenless imagery: OpenStreetMap raster tiles. No ion token, no terrain
        // server — the default WGS-84 ellipsoid stands in for terrain.
        const osm = ImageryLayer.fromProviderAsync(
          Promise.resolve(
            new OpenStreetMapImageryProvider({
              url: 'https://tile.openstreetmap.org/',
            }),
          ),
          {},
        );

        const viewer = new Viewer(containerRef.current, {
          baseLayer: osm,
          baseLayerPicker: false, // would offer ion-backed layers — keep tokenless
          geocoder: false, // ion-backed — disable
          terrainProvider: undefined, // plain ellipsoid
          ellipsoid: Ellipsoid.WGS84,
          animation: true, // play/pause clock widget
          timeline: true, // scrub widget
          fullscreenButton: false,
          homeButton: false,
          sceneModePicker: true,
          navigationHelpButton: false,
          infoBox: false,
          selectionIndicator: false,
        });
        viewerRef.current = viewer;
        // Mute the default "ion token" credit nag; we use none.
        viewer.scene.globe.enableLighting = false;

        const stride = sampleStride(n);

        const start = JulianDate.fromDate(new Date('2026-01-01T00:00:00Z'));
        const t0 = s.t[0];
        const tEnd = s.t[n - 1];
        const stop = JulianDate.addSeconds(start, tEnd - t0, new JulianDate());

        viewer.clock.startTime = start.clone();
        viewer.clock.stopTime = stop.clone();
        viewer.clock.currentTime = start.clone();
        viewer.clock.clockRange = ClockRange.LOOP_STOP;
        viewer.clock.multiplier = 1;
        viewer.timeline.zoomTo(start, stop);

        // Build a time-sampled position track from an ENU series.
        function buildTrack(
          xs: number[],
          ys: number[],
          zs: number[],
        ): InstanceType<typeof SampledPositionProperty> {
          const prop = new SampledPositionProperty();
          for (let i = 0; i < n; i += stride) {
            const when = JulianDate.addSeconds(
              start,
              s.t[i] - t0,
              new JulianDate(),
            );
            const g = enuToLatLonAlt(
              xs[i],
              ys[i],
              zs[i],
              lat0_deg,
              lon0_deg,
              alt0_m,
            );
            prop.addSample(
              when,
              Cartesian3.fromDegrees(g.lon_deg, g.lat_deg, g.alt_m),
            );
          }
          // Always include the final sample so the track ends exactly at impact.
          if ((n - 1) % stride !== 0) {
            const last = n - 1;
            const when = JulianDate.addSeconds(
              start,
              s.t[last] - t0,
              new JulianDate(),
            );
            const g = enuToLatLonAlt(
              xs[last],
              ys[last],
              zs[last],
              lat0_deg,
              lon0_deg,
              alt0_m,
            );
            prop.addSample(
              when,
              Cartesian3.fromDegrees(g.lon_deg, g.lat_deg, g.alt_m),
            );
          }
          return prop;
        }

        const availability = new TimeIntervalCollection([
          new TimeInterval({ start, stop }),
        ]);

        function addEntity(
          xs: number[],
          ys: number[],
          zs: number[],
          style: TrackStyle,
        ) {
          const color = Color.fromCssColorString(style.cssColor);
          const position = buildTrack(xs, ys, zs);
          viewer.entities.add({
            name: style.label,
            availability,
            position,
            point: {
              pixelSize: 10,
              color,
              outlineColor: Color.BLACK,
              outlineWidth: 1,
              heightReference: HeightReference.NONE,
            },
            // Time-dynamic trailing flight path.
            path: new PathGraphics({
              resolution: 1,
              width: 3,
              leadTime: 0,
              trailTime: tEnd - t0,
              material: new PolylineGlowMaterialProperty({
                glowPower: 0.2,
                color,
              }),
            }),
          });
        }

        addEntity(s.veh_x, s.veh_y, s.veh_z, VEH_STYLE);
        addEntity(s.tgt_x, s.tgt_y, s.tgt_z, TGT_STYLE);

        // Mark the intercept / closest-approach point with a static billboard-free pin.
        const last = n - 1;
        const impact = enuToLatLonAlt(
          s.veh_x[last],
          s.veh_y[last],
          s.veh_z[last],
          lat0_deg,
          lon0_deg,
          alt0_m,
        );
        viewer.entities.add({
          name: result.intercept ? 'Intercept' : 'Closest approach',
          position: Cartesian3.fromDegrees(
            impact.lon_deg,
            impact.lat_deg,
            impact.alt_m,
          ),
          point: {
            pixelSize: 12,
            color: Color.fromCssColorString('#fc8181'),
            outlineColor: Color.WHITE,
            outlineWidth: 2,
          },
        });

        // Frame the engagement and start playback paused at t0.
        viewer.zoomTo(viewer.entities).catch(() => {
          /* zoomTo rejects if cancelled by a quick unmount — ignore */
        });
        viewer.clock.shouldAnimate = false;
      })
      .catch((err) => {
        // Surface load failures (e.g. assets not served) without crashing React.
        console.error('[CesiumGlobe] failed to initialize:', err);
      });

    return () => {
      cancelled = true;
      const v = viewerRef.current;
      if (v && !v.isDestroyed()) v.destroy();
      viewerRef.current = null;
    };
  }, [result]);

  return (
    <div>
      <h3 style={{ margin: '0 0 4px' }}>3D Globe — Engagement Playback</h3>
      <div
        ref={containerRef}
        style={{
          height: 520,
          borderRadius: 8,
          overflow: 'hidden',
          border: '1px solid #1f2a36',
          background: '#0f141b',
        }}
      />
      <p className="muted" style={{ marginTop: 6 }}>
        Interceptor (teal) vs threat (orange), replayed on the Cesium clock —
        play / pause / scrub with the controls. Tokenless OpenStreetMap imagery on
        the WGS-84 ellipsoid; no Cesium ion token required. Geodetic positions are
        an ENU→lat/lon/alt projection about origin {lat0Fmt(result)}.
      </p>
    </div>
  );
}

function lat0Fmt(result: SimResult): string {
  return `${result.origin.lat0_deg.toFixed(4)}°, ${result.origin.lon0_deg.toFixed(4)}°`;
}
