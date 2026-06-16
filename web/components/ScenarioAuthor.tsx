'use client';

/**
 * Scenario authoring on the Cesium globe (issue #120).
 *
 * An interactive authoring surface: the user clicks the globe to place the
 * LAUNCH SITE (which becomes the geodetic origin), one THREAT, and optionally a
 * SENSOR with a translucent coverage dome. Inline number fields edit launch
 * speed/elevation/azimuth, threat speed/heading/climb, threat maneuver, and
 * sensor range. "Generate" turns the placement into a runnable SimConfig (via
 * lib/scenarioAuthor — the exact inverse of the globe's ENU->geodetic
 * projection), which can be run immediately (handed to the simulator surface) or
 * copied as JSON.
 *
 * Tokenless like CesiumGlobe: OpenStreetMap imagery, plain WGS-84 ellipsoid, no
 * ion token. This module is heavy (~Cesium) and is mounted ONLY via the lazy
 * `dynamic(ssr:false)` import on the active "Author" surface (app/page.tsx), so
 * the Cesium bundle never lands in the first-load JS or under any other tab.
 *
 * The Cesium runtime + globe playback (CesiumGlobe.tsx) are left untouched; this
 * is an additive authoring mode that shares the same dynamic-import strategy.
 */

import { useCallback, useEffect, useRef, useState } from 'react';
import 'cesium/Build/Cesium/Widgets/widgets.css';
import type { SimConfig, ScenarioPreset } from '@/lib/types';
import { loadPresetManifest, loadPresetConfig } from '@/lib/presets';
import { enuToLatLonAlt } from '@/lib/enuToGeodetic';
import {
  buildAuthoredConfig,
  authoredFromConfig,
  geodeticToEnu,
  type AuthoredScenario,
  type GeodeticPoint,
} from '@/lib/scenarioAuthor';

function ensureCesiumBaseUrl() {
  if (typeof window === 'undefined') return;
  const w = window as unknown as { CESIUM_BASE_URL?: string };
  if (!w.CESIUM_BASE_URL) w.CESIUM_BASE_URL = '/cesium';
}

export interface ScenarioAuthorProps {
  /** Hand the authored SimConfig to the simulator surface and run it. */
  onRun: (config: SimConfig) => void;
  running: boolean;
}

type PlaceMode = 'launch' | 'threat' | 'sensor';

const THREAT_MANEUVERS = ['constant', 'weave', 'barrel', 'jink'];

/** Default scenario when authoring from scratch (no preset). */
const DEFAULT_SCENARIO: AuthoredScenario = {
  launch: { lat_deg: 28.4889, lon_deg: -80.5778, alt_m: 0 },
  threat: { lat_deg: 28.52, lon_deg: -80.49, alt_m: 3500 },
  sensor: undefined,
  sensorRange_m: 15000,
  launch_speed_m_s: 900,
  launch_elevation_deg: 42,
  launch_azimuth_deg: 0,
  threat_speed_m_s: 280,
  threat_heading_deg: 225,
  threat_climb_m_s: -40,
  threat_maneuver: 'constant',
};

export default function ScenarioAuthor({ onRun, running }: ScenarioAuthorProps) {
  const containerRef = useRef<HTMLDivElement | null>(null);
  type CesiumModule = typeof import('cesium');
  type CesiumViewer = InstanceType<CesiumModule['Viewer']>;
  type CesiumEventHandler = InstanceType<CesiumModule['ScreenSpaceEventHandler']>;
  const viewerRef = useRef<CesiumViewer | null>(null);
  // The screen-space click handler, destroyed alongside the viewer.
  const handlerRef = useRef<CesiumEventHandler | null>(null);
  // Cesium module + helpers captured after dynamic import, used by handlers.
  const cesiumRef = useRef<CesiumModule | null>(null);
  const redrawRef = useRef<(() => void) | null>(null);

  const [ready, setReady] = useState(false);
  const [placeMode, setPlaceMode] = useState<PlaceMode>('launch');
  const [scenario, setScenario] = useState<AuthoredScenario>(DEFAULT_SCENARIO);
  // Preset base whose non-authored blocks pass through into the generated config.
  const [presets, setPresets] = useState<ScenarioPreset[]>([]);
  const [presetFile, setPresetFile] = useState('');
  const [baseConfig, setBaseConfig] = useState<SimConfig | null>(null);
  const [json, setJson] = useState<string>('');
  const [copied, setCopied] = useState(false);

  // Keep a ref to the latest scenario so Cesium click handlers (registered once)
  // always see current state without re-binding listeners.
  const scenarioRef = useRef(scenario);
  scenarioRef.current = scenario;
  const placeModeRef = useRef(placeMode);
  placeModeRef.current = placeMode;

  // Load presets for the "start from a preset" round-trip.
  useEffect(() => {
    let alive = true;
    loadPresetManifest()
      .then((list) => {
        if (alive) setPresets(list);
      })
      .catch(() => {
        /* offline/mock: no presets, author from scratch */
      });
    return () => {
      alive = false;
    };
  }, []);

  /** Redraw all authored entities from the current scenario. */
  const redraw = useCallback(() => {
    const Cesium = cesiumRef.current;
    const v = viewerRef.current;
    if (!Cesium || !v) return;
    const { Cartesian3, Color } = Cesium;
    v.entities.removeAll();
    const s = scenarioRef.current;

    // Launch site (origin).
    v.entities.add({
      name: 'Launch site (origin)',
      position: Cartesian3.fromDegrees(
        s.launch.lon_deg,
        s.launch.lat_deg,
        s.launch.alt_m,
      ),
      point: {
        pixelSize: 13,
        color: Color.fromCssColorString('#4fd1c5'),
        outlineColor: Color.BLACK,
        outlineWidth: 2,
      },
      label: {
        text: 'Launch',
        font: '12px sans-serif',
        fillColor: Color.WHITE,
        showBackground: true,
        backgroundColor: Color.fromCssColorString('#0f141bcc'),
        pixelOffset: new Cartesian3(0, -22, 0),
      },
    });

    // Threat.
    v.entities.add({
      name: 'Threat',
      position: Cartesian3.fromDegrees(
        s.threat.lon_deg,
        s.threat.lat_deg,
        s.threat.alt_m,
      ),
      point: {
        pixelSize: 13,
        color: Color.fromCssColorString('#f6ad55'),
        outlineColor: Color.BLACK,
        outlineWidth: 2,
      },
      label: {
        text: 'Threat',
        font: '12px sans-serif',
        fillColor: Color.WHITE,
        showBackground: true,
        backgroundColor: Color.fromCssColorString('#0f141bcc'),
        pixelOffset: new Cartesian3(0, -22, 0),
      },
    });

    // Optional sensor + translucent coverage dome.
    if (s.sensor) {
      const range = Math.max(100, s.sensorRange_m ?? 15000);
      v.entities.add({
        name: 'Sensor',
        position: Cartesian3.fromDegrees(
          s.sensor.lon_deg,
          s.sensor.lat_deg,
          s.sensor.alt_m,
        ),
        point: {
          pixelSize: 12,
          color: Color.fromCssColorString('#a78bfa'),
          outlineColor: Color.BLACK,
          outlineWidth: 2,
        },
        label: {
          text: 'Sensor',
          font: '12px sans-serif',
          fillColor: Color.WHITE,
          showBackground: true,
          backgroundColor: Color.fromCssColorString('#0f141bcc'),
          pixelOffset: new Cartesian3(0, -22, 0),
        },
        // Translucent hemispherical coverage volume (dome).
        ellipsoid: {
          radii: new Cartesian3(range, range, range),
          maximumCone: Math.PI / 2, // upper hemisphere
          material: Color.fromCssColorString('#a78bfa').withAlpha(0.18),
          outline: true,
          outlineColor: Color.fromCssColorString('#a78bfa').withAlpha(0.5),
        },
      });
    }
  }, []);
  redrawRef.current = redraw;

  // Mount the Cesium viewer once.
  useEffect(() => {
    let cancelled = false;
    ensureCesiumBaseUrl();

    import('cesium')
      .then((Cesium) => {
        if (cancelled || !containerRef.current) return;
        cesiumRef.current = Cesium;
        const {
          Viewer,
          OpenStreetMapImageryProvider,
          ImageryLayer,
          Ellipsoid,
          ScreenSpaceEventHandler,
          ScreenSpaceEventType,
        } = Cesium;

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
          baseLayerPicker: false,
          geocoder: false,
          terrainProvider: undefined,
          ellipsoid: Ellipsoid.WGS84,
          animation: false,
          timeline: false,
          fullscreenButton: false,
          homeButton: false,
          sceneModePicker: true,
          navigationHelpButton: false,
          infoBox: false,
          selectionIndicator: false,
        });
        viewerRef.current = viewer;
        viewer.scene.globe.enableLighting = false;

        // Click-to-place: convert the picked screen point to geodetic and write
        // it into the active slot (launch / threat / sensor).
        const handler = new ScreenSpaceEventHandler(viewer.scene.canvas);
        handlerRef.current = handler;
        handler.setInputAction(
          (movement: { position: InstanceType<CesiumModule['Cartesian2']> }) => {
            const cartesian = viewer.scene.pickPosition
              ? viewer.scene.pickPosition(movement.position)
              : undefined;
            const ray = viewer.camera.getPickRay(movement.position);
            const globePt =
              cartesian ??
              (ray ? viewer.scene.globe.pick(ray, viewer.scene) : undefined);
            if (!globePt) return;
            const carto =
              Cesium.Cartographic.fromCartesian(globePt);
            const lat_deg = Cesium.Math.toDegrees(carto.latitude);
            const lon_deg = Cesium.Math.toDegrees(carto.longitude);
            const mode = placeModeRef.current;
            setScenario((prev) => {
              if (mode === 'launch') {
                return {
                  ...prev,
                  launch: { lat_deg, lon_deg, alt_m: prev.launch.alt_m },
                };
              }
              if (mode === 'threat') {
                return {
                  ...prev,
                  threat: { lat_deg, lon_deg, alt_m: prev.threat.alt_m },
                };
              }
              return {
                ...prev,
                sensor: {
                  lat_deg,
                  lon_deg,
                  alt_m: prev.sensor?.alt_m ?? 0,
                },
              };
            });
          },
          ScreenSpaceEventType.LEFT_CLICK,
        );

        // Initial frame + draw.
        redrawRef.current?.();
        viewer.camera.flyTo({
          destination: Cesium.Cartesian3.fromDegrees(
            DEFAULT_SCENARIO.launch.lon_deg,
            DEFAULT_SCENARIO.launch.lat_deg,
            120000,
          ),
          duration: 0,
        });
        setReady(true);
      })
      .catch((err) => {
        console.error('[ScenarioAuthor] failed to initialize:', err);
      });

    return () => {
      cancelled = true;
      const h = handlerRef.current;
      if (h && !h.isDestroyed()) h.destroy();
      handlerRef.current = null;
      const v = viewerRef.current;
      if (v && !v.isDestroyed()) v.destroy();
      viewerRef.current = null;
      cesiumRef.current = null;
    };
  }, []);

  // Redraw whenever the scenario changes (and the viewer exists).
  useEffect(() => {
    if (ready) redrawRef.current?.();
  }, [scenario, ready]);

  // ---- Inline numeric editing ------------------------------------------------
  function setNum(key: keyof AuthoredScenario, raw: string) {
    const val = Number(raw);
    if (!Number.isFinite(val)) return;
    setScenario((prev) => ({ ...prev, [key]: val }));
  }
  function setPointAlt(which: 'launch' | 'threat' | 'sensor', raw: string) {
    const val = Number(raw);
    if (!Number.isFinite(val)) return;
    setScenario((prev) => {
      const pt = prev[which] as GeodeticPoint | undefined;
      if (!pt) return prev;
      return { ...prev, [which]: { ...pt, alt_m: val } };
    });
  }

  // ---- Preset round-trip -----------------------------------------------------
  async function onPresetChange(file: string) {
    setPresetFile(file);
    if (!file) {
      setBaseConfig(null);
      setScenario(DEFAULT_SCENARIO);
      return;
    }
    try {
      const cfg = await loadPresetConfig(file);
      setBaseConfig(cfg);
      const authored = authoredFromConfig(cfg);
      // Re-project the preset's ENU target.pos0 to a geodetic threat point so it
      // lands correctly on the globe; the launch point is the preset origin.
      const pos0 = cfg.target?.pos0 ?? [0, 0, 0];
      const g = enuToLatLonAlt(
        pos0[0],
        pos0[1],
        pos0[2],
        cfg.origin.lat0_deg,
        cfg.origin.lon0_deg,
        cfg.origin.alt0_m,
      );
      authored.threat = {
        lat_deg: g.lat_deg,
        lon_deg: g.lon_deg,
        alt_m: g.alt_m,
      };
      setScenario(authored);
      // Frame the new launch site.
      const Cesium = cesiumRef.current;
      const viewer = viewerRef.current;
      if (Cesium && viewer) {
        viewer.camera.flyTo({
          destination: Cesium.Cartesian3.fromDegrees(
            cfg.origin.lon0_deg,
            cfg.origin.lat0_deg,
            120000,
          ),
          duration: 0.6,
        });
      }
    } catch {
      /* leave as-is if the preset config can't be loaded */
    }
  }

  // ---- Generate / run / export ----------------------------------------------
  function generate(): SimConfig {
    return buildAuthoredConfig(scenario, baseConfig);
  }
  function handleGenerate() {
    setJson(JSON.stringify(generate(), null, 2));
    setCopied(false);
  }
  function handleRun() {
    onRun(generate());
  }
  async function handleCopy() {
    const text = json || JSON.stringify(generate(), null, 2);
    try {
      await navigator.clipboard.writeText(text);
      setCopied(true);
      setTimeout(() => setCopied(false), 1500);
    } catch {
      /* clipboard blocked: the JSON is still visible in the textarea */
    }
  }
  function toggleSensor() {
    setScenario((prev) =>
      prev.sensor
        ? { ...prev, sensor: undefined }
        : {
            ...prev,
            sensor: {
              lat_deg: prev.launch.lat_deg + 0.05,
              lon_deg: prev.launch.lon_deg,
              alt_m: 0,
            },
          },
    );
  }

  // Live ENU readout of the threat about the current origin (for transparency).
  const enu = geodeticToEnu(scenario.threat, {
    lat0_deg: scenario.launch.lat_deg,
    lon0_deg: scenario.launch.lon_deg,
    alt0_m: scenario.launch.alt_m,
  });

  return (
    <div>
      <h3 style={{ margin: '0 0 4px' }}>
        Author scenario on the globe
      </h3>
      <p className="muted" style={{ marginTop: 0 }}>
        Pick a placement mode, then click the globe to set the launch site
        (origin), the threat, and an optional sensor. Generate a runnable
        SimConfig and launch it in the simulator. Tokenless OpenStreetMap imagery
        on the WGS-84 ellipsoid.
      </p>

      <div
        style={{
          display: 'flex',
          gap: 8,
          flexWrap: 'wrap',
          alignItems: 'center',
          marginBottom: 8,
        }}
      >
        <div className="seg" role="tablist" aria-label="Placement mode">
          {(['launch', 'threat', 'sensor'] as PlaceMode[]).map((m) => (
            <button
              key={m}
              type="button"
              role="tab"
              aria-selected={placeMode === m}
              className={placeMode === m ? 'segActive' : ''}
              onClick={() => setPlaceMode(m)}
            >
              {m === 'launch'
                ? 'Place launch'
                : m === 'threat'
                  ? 'Place threat'
                  : 'Place sensor'}
            </button>
          ))}
        </div>
        {presets.length > 0 ? (
          <label className="field" style={{ margin: 0 }}>
            <span>Start from preset</span>
            <select
              value={presetFile}
              onChange={(e) => onPresetChange(e.target.value)}
              disabled={running}
            >
              <option value="">— from scratch —</option>
              {presets.map((p) => (
                <option key={p.id} value={p.file}>
                  {p.label}
                </option>
              ))}
            </select>
          </label>
        ) : null}
        <button type="button" onClick={toggleSensor}>
          {scenario.sensor ? 'Remove sensor' : 'Add sensor'}
        </button>
      </div>

      <div
        ref={containerRef}
        style={{
          height: 480,
          borderRadius: 8,
          overflow: 'hidden',
          border: '1px solid #1f2a36',
          background: '#0f141b',
        }}
      />

      <div
        style={{
          display: 'grid',
          gridTemplateColumns: 'repeat(auto-fit, minmax(220px, 1fr))',
          gap: 12,
          marginTop: 12,
        }}
      >
        <fieldset>
          <legend>Launch (origin)</legend>
          <Read label="lat" unit="deg" value={scenario.launch.lat_deg} />
          <Read label="lon" unit="deg" value={scenario.launch.lon_deg} />
          <Num
            label="alt"
            unit="m"
            value={scenario.launch.alt_m}
            onChange={(v) => setPointAlt('launch', v)}
          />
          <Num
            label="Launch speed"
            unit="m/s"
            value={scenario.launch_speed_m_s}
            onChange={(v) => setNum('launch_speed_m_s', v)}
          />
          <Num
            label="Elevation"
            unit="deg"
            value={scenario.launch_elevation_deg}
            onChange={(v) => setNum('launch_elevation_deg', v)}
          />
          <Num
            label="Azimuth"
            unit="deg"
            value={scenario.launch_azimuth_deg}
            onChange={(v) => setNum('launch_azimuth_deg', v)}
          />
        </fieldset>

        <fieldset>
          <legend>Threat</legend>
          <Read label="lat" unit="deg" value={scenario.threat.lat_deg} />
          <Read label="lon" unit="deg" value={scenario.threat.lon_deg} />
          <Num
            label="alt"
            unit="m"
            value={scenario.threat.alt_m}
            onChange={(v) => setPointAlt('threat', v)}
          />
          <Num
            label="Speed"
            unit="m/s"
            value={scenario.threat_speed_m_s}
            onChange={(v) => setNum('threat_speed_m_s', v)}
          />
          <Num
            label="Heading (CW from N)"
            unit="deg"
            value={scenario.threat_heading_deg}
            onChange={(v) => setNum('threat_heading_deg', v)}
          />
          <Num
            label="Climb rate"
            unit="m/s"
            value={scenario.threat_climb_m_s}
            onChange={(v) => setNum('threat_climb_m_s', v)}
          />
          <label className="field">
            <span>Maneuver</span>
            <select
              value={scenario.threat_maneuver}
              onChange={(e) =>
                setScenario((p) => ({ ...p, threat_maneuver: e.target.value }))
              }
            >
              {THREAT_MANEUVERS.map((m) => (
                <option key={m} value={m}>
                  {m}
                </option>
              ))}
            </select>
          </label>
          <p className="muted" style={{ margin: '4px 0 0', fontSize: 12 }}>
            ENU pos0 about origin: [{enu[0].toFixed(0)}, {enu[1].toFixed(0)},{' '}
            {enu[2].toFixed(0)}] m
          </p>
        </fieldset>

        {scenario.sensor ? (
          <fieldset>
            <legend>Sensor (coverage dome)</legend>
            <Read label="lat" unit="deg" value={scenario.sensor.lat_deg} />
            <Read label="lon" unit="deg" value={scenario.sensor.lon_deg} />
            <Num
              label="alt"
              unit="m"
              value={scenario.sensor.alt_m}
              onChange={(v) => setPointAlt('sensor', v)}
            />
            <Num
              label="Coverage range"
              unit="m"
              value={scenario.sensorRange_m ?? 15000}
              onChange={(v) => setNum('sensorRange_m', v)}
            />
            <p className="muted" style={{ margin: '4px 0 0', fontSize: 12 }}>
              Visual coverage volume only — not yet a core sensor block.
            </p>
          </fieldset>
        ) : null}
      </div>

      <div className="actions" style={{ marginTop: 12 }}>
        <button
          type="button"
          className="primary"
          onClick={handleRun}
          disabled={running}
        >
          {running ? 'Running…' : 'Run this scenario'}
        </button>
        <button type="button" onClick={handleGenerate} disabled={running}>
          Generate SimConfig JSON
        </button>
        <button type="button" onClick={handleCopy} disabled={running}>
          {copied ? 'Copied!' : 'Copy JSON'}
        </button>
      </div>

      {json ? (
        <textarea
          readOnly
          value={json}
          spellCheck={false}
          style={{
            width: '100%',
            height: 220,
            marginTop: 10,
            fontFamily: 'monospace',
            fontSize: 12,
            background: '#0f141b',
            color: '#cbd5e0',
            border: '1px solid #1f2a36',
            borderRadius: 8,
            padding: 8,
          }}
        />
      ) : null}
    </div>
  );
}

// --- Small inline-field helpers (kept local to avoid touching shared styles) --

function Num({
  label,
  unit,
  value,
  onChange,
}: {
  label: string;
  unit?: string;
  value: number;
  onChange: (raw: string) => void;
}) {
  return (
    <label className="field">
      <span>
        {label}
        {unit ? <em className="unit"> {unit}</em> : null}
      </span>
      <input
        type="number"
        step="any"
        value={Number.isFinite(value) ? value : ''}
        onChange={(e) => onChange(e.target.value)}
      />
    </label>
  );
}

function Read({
  label,
  unit,
  value,
}: {
  label: string;
  unit?: string;
  value: number;
}) {
  return (
    <label className="field">
      <span>
        {label}
        {unit ? <em className="unit"> {unit}</em> : null}
      </span>
      <input type="text" readOnly value={value.toFixed(5)} />
    </label>
  );
}
