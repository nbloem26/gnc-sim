/**
 * Environment Studio (issue #110) — interactive exploration of the validated
 * environment models: atmosphere (USSA76 / extended), gravity (central + J2/J3/J4
 * zonal fidelity, with latitude/oblateness), and the parameterized wind profile.
 *
 * Pure client-side: every model is closed-form. The math lives in envMath.ts,
 * ported one-to-one from core/src/env/{Environment,EnvFidelity}.cpp.
 *
 * Naming follows the repo convention (units in the key / label).
 */

import type { StudioDef, ParamValues, PlotSpec } from '../types';
import {
  atmosphere,
  egmGravityMagnitude_mps2,
  windSpeed_mps,
  linspace,
  type AtmosphereModel,
  type GravityFidelity,
  type WindConfig,
} from './envMath';

const SAMPLE_COUNT = 400;

// Color palette consistent with the shared dark theme.
const C_DENSITY = '#4fd1c5';
const C_TEMP = '#f6ad55';
const C_PRESSURE = '#63b3ed';
const C_SOUND = '#b794f4';
const C_GRAVITY = '#68d391';
const C_WIND = '#fc8181';

/** Map the gravity-model enum onto the J2/J3/J4 inclusion flags. */
function gravityFidelity(model: string): GravityFidelity {
  switch (model) {
    case 'j2':
      return { include_j2: true, include_j3: false, include_j4: false };
    case 'j3':
      return { include_j2: true, include_j3: true, include_j4: false };
    case 'j4':
      return { include_j2: true, include_j3: true, include_j4: true };
    case 'central':
    default:
      return { include_j2: false, include_j3: false, include_j4: false };
  }
}

export const environmentStudio: StudioDef = {
  id: 'environment',
  label: 'Environment (atmosphere / gravity / winds)',
  description:
    'Closed-form environment models ported from the C++ core: US Standard ' +
    'Atmosphere 1976 (+ extended high-altitude model), EGM zonal gravity ' +
    '(central + J2/J3/J4, latitude-dependent oblateness), and the parameterized ' +
    'wind profile. Vary the atmosphere/gravity fidelity, latitude and winds.',
  params: [
    {
      kind: 'enum',
      key: 'atmosphere_model',
      label: 'Atmosphere model',
      options: [
        { value: 'ussa76', label: 'USSA76 (0–86 km)' },
        { value: 'extended', label: 'Extended (0–1000 km)' },
      ],
      default_value: 'extended',
      help: 'USSA76 layered model below 86 km; extended adds log-linear node interpolation above.',
    },
    {
      kind: 'enum',
      key: 'gravity_model',
      label: 'Gravity fidelity',
      options: [
        { value: 'central', label: 'Central (point-mass)' },
        { value: 'j2', label: '+ J2 (oblateness)' },
        { value: 'j3', label: '+ J3 (pear-shape)' },
        { value: 'j4', label: '+ J4' },
      ],
      default_value: 'j2',
      help: 'Cumulative zonal terms; J3/J4 include the lower zonals too.',
    },
    {
      kind: 'number',
      key: 'max_altitude_m',
      label: 'Max altitude',
      unit: 'm',
      min_value: 10000,
      max_value: 1000000,
      step: 10000,
      default_value: 300000,
      help: 'Top of the altitude sweep for all profiles.',
    },
    {
      kind: 'number',
      key: 'latitude_deg',
      label: 'Geocentric latitude',
      unit: 'deg',
      min_value: -90,
      max_value: 90,
      step: 1,
      default_value: 45,
      help: 'Drives the oblateness (zonal) variation of |g|.',
    },
    {
      kind: 'number',
      key: 'surface_wind_mps',
      label: 'Surface wind speed',
      unit: 'm/s',
      min_value: 0,
      max_value: 50,
      step: 0.5,
      default_value: 5,
    },
    {
      kind: 'boolean',
      key: 'wind_shear',
      label: 'Wind shear (jet stream)',
      default_value: true,
      help: 'When on, wind shears up to a jet maximum then decays; off = constant surface wind.',
    },
  ],
  presets: [
    {
      id: 'sea-level-launch',
      label: 'Sea-level launch (USSA76)',
      values: {
        atmosphere_model: 'ussa76',
        gravity_model: 'j2',
        max_altitude_m: 86000,
        latitude_deg: 28,
        surface_wind_mps: 5,
        wind_shear: true,
      },
    },
    {
      id: 'exoatmospheric',
      label: 'Exoatmospheric (extended, J4)',
      values: {
        atmosphere_model: 'extended',
        gravity_model: 'j4',
        max_altitude_m: 1000000,
        latitude_deg: 45,
        surface_wind_mps: 0,
        wind_shear: false,
      },
    },
  ],
  compute(params: ParamValues): PlotSpec[] {
    const atmModel = String(params.atmosphere_model) as AtmosphereModel;
    const gravModel = String(params.gravity_model);
    const maxAlt_m = Number(params.max_altitude_m);
    const latitude_deg = Number(params.latitude_deg);
    const surfaceWind_mps = Number(params.surface_wind_mps);
    const windShear = Boolean(params.wind_shear);

    const fidelity = gravityFidelity(gravModel);

    const alt_m = linspace(0, maxAlt_m, SAMPLE_COUNT);
    const alt_km = alt_m.map((a) => a / 1000);

    // --- 1. Atmosphere profiles ------------------------------------------------
    const atm = alt_m.map((a) => atmosphere(a, atmModel));
    const density_kgpm3 = atm.map((s) => s.density_kgpm3);
    const temperature_k = atm.map((s) => s.temperature_k);
    const pressure_pa = atm.map((s) => s.pressure_pa);
    const sound_mps = atm.map((s) => s.speed_of_sound_mps);

    const atmDensityPlot: PlotSpec = {
      id: 'atm-density',
      title: 'Atmospheric density vs altitude (log scale)',
      data: [
        {
          x: density_kgpm3,
          y: alt_km,
          type: 'scatter',
          mode: 'lines',
          name: 'density',
          line: { color: C_DENSITY, width: 2 },
        },
      ],
      layout: {
        xaxis: { title: { text: 'density [kg/m³]' }, type: 'log' },
        yaxis: { title: { text: 'altitude [km]' } },
      },
    };

    const atmTempPlot: PlotSpec = {
      id: 'atm-temp',
      title: 'Temperature vs altitude',
      data: [
        {
          x: temperature_k,
          y: alt_km,
          type: 'scatter',
          mode: 'lines',
          name: 'temperature',
          line: { color: C_TEMP, width: 2 },
        },
      ],
      layout: {
        xaxis: { title: { text: 'temperature [K]' } },
        yaxis: { title: { text: 'altitude [km]' } },
      },
    };

    const atmPressurePlot: PlotSpec = {
      id: 'atm-pressure',
      title: 'Pressure vs altitude (log scale)',
      data: [
        {
          x: pressure_pa,
          y: alt_km,
          type: 'scatter',
          mode: 'lines',
          name: 'pressure',
          line: { color: C_PRESSURE, width: 2 },
        },
      ],
      layout: {
        xaxis: { title: { text: 'pressure [Pa]' }, type: 'log' },
        yaxis: { title: { text: 'altitude [km]' } },
      },
    };

    const atmSoundPlot: PlotSpec = {
      id: 'atm-sound',
      title: 'Speed of sound vs altitude',
      data: [
        {
          x: sound_mps,
          y: alt_km,
          type: 'scatter',
          mode: 'lines',
          name: 'sound speed',
          line: { color: C_SOUND, width: 2 },
        },
      ],
      layout: {
        xaxis: { title: { text: 'speed of sound [m/s]' } },
        yaxis: { title: { text: 'altitude [km]' } },
      },
    };

    // --- 2. Gravity magnitude vs altitude -------------------------------------
    // Selected fidelity at the chosen latitude, plus equator/pole references to
    // make the oblateness (latitude) spread visible.
    const gSelected = alt_m.map((a) =>
      egmGravityMagnitude_mps2(a, latitude_deg, fidelity),
    );
    const gEquator = alt_m.map((a) => egmGravityMagnitude_mps2(a, 0, fidelity));
    const gPole = alt_m.map((a) => egmGravityMagnitude_mps2(a, 90, fidelity));

    const gravityPlot: PlotSpec = {
      id: 'gravity',
      title: `Gravity magnitude vs altitude — ${gravModel.toUpperCase()} fidelity`,
      data: [
        {
          x: alt_km,
          y: gSelected,
          type: 'scatter',
          mode: 'lines',
          name: `|g| @ ${latitude_deg.toFixed(0)}° lat`,
          line: { color: C_GRAVITY, width: 2.5 },
        },
        {
          x: alt_km,
          y: gEquator,
          type: 'scatter',
          mode: 'lines',
          name: '|g| @ equator',
          line: { color: C_GRAVITY, width: 1, dash: 'dot' },
        },
        {
          x: alt_km,
          y: gPole,
          type: 'scatter',
          mode: 'lines',
          name: '|g| @ pole',
          line: { color: '#9ae6b4', width: 1, dash: 'dash' },
        },
      ],
      layout: {
        xaxis: { title: { text: 'altitude [km]' } },
        yaxis: { title: { text: '|g| [m/s²]' } },
      },
    };

    // --- 3. Wind profile vs altitude ------------------------------------------
    // Shear on: surface -> jet maximum -> exponential decay (EnvFidelity.cpp).
    // Shear off: constant surface wind at all altitudes.
    const windCfg: WindConfig = windShear
      ? {
          surface_mps: surfaceWind_mps,
          // Representative jet-stream values (tropopause ~12 km).
          jet_mps: Math.max(surfaceWind_mps, surfaceWind_mps + 25),
          jet_alt_m: 12000,
          decay_scale_m: 8000,
        }
      : {
          surface_mps: surfaceWind_mps,
          jet_mps: surfaceWind_mps,
          jet_alt_m: 0,
          decay_scale_m: 1,
        };

    // Winds are a low-altitude phenomenon — cap the sweep at 50 km for clarity.
    const windTop_m = Math.min(maxAlt_m, 50000);
    const windAlt_m = linspace(0, windTop_m, SAMPLE_COUNT);
    const wind_mps = windShear
      ? windAlt_m.map((a) => windSpeed_mps(a, windCfg))
      : windAlt_m.map(() => surfaceWind_mps);

    const windPlot: PlotSpec = {
      id: 'wind',
      title: 'Wind speed vs altitude',
      data: [
        {
          x: wind_mps,
          y: windAlt_m.map((a) => a / 1000),
          type: 'scatter',
          mode: 'lines',
          name: 'wind speed',
          line: { color: C_WIND, width: 2 },
        },
      ],
      layout: {
        xaxis: { title: { text: 'wind speed [m/s]' } },
        yaxis: { title: { text: 'altitude [km]' } },
      },
    };

    return [
      atmDensityPlot,
      atmTempPlot,
      atmPressurePlot,
      atmSoundPlot,
      gravityPlot,
      windPlot,
    ];
  },
};
