/**
 * envMath.ts — closed-form environment models for the Environment Studio,
 * ported from the validated C++ core. Pure functions, no engine / WASM needed.
 *
 * Mirrors:
 *   - core/src/env/Environment.cpp   (USSA76 layered atmosphere, inverse-square g)
 *   - core/src/env/EnvFidelity.cpp   (EGM J2/J3/J4 zonal gravity, extended
 *                                     high-altitude atmosphere, parameterized winds)
 *   - core/include/gncsim/env/Frames.hpp (WGS-84 constants)
 *
 * Naming follows the repo convention: physical quantities carry an SI unit
 * suffix (e.g. density_kgpm3, altitude_m, gravity_mps2).
 */

// ----------------------------------------------------------------------------
// Shared physical constants (Environment.cpp / EnvFidelity.cpp / Frames.hpp)
// ----------------------------------------------------------------------------

const G0_MPS2 = 9.80665; // standard gravity [m/s^2]
const R_GAS_JPKGK = 287.05287; // specific gas constant for air [J/(kg*K)]
const GAMMA = 1.4; // ratio of specific heats for air [-]

// Gravity inverse-square falloff uses the mean Earth radius (Environment.cpp).
const RE_GRAVITY_M = 6371000.0;
// USSA76 uses its own effective Earth radius for geometric->geopotential.
const RE_USSA_M = 6356766.0;

// WGS-84 / EGM constants (Frames.hpp + EnvFidelity.cpp zonal block).
const WGS84_GM_M3PS2 = 3.986004418e14; // gravitational parameter GM [m^3/s^2]
const WGS84_A_M = 6378137.0; // semi-major axis [m]

// Unnormalized zonal harmonic coefficients (EnvFidelity.cpp).
const J2 = 1.082626683e-3;
const J3 = -2.532435346e-6;
const J4 = -1.619331205e-6;

const DEG2RAD = Math.PI / 180.0;

// ----------------------------------------------------------------------------
// Atmosphere sample
// ----------------------------------------------------------------------------

export interface AtmSample {
  temperature_k: number;
  pressure_pa: number;
  density_kgpm3: number;
  speed_of_sound_mps: number;
}

// =============================================================================
// US Standard Atmosphere 1976 (USSA76), lower layers (0..86 km geometric)
// Port of Environment.cpp::atmosphereUSSA76.
// =============================================================================

interface Ussa76Layer {
  base_h_m: number; // geopotential base altitude [m]
  lapse_kpm: number; // lapse rate dT/dh [K/m]
}

// Layer breakpoints at geopotential base altitudes (Environment.cpp kLayers).
const USSA76_LAYERS: Ussa76Layer[] = [
  { base_h_m: 0.0, lapse_kpm: -0.0065 }, // 0 troposphere
  { base_h_m: 11000.0, lapse_kpm: 0.0 }, // 1 tropopause (isothermal)
  { base_h_m: 20000.0, lapse_kpm: 0.001 }, // 2 lower stratosphere
  { base_h_m: 32000.0, lapse_kpm: 0.0028 }, // 3 upper stratosphere
  { base_h_m: 47000.0, lapse_kpm: 0.0 }, // 4 stratopause (isothermal)
  { base_h_m: 51000.0, lapse_kpm: -0.0028 }, // 5 lower mesosphere
  { base_h_m: 71000.0, lapse_kpm: -0.002 }, // 6 upper mesosphere
];

const USSA76_TOP_H_M = 84852.0; // geopotential top of modeled region [m]
const USSA76_T0_K = 288.15; // sea-level reference temperature [K]
const USSA76_P0_PA = 101325.0; // sea-level reference pressure [Pa]

// Precompute the base temperature/pressure at the bottom of each layer by
// integrating the barometric formulas upward from sea level (matches the
// function-local-static init in Environment.cpp). Computed once at module load.
const { baseTemp_k: USSA76_BASE_TEMP_K, basePres_pa: USSA76_BASE_PRES_PA } =
  (() => {
    const n = USSA76_LAYERS.length;
    const baseTemp_k = new Array<number>(n);
    const basePres_pa = new Array<number>(n);
    baseTemp_k[0] = USSA76_T0_K;
    basePres_pa[0] = USSA76_P0_PA;
    for (let i = 1; i < n; i++) {
      const prev = USSA76_LAYERS[i - 1];
      const dh_m = USSA76_LAYERS[i].base_h_m - prev.base_h_m;
      const tTop_k = baseTemp_k[i - 1] + prev.lapse_kpm * dh_m;
      if (prev.lapse_kpm === 0.0) {
        // Isothermal layer.
        basePres_pa[i] =
          basePres_pa[i - 1] *
          Math.exp((-G0_MPS2 * dh_m) / (R_GAS_JPKGK * baseTemp_k[i - 1]));
      } else {
        // Linear-lapse layer.
        basePres_pa[i] =
          basePres_pa[i - 1] *
          Math.pow(
            tTop_k / baseTemp_k[i - 1],
            -G0_MPS2 / (prev.lapse_kpm * R_GAS_JPKGK),
          );
      }
      baseTemp_k[i] = tTop_k;
    }
    return { baseTemp_k, basePres_pa };
  })();

/** USSA76 atmosphere at geometric altitude_m (clamped to 0..86 km). */
export function atmosphereUSSA76(altitude_m: number): AtmSample {
  // Clamp geometric altitude, then convert to geopotential altitude.
  const z_m = Math.min(Math.max(altitude_m, 0.0), 86000.0);
  let h_m = (RE_USSA_M * z_m) / (RE_USSA_M + z_m);
  h_m = Math.min(h_m, USSA76_TOP_H_M);

  // Find the layer whose base is at or below h_m.
  let idx = 0;
  for (let i = 0; i < USSA76_LAYERS.length; i++) {
    if (h_m >= USSA76_LAYERS[i].base_h_m) idx = i;
    else break;
  }

  const layer = USSA76_LAYERS[idx];
  const tb_k = USSA76_BASE_TEMP_K[idx];
  const pb_pa = USSA76_BASE_PRES_PA[idx];
  const dh_m = h_m - layer.base_h_m;

  let temperature_k: number;
  let pressure_pa: number;
  if (layer.lapse_kpm === 0.0) {
    temperature_k = tb_k;
    pressure_pa = pb_pa * Math.exp((-G0_MPS2 * dh_m) / (R_GAS_JPKGK * tb_k));
  } else {
    temperature_k = tb_k + layer.lapse_kpm * dh_m;
    pressure_pa =
      pb_pa *
      Math.pow(
        temperature_k / tb_k,
        -G0_MPS2 / (layer.lapse_kpm * R_GAS_JPKGK),
      );
  }

  return {
    temperature_k,
    pressure_pa,
    density_kgpm3: pressure_pa / (R_GAS_JPKGK * temperature_k),
    speed_of_sound_mps: Math.sqrt(GAMMA * R_GAS_JPKGK * temperature_k),
  };
}

// =============================================================================
// Extended atmosphere (USSA76 below 86 km, log-linear node interpolation above)
// Port of EnvFidelity.cpp::atmosphereExtended.
// =============================================================================

interface ExtNode {
  alt_m: number; // geometric altitude [m]
  rho_kgpm3: number; // tabulated density [kg/m^3]
  temp_k: number; // tabulated temperature [K]
}

const EXT_HANDOVER_ALT_M = 86000.0;

// US Standard Atmosphere 1976 upper-table anchor nodes (EnvFidelity.cpp).
const EXT_NODES: ExtNode[] = [
  { alt_m: 86000.0, rho_kgpm3: 6.958e-6, temp_k: 186.87 },
  { alt_m: 100000.0, rho_kgpm3: 5.604e-7, temp_k: 195.08 },
  { alt_m: 150000.0, rho_kgpm3: 2.076e-9, temp_k: 634.39 },
  { alt_m: 200000.0, rho_kgpm3: 2.541e-10, temp_k: 854.56 },
  { alt_m: 300000.0, rho_kgpm3: 1.916e-11, temp_k: 976.01 },
  { alt_m: 500000.0, rho_kgpm3: 6.967e-13, temp_k: 999.24 },
  { alt_m: 1000000.0, rho_kgpm3: 3.561e-15, temp_k: 1000.0 },
];

/** Extended atmosphere: USSA76 below 86 km, node interpolation above. */
export function atmosphereExtended(altitude_m: number): AtmSample {
  if (altitude_m <= EXT_HANDOVER_ALT_M) return atmosphereUSSA76(altitude_m);

  // Pin the bottom node density to the live USSA76 value at 86 km for a
  // continuous handover (EnvFidelity.cpp).
  const rho0_kgpm3 = atmosphereUSSA76(EXT_HANDOVER_ALT_M).density_kgpm3;

  const topNode = EXT_NODES[EXT_NODES.length - 1];
  if (altitude_m >= topNode.alt_m) {
    return {
      temperature_k: topNode.temp_k,
      density_kgpm3: topNode.rho_kgpm3,
      pressure_pa: topNode.rho_kgpm3 * R_GAS_JPKGK * topNode.temp_k,
      speed_of_sound_mps: Math.sqrt(GAMMA * R_GAS_JPKGK * topNode.temp_k),
    };
  }

  // Bracket the interval [lo, lo+1].
  let lo = 0;
  for (let i = 0; i + 1 < EXT_NODES.length; i++) {
    if (altitude_m >= EXT_NODES[i].alt_m) lo = i;
  }
  const a = EXT_NODES[lo];
  const b = EXT_NODES[lo + 1];
  const rhoA_kgpm3 = lo === 0 ? rho0_kgpm3 : a.rho_kgpm3;
  const rhoB_kgpm3 = b.rho_kgpm3;

  // Log-linear (constant effective scale height) density interpolation.
  const frac = (altitude_m - a.alt_m) / (b.alt_m - a.alt_m);
  const density_kgpm3 = rhoA_kgpm3 * Math.pow(rhoB_kgpm3 / rhoA_kgpm3, frac);
  const temperature_k = a.temp_k + frac * (b.temp_k - a.temp_k);

  return {
    temperature_k,
    density_kgpm3,
    pressure_pa: density_kgpm3 * R_GAS_JPKGK * temperature_k, // ideal-gas closure
    speed_of_sound_mps: Math.sqrt(GAMMA * R_GAS_JPKGK * temperature_k),
  };
}

export type AtmosphereModel = 'ussa76' | 'extended';

/** Atmosphere dispatch by model id. */
export function atmosphere(
  altitude_m: number,
  model: AtmosphereModel,
): AtmSample {
  return model === 'extended'
    ? atmosphereExtended(altitude_m)
    : atmosphereUSSA76(altitude_m);
}

// =============================================================================
// Gravity
// =============================================================================

/**
 * Simple inverse-square gravity magnitude (Environment.cpp::GravityModel):
 *   g = g0 * (Re / (Re + alt))^2, alt clamped to >= 0.
 * Latitude-independent (the central / local-vertical model).
 */
export function gravityCentralMagnitude_mps2(altitude_m: number): number {
  const alt_m = Math.max(0.0, altitude_m);
  const ratio = RE_GRAVITY_M / (RE_GRAVITY_M + alt_m);
  return G0_MPS2 * ratio * ratio;
}

export interface GravityFidelity {
  include_j2: boolean;
  include_j3: boolean;
  include_j4: boolean;
}

/**
 * EGM-style zonal gravity vector (EnvFidelity.cpp::egmGravity), in an
 * Earth-fixed frame whose +Z is the spin axis. Returns the 3-vector
 * [gx, gy, gz] m/s^2 (points inward / negative-radial).
 *
 * The position is built from geocentric latitude + radius so the studio can
 * show the oblateness (latitude) dependence of |g|.
 */
export function egmGravityVector_mps2(
  r_m: { x: number; y: number; z: number },
  cfg: GravityFidelity,
): { x: number; y: number; z: number } {
  const rmag_m = Math.hypot(r_m.x, r_m.y, r_m.z);
  if (rmag_m < 1.0) return { x: 0, y: 0, z: 0 };

  const mu = WGS84_GM_M3PS2;
  const re_m = WGS84_A_M;
  const invR = 1.0 / rmag_m;
  const invR2 = invR * invR;

  // Central point-mass term: -mu/r^3 * r.
  const g = {
    x: r_m.x * (-mu * invR2 * invR),
    y: r_m.y * (-mu * invR2 * invR),
    z: r_m.z * (-mu * invR2 * invR),
  };

  const zr = r_m.z * invR; // sin(geocentric latitude)
  const zr2 = zr * zr;
  const reR = re_m * invR; // (Re/r)
  const reR2 = reR * reR;
  const muR2 = mu * invR2; // mu/r^2

  // --- J2 --- (matches centralGravity's expansion exactly)
  if (cfg.include_j2) {
    const factor = 1.5 * J2 * mu * re_m * re_m * invR2 * invR2; // 1.5 J2 mu Re^2 / r^4
    const zterm = 5.0 * zr2;
    g.x += factor * (zterm - 1.0) * r_m.x * invR;
    g.y += factor * (zterm - 1.0) * r_m.y * invR;
    g.z += factor * (zterm - 3.0) * r_m.z * invR;
  }

  // --- J3 --- (odd zonal; "pear-shape")
  if (cfg.include_j3) {
    const reR3 = reR2 * reR;
    const common = muR2 * reR3;
    const horiz = -2.5 * J3 * common * (7.0 * zr2 * zr - 3.0 * zr) * invR;
    g.x += horiz * r_m.x;
    g.y += horiz * r_m.y;
    g.z += -0.5 * J3 * common * (35.0 * zr2 * zr2 - 30.0 * zr2 + 3.0);
  }

  // --- J4 ---
  if (cfg.include_j4) {
    const reR4 = reR2 * reR2;
    const common = muR2 * reR4;
    const horiz =
      0.625 * J4 * common * (63.0 * zr2 * zr2 - 42.0 * zr2 + 3.0) * invR;
    g.x += horiz * r_m.x;
    g.y += horiz * r_m.y;
    g.z += 0.625 * J4 * common * (33.0 * zr2 * zr2 - 30.0 * zr2 + 5.0) * zr;
  }

  return g;
}

/**
 * Magnitude of the EGM gravity at a given geocentric latitude + altitude.
 * The radius is approximated as Re + altitude (geocentric-spherical), which is
 * adequate to visualize the zonal/oblateness dependence on latitude.
 */
export function egmGravityMagnitude_mps2(
  altitude_m: number,
  latitude_deg: number,
  cfg: GravityFidelity,
): number {
  const lat_rad = latitude_deg * DEG2RAD;
  const r_m = RE_GRAVITY_M + Math.max(0.0, altitude_m);
  const pos = {
    x: r_m * Math.cos(lat_rad),
    y: 0.0,
    z: r_m * Math.sin(lat_rad),
  };
  const g = egmGravityVector_mps2(pos, cfg);
  return Math.hypot(g.x, g.y, g.z);
}

// =============================================================================
// Winds (parameterized ENU profile) — EnvFidelity.cpp::windEnu
// =============================================================================

export interface WindConfig {
  surface_mps: number;
  jet_mps: number;
  jet_alt_m: number;
  decay_scale_m: number;
}

/**
 * Horizontal wind speed magnitude at altitude (EnvFidelity.cpp::windEnu):
 *   - linear shear from surface to the jet maximum below jet_alt_m,
 *   - exponential decay above the jet.
 */
export function windSpeed_mps(altitude_m: number, cfg: WindConfig): number {
  const z_m = Math.max(0.0, altitude_m);
  if (z_m <= cfg.jet_alt_m) {
    const frac = cfg.jet_alt_m > 0.0 ? z_m / cfg.jet_alt_m : 1.0;
    return cfg.surface_mps + frac * (cfg.jet_mps - cfg.surface_mps);
  }
  const decay = Math.exp(
    -(z_m - cfg.jet_alt_m) / Math.max(1.0, cfg.decay_scale_m),
  );
  return cfg.jet_mps * decay;
}

// ----------------------------------------------------------------------------
// Sampling helper
// ----------------------------------------------------------------------------

/** Linearly spaced sample grid, inclusive of both endpoints. */
export function linspace(start: number, end: number, count: number): number[] {
  if (count < 2) return [start];
  const out = new Array<number>(count);
  const span = (end - start) / (count - 1);
  for (let i = 0; i < count; i++) out[i] = start + i * span;
  return out;
}
