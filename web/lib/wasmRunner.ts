/**
 * WASM runner: loads the Emscripten module factory `createGncSim` from
 * /wasm/gncsim.js at runtime and exposes `runSim(config) -> SimResult`.
 *
 * The C++ core is cross-compiled to WASM by CI (scripts/build-wasm.sh) and lands
 * at web/public/wasm/gncsim.js + gncsim.wasm. The module exports a bound
 * function `run_sim(configJsonString) -> resultJsonString`.
 *
 * If the WASM artifact is absent (local dev without the CI build), we fall back
 * to fetching /sample_result.json so the entire UI remains usable. `isMockMode()`
 * reports which path is live so the UI can surface it.
 */

import type { SimConfig, SimResult } from './types';
import { isSimError } from './types';

// Emscripten module shape we depend on.
interface GncSimModule {
  run_sim: (configJson: string) => string;
  // Additive airframe-linearization entry (issue #122). Optional so an older WASM artifact that
  // predates it still loads (we feature-detect before calling).
  linearize?: (configJson: string) => string;
}

/** Trim flight condition for the airframe linearization (issue #122). */
export interface TrimCondition {
  mach: number;
  altitude_m: number;
  alpha_rad: number;
}

/**
 * Result of the real 6DOF airframe+actuator pitch-channel linearization (the WASM `linearize`
 * entry). Mirrors core/include/gncsim/dynamics/Linearize.hpp.
 */
export interface LinearizeResult {
  a_matrix: [number, number, number, number]; // 2x2 row-major [a00,a01,a10,a11]
  b_matrix: [number, number]; // 2x1 [b0,b1]
  omega_n_radps: number;
  zeta: number;
  stable: boolean;
  control_effectiveness_mps2_per_rad: number;
  q_bar_pa: number;
  speed_mps: number;
  iyy_kgm2: number;
  cm_alpha_per_rad: number;
  cn_alpha_per_rad: number;
}

type GncSimFactory = (opts?: Record<string, unknown>) => Promise<GncSimModule>;

const WASM_GLUE_URL = '/wasm/gncsim.js';
const SAMPLE_RESULT_URL = '/sample_result.json';

let modulePromise: Promise<GncSimModule | null> | null = null;
let mockMode = false;
let resolvedOnce = false;

/** True once we have determined whether we are running real WASM or the mock. */
export function isResolved(): boolean {
  return resolvedOnce;
}

/** True if we fell back to sample_result.json (no WASM artifact present). */
export function isMockMode(): boolean {
  return mockMode;
}

/**
 * Attempt to load and instantiate the WASM module. Returns null if the artifact
 * is missing or fails to load, which triggers mock fallback in runSim().
 */
async function loadModule(): Promise<GncSimModule | null> {
  // Probe for the glue file first; a 404 means no artifact => mock mode.
  try {
    const head = await fetch(WASM_GLUE_URL, { method: 'HEAD' });
    if (!head.ok) {
      mockMode = true;
      resolvedOnce = true;
      return null;
    }
  } catch {
    mockMode = true;
    resolvedOnce = true;
    return null;
  }

  try {
    // Dynamic import of the ES-module factory. webpackIgnore keeps the bundler
    // from trying to resolve a file that does not exist at build time.
    const mod = (await import(/* webpackIgnore: true */ WASM_GLUE_URL)) as {
      default?: GncSimFactory;
      createGncSim?: GncSimFactory;
    };
    const factory = mod.default ?? mod.createGncSim;
    if (typeof factory !== 'function') {
      mockMode = true;
      resolvedOnce = true;
      return null;
    }
    const instance = await factory();
    if (typeof instance.run_sim !== 'function') {
      mockMode = true;
      resolvedOnce = true;
      return null;
    }
    mockMode = false;
    resolvedOnce = true;
    return instance;
  } catch {
    mockMode = true;
    resolvedOnce = true;
    return null;
  }
}

function getModule(): Promise<GncSimModule | null> {
  if (!modulePromise) {
    modulePromise = loadModule();
  }
  return modulePromise;
}

async function fetchSampleResult(): Promise<SimResult> {
  const res = await fetch(SAMPLE_RESULT_URL);
  if (!res.ok) {
    throw new Error(`Failed to load fallback sample result (${res.status})`);
  }
  return (await res.json()) as SimResult;
}

/**
 * Run the simulation for a given config.
 *
 * Real mode: serializes the config, calls into WASM `run_sim`, parses the result.
 * Mock mode: returns the committed sample_result.json (the seed/config inputs are
 * ignored, but the UI still renders a real, contract-shaped trajectory).
 */
export async function runSim(config: SimConfig): Promise<SimResult> {
  const mod = await getModule();

  if (mod) {
    const out = mod.run_sim(JSON.stringify(config));
    let parsed: unknown;
    try {
      parsed = JSON.parse(out);
    } catch {
      throw new Error('WASM returned non-JSON output');
    }
    if (isSimError(parsed)) {
      throw new Error(parsed.error);
    }
    return parsed as SimResult;
  }

  // Mock fallback.
  return fetchSampleResult();
}

/**
 * Linearize the real 6DOF airframe+actuator pitch channel about a flight condition (issue #122),
 * via the WASM `linearize` entry. The trim condition rides in an optional top-level `trim` block of
 * the SAME config JSON the core loader consumes.
 *
 * Returns `null` in mock mode (no WASM artifact) or if the loaded module predates the `linearize`
 * export, so the Controls Studio can gracefully fall back to its reduced-order model.
 */
export async function linearizeAirframe(
  config: SimConfig,
  trim: TrimCondition,
): Promise<LinearizeResult | null> {
  const mod = await getModule();
  if (!mod || typeof mod.linearize !== 'function') return null;

  const payload = { ...config, trim };
  const out = mod.linearize(JSON.stringify(payload));
  let parsed: unknown;
  try {
    parsed = JSON.parse(out);
  } catch {
    throw new Error('WASM linearize returned non-JSON output');
  }
  if (isSimError(parsed)) {
    throw new Error(parsed.error);
  }
  return parsed as LinearizeResult;
}

/**
 * Convenience for Monte Carlo: kick off module resolution early so the first
 * batch run doesn't pay the probe/instantiation cost mid-loop.
 */
export async function warmUp(): Promise<void> {
  await getModule();
}
