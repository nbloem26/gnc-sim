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
 * Convenience for Monte Carlo: kick off module resolution early so the first
 * batch run doesn't pay the probe/instantiation cost mid-loop.
 */
export async function warmUp(): Promise<void> {
  await getModule();
}
