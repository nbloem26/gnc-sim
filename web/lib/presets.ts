/**
 * Scenario-preset loader. The curated set of canonical `configs/*.json` is
 * exposed to the static web build under `/scenarios/` with an `index.json`
 * manifest (see web/scripts/sync-scenarios.mjs). These helpers fetch the
 * manifest and individual preset configs at runtime (client-side only).
 */

import type { ScenarioManifest, ScenarioPreset, SimConfig } from './types';

const MANIFEST_URL = '/scenarios/index.json';

let manifestPromise: Promise<ScenarioPreset[]> | null = null;

/** Fetch (once, memoized) the list of available scenario presets. */
export function loadPresetManifest(): Promise<ScenarioPreset[]> {
  if (!manifestPromise) {
    manifestPromise = fetch(MANIFEST_URL)
      .then((res) => {
        if (!res.ok) throw new Error(`preset manifest ${res.status}`);
        return res.json() as Promise<ScenarioManifest>;
      })
      .then((m) => m.presets)
      .catch((e) => {
        // Reset so a later call can retry; surface an empty list meanwhile.
        manifestPromise = null;
        throw e;
      });
  }
  return manifestPromise;
}

/** Fetch a single preset's full SimConfig by its manifest file name. */
export async function loadPresetConfig(file: string): Promise<SimConfig> {
  const res = await fetch(`/scenarios/${file}`);
  if (!res.ok) throw new Error(`preset ${file} ${res.status}`);
  return (await res.json()) as SimConfig;
}
