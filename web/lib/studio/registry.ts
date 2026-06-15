/**
 * Studio registry — the single place every <StudioDef> registers so the
 * Studios area in the app can discover and list them.
 *
 * Studios self-register by importing this module's `registerStudio` (or by being
 * imported in `studios/index.ts`, which calls it). The registry is insertion-
 * ordered and de-duplicates by `id`, so re-importing a studio module under a
 * hot reload won't double-register it.
 */

import type { StudioDef } from './types';

const registry = new Map<string, StudioDef>();

/**
 * Register a studio. Last registration for a given id wins (idempotent under
 * HMR / repeated imports). Returns the def for convenient inline use.
 */
export function registerStudio(def: StudioDef): StudioDef {
  registry.set(def.id, def);
  return def;
}

/** All registered studios, in registration order. */
export function listStudios(): StudioDef[] {
  return Array.from(registry.values());
}

/** Look up a studio by id, or undefined if not registered. */
export function getStudio(id: string): StudioDef | undefined {
  return registry.get(id);
}
