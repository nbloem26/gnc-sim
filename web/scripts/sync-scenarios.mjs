/**
 * Sync curated scenario presets from the repo's canonical `configs/` into
 * `web/public/scenarios/` so they ship with the static export.
 *
 * The manifest `web/public/scenarios/index.json` is the source of truth for
 * *which* presets are exposed and their labels/descriptions. This script copies
 * each manifest `file` from `../configs/` over the committed copy, keeping the
 * web presets byte-identical to the canonical configs. The copies are also
 * committed so the build works without `configs/` (e.g. on Vercel where only
 * `web/` is the project root) — this script just refreshes them.
 *
 * Run automatically via the `prebuild` npm script; safe to run by hand.
 */

import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const scenariosDir = join(here, '..', 'public', 'scenarios');
const configsDir = join(here, '..', '..', 'configs');
const manifestPath = join(scenariosDir, 'index.json');

const manifest = JSON.parse(readFileSync(manifestPath, 'utf8'));

if (!existsSync(configsDir)) {
  // Canonical configs not available (e.g. web-only checkout). Trust committed copies.
  console.log('[sync-scenarios] configs/ not found — using committed copies as-is.');
  process.exit(0);
}

let copied = 0;
for (const preset of manifest.presets) {
  const src = join(configsDir, preset.file);
  const dst = join(scenariosDir, preset.file);
  if (!existsSync(src)) {
    console.warn(`[sync-scenarios] WARNING: ${preset.file} missing in configs/, keeping committed copy.`);
    continue;
  }
  const text = readFileSync(src, 'utf8');
  writeFileSync(dst, text);
  copied += 1;
}
console.log(`[sync-scenarios] synced ${copied}/${manifest.presets.length} presets into public/scenarios/.`);
