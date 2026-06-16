// Copy Cesium's static runtime assets into web/public/cesium so the Next.js
// static export (output:'export') serves them at /cesium/* with no CDN or ion
// token. Cesium loads Workers/Assets/Widgets/ThirdParty relative to
// window.CESIUM_BASE_URL (set to '/cesium' in CesiumGlobe.tsx).
//
// Runs as a prebuild/predev step (see package.json). The copied tree is
// git-ignored (public/cesium/) — it is a build artifact regenerated from the
// pinned `cesium` dependency, not source.

import { cp, mkdir, rm, stat } from 'node:fs/promises';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const webRoot = join(here, '..');
const src = join(webRoot, 'node_modules', 'cesium', 'Build', 'Cesium');
const dest = join(webRoot, 'public', 'cesium');

// The four directories Cesium fetches at runtime relative to CESIUM_BASE_URL.
const subdirs = ['Workers', 'Assets', 'Widgets', 'ThirdParty'];

async function exists(p) {
  try {
    await stat(p);
    return true;
  } catch {
    return false;
  }
}

async function main() {
  if (!(await exists(src))) {
    console.error(
      `[copy-cesium-assets] Cesium build not found at ${src}. ` +
        'Did you run `npm ci` / `npm install`?',
    );
    process.exit(1);
  }

  await rm(dest, { recursive: true, force: true });
  await mkdir(dest, { recursive: true });

  for (const sub of subdirs) {
    const from = join(src, sub);
    if (!(await exists(from))) {
      console.warn(`[copy-cesium-assets] skipping missing ${sub}`);
      continue;
    }
    await cp(from, join(dest, sub), { recursive: true });
  }

  console.log(`[copy-cesium-assets] copied Cesium assets -> ${dest}`);
}

main().catch((err) => {
  console.error('[copy-cesium-assets] failed:', err);
  process.exit(1);
});
