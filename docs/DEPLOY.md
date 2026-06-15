# Deploying the web app to Vercel

The browser app in [`web/`](../web/) is a **static Next.js export** (`output: 'export'`).
The whole simulation runs **client-side in WebAssembly**, so there is no server
runtime to deploy — Vercel just serves the static `web/out/` tree.

## How the deploy is wired

A repo-root [`vercel.json`](../vercel.json) points Vercel at the monorepo's
`web/` app:

```json
{
  "framework": "nextjs",
  "installCommand": "cd web && npm ci",
  "buildCommand": "cd web && npm run build",
  "outputDirectory": "web/out"
}
```

- `installCommand` / `buildCommand` run inside `web/`; `outputDirectory` is the
  static export (`web/out`).
- **Alternative:** instead of the root `vercel.json`, set the project's
  **Root Directory = `web`** in the Vercel dashboard and let it auto-detect
  Next.js. Either approach works; the committed `vercel.json` keeps the config
  in-repo so the deploy is reproducible.

## What the build does

`npm run build` runs the `prebuild` step first, which does **both**:

1. **`sync-scenarios.mjs`** — refreshes `web/public/scenarios/*.json` from the
   canonical `configs/` (no-op on a web-only checkout, where the committed
   copies are used as-is) so the scenario presets ship in the export under
   `/scenarios/`.
2. **`copy-cesium-assets.mjs`** — copies Cesium's `Workers/Assets/Widgets/ThirdParty`
   runtime assets into `web/public/cesium/` so the 3D globe loads with no CDN or
   ion token.

After the export you should see `web/out/scenarios/index.json` and
`web/out/cesium/` emitted.

## No Emscripten needed at build time

The WASM artifact (`web/public/wasm/gncsim.{js,wasm}`) is **committed**, so Vercel
does **not** need the Emscripten toolchain to build — `npm run build` only runs
Next.js + the two prebuild copy scripts. (If the WASM artifact is absent the app
still builds and runs in **mock mode**, serving `web/public/sample_result.json`.)

## WASM MIME type

Vercel serves `.wasm` files with the correct `application/wasm` content type, so
the Emscripten loader streams/instantiates the module without extra headers.
