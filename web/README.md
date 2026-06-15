# GNC Studio — Web App

**GNC Studio** is an interactive guided-interceptor guidance, navigation & control
simulator. The deterministic C++ core is cross-compiled to WebAssembly and runs
**entirely in the browser** — no backend. It spans 3/6-DOF flight dynamics, EKF/IMM/JPDA
estimation, radar/IR fusion & phenomenology, PN/APN/optimal guidance, ballistic & hypersonic
threats, and many-on-many engagement campaigns, with a 3D Cesium globe and Monte-Carlo UQ.
Built with Next.js (App Router, TypeScript), Plotly.js, Leaflet, and CesiumJS.

## Develop

```bash
npm install
npm run dev      # http://localhost:3000
```

Locally the WASM artifact is usually absent, so the app runs in **mock mode**: it
loads `public/sample_result.json` (a real native-build result, identical in shape
to WASM output). A `MOCK · sample data` badge shows in the header. Everything
renders — only the seed/config inputs are inert because the same sample is
returned each run, and live Monte Carlo is replaced by the committed
`montecarlo_cep.png`.

## Build

```bash
npm run build    # static export to ./out (also the build gate used in CI)
npm run lint
```

`next.config.js` sets `output: 'export'`, so the app is a static SPA that drops
straight onto Vercel or any static host.

## Real WASM mode

CI (`scripts/build-wasm.sh`) cross-compiles the C++ core with Emscripten to:

- `public/wasm/gncsim.js`  — ES-module factory `createGncSim`
- `public/wasm/gncsim.wasm`

The factory yields a module exposing `run_sim(configJsonString) -> resultJsonString`.
When those files are present, `lib/wasmRunner.ts` loads and instantiates the
module on first run, the header flips to `LIVE · WASM`, and the form inputs,
seed sweeps, and live Monte Carlo all drive the actual simulation. If the
artifact is missing or fails to load, it transparently falls back to mock mode.

## Layout

| Path | Purpose |
|---|---|
| `app/page.tsx` | Main simulator (param form + results) |
| `app/validation/page.tsx` | Engineering validation figures |
| `lib/wasmRunner.ts` | WASM load + mock fallback, `runSim()` |
| `lib/enuToGeodetic.ts` | ENU → lat/lon for the ground track (DATA_CONTRACT §6) |
| `lib/types.ts` | Config + result types (DATA_CONTRACT §1/§2) |
| `components/` | Plotly trajectory/state plots, Leaflet ground track, Monte Carlo |

Plotly and Leaflet are dynamically imported client-side (`ssr: false`) to avoid
SSR `window` errors during static export.
