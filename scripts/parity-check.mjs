// gnc-sim — native <-> WASM determinism parity check (run in CI after build-wasm.sh + build-native.sh).
// Same config + seed must produce the same outcome from the C++ binary and the WASM module,
// proving the browser sim matches the validated native build. Usage:
//   node scripts/parity-check.mjs configs/homing_3dof.json
import { execFileSync } from 'node:child_process';
import { readFileSync, writeFileSync, mkdtempSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';

const configPath = process.argv[2] ?? 'configs/homing_3dof.json';
const TOL = 1e-6;

// Parity compares a SINGLE deterministic run on both targets. If given a Monte Carlo config,
// neutralize the batch (num_cases=0) so the native CLI runs the same single engagement as WASM.
const cfg = JSON.parse(readFileSync(configPath, 'utf8'));
if (cfg.monte_carlo) cfg.monte_carlo.num_cases = 0;
const configText = JSON.stringify(cfg);

// --- Native: run the CLI, read miss_distance from manifest.json ---
const outDir = mkdtempSync(join(tmpdir(), 'gncsim-parity-'));
const normConfig = join(outDir, 'config.json');
writeFileSync(normConfig, configText);
execFileSync('./build-native/apps/cli/gncsim', ['--config', normConfig, '--out', outDir], {
  stdio: 'inherit',
});
const nativeManifest = JSON.parse(readFileSync(join(outDir, 'manifest.json'), 'utf8'));
const nativeMiss = nativeManifest.miss_distance;

// --- WASM: load the Emscripten module, call run_sim with the same config ---
// Pass the .wasm bytes directly so this works under Node's file:// loader (no fetch needed).
const wasmBinary = new Uint8Array(readFileSync(new URL('../web/public/wasm/gncsim.wasm', import.meta.url)));
const { default: createGncSim } = await import('../web/public/wasm/gncsim.js');
const mod = await createGncSim({ wasmBinary });
const wasmResult = JSON.parse(mod.run_sim(configText));
if (wasmResult.error) throw new Error('WASM run_sim error: ' + wasmResult.error);
const wasmMiss = wasmResult.miss_distance;

// --- Compare ---
const dMiss = Math.abs(nativeMiss - wasmMiss);
const nNative = nativeManifest.num_frames;
const nWasm = wasmResult.series.t.length;

console.log(`native miss=${nativeMiss}  wasm miss=${wasmMiss}  |Δ|=${dMiss}`);
console.log(`native frames=${nNative}  wasm frames=${nWasm}`);

let ok = true;
if (dMiss > TOL) {
  console.error(`FAIL: miss_distance differs by ${dMiss} (> ${TOL})`);
  ok = false;
}
if (nNative !== nWasm) {
  console.error(`FAIL: frame count differs (${nNative} vs ${nWasm})`);
  ok = false;
}
if (!ok) process.exit(1);
console.log('PARITY OK: native and WASM agree.');
