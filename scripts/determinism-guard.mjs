// gnc-sim — determinism guard (issue #50).
//
// Enforces the tiered-fidelity guarantee documented in docs/ARCHITECTURE.md §"Tiered model fidelity":
//
//   * FAST tier (WASM-safe, interactive): native<->WASM parity is GUARANTEED. This guard runs every
//     config in `fast_tier_configs` (configs/tiers.json) through the same native vs WASM comparison
//     as scripts/parity-check.mjs and asserts |Δ| within the parity tolerance for all of them.
//   * Coverage: every model the registry classifies as `fast` (configs/tiers.json `models`) must be
//     exercised by at least one fast-tier config (via `config_models`). Adding a fast-tier model to
//     the registry WITHOUT a parity-checked config that exercises it FAILS this guard — so parity
//     coverage always tracks the fast-tier model set.
//   * HIFI tier (high-fidelity native): asserted to be covered by the golden harness instead — each
//     `hifi_configs` entry must be a `configs/<name>.json` that exists (golden coverage is checked in
//     depth by postproc/gncpost/golden.py + the test_tiers.py / vnv leg).
//
// Usage (after scripts/build-native.sh + scripts/build-wasm.sh):
//   node scripts/determinism-guard.mjs
//
// Exit 0 = all fast-tier configs parity-clean and every fast-tier model covered; non-zero otherwise.
import { execFileSync } from 'node:child_process';
import { readFileSync, writeFileSync, mkdtempSync, existsSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const REPO_ROOT = join(dirname(fileURLToPath(import.meta.url)), '..');
const TIERS_PATH = join(REPO_ROOT, 'configs', 'tiers.json');
const CLI = join(REPO_ROOT, 'build-native', 'apps', 'cli', 'gncsim');
const WASM_JS = join(REPO_ROOT, 'web', 'public', 'wasm', 'gncsim.js');
const WASM_BIN = join(REPO_ROOT, 'web', 'public', 'wasm', 'gncsim.wasm');
// Same tolerance as scripts/parity-check.mjs: the established native<->WASM parity guarantee.
const TOL = 1e-6;

const tiers = JSON.parse(readFileSync(TIERS_PATH, 'utf8'));

// ---- Load the WASM module once (reused across configs). ----
const wasmBinary = new Uint8Array(readFileSync(WASM_BIN));
const { default: createGncSim } = await import(WASM_JS);
const mod = await createGncSim({ wasmBinary });

function runParity(name) {
  const configPath = join(REPO_ROOT, 'configs', `${name}.json`);
  const cfg = JSON.parse(readFileSync(configPath, 'utf8'));
  // Parity compares ONE deterministic run on both targets; neutralize any Monte-Carlo batch so the
  // native CLI runs the same single engagement WASM does (mirrors scripts/parity-check.mjs).
  if (cfg.monte_carlo) cfg.monte_carlo.num_cases = 0;
  const configText = JSON.stringify(cfg);

  const outDir = mkdtempSync(join(tmpdir(), 'gncsim-guard-'));
  const normConfig = join(outDir, 'config.json');
  writeFileSync(normConfig, configText);
  execFileSync(CLI, ['--config', normConfig, '--out', outDir], { stdio: 'pipe' });
  const nativeManifest = JSON.parse(readFileSync(join(outDir, 'manifest.json'), 'utf8'));

  const wasmResult = JSON.parse(mod.run_sim(configText));
  if (wasmResult.error) throw new Error(`WASM run_sim error: ${wasmResult.error}`);

  const dMiss = Math.abs(nativeManifest.miss_distance - wasmResult.miss_distance);
  const nNative = nativeManifest.num_frames;
  const nWasm = wasmResult.series.t.length;
  const ok = dMiss <= TOL && nNative === nWasm;
  return { dMiss, nNative, nWasm, ok };
}

let failures = 0;

// ---- (1) Fast-tier model coverage: every `fast` model must appear in some fast-tier config. ----
const fastModels = new Set();
for (const family of Object.values(tiers.models)) {
  for (const [key, tier] of Object.entries(family)) {
    if (tier === 'fast') fastModels.add(key);
  }
}
const covered = new Set();
for (const name of tiers.fast_tier_configs) {
  for (const m of tiers.config_models[name] ?? []) covered.add(m);
}
const uncovered = [...fastModels].filter((m) => !covered.has(m));
if (uncovered.length) {
  failures += uncovered.length;
  for (const m of uncovered) {
    console.error(
      `FAIL: fast-tier model '${m}' is in no fast_tier_configs entry (config_models) — ` +
        `add a parity-checked config that exercises it.`,
    );
  }
}
// Guard against a config claiming a model that isn't fast-tier (stale manifest).
for (const [name, models] of Object.entries(tiers.config_models)) {
  for (const m of models) {
    if (!fastModels.has(m)) {
      failures++;
      console.error(`FAIL: config_models['${name}'] lists '${m}', not a fast-tier model.`);
    }
  }
}

// ---- (2) Run parity on every fast-tier config. ----
console.log(`\nDeterminism guard — ${tiers.fast_tier_configs.length} fast-tier configs`);
console.log('='.repeat(78));
for (const name of tiers.fast_tier_configs) {
  const r = runParity(name);
  const tag = r.ok ? 'PARITY |Δ|=0 ' : 'FAIL        ';
  console.log(
    `  ${tag} ${name.padEnd(22)} |Δ|=${r.dMiss.toExponential(3)}  frames ${r.nNative}/${r.nWasm}`,
  );
  if (!r.ok) {
    failures++;
    if (r.dMiss > TOL) console.error(`    miss_distance differs by ${r.dMiss} (> ${TOL})`);
    if (r.nNative !== r.nWasm) console.error(`    frame count differs (${r.nNative} vs ${r.nWasm})`);
  }
}

// ---- (3) Hi-fi configs must exist (golden-checked, not parity-guaranteed). ----
console.log('-'.repeat(78));
for (const name of tiers.hifi_configs ?? []) {
  const p = join(REPO_ROOT, 'configs', `${name}.json`);
  if (existsSync(p)) {
    console.log(`  GOLDEN-tier ${name.padEnd(22)} (config present; covered by golden harness)`);
  } else {
    failures++;
    console.error(`FAIL: hifi config '${name}' has no configs/${name}.json`);
  }
}

console.log('='.repeat(78));
if (failures) {
  console.error(`\nDETERMINISM GUARD FAILED — ${failures} problem(s).`);
  process.exit(1);
}
console.log('\nDETERMINISM GUARD OK — every fast-tier config is native↔WASM parity-clean,');
console.log('every fast-tier model is parity-covered, and hi-fi configs are golden-tier.\n');
