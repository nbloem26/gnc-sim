# Golden-run regression harness

The golden harness protects gnc-sim's **headline metrics** as the physics fidelity
grows. It runs a fixed set of *canonical* scenario configs through the native CLI at
**fixed seeds**, extracts a few key numbers from each, and compares them to a committed
baseline with per-metric tolerances. A number that drifts outside its tolerance is a
**regression** — the harness prints a clear diff and exits non-zero.

It is intentionally **tooling only**: it never touches the C++ core, so native↔WASM
parity and the C++ unit tests (`ctest`) are unaffected. The harness lives in
[`postproc/gncpost/golden.py`](../postproc/gncpost/golden.py); the baseline is
[`postproc/golden/golden.json`](../postproc/golden/golden.json).

## How it works

1. **Canonical set.** `golden.py` declares a list of `GoldenCase`s — one per protected
   config (`configs/<name>.json`). Each names a fixed CLI `seed`, a `kind`
   (`single` or `mc`), and per-metric tolerances.
2. **Run.** Each config is run via the native CLI (`build-native/apps/cli/gncsim`) at
   its fixed seed. The seed is overridden on the command line so it never depends on
   the config file's seed drifting.
3. **Extract metrics.**
   - *Single runs* (`kind="single"`): metrics come from `manifest.json` —
     `miss_distance`, `intercept`, `num_frames`, `intercept_time`, and (for the cued
     engagement) `launch_time`.
   - *Monte-Carlo* (`kind="mc"`): the config is run with `monte_carlo.num_cases`
     reduced (via a temp config copy) for speed, then `pk` (intercept rate), `cep`
     (median miss), and `n` (case count) are read from `summary.csv` via
     `gncpost.montecarlo.compute_stats`.
4. **Compare.** Each metric passes if it is within **either** its absolute **or** its
   relative tolerance band of the stored golden value. Out-of-tolerance ⇒ regression.

## Canonical configs covered

| Config | Kind | Seed | Key metrics |
|---|---|---|---|
| `homing_3dof` | single | 1 | miss_distance, intercept, num_frames, intercept_time |
| `projectile_3dof` | single | 1 | miss_distance, intercept, num_frames, intercept_time |
| `homing_boost` | single | 1 | miss_distance, intercept, num_frames, intercept_time |
| `ballistic_round` | single | 1 | miss_distance, intercept, num_frames, intercept_time |
| `track_fused` | single | 7 | miss_distance, intercept, num_frames, intercept_time |
| `discrimination` | single | 7 | miss_distance, intercept, num_frames, intercept_time |
| `engagement_cued` | single | 7 | miss_distance, intercept, num_frames, intercept_time, launch_time |
| `montecarlo` | mc (32 cases) | 12345 | pk, cep, n |

The whole suite runs in ~1–2 s locally (the Monte-Carlo case count is reduced from the
config's default to keep it fast and deterministic).

## Running it

```bash
. .venv/bin/activate           # or your environment with numpy/scipy/pandas
cd postproc

# CHECK mode (default): compare to the committed golden, exit 0 if clean, 1 on regression
python -m gncpost.golden

# Restrict to a subset
python -m gncpost.golden --only homing_3dof track_fused
```

A fast subset also runs as a pytest:
[`postproc/tests/test_golden.py`](../postproc/tests/test_golden.py) (skipped if the
native CLI isn't built). The full suite runs in CI via
[`.github/workflows/golden.yml`](../.github/workflows/golden.yml), a workflow separate
from the main `ci.yml`.

## Re-baselining

When a change **legitimately** moves a metric (a fidelity improvement, a new physics
term, an intended scenario tweak), regenerate the baseline and commit it:

```bash
cd postproc
python -m gncpost.golden --update                 # rewrite all of golden.json
python -m gncpost.golden --update --only montecarlo   # rebaseline just one config (merged in)
```

Review the diff of `postproc/golden/golden.json` in the same PR as the change that
moved the numbers — the baseline diff *is* the record of what moved and by how much.
Never re-baseline to silence a regression you don't understand.

## Tolerance philosophy

Tolerances encode what kind of change is *expected noise* vs *a real regression*:

- **`miss_distance`** — a **relative** band (typically 1–10%). Miss distance shifts as
  fidelity grows; a couple-percent wobble is fine, a 2× change is a regression. Coarse
  closest-approach scenarios (discrimination, cued engagement) also carry a small
  **absolute** floor so a sub-metre baseline doesn't make the relative band absurdly
  tight.
- **`intercept`** — **exact** (`abs=rel=0`). A hit↔miss flip is always a regression.
- **`num_frames`** — **exact**. The trajectory length / termination condition must not
  move silently; if it does, something changed in integration or end-of-run logic.
- **`intercept_time` / `launch_time`** — small **absolute** band in seconds.
- **`pk` / `cep`** (Monte-Carlo) — **looser** bands. Even at a fixed seed *sequence*,
  statistics over a reduced case count carry sampling noise, so these protect against
  gross shifts (e.g. Pk dropping from 0.84 to 0.5) rather than fine wobble.

A metric passes if it is within **either** the absolute or the relative band — set both
to `0` (the `Tol()` default) for an exact-match metric.

## Scenario schema versioning

Each canonical config may carry an integer `schema_version` key (default `1` if
absent). The C++ config parser ignores unknown keys, so this is purely a tooling marker.
The harness records each config's `schema_version` in `golden.json`.

When you **intentionally** change a scenario's *meaning* (different geometry, different
sensor set, a new block that changes what the run represents), bump its
`schema_version`. On the next check the harness sees the version mismatch and flags that
config as:

```
[SCHEMA]  schema changed (golden vN -> config vM) — re-baseline needed (--update)
```

rather than reporting a confusing silent metric diff. This distinguishes *"the scenario
deliberately changed, re-baseline it"* from *"the numbers moved unexpectedly, investigate
the regression."* After bumping, re-run with `--update` to record the new version and
metrics together.

## What it does **not** do

- It does **not** modify or depend on the C++ core internals — only the CLI's
  documented CSV/JSON output contract ([`docs/DATA_CONTRACT.md`](DATA_CONTRACT.md)).
- It does **not** replace the analytic validation harness
  ([`postproc/gncpost/validate.py`](../postproc/gncpost/validate.py)), which checks the
  sim against closed-form solutions. Validation proves *correctness*; the golden harness
  guards against *unintended drift* in the numbers we've accepted as correct.
