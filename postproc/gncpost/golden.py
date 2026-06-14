"""Golden-run regression harness for gnc-sim.

Runs a fixed set of *canonical* scenario configs through the native CLI at a
**fixed seed**, extracts a handful of key metrics from each, and compares them to
a committed baseline (``postproc/golden/golden.json``) with per-metric absolute /
relative tolerances. Any out-of-tolerance metric is a regression: the harness
prints a clear diff and exits non-zero. As fidelity grows this is the net that
catches an unintended change to the headline numbers (miss distance, intercept,
frame count, Monte-Carlo P(kill) / CEP).

Usage::

    python -m gncpost.golden            # CHECK mode — compare to golden.json, exit 0/1
    python -m gncpost.golden --update   # regenerate golden.json from the current build
    python -m gncpost.golden --only homing_3dof track_fused   # subset (check or update)

Determinism: every config is run at its own fixed ``seed`` (overridden on the CLI
so it never depends on the config file's seed drifting). Monte-Carlo configs are
run with ``num_cases`` reduced via a temp config so the whole suite stays fast.

Scenario schema versioning: each config may carry an integer ``schema_version``
(default 1 if absent). The harness records it per config in golden.json. If a
config's ``schema_version`` differs from the stored baseline, that config is
flagged **"schema changed — re-baseline needed"** rather than reported as a
silent metric regression — bump the version when you intend to re-baseline.

See ``docs/GOLDEN_RUNS.md`` for the tolerance philosophy and how to re-baseline.
"""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from . import CLI_PATH, REPO_ROOT
from .loaders import load_run, load_summary, run_cli
from .montecarlo import compute_stats

GOLDEN_PATH = REPO_ROOT / "postproc" / "golden" / "golden.json"
CONFIG_DIR = REPO_ROOT / "configs"

# Format version of golden.json itself (bump only if the file *shape* changes).
GOLDEN_FORMAT_VERSION = 1


@dataclass(frozen=True)
class Tol:
    """Per-metric tolerance. A metric passes if within EITHER abs OR rel band.

    For boolean / integer-identity metrics (intercept flag, frame count) set both
    to 0 so any change is a regression.
    """

    abs: float = 0.0
    rel: float = 0.0

    def within(self, golden: float, actual: float) -> bool:
        diff = abs(actual - golden)
        # Exact match (abs == rel == 0): only diff == 0 passes — covered by diff <= 0.
        if diff <= self.abs:
            return True
        if self.rel > 0.0 and golden != 0.0:
            return diff <= self.rel * abs(golden)
        return False


@dataclass
class GoldenCase:
    """One canonical scenario the harness protects.

    Attributes:
        name: config stem (``configs/<name>.json``) and key in golden.json.
        seed: fixed CLI seed override (determinism).
        kind: ``"single"`` (one run, metrics from manifest) or ``"mc"``
            (Monte-Carlo batch, metrics from summary.csv).
        tols: per-metric tolerances.
        mc_cases: for ``kind == "mc"``, the reduced ``num_cases`` to run.
    """

    name: str
    seed: int
    kind: str
    tols: dict[str, Tol]
    mc_cases: int = 0


# --------------------------------------------------------------------------------------
# Canonical set. Tolerances reflect each metric's nature:
#   miss_distance   — relative band (grows/shrinks with fidelity; a 2% wobble is fine,
#                      a 2x change is a regression). Coarse-CPA scenarios get an abs floor.
#   intercept       — exact (hit/miss flip is always a regression).
#   num_frames      — exact (trajectory length / termination must not silently move).
#   intercept_time  — small absolute band [s].
#   pk / cep        — Monte-Carlo statistics on a reduced case count: looser bands since
#                      they carry sampling noise even at a fixed seed sequence.
# --------------------------------------------------------------------------------------
CANONICAL: list[GoldenCase] = [
    GoldenCase(
        "homing_3dof",
        seed=1,
        kind="single",
        tols={
            "miss_distance": Tol(abs=0.5, rel=0.05),
            "intercept": Tol(),
            "num_frames": Tol(),
            "intercept_time": Tol(abs=0.05),
        },
    ),
    GoldenCase(
        "projectile_3dof",
        seed=1,
        kind="single",
        tols={
            "miss_distance": Tol(rel=0.01),
            "intercept": Tol(),
            "num_frames": Tol(),
            "intercept_time": Tol(abs=0.05),
        },
    ),
    GoldenCase(
        "homing_boost",
        seed=1,
        kind="single",
        tols={
            "miss_distance": Tol(abs=0.5, rel=0.05),
            "intercept": Tol(),
            "num_frames": Tol(),
            "intercept_time": Tol(abs=0.05),
        },
    ),
    GoldenCase(
        "ballistic_round",
        seed=1,
        kind="single",
        tols={
            "miss_distance": Tol(rel=0.01),
            "intercept": Tol(),
            "num_frames": Tol(),
            "intercept_time": Tol(abs=0.2),
        },
    ),
    GoldenCase(
        "track_fused",
        seed=7,
        kind="single",
        tols={
            "miss_distance": Tol(abs=0.5, rel=0.05),
            "intercept": Tol(),
            "num_frames": Tol(),
            "intercept_time": Tol(abs=0.05),
        },
    ),
    GoldenCase(
        "discrimination",
        seed=7,
        kind="single",
        tols={
            "miss_distance": Tol(abs=2.0, rel=0.10),
            "intercept": Tol(),
            "num_frames": Tol(),
            "intercept_time": Tol(abs=0.1),
        },
    ),
    GoldenCase(
        "engagement_cued",
        seed=7,
        kind="single",
        tols={
            "miss_distance": Tol(abs=1.0, rel=0.05),
            "intercept": Tol(),
            "num_frames": Tol(),
            "intercept_time": Tol(abs=0.1),
            "launch_time": Tol(abs=0.05),
        },
    ),
    GoldenCase(
        "montecarlo",
        seed=12345,
        kind="mc",
        mc_cases=32,
        tols={
            "pk": Tol(abs=0.20),
            "cep": Tol(abs=2.0, rel=0.30),
            "n": Tol(),
        },
    ),
]


@dataclass
class CaseResult:
    """Outcome of evaluating one case (in check mode)."""

    name: str
    metrics: dict[str, float]
    schema_version: int
    passed: bool = True
    schema_changed: bool = False
    diffs: list[str] = field(default_factory=list)


def _config_schema_version(name: str) -> int:
    cfg = json.loads((CONFIG_DIR / f"{name}.json").read_text())
    return int(cfg.get("schema_version", 1))


def _run_single(case: GoldenCase, tmp: Path) -> dict[str, float]:
    """Run a single-shot config and pull headline metrics from its manifest."""
    out = run_cli(CONFIG_DIR / f"{case.name}.json", tmp / case.name, seed=case.seed)
    man = load_run(out).manifest
    metrics: dict[str, float] = {
        "miss_distance": float(man["miss_distance"]),
        "intercept": float(bool(man["intercept"])),
        "num_frames": float(man["num_frames"]),
        "intercept_time": float(man["intercept_time"]),
    }
    if "launch_time" in case.tols:
        metrics["launch_time"] = float(man.get("launch_time", 0.0))
    # Only keep metrics this case actually declares tolerances for.
    return {k: v for k, v in metrics.items() if k in case.tols}


def _run_mc(case: GoldenCase, tmp: Path) -> dict[str, float]:
    """Run a Monte-Carlo config (case count reduced for speed) and pull Pk / CEP.

    Writes a temp config copy with ``monte_carlo.num_cases`` lowered so the suite
    stays fast; the seed is fixed on the CLI for a deterministic case sequence.
    """
    cfg = json.loads((CONFIG_DIR / f"{case.name}.json").read_text())
    cfg.setdefault("monte_carlo", {})["num_cases"] = case.mc_cases
    tmp_cfg = tmp / f"{case.name}_reduced.json"
    tmp_cfg.write_text(json.dumps(cfg))
    out = run_cli(tmp_cfg, tmp / case.name, seed=case.seed)
    summary = load_summary(out)
    stats = compute_stats(summary)
    return {
        "pk": float(stats.intercept_rate),
        "cep": float(stats.cep),
        "n": float(stats.n),
    }


def _eval_case(case: GoldenCase, tmp: Path) -> tuple[dict[str, float], int]:
    """Run one case, returning (metrics, schema_version)."""
    schema_version = _config_schema_version(case.name)
    if case.kind == "mc":
        return _run_mc(case, tmp), schema_version
    return _run_single(case, tmp), schema_version


def generate(cases: list[GoldenCase]) -> dict[str, Any]:
    """Run every case and return a golden.json-shaped baseline dict."""
    entries: dict[str, Any] = {}
    with tempfile.TemporaryDirectory(prefix="gncgold_") as td:
        tmp = Path(td)
        for case in cases:
            metrics, schema_version = _eval_case(case, tmp)
            entries[case.name] = {
                "schema_version": schema_version,
                "kind": case.kind,
                "seed": case.seed,
                "mc_cases": case.mc_cases,
                "metrics": metrics,
            }
    return {"format_version": GOLDEN_FORMAT_VERSION, "cases": entries}


def check(cases: list[GoldenCase], golden: dict[str, Any]) -> list[CaseResult]:
    """Run every case and compare to the stored golden; return per-case results."""
    stored_cases = golden.get("cases", {})
    results: list[CaseResult] = []
    with tempfile.TemporaryDirectory(prefix="gncgold_") as td:
        tmp = Path(td)
        for case in cases:
            metrics, schema_version = _eval_case(case, tmp)
            res = CaseResult(name=case.name, metrics=metrics, schema_version=schema_version)
            stored = stored_cases.get(case.name)

            if stored is None:
                res.passed = False
                res.diffs.append("no golden entry — run with --update to baseline it")
                results.append(res)
                continue

            if int(stored.get("schema_version", 1)) != schema_version:
                # Schema changed: flag for re-baseline, do NOT report silent metric diffs.
                res.schema_changed = True
                res.passed = False
                res.diffs.append(
                    f"schema changed (golden v{stored.get('schema_version', 1)} "
                    f"-> config v{schema_version}) — re-baseline needed (--update)"
                )
                results.append(res)
                continue

            golden_metrics = stored.get("metrics", {})
            for key, tol in case.tols.items():
                if key not in metrics:
                    continue
                if key not in golden_metrics:
                    res.passed = False
                    res.diffs.append(f"{key}: missing from golden")
                    continue
                g = float(golden_metrics[key])
                a = float(metrics[key])
                if not tol.within(g, a):
                    res.passed = False
                    rel = (abs(a - g) / abs(g)) if g != 0.0 else float("inf")
                    res.diffs.append(
                        f"{key}: golden={g:.6g} actual={a:.6g} "
                        f"|Δ|={abs(a - g):.6g} (rel={rel:.3%}) "
                        f"tol(abs={tol.abs:g}, rel={tol.rel:g})"
                    )
            results.append(res)
    return results


def print_report(results: list[CaseResult]) -> bool:
    """Print the check report; return True iff every case passed."""
    print("\nGolden-run regression check")
    print("=" * 78)
    print(f"  {'config':22s} {'result':10s}  detail")
    print("-" * 78)
    all_ok = True
    for r in results:
        all_ok &= r.passed
        if r.schema_changed:
            status = "SCHEMA"
        elif r.passed:
            status = "PASS"
        else:
            status = "FAIL"
        detail = "; ".join(r.diffs) if r.diffs else "all metrics within tolerance"
        print(f"  {r.name:22s} [{status:6s}]  {detail}")
    print("=" * 78)
    print(f"  {'ALL PASSED' if all_ok else 'REGRESSIONS DETECTED'}\n")
    return all_ok


def load_golden(path: Path = GOLDEN_PATH) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(
            f"golden baseline not found at {path} — generate it with "
            f"`python -m gncpost.golden --update`"
        )
    return json.loads(path.read_text())


def save_golden(data: dict[str, Any], path: Path = GOLDEN_PATH) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")


def _select_cases(only: list[str] | None) -> list[GoldenCase]:
    if not only:
        return CANONICAL
    by_name = {c.name: c for c in CANONICAL}
    missing = [n for n in only if n not in by_name]
    if missing:
        raise SystemExit(f"unknown config(s): {', '.join(missing)}")
    return [by_name[n] for n in only]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="gnc-sim golden-run regression harness")
    parser.add_argument(
        "--update",
        action="store_true",
        help="regenerate the golden baseline (postproc/golden/golden.json) instead of checking",
    )
    parser.add_argument(
        "--only",
        nargs="+",
        metavar="CONFIG",
        help="restrict to a subset of canonical configs (by name)",
    )
    args = parser.parse_args(argv)

    if not CLI_PATH.exists():
        print(
            f"native CLI not found at {CLI_PATH} — build it first (build-native)", file=sys.stderr
        )
        return 2

    cases = _select_cases(args.only)

    if args.update:
        data = generate(cases)
        # When updating a subset, merge into any existing baseline so we don't drop entries.
        if args.only and GOLDEN_PATH.exists():
            existing = load_golden()
            existing.setdefault("cases", {}).update(data["cases"])
            existing["format_version"] = GOLDEN_FORMAT_VERSION
            data = existing
        save_golden(data)
        covered = ", ".join(c.name for c in cases)
        print(f"wrote {GOLDEN_PATH.relative_to(REPO_ROOT)} ({len(cases)} cases: {covered})")
        return 0

    golden = load_golden()
    results = check(cases, golden)
    ok = print_report(results)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
