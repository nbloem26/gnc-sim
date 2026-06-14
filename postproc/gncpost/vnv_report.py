"""Auto-generated V&V / credibility report for gnc-sim (issue #49).

Emits a single Markdown report that ties together the project's credibility evidence so
it can be attached per release:

  * **NASA-STD-7009 credibility scoring** — every shipped model scored on the eight
    credibility factors (:mod:`gncpost.credibility`), with the governing (min) level and
    the evidence rationale behind each factor.
  * **Claim -> evidence trace** — each model's V&V-matrix claim and its benchmark /
    golden / analytic legs (:mod:`gncpost.vnv`).
  * **UQ propagation** — a deterministic per-time trajectory dispersion band summary
    (:func:`gncpost.uq.ensemble_band`), the trajectory analogue of the terminal CEP/Pk
    CIs from issue #33.
  * **Golden baseline** — the frozen headline metrics from ``postproc/golden/golden.json``
    (the regression net), surfaced so the report shows *what numbers are protected*.
  * **Matrix consistency** — the machine-checked V&V-matrix status (:func:`gncpost.vnv.check_consistency`).

The generator is **deterministic and self-contained**: it reads source/docs/JSON as text
(exactly like :mod:`gncpost.vnv`) and builds the UQ band from a **seeded** synthetic
ensemble, so it runs in CI without a native build and yields a byte-identical report for a
fixed seed. Run::

    python -m gncpost.vnv_report                 # writes docs/VNV_REPORT.md
    python -m gncpost.vnv_report --out report.md # custom path
    python -m gncpost.vnv_report --stdout        # print, don't write

No new dependencies (stdlib + numpy).
"""

from __future__ import annotations

import argparse
import datetime as _dt
from pathlib import Path

import numpy as np

from . import REPO_ROOT, credibility, uq, vnv

DEFAULT_OUT = REPO_ROOT / "docs" / "VNV_REPORT.md"

# Seed for the synthetic UQ ensemble so the trajectory-band summary is reproducible.
UQ_SEED = 0x7009


def _factor_legend() -> str:
    """A compact legend mapping the 0-4 level scale to its gloss (report header)."""
    lines = [f"- **{lvl}** — {gloss}" for lvl, gloss in sorted(credibility.LEVEL_GLOSS.items())]
    return "\n".join(lines)


def _credibility_table(a: credibility.CredibilityAssessment) -> str:
    """Markdown table: one row per model, one column per NASA-STD-7009 factor + min."""
    short = {k: lab for k, lab, _ in credibility.FACTORS}
    headers = ["Family", "Model", *[short[k] for k in credibility.FACTOR_KEYS], "**Min**", "Mean"]
    out = ["| " + " | ".join(headers) + " |"]
    out.append("|" + "|".join(["---"] * len(headers)) + "|")
    for m in a.models:
        cells = [m.family, f"`{m.model}`"]
        cells += [str(m.factors[k].level) for k in credibility.FACTOR_KEYS]
        cells += [f"**{m.min_level}**", f"{m.mean_level:.2f}"]
        out.append("| " + " | ".join(cells) + " |")
    return "\n".join(out)


def _evidence_sections(a: credibility.CredibilityAssessment) -> str:
    """Per-model claim -> evidence -> factor-rationale blocks (the audit trail)."""
    blocks: list[str] = []
    for m in a.models:
        lines = [f"#### `{m.model}` — {m.family}", "", f"**Claim.** {m.claim}", ""]
        lines.append(
            f"Governing credibility level (NASA-STD-7009 min factor): **{m.min_level}** / {credibility.MAX_LEVEL}"
        )
        lines.append("")
        lines.append("| Factor | Level | Evidence |")
        lines.append("|---|---|---|")
        for k in credibility.FACTOR_KEYS:
            f = m.factors[k]
            lines.append(f"| {f.label} | {f.level} | {f.rationale} |")
        blocks.append("\n".join(lines))
    return "\n\n".join(blocks)


def _golden_section() -> str:
    """Surface the frozen golden baseline metrics (the protected headline numbers)."""
    import json

    path = REPO_ROOT / "postproc" / "golden" / "golden.json"
    if not path.exists():
        return "_No golden baseline found._"
    data = json.loads(path.read_text())
    cases = data.get("cases", {})
    lines = ["| Golden case | Kind | Seed | Protected metrics |", "|---|---|---|---|"]
    for name in sorted(cases):
        c = cases[name]
        metrics = c.get("metrics", {})
        mtxt = ", ".join(f"`{k}`={_fmt(v)}" for k, v in sorted(metrics.items()))
        lines.append(f"| `{name}` | {c.get('kind', '')} | {c.get('seed', '')} | {mtxt} |")
    return "\n".join(lines)


def _fmt(v: float) -> str:
    """Compact numeric formatting for the golden-metrics table."""
    f = float(v)
    if f == int(f) and abs(f) < 1e6:
        return str(int(f))
    return f"{f:.4g}"


def _uq_section() -> str:
    """A deterministic trajectory-dispersion summary (UQ propagated across the flight).

    Builds a seeded synthetic ensemble of relative-range-vs-time trajectories (a homing
    intercept whose miss disperses, then collapses toward CPA), then reports the per-time
    dispersion band from :func:`gncpost.uq.ensemble_band`. This demonstrates the #49
    trajectory-UQ propagation deterministically without needing a native Monte-Carlo run;
    a live campaign would feed real ``(t, channel)`` case pairs into the same function.
    """
    rng = np.random.default_rng(UQ_SEED)
    n_cases = 64
    cases: list[tuple[np.ndarray, np.ndarray]] = []
    for _ in range(n_cases):
        # Each case: range closes from ~5 km to a small dispersed miss over a dispersed flight time.
        t_end = 14.0 + rng.normal(0.0, 0.4)
        t = np.linspace(0.0, max(t_end, 1.0), 280)
        miss = abs(rng.normal(0.0, 1.5))  # terminal miss [m]
        r0 = 5000.0
        # Smooth closure to the terminal miss, with mild per-case process noise.
        rel = (r0 - miss) * (1.0 - t / t[-1]) + miss
        rel = rel + rng.normal(0.0, 8.0, size=t.size) * (t / t[-1])
        cases.append((t, np.abs(rel)))

    band = uq.ensemble_band(cases, level=0.90, n_points=200)
    lines = [
        f"Synthetic seeded ensemble (`seed=0x{UQ_SEED:X}`, N={band.n}) of relative-range-vs-time, "
        "banded per time. This is the trajectory analogue of the terminal CEP/Pk confidence "
        "intervals (issue #33) — uncertainty propagated across the whole engagement.",
        "",
        f"- Central coverage of the band: **{band.level:.0%}**",
        f"- Peak per-time dispersion (max std over the flight): **{band.peak_std:.2f} m**",
        f"- Terminal dispersion (std at last sample): **{band.terminal_std:.2f} m**",
        f"- Time of peak dispersion: **{band.t_s[int(np.argmax(band.std))]:.2f} s**",
        "",
        "| t [s] | mean [m] | median [m] | 5th pct [m] | 95th pct [m] | std [m] |",
        "|---|---|---|---|---|---|",
    ]
    # Tabulate a downsampled set of rows so the report stays small.
    idx = np.linspace(0, band.t_s.size - 1, 9).astype(int)
    for i in idx:
        lines.append(
            f"| {band.t_s[i]:.2f} | {band.mean[i]:.2f} | {band.median[i]:.2f} | "
            f"{band.low[i]:.2f} | {band.high[i]:.2f} | {band.std[i]:.2f} |"
        )
    return "\n".join(lines)


def build_report(*, now: _dt.datetime | None = None) -> str:
    """Assemble the full Markdown V&V / credibility report as a string.

    Deterministic given the repo contents and the fixed ``UQ_SEED``. ``now`` is injected
    (defaulting to a fixed epoch) so the generated text is byte-stable for tests; the CLI
    passes the real timestamp.
    """
    a = credibility.assess()
    report = vnv.check_consistency()

    parts: list[str] = []
    parts.append("# gnc-sim V&V & Credibility Report")
    parts.append("")
    parts.append(
        "_Auto-generated by `python -m gncpost.vnv_report` (issue #49). "
        "Do not edit by hand — regenerate._"
    )
    if now is not None:
        parts.append("")
        parts.append(f"Generated: {now.isoformat(timespec='seconds')}")
    parts.append("")
    parts.append(
        "This report assesses every **shipped model** (the set resolved by "
        "`core/src/model/Registry.cpp`) on the eight credibility factors of "
        "**NASA-STD-7009A**, tracing each model's claim to its evidence (GoogleTest "
        "benchmark, frozen golden baseline, analytic check) and summarizing the "
        "uncertainty-quantification and regression status. It scores the *evidence on "
        "hand* — not the *adequacy* of any model for a particular decision, which "
        "remains the project's call (per NASA-STD-7009 §4)."
    )

    # 1. Executive summary
    parts.append("\n## 1. Executive summary\n")
    parts.append(
        f"- Shipped models scored: **{len(a.models)}**"
        + (f" (uncovered: {len(a.uncovered)})" if a.uncovered else " (all covered)")
    )
    parts.append(
        f"- Programme governing credibility level (min over all models/factors): "
        f"**{a.overall_min_level()}** / {credibility.MAX_LEVEL}"
    )
    parts.append(
        f"- Programme mean factor level: **{a.overall_mean_level():.2f}** / {credibility.MAX_LEVEL}"
    )
    parts.append(
        f"- V&V-matrix consistency: **{'CONSISTENT' if report.ok else 'DRIFT — ' + str(len(report.problems)) + ' problem(s)'}**"
    )
    if not report.ok:
        for p in report.problems:
            parts.append(f"  - DRIFT: {p}")

    # 2. Scoring methodology / legend
    parts.append("\n## 2. NASA-STD-7009 factors & level scale\n")
    parts.append(
        "Each model is scored 0-4 on the eight NASA-STD-7009A credibility factors. The "
        "governing per-model level is the **minimum** across factors (a credibility chain "
        "is as strong as its weakest link). Level scale:"
    )
    parts.append("")
    parts.append(_factor_legend())
    parts.append("")
    parts.append("Factors by category:")
    parts.append("")
    last_cat = None
    for _k, label, cat in credibility.FACTORS:
        if cat != last_cat:
            parts.append(f"- *{cat}*:")
            last_cat = cat
        parts.append(f"  - **{label}**")

    # 3. Per-model credibility table
    parts.append("\n## 3. Credibility scores (per model)\n")
    parts.append(_credibility_table(a))
    if a.uncovered:
        parts.append("")
        parts.append("Uncovered shipped models (no V&V-matrix row): " + ", ".join(a.uncovered))

    # 4. Claim -> evidence trace
    parts.append("\n## 4. Claim → evidence → score (per model)\n")
    parts.append(_evidence_sections(a))

    # 5. UQ propagation
    parts.append("\n## 5. Uncertainty quantification (propagated across the engagement)\n")
    parts.append(_uq_section())

    # 6. Golden baseline
    parts.append("\n## 6. Frozen golden baseline (protected headline metrics)\n")
    parts.append(_golden_section())

    parts.append("")
    return "\n".join(parts) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Generate the gnc-sim V&V / credibility report")
    parser.add_argument(
        "--out",
        type=Path,
        default=DEFAULT_OUT,
        help=f"output Markdown path (default: {DEFAULT_OUT.relative_to(REPO_ROOT)})",
    )
    parser.add_argument("--stdout", action="store_true", help="print the report instead of writing")
    parser.add_argument(
        "--no-timestamp",
        action="store_true",
        help="omit the generated-at timestamp (byte-stable output for diffs/tests)",
    )
    args = parser.parse_args(argv)

    # timezone.utc (not datetime.UTC) — the latter is Python 3.11+ only; keep 3.10 compat.
    now = None if args.no_timestamp else _dt.datetime.now(_dt.timezone.utc)  # noqa: UP017
    text = build_report(now=now)

    if args.stdout:
        print(text)
        return 0
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(text)
    print(f"wrote {args.out} ({len(text)} bytes, {text.count(chr(10))} lines)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
