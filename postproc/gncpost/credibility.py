"""NASA-STD-7009 credibility assessment for the gnc-sim model set (issue #49).

A deterministic, machine-derived credibility score for **every shipped model**, built
on the V&V-matrix evidence (:mod:`gncpost.vnv`), the per-model documentation
(``docs/MODELS.md``) and the frozen golden baseline (``postproc/golden/golden.json``).

The scoring follows the eight **credibility factors** of *NASA-STD-7009A — Standard
for Models and Simulations*. The standard groups the factors into three categories and
scores each on a 0-4 *level* scale (0 = no evidence, 4 = formal/comprehensive). The eight
factors (verbatim factor names from NASA-STD-7009A §4.3 / Table 1):

  M&S Development
    1. Verification            — was the math/code solved right (vs the intended model)?
    2. Validation              — does the model match the referent (real world / closed form)?
  M&S Operation
    3. Input Pedigree          — provenance/quality of the inputs and assumptions.
    4. Results Uncertainty     — is output uncertainty characterized?
    5. Results Robustness      — sensitivity / coverage of the parameter space explored.
  Supporting Evidence
    6. Use History             — has this M&S been used/regression-protected before?
    7. M&S Management          — configuration management, process discipline.
    8. People Qualification    — qualification of the staff / review.

This module does **not** assert the *adequacy* of the M&S for any particular decision
(that is the project's call); it scores the *evidence on hand*, traceably, so a V&V
report can show claim -> evidence -> score. Each per-model factor score is derived from a
machine-checkable signal:

  * Verification        <- the V&V-matrix Benchmark leg(s) (GoogleTest cases under ``tests/``).
  * Validation          <- the V&V-matrix Analytic leg (closed-form / statistical check).
  * Input Pedigree      <- the model's ``docs/MODELS.md`` page carrying **References** + **Assumptions**.
  * Results Uncertainty <- a frozen Golden baseline entry (+ the campaign-level UQ for MC models).
  * Results Robustness  <- the model's ``docs/MODELS.md`` **Validity limits** section.
  * Use History         <- project level: the golden harness + CI matrix-consistency leg.
  * M&S Management       <- project level: version control + machine-checked V&V matrix.
  * People Qualification <- project level: documented review/PR workflow.

Pure stdlib + numpy. No new dependencies, no native build (reads source/docs/JSON as text,
exactly like :mod:`gncpost.vnv`).
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

from . import REPO_ROOT, vnv

MODELS_DOC_PATH = REPO_ROOT / "docs" / "MODELS.md"

# The eight NASA-STD-7009A credibility factors, in standard order, with their category.
# (key, human label, category) — the key is a stable identifier used in the report.
FACTORS: list[tuple[str, str, str]] = [
    ("verification", "Verification", "M&S Development"),
    ("validation", "Validation", "M&S Development"),
    ("input_pedigree", "Input Pedigree", "M&S Operation"),
    ("results_uncertainty", "Results Uncertainty", "M&S Operation"),
    ("results_robustness", "Results Robustness", "M&S Operation"),
    ("use_history", "Use History", "Supporting Evidence"),
    ("ms_management", "M&S Management", "Supporting Evidence"),
    ("people_qualification", "People Qualification", "Supporting Evidence"),
]

FACTOR_KEYS: list[str] = [k for k, _, _ in FACTORS]
FACTOR_LABELS: dict[str, str] = {k: label for k, label, _ in FACTORS}
FACTOR_CATEGORY: dict[str, str] = {k: cat for k, _, cat in FACTORS}

# NASA-STD-7009A scores each factor on a 0-4 level scale. We use the same range so a
# report reader can map our scores onto the standard's achievement levels.
MAX_LEVEL = 4

# Human-readable gloss of each level (generic across factors; the standard defines
# factor-specific level descriptors, summarized here for the report legend).
LEVEL_GLOSS: dict[int, str] = {
    0: "no evidence",
    1: "informal / minimal",
    2: "partial, some rigor",
    3: "substantial, traceable",
    4: "formal / comprehensive",
}


@dataclass(frozen=True)
class FactorScore:
    """One NASA-STD-7009 factor's level (0-4) plus the evidence string behind it."""

    key: str
    level: int
    rationale: str

    @property
    def label(self) -> str:
        return FACTOR_LABELS[self.key]

    @property
    def category(self) -> str:
        return FACTOR_CATEGORY[self.key]


@dataclass(frozen=True)
class ModelCredibility:
    """Per-model credibility: the eight factor scores + the matrix claim it backs."""

    family: str
    model: str
    claim: str
    factors: dict[str, FactorScore]

    @property
    def min_level(self) -> int:
        """The governing NASA-STD-7009 score is the **minimum** factor level.

        The standard's "credibility level" of an assessment is bounded by its weakest
        factor (a chain is as strong as its weakest link) — so the headline per-model
        score is the min across factors, not an average that could mask a 0.
        """
        return min(f.level for f in self.factors.values())

    @property
    def mean_level(self) -> float:
        """Mean factor level — a softer summary shown alongside the governing min."""
        levels = [f.level for f in self.factors.values()]
        return sum(levels) / len(levels)


# --------------------------------------------------------------------------------------
# docs/MODELS.md parsing — per-model section evidence (pedigree + validity limits)
# --------------------------------------------------------------------------------------


@dataclass(frozen=True)
class ModelDoc:
    """The signals we read out of a model's ``docs/MODELS.md`` section."""

    has_references: bool
    has_assumptions: bool
    has_validity_limits: bool
    has_governing_eq: bool


def parse_model_docs(path: Path = MODELS_DOC_PATH) -> dict[str, ModelDoc]:
    """Map model key -> the documentation signals in its ``### \\`key\\` —`` section.

    Each model gets a ``### \\`<key>\\` — <name>`` heading followed by bullets. We detect
    the bold-labelled bullets (**Assumptions.**, **Governing equation.**, **Validity
    limits.**, **References.**) the page convention uses. A model may appear once even
    though a config key (e.g. ``round``) has two sub-sections — we OR the signals.
    """
    text = path.read_text()
    # Split into ### sections; capture the first backticked token of each heading as key.
    heading_re = re.compile(r"^###\s+`([^`]+)`", re.MULTILINE)
    matches = list(heading_re.finditer(text))
    docs: dict[str, ModelDoc] = {}
    for i, m in enumerate(matches):
        key = m.group(1)
        start = m.end()
        end = matches[i + 1].start() if i + 1 < len(matches) else len(text)
        body = text[start:end].lower()
        doc = ModelDoc(
            has_references="**references." in body or "**reference." in body,
            has_assumptions="**assumptions." in body or "**assumption." in body,
            has_validity_limits="**validity limit" in body,
            has_governing_eq="**governing equation" in body or "**governing equations" in body,
        )
        if key in docs:
            prev = docs[key]
            docs[key] = ModelDoc(
                has_references=prev.has_references or doc.has_references,
                has_assumptions=prev.has_assumptions or doc.has_assumptions,
                has_validity_limits=prev.has_validity_limits or doc.has_validity_limits,
                has_governing_eq=prev.has_governing_eq or doc.has_governing_eq,
            )
        else:
            docs[key] = doc
    return docs


# --------------------------------------------------------------------------------------
# Factor scoring rules
# --------------------------------------------------------------------------------------


def _score_verification(rows: list[vnv.MatrixRow]) -> FactorScore:
    """Verification level from the GoogleTest benchmark legs across the model's rows.

    More independent benchmark files -> stronger verification. We cap the level so that
    even a single deterministic unit benchmark earns level 3 (substantial), and >=2
    distinct benchmark files (e.g. unit + integration) earns the full level 4.
    """
    files: set[str] = set()
    cases = 0
    for r in rows:
        for f in r.benchmark_files:
            files.add(f)
        # Count backticked benchmark tokens as a proxy for case count.
        cases += len(vnv._backticked(r.benchmark))
    if not files:
        return FactorScore("verification", 0, "no GoogleTest benchmark in the V&V matrix")
    level = 3 if len(files) == 1 else 4
    return FactorScore(
        "verification",
        level,
        f"{cases} GoogleTest case(s) across {len(files)} suite file(s): "
        + ", ".join(sorted(f"tests/{f}.cpp" for f in files)),
    )


def _score_validation(rows: list[vnv.MatrixRow]) -> FactorScore:
    """Validation level from the Analytic leg (closed-form / statistical check).

    A closed-form/statistical validation in ``postproc/`` earns level 3; if the model is
    *also* frozen end-to-end in a golden run (independent regression referent) we read
    that as corroborating validation evidence and award level 4. No analytic leg -> the
    claim is verified but not independently validated against a referent (level 1).
    """
    analytic = [r.analytic for r in rows if r.analytic != vnv.NONE_CELL and r.analytic.strip()]
    golden = sorted({gk for r in rows for gk in r.golden_keys})
    if analytic:
        level = 4 if golden else 3
        detail = "; ".join(sorted(set(analytic)))
        gtxt = f" (corroborated by golden {', '.join(golden)})" if golden else ""
        return FactorScore("validation", level, f"analytic check: {detail}{gtxt}")
    if golden:
        return FactorScore(
            "validation",
            2,
            f"end-to-end golden referent only ({', '.join(golden)}); no closed-form check",
        )
    return FactorScore("validation", 1, "no analytic / golden validation referent")


def _score_input_pedigree(doc: ModelDoc | None) -> FactorScore:
    """Input Pedigree from the documented assumptions + references in MODELS.md."""
    if doc is None:
        return FactorScore("input_pedigree", 0, "no docs/MODELS.md section")
    if doc.has_references and doc.has_assumptions:
        return FactorScore(
            "input_pedigree", 3, "documented assumptions + literature references in MODELS.md"
        )
    if doc.has_assumptions or doc.has_references:
        return FactorScore(
            "input_pedigree", 2, "partial MODELS.md provenance (assumptions OR refs)"
        )
    return FactorScore(
        "input_pedigree", 1, "MODELS.md section present but no assumptions/references"
    )


def _score_results_uncertainty(rows: list[vnv.MatrixRow], is_mc: bool) -> FactorScore:
    """Results Uncertainty from the frozen golden metrics (+ campaign UQ for MC models).

    A golden baseline pins the headline outputs with explicit tolerances (a bounded,
    monitored output band). Monte-Carlo models additionally carry the issue-#33 UQ stack
    (bootstrap CIs, convergence, Sobol, trajectory dispersion bands), which is a full
    characterization of output uncertainty -> level 4. A frozen single-run golden alone is
    level 3; no golden -> level 1 (outputs not pinned).
    """
    golden = sorted({gk for r in rows for gk in r.golden_keys})
    if is_mc and golden:
        return FactorScore(
            "results_uncertainty",
            4,
            f"Monte-Carlo UQ (bootstrap CI + Sobol + trajectory bands) on golden {', '.join(golden)}",
        )
    if golden:
        return FactorScore(
            "results_uncertainty",
            3,
            f"output pinned with tolerances in golden {', '.join(golden)}",
        )
    return FactorScore(
        "results_uncertainty", 1, "no frozen golden output band; uncertainty not characterized"
    )


def _score_results_robustness(doc: ModelDoc | None, rows: list[vnv.MatrixRow]) -> FactorScore:
    """Results Robustness from documented validity limits + breadth of benchmark cases.

    Robustness is about how far the model has been exercised / where it stops being valid.
    Documented validity limits (the envelope) plus multiple benchmark cases (the parameter
    coverage) earns level 3; either alone is level 2; neither is level 1.
    """
    n_cases = sum(len(vnv._backticked(r.benchmark)) for r in rows)
    has_limits = doc is not None and doc.has_validity_limits
    if has_limits and n_cases >= 2:
        return FactorScore(
            "results_robustness",
            3,
            f"documented validity limits + {n_cases} benchmark cases spanning the envelope",
        )
    if has_limits or n_cases >= 2:
        why = "validity limits documented" if has_limits else f"{n_cases} benchmark cases"
        return FactorScore("results_robustness", 2, why)
    return FactorScore("results_robustness", 1, "limited robustness evidence")


# Project-level (organizational) factors. These are uniform across models because the
# evidence is the *process*, not the individual model: the golden regression harness, the
# machine-checked V&V matrix in CI, version control, and the documented PR/review workflow
# (AGENTS.md). NASA-STD-7009 treats Use History / M&S Management / People Qualification as
# supporting-evidence factors that attach to the M&S programme, which is what these are.
def _project_factors() -> dict[str, FactorScore]:
    return {
        "use_history": FactorScore(
            "use_history",
            3,
            "golden-run regression harness + CI V&V-matrix consistency leg protect every release",
        ),
        "ms_management": FactorScore(
            "ms_management",
            3,
            "git version control + machine-checked V&V matrix (test_vnv_matrix.py) gate changes",
        ),
        "people_qualification": FactorScore(
            "people_qualification",
            2,
            "documented PR/review workflow + conventional commits (AGENTS.md)",
        ),
    }


# --------------------------------------------------------------------------------------
# Assessment
# --------------------------------------------------------------------------------------


def _is_mc_model(rows: list[vnv.MatrixRow]) -> bool:
    """A model is treated as Monte-Carlo-exercised iff a row references the ``montecarlo`` golden."""
    return any("montecarlo" in r.golden_keys for r in rows)


@dataclass
class CredibilityAssessment:
    """The full per-model NASA-STD-7009 assessment over the shipped model set."""

    models: list[ModelCredibility] = field(default_factory=list)
    # Models named in Registry.cpp but absent from the V&V matrix (scored 0 throughout).
    uncovered: list[str] = field(default_factory=list)

    @property
    def factor_keys(self) -> list[str]:
        return FACTOR_KEYS

    def overall_min_level(self) -> int:
        """Programme-level governing score = min over every model's governing min."""
        if not self.models:
            return 0
        return min(m.min_level for m in self.models)

    def overall_mean_level(self) -> float:
        if not self.models:
            return 0.0
        return sum(m.mean_level for m in self.models) / len(self.models)


def assess(
    matrix_path: Path = vnv.VNV_MATRIX_PATH,
    registry_path: Path = vnv.REGISTRY_PATH,
    models_doc_path: Path = MODELS_DOC_PATH,
) -> CredibilityAssessment:
    """Score every shipped model on the eight NASA-STD-7009 credibility factors.

    Cross-references the V&V matrix (evidence legs), ``Registry.cpp`` (the authoritative
    shipped-model set, via :func:`gncpost.vnv.parse_shipped_models`) and ``MODELS.md``
    (documentation signals). Deterministic: pure text/JSON reads, no RNG, no native build.
    """
    rows = vnv.parse_matrix(matrix_path)
    shipped = vnv.parse_shipped_models(registry_path)
    docs = parse_model_docs(models_doc_path)

    # Index matrix rows by bare model key (strip the `(config key)` annotation).
    rows_by_model: dict[str, list[vnv.MatrixRow]] = {}
    family_by_model: dict[str, str] = {}
    claim_by_model: dict[str, str] = {}
    for r in rows:
        toks = vnv._backticked(r.model)
        key = toks[0] if toks else r.model
        rows_by_model.setdefault(key, []).append(r)
        family_by_model.setdefault(key, r.family)
        claim_by_model.setdefault(key, r.claim)

    project = _project_factors()

    assessment = CredibilityAssessment()
    # Iterate the shipped set in a stable (family, key) order for deterministic output.
    for family in (
        "Guidance",
        "Navigation",
        "Dynamics",
        "Sensor",
        "Tracking",
        "Environment",
        "Threat",
    ):
        for key in shipped.get(family, []):
            mrows = rows_by_model.get(key)
            if not mrows:
                assessment.uncovered.append(f"{family}:{key}")
                continue
            doc = docs.get(key)
            is_mc = _is_mc_model(mrows)
            factors: dict[str, FactorScore] = {
                "verification": _score_verification(mrows),
                "validation": _score_validation(mrows),
                "input_pedigree": _score_input_pedigree(doc),
                "results_uncertainty": _score_results_uncertainty(mrows, is_mc),
                "results_robustness": _score_results_robustness(doc, mrows),
            }
            factors.update(project)
            assessment.models.append(
                ModelCredibility(
                    family=family_by_model.get(key, family),
                    model=key,
                    claim=claim_by_model.get(key, ""),
                    factors=factors,
                )
            )
    return assessment


def main() -> int:
    """CLI: print a compact per-model credibility table; exit 0 always (report-only)."""
    a = assess()
    print("\nNASA-STD-7009 credibility assessment")
    print("=" * 78)
    header = f"  {'model':16s} {'family':12s} " + " ".join(k[:4] for k in FACTOR_KEYS) + "  min"
    print(header)
    print("-" * 78)
    for m in a.models:
        levels = " ".join(f"{m.factors[k].level:>4d}" for k in FACTOR_KEYS)
        print(f"  {m.model:16s} {m.family:12s} {levels}  {m.min_level:>3d}")
    print("-" * 78)
    print(
        f"  models scored: {len(a.models)}   uncovered: {len(a.uncovered)}   "
        f"programme min level: {a.overall_min_level()}   "
        f"mean: {a.overall_mean_level():.2f} / {MAX_LEVEL}"
    )
    if a.uncovered:
        print(f"  [uncovered] {', '.join(a.uncovered)}")
    print("=" * 78 + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
