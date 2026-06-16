"""V&V-matrix consistency tooling for gnc-sim.

The V&V matrix ([`docs/VNV_MATRIX.md`](../../docs/VNV_MATRIX.md)) maps every shipped
model to its evidence (the GoogleTest benchmark, the golden-run baseline entry, and/or the
analytic check in ``postproc/``). This module is the **regression leg** that keeps the
matrix honest: it parses the matrix table, the set of shipped models declared in
[`core/src/model/Registry.cpp`](../../core/src/model/Registry.cpp), and the golden baseline
([`postproc/golden/golden.json`](../golden/golden.json)), then reports any drift:

* a shipped model with **no** matrix row,
* a matrix row whose **Golden** key is not an actual case in ``golden.json``,
* a matrix row whose **Benchmark** names a non-existent ``tests/*_test.cpp`` file,
* a matrix row with **no** evidence leg at all.

It is pure tooling (no C++ build, no CLI): it reads source/docs/JSON as text. The pytest
``postproc/tests/test_vnv_matrix.py`` asserts ``check_consistency()`` finds no problems, so
CI fails if the matrix and the shipped model set drift apart.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path

from . import REPO_ROOT

VNV_MATRIX_PATH = REPO_ROOT / "docs" / "VNV_MATRIX.md"
REGISTRY_PATH = REPO_ROOT / "core" / "src" / "model" / "Registry.cpp"
GOLDEN_PATH = REPO_ROOT / "postproc" / "golden" / "golden.json"
TESTS_DIR = REPO_ROOT / "tests"

# Cell value used to mean "no evidence of this kind for this row".
NONE_CELL = "—"


@dataclass(frozen=True)
class MatrixRow:
    """One parsed V&V-matrix row."""

    family: str
    model: str
    claim: str
    benchmark: str
    golden: str
    analytic: str

    @property
    def golden_keys(self) -> list[str]:
        """Golden case keys referenced by this row (``[]`` if the cell is ``—``)."""
        return _backticked(self.golden) if self.golden != NONE_CELL else []

    @property
    def benchmark_files(self) -> list[str]:
        """``tests/*_test.cpp`` stems referenced by the Benchmark cell.

        Benchmark cells look like ``gnc_test::ProNav.*`` — the token before ``::`` is the
        test-file stem (``gnc_test`` -> ``tests/gnc_test.cpp``).
        """
        files = []
        for tok in _backticked(self.benchmark):
            # Only ``file::Suite.Name`` tokens name a file; a bare ``Suite.Name`` token
            # (no ``::``) is a sibling case under the previous file, not a new file.
            if "::" not in tok:
                continue
            stem = tok.split("::", 1)[0].strip()
            if stem:
                files.append(stem)
        return files

    @property
    def has_evidence(self) -> bool:
        """True iff at least one evidence leg (benchmark / golden / analytic) is present."""
        return bool(
            self.benchmark_files
            or self.golden_keys
            or (self.analytic != NONE_CELL and self.analytic.strip())
        )


@dataclass
class ConsistencyReport:
    """Outcome of cross-checking the matrix against reality."""

    rows: list[MatrixRow] = field(default_factory=list)
    shipped: dict[str, list[str]] = field(default_factory=dict)
    problems: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.problems


def _backticked(cell: str) -> list[str]:
    """Extract the `code-spanned` tokens from a Markdown cell.

    Cells reference benchmarks / golden keys / configs as inline code (`` `like_this` ``).
    Pull each backticked span; ignore prose. The ``config key`` annotation tokens (which
    contain a ``.``/``[]``, e.g. ``guidance.law``) are config keys, not evidence, so this is
    only used on the evidence columns where backticked spans are evidence identifiers.
    """
    return [m.strip() for m in re.findall(r"`([^`]+)`", cell)]


def _split_md_row(line: str) -> list[str]:
    """Split a Markdown table row ``| a | b | … |`` into trimmed cells."""
    # Drop the leading/trailing pipe, then split. Escaped pipes aren't used here.
    inner = line.strip().strip("|")
    return [c.strip() for c in inner.split("|")]


def parse_matrix(path: Path = VNV_MATRIX_PATH) -> list[MatrixRow]:
    """Parse the 6-column evidence table out of ``docs/VNV_MATRIX.md``.

    The relevant table is identified by its header row containing both ``Family`` and
    ``Benchmark``. Only data rows (after the ``|---|`` separator) are returned.
    """
    text = path.read_text()
    rows: list[MatrixRow] = []
    in_table = False
    for raw in text.splitlines():
        line = raw.strip()
        if not line.startswith("|"):
            in_table = False
            continue
        cells = _split_md_row(line)
        # Identify the evidence table by its header.
        if not in_table:
            if "Family" in cells and any(c.startswith("Benchmark") for c in cells):
                in_table = True
            continue
        # Skip the |---|---| separator row.
        if all(set(c) <= {"-", ":"} and c for c in cells):
            continue
        if len(cells) != 6:
            continue
        rows.append(
            MatrixRow(
                family=cells[0],
                model=cells[1],
                claim=cells[2],
                benchmark=cells[3],
                golden=cells[4],
                analytic=cells[5],
            )
        )
    return rows


def parse_shipped_models(path: Path = REGISTRY_PATH) -> dict[str, list[str]]:
    """Extract the set of shipped models from ``Registry.cpp``.

    Each ``makeX`` resolver dispatches on a string key via ``if (key == "value")``. We map
    the resolver family -> the list of literal keys it accepts. This is the authoritative
    set of shipped models the matrix must cover.
    """
    text = path.read_text()
    # Family -> (resolver function name, the config-string param name is incidental).
    resolvers = {
        "Guidance": "makeGuidance",
        "Navigation": "makeNavigator",
        "Sensor": "makeSensor",
        "Dynamics": "makeDynamics",
        "Environment": "makeEnvironment",
        "Threat": "makeThreat",
    }
    shipped: dict[str, list[str]] = {}
    for family, fn in resolvers.items():
        # Slice from the resolver definition to the next resolver or end of namespace.
        start = text.find(f"::{fn}(")
        if start < 0:
            shipped[family] = []
            continue
        # Find the body up to the closing of the function (next "std::unique_ptr<I" def or EOF).
        rest = text[start:]
        nxt = re.search(r"\n\}\n", rest)
        body = rest[: nxt.end()] if nxt else rest
        # Resolvers dispatch on string literals via either ``key == "x"`` (positive
        # dispatch) or ``key != "x"`` (the sensor resolver validates with negation, then
        # picks the type with a ternary), so capture both comparison forms.
        keys = [m for m in re.findall(r'[!=]=\s*"([^"]+)"', body)]
        # De-dup, preserve order.
        seen: list[str] = []
        for k in keys:
            if k not in seen:
                seen.append(k)
        shipped[family] = seen
    return shipped


def check_consistency(
    matrix_path: Path = VNV_MATRIX_PATH,
    registry_path: Path = REGISTRY_PATH,
    golden_path: Path = GOLDEN_PATH,
    tests_dir: Path = TESTS_DIR,
) -> ConsistencyReport:
    """Cross-check the V&V matrix against the shipped model set + golden + tests.

    Returns a :class:`ConsistencyReport`; ``report.ok`` is True iff no drift was found.
    """
    rows = parse_matrix(matrix_path)
    shipped = parse_shipped_models(registry_path)
    golden = json.loads(golden_path.read_text())
    golden_cases = set(golden.get("cases", {}))

    report = ConsistencyReport(rows=rows, shipped=shipped)

    # Index matrix rows by the bare model key (strip the `(config key)` annotation).
    def bare_model(row: MatrixRow) -> str:
        # Model cell looks like ``\`pronav\` (\`guidance.law\`)`` -> first backticked token.
        toks = _backticked(row.model)
        return toks[0] if toks else row.model

    rows_by_model: dict[str, list[MatrixRow]] = {}
    for r in rows:
        rows_by_model.setdefault(bare_model(r), []).append(r)

    # (1) Every shipped model has >=1 row.
    for family, keys in shipped.items():
        for key in keys:
            if key not in rows_by_model:
                report.problems.append(f"shipped {family} model '{key}' has no V&V-matrix row")

    # (2) Every golden key in a row is a real golden case.
    for r in rows:
        for gk in r.golden_keys:
            if gk not in golden_cases:
                report.problems.append(
                    f"row '{bare_model(r)}' references golden key '{gk}' absent from golden.json"
                )

    # (3) Every benchmark file referenced exists under tests/.
    for r in rows:
        for stem in r.benchmark_files:
            if not (tests_dir / f"{stem}.cpp").exists():
                report.problems.append(
                    f"row '{bare_model(r)}' references benchmark file "
                    f"'tests/{stem}.cpp' which does not exist"
                )

    # (4) Every row has at least one evidence leg.
    for r in rows:
        if not r.has_evidence:
            report.problems.append(
                f"row '{bare_model(r)}' has no evidence (benchmark / golden / analytic)"
            )

    return report


def main() -> int:
    """CLI: print the consistency report; exit 0 if clean, 1 on drift."""
    report = check_consistency()
    print("\nV&V-matrix consistency check")
    print("=" * 70)
    n_shipped = sum(len(v) for v in report.shipped.values())
    print(f"  shipped models: {n_shipped}   matrix rows: {len(report.rows)}")
    print("-" * 70)
    if report.ok:
        print("  ALL CONSISTENT — every shipped model has a row; all keys resolve\n")
        return 0
    for p in report.problems:
        print(f"  [DRIFT] {p}")
    print("=" * 70)
    print(f"  {len(report.problems)} problem(s) — V&V matrix is out of sync\n")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
