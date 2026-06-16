"""V&V-matrix consistency tests (issue #34).

These pure-Python tests (no native build needed) enforce that the V&V matrix
([`docs/VNV_MATRIX.md`](../../docs/VNV_MATRIX.md)) stays in sync with reality:

* every model shipped by `core/src/model/Registry.cpp` has at least one matrix row;
* every **Golden** key named in the matrix is a real case in `golden.json`;
* every **Benchmark** names a real `tests/*_test.cpp` file;
* every row carries at least one evidence leg.

If a new model is added without a matrix row (or a row references a stale benchmark /
golden key), these fail — which is exactly the regression net the issue asks for.
"""

from __future__ import annotations

from gncpost import vnv


def test_matrix_parses_with_rows():
    """The matrix table parses into well-formed rows."""
    rows = vnv.parse_matrix()
    assert rows, "no rows parsed from docs/VNV_MATRIX.md"
    for r in rows:
        assert r.family and r.model and r.claim, f"incomplete row: {r}"


def test_shipped_models_discovered():
    """Registry.cpp yields the expected model families and keys."""
    shipped = vnv.parse_shipped_models()
    # Spot-check each family resolved at least its known keys (guards the parser).
    assert "pronav" in shipped["Guidance"] and "apn" in shipped["Guidance"]
    assert "ekf" in shipped["Navigation"] and "imm" in shipped["Navigation"]
    assert "6dof_hifi" in shipped["Dynamics"]
    assert set(shipped["Sensor"]) >= {"radar", "ir"}
    assert set(shipped["Environment"]) >= {"flat", "round"}
    assert set(shipped["Threat"]) >= {"constant", "weave"}


def test_every_shipped_model_has_a_row():
    """No shipped model may lack a V&V-matrix row."""
    report = vnv.check_consistency()
    missing = [p for p in report.problems if "has no V&V-matrix row" in p]
    assert not missing, "models without a matrix row:\n" + "\n".join(missing)


def test_golden_keys_resolve():
    """Every Golden key referenced by the matrix exists in golden.json."""
    report = vnv.check_consistency()
    bad = [p for p in report.problems if "golden key" in p]
    assert not bad, "matrix references golden keys not in golden.json:\n" + "\n".join(bad)


def test_benchmark_files_exist():
    """Every Benchmark cell names a real tests/*_test.cpp file."""
    report = vnv.check_consistency()
    bad = [p for p in report.problems if "benchmark file" in p]
    assert not bad, "matrix references missing benchmark files:\n" + "\n".join(bad)


def test_every_row_has_evidence():
    """Every matrix row carries at least one evidence leg."""
    report = vnv.check_consistency()
    bad = [p for p in report.problems if "has no evidence" in p]
    assert not bad, "rows with no evidence:\n" + "\n".join(bad)


def test_matrix_fully_consistent():
    """The aggregate consistency check passes (the regression leg for the matrix)."""
    report = vnv.check_consistency()
    assert report.ok, "V&V-matrix drift:\n" + "\n".join(report.problems)
