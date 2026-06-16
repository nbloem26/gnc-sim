"""Golden-run regression test (invokes the native CLI).

Runs a fast subset of the canonical configs through the golden harness in CHECK
mode against the committed baseline (``postproc/golden/golden.json``) and asserts
no regression. Skipped automatically if the build-native binary is absent.

The full suite (all canonical configs incl. Monte-Carlo) runs in CI via
``.github/workflows/golden.yml``; this test keeps the local/pytest loop quick by
covering a representative single-run subset.
"""

from __future__ import annotations

from conftest import requires_cli
from gncpost import golden


@requires_cli()
def test_golden_baseline_exists():
    """The committed golden baseline is present and well-formed."""
    g = golden.load_golden()
    assert "cases" in g and g["cases"], "golden.json has no cases"
    # Every canonical case must have a committed entry.
    for case in golden.CANONICAL:
        assert case.name in g["cases"], f"missing golden entry for {case.name}"
        assert "schema_version" in g["cases"][case.name]


@requires_cli()
def test_golden_no_regression_fast_subset():
    """A representative subset matches the committed golden within tolerance."""
    g = golden.load_golden()
    subset = golden._select_cases(["homing_3dof", "projectile_3dof", "track_fused"])
    results = golden.check(subset, g)
    failures = [r for r in results if not r.passed]
    detail = "; ".join(f"{r.name}: {', '.join(r.diffs)}" for r in failures)
    assert not failures, f"golden regression: {detail}"


@requires_cli()
def test_golden_monte_carlo_no_regression():
    """The reduced Monte-Carlo Pk / CEP stay within their (looser) tolerances."""
    g = golden.load_golden()
    subset = golden._select_cases(["montecarlo"])
    results = golden.check(subset, g)
    assert all(r.passed for r in results), "; ".join(
        f"{r.name}: {', '.join(r.diffs)}" for r in results if not r.passed
    )
