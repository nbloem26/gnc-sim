"""NASA-STD-7009 credibility-scoring + V&V-report tests (issue #49).

Pure-Python, deterministic (no native build): the scorer reads the V&V matrix, the
shipped-model set and ``docs/MODELS.md`` as text; the report generator and the UQ
trajectory band are seeded. These guard:

* the eight NASA-STD-7009 factors are scored for **every** shipped model (cross-checked
  against the same model set ``gncpost.vnv`` enforces);
* the factor rules assign the expected level on a **known** evidence set;
* the report generator runs and covers every shipped model;
* the UQ trajectory-dispersion band is deterministic and behaves correctly.
"""

from __future__ import annotations

import numpy as np
import pytest
from gncpost import credibility, uq, vnv

# --------------------------------------------------------------------------------------
# Coverage: every shipped model is scored on all eight factors
# --------------------------------------------------------------------------------------


def test_assessment_covers_every_shipped_model():
    """The assessment scores exactly the shipped models the V&V matrix enforces."""
    a = credibility.assess()
    shipped = vnv.parse_shipped_models()
    # Flatten the shipped set the consistency check governs (the 6 Registry resolvers).
    expected = {k for keys in shipped.values() for k in keys}
    scored = {m.model for m in a.models}
    assert scored == expected, f"scored {scored} != shipped {expected}"
    # No shipped model is left uncovered.
    assert not a.uncovered, f"uncovered shipped models: {a.uncovered}"


def test_every_model_has_all_eight_factors():
    a = credibility.assess()
    assert a.models
    for m in a.models:
        assert set(m.factors) == set(credibility.FACTOR_KEYS)
        for f in m.factors.values():
            assert 0 <= f.level <= credibility.MAX_LEVEL
            assert f.rationale.strip(), f"{m.model}/{f.key} has empty rationale"


def test_min_level_is_governing_score():
    a = credibility.assess()
    for m in a.models:
        assert m.min_level == min(f.level for f in m.factors.values())
        assert m.min_level <= m.mean_level


# --------------------------------------------------------------------------------------
# Factor rules on a KNOWN evidence set (synthetic matrix rows)
# --------------------------------------------------------------------------------------


def _row(benchmark="—", golden="—", analytic="—", model="`x` (`k`)", claim="c"):
    return vnv.MatrixRow(
        family="Guidance",
        model=model,
        claim=claim,
        benchmark=benchmark,
        golden=golden,
        analytic=analytic,
    )


def test_verification_levels_from_benchmark_count():
    # No benchmark -> 0.
    assert credibility._score_verification([_row()]).level == 0
    # One suite file -> 3.
    one = credibility._score_verification([_row(benchmark="`a_test::Suite.Case`")])
    assert one.level == 3
    # Two distinct suite files -> 4.
    two = credibility._score_verification(
        [_row(benchmark="`a_test::Suite.Case`, `b_test::Other.Case`")]
    )
    assert two.level == 4


def test_validation_levels_from_analytic_and_golden():
    # Analytic + golden -> 4 (corroborated).
    both = credibility._score_validation(
        [_row(analytic="`validate.py::check`", golden="`homing_3dof`")]
    )
    assert both.level == 4
    # Analytic only -> 3.
    ana = credibility._score_validation([_row(analytic="`validate.py::check`")])
    assert ana.level == 3
    # Golden only -> 2.
    gold = credibility._score_validation([_row(golden="`homing_3dof`")])
    assert gold.level == 2
    # Neither -> 1.
    assert credibility._score_validation([_row()]).level == 1


def test_input_pedigree_levels_from_docs():
    full = credibility.ModelDoc(True, True, True, True)
    partial = credibility.ModelDoc(True, False, False, False)  # refs only
    bare = credibility.ModelDoc(False, False, False, False)
    assert credibility._score_input_pedigree(full).level == 3
    assert credibility._score_input_pedigree(partial).level == 2
    assert credibility._score_input_pedigree(bare).level == 1
    assert credibility._score_input_pedigree(None).level == 0


def test_results_uncertainty_mc_beats_single_beats_none():
    mc = credibility._score_results_uncertainty([_row(golden="`montecarlo`")], is_mc=True)
    single = credibility._score_results_uncertainty([_row(golden="`homing_3dof`")], is_mc=False)
    none = credibility._score_results_uncertainty([_row()], is_mc=False)
    assert mc.level == 4
    assert single.level == 3
    assert none.level == 1


def test_results_robustness_limits_and_cases():
    doc_limits = credibility.ModelDoc(False, False, True, False)
    doc_nolimits = credibility.ModelDoc(False, False, False, False)
    two_cases = "`a_test::S.A`, `a_test::S.B`"
    one_case = "`a_test::S.A`"
    assert credibility._score_results_robustness(doc_limits, [_row(benchmark=two_cases)]).level == 3
    assert (
        credibility._score_results_robustness(doc_nolimits, [_row(benchmark=two_cases)]).level == 2
    )
    assert credibility._score_results_robustness(doc_limits, [_row(benchmark=one_case)]).level == 2
    assert (
        credibility._score_results_robustness(doc_nolimits, [_row(benchmark=one_case)]).level == 1
    )


def test_mc_models_score_uncertainty_4():
    """Models exercised by the `montecarlo` golden get the full UQ-uncertainty level."""
    a = credibility.assess()
    mc_models = {m.model for m in a.models if m.factors["results_uncertainty"].level == 4}
    # apn + weave reference the montecarlo golden in the shipped matrix.
    assert {"apn", "weave"} <= mc_models


def test_project_factors_uniform_across_models():
    """The three supporting-evidence factors are organizational -> identical everywhere."""
    a = credibility.assess()
    for key in ("use_history", "ms_management", "people_qualification"):
        levels = {m.factors[key].level for m in a.models}
        assert len(levels) == 1, f"{key} varies across models: {levels}"


def test_assessment_is_deterministic():
    a = credibility.assess()
    b = credibility.assess()
    sig_a = [
        (m.model, tuple(m.factors[k].level for k in credibility.FACTOR_KEYS)) for m in a.models
    ]
    sig_b = [
        (m.model, tuple(m.factors[k].level for k in credibility.FACTOR_KEYS)) for m in b.models
    ]
    assert sig_a == sig_b


# --------------------------------------------------------------------------------------
# MODELS.md parsing
# --------------------------------------------------------------------------------------


def test_model_docs_parse_known_pages():
    docs = credibility.parse_model_docs()
    # pronav's page carries assumptions, governing eq, validity limits and references.
    p = docs["pronav"]
    assert p.has_assumptions and p.has_references
    assert p.has_validity_limits and p.has_governing_eq


# --------------------------------------------------------------------------------------
# UQ trajectory dispersion band (issue #49)
# --------------------------------------------------------------------------------------


def test_trajectory_band_shapes_and_ordering():
    t = np.linspace(0, 10, 50)
    rng = np.random.default_rng(0)
    ens = np.vstack([rng.normal(t, 1.0) for _ in range(30)])
    band = uq.trajectory_band(t, ens, level=0.90)
    assert band.t_s.shape == band.mean.shape == band.std.shape == (50,)
    assert band.n == 30
    # The envelope brackets the median everywhere.
    assert np.all(band.low <= band.median + 1e-9)
    assert np.all(band.median <= band.high + 1e-9)


def test_trajectory_band_deterministic():
    t = np.linspace(0, 5, 20)
    ens = np.outer(np.arange(1, 11), np.ones(20)) + np.linspace(0, 1, 20)
    a = uq.trajectory_band(t, ens, level=0.8)
    b = uq.trajectory_band(t, ens, level=0.8)
    assert np.array_equal(a.mean, b.mean)
    assert np.array_equal(a.low, b.low)
    assert np.array_equal(a.high, b.high)
    assert a.peak_std == b.peak_std


def test_trajectory_band_rejects_bad_shapes():
    with pytest.raises(ValueError):
        uq.trajectory_band([0, 1, 2], np.zeros((3, 2)))  # width mismatch
    with pytest.raises(ValueError):
        uq.trajectory_band([0, 1], np.zeros((0, 2)))  # empty ensemble


def test_resample_holds_endpoints():
    out = uq.resample_to_grid([0, 1, 2], [10, 20, 30], [-1, 0.5, 3])
    # Out-of-range left holds first value, right holds last; interior interpolates.
    assert out[0] == 10.0
    assert out[1] == pytest.approx(15.0)
    assert out[2] == 30.0


def test_ensemble_band_resamples_varied_length_cases():
    # Two cases of different length/end time -> shared grid band.
    cases = [
        (np.linspace(0, 10, 11), np.linspace(100, 0, 11)),
        (np.linspace(0, 12, 25), np.linspace(100, 5, 25)),
    ]
    band = uq.ensemble_band(cases, level=0.9, n_points=50)
    assert band.t_s.size == 50
    assert band.n == 2
    # Grid is capped at the shorter case's end time (10 s).
    assert band.t_s[-1] == pytest.approx(10.0)
    assert np.isfinite(band.peak_std)


def test_ensemble_band_deterministic():
    cases = [
        (np.linspace(0, 8, 9), np.cos(np.linspace(0, 8, 9))),
        (np.linspace(0, 9, 12), np.sin(np.linspace(0, 9, 12))),
    ]
    a = uq.ensemble_band(cases, n_points=30)
    b = uq.ensemble_band(cases, n_points=30)
    assert np.array_equal(a.mean, b.mean)
    assert np.array_equal(a.high, b.high)


# --------------------------------------------------------------------------------------
# V&V report generator
# --------------------------------------------------------------------------------------


def test_report_builds_and_covers_all_models():
    from gncpost import vnv_report

    text = vnv_report.build_report(now=None)
    assert text.startswith("# gnc-sim V&V & Credibility Report")
    # Every shipped model name appears in the report (claim->evidence section).
    a = credibility.assess()
    for m in a.models:
        assert f"`{m.model}`" in text, f"model {m.model} missing from report"
    # The eight factor labels all appear.
    for _k, label, _cat in credibility.FACTORS:
        assert label in text
    # UQ + golden + matrix sections present.
    assert "Uncertainty quantification" in text
    assert "golden baseline" in text
    assert "consistency" in text.lower()


def test_report_is_deterministic_without_timestamp():
    from gncpost import vnv_report

    a = vnv_report.build_report(now=None)
    b = vnv_report.build_report(now=None)
    assert a == b


def test_report_reports_consistent_matrix():
    """The generated report should reflect a consistent V&V matrix (the repo invariant)."""
    from gncpost import vnv_report

    text = vnv_report.build_report(now=None)
    assert "CONSISTENT" in text
    assert "DRIFT" not in text
