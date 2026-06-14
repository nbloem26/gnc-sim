"""Run-database round-trip tests (issue #43).

A campaign written to the SQLite run DB must read back with identical per-case metrics, intact
provenance (config hash + seed + git version), and a ``summary.csv``-shaped DataFrame that the
existing CEP analysis consumes unchanged.
"""

from __future__ import annotations

import numpy as np
import pandas as pd
import pytest
from gncpost.montecarlo import compute_stats
from gncpost.rundb import (
    CaseRow,
    RunDB,
    cases_from_summary,
    config_hash,
    git_version,
)


def _rows(n: int) -> list[CaseRow]:
    rng = np.random.default_rng(0)
    miss = rng.uniform(0.0, 20.0, n)
    return [
        CaseRow(
            case_index=i,
            seed=1000 + i,
            miss_distance_m=float(miss[i]),
            intercept_time_s=4.0 + 0.01 * i,
            intercept=bool(miss[i] < 3.0),
        )
        for i in range(n)
    ]


def test_config_hash_is_order_independent() -> None:
    a = {"scenario": "homing", "seed": 1, "monte_carlo": {"num_cases": 10}}
    b = {"monte_carlo": {"num_cases": 10}, "seed": 1, "scenario": "homing"}
    assert config_hash(a) == config_hash(b)
    # dict and its JSON-string form hash identically.
    import json

    assert config_hash(a) == config_hash(json.dumps(b))
    # A real change in content changes the hash.
    c = dict(a, seed=2)
    assert config_hash(a) != config_hash(c)


def test_record_and_load_round_trip() -> None:
    cfg = {"scenario": "homing", "model": "3dof", "seed": 42, "monte_carlo": {"num_cases": 25}}
    rows = _rows(25)
    with RunDB(":memory:") as db:
        cid = db.record_campaign(
            config=cfg,
            seed=42,
            scenario="homing",
            model="3dof",
            cases=rows,
            num_workers=8,
            wall_time_s=1.23,
            git="abc1234",
        )

        # Per-case metrics survive the round trip exactly.
        summary = db.load_summary(cid)
        assert list(summary["case"]) == [r.case_index for r in rows]
        assert np.array_equal(
            summary["miss_distance"].to_numpy(),
            np.array([r.miss_distance_m for r in rows]),
        )
        assert np.array_equal(
            summary["intercept_time"].to_numpy(),
            np.array([r.intercept_time_s for r in rows]),
        )
        assert list(summary["seed"]) == [r.seed for r in rows]

        # Provenance is intact.
        meta = db.get_campaign(cid)
        assert meta.scenario == "homing"
        assert meta.model == "3dof"
        assert meta.seed == 42
        assert meta.num_cases == 25
        assert meta.num_workers == 8
        assert meta.wall_time_s == pytest.approx(1.23)
        assert meta.git_version == "abc1234"
        assert meta.config_hash == config_hash(cfg)
        expected_hits = sum(r.intercept for r in rows)
        assert meta.intercepts == expected_hits
        assert meta.p_kill == pytest.approx(expected_hits / 25)

        # The stored config round-trips, so the campaign is reproducible from the DB alone.
        assert db.load_config(cid) == cfg


def test_loaded_summary_feeds_compute_stats() -> None:
    """The DB's summary DataFrame is consumable by the existing CEP analysis with no adaptation."""
    rows = _rows(40)
    with RunDB(":memory:") as db:
        cid = db.record_campaign(
            config={"scenario": "homing"},
            seed=7,
            scenario="homing",
            model="3dof",
            cases=rows,
        )
        summary = db.load_summary(cid)

    stats = compute_stats(summary)
    miss = np.array([r.miss_distance_m for r in rows])
    assert stats.n == 40
    assert stats.cep == pytest.approx(float(np.median(miss)))
    assert stats.mean == pytest.approx(float(np.mean(miss)))
    assert stats.intercept_rate == pytest.approx(np.mean([r.intercept for r in rows]))


def test_cases_from_summary_matches_records() -> None:
    """A summary.csv-shaped DataFrame converts back to the same CaseRows that produced it."""
    rows = _rows(12)
    df = pd.DataFrame(
        {
            "case": [r.case_index for r in rows],
            "seed": [r.seed for r in rows],
            "miss_distance": [r.miss_distance_m for r in rows],
            "intercept_time": [r.intercept_time_s for r in rows],
            "intercept": [1 if r.intercept else 0 for r in rows],
        }
    )
    back = cases_from_summary(df)
    assert back == rows


def test_find_by_config_hash_and_list() -> None:
    cfg = {"scenario": "homing", "seed": 1}
    with RunDB(":memory:") as db:
        c1 = db.record_campaign(config=cfg, seed=1, scenario="homing", model="3dof", cases=_rows(5))
        c2 = db.record_campaign(config=cfg, seed=1, scenario="homing", model="3dof", cases=_rows(6))
        # Different config -> different hash bucket.
        db.record_campaign(
            config={"scenario": "ballistic"},
            seed=2,
            scenario="ballistic",
            model="3dof",
            cases=_rows(4),
        )

        found = db.find_by_config_hash(config_hash(cfg))
        assert {m.campaign_id for m in found} == {c1, c2}
        assert len(db.list_campaigns()) == 3


def test_git_version_returns_string() -> None:
    # On a git checkout this yields a short SHA (optionally -dirty); off one, "unknown".
    v = git_version()
    assert isinstance(v, str) and v


def test_persists_to_disk(tmp_path) -> None:
    """A file-backed DB reopens with the same campaign (durability across processes)."""
    db_path = tmp_path / "campaigns.db"
    rows = _rows(8)
    with RunDB(db_path) as db:
        cid = db.record_campaign(
            config={"scenario": "homing"}, seed=3, scenario="homing", model="3dof", cases=rows
        )
    # Reopen.
    with RunDB(db_path) as db2:
        summary = db2.load_summary(cid)
        assert len(summary) == 8
        assert db2.get_campaign(cid).num_cases == 8
