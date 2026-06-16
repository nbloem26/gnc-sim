"""Run database: a queryable SQLite store of Monte-Carlo campaigns (issue #43).

Every large campaign — whether driven by the native CLI (``summary.csv``) or the Python SDK
(``gncsim.monte_carlo``) — can be recorded here with full **provenance** so any headline number is
reproducible from the database alone:

* **campaign metadata** — scenario, model, master seed, case count, worker count, a wall-clock time,
  and an ISO-8601 timestamp;
* **provenance** — a stable ``config_hash`` (SHA-256 of the canonicalized config), the master
  ``seed``, and a ``git_version`` string, so a row ties back to an exact config + code revision;
* **per-case metrics** — one row per Monte-Carlo case: ``case_index``, ``seed``, ``miss_distance_m``,
  ``intercept_time_s``, ``intercept``.

Storage is **stdlib ``sqlite3`` only** — no new dependency. (Parquet/pyarrow would be used here if it
were already vendored, but it is not, so SQLite is the single backend.) Analysis and the golden
harness can read a campaign back with :meth:`RunDB.load_summary` and get the same DataFrame they would
from a ``summary.csv``.

Typical use::

    from gncpost.rundb import RunDB

    db = RunDB("runs/campaigns.db")
    cid = db.record_campaign(
        config=cfg_dict, seed=12345, scenario="homing", model="3dof",
        cases=mc_rows, wall_time_s=3.4, num_workers=8,
    )
    summary = db.load_summary(cid)          # -> pandas DataFrame, same cols as summary.csv
    stats = compute_stats(summary)          # reuse the existing CEP analysis
"""

from __future__ import annotations

import hashlib
import json
import sqlite3
import subprocess
import time
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pandas as pd

__all__ = [
    "CaseRow",
    "CampaignMeta",
    "RunDB",
    "config_hash",
    "git_version",
    "cases_from_summary",
]

# Bump only if the table shape changes (a migration would key off this).
SCHEMA_VERSION = 1


@dataclass(frozen=True)
class CaseRow:
    """One Monte-Carlo case result (units in names per AGENTS.md)."""

    case_index: int
    seed: int
    miss_distance_m: float
    intercept_time_s: float
    intercept: bool


@dataclass(frozen=True)
class CampaignMeta:
    """Campaign-level metadata + provenance read back from the DB."""

    campaign_id: int
    scenario: str
    model: str
    seed: int
    num_cases: int
    num_workers: int
    config_hash: str
    git_version: str
    wall_time_s: float
    created_utc: str
    intercepts: int
    p_kill: float


# --------------------------------------------------------------------------------------
# Provenance helpers
# --------------------------------------------------------------------------------------


def config_hash(config: Mapping[str, Any] | str) -> str:
    """Stable SHA-256 of a config.

    A ``dict`` is canonicalized (sorted keys, compact separators) before hashing so two equal configs
    hash identically regardless of key order; a JSON ``str`` is re-parsed and canonicalized the same
    way, so the CLI's config text and the SDK's dict produce the same hash for the same content.
    """
    if isinstance(config, str):
        obj = json.loads(config)
    else:
        obj = config
    canonical = json.dumps(obj, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def git_version(repo_root: Path | None = None) -> str:
    """Best-effort ``git describe`` (or short SHA) of the working tree; ``"unknown"`` if unavailable.

    Appended with ``-dirty`` when the tree has uncommitted changes, so a recorded campaign cannot
    silently claim a clean revision it did not run under.
    """
    cwd = str(repo_root) if repo_root is not None else None
    try:
        sha = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=cwd,
            capture_output=True,
            text=True,
            timeout=5,
        )
        if sha.returncode != 0:
            return "unknown"
        version = sha.stdout.strip()
        dirty = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=cwd,
            capture_output=True,
            text=True,
            timeout=5,
        )
        if dirty.returncode == 0 and dirty.stdout.strip():
            version += "-dirty"
        return version
    except (OSError, subprocess.SubprocessError):
        return "unknown"


def cases_from_summary(summary: pd.DataFrame) -> list[CaseRow]:
    """Convert a ``summary.csv`` DataFrame (case,seed,miss_distance,intercept_time,intercept) to rows."""
    rows: list[CaseRow] = []
    inter = summary.get("intercept")
    for i, (_, r) in enumerate(summary.iterrows()):
        flag = False
        if inter is not None:
            v = r["intercept"]
            flag = str(v).strip().lower() in ("1", "true", "yes") if isinstance(v, str) else bool(v)
        rows.append(
            CaseRow(
                case_index=int(r.get("case", i)),
                seed=int(r["seed"]),
                miss_distance_m=float(r["miss_distance"]),
                intercept_time_s=float(r["intercept_time"]),
                intercept=flag,
            )
        )
    return rows


# --------------------------------------------------------------------------------------
# The database
# --------------------------------------------------------------------------------------

_SCHEMA = """
CREATE TABLE IF NOT EXISTS campaigns (
    campaign_id  INTEGER PRIMARY KEY AUTOINCREMENT,
    scenario     TEXT    NOT NULL,
    model        TEXT    NOT NULL,
    seed         TEXT    NOT NULL,   -- uint64 master seed stored as text (exceeds signed-64 range)
    num_cases    INTEGER NOT NULL,
    num_workers  INTEGER NOT NULL,
    config_hash  TEXT    NOT NULL,
    git_version  TEXT    NOT NULL,
    config_json  TEXT    NOT NULL,
    wall_time_s  REAL    NOT NULL,
    created_utc  TEXT    NOT NULL,
    schema_version INTEGER NOT NULL
);
CREATE TABLE IF NOT EXISTS cases (
    campaign_id      INTEGER NOT NULL REFERENCES campaigns(campaign_id) ON DELETE CASCADE,
    case_index       INTEGER NOT NULL,
    seed             TEXT    NOT NULL,  -- uint64 per-case seed stored as text (full precision)
    miss_distance_m  REAL    NOT NULL,
    intercept_time_s REAL    NOT NULL,
    intercept        INTEGER NOT NULL,
    PRIMARY KEY (campaign_id, case_index)
);
CREATE INDEX IF NOT EXISTS idx_cases_campaign ON cases(campaign_id);
CREATE INDEX IF NOT EXISTS idx_campaigns_hash ON campaigns(config_hash);
"""


class RunDB:
    """A SQLite-backed campaign store. Pass ``":memory:"`` for an ephemeral DB (tests)."""

    def __init__(self, path: str | Path = ":memory:") -> None:
        self.path = str(path)
        if self.path != ":memory:":
            Path(self.path).parent.mkdir(parents=True, exist_ok=True)
        # check_same_thread=False is safe here: we never share a connection across threads, but it
        # lets a caller hand the DB to a worker that wrote the campaign on another thread.
        self._conn = sqlite3.connect(self.path, check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        self._conn.execute("PRAGMA foreign_keys = ON")
        self._conn.executescript(_SCHEMA)
        self._conn.commit()

    def close(self) -> None:
        self._conn.close()

    def __enter__(self) -> RunDB:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def record_campaign(
        self,
        *,
        config: Mapping[str, Any] | str,
        seed: int,
        scenario: str,
        model: str,
        cases: Iterable[CaseRow],
        wall_time_s: float = 0.0,
        num_workers: int = 1,
        git: str | None = None,
        repo_root: Path | None = None,
    ) -> int:
        """Insert a campaign + its per-case rows; returns the new ``campaign_id``.

        ``config`` supplies both the stored canonical config text and the provenance ``config_hash``.
        ``git`` overrides the auto-detected ``git_version`` (handy for tests / deterministic records).
        """
        case_list = list(cases)
        chash = config_hash(config)
        cfg_text = config if isinstance(config, str) else json.dumps(config, sort_keys=True)
        gv = git if git is not None else git_version(repo_root)
        # ISO-8601 UTC timestamp (no datetime.UTC alias dependency -> works on 3.10 and 3.11).
        created = time.strftime("%Y-%m-%dT%H:%M:%S+00:00", time.gmtime())

        cur = self._conn.cursor()
        cur.execute(
            "INSERT INTO campaigns (scenario, model, seed, num_cases, num_workers, config_hash, "
            "git_version, config_json, wall_time_s, created_utc, schema_version) "
            "VALUES (?,?,?,?,?,?,?,?,?,?,?)",
            (
                scenario,
                model,
                str(int(seed)),  # uint64 -> text (preserves values above 2**63)
                len(case_list),
                int(num_workers),
                chash,
                gv,
                cfg_text,
                float(wall_time_s),
                created,
                SCHEMA_VERSION,
            ),
        )
        campaign_id = int(cur.lastrowid or 0)
        cur.executemany(
            "INSERT INTO cases (campaign_id, case_index, seed, miss_distance_m, intercept_time_s, "
            "intercept) VALUES (?,?,?,?,?,?)",
            [
                (
                    campaign_id,
                    c.case_index,
                    str(int(c.seed)),  # uint64 -> text (full precision in SQLite)
                    float(c.miss_distance_m),
                    float(c.intercept_time_s),
                    1 if c.intercept else 0,
                )
                for c in case_list
            ],
        )
        self._conn.commit()
        return campaign_id

    def list_campaigns(self) -> list[CampaignMeta]:
        """All campaigns, newest first, as metadata records (with intercept count + P(kill))."""
        rows = self._conn.execute("SELECT * FROM campaigns ORDER BY campaign_id DESC").fetchall()
        return [self._meta_from_row(r) for r in rows]

    def get_campaign(self, campaign_id: int) -> CampaignMeta:
        """Metadata for one campaign (raises ``KeyError`` if absent)."""
        row = self._conn.execute(
            "SELECT * FROM campaigns WHERE campaign_id = ?", (campaign_id,)
        ).fetchone()
        if row is None:
            raise KeyError(f"no campaign with id {campaign_id}")
        return self._meta_from_row(row)

    def load_summary(self, campaign_id: int) -> pd.DataFrame:
        """Read a campaign's cases back as a ``summary.csv``-shaped DataFrame.

        Columns: ``case, seed, miss_distance, intercept_time, intercept`` — identical to what
        ``loaders.load_summary`` returns, so ``montecarlo.compute_stats`` consumes it unchanged.
        """
        df = pd.read_sql_query(
            'SELECT case_index AS "case", seed, miss_distance_m AS miss_distance, '
            "intercept_time_s AS intercept_time, intercept "
            "FROM cases WHERE campaign_id = ? ORDER BY case_index",
            self._conn,
            params=(campaign_id,),
        )
        # seed is stored as TEXT (uint64 > signed-64 range); present it as Python ints, matching the
        # values originally recorded (object dtype keeps full precision, like the CLI summary.csv).
        if "seed" in df.columns:
            df["seed"] = df["seed"].map(int)
        return df

    def load_config(self, campaign_id: int) -> dict[str, Any]:
        """The stored canonical config dict for a campaign (proves the run is reproducible)."""
        row = self._conn.execute(
            "SELECT config_json FROM campaigns WHERE campaign_id = ?", (campaign_id,)
        ).fetchone()
        if row is None:
            raise KeyError(f"no campaign with id {campaign_id}")
        return json.loads(row["config_json"])

    def find_by_config_hash(self, chash: str) -> list[CampaignMeta]:
        """Every campaign run against a given ``config_hash`` (provenance lookup)."""
        rows = self._conn.execute(
            "SELECT * FROM campaigns WHERE config_hash = ? ORDER BY campaign_id DESC",
            (chash,),
        ).fetchall()
        return [self._meta_from_row(r) for r in rows]

    # -- internals ----------------------------------------------------------------------

    def _meta_from_row(self, row: sqlite3.Row) -> CampaignMeta:
        cid = int(row["campaign_id"])
        agg = self._conn.execute(
            "SELECT COUNT(*) AS n, COALESCE(SUM(intercept), 0) AS hits FROM cases "
            "WHERE campaign_id = ?",
            (cid,),
        ).fetchone()
        n = int(agg["n"])
        hits = int(agg["hits"])
        return CampaignMeta(
            campaign_id=cid,
            scenario=str(row["scenario"]),
            model=str(row["model"]),
            seed=int(row["seed"]),
            num_cases=int(row["num_cases"]),
            num_workers=int(row["num_workers"]),
            config_hash=str(row["config_hash"]),
            git_version=str(row["git_version"]),
            wall_time_s=float(row["wall_time_s"]),
            created_utc=str(row["created_utc"]),
            intercepts=hits,
            p_kill=(hits / n) if n else 0.0,
        )


def record_summary_dir(
    db: RunDB,
    batch_dir: str | Path,
    *,
    config: Mapping[str, Any] | str,
    seed: int,
    scenario: str,
    model: str,
    num_workers: int = 1,
    wall_time_s: float = 0.0,
) -> int:
    """Convenience: read a CLI batch's ``summary.csv`` from ``batch_dir`` and record it as a campaign."""
    from .loaders import load_summary

    summary = load_summary(batch_dir)
    return db.record_campaign(
        config=config,
        seed=seed,
        scenario=scenario,
        model=model,
        cases=cases_from_summary(summary),
        num_workers=num_workers,
        wall_time_s=wall_time_s,
    )


def _cli_rows_to_cases(
    index: Sequence[int],
    seed: Sequence[int],
    miss: Sequence[float],
    tca: Sequence[float],
    intercept: Sequence[bool],
) -> list[CaseRow]:
    """Build CaseRows from the columnar arrays the SDK's ``monte_carlo`` returns."""
    return [
        CaseRow(int(index[i]), int(seed[i]), float(miss[i]), float(tca[i]), bool(intercept[i]))
        for i in range(len(index))
    ]
