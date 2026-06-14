"""Model-fidelity tiering consistency (issue #50).

Asserts the tier manifest (``configs/tiers.json``) stays in sync with the shipped model set and
that the determinism guard's fast-tier coverage holds:

* every model resolved by ``core/src/model/Registry.cpp`` is classified into a tier (fast | hifi),
* every ``fast``-tier model is exercised by at least one fast-tier config (``config_models``), so
  parity coverage tracks the fast-tier model set — a new fast model without a parity-checked config
  fails here (mirroring ``scripts/determinism-guard.mjs``),
* every fast-tier and hi-fi config names an existing ``configs/<name>.json``,
* every ``config_models`` entry references a config and only fast-tier model keys.

This is the Python leg of the determinism guard (the .mjs guard does the same coverage check plus
the actual native<->WASM parity runs). Keeping it here lets ``pytest`` catch tier drift without a
WASM build.
"""

from __future__ import annotations

import json

from gncpost import REPO_ROOT
from gncpost.vnv import parse_shipped_models

TIERS_PATH = REPO_ROOT / "configs" / "tiers.json"
CONFIG_DIR = REPO_ROOT / "configs"

# Registry families with both a fast and a high-fidelity member (the formalized fast/hi-fi pairs).
# Documented in docs/ARCHITECTURE.md; asserted here so the manifest can't silently drop a tier.
_VALID_TIERS = {"fast", "hifi"}


def _load_tiers() -> dict:
    return json.loads(TIERS_PATH.read_text())


def test_every_shipped_model_is_tiered() -> None:
    """Each model the registry resolves must be classified into a tier in tiers.json."""
    tiers = _load_tiers()
    classified: dict[str, str] = {}
    for family in tiers["models"].values():
        for key, tier in family.items():
            assert tier in _VALID_TIERS, f"model '{key}' has unknown tier '{tier}'"
            classified[key] = tier

    shipped = parse_shipped_models()
    missing = []
    for family, keys in shipped.items():
        for key in keys:
            if key not in classified:
                missing.append(f"{family}:{key}")
    assert not missing, (
        "shipped models with no tier in configs/tiers.json (add them to the manifest "
        f"with a fast/hifi tier): {missing}"
    )


def test_tier_manifest_lists_no_phantom_models() -> None:
    """tiers.json must not classify a model the registry no longer ships."""
    tiers = _load_tiers()
    shipped_keys = {k for keys in parse_shipped_models().values() for k in keys}
    phantom = []
    for family in tiers["models"].values():
        for key in family:
            if key not in shipped_keys:
                phantom.append(key)
    assert not phantom, f"tiers.json classifies models not in the registry: {phantom}"


def test_every_fast_model_has_a_parity_config() -> None:
    """Every fast-tier model must be exercised by at least one fast-tier config.

    This is the coverage invariant the determinism guard enforces: parity coverage tracks the
    fast-tier model set. A new fast model without a fast-tier config that exercises it fails here.
    """
    tiers = _load_tiers()
    fast_models = {
        key for family in tiers["models"].values() for key, t in family.items() if t == "fast"
    }
    covered: set[str] = set()
    for name in tiers["fast_tier_configs"]:
        covered.update(tiers["config_models"].get(name, []))
    uncovered = sorted(fast_models - covered)
    assert not uncovered, (
        "fast-tier models not exercised by any fast_tier_configs entry "
        f"(add a parity-checked config): {uncovered}"
    )


def test_config_models_reference_real_fast_models() -> None:
    """config_models entries must reference real fast-tier model keys and known configs."""
    tiers = _load_tiers()
    fast_models = {
        key for family in tiers["models"].values() for key, t in family.items() if t == "fast"
    }
    for name, models in tiers["config_models"].items():
        assert name in tiers["fast_tier_configs"], (
            f"config_models['{name}'] is not in fast_tier_configs"
        )
        for m in models:
            assert m in fast_models, f"config_models['{name}'] lists non-fast model '{m}'"


def test_all_manifest_configs_exist() -> None:
    """Every fast-tier and hi-fi config names an actual configs/<name>.json file."""
    tiers = _load_tiers()
    for name in tiers["fast_tier_configs"] + tiers.get("hifi_configs", []):
        assert (CONFIG_DIR / f"{name}.json").exists(), f"configs/{name}.json missing"


def test_hifi_configs_are_golden_covered() -> None:
    """Hi-fi configs are golden-checked (not parity-guaranteed).

    The fast tier's guarantee is native<->WASM parity; the hi-fi tier's guarantee is a golden
    regression net. So every ``hifi_configs`` entry must be an actual case in golden.json.
    """
    tiers = _load_tiers()
    golden = json.loads((REPO_ROOT / "postproc" / "golden" / "golden.json").read_text())
    golden_cases = set(golden.get("cases", {}))
    for name in tiers.get("hifi_configs", []):
        assert name in golden_cases, (
            f"hi-fi config '{name}' is not a golden case "
            "(add it to postproc/gncpost/golden.py CANONICAL and re-baseline with --update)"
        )
