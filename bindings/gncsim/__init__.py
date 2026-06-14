"""gnc-sim Python SDK.

Run the deterministic C++ guided-interceptor core directly from Python, with results returned as
numpy arrays — no CSV/JSON round-trip (see ``docs/TARGET_ARCHITECTURE.md`` §4). This package is a thin
wrapper over the compiled ``_gncsim`` pybind11 extension; the heavy lifting (and ALL numerics) lives
in the pure C++ core, the exact same engine the native CLI and the WebAssembly web build run. Results
from :func:`run` match the CLI/golden values bit-for-bit for the same config + seed.

Configs may be passed as a ``dict`` (mirroring ``configs/*.json``) or as a JSON string; both go through
the same ``loadConfigFromString`` parser the CLI and WASM entry use, so there is one schema, not a fork.

Example::

    import gncsim
    res = gncsim.run({"scenario": "homing", "model": "3dof", "seed": 1})
    print(res["miss_distance"], res["series"]["range"].min())

    batch = gncsim.monte_carlo(cfg, n=1000, workers=8)
    print(batch["p_kill"])
"""

from __future__ import annotations

import json
from typing import Any

from . import _gncsim  # type: ignore[attr-defined]

__all__ = ["run", "monte_carlo"]

ConfigType = dict[str, Any] | str


def _to_json(cfg: ConfigType) -> str:
    """Normalize a dict-or-JSON-string config to a JSON document string for the core parser."""
    if isinstance(cfg, str):
        return cfg
    if isinstance(cfg, dict):
        return json.dumps(cfg)
    raise TypeError(f"cfg must be a dict or JSON string, got {type(cfg).__name__}")


def run(cfg: ConfigType) -> dict[str, Any]:
    """Run one engagement to completion.

    Args:
        cfg: A config ``dict`` (same shape as ``configs/*.json``) or a JSON string. Missing keys fall
            back to the core's defaults, exactly as for the CLI/WASM.

    Returns:
        A dict of scalar metadata (``scenario``, ``model``, ``seed``, ``miss_distance``,
        ``intercept``, ``intercept_time``, ``launch_time``, ``origin``, ...) plus ``series`` — a dict
        mapping channel name to a 1-D ``numpy.ndarray`` (``float64``). The channel names match the
        columnar JSON the web app consumes.
    """
    return _gncsim.run(_to_json(cfg))  # type: ignore[no-any-return]


def monte_carlo(cfg: ConfigType, n: int = 0, workers: int = 1) -> dict[str, Any]:
    """Run a dispersed Monte Carlo batch.

    Initial-condition dispersion lives in the C++ core (``core/src/scenario/MonteCarlo.cpp``), driven
    by the ``monte_carlo`` sigma block in ``cfg`` and deterministic given the seed.

    Args:
        cfg: Config dict or JSON string. Its ``monte_carlo`` block supplies the dispersion sigmas.
        n: Number of cases; when > 0 it overrides ``cfg.monte_carlo.num_cases``.
        workers: Reserved for a future thread pool (issue #43). The core batch is currently serial and
            deterministic, so this is a no-op today; kept for API stability.

    Returns:
        A dict with summary scalars (``num_cases``, ``intercepts``, ``p_kill``) and columnar numpy
        arrays, one entry per case: ``index``, ``seed``, ``miss_distance``, ``intercept_time``,
        ``intercept``.
    """
    return _gncsim.monte_carlo(_to_json(cfg), n, workers)  # type: ignore[no-any-return]
