"""pytest fixtures/config for the gnc-sim Python SDK tests.

Puts the ``bindings/`` directory (which holds the importable ``gncsim`` package + the compiled
``_gncsim`` extension dropped there by the CMake build) on ``sys.path`` so ``import gncsim`` resolves
without an install step.
"""

from __future__ import annotations

import sys
from pathlib import Path

_BINDINGS_DIR = Path(__file__).resolve().parent
if str(_BINDINGS_DIR) not in sys.path:
    sys.path.insert(0, str(_BINDINGS_DIR))
