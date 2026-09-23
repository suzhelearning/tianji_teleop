"""Locate the workspace root from a native test script.

These scripts are driven by ctest from the build tree, so neither the working
directory nor a fixed parent count identifies the checkout. The marker file is
the search key, and `TIANJI_WORKSPACE` (injected by the environment) wins.
"""

from __future__ import annotations

import os
from pathlib import Path

_MARKER = "pixi.toml"


def workspace() -> Path:
    declared = os.environ.get("TIANJI_WORKSPACE")
    if declared:
        root = Path(declared).expanduser()
        if (root / _MARKER).is_file():
            return root
    for candidate in Path(__file__).resolve().parents:
        if (candidate / _MARKER).is_file():
            return candidate
    raise RuntimeError(
        "cannot locate the workspace root; run this through ctest (`pixi run test-native`) "
        "or set TIANJI_WORKSPACE"
    )


def native_binary(name: str) -> Path:
    """The built controller executable, wherever its build tree lives."""
    candidate = workspace() / "install" / "control" / "bin" / name
    if candidate.is_file():
        return candidate
    raise RuntimeError(f"{name} is not built; run 'pixi run build-workspace'")
