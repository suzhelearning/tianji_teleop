"""Checkout and dataset locations, re-exported from the shared runtime contract.

Kept as a module (not deleted) because the offline tools and the shell helpers
import these three names directly; they are now thin aliases over
:mod:`tianji_runtime.resources`, which is the single path authority.
"""

from __future__ import annotations

from tianji_runtime.resources import (
    compressed_dir as _compressed_dir,
    dataset_dir as _dataset_dir,
    workspace as _workspace,
)

ROOT = _workspace()
DATASET = _dataset_dir()
COMPRESSED_DATASET = _compressed_dir()

__all__ = ["ROOT", "DATASET", "COMPRESSED_DATASET"]
