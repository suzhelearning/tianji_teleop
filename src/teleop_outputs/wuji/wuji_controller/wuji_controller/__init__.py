"""Wuji Hand2 adapter.

Owns hand device I/O, the pinned Wuji C SDK ABI and its serial discovery tool.
Depends on ``tianji_runtime`` for the shared device contract; it is never
imported by it, so the two hardware adapters stay siblings rather than a cycle.
"""

from .hardware import Hand2Device

__all__ = ["Hand2Device"]
