"""Replay time pauses independently of output/wire freshness."""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class HoldToRunClock:
    """Accumulate only previously pressed intervals, optionally bounding jumps.

    Call update on every runner tick, including paused ticks. A press starts the
    next interval; release accounts for the preceding pressed interval. This is
    the original source replay-clock contract, with no wall-clock catch-up after
    a pause. Consumers continue their output freshness loop while elapsed holds.
    """

    elapsed_s: float = 0.0
    running: bool = False
    maximum_step_s: float | None = None
    _last_update_s: float | None = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.elapsed_s) or self.elapsed_s < 0:
            raise ValueError("elapsed_s must be non-negative and finite")
        if self.maximum_step_s is not None and (not math.isfinite(self.maximum_step_s) or self.maximum_step_s <= 0):
            raise ValueError("maximum_step_s must be positive and finite")

    def update(self, now_s: float, pressed: bool) -> float:
        now = float(now_s)
        if not math.isfinite(now):
            raise ValueError("now_s must be finite")
        if self._last_update_s is None:
            self._last_update_s = now
            self.running = bool(pressed)
            return self.elapsed_s
        if now < self._last_update_s:
            raise ValueError("now_s cannot move backwards")
        interval = now - self._last_update_s
        if self.running:
            if self.maximum_step_s is not None:
                interval = min(interval, self.maximum_step_s)
            self.elapsed_s += interval
        self._last_update_s = now
        self.running = bool(pressed)
        return self.elapsed_s
