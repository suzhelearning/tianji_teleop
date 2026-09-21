"""Measured-state assembly shared by the ROS collector runtime.

This module holds the parts of the acquisition engine that are independent of
where the samples come from: the freshness rules, the per-device token handling
and the two-side hand pairing. The ROS plumbing lives in :mod:`data_collector.runtime`.

Every timestamp produced here is the host ``time.monotonic_ns()`` at which the
complete sample became available in the collector's cache. Device stamps are
only ever used to decide whether a sample is *new*; they are never persisted.
"""

from __future__ import annotations

import math
import time

from tianji_runtime.constants import ARMS_COUNT, DEVICES, HAND_COUNT

# Defaults preserved from the validated acquisition engine.
STATE_STALE_S = 0.3
IMAGE_STALE_S = 2.0
STARTUP_TIMEOUT_S = 15.0
EPISODE_FRESH_TIMEOUT_S = 0.5


class Staleness:
    """Age/validity policy for measured feedback and images."""

    def __init__(self, state_stale_s: float = STATE_STALE_S, image_stale_s: float = IMAGE_STALE_S):
        self.state_stale_s = float(state_stale_s)
        self.state_stale_ns = int(round(self.state_stale_s * 1e9))
        self.image_stale_ns = int(round(float(image_stale_s) * 1e9))


def rejection_reason(feedback, count: int, now: int, policy: Staleness) -> str | None:
    """``None`` when the sample is healthy, fresh, finite and complete.

    The returned string is the operator-visible reason, so a rejected sample is
    always explainable rather than silently dropped.
    """
    if feedback is None:
        return "no feedback object"
    if not getattr(feedback, "healthy", False):
        return f"unhealthy ({getattr(feedback, 'detail', '') or 'no detail'})"
    qpos = getattr(feedback, "position_rad", None)
    if qpos is None or len(qpos) != count:
        return f"expected {count} joint positions"
    try:
        if not all(math.isfinite(value) for value in qpos):
            return "nonfinite joint position"
    except TypeError:
        return "nonfinite joint position"
    try:
        stamp = int(getattr(feedback, "received_monotonic_ns", 0))
    except (TypeError, ValueError):
        return "invalid feedback timestamp"
    if stamp <= 0:
        return "no feedback timestamp"
    age = now - stamp
    if age < 0:
        return "feedback timestamp is in the future"
    if age > policy.state_stale_ns:
        return f"feedback is {age / 1e9:.3f}s old"
    return None


class HandPairing:
    """Assemble the 40-dimension hand vector from two independently updated sides.

    A side only contributes when it carries a *new* source token, so refreshing
    one hand never re-publishes the other's last value. Both sides must be newer
    than the freshness limit at the moment of publication, which is what makes
    the pair a real simultaneous measurement rather than two stitched samples.
    """

    def __init__(self, policy: Staleness):
        self._policy = policy
        self._tokens: dict[str, int] = {}
        self._staged: dict[str, tuple[int, tuple[float, ...]]] = {}
        self._detail = ""

    @property
    def detail(self) -> str:
        return self._detail

    def stage(self, side: str, feedback, now: int) -> None:
        """Stage one side when it is a new, valid sample; otherwise record why not."""
        reason = rejection_reason(feedback, HAND_COUNT, now, self._policy)
        if reason is not None:
            self._detail = f"{side}: {reason}"
            return
        token = int(feedback.received_monotonic_ns)
        if self._tokens.get(side) == token:
            self._detail = ""  # a duplicate inside the window is not a stall
            return
        self._tokens[side] = token
        self._staged[side] = (token, tuple(float(value) for value in feedback.position_rad))

    def take(self, now: int) -> tuple[int, tuple[float, ...]] | None:
        """Return the combined vector and its availability stamp, once both sides are fresh.

        ``None`` until *both* hands have produced new samples inside the
        freshness window. Staged values are consumed on publication, so a
        stalled side cannot be silently reused.
        """
        if len(self._staged) != 2:
            self._detail = "waiting for a new sample from both hands"
            return None
        tokens = {side: token for side, (token, _) in self._staged.items()}
        if any(now - token > self._policy.state_stale_ns or now < token for token in tokens.values()):
            self._detail = "a hand sample is older than the freshness limit"
            self._staged.clear()
            return None
        stamp = time.monotonic_ns()
        qpos = tuple(value for side in ("left_hand", "right_hand") for value in self._staged[side][1])
        self._staged.clear()
        self._detail = ""
        return stamp, qpos

    def invalidate(self, side: str) -> None:
        """Drop one side's staged sample; used when a session identity changes."""
        self._staged.pop(side, None)
        self._tokens.pop(side, None)


def validate_devices(mapping) -> dict:
    """Require exactly the three known device names."""
    present = {name for name in DEVICES if mapping.get(name) is not None}
    missing = sorted(set(DEVICES) - present)
    if missing:
        raise ValueError(f"missing measured feedback for: {', '.join(missing)}")
    return {name: mapping[name] for name in DEVICES}
