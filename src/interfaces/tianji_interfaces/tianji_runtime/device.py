"""Shared hardware-feedback contracts.

``Feedback`` is the payload every adapter returns from ``read_feedback()`` and
is the only thing the executor and the collector agree on. It carries a device
clock stamp so a consumer can tell a *new* sample from a repeated read without
comparing joint values: a stationary arm legitimately reports identical
positions, so toll values are not a freshness signal.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading

# A sample older than this is treated as stalled. Kept here so the executor,
# the collector and their tests cannot drift apart.
FRESH_NS = 300_000_000
TIMEOUT_NS = 2_000_000_000


@dataclass(frozen=True)
class Feedback:
    """One measured device sample.

    ``received_monotonic_ns`` is the host ``time.monotonic_ns()`` taken when the
    SDK read completed. It is what the DeviceFeedback message forwards as
    ``source_monotonic_ns`` and is never rewritten by DDS or by the writer.
    """

    position_rad: tuple[float, ...]
    received_monotonic_ns: int
    healthy: bool
    enabled: bool
    detail: str = ""


def positions(values, count):
    """Validate a joint vector: exactly ``count`` finite radians."""
    values = tuple(float(value) for value in values)
    if len(values) != count or not all(math.isfinite(value) for value in values):
        raise ValueError(f"expected {count} finite joint positions in radians")
    return values


def fresh(stamp, now):
    """True when ``stamp`` is a usable device stamp relative to ``now``."""
    return stamp > 0 and 0 <= now - stamp <= FRESH_NS


def advances(current, previous, modulus):
    """True when a wrapping counter moved forward by less than half its range.

    Used for firmware stamp/sequence counters that wrap. A jump of half the
    range or more is treated as a rewind rather than a large forward step, so a
    restarted source is never mistaken for fresh data.
    """
    return 0 < (current - previous) % modulus < modulus // 2


class LockedDevice:
    """Serialize every call to one device behind a dedicated re-entrant lock.

    Both the control loop and any sampling thread enter the device through this
    proxy, so an SDK handle is never used concurrently. It is the only
    concurrency guarantee the hardware layer offers.
    """

    def __init__(self, device):
        self._device = device
        self._lock = threading.RLock()

    def connect(self):
        with self._lock:
            return self._device.connect()

    def read_feedback(self):
        with self._lock:
            return self._device.read_feedback()

    def enable(self, guard=None):
        with self._lock:
            return self._device.enable(guard=guard)

    def send(self, positions):  # noqa: A002 - mirrors the device API
        with self._lock:
            return self._device.send(positions)

    def stop(self):
        with self._lock:
            return self._device.stop()

    def close(self):
        with self._lock:
            return self._device.close()

    def __getattr__(self, name):
        device = object.__getattribute__(self, "_device")
        value = getattr(device, name)
        if not callable(value):
            return value
        lock = object.__getattribute__(self, "_lock")

        def locked(*args, **kwargs):
            with lock:
                return value(*args, **kwargs)

        return locked

    def __repr__(self):
        return f"LockedDevice({object.__getattribute__(self, '_device')!r})"
