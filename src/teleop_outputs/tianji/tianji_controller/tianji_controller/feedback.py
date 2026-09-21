"""Measured-feedback sampling and publication hand-off for the executor.

The executor owns the one and only device session. This module samples the same
``LockedDevice`` proxies the control loop uses, so publishing never opens a
second SDK handle. SDK calls are serialized; DDS serialization and service
waits happen on the observer's own thread, never on the control loop.

Two rules matter for a dataset:

* a sample is identified by its device stamp, not by its values — a stationary
  arm legitimately reports the same positions, so comparing positions would
  either drop real samples or re-publish stale ones;
* the stamp is read once, from the SDK, and then carried unchanged all the way
  into the published message. Neither DDS nor the writer may re-stamp it.
"""

from __future__ import annotations

import queue
import math
import threading
import time

from tianji_runtime import Feedback

SAMPLE_RATE_HZ = 120.0

# The publish queue is a *latest value* slot: a slow subscriber must never make
# the sampler queue up history, and a dropped intermediate sample is correct
# because every sample is independently timestamped.
QUEUE_DEPTH = 1


class FeedbackHub:
    """Sample every device at a fixed rate and hand the newest values out."""

    def __init__(self, hardware, devices, *, rate_hz: float = SAMPLE_RATE_HZ):
        missing = [name for name in devices if name not in hardware]
        if missing:
            raise ValueError(f"feedback sampling is missing devices: {', '.join(missing)}")
        self._hardware = {name: hardware[name] for name in devices}
        if not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError("feedback sampling rate must be finite and positive")
        self._period_ns = max(1, round(1e9 / float(rate_hz)))
        self._queue: queue.Queue[dict[str, Feedback]] = queue.Queue(maxsize=QUEUE_DEPTH)
        self._lock = threading.Lock()
        self._latest: dict[str, Feedback] = {}
        self._sequence: dict[str, int] = {name: 0 for name in devices}
        self._error: str | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # ------------------------------------------------------------------ lifecycle

    def start(self) -> None:
        """Begin sampling. Must run after the devices are connected."""
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._loop, name="feedback-sampler", daemon=True)
        self._thread.start()

    def request_stop(self) -> None:
        """Revoke future reads without waiting before the motors are stopped."""
        self._stop.set()

    def close(self) -> None:
        """Join only after hardware stop; report a stuck SDK rather than hiding it."""
        self.request_stop()
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(5.0)
            if thread.is_alive():
                raise RuntimeError("feedback sampler did not stop")

    @property
    def error(self) -> str | None:
        with self._lock:
            return self._error

    @property
    def devices(self):
        return tuple(self._hardware)

    def sequence(self, device: str) -> int:
        with self._lock:
            return self._sequence[device]

    # -------------------------------------------------------------------- sampling

    def _loop(self) -> None:
        period = self._period_ns
        next_ns = time.monotonic_ns()
        while not self._stop.is_set():
            now = time.monotonic_ns()
            if now < next_ns:
                # Resync instead of bursting after the control loop held a lock.
                self._stop.wait(min((next_ns - now) / 1e9, 0.05))
                continue
            next_ns += period
            if next_ns <= now:
                next_ns = now + period
            self._sample_once()

    def _sample_once(self) -> None:
        changed = False
        for name, device in self._hardware.items():
            if self._stop.is_set():
                break
            try:
                value = device.read_feedback()
                if not isinstance(value, Feedback):
                    raise TypeError("device did not return Feedback")
            except Exception as error:
                with self._lock:
                    self._error = f"{name}: read failed: {type(error).__name__}: {error}"
                    previous = self._latest.get(name)
                # Report health immediately, without inventing a measurement stamp.
                value = Feedback(
                    previous.position_rad if previous is not None else (),
                    previous.received_monotonic_ns if previous is not None else 0,
                    False, previous.enabled if previous is not None else False,
                    str(error))
            with self._lock:
                previous = self._latest.get(name)
                if previous is None or (
                    previous.received_monotonic_ns, previous.healthy, previous.enabled, previous.detail
                ) != (value.received_monotonic_ns, value.healthy, value.enabled, value.detail):
                    self._latest[name] = value
                    self._sequence[name] += 1
                    changed = True
        if changed:
            # Include the latest value from every device: replacing a pending slot
            # must not erase an unchanged device's only unpublished measurement.
            self._offer(self.latest())

    def _offer(self, batch: dict[str, Feedback]) -> None:
        """Publish the batch downstream without ever blocking the sampler."""
        try:
            self._queue.put_nowait(batch)
        except queue.Full:
            # Replace the pending batch: the newest measurement is the useful one.
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(batch)
            except queue.Full:
                pass

    def drain(self, timeout: float | None = None) -> dict[str, Feedback] | None:
        """Return the newest unpublished batch, or ``None`` when nothing is pending."""
        if timeout is None:
            try:
                return self._queue.get_nowait()
            except queue.Empty:
                return None
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def latest(self) -> dict[str, Feedback]:
        with self._lock:
            return dict(self._latest)

    def snapshot(self) -> dict[str, Feedback]:
        """Newest sample per device, for the control loop's own logging."""
        return self.latest()
