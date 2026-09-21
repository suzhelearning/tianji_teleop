"""ROS-facing acquisition runtime for schema-v1 episodes.

The collector owns no hardware. Measured joint state arrives as
``tianji_interfaces/DeviceFeedback`` from the executor and RGB arrives as
``sensor_msgs/Image`` + ``sensor_msgs/CameraInfo``/``Metadata`` from the official
RealSense driver; this module turns those streams into the two arrays the writer
persists.

Two rules are load-bearing and easy to get wrong:

* a payload is never separated from the identity that makes it trustworthy, so
  every sample is checked for boot id, session id and device length before it
  can reach the writer;
* a sample's *availability* timestamp is taken when the complete sample lands in
  this process's cache — never the device stamp, never a DDS receive time. That
  is what keeps one clock across arms, hands and cameras.
"""

from __future__ import annotations

import math
import threading
import time
from types import MappingProxyType

from tianji_runtime import device as _runtime_device
from tianji_runtime.constants import ARMS_COUNT, DEVICES, HAND_COUNT, RGB_SHAPE

from . import state
from .state import HandPairing, Staleness, rejection_reason

__all__ = [
    "CollectionRuntime",
    "DeviceChannel",
    "ImageFrame",
    "RuntimeSnapshot",
    "StateFrame",
]

_WRITER_METHODS = ("start", "stop", "check", "append_arms", "append_hands", "append_rgb")


class StateFrame:
    """One assembled measured-state sample and its availability time."""

    __slots__ = ("timestamp_ns", "qpos")

    def __init__(self, timestamp_ns: int, qpos: tuple[float, ...]):
        self.timestamp_ns = timestamp_ns
        self.qpos = qpos


class ImageFrame:
    """One complete camera sample and its availability time."""

    __slots__ = ("timestamp_ns", "rgb", "sequence", "frame_id")

    def __init__(self, timestamp_ns: int, rgb, sequence: int, frame_id: str):
        self.timestamp_ns = timestamp_ns
        self.rgb = rgb
        self.sequence = sequence
        self.frame_id = frame_id


class RuntimeSnapshot:
    """Read-only view of the newest complete samples."""

    __slots__ = ("arms", "hands", "images")

    def __init__(self, arms, hands, images):
        self.arms = arms
        self.hands = hands
        self.images = images


class DeviceChannel:
    """Latest validated feedback for one device, plus its identity checks."""

    def __init__(self, name: str, count: int, boot_id: str):
        self.name = name
        self.count = count
        self._boot_id = boot_id
        self._session_id: str | None = None
        self._sequence: int | None = None
        self._stamp: int | None = None
        self.feedback = None
        self.detail = "no sample"

    @property
    def session_id(self) -> str | None:
        return self._session_id

    @property
    def stamp(self) -> int | None:
        return self._stamp

    def reject(self, reason: str) -> str:
        self.detail = reason
        return reason

    def accept(self, message, now_ns: int, policy: Staleness) -> str | None:
        """Validate one DeviceFeedback and store it; returns a reason on rejection.

        Identity checks come first: a payload that cannot be attributed to this
        host and this executor session must never reach a dataset, no matter how
        plausible the numbers are.
        """
        if message.boot_id != self._boot_id:
            return self.reject(
                f"boot id mismatch ({message.boot_id!r} != {self._boot_id!r}); "
                "cross-host samples are never collected")
        if not message.session_id:
            return self.reject("sample has no executor session id")
        if self._session_id is None:
            self._session_id = message.session_id
        elif message.session_id != self._session_id:
            # A restarted executor is a different acquisition session. Refuse
            # rather than silently extending the previous one.
            return self.reject(
                f"executor session changed to {message.session_id!r}; restart the recording")
        positions = tuple(float(value) for value in message.position_rad)
        if len(positions) != self.count:
            return self.reject(f"expected {self.count} joint positions, got {len(positions)}")
        if not all(math.isfinite(value) for value in positions):
            return self.reject("nonfinite joint position")
        if self._sequence is not None and message.sequence < self._sequence:
            return self.reject(f"sequence went backwards ({message.sequence} < {self._sequence})")
        source = int(message.source_monotonic_ns)
        if source <= 0:
            return self.reject("no device timestamp")
        if self._stamp is not None and source < self._stamp:
            return self.reject("device timestamp went backwards")
        if source == self._stamp or message.sequence == self._sequence:
            return "duplicate"
        age = now_ns - source
        if age < 0:
            return self.reject("device timestamp is in the future")
        if age > policy.state_stale_ns:
            return self.reject(f"feedback is {age / 1e9:.3f}s old")
        if not message.healthy:
            return self.reject(f"unhealthy ({message.detail or 'no detail'})")
        self._sequence = message.sequence
        self._stamp = source
        self.feedback = _runtime_device.Feedback(
            positions, source, bool(message.healthy), bool(message.enabled), message.detail or "")
        self.detail = ""
        return None


class CollectionRuntime:
    """Assemble measured state and RGB into writer-ready samples.

    The executor publishes; this class only consumes. It never opens a device,
    never owns a camera pipeline and never commands anything, so a collector
    failure can hold an episode partial but cannot affect robot control.
    """

    def __init__(self, writer, *, boot_id: str, cameras, state_stale_s: float = state.STATE_STALE_S,
                 image_stale_s: float = state.IMAGE_STALE_S,
                 startup_timeout_s: float = state.STARTUP_TIMEOUT_S,
                 episode_fresh_timeout_s: float = state.EPISODE_FRESH_TIMEOUT_S):
        for method in _WRITER_METHODS:
            if not callable(getattr(writer, method, None)):
                raise TypeError(f"collection writer has no {method}()")
        self._writer = writer
        self._boot_id = boot_id
        self._policy = Staleness(state_stale_s, image_stale_s)
        self._startup_timeout_s = float(startup_timeout_s)
        self._episode_fresh_timeout_s = float(episode_fresh_timeout_s)
        if not cameras:
            raise ValueError("collection runtime needs at least one camera role")
        self._camera_names = tuple(cameras)

        self._lock = threading.RLock()
        self._ready = threading.Condition(self._lock)
        self._closed = False
        self._recording = False
        self._episode_started = False
        self._episode_origin_ns = 0
        self._fault: str | None = None
        self._episode_fault: str | None = None

        self._channels = {
            "arms": DeviceChannel("arms", ARMS_COUNT, boot_id),
            "left_hand": DeviceChannel("left_hand", HAND_COUNT, boot_id),
            "right_hand": DeviceChannel("right_hand", HAND_COUNT, boot_id),
        }
        self._pairing = HandPairing(self._policy)
        self._arms: StateFrame | None = None
        self._hands: StateFrame | None = None
        self._images: dict[str, ImageFrame] = {}
        self._image_details: dict[str, str] = {name: "no sample" for name in self._camera_names}
        self._arms_detail = "no sample"

    # ------------------------------------------------------------------ lifecycle

    @property
    def recording(self) -> bool:
        return self._recording

    @property
    def episode_started(self) -> bool:
        return self._episode_started

    @property
    def fault(self) -> str | None:
        with self._ready:
            return self._fault

    @property
    def cameras(self) -> tuple[str, ...]:
        return self._camera_names

    def start(self):
        """Validate the initial state; the streams themselves are already live."""
        with self._ready:
            if self._closed:
                raise RuntimeError("collection runtime is closed")
        return self

    def request_stop(self) -> None:
        with self._ready:
            self._closed = True
            self._ready.notify_all()

    def close(self) -> None:
        """Stop accepting samples and stop the writer. Idempotent."""
        with self._ready:
            if self._closed:
                return
            self._closed = True
        errors: list[str] = []
        try:
            self._writer.stop()
        except Exception as error:  # noqa: BLE001 - reported, not raised mid-cleanup
            errors.append(f"writer stop failed: {type(error).__name__}: {error}")
        with self._ready:
            self._recording = False
        if errors:
            raise RuntimeError("collection runtime cleanup failed: " + "; ".join(errors))

    # -------------------------------------------------------------------- inputs

    def on_arms(self, message, received_ns: int | None = None) -> None:
        self._on_state("arms", message, received_ns)

    def on_hand(self, side: str, message, received_ns: int | None = None) -> None:
        if side not in ("left_hand", "right_hand"):
            raise ValueError(f"unknown hand channel: {side}")
        self._on_state(side, message, received_ns)

    def _on_state(self, device: str, message, received_ns: int | None) -> None:
        now = time.monotonic_ns() if received_ns is None else int(received_ns)
        channel = self._channels[device]
        with self._ready:
            if self._closed and not self._recording:
                return
            reason = channel.accept(message, now, self._policy)
            if reason is not None:
                if reason != "duplicate" and self._recording:
                    self._latch_fault(f"{device}: {reason}")
                if device == "arms":
                    self._arms_detail = reason
                self._ready.notify_all()
                return
            self._writer_check_soon()
            if device == "arms":
                self._publish_arms(channel, now)
            else:
                self._pairing.stage(device, channel.feedback, now)
                self._publish_hands(now)
            self._check_recording_faults()
            self._ready.notify_all()

    def on_image(self, role: str, image) -> None:
        """Accept one validated camera frame from a stream validator.

        ``image`` is the validator's paired result: it already carries the source
        sequence, the frame id and the monotonic receive time of its *Image*
        callback, so nothing here re-derives or re-stamps it.
        """
        if role not in self._image_details:
            raise ValueError(f"unknown camera role: {role}")
        with self._ready:
            if self._closed and not self._recording:
                return
            frame = ImageFrame(image.timestamp_ns, image.rgb, image.sequence, image.frame_id)
            self._images[role] = frame
            self._image_details[role] = ""
            if self._recording and image.received_ns >= self._episode_origin_ns:
                self._writer.append_rgb(role, frame.timestamp_ns, frame.rgb)
            self._ready.notify_all()

    def on_camera_fault(self, role: str, detail: str) -> None:
        """Mark one camera unusable for the current episode.

        Called by the stream validator on a real source failure (publisher change,
        sequence rewind, unpaired stream). The current episode stays partial; the
        segment is never extended with unrelated frames.
        """
        with self._ready:
            if role not in self._image_details:
                raise ValueError(f"unknown camera role: {role}")
            self._image_details[role] = detail
            self._images.pop(role, None)
            self._latch_fault(f"camera {role}: {detail}")
            self._ready.notify_all()

    def rearm_cameras(self) -> None:
        """Forget a retired camera stream only after its episode was drained."""
        with self._ready:
            if self._recording:
                raise RuntimeError("cannot re-arm cameras during an episode")
            self._images.clear()
            self._image_details = {name: "waiting for recertified stream"
                                   for name in self._camera_names}
            if self._fault is not None and self._fault.startswith("collection runtime: camera "):
                self._fault = None
            # Preserve _episode_fault: a retired partial must never become savable.
            self._ready.notify_all()

    def rearm_feedback(self, session_id: str) -> None:
        """Bind fresh feedback to a new executor only between drained episodes."""
        with self._ready:
            if self._recording:
                raise RuntimeError("cannot replace the executor during an episode")
            self._channels = {
                name: DeviceChannel(name, ARMS_COUNT if name == "arms" else HAND_COUNT,
                                    self._boot_id)
                for name in DEVICES}
            for channel in self._channels.values():
                channel._session_id = session_id
            self._pairing = HandPairing(self._policy)
            self._arms = self._hands = None
            self._arms_detail = "waiting for the new executor"
            self._fault = None
            self._ready.notify_all()

    def _writer_check_soon(self) -> None:
        """Surface a writer fault promptly instead of only on the next poll."""
        try:
            self._writer.check()
        except Exception as error:  # noqa: BLE001 - latched, not raised on the callback
            self._latch_fault(f"writer failed: {error}")

    # --------------------------------------------------------------- publication

    def _publish_arms(self, channel: DeviceChannel, now: int) -> None:
        token = channel.stamp
        if token is None:
            return
        qpos = channel.feedback.position_rad  # immutable tuple from the channel
        with self._ready:
            stamp = time.monotonic_ns()      # availability after assembly, under the lock
            if self._recording and self._arms is not None and stamp - self._arms.timestamp_ns > self._policy.state_stale_ns:
                self._latch_fault("arms acquisition gap exceeded the state freshness limit")
            self._arms = StateFrame(stamp, qpos)
            self._arms_detail = ""
            if self._recording:
                self._writer.append_arms(stamp, qpos)

    def _publish_hands(self, now: int) -> None:
        combined = self._pairing.take(now)
        if combined is None:
            return
        stamp, qpos = combined
        with self._ready:
            if self._recording and self._hands is not None and stamp - self._hands.timestamp_ns > self._policy.state_stale_ns:
                self._latch_fault("hand acquisition gap exceeded the state freshness limit")
            self._hands = StateFrame(stamp, qpos)
            if self._recording:
                self._writer.append_hands(stamp, qpos)

    # ------------------------------------------------------------------- checks

    def check(self) -> None:
        """Raise on any fatal acquisition or writer failure."""
        with self._ready:
            if self._fault is not None:
                raise RuntimeError(self._fault)
            self._check_recording_faults()
        self._writer.check()

    def check_episode(self) -> None:
        """Raise for acquisition failure in the last segment, frozen by ``end_episode``."""
        with self._ready:
            if self._episode_fault is not None:
                raise RuntimeError(self._episode_fault)

    def _latch_fault(self, detail: str) -> None:
        with self._ready:
            message = f"collection runtime: {detail}"
            if self._fault is None:
                self._fault = message
            if self._recording and self._episode_fault is None:
                self._episode_fault = message

    def _check_recording_faults(self) -> None:
        """Only meaningful while recording; a pre-episode stall is not a fault."""
        if not self._recording:
            return
        now = time.monotonic_ns()
        arms = self._channels["arms"]
        if arms.feedback is None or arms.stamp is None:
            self._latch_fault(f"arms: {self._arms_detail or 'no feedback'}")
        elif now - arms.stamp > self._policy.state_stale_ns:
            self._latch_fault(f"arms: feedback is {(now - arms.stamp) / 1e9:.3f}s old")
        for side in ("left_hand", "right_hand"):
            channel = self._channels[side]
            if channel.feedback is None or channel.stamp is None:
                self._latch_fault(f"{side}: no feedback")
            elif now - channel.stamp > self._policy.state_stale_ns:
                self._latch_fault(f"{side}: feedback is {(now - channel.stamp) / 1e9:.3f}s old")
        for role in self._camera_names:
            frame = self._images.get(role)
            if frame is None:
                self._latch_fault(f"camera {role}: {self._image_details.get(role) or 'no image'}")
            elif now - frame.timestamp_ns > self._policy.image_stale_ns:
                self._latch_fault(
                    f"camera {role}: image is {(now - frame.timestamp_ns) / 1e9:.3f}s old")

    # ---------------------------------------------------------------- readiness

    def missing_inputs(self) -> list[str]:
        """Every input that is not currently usable, as operator-readable text."""
        now = time.monotonic_ns()
        problems: list[str] = []
        for name in DEVICES:
            channel = self._channels[name]
            if channel.feedback is None or channel.stamp is None:
                problems.append(f"{name}: no feedback")
            elif now - channel.stamp > self._policy.state_stale_ns:
                problems.append(f"{name}: feedback is {(now - channel.stamp) / 1e9:.3f}s old")
        for role in self._camera_names:
            frame = self._images.get(role)
            if frame is None:
                problems.append(f"camera {role}: {self._image_details.get(role) or 'no image'}")
            elif now - frame.timestamp_ns > self._policy.image_stale_ns:
                problems.append(f"camera {role}: image is {(now - frame.timestamp_ns) / 1e9:.3f}s old")
        return problems

    def _wait_fresh(self) -> None:
        deadline = time.monotonic() + self._episode_fresh_timeout_s
        with self._ready:
            while not self._closed:
                if not self.missing_inputs():
                    return
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError(
                        "episode start requires fresh measured feedback and images: "
                        + "; ".join(self.missing_inputs()))
                self._ready.wait(min(remaining, 0.05))
            raise RuntimeError("collection runtime is closed")

    # ----------------------------------------------------------- episode control

    def begin_episode(self, now_ns: int) -> None:
        """Open a writer episode. Requires fresh measured feedback and images."""
        with self._ready:
            if self._closed:
                raise RuntimeError("collection runtime is closed")
            if self._recording:
                raise RuntimeError("an episode is already being recorded")
        self._wait_fresh()
        self._writer.start(now_ns)
        with self._ready:
            self._episode_origin_ns = now_ns
            self._fault = None
            self._episode_fault = None
            self._recording = True
            self._episode_started = True
            self._ready.notify_all()

    def end_episode(self) -> bool:
        """Freeze the current episode. ``False`` when nothing was recording."""
        with self._ready:
            if not self._recording:
                return False
            healthy = self._episode_fault is None
            self._recording = False
            self._ready.notify_all()
        try:
            self._writer.stop()
        except Exception as error:  # noqa: BLE001 - surfaced by check_episode instead
            if healthy:
                self._episode_fault = f"collection runtime: writer stop failed: {error}"
            return False
        return healthy

    def snapshot(self) -> RuntimeSnapshot:
        with self._ready:
            return RuntimeSnapshot(self._arms, self._hands, MappingProxyType(dict(self._images)))


def validate_image_shape(rgb) -> str | None:
    """Structural check for one RGB buffer; ``None`` when acceptable."""
    shape = getattr(rgb, "shape", None)
    if shape != RGB_SHAPE:
        return f"expected RGB{tuple(RGB_SHAPE)}, got {shape}"
    if getattr(rgb, "dtype", None) is None or str(rgb.dtype) != "uint8":
        return f"expected uint8 RGB, got dtype {getattr(rgb, 'dtype', None)}"
    return None
