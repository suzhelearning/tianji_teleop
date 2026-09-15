"""Bounded-latency capture of real joint state and RGB for schema-v1 episodes.

This module owns acquisition only; hardware lifecycle, episode numbering and
finalization stay with the caller:

* joint state is polled through the existing per-device ``LockedDevice``
  delegates at a 120 Hz target, never through a second SDK session;
* RGB frames are copied once from the SDK buffer, published together with the
  host availability timestamp taken at publication, and handed to the writer so
  disk I/O stays off the acquisition threads.

Only observations are recorded here: measured arm/hand feedback and the RGB
frames. No target, command or interpolated action is derived, and training
features are built by the loader from these observation timestamps.

No device timestamp is persisted. Every timestamp written here is the host
``time.monotonic_ns`` at which the complete sample became available in the
common buffer, which is also the boundary inference reads through
``CollectionRuntime.snapshot``.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from types import MappingProxyType
from typing import Mapping, Protocol

import numpy as np

WIDTH = 1280
HEIGHT = 720
FPS = 30
RGB_SHAPE = (HEIGHT, WIDTH, 3)

ARMS_COUNT = 14
HAND_COUNT = 20
DEVICES = ("arms", "left_hand", "right_hand")
_WRITER_METHODS = ("start", "stop", "check", "append_arms", "append_hands", "append_rgb")


def _load_realsense():
    """Import pyrealsense2 lazily; the recorder imports without the camera SDK."""
    try:
        import pyrealsense2 as rs
    except ImportError as error:
        raise RuntimeError(
            "RealSense RGB capture requires pyrealsense2 in the interpreter that runs the "
            "recorder (see realsense.sh / REALSENSE_PYTHON); install it there or run the "
            "recorder with that Python interpreter") from error
    return rs


def _camera_selection(cameras: Mapping[str, str]) -> dict[str, str]:
    """Validate the enabled {name: serial} camera selection."""
    if not isinstance(cameras, Mapping) or not cameras:
        raise ValueError("collection cameras must be a non-empty {name: serial} mapping")
    selection: dict[str, str] = {}
    for name, serial in cameras.items():
        if not isinstance(name, str) or not name:
            raise ValueError("collection camera name must be a non-empty string")
        if not isinstance(serial, str) or not serial.strip():
            raise ValueError(f"collection camera {name}: serial must be a non-empty string")
        if serial in selection.values():
            raise ValueError(f"collection camera {name}: serial {serial} is selected twice")
        selection[name] = serial
    return selection


@dataclass(frozen=True)
class CameraProfile:
    """One validated camera: the selected serial really exposes RGB8 1280x720@30."""

    name: str
    serial: str
    product: str
    usb_type: str
    stream: str

    def describe(self) -> str:
        return f"{self.name} SN={self.serial} ({self.product}, USB {self.usb_type}) {self.stream}"


def _color_profiles(rs, device):
    """Real color stream profiles of one device; other streams and devices stay untouched."""
    for sensor in device.query_sensors():
        try:
            profiles = sensor.get_stream_profiles()
        except Exception:
            continue
        for profile in profiles:
            try:
                if profile.stream_type() == rs.stream.color:
                    yield profile
            except Exception:
                continue


def _video_mode(profile):
    video = profile.as_video_stream_profile()
    return video.format(), int(video.width()), int(video.height()), int(video.fps())


def _mode_text(mode) -> str:
    return f"{mode[0]} {mode[1]}x{mode[2]}@{mode[3]}"


def validate_camera_profiles(cameras: Mapping[str, str], *, realsense=None, context=None) -> dict[str, CameraProfile]:
    """Reject any selected camera that cannot stream exactly RGB8 1280x720@30.

    Enumerates only the configured serials and never starts a stream, so it runs
    before any robot or camera session is opened. A camera that needs a
    different mode (for example a USB 2.1 link that cannot carry this profile)
    is reported with its serial, USB descriptor and available color modes
    instead of being silently downgraded.
    """
    selection = _camera_selection(cameras)
    rs = _load_realsense() if realsense is None else realsense
    context = rs.context() if context is None else context
    attached: dict[str, object] = {}
    for device in context.query_devices():
        attached.setdefault(str(device.get_info(rs.camera_info.serial_number)), device)
    missing = [f"{name} SN={serial}" for name, serial in selection.items() if serial not in attached]
    if missing:
        listed = ", ".join(sorted(attached)) or "none"
        raise RuntimeError(f"collection camera profile preflight: not attached: {', '.join(missing)}; "
                           f"attached RealSense serials: {listed}")
    profiles: dict[str, CameraProfile] = {}
    for name, serial in selection.items():
        device = attached[serial]
        product = str(device.get_info(rs.camera_info.name))
        usb_type = str(device.get_info(rs.camera_info.usb_type_descriptor))
        wanted = (rs.format.rgb8, WIDTH, HEIGHT, FPS)
        modes = []
        match = None
        for profile in _color_profiles(rs, device):
            try:
                mode = _video_mode(profile)
            except Exception:
                continue
            text = _mode_text(mode)
            if text not in modes:
                modes.append(text)
            if match is None and mode == wanted:
                match = text
        if match is None:
            available = ", ".join(modes) or "no color stream profiles"
            raise RuntimeError(
                f"collection camera {name} SN={serial} ({product}, USB {usb_type}) does not expose "
                f"RGB8 {WIDTH}x{HEIGHT}@{FPS}; available color modes: {available}")
        profiles[name] = CameraProfile(name, serial, product, usb_type, match)
    return profiles


class LockedDevice:
    """Serialize every call to one device behind a dedicated re-entrant lock.

    The control loop keeps sending and stopping through this proxy while the
    recorder polls the same device, so a device is never entered concurrently.
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

    def send(self, positions):
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


class CameraSource(Protocol):
    """One color stream: ``read`` returns the next frame or ``None`` on timeout."""

    def read(self, timeout_ms: int):
        """Return ``(frame_sequence, rgb_uint8_hwc)`` for a new frame, else ``None``."""

    def close(self) -> None:
        """Release the stream; idempotent."""


@dataclass(frozen=True)
class StateFrame:
    """A complete joint snapshot: host availability stamp plus immutable positions."""

    timestamp_ns: int
    qpos: tuple[float, ...]


@dataclass(frozen=True)
class ImageFrame:
    """A complete RGB frame: host availability stamp plus an owned ``uint8[H,W,3]``.

    The array is marked non-writeable: the snapshot and the writer queue share
    the same buffer, so no consumer may mutate it.
    """

    timestamp_ns: int
    rgb: np.ndarray


@dataclass(frozen=True)
class RuntimeSnapshot:
    """Latest common input buffer; cached values keep their original availability stamps."""

    arms: StateFrame | None
    hands: StateFrame | None
    images: Mapping[str, ImageFrame]


@dataclass
class _CameraState:
    name: str
    serial: str
    thread: threading.Thread | None = None
    source: object | None = None
    first_frame: bool = False
    close_error: str | None = None


class _RealSenseCamera:
    """One RealSense color stream forced to the exact configured mode."""

    def __init__(self, rs, context, name: str, serial: str,
                 width: int = WIDTH, height: int = HEIGHT, fps: int = FPS):
        self._name = name
        self._serial = serial
        self._close_lock = threading.Lock()
        self._running = False
        self._frame = None  # keep SDK storage alive until the acquisition loop copies it
        self._pipeline = rs.pipeline(context)
        try:
            config = rs.config()
            config.enable_device(serial)
            config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
            profile = self._pipeline.start(config)
            self._running = True
            video = profile.get_stream(rs.stream.color).as_video_stream_profile()
            actual = (video.format(), int(video.width()), int(video.height()), int(video.fps()))
        except BaseException:
            self._close_quietly()
            raise
        if actual != (rs.format.rgb8, int(width), int(height), int(fps)):
            self._close_quietly()
            raise RuntimeError(
                f"camera {name} SN={serial}: stream started as {_mode_text(actual)} instead of "
                f"RGB8 {width}x{height}@{fps}; refusing to collect a different mode")

    def read(self, timeout_ms: int):
        frames = self._pipeline.wait_for_frames(timeout_ms)
        color = frames.get_color_frame()
        if not color:
            return None
        self._frame = color
        return int(color.get_frame_number()), np.asanyarray(color.get_data())

    def close(self):
        with self._close_lock:
            if not self._running:
                return
            self._running = False
            pipeline = self._pipeline
        pipeline.stop()
        self._frame = None

    def _close_quietly(self):
        try:
            self.close()
        except Exception:
            pass  # the original profile/startup error is the one that matters


class CollectionRuntime:
    """Acquire measured joint state and RGB while the robot runs.

    ``hardware`` holds the already connected ``LockedDevice`` proxies for
    ``arms``/``left_hand``/``right_hand``; ``cameras`` maps dataset camera names
    to RealSense serials. ``start`` must run after the devices are connected and
    before they are enabled; the writer owns episode numbering and file
    lifecycle. Nothing here reconnects, stitches, finalizes or derives targets:
    only observations are captured. Samples are handed to the writer's
    non-blocking ``append_*`` queue; a rejected enqueue surfaces through
    ``writer.check()``, which ``check`` polls.
    """

    def __init__(self, hardware: Mapping[str, LockedDevice], cameras: Mapping[str, str], writer,
                 state_rate_hz: float = 120.0, *, state_stale_s: float = 0.3,
                 image_stale_s: float = 2.0, startup_timeout_s: float = 15.0,
                 episode_fresh_timeout_s: float = 0.5,
                 camera_read_timeout_ms: int = 200, join_timeout_s: float = 5.0,
                 _camera_factory=None, _realsense=None):
        missing = [name for name in DEVICES if name not in hardware]
        if missing:
            raise ValueError(f"collection runtime is missing hardware devices: {', '.join(missing)}")
        for name in DEVICES:
            if not isinstance(hardware[name], LockedDevice):
                raise TypeError(f"collection hardware {name} must be a LockedDevice proxy")
        for method in _WRITER_METHODS:
            if not callable(getattr(writer, method, None)):
                raise TypeError(f"collection writer has no {method}()")
        rate = float(state_rate_hz)
        if not (math.isfinite(rate) and rate > 0):
            raise ValueError("collection state_rate_hz must be positive")
        self._hardware = {name: hardware[name] for name in DEVICES}
        self._cameras = {name: _CameraState(name, serial) for name, serial in _camera_selection(cameras).items()}
        self._writer = writer
        self._state_period_ns = max(1, round(1e9 / rate))
        self._state_stale_s = float(state_stale_s)
        self._state_stale_ns = int(round(float(state_stale_s) * 1e9))
        self._image_stale_ns = int(round(float(image_stale_s) * 1e9))
        self._startup_timeout_s = float(startup_timeout_s)
        self._episode_fresh_timeout_s = float(episode_fresh_timeout_s)
        self._camera_read_timeout_ms = int(camera_read_timeout_ms)
        self._join_timeout_s = float(join_timeout_s)
        self._camera_factory = _camera_factory
        self._realsense = _realsense
        self._lock = threading.RLock()
        self._ready = threading.Condition(self._lock)
        self._stop = threading.Event()
        self._started = False
        self._closed = False
        self._recording = False
        self._episode_started = False
        self._fault: str | None = None
        self._episode_fault: str | None = None
        self._startup_error: str | None = None
        self._state_thread: threading.Thread | None = None
        self._arms: StateFrame | None = None
        self._arms_token: int | None = None
        self._arms_detail = ""
        self._hands: StateFrame | None = None
        self._hand_tokens: dict[str, int] = {}
        self._hands_detail = ""
        self._images: dict[str, ImageFrame] = {}

    # ---------------------------------------------------------------- lifecycle

    def start(self):
        """Start camera threads and the state sampler; wait for first samples.

        Call after the hardware is connected and before it is enabled. A camera
        that cannot start or stream, or a device without fresh feedback, fails
        startup with a descriptive error instead of collecting silently.
        """
        with self._ready:
            if self._closed:
                raise RuntimeError("collection runtime is closed")
            if self._started:
                raise RuntimeError("collection runtime is already started")
            self._started = True
        try:
            open_camera = self._camera_factory
            if open_camera is None:
                rs = _load_realsense() if self._realsense is None else self._realsense
                context = rs.context()

                def open_camera(name, serial):
                    return _RealSenseCamera(rs, context, name, serial)
            for camera in self._cameras.values():
                camera.thread = threading.Thread(target=self._camera_loop, args=(camera, open_camera),
                                                 name=f"collection-{camera.name}", daemon=True)
                camera.thread.start()
            self._state_thread = threading.Thread(target=self._state_loop, name="collection-state", daemon=True)
            self._state_thread.start()
            self._wait_ready()
        except Exception as error:
            detail = str(error) if isinstance(error, RuntimeError) else f"{type(error).__name__}: {error}"
            cleanup = self._shutdown()
            message = f"collection runtime startup failed: {detail}"
            if cleanup:
                message += "; cleanup: " + "; ".join(cleanup)
            with self._ready:
                self._startup_error = message
            raise RuntimeError(message) from error
        except BaseException:
            self._shutdown()
            raise
        return self

    def request_stop(self):
        """Non-blocking shutdown request; capture stops without stitching episodes."""
        self._stop.set()
        with self._ready:
            self._ready.notify_all()

    def close(self):
        """Join acquisition threads, release cameras and stop the writer.

        Must run after the caller stopped and closed the motors. The writer only
        stops accepting; finishing (or aborting) the episode file and the
        operator success label stay with the caller. Idempotent.
        """
        with self._ready:
            if self._closed:
                return
            self._closed = True
        errors = self._shutdown()
        try:
            self._writer.stop()
        except Exception as error:
            errors.append(f"writer stop failed: {type(error).__name__}: {error}")
        with self._ready:
            self._recording = False
        if errors:
            raise RuntimeError("collection runtime cleanup failed: " + "; ".join(errors))

    def _shutdown(self) -> list[str]:
        self._stop.set()
        with self._ready:
            self._ready.notify_all()
        return self._stop_capture()

    def _stop_capture(self) -> list[str]:
        """Join every acquisition thread and release each camera, even on partial startup."""
        with self._ready:
            threads = [thread for thread in (self._state_thread, *(c.thread for c in self._cameras.values()))
                       if thread is not None]
            sources = {camera.name: camera.source for camera in self._cameras.values()}
        errors: list[str] = []
        for thread in threads:
            try:
                thread.join(self._join_timeout_s)
            except Exception as error:
                errors.append(f"joining {thread.name} failed: {type(error).__name__}: {error}")
                continue
            if thread.is_alive():
                errors.append(f"{thread.name} did not stop within {self._join_timeout_s:g}s")
        for name, source in sources.items():
            if source is None:
                continue
            try:
                source.close()
            except Exception as error:
                errors.append(f"camera {name} release failed: {type(error).__name__}: {error}")
        for camera in self._cameras.values():
            if camera.close_error:
                errors.append(f"camera {camera.name} release failed: {camera.close_error}")
        return errors

    # ------------------------------------------------------------------ checks

    def check(self):
        """Raise on any fatal acquisition, thread or writer failure.

        A duplicated feedback frame inside the stale window is not a fault; a
        state stream that stays stale ends the episode. Readiness before an
        episode opens is enforced by ``begin_episode``, so a device lock held by
        enable/stop cannot fault a session that is not collecting yet. Use the
        non-raising ``fault`` property inside a robot control loop; this method
        is for paths that intend to stop or finalize.
        """
        with self._ready:
            if self._startup_error is not None:
                raise RuntimeError(self._startup_error)
            self._check_recording_faults()
            self._check_thread_faults()
            if self._fault is not None:
                raise RuntimeError(self._fault)
        self._writer.check()

    def check_episode(self) -> None:
        """Raise for acquisition failure in the last segment, frozen by ``end_episode``.

        Later service faults do not invalidate a stopped segment. Persistence
        errors remain the writer's responsibility when the caller finalizes it.
        """
        with self._ready:
            if self._episode_fault is not None:
                raise RuntimeError(self._episode_fault)

    def _check_thread_faults(self):
        """Inspect thread liveness while holding ``_ready``; never join or read devices."""
        if not self._stop.is_set():
            for thread in (self._state_thread, *(c.thread for c in self._cameras.values())):
                if thread is not None and thread.ident is not None and not thread.is_alive():
                    self._latch_fault(f"{thread.name} thread exited unexpectedly")

    def _latch_fault(self, detail: str):
        with self._ready:
            if self._stop.is_set() and not self._recording:
                return
            message = f"collection runtime: {detail}"
            if self._fault is None:
                self._fault = message
            if self._recording and self._episode_fault is None:
                self._episode_fault = message
            self._ready.notify_all()

    # ---------------------------------------------------------- state sampling

    def _state_loop(self):
        period_ns = self._state_period_ns
        next_ns = time.monotonic_ns()
        staged: dict[str, tuple[int, tuple[float, ...]]] = {}
        try:
            while not self._stop.is_set():
                now = time.monotonic_ns()
                if now < next_ns:
                    self._stop.wait(min((next_ns - now) / 1e9, .05))
                    continue
                next_ns += period_ns
                if next_ns <= now:
                    next_ns = now + period_ns  # fell behind: resync instead of bursting
                self._sample_once(staged)
        except Exception as error:
            self._latch_fault(f"state sampler failed: {type(error).__name__}: {error}")

    def _sample_once(self, staged):
        arms = self._hardware["arms"].read_feedback()
        self._publish_arms(arms, time.monotonic_ns())
        left = self._hardware["left_hand"].read_feedback()
        right = self._hardware["right_hand"].read_feedback()
        self._publish_hands(left, right, staged, time.monotonic_ns())
        self._check_recording_faults()

    def _staleness(self, feedback, count: int, now: int) -> str | None:
        """``None`` when the feedback is healthy, fresh, finite and complete."""
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
        if age > self._state_stale_ns:
            return f"feedback is {age / 1e9:.3f}s old"
        return None

    def _publish_arms(self, feedback, now: int):
        reason = self._staleness(feedback, ARMS_COUNT, now)
        if reason is not None:
            with self._ready:
                self._arms_detail = reason
            return
        token = feedback.received_monotonic_ns
        if token == self._arms_token:
            with self._ready:
                self._arms_detail = ""  # a duplicate inside the window is not a stall
            return
        qpos = tuple(feedback.position_rad)  # free for the driver's immutable tuple
        with self._ready:
            stamp = time.monotonic_ns()      # availability after assembly and buffer lock
            if self._recording and self._arms is not None and stamp - self._arms.timestamp_ns > self._state_stale_ns:
                self._latch_fault("arms acquisition gap exceeded the state freshness limit")
            self._arms = StateFrame(stamp, qpos)
            self._arms_token = token
            self._arms_detail = ""
            self._ready.notify_all()
            self._writer.append_arms(stamp, qpos)

    def _publish_hands(self, left, right, staged, now: int):
        left_reason = self._staleness(left, HAND_COUNT, now)
        right_reason = self._staleness(right, HAND_COUNT, now)
        with self._ready:
            published = self._hand_tokens
            self._hands_detail = "; ".join(
                part for part in (f"left_hand: {left_reason}" if left_reason else "",
                                  f"right_hand: {right_reason}" if right_reason else "") if part)
        if left_reason is not None:
            staged.pop("left_hand", None)
        if right_reason is not None:
            staged.pop("right_hand", None)
        if left_reason is None and left.received_monotonic_ns != published.get("left_hand"):
            staged["left_hand"] = (left.received_monotonic_ns, tuple(left.position_rad))
        if right_reason is None and right.received_monotonic_ns != published.get("right_hand"):
            staged["right_hand"] = (right.received_monotonic_ns, tuple(right.position_rad))
        if len(staged) != 2:
            return
        now = time.monotonic_ns()
        for side in ("left_hand", "right_hand"):
            if now - staged[side][0] > self._state_stale_ns:
                del staged[side]  # a staged half that outlived the window is never published
        if len(staged) != 2:
            return
        left_token, left_qpos = staged["left_hand"]
        right_token, right_qpos = staged["right_hand"]
        qpos = left_qpos + right_qpos  # the 40-value tuple is formed once
        with self._ready:
            stamp = time.monotonic_ns()
            if self._recording and self._hands is not None and stamp - self._hands.timestamp_ns > self._state_stale_ns:
                self._latch_fault("hands acquisition gap exceeded the state freshness limit")
            self._hands = StateFrame(stamp, qpos)
            self._hand_tokens = {"left_hand": left_token, "right_hand": right_token}
            self._hands_detail = ""
            self._ready.notify_all()
            staged.clear()
            self._writer.append_hands(stamp, qpos)

    def _check_recording_faults(self):
        """Check buffered progress without depending on acquisition threads returning.

        Device locks can delay enable/stop outside a segment without faulting
        it. During recording, both fresh input tokens and recent publications
        are required; checking only the last read's health misses a blocked read.
        """
        with self._ready:
            if not self._recording:
                return
            now = time.monotonic_ns()
            streams = (("arms", self._arms, self._arms_detail, (self._arms_token,)),
                       ("hands", self._hands, self._hands_detail,
                        (self._hand_tokens.get("left_hand"), self._hand_tokens.get("right_hand"))))
            for label, frame, detail, tokens in streams:
                if not detail and (frame is None or now - frame.timestamp_ns > self._state_stale_ns
                                   or any(token is None or now - token > self._state_stale_ns for token in tokens)):
                    detail = "feedback or publication exceeded the state freshness limit"
                if detail:
                    self._latch_fault(f"{label} feedback stalled for more than {self._state_stale_ns / 1e9:g}s "
                                      f"during collection: {detail}")
                    return
            for camera in self._cameras.values():
                frame = self._images.get(camera.name)
                if frame is None or now - frame.timestamp_ns > self._image_stale_ns:
                    self._latch_fault(f"camera {camera.name} SN={camera.serial} stalled during collection: "
                                      "publication exceeded the image freshness limit")
                    return

    # --------------------------------------------------------- RGB acquisition

    def _camera_loop(self, camera: _CameraState, factory):
        source = None
        try:
            source = factory(camera.name, camera.serial)
            with self._ready:
                camera.source = source
        except Exception as error:
            self._fail_camera(camera, f"{type(error).__name__}: {error}")
            return
        last_sequence = None
        progress_ns = time.monotonic_ns()
        detail = ""
        try:
            while not self._stop.is_set():
                try:
                    frame = source.read(self._camera_read_timeout_ms)
                except Exception as error:
                    detail = f"read failed: {type(error).__name__}: {error}"
                else:
                    if frame is None:
                        detail = "no frame within the read timeout"
                    else:
                        sequence, pixels = frame
                        sequence = int(sequence)
                        if sequence == last_sequence:
                            detail = f"duplicate frame sequence {sequence}"  # not a new sample
                        elif last_sequence is not None and sequence < last_sequence:
                            raise RuntimeError(
                                f"camera {camera.name} SN={camera.serial} frame sequence regressed: "
                                f"{sequence} after {last_sequence}")
                        elif (not isinstance(pixels, np.ndarray) or pixels.dtype != np.uint8
                                or pixels.shape != RGB_SHAPE):
                            raise RuntimeError(
                                f"camera {camera.name} SN={camera.serial} frame is not uint8 {RGB_SHAPE}")
                        else:
                            owned = np.array(pixels, dtype=np.uint8, order="C", copy=True)  # one copy of the SDK buffer
                            owned.setflags(write=False)  # queued/shared buffers stay immutable for every consumer
                            with self._ready:
                                stamp = time.monotonic_ns()  # availability at publication, never the SDK time
                                previous = self._images.get(camera.name)
                                if self._recording and previous is not None and stamp - previous.timestamp_ns > self._image_stale_ns:
                                    self._latch_fault(f"camera {camera.name} acquisition gap exceeded the image freshness limit")
                                self._images[camera.name] = ImageFrame(stamp, owned)
                                camera.first_frame = True
                                self._ready.notify_all()
                                self._writer.append_rgb(camera.name, stamp, owned)
                            last_sequence = sequence
                            progress_ns = stamp
                            detail = ""
                if time.monotonic_ns() - progress_ns > self._image_stale_ns:
                    raise RuntimeError(f"camera {camera.name} SN={camera.serial} stalled: {detail}")
        except Exception as error:
            self._fail_camera(camera, f"{type(error).__name__}: {error}")
        finally:
            if source is not None:
                try:
                    source.close()
                except Exception as error:
                    camera.close_error = f"{type(error).__name__}: {error}"

    def _fail_camera(self, camera: _CameraState, detail: str):
        if self._stop.is_set():
            return
        self._latch_fault(f"camera {camera.name} SN={camera.serial}: {detail}")

    # ------------------------------------------------------------- public state

    def _wait_ready(self):
        deadline = time.monotonic() + self._startup_timeout_s
        with self._ready:
            while True:
                if self._fault is not None:
                    raise RuntimeError(self._fault)
                missing = []
                if self._arms is None:
                    missing.append(f"arms state ({self._arms_detail or 'no sample yet'})")
                if self._hands is None:
                    missing.append(f"hands state ({self._hands_detail or 'no sample yet'})")
                for camera in self._cameras.values():
                    if not camera.first_frame:
                        missing.append(f"camera {camera.name} SN={camera.serial} (no image yet)")
                if not missing:
                    return
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError("timed out waiting for first samples: " + ", ".join(missing))
                self._ready.wait(remaining)

    def _wait_fresh(self):
        """Wait, bounded, until every recorded stream carries fresh input."""
        deadline = time.monotonic() + self._episode_fresh_timeout_s
        with self._ready:
            while True:
                now = time.monotonic_ns()
                stale = []
                if self._arms is None or now - self._arms_token > self._state_stale_ns:
                    stale.append("arms state")
                for side in ("left_hand", "right_hand"):
                    token = self._hand_tokens.get(side)
                    if token is None or now - token > self._state_stale_ns:
                        stale.append(f"{side} state")
                for camera in self._cameras.values():
                    frame = self._images.get(camera.name)
                    if frame is None or now - frame.timestamp_ns > self._image_stale_ns:
                        stale.append(f"camera {camera.name}")
                if not stale:
                    return
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError("collection runtime: refusing to start an episode with stale input: "
                                       + ", ".join(stale))
                self._ready.wait(remaining)

    def begin_episode(self, now_ns: int):
        """Open one recording segment at a host monotonic time.

        Segments repeat for as long as the runtime lives: persistence and
        sampling threads are never restarted, so a segment boundary is only a
        writer start/stop. Waits, bounded, for every recorded stream to carry
        fresh input so a segment never opens on stale samples and never opens
        the writer file when that cannot be satisfied; input from before the
        segment keeps its original availability stamps and is never re-stamped.
        A fault already reported during an earlier segment does not block a new
        segment once the streams are fresh again.
        """
        if not isinstance(now_ns, int) or isinstance(now_ns, bool) or now_ns <= 0:
            raise ValueError("collection episode start timestamp must be a positive host monotonic nanosecond int")
        with self._ready:
            if self._closed:
                raise RuntimeError("collection runtime is closed")
            if not self._started:
                raise RuntimeError("collection runtime must be started before an episode")
            if self._recording:
                raise RuntimeError("collection runtime is already recording an episode")
        self._wait_fresh()
        with self._ready:
            if self._stop.is_set() or self._closed:
                raise RuntimeError("collection runtime is stopping")
            for thread in (self._state_thread, *(c.thread for c in self._cameras.values())):
                if thread is not None and not thread.is_alive():
                    raise RuntimeError(f"cannot start recording: {thread.name} is not running")
            if self._arms_detail or self._hands_detail:
                raise RuntimeError("cannot start recording with invalid hardware feedback")
            self._writer.start(now_ns)
            self._fault = None
            self._episode_fault = None
            self._recording = True
            self._episode_started = True

    def end_episode(self) -> bool:
        """Close the current segment; the writer keeps every queued sample.

        Safe on shutdown and fault paths: with no segment open this is a no-op
        returning ``False``. Final acquisition health and the writer cutoff are
        captured under the publication lock; acquisition faults are frozen, not
        raised, so the caller can keep a failed segment's partial file.
        """
        with self._ready:
            if not self._recording:
                return False
            self._check_recording_faults()
            self._check_thread_faults()
            try:
                self._writer.stop()
            except Exception as error:
                self._latch_fault(f"episode stop failed: {type(error).__name__}: {error}")
                raise
            finally:
                self._recording = False
        return True

    @property
    def recording(self) -> bool:
        """True while a segment is open; a new segment may open once it is closed."""
        with self._ready:
            return self._recording

    @property
    def episode_started(self) -> bool:
        """True once any segment was opened; stays true across segments and close."""
        with self._ready:
            return self._episode_started

    @property
    def fault(self) -> str | None:
        """Latest acquisition failure, readable without raising into a control loop."""
        with self._ready:
            return self._fault

    def snapshot(self) -> RuntimeSnapshot:
        """Latest common input buffer with each value's original availability stamp."""
        with self._ready:
            return RuntimeSnapshot(arms=self._arms, hands=self._hands,
                                   images=MappingProxyType(dict(self._images)))
