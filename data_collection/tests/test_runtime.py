"""Recorder acquisition semantics with fake devices, writers and camera sources.

Nothing here touches a robot, a camera or an SDK: every boundary is a
deterministic fake that the test drives explicitly.
"""
from pathlib import Path
from types import SimpleNamespace
import sys
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np

from data_collection.runtime import (ARMS_COUNT, HAND_COUNT, HEIGHT, WIDTH,
                                     CollectionRuntime, LockedDevice, validate_camera_profiles)

WINDOW_S = 5.0  # generous stale window so only the stream under test can fault


def wait_for(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.002)
    return False


def check_raises(runtime, timeout=1.0):
    """Poll ``check`` until it reports a fault; returns the message or ``None``."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            runtime.check()
        except RuntimeError as error:
            return str(error)
        time.sleep(0.002)
    return None


def frame_rgb(value=0):
    return np.full((HEIGHT, WIDTH, 3), value, dtype=np.uint8)


class FakeStateDevice:
    """Feedback source whose token changes only when the test publishes."""

    def __init__(self, count):
        self.count = count
        self._lock = threading.Lock()
        self._qpos = tuple(0.05 * (index + 1) for index in range(count))
        self._stamp = 0
        self._healthy = True
        self._detail = ""

    def publish(self, qpos=None, stamp=None):
        with self._lock:
            if qpos is not None:
                self._qpos = tuple(qpos)
            self._stamp = time.monotonic_ns() if stamp is None else int(stamp)
            return self._stamp

    def break_feedback(self, detail="device fault"):
        with self._lock:
            self._healthy = False
            self._detail = detail

    def read_feedback(self):
        with self._lock:
            return SimpleNamespace(position_rad=self._qpos, received_monotonic_ns=self._stamp,
                                   healthy=self._healthy, enabled=True, detail=self._detail)


class GatedDevice(FakeStateDevice):
    """Feedback source that can hold a read the way a long enable call holds the lock."""

    def __init__(self, count):
        super().__init__(count)
        self.gate = threading.Event()
        self.gate.set()
        self.blocked = threading.Event()

    def read_feedback(self):
        if not self.gate.is_set():
            self.blocked.set()
        try:
            if not self.gate.wait(5.0):
                raise RuntimeError("gated read never released")
            return super().read_feedback()
        finally:
            self.blocked.clear()


class TrackingDevice(FakeStateDevice):
    """Records overlapping entries so tests can prove the lock serializes calls."""

    def __init__(self, count):
        super().__init__(count)
        self.active = 0
        self.peak = 0
        self.reads = 0
        self.sends = 0

    def _enter(self):
        self.active += 1
        self.peak = max(self.peak, self.active)

    def read_feedback(self):
        self._enter()
        self.reads += 1
        try:
            time.sleep(0.002)
            return super().read_feedback()
        finally:
            self.active -= 1

    def send(self, positions):
        self._enter()
        self.sends += 1
        try:
            time.sleep(0.002)
        finally:
            self.active -= 1


class ScriptedCamera:
    """Camera source whose frames the test pushes explicitly."""

    def __init__(self, name, serial):
        self.name = name
        self.serial = serial
        self.condition = threading.Condition()
        self.frames = []
        self.closed = False
        self.failure = None

    def push(self, sequence, rgb):
        with self.condition:
            self.frames.append((sequence, rgb))
            self.condition.notify_all()

    def fail(self, message):
        with self.condition:
            self.failure = message
            self.condition.notify_all()

    def read(self, timeout_ms):
        deadline = time.monotonic() + timeout_ms / 1000
        with self.condition:
            while not self.frames and not self.closed and self.failure is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.condition.wait(remaining)
            if self.failure is not None:
                raise RuntimeError(self.failure)
            if self.frames:
                return self.frames.pop(0)
            raise RuntimeError("camera source closed")

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()


class AutoCamera(ScriptedCamera):
    """Always-fresh source: every read returns a new sequence with the same pixels."""

    def __init__(self, name, serial):
        super().__init__(name, serial)
        self.pixels = frame_rgb(6)
        self.sequence = 0

    def read(self, timeout_ms):
        self.sequence += 1
        return self.sequence, self.pixels


class FakeWriter:
    """Records what the runtime hands over; mirrors the writer's non-blocking API."""

    def __init__(self):
        self.lock = threading.Lock()
        self.started_ns = None
        self.stop_calls = 0
        self.check_calls = 0
        self.check_error = None
        self.arms = []
        self.hands = []
        self.images = []
        # The runtime must never finalize the file or the operator label.
        self.finish_calls = 0
        self.abort_calls = 0

    def start(self, now_ns):
        with self.lock:
            self.started_ns = now_ns

    def stop(self):
        with self.lock:
            self.stop_calls += 1

    def check(self):
        with self.lock:
            self.check_calls += 1
            if self.check_error is not None:
                raise RuntimeError(self.check_error)

    def _record(self, target, args):
        with self.lock:
            target.append(args)
        return True

    def append_arms(self, timestamp_ns, qpos):
        return self._record(self.arms, (timestamp_ns, qpos))

    def append_hands(self, timestamp_ns, qpos):
        return self._record(self.hands, (timestamp_ns, qpos))

    def append_rgb(self, name, timestamp_ns, rgb):
        return self._record(self.images, (name, timestamp_ns, rgb))

    def finish(self, success):
        self.finish_calls += 1

    def abort(self):
        self.abort_calls += 1


class Harness:
    """One runtime wired to fakes; camera frames are pushed before ``start``."""

    def __init__(self, *, device_type=FakeStateDevice, factory=None, frames=True, realsense=None, **options):
        self.devices = {name: device_type(count)
                        for name, count in (("arms", ARMS_COUNT), ("left_hand", HAND_COUNT),
                                            ("right_hand", HAND_COUNT))}
        self.locked = {name: LockedDevice(device) for name, device in self.devices.items()}
        self.writer = FakeWriter()
        self.cameras = {"top": ScriptedCamera("top", "SN-top"),
                        "left_wrist": ScriptedCamera("left_wrist", "SN-left")}
        settings = {"state_rate_hz": 200.0, "state_stale_s": WINDOW_S, "image_stale_s": WINDOW_S,
                    "startup_timeout_s": 2.0, "camera_read_timeout_ms": 20}
        if realsense is not None:
            settings["_realsense"] = realsense
        else:
            settings["_camera_factory"] = factory or (lambda name, serial: self.cameras[name])
        settings.update(options)
        self.runtime = CollectionRuntime(self.locked, {name: camera.serial for name, camera in self.cameras.items()},
                                         self.writer, **settings)
        if frames:
            for value, camera in enumerate(self.cameras.values(), start=1):
                camera.push(1, frame_rgb(value))

    def start(self):
        for device in self.devices.values():
            device.publish()
        self.runtime.start()
        return self.runtime


class TestLockedDevice(unittest.TestCase):
    def test_one_device_serializes_its_calls(self):
        device = TrackingDevice(ARMS_COUNT)
        locked = LockedDevice(device)

        def call():
            for _ in range(3):
                locked.read_feedback()

        threads = [threading.Thread(target=call) for _ in range(2)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(2)
        self.assertEqual(device.reads, 6)
        self.assertEqual(device.peak, 1)

    def test_separate_devices_use_separate_locks(self):
        entered = [threading.Event(), threading.Event()]

        class Paired:
            def __init__(self, index):
                self.index = index

            def read_feedback(self):
                entered[self.index].set()
                if not entered[1 - self.index].wait(1.0):
                    raise RuntimeError("independent devices were serialized together")
                return None

        proxies = [LockedDevice(Paired(0)), LockedDevice(Paired(1))]
        errors = []

        def call(proxy):
            try:
                proxy.read_feedback()
            except Exception as error:  # pragma: no cover - only on a locking bug
                errors.append(error)

        threads = [threading.Thread(target=call, args=(proxy,)) for proxy in proxies]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(2)
        self.assertEqual(errors, [])

    def test_runtime_sampling_never_overlaps_control_calls(self):
        harness = Harness(device_type=TrackingDevice, state_stale_s=WINDOW_S, image_stale_s=WINDOW_S)
        runtime = harness.start()
        stop = threading.Event()
        arms = harness.locked["arms"]

        def control():
            while not stop.is_set():
                arms.send(tuple(0.0 for _ in range(ARMS_COUNT)))

        thread = threading.Thread(target=control)
        thread.start()
        try:
            time.sleep(0.1)
        finally:
            stop.set()
            thread.join(2)
            runtime.close()
        device = harness.devices["arms"]
        self.assertEqual(device.peak, 1)


class TestStateSampling(unittest.TestCase):
    def test_duplicates_are_not_new_samples_and_a_stall_faults(self):
        harness = Harness(state_stale_s=0.15)
        runtime, writer = harness.runtime, harness.writer
        harness.start()
        try:
            runtime.begin_episode(time.monotonic_ns())
            self.assertTrue(wait_for(lambda: writer.arms))
            samples = len(writer.arms)
            stamp = runtime.snapshot().arms.timestamp_ns
            time.sleep(0.05)  # duplicates inside the stale window are tolerated
            runtime.check()
            self.assertEqual(len(writer.arms), samples)
            self.assertEqual(runtime.snapshot().arms.timestamp_ns, stamp)
            message = check_raises(runtime)
            self.assertIsNotNone(message)
            self.assertIn("arms feedback stalled", message)
            self.assertIn("during collection", message)
        finally:
            runtime.close()

    def test_hands_need_a_new_token_from_both_hands(self):
        harness = Harness()
        runtime, writer = harness.runtime, harness.writer
        harness.start()
        try:
            self.assertTrue(wait_for(lambda: writer.hands))
            self.assertEqual(len(writer.hands), 1)
            first = runtime.snapshot().hands
            left_values = tuple(-1.0 - index for index in range(HAND_COUNT))
            harness.devices["left_hand"].publish(qpos=left_values)
            time.sleep(0.05)  # one hand alone must not produce a paired snapshot
            self.assertEqual(len(writer.hands), 1)
            self.assertIs(runtime.snapshot().hands, first)
            right_values = tuple(0.5 + index for index in range(HAND_COUNT))
            published_ns = time.monotonic_ns()
            token = harness.devices["right_hand"].publish(qpos=right_values)
            self.assertTrue(wait_for(lambda: len(writer.hands) == 2))
            stamp, qpos = writer.hands[-1]
            self.assertEqual(list(qpos[:HAND_COUNT]), list(left_values))
            self.assertEqual(list(qpos[HAND_COUNT:]), list(right_values))
            self.assertEqual(stamp, runtime.snapshot().hands.timestamp_ns)
            self.assertGreaterEqual(stamp, published_ns)
            self.assertGreater(stamp, token)  # host availability, not the driver token
        finally:
            runtime.close()

    def test_unhealthy_feedback_ends_the_episode(self):
        harness = Harness(state_stale_s=0.15)
        runtime = harness.runtime
        harness.start()
        try:
            runtime.begin_episode(time.monotonic_ns())
            harness.devices["left_hand"].break_feedback("hand servo error")
            message = check_raises(runtime)
            self.assertIsNotNone(message)
            self.assertIn("hands feedback stalled", message)
            self.assertIn("hand servo error", message)
        finally:
            runtime.close()

    def test_blocked_device_wait_does_not_fault_a_pending_episode(self):
        harness = Harness(device_type=GatedDevice, state_stale_s=0.15, episode_fresh_timeout_s=1.0)
        runtime, writer = harness.runtime, harness.writer
        harness.start()
        try:
            arms = harness.devices["arms"]
            arms.gate.clear()  # as if enable or stop held the device lock
            time.sleep(0.3)    # far beyond the stale window
            runtime.check()    # a blocked poll before the episode is not a capture fault
            for device in harness.devices.values():
                device.publish()
            arms.gate.set()
            runtime.begin_episode(time.monotonic_ns())
            self.assertTrue(runtime.episode_started)
            self.assertIsNotNone(writer.started_ns)
            runtime.end_episode()
        finally:
            runtime.close()

    def test_begin_episode_refuses_stale_input_without_locking_the_session(self):
        harness = Harness(state_stale_s=0.15, episode_fresh_timeout_s=0.2)
        runtime, writer = harness.runtime, harness.writer
        harness.start()
        try:
            time.sleep(0.3)  # every state stream stopped advancing
            with self.assertRaises(RuntimeError) as caught:
                runtime.begin_episode(time.monotonic_ns())
            message = str(caught.exception)
            self.assertIn("stale input", message)
            self.assertIn("arms state", message)
            self.assertIn("left_hand state", message)
            self.assertFalse(runtime.episode_started)
            self.assertIsNone(writer.started_ns)  # the writer never opened an episode
            runtime.check()                       # the refusal is not a permanent fault
            for device in harness.devices.values():
                device.publish()
            runtime.begin_episode(time.monotonic_ns())
            self.assertTrue(runtime.episode_started)
            runtime.end_episode()
        finally:
            runtime.close()

    def test_arms_and_hands_publish_separate_streams(self):
        harness = Harness()
        runtime, writer = harness.runtime, harness.writer
        harness.start()
        try:
            snapshot = runtime.snapshot()
            self.assertEqual(len(snapshot.arms.qpos), ARMS_COUNT)
            self.assertEqual(len(snapshot.hands.qpos), HAND_COUNT * 2)
            self.assertEqual(writer.arms[0][1], snapshot.arms.qpos)
            self.assertEqual(writer.hands[0][1], snapshot.hands.qpos)
            self.assertTrue(wait_for(lambda: len(writer.images) == 2))
            self.assertEqual(sorted(entry[0] for entry in writer.images), ["left_wrist", "top"])
        finally:
            runtime.close()


class TestCameraAcquisition(unittest.TestCase):
    def test_frames_are_deduplicated_copied_once_and_host_stamped(self):
        harness = Harness()
        runtime, writer, top = harness.runtime, harness.writer, harness.cameras["top"]
        harness.start()
        try:
            self.assertTrue(wait_for(lambda: any(entry[0] == "top" for entry in writer.images)))
            first = [entry for entry in writer.images if entry[0] == "top"]
            self.assertEqual(len(first), 1)
            stamp, rgb = first[0][1], first[0][2]
            self.assertEqual(rgb.shape, (HEIGHT, WIDTH, 3))
            self.assertEqual(rgb.dtype, np.uint8)
            frame = runtime.snapshot().images["top"]
            self.assertEqual(frame.timestamp_ns, stamp)
            self.assertIs(frame.rgb, rgb)  # one owned buffer shared, never recopied
            before = time.monotonic_ns()
            pushed = frame_rgb(9)
            top.push(7, pushed)
            self.assertTrue(wait_for(lambda: len([e for e in writer.images if e[0] == "top"]) == 2))
            after = time.monotonic_ns()
            entries = [entry for entry in writer.images if entry[0] == "top"]
            stamp2, rgb2 = entries[-1][1], entries[-1][2]
            self.assertGreaterEqual(stamp2, before)
            self.assertLessEqual(stamp2, after)
            self.assertIsNot(rgb2, pushed)  # copied once out of the SDK buffer
            pushed[:] = 0
            self.assertTrue(np.array_equal(rgb2, frame_rgb(9)))
            top.push(7, frame_rgb(3))  # an already seen sequence is not a new sample
            time.sleep(0.03)
            self.assertEqual(len([entry for entry in writer.images if entry[0] == "top"]), 2)
            self.assertEqual(runtime.snapshot().images["top"].timestamp_ns, stamp2)
        finally:
            runtime.close()

    def test_frame_sequence_regression_faults(self):
        harness = Harness()
        runtime, top = harness.runtime, harness.cameras["top"]
        harness.start()
        try:
            top.push(2, frame_rgb(4))
            self.assertTrue(wait_for(lambda: runtime.snapshot().images.get("top")))
            top.push(1, frame_rgb(5))
            message = check_raises(runtime)
            self.assertIsNotNone(message)
            self.assertIn("sequence regressed", message)
        finally:
            runtime.close()

    def test_stalled_camera_is_a_visible_fault(self):
        harness = Harness(image_stale_s=0.05)
        runtime = harness.runtime
        harness.start()
        try:
            message = check_raises(runtime)
            self.assertIsNotNone(message)
            self.assertIn("camera", message)
            self.assertIn("stalled", message)
        finally:
            runtime.close()

    def test_camera_read_failure_is_a_visible_fault(self):
        auto = AutoCamera("left_wrist", "SN-left")

        def factory(name, serial):
            return harness.cameras[name] if name == "top" else auto

        harness = Harness(factory=factory, image_stale_s=0.05)
        runtime, top = harness.runtime, harness.cameras["top"]
        harness.start()
        try:
            # Only the failing camera can fault: the other one always has frames.
            top.fail("USB link dropped")
            message = check_raises(runtime)
            self.assertIsNotNone(message)
            self.assertIn("USB link dropped", message)
        finally:
            runtime.close()

    def test_camera_startup_failure_cleans_up_partial_startup(self):
        def factory(name, serial):
            if name == "left_wrist":
                raise RuntimeError("device busy")
            return harness.cameras[name]

        harness = Harness(factory=factory)
        with self.assertRaises(RuntimeError) as caught:
            harness.runtime.start()
        message = str(caught.exception)
        self.assertIn("left_wrist", message)
        self.assertIn("device busy", message)
        self.assertTrue(harness.cameras["top"].closed)  # partially started camera released
        self.assertIn("left_wrist", check_raises(harness.runtime))
        harness.runtime.close()
        self.assertEqual(harness.writer.stop_calls, 1)
        self.assertEqual((harness.writer.finish_calls, harness.writer.abort_calls), (0, 0))

    def test_startup_requires_first_state_and_images(self):
        harness = Harness(startup_timeout_s=0.3)
        with self.assertRaises(RuntimeError) as caught:
            harness.runtime.start()
        message = str(caught.exception)
        self.assertIn("timed out waiting for first samples", message)
        self.assertIn("arms state", message)
        self.assertIn("hands state", message)
        self.assertIn("left_hand", message)
        harness.runtime.close()
        self.assertTrue(all(camera.closed for camera in harness.cameras.values()))


class TestLifecycle(unittest.TestCase):
    def test_episode_lifecycle_keeps_previous_availability_stamps(self):
        harness = Harness()
        runtime, writer = harness.runtime, harness.writer
        harness.start()
        try:
            arms_before = runtime.snapshot().arms.timestamp_ns
            self.assertFalse(runtime.recording)
            self.assertFalse(runtime.episode_started)
            self.assertFalse(runtime.end_episode())  # no segment open: a no-op, not an error
            self.assertEqual(writer.stop_calls, 0)
            start_ns = time.monotonic_ns()
            runtime.begin_episode(start_ns)
            self.assertTrue(runtime.recording)
            self.assertTrue(runtime.episode_started)
            self.assertEqual(writer.started_ns, start_ns)
            self.assertEqual(runtime.snapshot().arms.timestamp_ns, arms_before)  # never zeroed
            with self.assertRaises(RuntimeError):
                runtime.begin_episode(start_ns + 1)
            harness.devices["arms"].publish()
            self.assertTrue(wait_for(lambda: len(writer.arms) > 1))
            self.assertTrue(runtime.end_episode())
            self.assertFalse(runtime.recording)
            self.assertTrue(runtime.episode_started)  # the session did record a segment
            self.assertEqual(writer.stop_calls, 1)
            self.assertFalse(runtime.end_episode())  # repeated stop requests stay harmless
            self.assertEqual(writer.stop_calls, 1)
        finally:
            runtime.close()

    def test_segments_repeat_without_restarting_acquisition(self):
        harness = Harness()
        runtime, writer = harness.runtime, harness.writer
        harness.start()
        try:
            runtime.begin_episode(time.monotonic_ns())
            before = len(writer.arms)
            harness.devices["arms"].publish()
            self.assertTrue(wait_for(lambda: len(writer.arms) > before))
            self.assertTrue(runtime.end_episode())
            self.assertFalse(runtime.recording)
            # Between segments acquisition keeps running and keeps its own stamps.
            held = runtime.snapshot().arms.timestamp_ns
            harness.devices["arms"].publish()
            self.assertTrue(wait_for(lambda: runtime.snapshot().arms.timestamp_ns > held))
            between = runtime.snapshot().arms.timestamp_ns
            second_start = time.monotonic_ns()
            runtime.begin_episode(second_start)
            self.assertTrue(runtime.recording)
            self.assertEqual(writer.started_ns, second_start)
            self.assertEqual(runtime.snapshot().arms.timestamp_ns, between)  # never re-stamped
            before = len(writer.arms)
            harness.devices["arms"].publish()
            self.assertTrue(wait_for(lambda: len(writer.arms) > before))
            self.assertTrue(runtime.end_episode())
            self.assertEqual(writer.stop_calls, 2)
        finally:
            runtime.close()

    def test_episode_requires_a_started_runtime(self):
        harness = Harness(frames=False)
        with self.assertRaises(RuntimeError):
            harness.runtime.begin_episode(time.monotonic_ns())
        self.assertFalse(harness.runtime.episode_started)
        with self.assertRaises(ValueError):
            harness.runtime.begin_episode("now")

    def test_writer_failure_is_visible_and_close_never_finalizes(self):
        harness = Harness()
        runtime, writer = harness.runtime, harness.writer
        harness.start()
        try:
            writer.check_error = "capture queue overflow"
            with self.assertRaises(RuntimeError) as caught:
                runtime.check()
            self.assertIn("capture queue overflow", str(caught.exception))
            writer.check_error = None
            runtime.check()  # the failure was the writer's, not a permanent runtime fault
            harness.devices["arms"].publish()
            self.assertTrue(wait_for(lambda: len(writer.arms) > 1))
        finally:
            runtime.close()
        self.assertEqual(writer.stop_calls, 1)
        self.assertEqual((writer.finish_calls, writer.abort_calls), (0, 0))

    def test_close_is_idempotent_and_releases_every_camera(self):
        harness = Harness()
        runtime, writer = harness.runtime, harness.writer
        harness.start()
        runtime.close()
        self.assertTrue(all(camera.closed for camera in harness.cameras.values()))
        runtime.close()
        self.assertEqual(writer.stop_calls, 1)

    def test_snapshot_before_start_is_empty(self):
        runtime = Harness(frames=False).runtime
        snapshot = runtime.snapshot()
        self.assertIsNone(snapshot.arms)
        self.assertIsNone(snapshot.hands)
        self.assertEqual(dict(snapshot.images), {})


# --------------------------------------------------- profile preflight (fake SDK)

SERIAL_INFO, NAME_INFO, USB_INFO = object(), object(), object()
COLOR_STREAM = object()


class FakeFormat:
    """Stands in for the pyrealsense2 stream format enum (``str`` is its name)."""

    def __init__(self, name):
        self.name = name

    def __str__(self):
        return self.name

    def __repr__(self):
        return self.name


RGB8 = FakeFormat("rgb8")
BGR8 = FakeFormat("bgr8")


class FakeVideoProfile:
    def __init__(self, fmt, width, height, fps):
        self._mode = (fmt, width, height, fps)

    def format(self):
        return self._mode[0]

    def width(self):
        return self._mode[1]

    def height(self):
        return self._mode[2]

    def fps(self):
        return self._mode[3]


class FakeStreamProfile:
    def __init__(self, video):
        self._video = video

    def stream_type(self):
        return COLOR_STREAM

    def as_video_stream_profile(self):
        return self._video


class FakeRealSenseDevice:
    def __init__(self, serial, product, usb_type, modes):
        self.serial, self.product, self.usb_type = serial, product, usb_type
        self.modes = modes
        self.sensor_queries = 0

    def get_info(self, key):
        return {SERIAL_INFO: self.serial, NAME_INFO: self.product, USB_INFO: self.usb_type}[key]

    def query_sensors(self):
        self.sensor_queries += 1
        return [SimpleNamespace(get_stream_profiles=lambda: [FakeStreamProfile(FakeVideoProfile(*mode))
                                                             for mode in self.modes])]


def fake_realsense(devices):
    rs = SimpleNamespace(devices=devices)
    rs.camera_info = SimpleNamespace(serial_number=SERIAL_INFO, name=NAME_INFO, usb_type_descriptor=USB_INFO)
    rs.stream = SimpleNamespace(color=COLOR_STREAM)
    rs.format = SimpleNamespace(rgb8=RGB8, bgr8=BGR8)
    rs.context = lambda: SimpleNamespace(query_devices=lambda: list(devices))
    return rs


class TestProfilePreflight(unittest.TestCase):
    def test_exact_mode_is_accepted_without_touching_other_devices(self):
        selected = FakeRealSenseDevice("SN-top", "Intel RealSense D435I", "3.2",
                                       [(BGR8, 640, 480, 30), (RGB8, WIDTH, HEIGHT, 30)])
        other = FakeRealSenseDevice("SN-other", "Intel RealSense D455", "3.2", [(RGB8, WIDTH, HEIGHT, 30)])
        rs = fake_realsense([selected, other])
        profiles = validate_camera_profiles({"top": "SN-top"}, realsense=rs)
        self.assertEqual(sorted(profiles), ["top"])
        self.assertEqual(profiles["top"].serial, "SN-top")
        self.assertEqual(profiles["top"].product, "Intel RealSense D435I")
        self.assertEqual(profiles["top"].usb_type, "3.2")
        self.assertEqual(profiles["top"].stream, f"rgb8 {WIDTH}x{HEIGHT}@30")
        self.assertEqual(other.sensor_queries, 0)  # only the selected serial is enumerated

    def test_missing_device_reports_serial_and_attached_list(self):
        rs = fake_realsense([FakeRealSenseDevice("SN-other", "Intel RealSense D455", "3.2",
                                                 [(RGB8, WIDTH, HEIGHT, 30)])])
        with self.assertRaises(RuntimeError) as caught:
            validate_camera_profiles({"top": "SN-top"}, realsense=rs)
        message = str(caught.exception)
        self.assertIn("top SN=SN-top", message)
        self.assertIn("not attached", message)
        self.assertIn("SN-other", message)

    def test_wrong_modes_are_rejected_with_usb_and_available_modes(self):
        device = FakeRealSenseDevice("SN-top", "Intel RealSense D435I", "2.1",
                                     [(BGR8, WIDTH, HEIGHT, 30), (RGB8, 640, 480, 30)])
        rs = fake_realsense([device])
        with self.assertRaises(RuntimeError) as caught:
            validate_camera_profiles({"top": "SN-top"}, realsense=rs)
        message = str(caught.exception)
        self.assertIn(f"RGB8 {WIDTH}x{HEIGHT}@30", message)
        self.assertIn("USB 2.1", message)
        self.assertIn("bgr8 1280x720@30", message)
        self.assertIn("rgb8 640x480@30", message)

    def test_selection_is_validated_before_any_device_is_read(self):
        rs = fake_realsense([])
        for cameras in ({}, {"top": ""}, {"top": "SN-1", "left_wrist": "SN-1"}, [("top", "SN-1")]):
            with self.assertRaises(ValueError):
                validate_camera_profiles(cameras, realsense=rs)


class TestRealSenseModeGuard(unittest.TestCase):
    def test_runtime_rejects_a_different_started_mode(self):
        class FakePipeline:
            def __init__(self):
                self.stopped = False

            def start(self, config):
                return SimpleNamespace(
                    get_stream=lambda stream: SimpleNamespace(
                        as_video_stream_profile=lambda: FakeVideoProfile(BGR8, WIDTH, HEIGHT, 30)))

            def stop(self):
                self.stopped = True

            def wait_for_frames(self, timeout_ms):
                raise RuntimeError("no frames")

        pipes = []

        def pipeline_factory(context):
            pipes.append(FakePipeline())
            return pipes[-1]

        rs = SimpleNamespace(config=lambda: SimpleNamespace(enable_device=lambda serial: None,
                                                            enable_stream=lambda *args: None),
                             pipeline=pipeline_factory,
                             stream=SimpleNamespace(color=COLOR_STREAM),
                             format=SimpleNamespace(rgb8=RGB8, bgr8=BGR8),
                             context=lambda: SimpleNamespace(query_devices=lambda: []))
        harness = Harness(realsense=rs)
        with self.assertRaises(RuntimeError) as caught:
            harness.runtime.start()
        message = str(caught.exception)
        self.assertIn("instead of", message)
        self.assertIn(f"RGB8 {WIDTH}x{HEIGHT}@30", message)
        self.assertTrue(pipes and all(pipe.stopped for pipe in pipes))
        harness.runtime.close()


if __name__ == "__main__":
    unittest.main()
