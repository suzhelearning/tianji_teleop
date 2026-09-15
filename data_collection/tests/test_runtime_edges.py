import time
import threading

import pytest

from data_collection.tests.test_runtime import (FakeStateDevice, GatedDevice, Harness,
                                                ScriptedCamera, frame_rgb, wait_for)


class LiveReadDevice(FakeStateDevice):
    def read_feedback(self):
        self.publish()  # Like Marvin: a new host timestamp is assigned inside subscribe/read.
        return super().read_feedback()


def test_fresh_feedback_timestamped_during_read_is_not_rejected_as_future():
    harness = Harness(device_type=LiveReadDevice, startup_timeout_s=.2)
    try:
        runtime = harness.start()
        assert runtime.snapshot().arms is not None
        assert runtime.snapshot().hands is not None
    finally:
        harness.runtime.close()


def test_a_staged_hand_half_is_invalidated_by_a_new_unhealthy_read():
    harness = Harness()
    left = harness.devices['left_hand']
    right = harness.devices['right_hand']
    left.publish()
    staged = {}
    harness.runtime._publish_hands(left.read_feedback(), right.read_feedback(), staged, time.monotonic_ns())
    assert harness.runtime.snapshot().hands is None
    left.break_feedback('encoder fault')
    right.publish()
    harness.runtime._publish_hands(left.read_feedback(), right.read_feedback(), staged, time.monotonic_ns())
    assert harness.runtime.snapshot().hands is None


def test_new_segment_clears_old_fault_only_after_fresh_input_is_available():
    harness = Harness()
    runtime = harness.start()
    try:
        runtime.begin_episode(time.monotonic_ns())
        runtime._latch_fault('old segment fault')
        runtime.end_episode()
        with pytest.raises(RuntimeError, match='old segment fault'):
            runtime.check_episode()
        for device in harness.devices.values():
            device.publish()
        runtime.begin_episode(time.monotonic_ns())
        runtime.check()
        runtime.check_episode()
        assert runtime.fault is None
    finally:
        runtime.close()


def test_delayed_read_cannot_hide_a_recording_gap_by_returning_fresh_feedback():
    harness = Harness(device_type=GatedDevice, state_stale_s=.15)
    runtime = harness.start()
    arms = harness.devices['arms']
    try:
        runtime.begin_episode(time.monotonic_ns())
        before = runtime.snapshot().arms.timestamp_ns
        arms.gate.clear()
        time.sleep(.2)
        for device in harness.devices.values():
            device.publish()
        arms.gate.set()
        assert wait_for(lambda: runtime.snapshot().arms.timestamp_ns > before)
        assert runtime.fault is not None
        assert 'gap' in runtime.fault
    finally:
        arms.gate.set()
        runtime.close()


def test_check_detects_a_blocked_state_read_while_it_is_still_blocked():
    harness = Harness(device_type=GatedDevice, state_stale_s=.15)
    runtime = harness.start()
    arms = harness.devices['arms']
    try:
        runtime.begin_episode(time.monotonic_ns())
        arms.gate.clear()
        assert arms.blocked.wait(1)
        time.sleep(.2)
        with pytest.raises(RuntimeError, match='arms.*stalled'):
            runtime.check()
        assert arms.blocked.is_set()
        assert runtime.end_episode()
        with pytest.raises(RuntimeError, match='arms.*stalled'):
            runtime.check_episode()
    finally:
        arms.gate.set()
        runtime.close()


def test_stop_detects_a_stale_tail_without_an_intervening_check():
    harness = Harness(device_type=GatedDevice, state_stale_s=.15)
    runtime = harness.start()
    arms = harness.devices['arms']
    try:
        runtime.begin_episode(time.monotonic_ns())
        arms.gate.clear()
        assert arms.blocked.wait(1)
        time.sleep(.2)
        assert runtime.end_episode()  # never raises an acquisition error
        assert not runtime.recording
        assert arms.blocked.is_set()
        with pytest.raises(RuntimeError, match='arms.*stalled'):
            runtime.check_episode()
    finally:
        arms.gate.set()
        runtime.close()


class GatedCamera(ScriptedCamera):
    def __init__(self, name, serial):
        super().__init__(name, serial)
        self.gate = threading.Event()
        self.gate.set()
        self.blocked = threading.Event()

    def read(self, timeout_ms):
        if not self.gate.is_set():
            self.blocked.set()
        try:
            if not self.gate.wait(5):
                raise RuntimeError('gated camera read never released')
            return super().read(timeout_ms)
        finally:
            self.blocked.clear()


def test_check_detects_a_camera_read_that_ignores_its_timeout():
    harness = Harness(device_type=GatedDevice, image_stale_s=.15)
    harness.cameras = {name: GatedCamera(name, camera.serial) for name, camera in harness.cameras.items()}
    for camera in harness.cameras.values():
        camera.push(1, frame_rgb(1))
    top = harness.cameras['top']
    runtime = harness.start()
    arms = harness.devices['arms']
    try:
        runtime.begin_episode(time.monotonic_ns())
        arms.gate.clear()
        assert arms.blocked.wait(1)
        for camera in harness.cameras.values():
            camera.gate.clear()
        assert all(camera.blocked.wait(1) for camera in harness.cameras.values())
        time.sleep(.2)
        assert runtime.fault is None  # no acquisition thread can detect the stale camera
        with pytest.raises(RuntimeError, match='camera top.*stalled'):
            runtime.check()
        assert top.blocked.is_set()
        runtime.end_episode()
        with pytest.raises(RuntimeError, match='camera top.*stalled'):
            runtime.check_episode()
    finally:
        for camera in harness.cameras.values():
            camera.gate.set()
        arms.gate.set()
        runtime.close()


def test_post_stop_service_failure_does_not_invalidate_a_healthy_segment():
    harness = Harness()
    runtime = harness.start()
    top = harness.cameras['top']
    try:
        runtime.begin_episode(time.monotonic_ns())
        assert runtime.end_episode()
        top.push(0, frame_rgb(2))  # sequence regression kills the live camera service
        assert wait_for(lambda: runtime.fault is not None)
        with pytest.raises(RuntimeError, match='sequence regressed'):
            runtime.check()
        runtime.check_episode()
        assert not runtime.end_episode()  # repeated stop must not recapture the live fault
        runtime.check_episode()
    finally:
        runtime.close()


def test_blocked_read_after_stop_does_not_fault_the_stopped_segment():
    harness = Harness(device_type=GatedDevice, state_stale_s=.15)
    runtime = harness.start()
    arms = harness.devices['arms']
    try:
        runtime.begin_episode(time.monotonic_ns())
        assert runtime.end_episode()
        arms.gate.clear()
        assert arms.blocked.wait(1)
        time.sleep(.2)
        runtime.check()
        runtime.check_episode()
        assert runtime.fault is None
    finally:
        arms.gate.set()
        runtime.close()
