"""Observable latest-value handoff; no SDK, ROS context, or hardware access."""
from dataclasses import replace
import unittest

from tianji_runtime import Feedback
from tianji_controller.feedback import FeedbackHub


class Device:
    def __init__(self, value):
        self.value = value

    def read_feedback(self):
        if isinstance(self.value, Exception):
            raise self.value
        return self.value


class FeedbackTests(unittest.TestCase):
    def test_stationary_new_samples_and_health_changes_keep_device_stamp(self):
        device = Device(Feedback((0.0,) * 14, 100, True, False))
        sampler = FeedbackHub({"arms": device}, ("arms",))
        sampler._sample_once()
        self.assertEqual(sampler.drain()["arms"].received_monotonic_ns, 100)
        sampler._sample_once()
        self.assertIsNone(sampler.drain())
        device.value = replace(device.value, received_monotonic_ns=200)
        sampler._sample_once()
        self.assertEqual(sampler.drain()["arms"].received_monotonic_ns, 200)
        device.value = replace(device.value, healthy=False, detail="SDK lost connection")
        sampler._sample_once()
        value = sampler.drain()["arms"]
        self.assertFalse(value.healthy)
        self.assertEqual(value.received_monotonic_ns, 200)

    def test_slow_consumer_preserves_other_device_and_read_failure_revokes_health(self):
        left = Device(Feedback((0.0,) * 20, 100, True, False))
        right = Device(Feedback((0.0,) * 20, 110, True, False))
        sampler = FeedbackHub({"left_hand": left, "right_hand": right},
                              ("left_hand", "right_hand"))
        sampler._sample_once()
        left.value = replace(left.value, received_monotonic_ns=200)
        sampler._sample_once()
        latest = sampler.drain()
        self.assertEqual(latest["left_hand"].received_monotonic_ns, 200)
        self.assertEqual(latest["right_hand"].received_monotonic_ns, 110)
        self.assertIsNone(sampler.drain())
        right.value = RuntimeError("read unavailable")
        sampler._sample_once()
        latest = sampler.drain()
        self.assertFalse(latest["right_hand"].healthy)
        self.assertEqual(latest["right_hand"].received_monotonic_ns, 110)
        self.assertEqual(latest["left_hand"].received_monotonic_ns, 200)
