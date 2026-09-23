"""Producer readiness policy; synthetic frames and no ROS/hardware workers."""
from dataclasses import replace
import unittest
from unittest.mock import patch

import numpy as np

from pico2_hands.shared_root_core import SharedRootCore
from pico2_hands.tests.test_dropout_recovery import FakeDls
from pico2_hands.tests.test_shared_root import frame


class MetadataDls(FakeDls):
    def __init__(self):
        super().__init__()
        self.epoch = 0
        self.reject = False

    def reset(self, q):
        self.epoch += 1

    def solve(self, q, targets, **kwargs):
        if self.reject:
            self.command(6, q, kwargs["now"])
            return self.result(q, accepted=False)
        return super().solve(q, targets, **kwargs)


class SyntheticHand:
    def __init__(self):
        self.pending = False

    def submit(self, points, sequence, timestamp_ns):
        if self.pending:
            raise RuntimeError("hand request already pending")
        self.pending = True

    def receive(self):
        if not self.pending:
            raise RuntimeError("no pending hand request")
        self.pending = False
        return np.full(20, .1)


class SpdOutputTests(unittest.TestCase):
    def setUp(self):
        self.now = 1_000_000_000
        self.worker = MetadataDls()
        self.core = SharedRootCore(np.zeros(54), self.worker,
                                   {s: SyntheticHand() for s in ("left", "right")},
                                   height_m=1.62)

    def sample(self, *, invalid_side=None, generation=1):
        sample = frame(stamp=self.now, generation=generation)
        if invalid_side is not None:
            hands = dict(sample.hands)
            hand = hands[invalid_side]
            joints = list(hand.joints)
            joints[25] = replace(joints[25], valid=False)
            hands[invalid_side] = replace(hand, joints=tuple(joints))
            sample = replace(sample, hands=hands)
        self.core.offer(sample, self.now)
        return sample

    def calibrate(self):
        self.assertTrue(self.core.action("c", self.now))
        for _ in range(51):
            self.sample()
            self.core.tick(self.now)
            self.now += 20_000_000
        self.assertTrue(self.core.mapping.calibration_allows_start)

    def start(self):
        self.calibrate()
        self.sample()
        self.assertTrue(self.core.action("s", self.now))
        self.core.tick(self.now)
        self.assertEqual(self.core.spd_output(self.now)[0], 7)

    def settle(self):
        for _ in range(25):
            self.now += 5_000_000
            self.core.tick(self.now)

    def test_calibration_endorses_home_without_start_and_disabled_hands_stay_off(self):
        self.assertEqual(self.core.spd_output(self.now)[0], 0)
        self.core.hands.pop("left")
        self.calibrate()
        self.assertEqual(self.core.phase, "WAITING")
        self.assertEqual(self.core.spd_output(self.now)[0], 3)
        self.now += 5_000_000_000
        self.core.tick(self.now)
        self.assertEqual(self.core.spd_output(self.now)[0], 3)

    def test_failed_recalibration_revokes_session_immediately(self):
        self.calibrate()
        previous = self.core.spd_output(self.now)[1]
        self.assertTrue(self.core.action("c", self.now))
        mask, current = self.core.spd_output(self.now)
        self.assertEqual(mask, 0)
        self.assertNotEqual(current, previous)
        self.now += 300_000_000
        self.core.tick(self.now)
        self.assertEqual(self.core.mapping.state, "failed")
        self.assertEqual(self.core.spd_output(self.now), (0, current))

    def test_failed_peer_never_commits_half_a_hand_frame(self):
        self.start()
        previous = self.core.q[14:].copy()
        self.now += 5_000_000
        self.sample()
        with patch.object(self.core.hands["left"], "receive", return_value=np.full(20, .25)), \
                patch.object(self.core.hands["right"], "receive", side_effect=TimeoutError("peer stalled")):
            with self.assertRaises(TimeoutError):
                self.core.tick(self.now)
        np.testing.assert_array_equal(self.core.q[14:], previous)
        self.assertEqual(self.core.spd_output(self.now)[0] & 6, 0)

    def test_invalid_hand_does_not_revalidate_from_other_side_or_offer_only(self):
        self.start()
        self.now += 5_000_000
        self.sample(invalid_side="left")
        self.core.tick(self.now)
        self.assertEqual(self.core.spd_output(self.now)[0], 3)
        self.assertTrue(self.core.action("p", self.now))
        self.now += 5_000_000
        self.sample()
        self.core.tick(self.now)
        self.assertEqual(self.core.spd_output(self.now)[0], 3)
        self.settle()
        self.assertEqual(self.core.spd_output(self.now)[0], 3)
        self.now += 5_000_000
        self.sample()
        self.assertTrue(self.core.action("s", self.now))
        self.core.tick(self.now)
        self.assertEqual(self.core.spd_output(self.now)[0], 7)

    def test_deliberate_pause_home_and_exit_preserve_known_fingers_without_input(self):
        for key in ("p", "h", "q"):
            with self.subTest(key=key):
                self.setUp()
                self.start()
                session = self.core.spd_output(self.now)[1]
                fingers = self.core.q[14:].copy()
                self.assertTrue(self.core.action(key, self.now))
                self.settle()
                self.now += 5_000_000_000
                self.core.tick(self.now)
                self.assertEqual(self.core.spd_output(self.now), (7, session))
                np.testing.assert_array_equal(self.core.q[14:], fingers)

    def test_completion_overrun_and_late_pause_cannot_refresh_tracking(self):
        self.start()
        completion = self.now + 46_000_000
        self.assertEqual(self.core.spd_output(completion)[0], 0)
        self.assertEqual(self.core.spd_output(completion + 1)[0], 0)
        self.assertTrue(self.core.action("p", completion))
        self.assertEqual(self.core.spd_output(completion)[0], 1)

    def test_stale_braking_remains_valid_but_unintended_settled_hold_does_not(self):
        self.start()
        self.now += 60_000_000
        self.core.tick(self.now)
        self.assertEqual(self.core.phase, "BRAKING")
        self.assertEqual(self.core.spd_output(self.now)[0], 1)
        self.settle()
        self.assertEqual(self.core.phase, "HOLD")
        self.assertEqual(self.core.spd_output(self.now)[0], 0)

    def test_fresh_frames_do_not_refresh_unretargeted_hand_during_recovery(self):
        self.start()
        for n in range(25):
            self.now += 5_000_000
            sample = frame(stamp=self.now)
            hands = dict(sample.hands)
            hands["left"] = replace(hands["left"], wrist_valid=False)
            self.core.offer(replace(sample, hands=hands), self.now)
            self.core.tick(self.now)
            if n == 0:
                self.assertEqual(self.core.spd_output(self.now)[0], 3)
            elif n == 9:
                self.assertEqual(self.core.spd_output(self.now)[0], 1)
        self.assertEqual(self.core.phase, "HOLD")
        self.assertEqual(self.core.spd_output(self.now)[0], 0)
        # A later explicit P can endorse stopped arms, not expired fingers.
        self.assertTrue(self.core.action("p", self.now))
        self.assertEqual(self.core.spd_output(self.now)[0], 1)

    def test_rejected_tracking_keeps_bounded_braking_not_fault_or_settled_output(self):
        self.start()
        self.worker.reject = True
        self.now += 5_000_000
        self.sample()
        self.core.tick(self.now)
        self.assertEqual(self.core.phase, "BRAKING")
        self.assertEqual(self.core.spd_output(self.now)[0], 1)
        self.settle()
        self.assertEqual(self.core.spd_output(self.now)[0], 0)
        self.worker.phase = "FAULT"
        self.core._consume(self.worker.result(self.core.q[:14].reshape(2, 7)))
        self.assertEqual(self.core.spd_output(self.now)[0], 0)

    def test_disconnect_latches_once_and_reconnection_epoch_never_reuses_session(self):
        self.start()
        old = self.core.spd_output(self.now)[1]
        self.core.disconnected()
        disconnected = self.core.spd_output(self.now)[1]
        self.assertNotEqual(old, disconnected)
        self.core.disconnected()
        self.assertEqual(self.core.spd_output(self.now)[1], disconnected)
        self.core.tick(self.now)
        self.assertEqual(self.core.spd_output(self.now)[0], 1)
        self.now += 5_000_000
        self.sample(generation=2)
        connected = self.core.spd_output(self.now)[1]
        self.assertNotEqual(connected, disconnected)
        self.settle()
        self.assertEqual(self.core.spd_output(self.now)[0], 0)
        self.core.disconnected()
        self.assertNotEqual(self.core.spd_output(self.now)[1], connected)
        before_reset = self.core.spd_output(self.now)[1]
        self.worker.reset(self.core.q[:14].reshape(2, 7))
        self.assertNotEqual(self.core.spd_output(self.now)[1], before_reset)
