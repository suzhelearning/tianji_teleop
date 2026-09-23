"""Frame reuse preserves source aging, calibration gates, and mapped geometry."""
from dataclasses import replace
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from pico2_hands.shared_root_core import SharedRootCore
from pico2_hands.shared_root_mapping import SharedRootMapping
from pico2_hands.tests.test_dropout_recovery import FakeDls
from pico2_hands.tests.test_shared_root import calibrate, fk, frame


class FrameCacheTests(unittest.TestCase):
    def core(self):
        core = SharedRootCore(np.zeros(54), FakeDls(), height_m=1.62)
        calibrate(core.mapping)
        return core

    def test_reused_frame_ages_and_rejected_frames_cannot_refresh_it(self):
        core = self.core()
        stamp = 2_020_000_000
        sample = frame(stamp=stamp)
        self.assertTrue(core.offer(sample, stamp))
        target = core.mapping.targets(sample).copy()
        self.assertTrue(core.fresh(stamp, 45_000_000))
        self.assertTrue(core.action("s", stamp))
        core.tick(stamp)
        self.assertEqual(core.phase, "TELEOP")
        self.assertTrue(core.fresh(stamp + 45_000_000, 45_000_000))
        now = stamp + 45_000_001
        duplicate = replace(sample, received_timestamp_ns=now, head_valid=False)
        self.assertFalse(core.offer(duplicate, now))
        newer_sequence_old_source = replace(
            sample, receiver_frame_sequence=sample.receiver_frame_sequence + 1,
            received_timestamp_ns=now)
        self.assertFalse(core.offer(newer_sequence_old_source, now))
        newer_source_old_sequence = replace(
            sample, source_timestamp_ms=sample.source_timestamp_ms + 1,
            received_timestamp_ns=now)
        self.assertFalse(core.offer(newer_source_old_sequence, now))
        np.testing.assert_allclose(core.mapping.targets(core.frame), target, atol=1e-12)
        self.assertFalse(core.fresh(now, 45_000_000))
        self.assertFalse(core.fresh(stamp - 1, 45_000_000))
        core.tick(now)
        self.assertEqual(core.phase, "BRAKING")
        self.assertEqual(core.braking_reason, "stale_input")

    def test_invalid_sample_cannot_reuse_previous_valid_geometry(self):
        core = self.core()
        stamp = 2_020_000_000
        sample = frame(stamp=stamp)
        core.offer(sample, stamp)
        expected = core.mapping.targets(sample).copy()
        self.assertTrue(core.fresh(stamp))
        for index, invalid_kind in enumerate(("head", "wrist", "middle"), 1):
            now = stamp + index * 10_000_000
            invalid = frame(stamp=now)
            if invalid_kind == "head":
                invalid = replace(invalid, head_valid=False)
            else:
                hands = dict(invalid.hands)
                hand = hands["left"]
                if invalid_kind == "wrist":
                    hand = replace(hand, wrist_valid=False)
                else:
                    joints = list(hand.joints)
                    joints[12] = replace(joints[12], valid=False)
                    hand = replace(hand, joints=tuple(joints))
                hands["left"] = hand
                invalid = replace(invalid, hands=hands)
            self.assertTrue(core.offer(invalid, now))
            for delay in (0, 1_000_000):
                self.assertFalse(core.fresh(now + delay), invalid_kind)
                with self.assertRaises(ValueError):
                    core.mapping.targets(core.frame)
        now += 10_000_000
        restored = frame(stamp=now)
        self.assertTrue(core.offer(restored, now))
        self.assertTrue(core.fresh(now))
        np.testing.assert_allclose(core.mapping.targets(restored), expected, atol=1e-12)

    def test_disconnect_and_new_identity_revoke_cached_calibration(self):
        for identity_change in ("disconnect", "generation", "receiver"):
            with self.subTest(identity_change=identity_change):
                core = self.core()
                stamp = 2_020_000_000
                sample = frame(stamp=stamp)
                core.offer(sample, stamp)
                core.mapping.targets(sample)
                self.assertTrue(core.fresh(stamp))
                if identity_change == "disconnect":
                    core.disconnected()
                    self.assertFalse(core.fresh(stamp))
                    sample = frame(stamp=stamp + 10_000_000)
                elif identity_change == "generation":
                    sample = frame(stamp=stamp, generation=2)
                else:
                    sample = replace(sample, receiver_instance_id="new-receiver")
                self.assertTrue(core.offer(sample, sample.received_timestamp_ns))
                self.assertTrue(core.fresh(sample.received_timestamp_ns))
                self.assertFalse(core.action("s", sample.received_timestamp_ns))
                with self.assertRaises(ValueError):
                    core.mapping.targets(sample)

    def test_recalibration_remaps_same_frame_and_failure_revokes_targets(self):
        mapping = SharedRootMapping(1.62, fk)
        calibrate(mapping)
        start = 4_000_000_000
        probe = frame(stamp=start + 1_000_000_000, move=(.03, 0, -.02))
        before = mapping.targets(probe).copy()
        self.assertTrue(mapping.request_calibration(start, idle=True))
        with self.assertRaises(ValueError):
            mapping.targets(probe)
        for index in range(51):
            now = start + index * 20_000_000
            sample = probe if index == 50 else frame(stamp=now, move=(.03, 0, -.02))
            mapping.offer_frame(sample, now)
            mapping.tick(now)
        self.assertTrue(mapping.calibration_allows_start)
        after = mapping.targets(probe)
        expected_shift = mapping.robot_rotation @ (mapping.scale * np.array([.03, 0, -.02]) * mapping.height)
        np.testing.assert_allclose(before[:, :3] - after[:, :3],
                                   np.broadcast_to(expected_shift, (2, 3)), atol=1e-12)
        np.testing.assert_allclose(before[:, 3:], after[:, 3:], atol=1e-12)
        self.assertTrue(mapping.request_calibration(now, idle=True))
        mapping.tick(now + 250_000_001)
        self.assertFalse(mapping.calibration_allows_start)
        with self.assertRaises(ValueError):
            mapping.targets(probe)

    def test_solution_replacement_remaps_already_cached_frame(self):
        mapping = SharedRootMapping(1.62, fk)
        calibrate(mapping)
        sample = frame(stamp=3_000_000_000)
        before = mapping.targets(sample).copy()
        basis, offset, corrections = mapping.solution
        delta = np.array([.02, -.01, .03])
        mapping.solution = (basis, offset + delta, corrections)
        after = mapping.targets(sample)
        expected_shift = mapping.robot_rotation @ (mapping.scale * (basis.T @ delta))
        np.testing.assert_allclose(before[:, :3] - after[:, :3],
                                   np.broadcast_to(expected_shift, (2, 3)), atol=1e-12)
        np.testing.assert_allclose(before[:, 3:], after[:, 3:], atol=1e-12)

    def test_cached_targets_match_shared_root_equation_and_ignore_head_rotation(self):
        height, yaw = 1.85, .7
        mapping = SharedRootMapping(height, fk)
        calibrate(mapping, height, yaw)
        move = np.array([-.05, .02, -.10])
        sample = frame(height, 3_000_000_000, yaw, move=move)
        local_palms = np.array([[mapping.reach, mapping.width / 2, 0],
                                [mapping.reach, -mapping.width / 2, 0]])
        expected_positions = mapping.origin + ((local_palms + move * height) * mapping.scale) @ mapping.robot_rotation.T
        for _ in range(3):
            targets = mapping.targets(sample)
            np.testing.assert_allclose(targets[:, :3], expected_positions, atol=1e-12)
            np.testing.assert_allclose(Rotation.from_quat(targets[:, 3:]).as_matrix(),
                                       np.broadcast_to(np.eye(3), (2, 3, 3)), atol=1e-12)
        turned_head = replace(sample, head_pose=np.r_[sample.head_pose[:3],
                               Rotation.from_euler("xyz", [.3, .2, -.8]).as_quat()])
        np.testing.assert_allclose(mapping.targets(turned_head), targets, atol=1e-12)
        invalid_same_association = replace(sample, head_valid=False)
        with self.assertRaises(ValueError):
            mapping.targets(invalid_same_association)


if __name__ == "__main__":
    unittest.main()
