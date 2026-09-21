from dataclasses import replace
import importlib.util
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from pico2_hands.tests.test_pico_hand_tracking import _packet
from pico2_hands.reference.pico import parse_pico_packet, PICO_TO_MEDIAPIPE

from pico2_hands.resources import hand_retargeting_root


class OfficialPicoInputTest(unittest.TestCase):
    def frame(self):
        frame = parse_pico_packet(_packet(), receiver_instance_id='pico', connection_generation=3,
                                 receiver_frame_sequence=7, received_timestamp_ns=1000)
        hands = {}
        for side, hand in frame.hands.items():
            joints = []
            for i, joint in enumerate(hand.joints):
                # Synthetic nondegenerate 3-D geometry, not a physical sample.
                p = [.03*np.sin(i), .02*np.cos(i), .004*i]
                if side == 'left':
                    p[1] *= -1
                joints.append(replace(joint, pose=np.r_[p, [0, 0, 0, 1]]))
            hands[side] = replace(hand, joints=tuple(joints))
        return replace(frame, hands=hands)

    def convert(self, frame):
        self.assertIsNotNone(importlib.util.find_spec('pico2_hands.reference.official_pico'))
        from pico2_hands.reference.official_pico import pico_official_hand_observations
        return pico_official_hand_observations(frame)

    def test_selection_and_explicit_chirality_keep_source_metadata(self):
        frame = self.frame()
        observations = self.convert(frame)
        for side, row in observations.items():
            expected = np.array([frame.hands[side].joints[i].pose[:3] for i in PICO_TO_MEDIAPIPE])
            np.testing.assert_array_equal(row.keypoints_m, expected * [1, -1, 1])
            self.assertTrue(row.valid)
            self.assertEqual(row.source, 'pico2')
            self.assertEqual(row.coordinate_frame, 'pico_tracking_initial_y_reflected')
            self.assertEqual(row.frame_association_id, frame.association_id)
            self.assertEqual(row.source_timestamp_ns, frame.source_timestamp_ns)
            self.assertEqual(row.received_timestamp_ns, 1000)
            self.assertEqual(row.receiver_frame_sequence, 7)
            self.assertIsNone(row.wrist_pose)
        observations['left'].keypoints_m[:] = 0
        self.assertNotEqual(frame.hands['left'].joints[1].pose[2], 0)

    def test_head_loss_does_not_invalidate_hand_geometry(self):
        frame = replace(self.frame(), head_valid=False, flags=6)
        self.assertTrue(all(row.valid for row in self.convert(frame).values()))

    def test_official_hand2_retarget_input_scales_about_wrist_without_mutating_observation(self):
        from pico2_hands.reference.official_pico import (
            PICO_OFFICIAL_HAND2_GEOMETRY_SCALE,
            pico_official_hand2_retarget_input,
        )

        points = np.arange(63, dtype=np.float64).reshape(21, 3)
        original = points.copy()
        scaled = pico_official_hand2_retarget_input(points)

        np.testing.assert_array_equal(points, original)
        np.testing.assert_array_equal(scaled[0], points[0])
        np.testing.assert_allclose(
            scaled[1:],
            points[0] + (points[1:] - points[0]) * PICO_OFFICIAL_HAND2_GEOMETRY_SCALE,
        )
        self.assertIsNot(scaled, points)

    def test_missing_selected_joint_invalidates_only_that_side(self):
        frame = self.frame()
        hand = frame.hands['left']
        joints = list(hand.joints)
        joints[7] = replace(joints[7], valid=False)
        frame = replace(frame, hands=dict(frame.hands, left=replace(hand, joints=tuple(joints))))
        rows = self.convert(frame)
        self.assertFalse(rows['left'].valid)
        self.assertFalse(rows['left'].joint_valid[5])
        self.assertTrue(rows['right'].valid)

    def test_degenerate_palm_is_invalid_before_official_svd(self):
        frame = parse_pico_packet(_packet(), receiver_instance_id='pico', connection_generation=0,
                                 receiver_frame_sequence=0, received_timestamp_ns=1000)
        # _packet positions are collinear, despite all native validity bits.
        self.assertFalse(any(row.valid for row in self.convert(frame).values()))

    def test_pinned_official_preparation_is_rigid_invariant_and_not_mirrored(self):
        path = hand_retargeting_root() / 'wuji_retargeting/mediapipe.py'
        spec = importlib.util.spec_from_file_location('official_mediapipe_geometry', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        frame = self.frame()
        rows = self.convert(frame)
        rotation = Rotation.from_euler('xyz', [.7, -.3, 1.1])
        moved = replace(frame, hands={side: replace(hand, joints=tuple(
            replace(joint, pose=np.r_[rotation.apply(joint.pose[:3]) + [.4, -.2, 1.3], [0, 0, 0, 1]])
            for joint in hand.joints)) for side, hand in frame.hands.items()})
        for side in ('left', 'right'):
            native = np.array([frame.hands[side].joints[i].pose[:3] for i in PICO_TO_MEDIAPIPE])
            expected = module.apply_mediapipe_transformations(native, side)
            # Actual pinned Manus YAML reflects Z before the common wrist frame.
            for row in (rows[side], self.convert(moved)[side]):
                prepared = module.apply_mediapipe_transformations(row.keypoints_m * [1, 1, -1], side)
                np.testing.assert_allclose(prepared, expected, atol=1e-12, rtol=0)
