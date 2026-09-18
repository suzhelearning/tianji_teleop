from dataclasses import replace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

from pico2_hands.mapping import OptionalHeightMapping
from pico2_hands.tests.test_pose_mapping import _input


class OptionalCalibrationTest(unittest.TestCase):
    def test_invalid_model_fails_without_authorizing_start(self):
        mapping = OptionalHeightMapping()
        with patch("pico2_hands.mapping.horizontal_reference", side_effect=ET.ParseError("bad XML")):
            self.assertFalse(mapping.request_calibration(1_000_000_000, idle=True))
        self.assertFalse(mapping.calibration_allows_start)
        self.assertEqual(mapping.status()["state"], "failed")

    def test_wrong_coordinate_frame_cannot_enter_calibration(self):
        mapping = OptionalHeightMapping()
        mapping.request_calibration(1_000_000_000, idle=True)
        observation = replace(self.sample("left", 1_000_000_000), reference_frame="world")
        with self.assertRaises(ValueError):
            mapping.add(observation, 1_000_000_000)
        self.assertFalse(mapping.calibration_allows_start)

    def sample(self, side, now, x=.4):
        return replace(_input([x, .2, -.3, 0, 0, 0, 1], side=side),
                       received_timestamp_ns=now)

    def collect(self, mapping, start, moving=False):
        for i in range(101):
            now = start + i * 20_000_000
            for side in ("left", "right"):
                mapping.add(self.sample(side, now, .4 + (.002*i if moving else 0)), now)
            mapping.tick(now)

    def test_never_pressed_c_keeps_fixed_mapping_and_permits_gate(self):
        mapping = OptionalHeightMapping()
        self.assertTrue(mapping.calibration_allows_start)
        self.assertTrue(mapping.map(self.sample("left", 1)).valid)
        self.assertFalse(mapping.status()["motion_authorized"])
        self.assertEqual(mapping.status()["state"], "uncalibrated")

    def test_collecting_and_first_failure_prevent_start(self):
        mapping = OptionalHeightMapping()
        self.assertFalse(mapping.request_calibration(1_000_000_000, idle=False))
        self.assertTrue(mapping.calibration_allows_start)
        self.assertTrue(mapping.request_calibration(1_000_000_000, idle=True))
        self.assertFalse(mapping.calibration_allows_start)
        self.assertFalse(mapping.request_calibration(1_000_000_001, idle=True))
        mapping.tick(1_300_000_000)
        self.assertEqual(mapping.status()["state"], "failed")
        self.assertFalse(mapping.calibration_allows_start)
        self.assertTrue(mapping.request_calibration(2_000_000_000, idle=True))
        self.collect(mapping, 2_000_000_000)
        self.assertTrue(mapping.calibration_allows_start)

    def test_calibration_changes_only_world_height_not_xy_or_orientation(self):
        mapping = OptionalHeightMapping()
        samples = {s: self.sample(s, 1_000_000_000) for s in ("left", "right")}
        before = {s: mapping.map(v).pose for s, v in samples.items()}
        mapping.request_calibration(1_000_000_000, idle=True)
        self.collect(mapping, 1_000_000_000)
        self.assertEqual(mapping.status()["state"], "calibrated")
        for side, sample in samples.items():
            after = mapping.map(sample).pose
            reference = mapping.reference[side]
            # Source mapper uses the configured vertical axis. URDF RPY is
            # rounded, so its FK axis differs by a few microradians; do not
            # change source mathematics to satisfy an exact-axis assertion.
            up = mapping.mapper._rotations[side][:, 2]
            delta = after[:3] - before[side][:3]
            np.testing.assert_allclose(delta - up * (up @ delta), 0, atol=1e-12)
            np.testing.assert_allclose(after[3:], before[side][3:], atol=1e-12)
            center = after[:3] + Rotation.from_quat(after[3:]).apply([0, 0, .0365])
            self.assertAlmostEqual(up @ center, up @ reference["control_position_base_m"])

    def test_failed_recalibration_preserves_previous_valid_result(self):
        mapping = OptionalHeightMapping()
        mapping.request_calibration(1_000_000_000, idle=True)
        self.collect(mapping, 1_000_000_000)
        before = mapping.map(self.sample("left", 1)).pose.copy()
        mapping.request_calibration(4_000_000_000, idle=True)
        self.collect(mapping, 4_000_000_000, moving=True)
        self.assertEqual(mapping.status()["state"], "failed")
        self.assertTrue(mapping.calibration_allows_start)
        np.testing.assert_array_equal(mapping.map(self.sample("left", 1)).pose, before)

    def test_duplicate_receipts_cannot_finish_calibration(self):
        mapping = OptionalHeightMapping()
        mapping.request_calibration(1_000_000_000, idle=True)
        for i in range(101):
            now = 1_000_000_000 + i * 20_000_000
            for side in ("left", "right"):
                mapping.add(self.sample(side, 1_000_000_000), now)
            mapping.tick(now)
        self.assertEqual(mapping.status()["state"], "failed")
