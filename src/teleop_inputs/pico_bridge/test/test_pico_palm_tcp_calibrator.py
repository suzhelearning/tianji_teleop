#!/usr/bin/env python3
import math
import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
import pico_palm_tcp_calibrator as calibrator_module  # noqa: E402
from pico_palm_orientation_core import (  # noqa: E402
    OrientationCaptureBuffer,
    OrientationGates,
    OrientationSolution,
    gravity_leveled_heading_rotation,
)
from pico_palm_tcp_calibrator import (  # noqa: E402
    PalmTcpCalibrator,
    build_tcp_artifact,
    default_output,
    quaternion_values,
    quaternion_from_rotation,
    solve_tcp_rotation_for_gravity_leveled_hmd_heading,
    rotation_from_xyzw,
    solve_reference_tcp_position,
    tcp_orientation_prompt,
    tcp_position_prompt,
    translation_values,
)


class PalmTcpCalibratorTest(unittest.TestCase):
    def test_operator_stage_prompts_are_chinese(self):
        position = tcp_position_prompt("left", 4)
        orientation = tcp_orientation_prompt("left")

        self.assertIn("左侧", position)
        self.assertIn("按空格采集 4 次", position)
        self.assertIn("双臂向正前方水平伸直", orientation)
        self.assertIn("左右掌心相对", orientation)
        self.assertIn("启动左侧 TCP 姿态采集", orientation)
        self.assertIn("保持约 1～2 秒", orientation)
        self.assertNotIn("Orientation stage", orientation)

    def test_tcp_rotation_uses_gravity_leveled_hmd_heading(self):
        controller = rotation_from_xyzw(np.array([0.0, 0.0, 0.2, 0.98]))
        head_yaw = rotation_from_xyzw(np.array([0.0, 0.0, 0.35, 0.94]))
        head_pitch = rotation_from_xyzw(np.array([0.0, 0.1, 0.0, 0.995]))
        head = head_yaw @ head_pitch
        tcp = solve_tcp_rotation_for_gravity_leveled_hmd_heading(controller, head)
        result = controller @ tcp
        expected = gravity_leveled_heading_rotation(head)
        np.testing.assert_allclose(result, expected, atol=1e-2)

    def test_tcp_artifact_v2_has_explicit_semantics_and_lineage(self):
        solution = OrientationSolution(
            rotation=np.eye(3),
            covariance=np.eye(3) * 1.0e-5,
            sample_count=120,
            tracking_epoch=7,
            orientation_rms_rad=0.01,
            correction_angle_rad=0.02,
        )
        document = build_tcp_artifact(
            side="left",
            source_topic="/pico/pose/left_hand",
            translation=np.array([0.1, -0.02, 0.03]),
            rotation=np.eye(3),
            sample_count=20,
            position_rms=0.003,
            calibration_revision=4,
            sample_matrix_rank=6,
            sample_matrix_condition=12.0,
            orientation_solution=solution,
        )
        self.assertEqual(document["schema_version"], 2)
        self.assertEqual(document["pose_semantics"], "controller_pose")
        self.assertEqual(document["orientation_reference"], "gravity_leveled_hmd_heading")
        self.assertNotIn("pico_pelvis_yaw", document.values())
        self.assertEqual(len(document["covariance_upper_triangle_6x6"]), 21)
        self.assertTrue(document["lineage"])
        self.assertEqual(document["quality"]["sample_matrix_rank"], 6)
        self.assertEqual(document["quality"]["position_sample_count"], 20)
        self.assertEqual(document["quality"]["orientation_sample_count"], 120)
        self.assertTrue(document["quality"]["orientation_capture_separate"])
        self.assertEqual(document["translation_revision"], 4)
        self.assertEqual(document["orientation_revision"], 1)
        self.assertEqual(len(document["translation_fingerprint_sha256"]), 64)
        self.assertEqual(document["orientation_only_ancestor_sha256"], [])
        self.assertFalse(hasattr(calibrator_module, "solve_tcp_rotation_for_pelvis_yaw"))

    def test_tcp_artifact_records_multi_frame_orientation_quality_and_covariance(self):
        orientation_covariance = np.array([
            [1.0e-4, 2.0e-5, 3.0e-5],
            [2.0e-5, 4.0e-4, 5.0e-5],
            [3.0e-5, 5.0e-5, 9.0e-4],
        ])
        solution = OrientationSolution(
            rotation=np.eye(3),
            covariance=orientation_covariance,
            sample_count=120,
            tracking_epoch=7,
            orientation_rms_rad=0.012,
            correction_angle_rad=0.08,
        )

        document = build_tcp_artifact(
            side="left",
            source_topic="/pico/pose/left_hand",
            translation=np.array([0.1, -0.02, 0.03]),
            rotation=solution.rotation,
            sample_count=4,
            position_rms=0.003,
            calibration_revision=4,
            sample_matrix_rank=6,
            sample_matrix_condition=12.0,
            orientation_solution=solution,
        )

        self.assertEqual(document["quality"]["orientation_sample_count"], 120)
        self.assertEqual(document["orientation_calibration"]["tracking_epoch"], 7)
        self.assertEqual(
            document["orientation_calibration"]["method"],
            "gravity_leveled_hmd_heading_bilateral_forward_palms_facing",
        )
        self.assertAlmostEqual(
            document["orientation_calibration"]["orientation_rms_rad"], 0.012
        )
        covariance = np.zeros((6, 6))
        cursor = 0
        for row in range(6):
            for column in range(row, 6):
                covariance[row, column] = document["covariance_upper_triangle_6x6"][cursor]
                covariance[column, row] = covariance[row, column]
                cursor += 1
        np.testing.assert_allclose(covariance[3:6, 3:6], orientation_covariance)

    def test_tcp_artifact_rejects_missing_multi_frame_orientation_solution(self):
        with self.assertRaisesRegex(ValueError, "orientation_solution"):
            build_tcp_artifact(
                side="left",
                source_topic="/pico/pose/left_hand",
                translation=np.array([0.1, -0.02, 0.03]),
                rotation=np.eye(3),
                sample_count=4,
                position_rms=0.003,
                calibration_revision=4,
                sample_matrix_rank=6,
                sample_matrix_condition=12.0,
                orientation_solution=None,
            )

    def test_palm_output_keeps_controller_source_stamp(self):
        class Stamp:
            sec = 12
            nanosec = 34

        class Publisher:
            def __init__(self):
                self.message = None

            def publish(self, message):
                self.message = message

        class PoseStamped:
            def __init__(self):
                self.header = type("Header", (), {})()
                self.pose = type("Pose", (), {})()
                self.pose.position = type("Position", (), {})()
                self.pose.orientation = type("Orientation", (), {})()

        calibrator = object.__new__(PalmTcpCalibrator)
        calibrator._pose_stamped_type = PoseStamped
        calibrator._tcp_position = np.zeros(3)
        calibrator._tcp_rotation = np.eye(3)
        calibrator._orientation_calibrated = True
        calibrator._lock = __import__("threading").Lock()
        calibrator._latest_controller = (np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]), time.monotonic(), Stamp())
        calibrator.publisher = Publisher()
        calibrator._publish_pose()
        self.assertIs(calibrator.publisher.message.header.stamp, calibrator._latest_controller[3])

    def test_quaternion_values_accepts_yaml_mapping(self):
        values = quaternion_values({"x": 0.1, "y": -0.2, "z": 0.3, "w": 0.9})
        np.testing.assert_allclose(values, [0.1, -0.2, 0.3, 0.9])

    def test_translation_values_accepts_schema_v2_sequence(self):
        values = translation_values({"translation_m": [0.1, -0.2, 0.3]})
        np.testing.assert_allclose(values, [0.1, -0.2, 0.3])

    def test_translation_values_accepts_legacy_mapping(self):
        values = translation_values({"translation_m": {"x": 0.1, "y": -0.2, "z": 0.3}})
        np.testing.assert_allclose(values, [0.1, -0.2, 0.3])

    def test_side_specific_default_output(self):
        self.assertTrue(default_output("left").endswith("pico_left_palm_tcp.yaml"))
        self.assertTrue(default_output("right").endswith("pico_right_palm_tcp.yaml"))

    def test_reference_position_solver_recovers_controller_offset(self):
        tcp = np.array([0.12, -0.04, 0.08])
        fixed_world = np.array([0.7, -0.2, 1.1])
        angles = ((0.0, 0.0), (0.4, 0.0), (0.0, -0.5), (0.3, 0.4))
        samples = []
        for yaw, pitch in angles:
            cy, sy = math.cos(yaw), math.sin(yaw)
            cp, sp = math.cos(pitch), math.sin(pitch)
            rotation = np.array([
                [cy * cp, -sy, cy * sp],
                [sy * cp, cy, sy * sp],
                [-sp, 0.0, cp],
            ])
            samples.append((rotation, fixed_world - rotation @ tcp))

        estimate, residual = solve_reference_tcp_position(samples)

        np.testing.assert_allclose(estimate, tcp, atol=1e-9)
        self.assertLess(residual, 1e-9)

    def test_reference_position_solver_rejects_identical_pose_samples(self):
        samples = [(np.eye(3), np.array([0.2, 0.1, 1.0])) for _ in range(4)]

        with self.assertRaisesRegex(ValueError, "姿态变化不足"):
            solve_reference_tcp_position(samples)

    def test_quaternion_to_rotation_rejects_zero(self):
        with self.assertRaises(ValueError):
            rotation_from_xyzw(np.zeros(4))

    def test_rotation_quaternion_round_trip(self):
        rotation = rotation_from_xyzw(np.array([0.2, -0.3, 0.1, 0.9]))
        quaternion = quaternion_from_rotation(rotation)
        np.testing.assert_allclose(rotation_from_xyzw(quaternion), rotation, atol=1e-9)

    def _two_stage_calibrator(self, samples):
        shutdown_calls = []
        save_calls = []
        calibrator = object.__new__(PalmTcpCalibrator)
        calibrator._lock = threading.Lock()
        calibrator._latest_controller = (
            samples[-1][1],
            quaternion_from_rotation(samples[-1][0]),
            time.monotonic(),
            object(),
        )
        calibrator._latest_head = (np.array([0.0, 0.0, 0.0, 1.0]), object())
        calibrator._samples = samples[:-1]
        calibrator.sample_count = 4
        calibrator._stage = "position"
        calibrator._tcp_position = None
        calibrator._tcp_rotation = np.eye(3)
        calibrator._orientation_calibrated = False
        calibrator._orientation_capture = OrientationCaptureBuffer()
        calibrator._orientation_gates = OrientationGates(min_samples=4)
        calibrator._orientation_capture_timeout_s = 5.0
        calibrator._orientation_capture_active = False
        calibrator._orientation_capture_deadline = 0.0
        calibrator._orientation_solution = None
        calibrator._tracking_epoch = 7
        calibrator._tracking_epoch_source = "tcp_connection"
        calibrator.side = "left"
        calibrator.controller_topic = "/pico/pose/left_hand"
        calibrator._stop_keyboard = threading.Event()
        calibrator.rclpy = SimpleNamespace(shutdown=lambda: shutdown_calls.append(True))
        calibrator.node = SimpleNamespace(
            get_logger=lambda: SimpleNamespace(
                info=lambda *_: None, warning=lambda *_: None, error=lambda *_: None
            )
        )
        calibrator._save = lambda: save_calls.append(True)
        return calibrator, save_calls, shutdown_calls

    def test_fourth_sample_accepts_position_but_waits_for_orientation(self):
        fixed_world = np.array([0.7, -0.2, 1.1])
        tcp = np.array([0.12, -0.04, 0.08])
        samples = []
        for yaw, pitch in ((0.0, 0.0), (0.4, 0.0), (0.0, -0.5), (0.3, 0.4)):
            cy, sy = math.cos(yaw), math.sin(yaw)
            cp, sp = math.cos(pitch), math.sin(pitch)
            rotation = np.array([
                [cy * cp, -sy, cy * sp],
                [sy * cp, cy, sy * sp],
                [-sp, 0.0, cp],
            ])
            samples.append((rotation, fixed_world - rotation @ tcp, np.eye(3)))

        calibrator, save_calls, shutdown_calls = self._two_stage_calibrator(samples)

        calibrator.capture()

        self.assertEqual(calibrator._stage, "orientation")
        np.testing.assert_allclose(calibrator._tcp_position, tcp, atol=1e-9)
        self.assertFalse(calibrator._orientation_calibrated)
        self.assertFalse(calibrator._stop_keyboard.is_set())
        self.assertEqual(save_calls, [])
        self.assertEqual(shutdown_calls, [])

    def test_fifth_space_starts_multi_frame_orientation_capture_then_saves(self):
        fixed_world = np.array([0.7, -0.2, 1.1])
        tcp = np.array([0.12, -0.04, 0.08])
        samples = []
        for yaw, pitch in ((0.0, 0.0), (0.4, 0.0), (0.0, -0.5), (0.3, 0.4)):
            cy, sy = math.cos(yaw), math.sin(yaw)
            cp, sp = math.cos(pitch), math.sin(pitch)
            rotation = np.array([
                [cy * cp, -sy, cy * sp],
                [sy * cp, cy, sy * sp],
                [-sp, 0.0, cp],
            ])
            samples.append((rotation, fixed_world - rotation @ tcp, np.eye(3)))
        calibrator, save_calls, shutdown_calls = self._two_stage_calibrator(samples)
        calibrator.capture()
        accepted_position = calibrator._tcp_position.copy()

        controller_rotation = np.eye(3)
        yaw = 0.2
        pitch = 0.1
        head_rotation = np.array([
            [math.cos(yaw) * math.cos(pitch), -math.sin(yaw), math.cos(yaw) * math.sin(pitch)],
            [math.sin(yaw) * math.cos(pitch), math.cos(yaw), math.sin(yaw) * math.sin(pitch)],
            [-math.sin(pitch), 0.0, math.cos(pitch)],
        ])
        stamp = SimpleNamespace(sec=1, nanosec=0)
        calibrator._latest_controller = (
            np.array([9.0, 8.0, 7.0]),
            quaternion_from_rotation(controller_rotation),
            time.monotonic(),
            stamp,
        )
        calibrator._latest_head = (quaternion_from_rotation(head_rotation), stamp)

        calibrator.capture()

        self.assertTrue(calibrator._orientation_capture_active)
        self.assertEqual(save_calls, [])
        self.assertEqual(shutdown_calls, [])
        for index in range(4):
            sample_stamp = 1_000_000_000 + index
            calibrator._orientation_capture.add_head(head_rotation, sample_stamp)
            self.assertTrue(
                calibrator._orientation_capture.add_controller(
                    controller_rotation, sample_stamp, 7
                )
            )
        calibrator._finalize_orientation_capture()

        np.testing.assert_allclose(calibrator._tcp_position, accepted_position, atol=0.0)
        np.testing.assert_allclose(
            calibrator._tcp_rotation,
            gravity_leveled_heading_rotation(head_rotation),
            atol=1e-9,
        )
        self.assertTrue(calibrator._orientation_calibrated)
        self.assertTrue(calibrator._stop_keyboard.is_set())
        self.assertEqual(save_calls, [True])
        self.assertEqual(shutdown_calls, [True])


if __name__ == "__main__":
    unittest.main()
