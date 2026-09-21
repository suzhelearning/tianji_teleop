#!/usr/bin/env python3
import math
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from pico_palm_wrist_calibrator import (  # noqa: E402
    PalmWristCalibrator,
    palm_sample_from_message,
    solve_wrist_pivot,
    wrist_calibration_prompt,
    wrist_position_from_palm_pose,
)


class PalmWristPivotTest(unittest.TestCase):
    def test_operator_capture_prompt_is_chinese(self):
        prompt = wrist_calibration_prompt("left", 450, "yaw")

        self.assertIn("左侧", prompt)
        self.assertIn("按空格开始采集 450 个样本", prompt)
        self.assertIn("保持手腕不动", prompt)
        self.assertIn("缓慢左右转动手掌", prompt)
        self.assertIn("按 q 取消", prompt)
        self.assertNotIn("Press SPACE", prompt)

    def test_yaw_only_recovers_planar_offset_and_zeroes_normal_offset(self):
        wrist = np.array([0.4, -0.2, 0.9])
        offset = np.array([0.08, 0.02, 0.0])
        samples = []
        for yaw in (0.0, 0.5, -0.7, 1.0, -1.1, 0.8):
            rotation = np.array([
                [math.cos(yaw), -math.sin(yaw), 0.0],
                [math.sin(yaw), math.cos(yaw), 0.0],
                [0.0, 0.0, 1.0],
            ])
            samples.append((wrist + rotation @ offset, rotation))
        estimate, residual, condition = solve_wrist_pivot(samples, motion="yaw")
        np.testing.assert_allclose(estimate.wrist_world, wrist, atol=1e-9)
        np.testing.assert_allclose(estimate.wrist_to_palm, offset, atol=1e-9)
        self.assertLess(residual, 1e-9)
        self.assertLess(condition, 100.0)

    def test_recovers_wrist_and_palm_offset_from_multi_axis_motion(self):
        wrist = np.array([0.4, -0.2, 0.9])
        offset = np.array([0.08, 0.02, 0.06])
        samples = []
        for yaw, pitch in ((0.0, 0.0), (0.6, 0.0), (-0.5, 0.0), (0.0, 0.45), (0.0, -0.4), (0.3, 0.25)):
            cy, sy = math.cos(yaw), math.sin(yaw)
            cp, sp = math.cos(pitch), math.sin(pitch)
            rotation = np.array([
                [cy * cp, -sy, cy * sp],
                [sy * cp, cy, sy * sp],
                [-sp, 0.0, cp],
            ])
            samples.append((wrist + rotation @ offset, rotation))
        estimate, residual, condition = solve_wrist_pivot(samples, motion="full")
        np.testing.assert_allclose(estimate.wrist_world, wrist, atol=1e-9)
        np.testing.assert_allclose(estimate.wrist_to_palm, offset, atol=1e-9)
        self.assertLess(residual, 1e-9)
        self.assertLess(condition, 100.0)

    def test_runtime_wrist_position_uses_palm_rotation(self):
        palm = np.array([1.0, 2.0, 3.0])
        offset = np.array([0.1, 0.0, 0.0])
        rotation = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        np.testing.assert_allclose(
            wrist_position_from_palm_pose(palm, rotation, offset),
            [1.0, 1.9, 3.0],
            atol=1e-9,
        )

    def test_solver_rejects_nonfinite_position_and_rotation(self):
        rotation = np.eye(3)
        samples = [(np.zeros(3), rotation) for _ in range(4)]
        samples[0] = (np.array([np.nan, 0.0, 0.0]), rotation)
        with self.assertRaisesRegex(ValueError, "finite"):
            solve_wrist_pivot(samples)

        samples = [(np.zeros(3), rotation) for _ in range(4)]
        invalid_rotation = rotation.copy()
        invalid_rotation[0, 0] = np.inf
        samples[0] = (np.zeros(3), invalid_rotation)
        with self.assertRaisesRegex(ValueError, "finite"):
            solve_wrist_pivot(samples)

    def test_palm_sample_requires_canonical_frame_and_finite_pose(self):
        message = SimpleNamespace(
            header=SimpleNamespace(frame_id="pico"),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )
        position, rotation, quaternion = palm_sample_from_message(message)
        np.testing.assert_allclose(position, [1.0, 2.0, 3.0])
        np.testing.assert_allclose(rotation, np.eye(3))
        np.testing.assert_allclose(quaternion, [0.0, 0.0, 0.0, 1.0])

        message.header.frame_id = "world"
        with self.assertRaisesRegex(ValueError, "frame"):
            palm_sample_from_message(message)
        message.header.frame_id = "pico"
        message.pose.position.x = math.nan
        with self.assertRaisesRegex(ValueError, "finite"):
            palm_sample_from_message(message)

    def test_epoch_transition_aborts_and_clears_active_capture(self):
        calibrator = object.__new__(PalmWristCalibrator)
        calibrator._lock = threading.Lock()
        calibrator._tracking_epoch = 4
        calibrator._tracking_epoch_numeric = 4
        calibrator._tracking_epoch_source = "tcp_connection"
        calibrator._capturing = True
        calibrator._samples = [(np.zeros(3), np.eye(3))]

        calibrator._epoch_status_callback(
            SimpleNamespace(
                data='{"tracking_epoch":5,"tracking_epoch_source":"wire_world_reset"}'
            )
        )

        self.assertFalse(calibrator._capturing)
        self.assertEqual(calibrator._samples, [])
        self.assertEqual(calibrator._tracking_epoch, 5)

    def test_successful_wrist_solution_exits_without_allowing_overwrite(self):
        wrist = np.array([0.4, -0.2, 0.9])
        offset = np.array([0.08, 0.02, 0.0])
        samples = []
        for yaw in np.linspace(-1.0, 1.0, 120):
            rotation = np.array([
                [math.cos(yaw), -math.sin(yaw), 0.0],
                [math.sin(yaw), math.cos(yaw), 0.0],
                [0.0, 0.0, 1.0],
            ])
            samples.append((wrist + rotation @ offset, rotation))

        calibrator = object.__new__(PalmWristCalibrator)
        calibrator.motion = "yaw"
        calibrator._stop_keyboard = threading.Event()
        calibrator.rclpy = SimpleNamespace(
            shutdown=lambda: self.fail("subscription callback must not shut down rclpy")
        )
        calibrator.node = SimpleNamespace(
            get_logger=lambda: SimpleNamespace(info=lambda *_: None, error=lambda *_: None)
        )
        calibrator._save = lambda *_: None

        calibrator._finish_calibration(samples)

        self.assertTrue(calibrator._stop_keyboard.is_set())

    def test_near_zero_wrist_to_palm_distance_is_rejected(self):
        wrist = np.array([0.4, -0.2, 0.9])
        samples = []
        for yaw in np.linspace(-1.0, 1.0, 120):
            rotation = np.array([
                [math.cos(yaw), -math.sin(yaw), 0.0],
                [math.sin(yaw), math.cos(yaw), 0.0],
                [0.0, 0.0, 1.0],
            ])
            samples.append((wrist + rotation @ np.array([0.001, 0.0, 0.0]), rotation))

        errors = []
        calibrator = object.__new__(PalmWristCalibrator)
        calibrator.motion = "yaw"
        calibrator._stop_keyboard = threading.Event()
        calibrator.node = SimpleNamespace(
            get_logger=lambda: SimpleNamespace(
                info=lambda *_: None, error=lambda message: errors.append(message)
            )
        )
        calibrator._save = lambda *_: self.fail("invalid wrist result was saved")

        calibrator._finish_calibration(samples)

        self.assertFalse(calibrator._stop_keyboard.is_set())
        self.assertTrue(any("掌心到手腕距离过短" in message for message in errors))

    def test_wrist_to_palm_distance_above_one_centimeter_is_accepted(self):
        wrist = np.array([0.4, -0.2, 0.9])
        offset = np.array([0.015, 0.0, 0.0])
        samples = []
        for yaw in np.linspace(-1.0, 1.0, 120):
            rotation = np.array([
                [math.cos(yaw), -math.sin(yaw), 0.0],
                [math.sin(yaw), math.cos(yaw), 0.0],
                [0.0, 0.0, 1.0],
            ])
            samples.append((wrist + rotation @ offset, rotation))

        calibrator = object.__new__(PalmWristCalibrator)
        calibrator.motion = "yaw"
        calibrator._stop_keyboard = threading.Event()
        calibrator.node = SimpleNamespace(
            get_logger=lambda: SimpleNamespace(info=lambda *_: None, error=lambda *_: None)
        )
        calibrator._save = lambda *_: None

        calibrator._finish_calibration(samples)

        self.assertTrue(calibrator._stop_keyboard.is_set())

    def test_negative_local_x_is_rejected_even_when_distance_is_valid(self):
        wrist = np.array([0.4, -0.2, 0.9])
        offset = np.array([-0.005, 0.02, 0.0])
        samples = []
        for yaw in np.linspace(-1.0, 1.0, 120):
            rotation = np.array([
                [math.cos(yaw), -math.sin(yaw), 0.0],
                [math.sin(yaw), math.cos(yaw), 0.0],
                [0.0, 0.0, 1.0],
            ])
            samples.append((wrist + rotation @ offset, rotation))

        errors = []
        calibrator = object.__new__(PalmWristCalibrator)
        calibrator.motion = "yaw"
        calibrator._stop_keyboard = threading.Event()
        calibrator.node = SimpleNamespace(
            get_logger=lambda: SimpleNamespace(
                info=lambda *_: None, error=lambda message: errors.append(message)
            )
        )
        calibrator._save = lambda *_: self.fail("invalid wrist result was saved")

        calibrator._finish_calibration(samples)

        self.assertFalse(calibrator._stop_keyboard.is_set())
        self.assertTrue(any("局部 +X" in message for message in errors))

    def test_run_returns_when_success_event_is_set_from_callback(self):
        calibrator = object.__new__(PalmWristCalibrator)
        calibrator.node = object()
        calibrator._stop_keyboard = threading.Event()
        spin_calls = []

        def spin_once(_node, timeout_sec):
            spin_calls.append(timeout_sec)
            calibrator._stop_keyboard.set()

        calibrator.rclpy = SimpleNamespace(ok=lambda: True, spin_once=spin_once)
        calibrator.keyboard_loop = lambda: None

        calibrator.run()

        self.assertEqual(spin_calls, [0.1])


if __name__ == "__main__":
    unittest.main()
