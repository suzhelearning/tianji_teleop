#!/usr/bin/env python3
"""数据手套标定与一代手遥操作消息公共合同测试。"""

from __future__ import annotations

import io
import math
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch

from data_glove_wuji_teleop.adapters.glove.encoder_stream import EncoderFrame
from data_glove_wuji_teleop.application.calibration import (
    CaptureWindow,
    FINGER_FLEX_INDICES,
    FINGER_LATERAL_INDICES,
    StableCapture,
    SweepCapture,
    THUMB_FLEX_INDICES,
    _capture_lateral_until_valid,
    _capture_stable_until_valid,
    build_workflow_profile,
    command_calibrate,
    command_inspect,
    summarize_stable_frames,
    summarize_sweep_frames,
)
from data_glove_wuji_teleop.application.teleoperation import (
    _publish_zero_burst,
    build_hand_target,
)
from data_glove_wuji_teleop.cli import build_parser, command_run
from data_glove_wuji_teleop.profiles.dataglove.mapping import CalibrationProfile


class CalibrationWorkflowTest(unittest.TestCase):
    def test_workflow_wraps_flex_endpoint_across_encoder_boundary(
        self,
    ) -> None:
        zero_offsets = [0.0] * 21
        zero_offsets[5] = 80.0
        thumb_flex = zero_offsets.copy()
        thumb_flex[5] = 340.0
        disabled = {f"J{index}" for index in range(1, 21)} - {"J6"}

        profile = build_workflow_profile(
            hand="left",
            cs_by_joint=range(21),
            finger_flex=StableCapture.from_values(zero_offsets),
            thumb_flex=StableCapture.from_values(thumb_flex),
            finger_lateral=SweepCapture.from_ranges(
                zero_offsets,
                zero_offsets,
            ),
            thumb_lateral=SweepCapture.from_ranges(
                zero_offsets,
                zero_offsets,
            ),
            disabled_channels=disabled,
            zero_offsets_deg=zero_offsets,
        )

        endpoint = profile.axes[5].negative
        self.assertIsNotNone(endpoint)
        self.assertAlmostEqual(endpoint.input_deg, -100.0)
        self.assertAlmostEqual(
            profile.map_to_finger_target(thumb_flex)[0][0],
            math.radians(60.0),
        )

    def test_workflow_builds_endpoints_relative_to_calibrated_zero(
        self,
    ) -> None:
        zero_offsets = [0.0] * 21
        zero_offsets[0] = 20.0
        zero_offsets[5] = 5.0
        thumb_flex = zero_offsets.copy()
        thumb_flex[5] = 35.0
        thumb_min = zero_offsets.copy()
        thumb_max = zero_offsets.copy()
        thumb_min[0] = 10.0
        thumb_max[0] = 30.0
        enabled = {"J1", "J6"}
        disabled = {f"J{index}" for index in range(1, 21)} - enabled

        profile = build_workflow_profile(
            hand="right",
            cs_by_joint=range(21),
            finger_flex=StableCapture.from_values(zero_offsets),
            thumb_flex=StableCapture.from_values(thumb_flex),
            finger_lateral=SweepCapture.from_ranges(
                zero_offsets,
                zero_offsets,
            ),
            thumb_lateral=SweepCapture.from_ranges(thumb_min, thumb_max),
            disabled_channels=disabled,
            zero_offsets_deg=zero_offsets,
        )

        self.assertEqual(
            profile.map_to_finger_target(zero_offsets)[0],
            [0.0] * 4,
        )
        moved = zero_offsets.copy()
        moved[0] = 30.0
        moved[5] = 35.0
        self.assertAlmostEqual(
            profile.map_to_finger_target(moved)[0][0],
            math.radians(60.0),
        )
        self.assertAlmostEqual(
            profile.map_to_finger_target(moved)[0][1],
            math.radians(10.0),
        )

    def test_rejects_lateral_target_above_wuji_safe_range(self) -> None:
        with self.assertRaisesRegex(ValueError, "侧摆目标.*20"):
            build_workflow_profile(
                hand="left",
                cs_by_joint=range(21),
                finger_flex=StableCapture.from_values([0.0] * 21),
                thumb_flex=StableCapture.from_values([0.0] * 21),
                finger_lateral=SweepCapture.from_ranges(
                    [0.0] * 21,
                    [0.0] * 21,
                ),
                thumb_lateral=SweepCapture.from_ranges(
                    [0.0] * 21,
                    [0.0] * 21,
                ),
                lateral_target_deg=20.1,
            )

    def test_stable_step_retries_only_current_capture(self) -> None:
        unstable_values = [0.0] * 21
        unstable_motion = [0.0] * 21
        unstable_values[6] = -50.0
        unstable_motion[6] = 20.0
        stable_values = unstable_values.copy()
        stable_motion = [0.0] * 21
        stable_motion[6] = 1.0
        unstable = StableCapture.from_values(
            unstable_values,
            unstable_motion,
        )
        stable = StableCapture.from_values(stable_values, stable_motion)
        mapping = tuple(range(21))
        disabled = {f"J{index}" for index in range(1, 21)} - {"J7"}

        with (
            patch("builtins.input"),
            patch(
                "data_glove_wuji_teleop.application.calibration._read_window",
                side_effect=[
                    CaptureWindow((), mapping, True),
                    CaptureWindow((), mapping, True),
                ],
            ) as read_window,
            patch(
                "data_glove_wuji_teleop.application.calibration.summarize_stable_frames",
                side_effect=[unstable, stable],
            ),
            patch("data_glove_wuji_teleop.application.calibration._print_stable"),
            patch("sys.stdout", new=io.StringIO()),
            patch("sys.stderr", new=io.StringIO()),
        ):
            capture, captured_mapping = _capture_stable_until_valid(
                prompt="test",
                label="四指屈伸",
                indices=(6,),
                host="unused",
                port=9100,
                duration_s=1.0,
                expected_mapping=None,
                disabled_channels=disabled,
                max_motion_deg=3.0,
                max_motion_ratio=0.1,
                min_travel_deg=5.0,
            )

        self.assertIs(capture, stable)
        self.assertEqual(captured_mapping, mapping)
        self.assertEqual(read_window.call_count, 2)

    def test_stable_step_validates_travel_relative_to_captured_zero(
        self,
    ) -> None:
        zero_offsets = [100.0] * 21
        too_close = zero_offsets.copy()
        too_close[6] = 102.0
        far_enough = zero_offsets.copy()
        far_enough[6] = 110.0
        captures = (
            StableCapture.from_values(too_close),
            StableCapture.from_values(far_enough),
        )
        mapping = tuple(range(21))
        disabled = {f"J{index}" for index in range(1, 21)} - {"J7"}

        with (
            patch("builtins.input"),
            patch(
                "data_glove_wuji_teleop.application.calibration._read_window",
                side_effect=[
                    CaptureWindow((), mapping, False),
                    CaptureWindow((), mapping, False),
                ],
            ) as read_window,
            patch(
                "data_glove_wuji_teleop.application.calibration.summarize_stable_frames",
                side_effect=captures,
            ),
            patch("data_glove_wuji_teleop.application.calibration._print_stable"),
            patch("sys.stdout", new=io.StringIO()),
            patch("sys.stderr", new=io.StringIO()),
        ):
            capture, _ = _capture_stable_until_valid(
                prompt="test",
                label="食指屈伸",
                indices=(6,),
                host="unused",
                port=9100,
                duration_s=1.0,
                expected_mapping=mapping,
                disabled_channels=disabled,
                max_motion_deg=3.0,
                max_motion_ratio=0.1,
                min_travel_deg=5.0,
                zero_offsets_deg=zero_offsets,
            )

        self.assertIs(capture, captures[1])
        self.assertEqual(read_window.call_count, 2)

    def test_lateral_step_retries_then_accepts_one_sided_capture(self) -> None:
        bad_min = [0.0] * 21
        bad_max = [0.0] * 21
        bad_min[1], bad_max[1] = -1.0, 2.0
        good_min = [0.0] * 21
        good_max = [0.0] * 21
        good_min[1], good_max[1] = 0.837, 16.569
        bad = SweepCapture.from_ranges(bad_min, bad_max)
        good = SweepCapture.from_ranges(good_min, good_max)
        mapping = tuple(range(21))
        disabled = {f"J{index}" for index in range(1, 21)} - {"J2"}

        with (
            patch("builtins.input"),
            patch(
                "data_glove_wuji_teleop.application.calibration._read_window",
                side_effect=[
                    CaptureWindow((), mapping, True),
                    CaptureWindow((), mapping, True),
                ],
            ) as read_window,
            patch(
                "data_glove_wuji_teleop.application.calibration.summarize_sweep_frames",
                side_effect=[bad, good],
            ),
            patch("data_glove_wuji_teleop.application.calibration._print_sweep"),
            patch("sys.stdout", new=io.StringIO()),
            patch("sys.stderr", new=io.StringIO()),
        ):
            capture = _capture_lateral_until_valid(
                prompt="test",
                label="四指侧摆",
                indices=(1,),
                host="unused",
                port=9100,
                duration_s=1.0,
                expected_mapping=mapping,
                disabled_channels=disabled,
                min_travel_deg=3.0,
            )

        self.assertIs(capture, good)
        self.assertEqual(read_window.call_count, 2)

    def test_lateral_step_validates_travel_relative_to_captured_zero(
        self,
    ) -> None:
        zero_offsets = [50.0] * 21
        bad_min = zero_offsets.copy()
        bad_max = zero_offsets.copy()
        bad_min[1], bad_max[1] = 49.0, 52.0
        good_min = zero_offsets.copy()
        good_max = zero_offsets.copy()
        good_min[1], good_max[1] = 49.0, 56.0
        captures = (
            SweepCapture.from_ranges(bad_min, bad_max),
            SweepCapture.from_ranges(good_min, good_max),
        )
        mapping = tuple(range(21))
        disabled = {f"J{index}" for index in range(1, 21)} - {"J2"}

        with (
            patch("builtins.input"),
            patch(
                "data_glove_wuji_teleop.application.calibration._read_window",
                side_effect=[
                    CaptureWindow((), mapping, False),
                    CaptureWindow((), mapping, False),
                ],
            ) as read_window,
            patch(
                "data_glove_wuji_teleop.application.calibration.summarize_sweep_frames",
                side_effect=captures,
            ),
            patch("data_glove_wuji_teleop.application.calibration._print_sweep"),
            patch("sys.stdout", new=io.StringIO()),
            patch("sys.stderr", new=io.StringIO()),
        ):
            capture = _capture_lateral_until_valid(
                prompt="test",
                label="食指侧摆",
                indices=(1,),
                host="unused",
                port=9100,
                duration_s=1.0,
                expected_mapping=mapping,
                disabled_channels=disabled,
                min_travel_deg=3.0,
                zero_offsets_deg=zero_offsets,
            )

        self.assertIs(capture, captures[1])
        self.assertEqual(read_window.call_count, 2)

    def test_accepts_real_one_sided_finger_lateral_capture(self) -> None:
        """回归：外侧手指以零位为边界时，不强制扫过零点两侧。"""

        finger_values = [0.0] * 21
        finger_motion = [0.0] * 21
        thumb_values = [0.0] * 21
        thumb_motion = [0.0] * 21
        for index, value, motion in (
            (6, -55.176, 0.374),
            (7, -82.019, 0.143),
            (8, -78.012, 0.198),
            (11, -138.508, 0.187),
            (12, -106.547, 0.264),
            (13, -114.668, 0.291),
            (16, -8.156, 0.198),
            (17, -64.049, 0.247),
            (18, -35.706, 0.203),
        ):
            finger_values[index] = value
            finger_motion[index] = motion
        for index, value, motion in (
            (5, -9.905, 0.857),
            (10, -52.969, 0.198),
            (15, -112.059, 1.132),
        ):
            thumb_values[index] = value
            thumb_motion[index] = motion

        finger_min = [0.0] * 21
        finger_max = [0.0] * 21
        finger_min[1], finger_max[1] = 0.837, 16.569
        finger_min[4], finger_max[4] = -30.564, -1.884
        thumb_min = [0.0] * 21
        thumb_max = [0.0] * 21
        thumb_min[0], thumb_max[0] = -13.461, 36.620

        profile = build_workflow_profile(
            hand="left",
            cs_by_joint=range(21),
            finger_flex=StableCapture.from_values(
                finger_values,
                finger_motion,
            ),
            thumb_flex=StableCapture.from_values(
                thumb_values,
                thumb_motion,
            ),
            finger_lateral=SweepCapture.from_ranges(finger_min, finger_max),
            thumb_lateral=SweepCapture.from_ranges(thumb_min, thumb_max),
        )

        self.assertIsNone(profile.axes[1].negative)
        self.assertIsNotNone(profile.axes[1].positive)
        self.assertIsNotNone(profile.axes[4].negative)
        self.assertIsNone(profile.axes[4].positive)
        self.assertAlmostEqual(
            profile.axes[1].map(16.569),
            math.radians(10.0),
        )
        self.assertAlmostEqual(
            profile.axes[4].map(-30.564),
            math.radians(-10.0),
        )

    def test_left_thumb_uses_left_wuji_joint_directions(self) -> None:
        thumb_values = [0.0] * 21
        thumb_values[5] = -30.0
        thumb_values[10] = 20.0
        thumb_values[15] = 40.0
        thumb_min = [0.0] * 21
        thumb_max = [0.0] * 21
        thumb_min[0], thumb_max[0] = -25.0, 15.0
        enabled = {"J1", "J6", "J11", "J16"}
        disabled = {f"J{index}" for index in range(1, 21)} - enabled

        profile = build_workflow_profile(
            hand="left",
            cs_by_joint=range(21),
            finger_flex=StableCapture.from_values([0.0] * 21),
            thumb_flex=StableCapture.from_values(thumb_values),
            finger_lateral=SweepCapture.from_ranges(
                [0.0] * 21,
                [0.0] * 21,
            ),
            thumb_lateral=SweepCapture.from_ranges(thumb_min, thumb_max),
            disabled_channels=disabled,
            inverted_channels={"J1"},
            lateral_target_deg=20.0,
        )

        self.assertEqual(profile.axes[0].deadband_deg, 0.0)
        angles = [0.0] * 21
        angles[0] = 15.0
        angles[5] = -30.0
        angles[10] = 20.0
        angles[15] = 40.0

        self.assertEqual(
            profile.map_to_finger_target(angles)[0],
            [
                math.radians(60.0),
                math.radians(-7.5),
                math.radians(36.0),
                math.radians(80.0),
            ],
        )

        angles[0] = -25.0
        self.assertAlmostEqual(
            profile.map_to_finger_target(angles)[0][1],
            math.radians(20.0),
        )

    def test_thumb_and_other_fingers_use_separate_captures(self) -> None:
        finger_values = [0.0] * 21
        thumb_values = [0.0] * 21
        finger_values[6] = -40.0   # J7: 食指屈伸1
        finger_values[11] = -50.0  # J12: 食指屈伸2
        finger_values[16] = -30.0  # J17: 食指屈伸3
        thumb_values[5] = -30.0    # J6: 拇指屈伸1
        thumb_values[10] = 20.0    # J11: 拇指屈伸2
        thumb_values[15] = 40.0    # J16: 拇指屈伸3

        finger_min = [0.0] * 21
        finger_max = [0.0] * 21
        thumb_min = [0.0] * 21
        thumb_max = [0.0] * 21
        finger_min[1], finger_max[1] = -20.0, 30.0  # J2: 食指侧摆
        thumb_min[0], thumb_max[0] = -25.0, 15.0    # J1: 拇指侧摆

        enabled = {"J1", "J2", "J6", "J7", "J11", "J12", "J16", "J17"}
        disabled = {f"J{index}" for index in range(1, 21)} - enabled
        profile = build_workflow_profile(
            hand="right",
            cs_by_joint=range(21),
            finger_flex=StableCapture.from_values(finger_values),
            thumb_flex=StableCapture.from_values(thumb_values),
            finger_lateral=SweepCapture.from_ranges(finger_min, finger_max),
            thumb_lateral=SweepCapture.from_ranges(thumb_min, thumb_max),
            disabled_channels=disabled,
            deadband_deg=0.0,
        )

        target = profile.map_to_finger_target(
            [
                15.0,
                -20.0,
                0.0,
                0.0,
                0.0,
                -30.0,
                -40.0,
                0.0,
                0.0,
                0.0,
                20.0,
                -50.0,
                0.0,
                0.0,
                0.0,
                40.0,
                -30.0,
                0.0,
                0.0,
                0.0,
                999.0,
            ]
        )

        self.assertEqual(
            target[0],
            [
                math.radians(60.0),
                math.radians(10.0),
                math.radians(-36.0),
                math.radians(-80.0),
            ],
        )
        self.assertEqual(
            target[1],
            [
                math.radians(80.0),
                math.radians(-10.0),
                math.radians(80.0),
                math.radians(60.0),
            ],
        )

    def test_encoder_frame_becomes_backend_independent_hand_target(self) -> None:
        disabled = {f"J{index}" for index in range(1, 21)}
        profile = build_workflow_profile(
            hand="left",
            cs_by_joint=range(21),
            finger_flex=StableCapture.from_values([0.0] * 21),
            thumb_flex=StableCapture.from_values([0.0] * 21),
            finger_lateral=SweepCapture.from_ranges([0.0] * 21, [0.0] * 21),
            thumb_lateral=SweepCapture.from_ranges([0.0] * 21, [0.0] * 21),
            disabled_channels=disabled,
        )
        frame = EncoderFrame(
            sequence=17,
            timestamp_ns=123456789,
            dropped=2,
            angles_deg=[99.0] * 21,
        )

        target = build_hand_target(profile, frame)

        self.assertEqual(target.hand, "left")
        self.assertEqual(target.source, "dataglove")
        self.assertEqual(target.sequence, 17)
        self.assertEqual(target.timestamp_ns, 123456789)
        self.assertEqual(target.dropped, 2)
        self.assertEqual(target.values, (0.0,) * 20)

    def test_rejects_unstable_enabled_axis_but_ignores_disabled_axis(self) -> None:
        finger_values = [0.0] * 21
        finger_motion = [0.0] * 21
        thumb_values = [0.0] * 21
        finger_values[6] = 20.0
        thumb_values[5] = 20.0
        finger_motion[9] = 999.0  # J10 已屏蔽，不应阻断标定

        finger_min = [0.0] * 21
        finger_max = [0.0] * 21
        thumb_min = [0.0] * 21
        thumb_max = [0.0] * 21
        finger_min[1], finger_max[1] = -5.0, 5.0
        thumb_min[0], thumb_max[0] = -5.0, 5.0
        enabled = {"J1", "J2", "J6", "J7"}
        disabled = {f"J{index}" for index in range(1, 21)} - enabled

        build_workflow_profile(
            hand="right",
            cs_by_joint=range(21),
            finger_flex=StableCapture.from_values(finger_values, finger_motion),
            thumb_flex=StableCapture.from_values(thumb_values),
            finger_lateral=SweepCapture.from_ranges(finger_min, finger_max),
            thumb_lateral=SweepCapture.from_ranges(thumb_min, thumb_max),
            disabled_channels=disabled,
            max_stable_motion_deg=3.0,
        )

        thumb_motion = [0.0] * 21
        thumb_motion[5] = 4.0
        with self.assertRaisesRegex(ValueError, "J6.*不稳定"):
            build_workflow_profile(
                hand="right",
                cs_by_joint=range(21),
                finger_flex=StableCapture.from_values(finger_values, finger_motion),
                thumb_flex=StableCapture.from_values(thumb_values, thumb_motion),
                finger_lateral=SweepCapture.from_ranges(finger_min, finger_max),
                thumb_lateral=SweepCapture.from_ranges(thumb_min, thumb_max),
                disabled_channels=disabled,
                max_stable_motion_deg=3.0,
            )

    def test_relative_stability_limit_accepts_large_range_joint(self) -> None:
        finger_values = [0.0] * 21
        finger_motion = [0.0] * 21
        thumb_values = [0.0] * 21
        thumb_motion = [0.0] * 21
        finger_values[6], finger_motion[6] = -57.148, 3.016
        thumb_values[15], thumb_motion[15] = -115.305, 6.218

        finger_min = [0.0] * 21
        finger_max = [0.0] * 21
        thumb_min = [0.0] * 21
        thumb_max = [0.0] * 21
        finger_min[1], finger_max[1] = -6.788, 19.700
        thumb_min[0], thumb_max[0] = -16.268, 39.817
        enabled = {"J1", "J2", "J7", "J16"}
        disabled = {f"J{index}" for index in range(1, 21)} - enabled

        kwargs = {
            "hand": "left",
            "cs_by_joint": range(21),
            "finger_flex": StableCapture.from_values(
                finger_values,
                finger_motion,
            ),
            "thumb_flex": StableCapture.from_values(
                thumb_values,
                thumb_motion,
            ),
            "finger_lateral": SweepCapture.from_ranges(
                finger_min,
                finger_max,
            ),
            "thumb_lateral": SweepCapture.from_ranges(thumb_min, thumb_max),
            "disabled_channels": disabled,
            "max_stable_motion_deg": 3.0,
            "max_stable_motion_ratio": 0.1,
        }
        build_workflow_profile(**kwargs)

        gross_motion = finger_motion.copy()
        gross_motion[6] = 41.852
        kwargs["finger_flex"] = StableCapture.from_values(
            finger_values,
            gross_motion,
        )
        with self.assertRaisesRegex(ValueError, "J7.*不稳定"):
            build_workflow_profile(**kwargs)

    def test_rejects_insufficient_flex_or_lateral_capture(self) -> None:
        finger_values = [0.0] * 21
        thumb_values = [0.0] * 21
        finger_values[6] = 2.0
        thumb_values[5] = 20.0
        finger_min = [0.0] * 21
        finger_max = [0.0] * 21
        thumb_min = [0.0] * 21
        thumb_max = [0.0] * 21
        finger_min[1], finger_max[1] = -5.0, 5.0
        thumb_min[0], thumb_max[0] = -5.0, 5.0
        enabled = {"J1", "J2", "J6", "J7"}
        disabled = {f"J{index}" for index in range(1, 21)} - enabled

        with self.assertRaisesRegex(ValueError, "J7.*幅度不足"):
            build_workflow_profile(
                hand="right",
                cs_by_joint=range(21),
                finger_flex=StableCapture.from_values(finger_values),
                thumb_flex=StableCapture.from_values(thumb_values),
                finger_lateral=SweepCapture.from_ranges(finger_min, finger_max),
                thumb_lateral=SweepCapture.from_ranges(thumb_min, thumb_max),
                disabled_channels=disabled,
                min_flex_travel_deg=5.0,
            )

        finger_values[6] = 20.0
        finger_min[1], finger_max[1] = -1.0, 2.5
        with self.assertRaisesRegex(ValueError, "J2.*有效量程不足"):
            build_workflow_profile(
                hand="right",
                cs_by_joint=range(21),
                finger_flex=StableCapture.from_values(finger_values),
                thumb_flex=StableCapture.from_values(thumb_values),
                finger_lateral=SweepCapture.from_ranges(finger_min, finger_max),
                thumb_lateral=SweepCapture.from_ranges(thumb_min, thumb_max),
                disabled_channels=disabled,
                min_lateral_travel_deg=3.0,
            )

    def test_frame_windows_use_median_robust_motion_and_extrema(self) -> None:
        channel0 = [-100.0] + [10.0] * 18 + [100.0]
        frames = [
            EncoderFrame(
                sequence=index,
                timestamp_ns=index * 10,
                dropped=0,
                angles_deg=[value] + [0.0] * 20,
            )
            for index, value in enumerate(channel0)
        ]

        stable = summarize_stable_frames(frames)
        sweep = summarize_sweep_frames(frames)

        self.assertEqual(stable.values_deg[0], 10.0)
        self.assertEqual(stable.motion_deg[0], 0.0)
        self.assertEqual(sweep.minimum_deg[0], -100.0)
        self.assertEqual(sweep.maximum_deg[0], 100.0)


class WorkflowScriptSafetyTest(unittest.TestCase):
    def test_calibration_captures_natural_zero_before_motion_steps(
        self,
    ) -> None:
        zero_values = [10.0] * 21
        finger_values = zero_values.copy()
        thumb_values = zero_values.copy()
        for index in FINGER_FLEX_INDICES:
            finger_values[index] = 30.0
        for index in THUMB_FLEX_INDICES:
            thumb_values[index] = 30.0
        finger_min = zero_values.copy()
        finger_max = zero_values.copy()
        for index in FINGER_LATERAL_INDICES:
            finger_min[index] = 5.0
            finger_max[index] = 15.0
        thumb_min = zero_values.copy()
        thumb_max = zero_values.copy()
        thumb_min[0] = 5.0
        thumb_max[0] = 15.0
        events: list[str] = []
        prompts: list[str] = []

        def capture_zero(**_kwargs):
            events.append("zero")
            return (
                StableCapture.from_values(zero_values),
                tuple(range(21)),
            )

        stable_captures = iter(
            (
                StableCapture.from_values(finger_values),
                StableCapture.from_values(thumb_values),
            )
        )

        def capture_stable(**kwargs):
            events.append("stable")
            prompts.append(kwargs["prompt"])
            return next(stable_captures), tuple(range(21))

        lateral_captures = iter(
            (
                SweepCapture.from_ranges(finger_min, finger_max),
                SweepCapture.from_ranges(thumb_min, thumb_max),
            )
        )

        def capture_lateral(**kwargs):
            events.append("lateral")
            prompts.append(kwargs["prompt"])
            return next(lateral_captures)

        with TemporaryDirectory() as directory:
            output = Path(directory) / "left.json"
            args = build_parser().parse_args(
                [
                    "calibrate",
                    "--hand",
                    "left",
                    "--output",
                    str(output),
                ]
            )
            with (
                patch(
                    "data_glove_wuji_teleop.application.calibration."
                    "_capture_zero_until_valid",
                    side_effect=capture_zero,
                ),
                patch(
                    "data_glove_wuji_teleop.application.calibration."
                    "_capture_stable_until_valid",
                    side_effect=capture_stable,
                ),
                patch(
                    "data_glove_wuji_teleop.application.calibration."
                    "_capture_lateral_until_valid",
                    side_effect=capture_lateral,
                ),
                patch("sys.stdout", new=io.StringIO()),
            ):
                result = command_calibrate(args)
            profile = CalibrationProfile.load(output)

        self.assertEqual(result, 0)
        self.assertEqual(
            events,
            ["zero", "stable", "stable", "lateral", "lateral"],
        )
        self.assertIn("食指、中指、无名指", prompts[0])
        self.assertNotIn("小指", prompts[0])
        self.assertIn("食指、小指", prompts[2])
        self.assertNotIn("中指", prompts[2])
        self.assertNotIn("无名指", prompts[2])
        self.assertEqual(
            profile.map_to_dof_values(zero_values),
            [0.0] * 20,
        )

    def test_inspect_reads_unzeroed_stream_and_reports_actual_state(
        self,
    ) -> None:
        class UnzeroedConnection:
            channels = 21
            zeroed = False
            range_min = 0.0
            range_max = 360.0
            cs_by_joint = list(range(21))
            sequence = 0

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def read_frame(self, **_kwargs):
                frame = EncoderFrame(
                    sequence=self.sequence,
                    timestamp_ns=self.sequence * 20_000_000,
                    dropped=0,
                    angles_deg=[0.0] * 21,
                )
                self.sequence += 1
                return frame

        with (
                patch(
                    "data_glove_wuji_teleop.application.encoder_capture."
                    "EncoderConnection.connect",
                return_value=UnzeroedConnection(),
            ),
            patch("sys.stdout", new=io.StringIO()) as stdout,
        ):
            result = command_inspect(
                SimpleNamespace(
                    host="unused",
                    port=9100,
                    seconds=0.01,
                )
            )

        self.assertEqual(result, 0)
        self.assertIn("zeroed=false", stdout.getvalue())

    def test_backend_failure_does_not_mask_shutdown_cleanup(self) -> None:
        calibration = (
            Path(__file__).resolve().parents[2]
            / "config"
            / "dataglove"
            / "calibration"
            / "left.json"
        )
        profile = CalibrationProfile.load(calibration)

        class FailedPublisher:
            calls = 0

            def publish(self, _target):
                self.calls += 1
                raise RuntimeError("hardware already closed")

        publisher = FailedPublisher()
        _publish_zero_burst(publisher, profile)

        self.assertEqual(publisher.calls, 1)





    def test_run_rejects_calibration_from_the_other_hand(self) -> None:
        left_calibration = (
            Path(__file__).resolve().parents[2]
            / "config"
            / "dataglove"
            / "calibration"
            / "left.json"
        )
        args = build_parser().parse_args(
            [
                "run",
                "--hand",
                "right",
                "--cal-file",
                str(left_calibration),
                "--dry-run",
            ]
        )

        with self.assertRaisesRegex(ValueError, "不能用于 right"):
            command_run(args)

if __name__ == "__main__":
    unittest.main()
