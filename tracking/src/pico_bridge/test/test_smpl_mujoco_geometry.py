#!/usr/bin/env python3
import math
import inspect
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
import smpl_mujoco_visualizer as smpl_viewer  # noqa: E402
from smpl_mujoco_visualizer import (  # noqa: E402
    BONE_EDGES,
    JOINT_NAMES,
    PoseFrameCache,
    SmplMujocoVisualizer,
    _make_xml,
    build_parser,
    capsule_pose,
    transform_orientation,
    transform_position,
    format_hand_position_overlay,
    HAND_VIEW_JOINT_INDICES,
    HAND_VIEW_BONE_INDICES,
    ARM_AXIS_JOINT_NAMES,
    ARM_AXIS_JOINT_INDICES,
    derive_tcp_palm_pose,
    derive_wrist_pose,
    resolve_visualization_wrist_pose,
    load_visualization_tcp_artifact,
    load_visualization_wrist_pivot_artifact,
)


class SmplGeometryTest(unittest.TestCase):
    @staticmethod
    def _pose_array(position_x=0.0):
        return SimpleNamespace(poses=[
            SimpleNamespace(
                position=SimpleNamespace(x=position_x, y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
            for _ in JOINT_NAMES
        ])

    def test_non_finite_position_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "finite"):
            transform_position((math.nan, 0.0, 0.0))

    def test_invalid_raw_frame_does_not_replace_last_valid_frame(self):
        warnings = []
        visualizer = SmplMujocoVisualizer.__new__(SmplMujocoVisualizer)
        visualizer.scale = 1.0
        visualizer.yaw = 0.0
        visualizer.show_raw = False
        visualizer._last_warning = {"primary": 0.0, "raw": 0.0}
        visualizer._raw_cache = PoseFrameCache()
        visualizer.node = SimpleNamespace(
            get_logger=lambda: SimpleNamespace(warning=warnings.append)
        )
        visualizer._raw_pose_callback(self._pose_array(position_x=1.0))

        visualizer._raw_pose_callback(self._pose_array(position_x=math.nan))

        positions, _, _ = visualizer._raw_cache.snapshot()
        self.assertTrue(np.all(np.isfinite(positions[0])))
        self.assertEqual(positions[0][0], 1.0)
        self.assertTrue(warnings)

    def test_visualizer_accepts_optional_raw_stream(self):
        parameters = inspect.signature(SmplMujocoVisualizer).parameters

        self.assertIn("show_raw", parameters)
        self.assertIn("raw_topic", parameters)
        self.assertFalse(parameters["show_raw"].default)
        self.assertEqual(parameters["raw_topic"].default, "/pico/smpl")

    def test_hands_only_parser_and_hand_overlay_format(self):
        arguments = build_parser().parse_args(["--hands-only"])

        self.assertTrue(arguments.hands_only)
        positions = [np.zeros(3) for _ in JOINT_NAMES]
        positions[JOINT_NAMES.index("LEFT_HAND")] = np.array([0.1234, -0.2, 1.5])
        positions[JOINT_NAMES.index("RIGHT_HAND")] = np.array([-0.4, 0.05, 1.2345])
        overlay = format_hand_position_overlay(positions)
        self.assertIn("LEFT_HAND", overlay)
        self.assertIn("x=0.123", overlay)
        self.assertIn("RIGHT_HAND", overlay)
        self.assertIn("z=1.234", overlay)

    def test_arm_axes_parser_and_joint_scope(self):
        arguments = build_parser().parse_args(["--hands-only", "--show-arm-axes"])

        self.assertTrue(arguments.show_arm_axes)
        self.assertFalse(arguments.show_raw_arm_axes)
        self.assertFalse(arguments.show_raw_pelvis_axes)
        self.assertEqual(
            set(ARM_AXIS_JOINT_NAMES),
            {
                "LEFT_SHOULDER", "LEFT_ELBOW", "LEFT_WRIST", "LEFT_HAND",
                "RIGHT_SHOULDER", "RIGHT_ELBOW", "RIGHT_WRIST", "RIGHT_HAND",
            },
        )
        self.assertEqual(len(ARM_AXIS_JOINT_INDICES), 8)
        self.assertNotIn("Pelvis", ARM_AXIS_JOINT_NAMES)
        self.assertNotIn("SPINE3", ARM_AXIS_JOINT_NAMES)

    def test_xml_contains_optional_upper_limb_axes_only(self):
        xml = _make_xml(show_arm_axes=True, show_raw=True, show_raw_arm_axes=True)

        self.assertIn('name="arm_axes_16"', xml)
        self.assertIn('name="raw_arm_axes_16"', xml)
        self.assertEqual(xml.count('class="arm_axis"'), 37)
        self.assertEqual(xml.count('class="raw_arm_axis"'), 25)
        self.assertNotIn('name="arm_axes_0"', xml)
        self.assertNotIn('name="arm_axes_9"', xml)

    def test_raw_arm_axes_are_opt_in(self):
        xml = _make_xml(show_arm_axes=True, show_raw=True)

        self.assertIn('name="arm_axes_16"', xml)
        self.assertNotIn('name="raw_arm_axes_16"', xml)

    def test_hands_only_keeps_upper_body_from_pelvis(self):
        self.assertIn(JOINT_NAMES.index("Pelvis"), HAND_VIEW_JOINT_INDICES)
        self.assertIn(JOINT_NAMES.index("SPINE3"), HAND_VIEW_JOINT_INDICES)
        self.assertIn(JOINT_NAMES.index("LEFT_HAND"), HAND_VIEW_JOINT_INDICES)
        self.assertNotIn(JOINT_NAMES.index("LEFT_HIP"), HAND_VIEW_JOINT_INDICES)
        self.assertNotIn(JOINT_NAMES.index("LEFT_KNEE"), HAND_VIEW_JOINT_INDICES)
        self.assertIn(
            BONE_EDGES.index((JOINT_NAMES.index("Pelvis"), JOINT_NAMES.index("SPINE1"))),
            HAND_VIEW_BONE_INDICES,
        )

    def test_primary_and_raw_frame_caches_are_independent(self):
        primary = PoseFrameCache()
        raw = PoseFrameCache()
        positions = [np.array([1.0, 2.0, 3.0])]
        orientations = [np.array([1.0, 0.0, 0.0, 0.0])]

        raw.update(positions, orientations, received_at=42.0)

        self.assertIsNone(primary.snapshot()[0])
        raw_positions, raw_orientations, raw_time = raw.snapshot()
        np.testing.assert_allclose(raw_positions[0], positions[0])
        np.testing.assert_allclose(raw_orientations[0], orientations[0])
        self.assertEqual(raw_time, 42.0)

    def test_frame_cache_snapshot_does_not_expose_mutable_state(self):
        cache = PoseFrameCache()
        cache.update(
            [np.array([1.0, 2.0, 3.0])],
            [np.array([1.0, 0.0, 0.0, 0.0])],
            received_at=7.0,
        )

        positions, orientations, _ = cache.snapshot()
        positions[0][0] = 99.0
        orientations[0][0] = 0.0

        cached_positions, cached_orientations, _ = cache.snapshot()
        self.assertEqual(cached_positions[0][0], 1.0)
        self.assertEqual(cached_orientations[0][0], 1.0)

    def test_raw_overlay_parser_defaults_to_disabled(self):
        arguments = build_parser().parse_args([])

        self.assertFalse(arguments.show_raw)
        self.assertEqual(arguments.raw_topic, "/pico/smpl")
        self.assertEqual(arguments.palm_topic, "/pico/palm_left")
        self.assertEqual(arguments.right_palm_topic, "/pico/palm_right")
        self.assertEqual(arguments.wrist_topic, "/pico/wrist_left")
        self.assertEqual(arguments.right_wrist_topic, "/pico/wrist_right")
        np.testing.assert_allclose(arguments.raw_offset, [0.45, 0.0, 0.0])

    def test_controller_overlay_parser_accepts_head_and_hand_flag(self):
        arguments = build_parser().parse_args(["--show-controllers"])

        self.assertTrue(arguments.show_controllers)
        self.assertEqual(arguments.head_topic, "/pico/pose/head")
        self.assertEqual(arguments.left_controller_topic, "/pico/pose/left_hand")
        self.assertEqual(arguments.right_controller_topic, "/pico/pose/right_hand")

    def test_visualizer_can_load_tcp_and_wrist_artifacts_for_topic_fallback(self):
        import tempfile
        import yaml

        with tempfile.TemporaryDirectory() as directory:
            tcp_path = Path(directory) / "left_tcp.yaml"
            tcp_path.write_text(
                yaml.safe_dump(
                    {
                        "valid": True,
                        "side": "left",
                        "pose_semantics": "controller_pose",
                        "translation_m": [0.10, 0.0, 0.0],
                        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                    }
                ),
                encoding="utf-8",
            )
            pivot_path = Path(directory) / "left_pivot.yaml"
            pivot_path.write_text(
                yaml.safe_dump(
                    {
                        "valid": True,
                        "side": "left",
                        "transform_convention": "wrist_to_palm",
                        "wrist_to_palm_m": [0.06, -0.08, 0.0],
                    }
                ),
                encoding="utf-8",
            )

            tcp = load_visualization_tcp_artifact(tcp_path, "left")
            pivot = load_visualization_wrist_pivot_artifact(pivot_path, "left")

        self.assertIsNotNone(tcp)
        np.testing.assert_allclose(tcp[0], [0.10, 0.0, 0.0])
        np.testing.assert_allclose(tcp[1], [1.0, 0.0, 0.0, 0.0])
        self.assertAlmostEqual(pivot, 0.10)

    def test_tcp_and_wrist_fallback_preserve_palm_wrist_geometry(self):
        palm_position, palm_orientation = derive_tcp_palm_pose(
            controller_position=np.array([1.0, 2.0, 3.0]),
            controller_orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            tcp_translation=np.array([0.10, 0.0, 0.0]),
            tcp_orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            scale=1.0,
        )
        wrist_position, wrist_orientation = derive_wrist_pose(
            palm_position,
            palm_orientation,
            0.08,
        )

        np.testing.assert_allclose(palm_position, [1.10, 2.0, 3.0])
        np.testing.assert_allclose(wrist_position, [1.02, 2.0, 3.0])
        np.testing.assert_allclose(palm_orientation, [1.0, 0.0, 0.0, 0.0])
        np.testing.assert_allclose(wrist_orientation, palm_orientation)

    def test_tcp_translation_is_rotated_by_controller_orientation(self):
        quarter_turn = math.sqrt(0.5)
        palm_position, palm_orientation = derive_tcp_palm_pose(
            controller_position=np.zeros(3),
            controller_orientation=np.array(
                [quarter_turn, 0.0, 0.0, quarter_turn]
            ),
            tcp_translation=np.array([0.10, 0.0, 0.0]),
            tcp_orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )

        np.testing.assert_allclose(palm_position, [0.0, 0.10, 0.0], atol=1e-8)
        np.testing.assert_allclose(
            palm_orientation,
            [quarter_turn, 0.0, 0.0, quarter_turn],
            atol=1e-8,
        )

    def test_wrist_fallback_uses_rotated_palm_local_positive_x(self):
        quarter_turn = math.sqrt(0.5)
        palm_orientation = np.array(
            [quarter_turn, 0.0, 0.0, quarter_turn], dtype=float
        )
        wrist_position, wrist_orientation = derive_wrist_pose(
            np.array([1.0, 2.0, 3.0]), palm_orientation, 0.10
        )
        np.testing.assert_allclose(wrist_position, [1.0, 1.9, 3.0], atol=1e-8)
        np.testing.assert_allclose(wrist_orientation, palm_orientation)

    def test_controller_overlay_geometry_is_optional_and_complete(self):
        hidden_xml = _make_xml()
        visible_xml = _make_xml(show_controllers=True)

        self.assertNotIn('name="controller_head_geom"', hidden_xml)
        for name in ("head", "left_hand", "right_hand"):
            self.assertIn(f'name="controller_{name}_free" type="free"', visible_xml)
            self.assertIn(f'name="controller_{name}_axes_free" type="free"', visible_xml)
            self.assertIn(f'name="controller_{name}_geom"', visible_xml)
            for axis in ("x", "y", "z"):
                self.assertIn(f'name="controller_{name}_axis_{axis}"', visible_xml)
        self.assertIn('<default class="controller_axis">', visible_xml)

    def test_raw_overlay_parser_accepts_topic(self):
        arguments = build_parser().parse_args(
            [
                "--show-raw",
                "--raw-topic",
                "/custom/raw",
                "--raw-offset",
                "0.0",
                "0.5",
                "-0.1",
            ]
        )

        self.assertTrue(arguments.show_raw)
        self.assertEqual(arguments.raw_topic, "/custom/raw")
        np.testing.assert_allclose(arguments.raw_offset, [0.0, 0.5, -0.1])

    def test_default_xml_omits_raw_overlay_geometry(self):
        xml = _make_xml()

        self.assertNotIn('name="raw_joint_0"', xml)
        self.assertNotIn('name="raw_bone_0"', xml)
        self.assertNotIn('name="raw_foot_left"', xml)

    def test_xml_contains_optional_calibrated_palm_marker(self):
        xml = _make_xml()
        self.assertIn('name="palm_left"', xml)
        self.assertIn('name="palm_left_free" type="free"', xml)
        self.assertIn('name="palm_left_geom"', xml)
        self.assertIn('name="palm_left_axes"', xml)
        self.assertEqual(xml.count('name="palm_left_axis_'), 3)
        self.assertIn('name="palm_right_geom"', xml)
        self.assertEqual(xml.count('name="palm_right_axis_'), 3)

    def test_xml_contains_wrist_markers_and_palm_connectors(self):
        xml = _make_xml()

        for side in ("left", "right"):
            self.assertIn(f'name="wrist_{side}_free" type="free"', xml)
            self.assertIn(f'name="wrist_{side}_axes_free" type="free"', xml)
            self.assertIn(f'name="wrist_{side}_geom"', xml)
            self.assertIn(f'name="wrist_{side}_palm_bone_free" type="free"', xml)
            self.assertIn(f'name="wrist_{side}_palm_bone_geom"', xml)

    def test_ground_body_starts_at_neutral_pose_before_runtime_lock(self):
        xml = _make_xml()

        self.assertIn('<body name="ground" pos="0 0 0">', xml)
        self.assertNotIn('<body name="ground" pos="0 0 -1.6">', xml)

    def test_xml_uses_reference_checker_environment_and_lighting(self):
        xml = _make_xml()

        self.assertIn('type="skybox" builtin="gradient"', xml)
        self.assertIn('name="groundplane" builtin="checker"', xml)
        self.assertIn('material name="groundplane"', xml)
        self.assertIn('<light name="sun"', xml)
        self.assertIn('<headlight diffuse="0.6 0.6 0.6"', xml)

    def test_ground_and_world_axes_start_hidden(self):
        xml = _make_xml()

        self.assertIn(
            'name="ground_geom" type="plane" size="10 10 .01" '
            'material="groundplane" rgba="1 1 1 0"',
            xml,
        )
        for axis in ("x", "y", "z"):
            self.assertIn(f'name="world_axis_{axis}"', xml)
        self.assertEqual(xml.count('class="world_axis"'), 4)

    def test_xml_contains_primary_and_optional_raw_pelvis_axes(self):
        primary_xml = _make_xml()
        overlay_xml = _make_xml(show_raw=True, show_raw_pelvis_axes=True)

        self.assertIn('name="pelvis_axes"', primary_xml)
        self.assertIn('name="pelvis_axes_free" type="free"', primary_xml)
        self.assertNotIn('name="raw_pelvis_axes"', primary_xml)
        self.assertIn('name="raw_pelvis_axes"', overlay_xml)
        self.assertIn('name="raw_pelvis_axes_free" type="free"', overlay_xml)
        self.assertIn(
            'name="raw_pelvis_axis_x" class="raw_pelvis_axis" '
            'fromto="0 0 0 .18 0 0" rgba="1 0 0 0"',
            overlay_xml,
        )
        self.assertEqual(overlay_xml.count('class="pelvis_axis"'), 4)
        self.assertEqual(overlay_xml.count('class="raw_pelvis_axis"'), 4)

    def test_raw_overlay_has_no_axes_by_default(self):
        xml = _make_xml(show_raw=True)

        self.assertNotIn('name="raw_pelvis_axes"', xml)
        self.assertNotIn('name="raw_arm_axes_16"', xml)

    def test_foot_sole_height_uses_oriented_box_extent(self):
        identity_height = smpl_viewer.foot_sole_height(
            np.array([0.0, 0.0, 1.0]),
            np.array([1.0, 0.0, 0.0, 0.0]),
        )
        pitch_90_height = smpl_viewer.foot_sole_height(
            np.array([0.0, 0.0, 1.0]),
            np.array([math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0]),
        )

        self.assertAlmostEqual(identity_height, 0.975)
        self.assertAlmostEqual(pitch_90_height, 0.88)

    def test_ground_plane_estimator_locks_stable_window_median_once(self):
        estimator = smpl_viewer.GroundPlaneEstimator(
            window_size=3, stability_tolerance=0.02
        )

        self.assertIsNone(estimator.observe(-1.001, -0.999))
        self.assertIsNone(estimator.observe(-1.000, -0.998))
        locked = estimator.observe(-1.002, -1.000)

        self.assertAlmostEqual(locked, -1.0)
        self.assertAlmostEqual(estimator.height, -1.0)
        self.assertAlmostEqual(estimator.observe(0.4, 0.5), -1.0)
        self.assertAlmostEqual(estimator.height, -1.0)

    def test_default_ground_estimator_requires_thirty_complete_frames(self):
        estimator = smpl_viewer.GroundPlaneEstimator()

        for _ in range(29):
            self.assertIsNone(estimator.observe(-1.0, -1.0))
        self.assertAlmostEqual(estimator.observe(-1.0, -1.0), -1.0)

    def test_ground_plane_estimator_rejects_unstable_window_and_resets(self):
        estimator = smpl_viewer.GroundPlaneEstimator(
            window_size=3, stability_tolerance=0.02
        )
        estimator.observe(-1.0, -1.0)
        estimator.observe(-0.9, -1.0)
        self.assertIsNone(estimator.observe(-1.0, -1.0))
        self.assertIsNone(estimator.height)

        estimator.reset()
        estimator.observe(-0.51, -0.50)
        estimator.observe(-0.50, -0.49)
        self.assertAlmostEqual(estimator.observe(-0.50, -0.50), -0.5)

    def test_floor_source_prefers_raw_callback_when_overlay_is_enabled(self):
        calls = []
        positions = [np.array([0.0, 0.0, 1.0]) for _ in JOINT_NAMES]
        orientations = [np.array([1.0, 0.0, 0.0, 0.0]) for _ in JOINT_NAMES]
        visualizer = SmplMujocoVisualizer.__new__(SmplMujocoVisualizer)
        visualizer.show_raw = True
        visualizer.timeout = 1.0
        visualizer._ground_reset_at = 0.0
        visualizer._primary_cache = PoseFrameCache()
        visualizer._raw_cache = PoseFrameCache()
        visualizer._convert_pose_array = lambda _message, stream: (
            positions,
            orientations,
        )
        visualizer._feed_floor = lambda source, pos, ori: calls.append(source)

        visualizer._raw_pose_callback(object())
        self.assertEqual(calls, ["raw"])
        visualizer._pose_callback(object())
        self.assertEqual(calls, ["raw"])

    def test_floor_source_immediately_uses_primary_when_raw_is_absent(self):
        calls = []
        positions = [np.array([0.0, 0.0, 1.0]) for _ in JOINT_NAMES]
        orientations = [np.array([1.0, 0.0, 0.0, 0.0]) for _ in JOINT_NAMES]
        visualizer = SmplMujocoVisualizer.__new__(SmplMujocoVisualizer)
        visualizer.show_raw = True
        visualizer.timeout = 1.0
        visualizer._ground_reset_at = 0.0
        visualizer._primary_cache = PoseFrameCache()
        visualizer._raw_cache = PoseFrameCache()
        visualizer._convert_pose_array = lambda _message, stream: (
            positions,
            orientations,
        )
        visualizer._feed_floor = lambda source, pos, ori: calls.append(source)

        visualizer._pose_callback(object())

        self.assertEqual(calls, ["primary"])

    def test_raw_floor_source_uses_raw_foot_box_half_extents(self):
        observations = []
        visualizer = SmplMujocoVisualizer.__new__(SmplMujocoVisualizer)
        visualizer._ground_estimator = SimpleNamespace(
            height=None,
            reset=lambda: None,
            observe=lambda left, right: observations.append((left, right)),
        )
        visualizer._floor_source = None
        positions = [np.array([0.0, 0.0, 1.0]) for _ in JOINT_NAMES]
        orientations = [np.array([1.0, 0.0, 0.0, 0.0]) for _ in JOINT_NAMES]

        visualizer._feed_floor("raw", positions, orientations)

        self.assertEqual(len(observations), 1)
        self.assertAlmostEqual(observations[0][0], 0.978)
        self.assertAlmostEqual(observations[0][1], 0.978)

    def test_zero_display_timeout_does_not_prevent_raw_floor_lock(self):
        positions = [np.array([0.0, 0.0, 1.0]) for _ in JOINT_NAMES]
        orientations = [np.array([1.0, 0.0, 0.0, 0.0]) for _ in JOINT_NAMES]
        visualizer = SmplMujocoVisualizer.__new__(SmplMujocoVisualizer)
        visualizer.show_raw = True
        visualizer.timeout = 0.0
        visualizer._ground_reset_at = 0.0
        visualizer._ground_estimator = smpl_viewer.GroundPlaneEstimator(
            window_size=3
        )
        visualizer._floor_source = None
        visualizer._primary_cache = PoseFrameCache()
        visualizer._raw_cache = PoseFrameCache()
        visualizer._convert_pose_array = lambda _message, stream: (
            positions,
            orientations,
        )

        for _ in range(3):
            visualizer._raw_pose_callback(object())
            visualizer._pose_callback(object())

        self.assertIsNotNone(visualizer._ground_estimator.height)
        self.assertEqual(visualizer._floor_source, "raw")

    def test_floor_source_uses_primary_callback_without_overlay(self):
        calls = []
        positions = [np.array([0.0, 0.0, 1.0]) for _ in JOINT_NAMES]
        orientations = [np.array([1.0, 0.0, 0.0, 0.0]) for _ in JOINT_NAMES]
        visualizer = SmplMujocoVisualizer.__new__(SmplMujocoVisualizer)
        visualizer.show_raw = False
        visualizer._primary_cache = PoseFrameCache()
        visualizer._convert_pose_array = lambda _message, stream: (
            positions,
            orientations,
        )
        visualizer._feed_floor = lambda source, pos, ori: calls.append(source)

        visualizer._pose_callback(object())

        self.assertEqual(calls, ["primary"])

    def test_locked_floor_does_not_change_when_preferred_source_changes(self):
        visualizer = SmplMujocoVisualizer.__new__(SmplMujocoVisualizer)
        visualizer._ground_estimator = smpl_viewer.GroundPlaneEstimator(window_size=1)
        visualizer._floor_source = None
        positions = [np.array([0.0, 0.0, 1.0]) for _ in JOINT_NAMES]
        orientations = [np.array([1.0, 0.0, 0.0, 0.0]) for _ in JOINT_NAMES]

        visualizer._feed_floor("primary", positions, orientations)
        locked = visualizer._ground_estimator.height
        positions[10] = np.array([0.0, 0.0, 5.0])
        positions[11] = np.array([0.0, 0.0, 5.0])
        visualizer._feed_floor("raw", positions, orientations)

        self.assertEqual(visualizer._ground_estimator.height, locked)
        self.assertEqual(visualizer._floor_source, "primary")

    def test_world_reset_clears_locked_floor(self):
        visualizer = SmplMujocoVisualizer.__new__(SmplMujocoVisualizer)
        visualizer._ground_estimator = smpl_viewer.GroundPlaneEstimator(window_size=1)
        visualizer._ground_estimator.observe(-1.0, -1.0)
        visualizer._floor_source = "raw"
        visualizer._raw_cache = PoseFrameCache()
        visualizer._raw_cache.update(
            [np.zeros(3)], [np.array([1.0, 0.0, 0.0, 0.0])], time.monotonic()
        )
        visualizer._ground_reset_at = 0.0
        visualizer.node = SimpleNamespace(
            get_logger=lambda: SimpleNamespace(info=lambda _message: None)
        )

        visualizer._world_reset_callback(object())

        self.assertIsNone(visualizer._ground_estimator.height)
        self.assertIsNone(visualizer._floor_source)
        self.assertFalse(visualizer._raw_floor_is_preferred())

    def test_apply_ground_plane_moves_and_reveals_reference_geometry(self):
        visualizer = SmplMujocoVisualizer.__new__(SmplMujocoVisualizer)
        visualizer._ground_estimator = smpl_viewer.GroundPlaneEstimator(window_size=1)
        visualizer._ground_estimator.observe(-1.25, -1.25)
        visualizer.model = SimpleNamespace(
            body_pos=np.zeros((1, 3)),
            geom_rgba=np.zeros((4, 4)),
        )
        visualizer.ground_body_id = 0
        visualizer.ground_geom_id = 0
        visualizer.world_axis_geom_ids = [1, 2, 3]
        visualizer._world_axis_colors = (
            np.array([1.0, 0.0, 0.0, 1.0]),
            np.array([0.0, 1.0, 0.0, 1.0]),
            np.array([0.1, 0.35, 1.0, 1.0]),
        )

        self.assertTrue(visualizer._apply_ground_plane())
        self.assertAlmostEqual(visualizer.model.body_pos[0, 2], -1.25)
        self.assertEqual(visualizer.model.geom_rgba[0, 3], 1.0)
        np.testing.assert_allclose(
            visualizer.model.geom_rgba[1:4], visualizer._world_axis_colors
        )

        visualizer._ground_estimator.reset()
        self.assertFalse(visualizer._apply_ground_plane())
        self.assertEqual(visualizer.model.geom_rgba[0, 3], 0.0)
        np.testing.assert_allclose(visualizer.model.geom_rgba[1:4, 3], 0.0)

    def test_camera_initialization_holds_passive_viewer_lock(self):
        state = {"entered": False, "exited": False}

        class RecordingLock:
            def __enter__(self):
                state["entered"] = True

            def __exit__(self, *_args):
                state["exited"] = True

        viewer = SimpleNamespace(
            cam=SimpleNamespace(
                lookat=np.zeros(3), distance=0.0, azimuth=0.0, elevation=0.0
            ),
            lock=lambda: RecordingLock(),
        )
        visualizer = SmplMujocoVisualizer.__new__(SmplMujocoVisualizer)

        visualizer._initialize_camera(viewer, np.array([1.0, 2.0, 3.0]))

        self.assertTrue(state["entered"])
        self.assertTrue(state["exited"])
        np.testing.assert_allclose(viewer.cam.lookat, [1.0, 2.0, 3.0])
        self.assertEqual(viewer.cam.distance, 2.5)

    def test_scene_update_holds_passive_viewer_lock(self):
        state = {"entered": False, "exited": False, "updated": False}

        class RecordingLock:
            def __enter__(self):
                state["entered"] = True

            def __exit__(self, *_args):
                state["exited"] = True

        visualizer = SmplMujocoVisualizer.__new__(SmplMujocoVisualizer)

        def update_scene():
            self.assertTrue(state["entered"])
            self.assertFalse(state["exited"])
            state["updated"] = True
            return True

        visualizer._update_scene = update_scene
        viewer = SimpleNamespace(lock=lambda: RecordingLock())

        self.assertTrue(visualizer._update_viewer_scene(viewer))
        self.assertTrue(state["updated"])
        self.assertTrue(state["exited"])

    def test_raw_overlay_xml_uses_high_contrast_magenta_geometry(self):
        xml = _make_xml(show_raw=True)

        self.assertIn('name="raw_joint_0"', xml)
        self.assertIn('name="raw_bone_0"', xml)
        self.assertIn('name="raw_foot_left"', xml)
        self.assertIn('name="raw_foot_right"', xml)
        self.assertIn('name="raw_joint_geom_0" type="sphere" size="0.024"', xml)
        self.assertIn('name="raw_bone_geom_0" type="capsule"', xml)
        self.assertIn('size="0.012 0.5" rgba="1 0.05 0.8 0.82"', xml)

    def test_topology_has_24_joints_and_23_edges(self):
        self.assertEqual(len(JOINT_NAMES), 24)
        self.assertEqual(len(set(JOINT_NAMES)), 24)
        self.assertEqual(len(BONE_EDGES), 23)
        self.assertIn((0, 1), BONE_EDGES)
        self.assertTrue(all(parent < 24 and child < 24 for parent, child in BONE_EDGES))

    def test_identity_transform(self):
        np.testing.assert_allclose(transform_position((1, 2, 3)), (1, 2, 3))

    def test_scale_and_yaw(self):
        result = transform_position((1, 0, 0), scale=2.0, yaw=math.pi / 2)
        np.testing.assert_allclose(result, (0, 2, 0), atol=1e-7)

    def test_orientation_converts_xyzw_to_wxyz_and_applies_view_yaw(self):
        result = transform_orientation((math.sin(math.pi / 4), 0, 0, math.cos(math.pi / 4)), yaw=math.pi / 2)
        expected = np.array([0.5, 0.5, 0.5, 0.5])
        np.testing.assert_allclose(result, expected, atol=1e-7)

    def test_zero_length_capsule(self):
        center, quat, length = capsule_pose((1, 2, 3), (1, 2, 3))
        np.testing.assert_allclose(center, (1, 2, 3))
        np.testing.assert_allclose(quat, (1, 0, 0, 0))
        self.assertEqual(length, 0.0)

    def test_vertical_capsule(self):
        center, quat, length = capsule_pose((0, 0, 0), (0, 0, 2))
        np.testing.assert_allclose(center, (0, 0, 1))
        np.testing.assert_allclose(quat, (1, 0, 0, 0))
        self.assertEqual(length, 2.0)

    def test_wrist_pivot_wins_over_stale_topic_and_primary_pose(self):
        palm_position = np.array([1.0, 2.0, 3.0])
        palm_orientation = np.array([1.0, 0.0, 0.0, 0.0])
        wrist_position, wrist_orientation, received_at, source = (
            resolve_visualization_wrist_pose(
                explicit_position=np.array([8.0, 8.0, 8.0]),
                explicit_orientation=np.array([1.0, 0.0, 0.0, 0.0]),
                explicit_received_at=9.9,
                now=10.0,
                timeout=1.0,
                palm_position=palm_position,
                palm_orientation=palm_orientation,
                palm_received_at=9.5,
                wrist_to_palm=0.08,
                primary_position=np.array([9.0, 9.0, 9.0]),
                primary_orientation=np.array([1.0, 0.0, 0.0, 0.0]),
                primary_received_at=9.9,
            )
        )
        self.assertEqual(source, "wrist_pivot")
        np.testing.assert_allclose(wrist_position, [0.92, 2.0, 3.0])
        np.testing.assert_allclose(wrist_orientation, palm_orientation)
        self.assertEqual(received_at, 9.5)


if __name__ == "__main__":
    unittest.main()
