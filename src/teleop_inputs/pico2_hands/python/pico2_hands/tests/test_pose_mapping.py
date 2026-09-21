from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from pico2_hands.reference.models import ArmInputObservation
from pico2_hands.reference.pose_mapping import (
    create_arm_pose_mapper,
)


def _input(
    pose,
    side="right",
    *,
    tracked_frame="wrist",
    reference_frame="pico_head_current",
) -> ArmInputObservation:
    return ArmInputObservation(
        source="pico",
        side=side,
        tracked_frame=tracked_frame,
        reference_frame=reference_frame,
        pose=np.asarray(pose, dtype=np.float64),
        valid=True,
        source_timestamp_ns=1,
        received_timestamp_ns=2,
        receiver_instance_id="receiver",
        receiver_frame_sequence=3,
        mapping_version="input-v1",
        frame_association_id="receiver:1:3",
    )


class PoseMappingFactoryTest(unittest.TestCase):
    def test_pico_config_keeps_hand_center_fixed_during_wrist_rotation(self):
        import yaml
        path = Path(__file__).resolve().parents[1] / "config/hand_tracking_target.yaml"
        config = yaml.safe_load(path.read_text())["arm_pose_mapper_config"]
        mapper = create_arm_pose_mapper("relative_home", config)
        offset = np.array([0, 0, 0.0365])
        for side in ("left", "right"):
            reference = _input([0, 0, 0, 0, 0, 0, 1], side=side)
            mapper.initialize(reference)
            home = np.asarray(config["home_pose"][side])
            np.testing.assert_allclose(mapper.map(reference).pose[:3], home[:3], atol=1e-12)
            result = mapper.map(_input([0, 0, 0, *Rotation.from_euler("x", 90, degrees=True).as_quat()], side=side))
            np.testing.assert_allclose(result.pose[:3] + Rotation.from_quat(result.pose[3:]).apply(offset),
                                       home[:3] + Rotation.from_quat(home[3:]).apply(offset), atol=1e-12)
            self.assertGreater(np.linalg.norm(result.pose[:3] - home[:3]), 0.001)

    def test_relative_home_rotates_about_hand_center_without_initial_jump(self):
        for side in ("left", "right"):
            with self.subTest(side=side):
                home_rotation = Rotation.from_euler("xyz", [20, -35, 15], degrees=True)
                home_position = np.array([0.4, 0.2, 0.3])
                offset = np.array([0, 0, 0.0365])
                mapper = create_arm_pose_mapper("relative_home", {
                    "home_pose": {side: [*home_position, *home_rotation.as_quat()]},
                    "ik_tcp_to_control_pose": {side: [*offset, 0, 0, 0, 1]},
                })
                reference = _input([0, 0, 0, 0, 0, 0, 1], side=side)
                mapper.initialize(reference)
                initial = mapper.map(reference)
                np.testing.assert_allclose(initial.pose[:3], home_position, atol=1e-12)
                delta = Rotation.from_euler("y", 90, degrees=True)
                translation = np.array([0.02, -0.03, 0.01])
                result = mapper.map(_input([*translation, *delta.as_quat()], side=side))
                result_rotation = Rotation.from_quat(result.pose[3:])
                # Recover hand-center position from the emitted IK TCP pose.
                expected_center = home_position + home_rotation.apply(offset) + translation
                np.testing.assert_allclose(result.pose[:3] + result_rotation.apply(offset),
                                           expected_center, atol=1e-12)
                np.testing.assert_allclose(result_rotation.as_matrix(),
                                           (delta * home_rotation).as_matrix(), atol=1e-12)

    def test_direct_pose_composes_reference_tracked_and_tcp_transforms(self) -> None:
        mapper = create_arm_pose_mapper(
            "direct_pose",
            {
                "input_to_base_rotation": {"right": np.eye(3)},
                "input_origin_in_base_m": {"right": [1.0, 2.0, 3.0]},
                "tracked_to_tcp_pose": {"right": [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]},
            },
        )
        result = mapper.map(_input([0.2, 0.3, 0.4, 0.0, 0.0, 0.0, 1.0]))

        np.testing.assert_allclose(result.pose[:3], [1.3, 2.3, 3.4])
        np.testing.assert_allclose(result.pose[3:], [0.0, 0.0, 0.0, 1.0])
        self.assertTrue(result.valid)

    def test_relative_home_requires_initialization_and_resets_state(self) -> None:
        mapper = create_arm_pose_mapper(
            "relative_home",
            {
                "home_pose": {"right": [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]},
                "input_to_base_rotation": {"right": np.eye(3)},
            },
        )
        current = _input([0.4, 0.5, 0.6, 0.0, 0.0, 0.0, 1.0])
        with self.assertRaises(RuntimeError):
            mapper.map(current)

        mapper.initialize(current)
        initial = mapper.map(current)
        np.testing.assert_allclose(initial.pose[:3], [1.0, 2.0, 3.0])
        moved = mapper.map(_input([0.5, 0.5, 0.4, 0.0, 0.0, 0.0, 1.0]))
        np.testing.assert_allclose(moved.pose[:3], [1.1, 2.0, 2.8])
        mapper.reset()
        with self.assertRaises(RuntimeError):
            mapper.map(current)

    def test_factory_rejects_unknown_backend(self) -> None:
        with self.assertRaises(ValueError):
            create_arm_pose_mapper("does_not_exist", {})

    def test_direct_pose_checks_reference_and_tracked_frame_contracts(self) -> None:
        mapper = create_arm_pose_mapper(
            "direct_pose",
            {
                "expected_reference_frame": {"right": "pico_head_current"},
                "expected_tracked_frame": {"right": "wrist"},
            },
        )
        mapper.map(_input([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]))
        wrong_reference = _input(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            reference_frame="legacy_pico_tracking",
        )
        with self.assertRaises(ValueError):
            mapper.map(wrong_reference)

    def test_relative_home_can_reproduce_historical_side_specific_chest_mapping(self) -> None:
        # TJ_arm_control's right-hand world->chest conversion is
        # [x, z, -y].  Folding that matrix into the mapper configuration must
        # preserve both the translated position and the rotated orientation.
        world_to_chest_right = np.array(
            [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]],
        )
        mapper = create_arm_pose_mapper(
            "relative_home",
            {
                "home_pose": {"right": [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]},
                "input_to_base_rotation": {"right": world_to_chest_right},
            },
        )
        reference = _input(
            [0.4, 0.5, 0.6, 0.0, 0.0, 0.0, 1.0],
            side="right",
        )
        current_rotation = Rotation.from_euler("x", 35.0, degrees=True)
        current = _input(
            [0.5, 0.5, 0.4, *current_rotation.as_quat()],
            side="right",
        )

        mapper.initialize(reference)
        result = mapper.map(current)

        np.testing.assert_allclose(result.pose[:3], [1.1, 1.8, 3.0], atol=1.0e-12)
        expected_rotation = Rotation.from_matrix(
            world_to_chest_right
            @ current_rotation.as_matrix()
            @ world_to_chest_right.T
        )
        np.testing.assert_allclose(
            Rotation.from_quat(result.pose[3:]).as_matrix(),
            expected_rotation.as_matrix(),
            atol=1.0e-12,
        )


if __name__ == "__main__":
    unittest.main()
