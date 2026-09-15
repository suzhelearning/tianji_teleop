"""实物数据手套 URDF 骨架叠层合同。"""

from __future__ import annotations

import unittest
from pathlib import Path

import mujoco
import numpy as np

from data_glove_wuji_teleop.adapters.simulation.dataglove_skeleton import (
    HandSkeleton,
)
from data_glove_wuji_teleop.adapters.simulation.mujoco_dataglove import (
    load_dataglove_urdf,
)


PROJECT_ROOT = Path(__file__).resolve().parents[2]
URDF_PATH = (
    PROJECT_ROOT
    / "assets/data_glove_urdf/data_glove_urdf_R/urdf/data_glove_urdf_R.urdf"
)
EXPECTED_CHAINS = (
    (
        "thumb",
        (
            "thumb_cmc_flex_joint",
            "thumb_cmc_abd_joint",
            "thumb_mcp_joint",
            "thumb_ip_joint",
        ),
    ),
    (
        "index",
        (
            "index_mcp_abd_joint",
            "index_mcp_flex_joint",
            "index_pip_joint",
            "index_dip_joint",
        ),
    ),
    (
        "middle",
        (
            "middle_mcp_abd_joint",
            "middle_mcp_flex_joint",
            "middle_pip_joint",
            "middle_dip_joint",
        ),
    ),
    (
        "ring",
        (
            "ring_mcp_abd_joint",
            "ring_mcp_flex_joint",
            "ring_pip_joint",
            "ring_dip_joint",
        ),
    ),
    (
        "pinky",
        (
            "pinky_cmc_joint",
            "pinky_mcp_abd_joint",
            "pink_mcp_flex_joint",
            "pink_pip_joint",
            "pink_dip_joint",
        ),
    ),
)


class HandSkeletonTest(unittest.TestCase):
    def test_thumb_tip_is_five_millimeters_longer_than_other_tips(
        self,
    ) -> None:
        model, data, _qpos = load_dataglove_urdf(URDF_PATH)
        skeleton = HandSkeleton.from_model(model)
        mujoco.mj_forward(model, data)

        fingertip_lengths = {
            chain.name: float(
                np.linalg.norm(
                    skeleton.fingertip_position(data, chain)
                    - data.xanchor[chain.joint_ids[-1]]
                )
            )
            for chain in skeleton.chains
        }

        self.assertAlmostEqual(fingertip_lengths["thumb"], 0.030)
        for name in ("index", "middle", "ring", "pinky"):
            self.assertAlmostEqual(fingertip_lengths[name], 0.025)

    def test_rejects_degenerate_thumb_reference_direction(self) -> None:
        model, _data, _qpos = load_dataglove_urdf(URDF_PATH)
        thumb_ip_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            "thumb_ip_joint",
        )
        thumb_ip_body_id = int(model.jnt_bodyid[thumb_ip_id])
        model.body_pos[thumb_ip_body_id] = 0.0

        with self.assertRaisesRegex(ValueError, "无法确定指尖方向"):
            HandSkeleton.from_model(model)

    def test_thumb_tip_uses_parent_direction_at_zero_position(self) -> None:
        model, data, _qpos = load_dataglove_urdf(URDF_PATH)
        skeleton = HandSkeleton.from_model(model)
        mujoco.mj_forward(model, data)
        thumb = skeleton.chains[0]
        terminal_anchor = data.xanchor[thumb.joint_ids[-1]]
        parent_anchor = data.xanchor[thumb.joint_ids[-2]]
        expected_direction = terminal_anchor - parent_anchor
        expected_direction /= np.linalg.norm(expected_direction)

        tip_direction = (
            skeleton.fingertip_position(data, thumb) - terminal_anchor
        )
        tip_direction /= np.linalg.norm(tip_direction)

        np.testing.assert_allclose(
            tip_direction,
            expected_direction,
            atol=1e-8,
        )

    def test_thumb_tip_direction_follows_ip_body_rotation(self) -> None:
        model, data, _qpos = load_dataglove_urdf(URDF_PATH)
        skeleton = HandSkeleton.from_model(model)
        thumb = skeleton.chains[0]
        mujoco.mj_forward(model, data)
        terminal_anchor = data.xanchor[thumb.joint_ids[-1]]
        zero_direction = (
            skeleton.fingertip_position(data, thumb) - terminal_anchor
        )
        zero_direction /= np.linalg.norm(zero_direction)
        zero_rotation = data.xmat[thumb.terminal_body_id].reshape(3, 3).copy()
        local_reference = zero_rotation.T @ zero_direction

        terminal_qpos = int(model.jnt_qposadr[thumb.joint_ids[-1]])
        data.qpos[terminal_qpos] = 0.4
        mujoco.mj_forward(model, data)
        terminal_anchor = data.xanchor[thumb.joint_ids[-1]]
        moved_direction = (
            skeleton.fingertip_position(data, thumb) - terminal_anchor
        )
        moved_direction /= np.linalg.norm(moved_direction)
        expected_direction = (
            data.xmat[thumb.terminal_body_id].reshape(3, 3)
            @ local_reference
        )

        self.assertGreater(
            float(np.linalg.norm(moved_direction - zero_direction)),
            1e-3,
        )
        np.testing.assert_allclose(
            moved_direction,
            expected_direction,
            atol=1e-8,
        )

    def test_extends_fingers_from_terminal_body_orientation(
        self,
    ) -> None:
        model, data, _qpos = load_dataglove_urdf(URDF_PATH)
        skeleton = HandSkeleton.from_model(model)
        for chain in skeleton.chains:
            terminal_joint_id = chain.joint_ids[-1]
            terminal_qpos = int(model.jnt_qposadr[terminal_joint_id])
            data.qpos[terminal_qpos] = 0.4
        mujoco.mj_forward(model, data)
        scene = mujoco.MjvScene(model, maxgeom=64)

        skeleton.update_scene(data, scene)

        sphere_positions = np.array(
            [
                scene.geoms[index].pos.copy()
                for index in range(scene.ngeom)
                if int(scene.geoms[index].type)
                == int(mujoco.mjtGeom.mjGEOM_SPHERE)
            ]
        )
        terminal_segments: list[tuple[np.ndarray, np.ndarray]] = []
        for chain in skeleton.chains:
            terminal_joint_id = chain.joint_ids[-1]
            terminal_body_id = int(model.jnt_bodyid[terminal_joint_id])
            terminal_rotation = data.xmat[terminal_body_id].reshape(3, 3)
            terminal_anchor = data.xanchor[terminal_joint_id]
            expected_tip = skeleton.fingertip_position(data, chain)
            distances = np.linalg.norm(
                sphere_positions - expected_tip,
                axis=1,
            )
            observed_tip = sphere_positions[int(distances.argmin())]
            terminal_segments.append((terminal_anchor, observed_tip))
            self.assertLess(float(distances.min()), 1e-8, chain.name)
            self.assertAlmostEqual(
                float(np.linalg.norm(observed_tip - terminal_anchor)),
                0.030 if chain.name == "thumb" else 0.025,
                places=7,
            )
            if chain.name != "thumb":
                local_extension = terminal_rotation.T @ (
                    observed_tip - terminal_anchor
                )
                np.testing.assert_allclose(
                    local_extension,
                    (0.0, 0.0, 0.025),
                    atol=1e-8,
                    err_msg=chain.name,
                )
        capsules = [
            scene.geoms[index]
            for index in range(scene.ngeom)
            if int(scene.geoms[index].type)
            == int(mujoco.mjtGeom.mjGEOM_CAPSULE)
        ]
        for terminal_anchor, observed_tip in terminal_segments:
            errors = []
            for capsule in capsules:
                half_length = float(capsule.size[2])
                axis = capsule.mat.reshape(3, 3)[:, 2]
                start = capsule.pos - half_length * axis
                end = capsule.pos + half_length * axis
                errors.append(
                    min(
                        np.linalg.norm(start - terminal_anchor)
                        + np.linalg.norm(end - observed_tip),
                        np.linalg.norm(end - terminal_anchor)
                        + np.linalg.norm(start - observed_tip),
                    )
                )
            self.assertLess(float(min(errors)), 2e-8)

    def test_rejects_joint_chain_that_is_not_parent_child_connected(
        self,
    ) -> None:
        model, _data, _qpos = load_dataglove_urdf(URDF_PATH)
        index_pip_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            "index_pip_joint",
        )
        index_pip_body_id = int(model.jnt_bodyid[index_pip_id])
        model.body_parentid[index_pip_body_id] = 0

        with self.assertRaisesRegex(ValueError, "父子 body 顺序"):
            HandSkeleton.from_model(model)

    def test_builds_five_parent_child_chains_in_urdf_order(self) -> None:
        model, _data, _qpos = load_dataglove_urdf(URDF_PATH)

        skeleton = HandSkeleton.from_model(model)

        self.assertEqual(
            tuple(
                (chain.name, chain.joint_names)
                for chain in skeleton.chains
            ),
            EXPECTED_CHAINS,
        )

    def test_draws_joint_nodes_bones_and_palm_frame(self) -> None:
        model, data, _qpos = load_dataglove_urdf(URDF_PATH)
        mujoco.mj_forward(model, data)
        skeleton = HandSkeleton.from_model(model)
        scene = mujoco.MjvScene(model, maxgeom=64)

        skeleton.update_scene(data, scene)

        geom_types = tuple(
            int(scene.geoms[index].type) for index in range(scene.ngeom)
        )
        self.assertEqual(scene.ngeom, 53)
        self.assertEqual(
            geom_types.count(int(mujoco.mjtGeom.mjGEOM_SPHERE)),
            27,
        )
        self.assertEqual(
            geom_types.count(int(mujoco.mjtGeom.mjGEOM_CAPSULE)),
            26,
        )
        sphere_colors = {
            tuple(round(float(value), 3) for value in scene.geoms[index].rgba)
            for index, geom_type in enumerate(geom_types)
            if geom_type == int(mujoco.mjtGeom.mjGEOM_SPHERE)
        }
        self.assertEqual(len(sphere_colors), 6)

    def test_applies_same_rigid_display_transform_to_skeleton(self) -> None:
        model, data, _qpos = load_dataglove_urdf(URDF_PATH)
        mujoco.mj_forward(model, data)
        skeleton = HandSkeleton.from_model(model)
        scene = mujoco.MjvScene(model, maxgeom=64)
        rotation = np.array(
            (
                (0.0, -1.0, 0.0),
                (1.0, 0.0, 0.0),
                (0.0, 0.0, 1.0),
            )
        )
        offset = np.array((-0.22, 0.01, 0.02))
        first_joint_id = skeleton.chains[0].joint_ids[0]

        skeleton.update_scene(
            data,
            scene,
            world_offset=offset,
            world_rotation=rotation,
        )

        np.testing.assert_allclose(
            scene.geoms[0].pos,
            rotation @ data.xanchor[first_joint_id] + offset,
            atol=1e-8,
        )

    def test_rejects_non_rotation_display_matrix(self) -> None:
        model, data, _qpos = load_dataglove_urdf(URDF_PATH)
        mujoco.mj_forward(model, data)
        skeleton = HandSkeleton.from_model(model)
        scene = mujoco.MjvScene(model, maxgeom=64)

        with self.assertRaisesRegex(ValueError, "3x3 旋转矩阵"):
            skeleton.update_scene(
                data,
                scene,
                world_rotation=np.diag((1.0, 1.0, -1.0)),
            )

    def test_rejects_user_scene_without_capacity_for_complete_skeleton(
        self,
    ) -> None:
        model, data, _qpos = load_dataglove_urdf(URDF_PATH)
        mujoco.mj_forward(model, data)
        skeleton = HandSkeleton.from_model(model)
        scene = mujoco.MjvScene(model, maxgeom=52)

        with self.assertRaisesRegex(ValueError, "需要 53.*仅允许 52"):
            skeleton.update_scene(data, scene)

    def test_refresh_replaces_previous_geoms_and_tracks_joint_motion(
        self,
    ) -> None:
        model, data, qpos_by_joint = load_dataglove_urdf(URDF_PATH)
        skeleton = HandSkeleton.from_model(model)
        scene = mujoco.MjvScene(model, maxgeom=64)
        mujoco.mj_forward(model, data)
        skeleton.update_scene(data, scene)
        first_positions = np.array(
            [scene.geoms[index].pos.copy() for index in range(scene.ngeom)]
        )

        data.qpos[qpos_by_joint["index_mcp_flex_joint"]] = 0.6
        mujoco.mj_forward(model, data)
        skeleton.update_scene(data, scene)
        second_positions = np.array(
            [scene.geoms[index].pos.copy() for index in range(scene.ngeom)]
        )

        self.assertEqual(scene.ngeom, 53)
        self.assertFalse(np.allclose(first_positions, second_positions))

    def test_makes_mesh_translucent_without_increasing_existing_alpha(
        self,
    ) -> None:
        model, _data, _qpos = load_dataglove_urdf(URDF_PATH)
        skeleton = HandSkeleton.from_model(model)
        model.geom_rgba[0, 3] = 0.1

        skeleton.make_mesh_translucent(model, max_alpha=0.2)

        self.assertAlmostEqual(float(model.geom_rgba[0, 3]), 0.1)
        self.assertLessEqual(float(model.geom_rgba[:, 3].max()), 0.200001)


if __name__ == "__main__":
    unittest.main()
