"""Wuji 二代手语义 DOF 到 MuJoCo 执行器的合同测试。"""

from __future__ import annotations

import unittest

import mujoco

from data_glove_wuji_teleop.adapters.simulation.mujoco_wuji_v2 import (
    apply_dof_target,
    build_control_bindings,
    load_model,
)
from data_glove_wuji_teleop.project import get_hand_resources


class WujiV2SimulationTest(unittest.TestCase):
    def test_model_is_upright_in_the_default_view(self) -> None:
        for hand, prefix in (("left", "l_"), ("right", "r_")):
            resources = get_hand_resources(hand, generation="v2")
            model, data = load_model(resources.mjcf)
            mujoco.mj_forward(model, data)

            camera = mujoco.MjvCamera()
            mujoco.mjv_defaultFreeCamera(model, camera)
            camera.lookat[:] = data.xpos[1:].mean(axis=0)
            camera.distance = 0.5
            camera.azimuth = 90.0
            camera.elevation = -20.0
            scene = mujoco.MjvScene(model, maxgeom=1000)
            option = mujoco.MjvOption()
            mujoco.mjv_updateScene(
                model,
                data,
                option,
                None,
                camera,
                mujoco.mjtCatBit.mjCAT_ALL,
                scene,
            )

            proximal_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_BODY,
                f"{prefix}middle_finger_proximal",
            )
            distal_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_BODY,
                f"{prefix}middle_finger_distal",
            )
            hand_axis = data.xpos[distal_id] - data.xpos[proximal_id]
            screen_up_delta = float(hand_axis @ scene.camera[0].up)
            self.assertGreater(
                screen_up_delta,
                0.0,
                f"{hand} 二代手在默认相机中仍然倒置",
            )

            tip_x = []
            for finger in (
                "thumb",
                "index_finger",
                "middle_finger",
                "ring_finger",
                "pinky",
            ):
                body_id = mujoco.mj_name2id(
                    model,
                    mujoco.mjtObj.mjOBJ_BODY,
                    f"{prefix}{finger}_distal",
                )
                tip_x.append(float(data.xpos[body_id, 0]))
            self.assertEqual(
                tip_x,
                sorted(tip_x, reverse=True),
                f"{hand} 二代手五指横向顺序被翻转",
            )

    def test_binds_all_semantic_dofs_to_unique_actuators(self) -> None:
        for hand, prefix in (("left", "l_"), ("right", "r_")):
            resources = get_hand_resources(hand, generation="v2")
            model, _ = load_model(resources.mjcf)
            bindings = build_control_bindings(
                model,
                hand=hand,
                config_path=resources.mapping,
            )

            self.assertEqual(len(bindings), 20)
            self.assertEqual(len({item.actuator_id for item in bindings}), 20)
            by_dof = {item.dof_name: item for item in bindings}
            self.assertEqual(
                by_dof["thumb1_flex"].joint_name,
                f"{prefix}thumb_cmc_flex",
            )
            self.assertEqual(
                by_dof["index1_abd"].joint_name,
                f"{prefix}index_finger_mcp_abd",
            )
            # 左手现场 mapping 已放大拇指末两段，右手仍保持镜像基线。
            expected_thumb_scale = -1.0 if hand == "right" else 1.5
            self.assertEqual(
                by_dof["thumb2_flex"].scale,
                expected_thumb_scale,
            )
            self.assertEqual(
                by_dof["thumb3_flex"].scale,
                expected_thumb_scale,
            )

    def test_applies_and_clips_targets_to_official_actuator_limits(self) -> None:
        resources = get_hand_resources("left", generation="v2")
        model, data = load_model(resources.mjcf)
        bindings = build_control_bindings(
            model,
            hand="left",
            config_path=resources.mapping,
        )
        by_dof = {item.dof_name: item for item in bindings}

        apply_dof_target(
            data,
            bindings,
            {
                "thumb1_flex": 0.4,
                "index2_flex": 999.0,
            },
        )

        thumb = by_dof["thumb1_flex"]
        index_pip = by_dof["index2_flex"]
        self.assertAlmostEqual(data.ctrl[thumb.actuator_id], 0.4)
        self.assertAlmostEqual(
            data.ctrl[index_pip.actuator_id],
            index_pip.upper,
        )

    def test_official_position_actuators_follow_glove_targets(self) -> None:
        for hand in ("left", "right"):
            resources = get_hand_resources(hand, generation="v2")
            model, data = load_model(resources.mjcf)
            bindings = build_control_bindings(
                model,
                hand=hand,
                config_path=resources.mapping,
            )
            by_dof = {item.dof_name: item for item in bindings}
            apply_dof_target(
                data,
                bindings,
                {
                    "index1_flex": 0.6,
                    "index2_flex": 0.8,
                    "index3_flex": 0.5,
                },
            )
            for _ in range(500):
                mujoco.mj_step(model, data)

            for name in ("index1_flex", "index2_flex", "index3_flex"):
                binding = by_dof[name]
                joint_id = int(
                    model.actuator_trnid[binding.actuator_id, 0]
                )
                qpos_index = int(model.jnt_qposadr[joint_id])
                self.assertGreater(data.qpos[qpos_index], 0.3)


if __name__ == "__main__":
    unittest.main()
