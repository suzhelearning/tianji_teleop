"""官方 Wuji 命名关节目标到二代 MuJoCo 的绑定合同。"""

from __future__ import annotations

import json
import unittest
from contextlib import contextmanager, nullcontext
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch

import mujoco
import numpy as np

from data_glove_wuji_teleop.adapters.glove.encoder_stream import EncoderFrame
from data_glove_wuji_teleop.adapters.retargeting.glove_pipeline import GloveRetargetPipeline
from data_glove_wuji_teleop.adapters.retargeting.wuji_process import JsonLineProcessTransport
from data_glove_wuji_teleop.adapters.simulation import mujoco_wuji_official as official

from data_glove_wuji_teleop.adapters.simulation.mujoco_wuji_official import (
    apply_named_qpos,
    build_parser,
    build_named_qpos_binding,
    build_official_target_binding,
    make_official_hand_target,
    open_official_target_publisher,
    update_mediapipe_target_scene,
)
from data_glove_wuji_teleop.adapters.simulation.dataglove_skeleton import HandSkeleton
from data_glove_wuji_teleop.adapters.simulation.mujoco_dataglove import (
    load_dataglove_urdf,
)
from data_glove_wuji_teleop.adapters.simulation.mujoco_retarget_composite import (
    load_retarget_composite_model,
)
from data_glove_wuji_teleop.adapters.simulation.mujoco_wuji_v2 import load_model
from data_glove_wuji_teleop.domain.hand_target import DOF_ORDER
from data_glove_wuji_teleop.profiles.dataglove.device import GloveDeviceProfile
from data_glove_wuji_teleop.profiles.dataglove.urdf_zero import URDF_ZERO_GROUPS, UrdfZeroProfile
from data_glove_wuji_teleop.profiles.joint_mapping import load_joint_mapping
from data_glove_wuji_teleop.profiles.wuji_v2.model import full_joint_name
from data_glove_wuji_teleop.project import get_hand_resources


class OfficialWujiMujocoTest(unittest.TestCase):
    def test_shared_config_rejects_wrong_side_before_starting_worker(self) -> None:
        args = build_parser().parse_args([
            "--hand", "right", "--wuji-config",
            str(official.PROJECT_ROOT / "config/retargeting/official/left.yaml"),
        ])
        with patch.object(JsonLineProcessTransport, "launch") as launch:
            with self.assertRaises(ValueError):
                official.launch_official_adapter(args)
            launch.assert_not_called()

    def test_display_only_never_opens_control_outputs_even_on_connection_failure(self) -> None:
        args = build_parser().parse_args(
            ["--skip-network-setup", "--commission-directions"]
        )
        args.print_interval = float("inf")
        glove = GloveRetargetPipeline.load(
            glove_urdf=args.glove_urdf,
            glove_config=args.glove_config,
            zero_file=None,
            morphology_file=args.mano_morphology,
            commissioning=True,
        )
        points = glove.current_mediapipe()
        resources = get_hand_resources("right", generation="v2")
        rules = load_joint_mapping(
            resources.mapping, expected_hand="right", expected_generation="v2"
        )
        result = SimpleNamespace(
            joint_names=tuple(full_joint_name("right", rule.joint_name) for rule in rules),
            qpos=np.zeros(20),
            transformed_keypoints=points,
        )
        for connection_error in (None, TimeoutError("手套掉线")):
            with self.subTest(connection_error=connection_error):
                running = iter((True, False))
                viewer = SimpleNamespace(
                    cam=mujoco.MjvCamera(),
                    user_scn=mujoco.MjvScene(glove.model, maxgeom=256),
                    is_running=lambda: next(running),
                    sync=lambda: None,
                )
                with (
                    patch.object(official.GloveRetargetPipeline, "load", return_value=glove),
                    patch.object(glove, "validate_stream"),
                    patch.object(glove, "read_mediapipe", return_value=points),
                    patch.object(
                        official, "connect_glove",
                        side_effect=connection_error, return_value=nullcontext(object()),
                    ),
                    patch.object(
                        official, "launch_official_adapter",
                        return_value=nullcontext(SimpleNamespace(retarget=lambda *_a, **_k: result)),
                    ),
                    patch.object(official.mujoco.viewer, "launch_passive", return_value=nullcontext(viewer)),
                    patch.object(official, "ZmqDofPublisher", side_effect=AssertionError("仅显示不得发布控制目标")),
                    patch.object(official, "SimulationLease", side_effect=AssertionError("仅显示不得授予真机租约")),
                ):
                    if connection_error is None:
                        official.run(args)
                    else:
                        with self.assertRaises(TimeoutError):
                            official.run(args)

    def test_embedded_calibrations_drive_both_sides_without_resource_files(self) -> None:
        for identity, hand, zero_angle in ((1, "left", 34.0), (2, "right", 12.0)):
            with self.subTest(hand=hand), TemporaryDirectory() as directory:
                root = Path(directory)
                mapping = json.loads(
                    (official.PROJECT_ROOT / f"config/dataglove/urdf/{hand}.json").read_text(
                        encoding="utf-8"
                    )
                )
                zero = UrdfZeroProfile.empty(hand, mapping["expected_cs_by_joint"])
                for group in URDF_ZERO_GROUPS:
                    zero, _ = zero.capture_group(
                        group.name, [[zero_angle] * 21], max_motion_deg=1.0
                    )
                name = f"bench-{hand}"
                profile = GloveDeviceProfile.from_dict(
                    {
                        "version": 4,
                        "name": name,
                        "hand": hand,
                        "device_id": f"0x{identity:016x}",
                        "network": {
                            "mac": "02:00:00:00:00:01",
                            "interface": None,
                            "host": "192.168.7.2",
                            "port": 5580,
                            "http_port": 80,
                            "mtu": 1500,
                            "host_address": f"192.168.7.{identity + 2}/24",
                            "route_table": 17000 + identity,
                            "protocol": "encoder-v1",
                        },
                        "glove_urdf": str(
                            official.PROJECT_ROOT
                            / f"assets/data_glove_urdf/data_glove_urdf_{hand[0].upper()}"
                            / f"urdf/data_glove_urdf_{hand[0].upper()}.urdf"
                        ),
                        "mapping": mapping,
                        "zeros": {"morning": zero.to_dict()},
                        "active_zero": "morning",
                    },
                    path=root / "selected-glove.json",
                )
                profile.save()
                args = build_parser().parse_args([
                    "--glove-profile", str(profile.path),
                    "--skip-network-setup", "--commission-directions",
                ])
                # 文件模式的旧默认值即使不可用，设备模式也只能读取内嵌配置。
                args.glove_config = root / "不存在的机构.json"
                args.zero_file = root / "不存在的零位.json"
                args.print_interval = float("inf")
                moved = [zero_angle] * 21
                moved[5] += 5.0
                frames = iter((
                    EncoderFrame(1, 10_000_000, 0, [zero_angle] * 21),
                    EncoderFrame(2, 20_000_000, 0, moved),
                ))
                connection = SimpleNamespace(
                    channels=21,
                    cs_by_joint=mapping["expected_cs_by_joint"],
                    zeroed=False,
                    range_min=0.0,
                    range_max=360.0,
                    read_latest_frame=lambda: next(frames),
                )
                resources = get_hand_resources(hand, generation="v2")
                rules = load_joint_mapping(
                    resources.mapping, expected_hand=hand, expected_generation="v2"
                )
                observations = []
                glove_qpos = []

                def retarget(points, **_kwargs):
                    observations.append(points.copy())
                    return SimpleNamespace(
                        joint_names=tuple(full_joint_name(hand, rule.joint_name) for rule in rules),
                        qpos=np.zeros(20),
                        transformed_keypoints=points,
                    )

                @contextmanager
                def launch_viewer(model, data, **_kwargs):
                    running = iter((True, True, False))
                    addresses = [
                        model.joint(f"glove_{joint['joint_name']}").qposadr[0]
                        for joint in mapping["joints"]
                    ]
                    yield SimpleNamespace(
                        cam=mujoco.MjvCamera(),
                        user_scn=mujoco.MjvScene(model, maxgeom=256),
                        is_running=lambda: next(running),
                        sync=lambda: glove_qpos.append(data.qpos[addresses].copy()),
                    )

                with (
                    patch.object(official.GloveRetargetPipeline, "load", side_effect=AssertionError("设备不得展开独立配置文件")),
                    patch.object(official, "prepare_glove_network"),
                    patch.object(official, "connect_glove", return_value=nullcontext(connection)),
                    patch.object(official, "launch_official_adapter", return_value=nullcontext(SimpleNamespace(retarget=retarget))),
                    patch.object(official.mujoco.viewer, "launch_passive", side_effect=launch_viewer),
                    patch.object(official, "ZmqDofPublisher", side_effect=AssertionError("仅显示不得发布目标")),
                    patch.object(official, "SimulationLease", side_effect=AssertionError("仅显示不得创建租约")),
                    patch("sys.stdout"),
                ):
                    official.run(args)
                np.testing.assert_allclose(glove_qpos[0], 0.0, atol=1e-10)
                expected = np.zeros(len(mapping["joints"]))
                for index, joint in enumerate(mapping["joints"]):
                    if joint["source_channel"] == "J6":
                        expected[index] = np.deg2rad(5.0) * joint["direction"] * joint.get("scale", 1.0)
                np.testing.assert_allclose(glove_qpos[1], expected, atol=1e-10)
                self.assertEqual(np.asarray(observations).shape, (2, 21, 3))
                self.assertTrue(np.isfinite(observations).all())
                self.assertGreater(np.linalg.norm(observations[1] - observations[0]), 1e-5)
                self.assertEqual(set(root.iterdir()), {profile.path})

    def test_connection_check_exits_without_loading_simulation_or_control_outputs(self) -> None:
        args = build_parser().parse_args(
            ["--skip-network-setup", "--check-connection", "--publish-targets"]
        )
        connection = SimpleNamespace(
            channels=21, read_frame=lambda: SimpleNamespace(sequence=42)
        )
        with (
            patch.object(official, "connect_glove", return_value=nullcontext(connection)),
            patch.object(official.GloveRetargetPipeline, "load", side_effect=AssertionError("连接检查不加载仿真")),
            patch.object(official, "ZmqDofPublisher", side_effect=AssertionError("连接检查不得发布目标")),
            patch.object(official, "SimulationLease", side_effect=AssertionError("连接检查不得创建租约")),
            patch.object(official, "launch_official_adapter", side_effect=AssertionError("连接检查不启动IK")),
        ):
            official.run(args)

    def test_official_target_publisher_sends_matching_side_zero_before_close(self) -> None:
        events: list[object] = []

        class FakePublisher:
            def __init__(self, host, port):
                pass

            def publish(self, target):
                events.append(target)

            def close(self):
                events.append("close")

        for hand, port in (("left", 15559), ("right", 15558)):
            with self.subTest(hand=hand):
                events.clear()
                with (
                    patch.object(official, "ZmqDofPublisher", FakePublisher),
                    patch.object(official.time, "sleep"),
                ):
                    with open_official_target_publisher("127.0.0.1", port, hand=hand):
                        pass
                self.assertEqual(events[-1], "close")
                self.assertTrue(events[:-1], "退出必须发送同侧安全零目标")
                for target in events[:-1]:
                    self.assertEqual(target.hand, hand)
                    self.assertEqual(target.values, (0.0,) * len(DOF_ORDER))
                    self.assertEqual(target.source, "official_wuji_retarget_shutdown")

    def test_converts_both_sides_named_qpos_to_canonical_hand_target(self) -> None:
        for hand in ("left", "right"):
            with self.subTest(hand=hand):
                resources = get_hand_resources(hand, generation="v2")
                rules = load_joint_mapping(
                    resources.mapping, expected_hand=hand, expected_generation="v2",
                )
                semantic_values = tuple(0.01 * (index + 1) for index in range(len(DOF_ORDER)))
                semantic_by_name = dict(zip(DOF_ORDER, semantic_values))
                official_by_name = {
                    full_joint_name(hand, rule.joint_name): (
                        semantic_by_name[rule.dof_name] * rule.scale + rule.offset_rad
                    )
                    for rule in rules
                }
                shuffled_names = tuple(reversed(tuple(official_by_name)))
                shuffled_qpos = np.array([official_by_name[name] for name in shuffled_names])
                binding = build_official_target_binding(
                    shuffled_names, mapping_path=resources.mapping, hand=hand,
                )
                target = make_official_hand_target(
                    binding, shuffled_qpos, sequence=7, timestamp_ns=123,
                )
                self.assertEqual(target.hand, hand)
                np.testing.assert_allclose(target.values, semantic_values, rtol=0, atol=1e-15)
                self.assertEqual(target.source, "official_wuji_retarget")

    def test_conflicting_profile_resources_fail_before_network_or_worker(self) -> None:
        args = build_parser().parse_args([
            "--zero-file", "another-device.json", "--glove-profile", "selected-device",
        ])
        with (
            patch.object(official, "prepare_glove_network", side_effect=AssertionError("不得配网")),
            patch.object(official, "connect_glove", side_effect=AssertionError("不得连接")),
            patch.object(official, "launch_official_adapter", side_effect=AssertionError("不得启动worker")),
        ):
            with self.assertRaisesRegex(ValueError, "--glove-profile.*--zero-file"):
                official.run(args)

    def test_left_scene_aligns_glove_and_robot_palm_directions(self) -> None:
        resources = get_hand_resources("left", generation="v2")
        model, data = load_retarget_composite_model(
            glove_urdf=(
                Path(__file__).resolve().parents[2]
                / "assets/data_glove_urdf/data_glove_urdf_L/urdf/data_glove_urdf_L.urdf"
            ),
            wuji_mjcf=resources.mjcf,
            column_spacing=0.22,
            hand="left",
        )
        mujoco.mj_forward(model, data)
        glove_forward = (
            data.xanchor[model.joint("glove_middle_mcp_flex_joint").id]
            - data.body("glove_wrist_link").xpos
        )
        robot_forward = (
            data.xanchor[model.joint("wuji_l_middle_finger_mcp_flex").id]
            - data.body("wuji_l_wrist").xpos
        )
        np.testing.assert_allclose(
            glove_forward / np.linalg.norm(glove_forward),
            robot_forward / np.linalg.norm(robot_forward),
            atol=1e-10, rtol=0,
        )

    def test_subset_binding_rejects_missing_prefixed_joint(self) -> None:
        resources = get_hand_resources("right", generation="v2")
        model, _data = load_retarget_composite_model(
            glove_urdf=(
                Path(__file__).resolve().parents[2]
                / "assets/data_glove_urdf/data_glove_urdf_R/urdf/data_glove_urdf_R.urdf"
            ),
            wuji_mjcf=resources.mjcf,
            column_spacing=0.22,
        )

        with self.assertRaisesRegex(ValueError, "缺少关节.*bad_r_thumb"):
            build_named_qpos_binding(
                model,
                ("r_thumb_cmc_flex",),
                model_prefix="bad_",
                require_complete_model=False,
            )


    def test_composite_model_contains_separated_glove_and_wuji_columns(
        self,
    ) -> None:
        resources = get_hand_resources("right", generation="v2")
        model, data = load_retarget_composite_model(
            glove_urdf=(
                Path(__file__).resolve().parents[2]
                / "assets/data_glove_urdf/data_glove_urdf_R/urdf/data_glove_urdf_R.urdf"
            ),
            wuji_mjcf=resources.mjcf,
            column_spacing=0.22,
        )
        mujoco.mj_forward(model, data)

        self.assertEqual((model.nq, model.nv, model.nu), (41, 41, 20))
        self.assertGreaterEqual(
            mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_JOINT,
                "glove_index_pip_joint",
            ),
            0,
        )
        wrist_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "wuji_r_wrist",
        )
        self.assertGreaterEqual(wrist_id, 0)
        self.assertAlmostEqual(float(data.xpos[wrist_id, 0]), 0.22)
        glove_root_joint = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            "glove_index_mcp_abd_joint",
        )
        self.assertLess(float(data.xanchor[glove_root_joint, 0]), 0.0)
        glove_alphas = [
            float(model.geom_rgba[geom_id, 3])
            for geom_id in range(model.ngeom)
            if (
                mujoco.mj_id2name(
                    model,
                    mujoco.mjtObj.mjOBJ_BODY,
                    int(model.geom_bodyid[geom_id]),
                )
                or ""
            ).startswith("glove_")
        ]
        self.assertTrue(glove_alphas)
        self.assertLessEqual(max(glove_alphas), 0.200001)

    def test_composite_glove_palm_basis_aligns_with_wuji_column(self) -> None:
        resources = get_hand_resources("right", generation="v2")
        model, data = load_retarget_composite_model(
            glove_urdf=(
                Path(__file__).resolve().parents[2]
                / "assets/data_glove_urdf/data_glove_urdf_R/urdf/data_glove_urdf_R.urdf"
            ),
            wuji_mjcf=resources.mjcf,
            column_spacing=0.22,
        )
        mujoco.mj_forward(model, data)

        glove_wrist_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "glove_wrist_link",
        )
        glove_middle_dip_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            "glove_middle_dip_joint",
        )
        glove_middle_body_id = int(
            model.jnt_bodyid[glove_middle_dip_id]
        )
        glove_middle_tip = (
            data.xanchor[glove_middle_dip_id]
            + data.xmat[glove_middle_body_id].reshape(3, 3)
            @ np.array((0.0, 0.0, 0.025))
        )
        glove_longitudinal = (
            glove_middle_tip - data.xpos[glove_wrist_id]
        )
        glove_index_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            "glove_index_mcp_abd_joint",
        )
        glove_pinky_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            "glove_pinky_cmc_joint",
        )
        glove_lateral = (
            data.xanchor[glove_index_id] - data.xanchor[glove_pinky_id]
        )

        wuji_wrist_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "wuji_r_wrist",
        )
        wuji_middle_tip_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_SITE,
            "wuji_r_middle_finger_tip",
        )
        wuji_longitudinal = (
            data.site_xpos[wuji_middle_tip_id] - data.xpos[wuji_wrist_id]
        )
        wuji_index_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "wuji_r_index_finger_proximal",
        )
        wuji_pinky_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "wuji_r_pinky_proximal",
        )
        wuji_lateral = data.xpos[wuji_index_id] - data.xpos[wuji_pinky_id]

        def normalized(vector: np.ndarray) -> np.ndarray:
            return vector / np.linalg.norm(vector)

        glove_longitudinal = normalized(glove_longitudinal)
        wuji_longitudinal = normalized(wuji_longitudinal)
        glove_lateral = normalized(
            glove_lateral
            - glove_longitudinal
            * np.dot(glove_lateral, glove_longitudinal)
        )
        wuji_lateral = normalized(
            wuji_lateral
            - wuji_longitudinal
            * np.dot(wuji_lateral, wuji_longitudinal)
        )

        self.assertGreater(
            float(np.dot(glove_longitudinal, wuji_longitudinal)),
            0.99,
        )
        self.assertGreater(
            float(np.dot(glove_lateral, wuji_lateral)),
            0.99,
        )

    def test_draws_connected_mediapipe_target_in_optimizer_frame(self) -> None:
        resources = get_hand_resources("right", generation="v2")
        model, data = load_model(resources.mjcf)
        mujoco.mj_forward(model, data)
        points = np.zeros((21, 3), dtype=np.float64)
        for finger in range(5):
            start = 1 + 4 * finger
            points[start : start + 4, 0] = 0.01 * (finger + 1)
            points[start : start + 4, 2] = np.arange(1, 5) * 0.02
        scene = mujoco.MjvScene(model, maxgeom=64)
        display_offset = np.array((0.0, 0.15, 0.0))

        update_mediapipe_target_scene(
            model,
            data,
            scene,
            points,
            world_offset=display_offset,
        )

        geom_types = tuple(
            int(scene.geoms[index].type) for index in range(scene.ngeom)
        )
        self.assertEqual(scene.ngeom, 41)
        self.assertEqual(
            geom_types.count(int(mujoco.mjtGeom.mjGEOM_SPHERE)),
            21,
        )
        self.assertEqual(
            geom_types.count(int(mujoco.mjtGeom.mjGEOM_CAPSULE)),
            20,
        )
        wrist_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "r_wrist",
        )
        expected_tip = (
            data.xpos[wrist_id]
            + points[4]
            + display_offset
        )
        sphere_positions = np.array(
            [
                scene.geoms[index].pos.copy()
                for index, geom_type in enumerate(geom_types)
                if geom_type == int(mujoco.mjtGeom.mjGEOM_SPHERE)
            ]
        )
        self.assertLess(
            float(np.linalg.norm(sphere_positions - expected_tip, axis=1).min()),
            1e-8,
        )
        np.testing.assert_allclose(
            scene.geoms[0].pos,
            data.xpos[wrist_id] + display_offset,
            atol=1e-8,
        )

    def test_mediapipe_fingers_point_with_upright_wuji_hand(self) -> None:
        project_root = Path(__file__).resolve().parents[2]
        resources = get_hand_resources("right", generation="v2")
        model, data = load_retarget_composite_model(
            glove_urdf=(
                project_root
                / "assets/data_glove_urdf/data_glove_urdf_R/urdf/data_glove_urdf_R.urdf"
            ),
            wuji_mjcf=resources.mjcf,
            column_spacing=0.22,
        )
        mujoco.mj_forward(model, data)
        points = np.zeros((21, 3), dtype=np.float64)
        for finger in range(5):
            start = 1 + 4 * finger
            points[start : start + 4, 2] = np.arange(1, 5) * 0.04
        scene = mujoco.MjvScene(model, maxgeom=64)

        update_mediapipe_target_scene(
            model,
            data,
            scene,
            points,
            world_offset=np.array((-0.22, 0.0, 0.0)),
            wrist_body_name="wuji_r_wrist",
        )

        target_middle_direction = (
            scene.geoms[12].pos - scene.geoms[0].pos
        )
        middle_tip_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_SITE,
            "wuji_r_middle_finger_tip",
        )
        wrist_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "wuji_r_wrist",
        )
        wuji_middle_direction = (
            data.site_xpos[middle_tip_id] - data.xpos[wrist_id]
        )
        cosine = float(
            np.dot(target_middle_direction, wuji_middle_direction)
            / np.linalg.norm(target_middle_direction)
            / np.linalg.norm(wuji_middle_direction)
        )

        self.assertGreater(cosine, 0.95)

    def test_applies_shuffled_official_values_by_joint_name(self) -> None:
        resources = get_hand_resources("right", generation="v2")
        model, data = load_model(resources.mjcf)
        model_joint_names = tuple(
            mujoco.mj_id2name(
                model,
                mujoco.mjtObj.mjOBJ_JOINT,
                joint_id,
            )
            for joint_id in range(model.njnt)
        )
        source_names = tuple(reversed(model_joint_names))
        source_values = np.arange(20, dtype=np.float64) / 100.0

        binding = build_named_qpos_binding(model, source_names)
        apply_named_qpos(data, binding, source_values)

        for source_index, joint_name in enumerate(source_names):
            joint_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_JOINT,
                joint_name,
            )
            qpos_address = int(model.jnt_qposadr[joint_id])
            self.assertAlmostEqual(
                data.qpos[qpos_address],
                source_values[source_index],
            )


if __name__ == "__main__":
    unittest.main()
