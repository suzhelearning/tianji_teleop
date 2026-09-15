"""独立参数化手型的末端优先拟合与机械手套集成合同。"""

from __future__ import annotations

import copy
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from data_glove_wuji_teleop.adapters.glove.encoder_kinematics import GloveJointFrame
from data_glove_wuji_teleop.adapters.retargeting.glove_pipeline import (
    GloveRetargetPipeline,
)
from data_glove_wuji_teleop.adapters.retargeting.mano_morphology import (
    ManoMorphologyFitter,
)


PROJECT_ROOT = Path(__file__).resolve().parents[2]
MORPHOLOGY_PATH = (
    PROJECT_ROOT / "config/retargeting/wuji_v2_right_mano_morphology.json"
)
FINGERS = ("thumb", "index", "middle", "ring", "pinky")
ROOTS = np.array([1, 5, 9, 13, 17])
TIPS = ROOTS + 3


class ManoMorphologyFitterTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = json.loads(MORPHOLOGY_PATH.read_text(encoding="utf-8"))
        # 独立手型的基础契约关闭吸附；对指测试与真实 pipeline 显式覆盖启用。
        for key in ("pinch_distance_m", "pinch_full_contact_distance_m", "pinch_target_distance_m"):
            self.config[key] = 0.0
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.config_path = Path(self.directory.name) / "morphology.json"

    def load_fitter(
        self,
        config: dict | None = None,
        *,
        reference: np.ndarray | None = None,
    ) -> ManoMorphologyFitter:
        self.config_path.write_text(
            json.dumps(self.config if config is None else config), encoding="utf-8"
        )
        return ManoMorphologyFitter.load(
            self.config_path, reference_keypoints=reference
        )

    def pose(self, fraction: float) -> np.ndarray:
        limits = np.array(
            [self.config["fingers"][name]["joint_limits_rad"] for name in FINGERS]
        )
        return limits[:, :, 0] + fraction * (limits[:, :, 1] - limits[:, :, 0])

    def assert_shape(
        self,
        points: np.ndarray,
        *,
        rotation: np.ndarray | None = None,
        config: dict | None = None,
    ) -> None:
        shape = self.config if config is None else config
        rotation = np.eye(3) if rotation is None else rotation
        self.assertEqual(points.shape, (21, 3))
        self.assertTrue(np.isfinite(points).all())
        for name, start in zip(FINGERS, ROOTS):
            finger = shape["fingers"][name]
            np.testing.assert_allclose(
                points[start] - points[0],
                np.asarray(finger["root_m"]) @ rotation.T,
                atol=1e-10,
                rtol=0,
            )
            np.testing.assert_allclose(
                np.linalg.norm(np.diff(points[start : start + 4], axis=0), axis=1),
                finger["segment_lengths_m"],
                atol=1e-10,
                rtol=0,
            )

    def test_reachable_step_targets_converge_with_fixed_shape(self) -> None:
        source = self.load_fitter()
        fitter = self.load_fitter()
        for fraction, wrist in (
            (0.3, np.array([0.03, -0.02, 0.01])),
            (0.65, np.array([-0.04, 0.05, 0.02])),
        ):
            observed = source.forward(self.pose(fraction), wrist=wrist)
            # 阶跃目标允许帧间正则逐步追上，不要求用瞬间跳变换取首帧零误差。
            for _ in range(12):
                fitted = fitter.fit(observed)
                self.assert_shape(fitted)
            np.testing.assert_allclose(fitted[0], wrist, atol=1e-12, rtol=0)
            self.assertLess(
                np.linalg.norm(fitted[TIPS] - observed[TIPS], axis=1).max(),
                0.001,
            )

    def test_observed_intermediate_points_are_soft_directions_not_positions(
        self,
    ) -> None:
        source = self.load_fitter()
        observed = source.forward(self.pose(0.4))
        baseline = self.load_fitter().fit(observed)
        altered = observed.copy()
        altered[ROOTS] += [0.025, -0.012, 0.017]
        altered[ROOTS + 1] += [0.015, -0.008, 0.010]
        altered[ROOTS + 2] += [-0.012, 0.005, 0.009]
        fitted = self.load_fitter().fit(altered)

        self.assert_shape(fitted)
        self.assertLess(
            np.linalg.norm(fitted[TIPS] - observed[TIPS], axis=1).max(), 0.004
        )
        self.assertGreater(
            np.linalg.norm(fitted[ROOTS] - altered[ROOTS], axis=1).min(), 0.005
        )
        # 方向仍能影响冗余姿态，不能退化为仅追末端、完全忽略机械骨段。
        internal = np.concatenate((ROOTS + 1, ROOTS + 2))
        self.assertGreater(np.linalg.norm(fitted[internal] - baseline[internal]), 1e-5)

    def test_runtime_observations_do_not_rotate_the_fixed_palm(self) -> None:
        source = self.load_fitter()
        fitter = self.load_fitter()
        observed = source.forward(self.pose(0.35))
        first = fitter.fit(observed)
        rotation = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        wrist = np.array([0.1, -0.05, 0.03])
        second = fitter.fit(observed @ rotation.T + wrist)

        self.assert_shape(second)
        np.testing.assert_allclose(second[0], wrist, atol=1e-12, rtol=0)
        np.testing.assert_allclose(
            second[ROOTS] - second[0], first[ROOTS] - first[0], atol=1e-10, rtol=0
        )

    def test_unreachable_tips_keep_roots_lengths_and_joint_limits(self) -> None:
        config = copy.deepcopy(self.config)
        limits = np.array([[-0.25, 0.25], [0.1, 0.8], [0.0, 0.9], [0.0, 0.65]])
        for finger in config["fingers"].values():
            finger["joint_limits_rad"] = limits.tolist()
        config.update(
            pinch_distance_m=2.0,
            pinch_full_contact_distance_m=1.0,
            pinch_target_distance_m=0.0,
        )
        fitter = self.load_fitter(config)
        observed = fitter.forward(np.tile(limits.mean(axis=1), (5, 1)))
        observed[TIPS] += [0.4, -0.3, 0.5]
        fitted = fitter.fit(observed)

        self.assert_shape(fitted, config=config)
        for name, start in zip(FINGERS, ROOTS):
            # 从公开输出的骨段恢复关节角，检查合法姿态而非求解器内部状态。
            directions = np.diff(fitted[start : start + 4], axis=0)
            directions /= np.linalg.norm(directions, axis=1, keepdims=True)
            local = directions @ np.asarray(config["fingers"][name]["rest_basis"])
            abduction = np.arctan2(-local[0, 0], local[0, 1])
            c, s = np.cos(abduction), np.sin(abduction)
            unspread = local @ np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
            np.testing.assert_allclose(unspread[:, 0], 0.0, atol=1e-10, rtol=0)
            cumulative = np.arctan2(unspread[:, 2], unspread[:, 1])
            bends = np.diff(cumulative)
            bends = np.arctan2(np.sin(bends), np.cos(bends))
            angles = np.array([abduction, cumulative[0], *bends])
            self.assertTrue(np.all(angles >= limits[:, 0] - 1e-8))
            self.assertTrue(np.all(angles <= limits[:, 1] + 1e-8))

    def test_invalid_frames_do_not_change_the_next_valid_result(self) -> None:
        source = self.load_fitter()
        config = copy.deepcopy(self.config)
        config.update(
            pinch_distance_m=0.03,
            pinch_full_contact_distance_m=0.02,
            pinch_target_distance_m=0.0,
        )
        fitter = self.load_fitter(config)
        control = self.load_fitter(config)
        first = source.forward(self.pose(0.25))
        next_frame = source.forward(self.pose(0.6), wrist=np.array([0.02, 0.01, 0.0]))
        fitter.fit(first)
        control.fit(first)
        nonfinite = next_frame.copy()
        nonfinite[-1, -1] = np.nan
        degenerate = next_frame.copy()
        degenerate[-1] = degenerate[-2]
        for invalid in (nonfinite, degenerate, next_frame[:-1]):
            with self.assertRaises(ValueError):
                fitter.fit(invalid)
        np.testing.assert_allclose(
            fitter.fit(next_frame), control.fit(next_frame), atol=1e-10, rtol=0
        )

    def test_reference_only_aligns_rotation_and_fit_is_rigid_equivariant(
        self,
    ) -> None:
        canonical = self.load_fitter()
        reference = canonical.forward(self.pose(0.3))
        reference[0] = 0.0
        reference[9] = [0.0, 0.15, 0.0]
        reference[5] = [0.07, 0.12, 0.0]
        reference[17] = [-0.07, 0.12, 0.0]
        yaw, tilt = 0.63, -0.4
        cy, sy, ct, st = np.cos(yaw), np.sin(yaw), np.cos(tilt), np.sin(tilt)
        rotation = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]]) @ np.array(
            [[1.0, 0.0, 0.0], [0.0, ct, -st], [0.0, st, ct]]
        )
        translation = np.array([0.12, -0.07, 0.05])
        # 任意放大参考机构不能改变独立手型的根点、骨长或静态关节框架。
        aligned = self.load_fitter(reference=(2.3 * reference) @ rotation.T + translation)
        pose = self.pose(0.45)
        canonical_points = canonical.forward(pose)
        aligned_points = aligned.forward(pose, wrist=translation)
        np.testing.assert_allclose(
            aligned_points, canonical_points @ rotation.T + translation, atol=1e-10, rtol=0
        )
        self.assert_shape(aligned_points, rotation=rotation)

        observed = canonical_points.copy()
        observed[ROOTS + 1] += [0.006, -0.003, 0.004]
        fitted = canonical.fit(observed)
        transformed_fit = aligned.fit(observed @ rotation.T + translation)
        np.testing.assert_allclose(
            transformed_fit, fitted @ rotation.T + translation, atol=1e-5, rtol=0
        )

    def test_rejects_unsafe_shape_units_geometry_and_limits(self) -> None:
        wrong_units = copy.deepcopy(self.config)
        wrong_units["length_unit"] = "millimeter"
        negative_bone = copy.deepcopy(self.config)
        negative_bone["fingers"]["index"]["segment_lengths_m"][0] = -0.04
        reflected_frame = copy.deepcopy(self.config)
        reflected_frame["fingers"]["index"]["rest_basis"][0][0] = -1.0
        reversed_limits = copy.deepcopy(self.config)
        reversed_limits["fingers"]["thumb"]["joint_limits_rad"][0] = [0.5, -0.5]
        for config in (wrong_units, negative_bone, reflected_frame, reversed_limits):
            with self.subTest(config=config), self.assertRaises(ValueError):
                self.load_fitter(config)

    def test_restored_contact_pulls_glove_tip_to_fitted_thumb(self) -> None:
        pipeline = GloveRetargetPipeline.load(
            glove_urdf=PROJECT_ROOT / "assets/data_glove_urdf/data_glove_urdf_R/urdf/data_glove_urdf_R.urdf",
            glove_config=PROJECT_ROOT / "config/dataglove/urdf/right.json",
            zero_file=None,
            morphology_file=None,
            commissioning=True,
        )
        reference = pipeline.current_mediapipe()
        control = self.load_fitter(reference=reference)
        config = copy.deepcopy(self.config)
        config.update(
            pinch_distance_m=0.03,
            pinch_full_contact_distance_m=0.02,
            pinch_target_distance_m=0.0,
        )
        fitter = self.load_fitter(config, reference=reference)
        raw = np.zeros(21)
        raw[0] = -15.0
        raw[10] = -30.0
        raw[6] = -30.0
        raw[11] = raw[16] = -60.0
        observed = pipeline.process_joint_frame(
            GloveJointFrame(1, 2, 0, pipeline.kinematics.convert_angles(raw))
        )
        self.assertLess(np.linalg.norm(observed[8] - observed[4]), 0.02)
        for _ in range(40):
            baseline = control.fit(observed)
            fitted = fitter.fit(observed)
        self.assertLess(np.linalg.norm(fitted[8] - fitted[4]), 0.002)
        np.testing.assert_allclose(fitted[:5], baseline[:5], atol=1e-10, rtol=0)
        np.testing.assert_allclose(fitted[ROOTS], baseline[ROOTS], atol=1e-10, rtol=0)
        lengths = np.linalg.norm(np.diff(fitted[1:].reshape(5, 4, 3), axis=1), axis=2)
        np.testing.assert_allclose(
            lengths,
            [self.config["fingers"][name]["segment_lengths_m"] for name in FINGERS],
            atol=1e-10, rtol=0,
        )
        # 过渡中点压缩一半；保持同一观测不能把上一帧的半间距再次折半。
        source_gap = float(np.linalg.norm(observed[8] - observed[4]))
        config.update(pinch_distance_m=2.0 * source_gap, pinch_full_contact_distance_m=0.0)
        partial = self.load_fitter(config, reference=reference)
        for _ in range(80):
            halfway = partial.fit(observed)
        halfway_gap = float(np.linalg.norm(halfway[8] - halfway[4]))
        self.assertAlmostEqual(
            halfway_gap, 0.5 * np.linalg.norm(baseline[8] - baseline[4]), delta=0.001
        )
        for _ in range(80):
            held = partial.fit(observed)
        self.assertAlmostEqual(
            np.linalg.norm(held[8] - held[4]), halfway_gap, delta=0.0001
        )
        # 松开后退出吸附，不遗留锁定目标。
        for _ in range(40):
            released = fitter.fit(reference)
            baseline = control.fit(reference)
        np.testing.assert_allclose(released, baseline, atol=1e-8, rtol=0)

    def test_physical_glove_flexion_bends_the_fitted_fingers_the_same_way(self) -> None:
        pipeline = GloveRetargetPipeline.load(
            glove_urdf=PROJECT_ROOT / "assets/data_glove_urdf/data_glove_urdf_R/urdf/data_glove_urdf_R.urdf",
            glove_config=PROJECT_ROOT / "config/dataglove/urdf/right.json",
            zero_file=None,
            morphology_file=MORPHOLOGY_PATH,
            commissioning=True,
        )
        frames = []
        for delta in (0.0, -60.0):
            # 实物已确认 J12..J20 屈曲时编码器减小；必须经过真实四连杆换算，
            # 不能用目标模型自身的 FK 或手写的 URDF 角度生成“正确”输入。
            raw = np.zeros(21)
            raw[11:20] = delta
            converted = pipeline.kinematics.convert_angles(raw)
            frame = GloveJointFrame(1, 2, 0, tuple(converted))
            for _ in range(21):
                fitted = pipeline.process_joint_frame(frame)
            observed = pipeline.landmarks.to_mediapipe(pipeline.data)
            chains = np.stack((observed, fitted))[:, 1:].reshape(2, 5, 4, 3)
            segments = np.diff(chains, axis=2)
            frames.append(
                segments / np.linalg.norm(segments, axis=3, keepdims=True)
            )

        # 从彩色骨架的实际运动建立屈曲轴，不预设世界坐标的正负。
        axes = np.cross(frames[0][0, :, 2], frames[1][0, :, 2])
        axes /= np.linalg.norm(axes, axis=1, keepdims=True)
        bends = []
        for segments in frames:
            sine = np.einsum(
                "fi,kfi->kf", axes, np.cross(segments[:, :, 0], segments[:, :, 2])
            )
            cosine = np.einsum("kfi,kfi->kf", segments[:, :, 0], segments[:, :, 2])
            bends.append(np.arctan2(sine, cosine))
        self.assertTrue(
            np.all(bends[1][1, 1:] > np.deg2rad(5.0)),
            f"四指应形成同向弯曲，不能仅靠基关节过伸追指尖：{np.degrees(bends[1][1, 1:])}",
        )
        # 比较屈曲增量而非两副骨架的绝对夹角：拇指外骨骼零位自身带折角，
        # 独立手型不应被迫复制该机械折角。
        delta = bends[1] - bends[0]
        fitted_changes = np.arctan2(np.sin(delta[1]), np.cos(delta[1]))
        self.assertTrue(
            np.all(fitted_changes > np.deg2rad(5.0)),
            f"五指应随彩色骨架同向屈曲，实际角度增量：{np.degrees(fitted_changes)}",
        )

    def test_continuous_glove_curl_does_not_flip_abduction_branches(self) -> None:
        pipeline = GloveRetargetPipeline.load(
            glove_urdf=PROJECT_ROOT / "assets/data_glove_urdf/data_glove_urdf_R/urdf/data_glove_urdf_R.urdf",
            glove_config=PROJECT_ROOT / "config/dataglove/urdf/right.json",
            zero_file=None,
            morphology_file=MORPHOLOGY_PATH,
            commissioning=True,
        )
        previous = None
        for phase in np.linspace(0.0, 2.0 * np.pi, 121):
            amount = 0.5 * (1.0 - np.cos(phase))
            # 编码器输入经过真实机构换算；URDF 的负向屈曲不是已换算帧的负值。
            raw = np.zeros(21)
            raw[6:10] = -30.0 * amount
            raw[11:20] = -80.0 * amount
            angles = pipeline.kinematics.convert_angles(raw)
            fitted = pipeline.process_joint_frame(
                GloveJointFrame(1, 2, 0, tuple(angles))
            )
            if previous is not None:
                self.assertLess(
                    np.linalg.norm(fitted - previous, axis=1).max(), 0.01
                )
            previous = fitted

    def test_left_observations_preserve_mirrored_flexion_and_release(self) -> None:
        arguments = dict(
            glove_urdf=PROJECT_ROOT / "assets/data_glove_urdf/data_glove_urdf_R/urdf/data_glove_urdf_R.urdf",
            glove_config=PROJECT_ROOT / "config/dataglove/urdf/right.json",
            zero_file=None,
            morphology_file=MORPHOLOGY_PATH,
            commissioning=True,
        )
        right = GloveRetargetPipeline.load(**arguments)
        reflection = np.array((-1.0, 1.0, 1.0))
        # 仅测试使用的左手观测夹具，不是可用于实物的左手标定或模型。
        left_config = json.loads(arguments["glove_config"].read_text(encoding="utf-8"))
        left_config["hand"] = "left"
        left_path = Path(self.directory.name) / "synthetic-left.json"
        left_path.write_text(json.dumps(left_config), encoding="utf-8")
        reflected_landmarks = SimpleNamespace(
            to_mediapipe=lambda _data: right.landmarks.to_mediapipe(right.data) * reflection,
        )
        with patch(
            "data_glove_wuji_teleop.adapters.retargeting.glove_pipeline.GloveLandmarkAdapter.from_model",
            return_value=reflected_landmarks,
        ):
            left = GloveRetargetPipeline.load(**(arguments | {"glove_config": left_path}))
        for sequence, angle in enumerate((0.0, 5.0, 15.0, 25.0, 15.0, 5.0, 0.0)):
            right_points = right.process_joint_frame(
                GloveJointFrame(sequence, sequence, 0, (angle,) * 21)
            )
            left_points = left.current_mediapipe()
            np.testing.assert_allclose(
                left_points, right_points * reflection, atol=1e-10, rtol=0,
            )

    def test_glove_pipeline_exports_independent_roots_and_bone_lengths(self) -> None:
        arguments = dict(
            glove_urdf=(PROJECT_ROOT / "assets/data_glove_urdf/data_glove_urdf_R/urdf/data_glove_urdf_R.urdf"),
            glove_config=PROJECT_ROOT / "config/dataglove/urdf/right.json",
            zero_file=None,
            commissioning=True,
        )
        raw_pipeline = GloveRetargetPipeline.load(**arguments, morphology_file=None)
        pipeline = GloveRetargetPipeline.load(**arguments, morphology_file=MORPHOLOGY_PATH)
        relative_roots = None
        for sequence, angle in enumerate((0.0, 12.0), start=1):
            frame = GloveJointFrame(
                sequence=sequence,
                timestamp_ns=sequence,
                dropped=0,
                joint_angles_deg=(angle,) * 21,
            )
            raw = raw_pipeline.process_joint_frame(frame)
            fitted = pipeline.process_joint_frame(frame)
            self.assertTrue(np.isfinite(fitted).all())
            np.testing.assert_allclose(fitted[0], raw[0], atol=1e-12, rtol=0)
            roots = fitted[ROOTS] - fitted[0]
            if relative_roots is not None:
                np.testing.assert_allclose(roots, relative_roots, atol=1e-10, rtol=0)
            relative_roots = roots.copy()
            expected_roots = np.array(
                [self.config["fingers"][name]["root_m"] for name in FINGERS]
            )
            # 不依赖机构参考旋转：根点间内积由独立手掌形状决定。
            np.testing.assert_allclose(
                roots @ roots.T, expected_roots @ expected_roots.T, atol=1e-10, rtol=0
            )
            self.assertGreater(np.linalg.norm(fitted[ROOTS] - raw[ROOTS]), 0.001)
            self.assertGreater(np.linalg.norm(fitted[1] - raw[1]), 0.001)
            for name, start in zip(FINGERS, ROOTS):
                np.testing.assert_allclose(
                    np.linalg.norm(np.diff(fitted[start : start + 4], axis=0), axis=1),
                    self.config["fingers"][name]["segment_lengths_m"],
                    atol=1e-10,
                    rtol=0,
                )


if __name__ == "__main__":
    unittest.main()
