"""实物数据手套帧到连续 MediaPipe/MANO 目标骨架的完整 pipeline。"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from ...profiles.dataglove.urdf_mapping import DataGloveUrdfMapping
from ...profiles.dataglove.urdf_zero import UrdfZeroProfile
from ..glove.encoder_kinematics import EncoderKinematics, GloveJointFrame
from ..glove.encoder_stream import EncoderConnection, EncoderFrame
from ..simulation.mujoco_dataglove import (
    apply_urdf_joint_frame,
    convert_encoder_frame_for_urdf,
    load_dataglove_urdf,
)
from .glove_landmarks import GloveLandmarkAdapter
from .mano_morphology import ManoMorphologyFitter


@dataclass
class GloveRetargetPipeline:
    """封装手套观测 FK 和可选的独立手型姿态拟合。"""

    mapping: DataGloveUrdfMapping
    kinematics: EncoderKinematics
    model: mujoco.MjModel
    data: mujoco.MjData
    qpos_by_joint: dict[str, int]
    landmarks: GloveLandmarkAdapter
    morphology: ManoMorphologyFitter | None
    zero_profile: UrdfZeroProfile | None

    @classmethod
    def load(
        cls,
        *,
        glove_urdf: str | Path,
        glove_config: str | Path,
        zero_file: str | Path | None,
        morphology_file: str | Path | None,
        commissioning: bool,
    ) -> "GloveRetargetPipeline":
        with Path(glove_config).open("r", encoding="utf-8") as file:
            mapping_data = json.load(file)
        zero_profile = (
            UrdfZeroProfile.load(zero_file) if zero_file is not None else None
        )
        return cls.from_config(
            glove_urdf=glove_urdf,
            mapping_data=mapping_data,
            zero_profile=zero_profile,
            morphology_file=morphology_file,
            commissioning=commissioning,
        )

    @classmethod
    def from_config(
        cls,
        *,
        glove_urdf: str | Path,
        mapping_data: dict,
        zero_profile: UrdfZeroProfile | None,
        morphology_file: str | Path | None,
        commissioning: bool,
    ) -> "GloveRetargetPipeline":
        """从单文件设备中的机构和零位配置构建真实观测求解管线。"""
        urdf_path = Path(glove_urdf)
        mapping = DataGloveUrdfMapping.from_dict(mapping_data)
        mapping.validate_urdf(urdf_path)
        mapping.require_all_joints_enabled()
        if not commissioning:
            mapping.require_verified_directions()
        kinematics = EncoderKinematics.from_dict(
            mapping_data,
            commissioning=commissioning,
        )
        model, data, qpos_by_joint = load_dataglove_urdf(urdf_path)
        mujoco.mj_forward(model, data)
        landmarks = GloveLandmarkAdapter.from_model(model)
        reference = None
        if morphology_file is not None:
            reference = landmarks.to_mediapipe(data)
            if mapping.hand == "left":
                # 仅在手套观测侧规范化手性，复用不变的右手规范手型拟合器。
                reference[:, 0] *= -1.0
        morphology = (
            ManoMorphologyFitter.load(
                morphology_file,
                reference_keypoints=reference,
            )
            if morphology_file is not None
            else None
        )
        if zero_profile is not None and zero_profile.hand != mapping.hand:
            raise ValueError(
                f"所选零位是 {zero_profile.hand}，"
                f"不能用于 {mapping.hand} 手套"
            )
        return cls(
            mapping=mapping,
            kinematics=kinematics,
            model=model,
            data=data,
            qpos_by_joint=qpos_by_joint,
            landmarks=landmarks,
            morphology=morphology,
            zero_profile=zero_profile,
        )

    def validate_stream(self, connection: EncoderConnection) -> None:
        self.mapping.validate_stream(
            channels=connection.channels,
            cs_by_joint=connection.cs_by_joint,
        )
        if self.zero_profile is not None:
            self.zero_profile.validate_stream(
                connection.channels,
                connection.cs_by_joint,
                zeroed=connection.zeroed,
                range_min=connection.range_min,
                range_max=connection.range_max,
            )

    def read_mediapipe(
        self,
        connection: EncoderConnection,
        *,
        trust_hardware_zero: bool,
    ) -> np.ndarray:
        """读取最新手套帧，以末端目标和方向约束拟合 21 点手型。"""

        if self.zero_profile is None:
            glove_frame = connection.read_latest_joint_frame(
                self.kinematics,
                trust_hardware_zero=trust_hardware_zero,
            )
        else:
            return self.process_frame(
                connection.read_latest_frame(),
                zero_profile=self.zero_profile,
            )
        return self.process_joint_frame(glove_frame)

    def process_frame(
        self,
        frame: EncoderFrame,
        *,
        zero_profile: UrdfZeroProfile | None,
    ) -> np.ndarray:
        """将已有编码器帧经可选零位更新为 MediaPipe 骨架。"""

        glove_frame = convert_encoder_frame_for_urdf(
            frame,
            self.kinematics,
            zero_profile=zero_profile,
        )
        return self.process_joint_frame(glove_frame)

    def process_joint_frame(self, glove_frame: GloveJointFrame) -> np.ndarray:
        """将已完成零位/机构换算的帧更新为 MediaPipe 骨架。"""

        urdf_frame = self.mapping.map_frame(glove_frame)
        apply_urdf_joint_frame(
            self.model,
            self.data,
            self.qpos_by_joint,
            urdf_frame,
        )
        return self.current_mediapipe()

    def current_mediapipe(self) -> np.ndarray:
        """将机械观测拟合为独立手型；关闭拟合时保留原始观测路径。"""

        keypoints = self.landmarks.to_mediapipe(self.data)
        if self.morphology is None:
            return keypoints
        if self.mapping.hand == "left":
            keypoints[:, 0] *= -1.0
        fitted = self.morphology.fit(keypoints)
        if self.mapping.hand == "left":
            fitted[:, 0] *= -1.0
        return fitted
