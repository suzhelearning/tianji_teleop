"""从外骨骼 URDF 提取 21 点机械观测，供末端与骨段方向约束使用。"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from ..simulation.dataglove_skeleton import HandSkeleton


MEDIAPIPE_LANDMARK_COUNT = 21
_MIN_SEGMENT_LENGTH_M = 1e-6


@dataclass(frozen=True)
class GloveLandmarkAdapter:
    """折叠机械冗余轴；观测锚点不代表独立手型的解剖关节点。"""

    skeleton: HandSkeleton

    @classmethod
    def from_model(cls, model: mujoco.MjModel) -> "GloveLandmarkAdapter":
        return cls(skeleton=HandSkeleton.from_model(model))

    def to_mediapipe(self, data: mujoco.MjData) -> np.ndarray:
        """返回世界根节点及五指各三个机械锚点和虚拟指尖，单位米。"""

        points = np.empty((MEDIAPIPE_LANDMARK_COUNT, 3), dtype=np.float64)
        points[0] = data.xpos[0]
        for finger_index, chain in enumerate(self.skeleton.chains):
            target_start = 1 + finger_index * 4
            selected_joint_ids = chain.joint_ids[-3:]
            points[target_start : target_start + 3] = data.xanchor[
                list(selected_joint_ids)
            ]
            points[target_start + 3] = self.skeleton.fingertip_position(
                data,
                chain,
            )
            segment_lengths = np.linalg.norm(
                np.diff(points[target_start : target_start + 4], axis=0),
                axis=1,
            )
            if (
                not np.isfinite(segment_lengths).all()
                or np.any(segment_lengths <= _MIN_SEGMENT_LENGTH_M)
            ):
                raise ValueError(f"{chain.name} MANO 骨段退化")
        if not np.isfinite(points).all():
            raise ValueError("手套 MediaPipe 关键点包含非有限坐标")
        return points
