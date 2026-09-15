"""官方 Wuji retargeting 的进程无关接口。"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Protocol

import numpy as np


class RetargetTransport(Protocol):
    """向常驻官方 retarget worker 发送一帧并等待结果。"""

    def request(self, payload: dict[str, object]) -> Mapping[str, object]: ...

    def close(self) -> None: ...


@dataclass(frozen=True)
class WujiRetargetResult:
    """官方求解器返回的一帧命名关节目标和诊断数据。"""

    qpos: np.ndarray
    joint_names: tuple[str, ...]
    transformed_keypoints: np.ndarray
    robot_wrist: np.ndarray
    robot_dip_positions: np.ndarray
    robot_tip_positions: np.ndarray
    cost: float


@dataclass(frozen=True)
class WujiTipFitMetrics:
    """五指末端位置误差和 DIP→tip 方向误差。"""

    position_error_m: np.ndarray
    direction_error_deg: np.ndarray


def compute_tip_fit_metrics(result: WujiRetargetResult) -> WujiTipFitMetrics:
    """在官方 wrist frame 中比较目标与机器人五个末端。"""

    dip_indices = (3, 7, 11, 15, 19)
    tip_indices = (4, 8, 12, 16, 20)
    target_wrist = result.transformed_keypoints[0]
    target_dips = result.transformed_keypoints[list(dip_indices)]
    target_tips = result.transformed_keypoints[list(tip_indices)]
    target_tip_vectors = target_tips - target_wrist
    robot_tip_vectors = result.robot_tip_positions - result.robot_wrist
    position_error = np.linalg.norm(
        robot_tip_vectors - target_tip_vectors,
        axis=1,
    )

    target_dirs = target_tips - target_dips
    robot_dirs = result.robot_tip_positions - result.robot_dip_positions
    target_norms = np.linalg.norm(target_dirs, axis=1)
    robot_norms = np.linalg.norm(robot_dirs, axis=1)
    if np.any(target_norms <= 1e-9) or np.any(robot_norms <= 1e-9):
        raise ValueError("DIP→tip 向量退化，无法计算方向误差")
    target_dirs /= target_norms[:, None]
    robot_dirs /= robot_norms[:, None]
    cosines = np.sum(target_dirs * robot_dirs, axis=1)
    direction_error = np.degrees(np.arccos(np.clip(cosines, -1.0, 1.0)))
    return WujiTipFitMetrics(
        position_error_m=position_error,
        direction_error_deg=direction_error,
    )


class OfficialWujiRetargetAdapter:
    """校验 MediaPipe 输入及官方 worker 的命名输出合同。"""

    def __init__(self, transport: RetargetTransport) -> None:
        self._transport = transport

    def retarget(
        self,
        keypoints: np.ndarray,
        *,
        apply_filter: bool,
    ) -> WujiRetargetResult:
        points = np.asarray(keypoints, dtype=np.float64)
        if points.shape != (21, 3) or not np.isfinite(points).all():
            raise ValueError(
                "官方 Wuji 映射输入必须是有限的 (21, 3) 关键点"
            )
        response = self._transport.request(
            {
                "keypoints": points.tolist(),
                "apply_filter": bool(apply_filter),
            }
        )
        if response.get("error"):
            raise RuntimeError(f"官方 Wuji retarget worker：{response['error']}")

        qpos = np.asarray(response.get("qpos"), dtype=np.float64)
        transformed = np.asarray(
            response.get("transformed_keypoints"),
            dtype=np.float64,
        )
        robot_wrist = np.asarray(response.get("robot_wrist"), dtype=np.float64)
        robot_dips = np.asarray(
            response.get("robot_dip_positions"),
            dtype=np.float64,
        )
        robot_tips = np.asarray(
            response.get("robot_tip_positions"),
            dtype=np.float64,
        )
        names_raw = response.get("joint_names")
        if not isinstance(names_raw, list):
            raise ValueError("官方 Wuji 输出 joint_names 必须是数组")
        joint_names = tuple(str(name) for name in names_raw)
        try:
            cost = float(response.get("cost"))
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("官方 Wuji 输出 cost 必须是有限数值") from exc
        if qpos.shape != (20,) or not np.isfinite(qpos).all():
            raise ValueError("官方 Wuji 输出 qpos 必须是有限的 20 维数组")
        if len(joint_names) != 20 or len(set(joint_names)) != 20:
            raise ValueError("官方 Wuji 输出必须包含 20 个唯一关节名")
        if transformed.shape != (21, 3) or not np.isfinite(transformed).all():
            raise ValueError("官方 Wuji 变换后关键点必须是有限的 (21, 3)")
        if robot_wrist.shape != (3,) or not np.isfinite(robot_wrist).all():
            raise ValueError(
                "官方 Wuji 机器人 wrist 必须是有限的 3 维坐标"
            )
        if robot_dips.shape != (5, 3) or not np.isfinite(robot_dips).all():
            raise ValueError("官方 Wuji 机器人 DIP 必须是有限的 (5, 3)")
        if robot_tips.shape != (5, 3) or not np.isfinite(robot_tips).all():
            raise ValueError("官方 Wuji 机器人 tip 必须是有限的 (5, 3)")
        if not math.isfinite(cost):
            raise ValueError("官方 Wuji 输出 cost 必须是有限数值")
        return WujiRetargetResult(
            qpos=qpos,
            joint_names=joint_names,
            transformed_keypoints=transformed,
            robot_wrist=robot_wrist,
            robot_dip_positions=robot_dips,
            robot_tip_positions=robot_tips,
            cost=cost,
        )

    def close(self) -> None:
        self._transport.close()

    def __enter__(self) -> "OfficialWujiRetargetAdapter":
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()
