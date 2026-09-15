"""独立的 MANO 拓扑参数化骨架；不包含授权 MANO 网格或形状参数。"""

from __future__ import annotations

import json
from collections.abc import Mapping
from pathlib import Path

import numpy as np

from ..._native import MorphologyFitter as _NativeMorphologyFitter


_FINGERS = ("thumb", "index", "middle", "ring", "pinky")


def _array(value: object, shape: tuple[int, ...], name: str) -> np.ndarray:
    """验证数值、维度和有限性，并切断与调用方数组的共享。"""
    try:
        raw = np.asarray(value)
        if raw.dtype.kind not in "iuf":
            raise ValueError
        result = np.array(raw, dtype=np.float64, copy=True)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} 必须是形状 {shape} 的有限数值数组") from exc
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"{name} 必须是形状 {shape} 的有限数值数组")
    return result


def _weight(value: object, name: str) -> float:
    number = float(_array(value, (), name))
    if number < 0.0:
        raise ValueError(f"{name} 必须是非负有限数值（米）")
    return number


class ManoMorphologyFitter:
    """固定手型、五指各四个弧度自由度，以末端位置为主目标拟合姿态。

    每指自由度为外展、基关节屈曲、中关节屈曲、远端关节屈曲。
    根点、静态关节框架和骨长仅来自配置。参考帧只确定一次掌部旋转；
    运行帧只提供腕部平移、末端目标和软方向约束，不提供模型关节点。
    """

    def __init__(
        self,
        fingers: Mapping[str, object],
        *,
        solver: Mapping[str, object],
        reference_keypoints: np.ndarray | None = None,
        pinch_distance_m: float = 0.0,
        pinch_full_contact_distance_m: float = 0.0,
        pinch_target_distance_m: float = 0.0,
    ) -> None:
        if not isinstance(fingers, Mapping) or set(fingers) != set(_FINGERS):
            raise ValueError("fingers 必须配置 thumb/index/middle/ring/pinky")
        roots, bases, lengths, limits = [], [], [], []
        for finger in _FINGERS:
            spec = fingers[finger]
            if not isinstance(spec, Mapping):
                raise ValueError(f"{finger} 配置必须是对象")
            root = _array(spec.get("root_m"), (3,), f"{finger}.root_m")
            basis = _array(spec.get("rest_basis"), (3, 3), f"{finger}.rest_basis")
            if not np.allclose(basis.T @ basis, np.eye(3), rtol=0.0, atol=1e-7) or not np.isclose(
                np.linalg.det(basis), 1.0, rtol=0.0, atol=1e-7
            ):
                raise ValueError(f"{finger}.rest_basis 必须是右手正交旋转（列为局部基向量）")
            segment_lengths = _array(
                spec.get("segment_lengths_m"), (3,), f"{finger}.segment_lengths_m"
            )
            if np.any(segment_lengths <= 0.0):
                raise ValueError(f"{finger}.segment_lengths_m 必须为正数（米）")
            joint_limits = _array(
                spec.get("joint_limits_rad"), (4, 2), f"{finger}.joint_limits_rad"
            )
            if np.any(joint_limits[:, 0] > joint_limits[:, 1]):
                raise ValueError(f"{finger}.joint_limits_rad 下限不得大于上限（弧度）")
            roots.append(root)
            bases.append(basis)
            lengths.append(segment_lengths)
            limits.append(joint_limits)
        if not isinstance(solver, Mapping):
            raise ValueError("solver 必须是对象")
        self._direction_weight = _weight(solver.get("direction_weight_m"), "direction_weight_m")
        self._temporal_weight = _weight(solver.get("temporal_weight_m"), "temporal_weight_m")
        iterations = solver.get("max_iterations")
        if isinstance(iterations, bool) or not isinstance(iterations, int) or iterations <= 0:
            raise ValueError("max_iterations 必须是正整数")
        self._max_iterations = iterations
        self._pinch_distance = _weight(pinch_distance_m, "pinch_distance_m")
        self._pinch_full_contact = _weight(
            pinch_full_contact_distance_m, "pinch_full_contact_distance_m"
        )
        self._pinch_target = _weight(pinch_target_distance_m, "pinch_target_distance_m")
        if self._pinch_distance == 0.0:
            if self._pinch_full_contact != 0.0 or self._pinch_target != 0.0:
                raise ValueError("关闭吸附时，三个 pinch 距离必须全部为 0")
        elif not self._pinch_target <= self._pinch_full_contact < self._pinch_distance:
            raise ValueError("pinch 距离必须满足 0 <= 目标间距 <= 完全吸附距离 < 触发距离")
        self._roots = np.stack(roots)
        self._bases = np.stack(bases)
        self._lengths = np.stack(lengths)
        self._limits = np.stack(limits)
        self._rotation = np.eye(3)
        if reference_keypoints is not None:
            reference = _array(reference_keypoints, (21, 3), "reference_keypoints")
            y = reference[9] - reference[0]
            y_length = np.linalg.norm(y)
            if not np.isfinite(y_length) or y_length <= 1e-9:
                raise ValueError("参考帧腕部到中指根点方向退化")
            y /= y_length
            x = reference[5] - reference[17]
            x -= y * np.dot(y, x)
            x_length = np.linalg.norm(x)
            if not np.isfinite(x_length) or x_length <= 1e-9:
                raise ValueError("参考帧掌部横向方向退化")
            x /= x_length
            self._rotation = np.column_stack((x, y, np.cross(x, y)))
        for parameter in (self._roots, self._bases, self._lengths, self._limits, self._rotation):
            parameter.setflags(write=False)
        self._native = _NativeMorphologyFitter(
            self._roots,
            self._bases,
            self._lengths,
            self._limits,
            self._rotation,
            self._direction_weight,
            self._temporal_weight,
            self._max_iterations,
            self._pinch_distance,
            self._pinch_full_contact,
            self._pinch_target,
        )

    @classmethod
    def load(
        cls,
        path: str | Path,
        *,
        reference_keypoints: np.ndarray | None = None,
    ) -> "ManoMorphologyFitter":
        with Path(path).open(encoding="utf-8") as file:
            data = json.load(file)
        if not isinstance(data, Mapping):
            raise ValueError("手型配置根节点必须是对象")
        if type(data.get("version")) is not int or data["version"] != 2:
            raise ValueError("手型配置 version 必须是 2；不支持旧版骨长重建配置")
        if data.get("hand") != "right" or data.get("length_unit") != "meter":
            raise ValueError("手型配置必须使用 right 手和 meter 长度单位")
        if data.get("model") != "parametric_mano_skeleton":
            raise ValueError("model 必须是 parametric_mano_skeleton（不是完整 MANO 网格）")
        return cls(
            data.get("fingers"),
            solver=data.get("solver"),
            reference_keypoints=reference_keypoints,
            pinch_distance_m=data.get("pinch_distance_m", 0.0),
            pinch_full_contact_distance_m=data.get("pinch_full_contact_distance_m", 0.0),
            pinch_target_distance_m=data.get("pinch_target_distance_m", 0.0),
        )

    @property
    def _previous_pose(self) -> np.ndarray | None:
        """按需读取最终输出历史，不在逐帧路径复制状态。"""
        return self._native.previous_pose

    @property
    def _previous_observed_pose(self) -> np.ndarray | None:
        """普通拟合独立保留历史，避免吸附重复压缩上一帧间距。"""
        return self._native.previous_observed_pose

    def forward(self, pose: np.ndarray, *, wrist: np.ndarray | None = None) -> np.ndarray:
        """由 (5, 4) 弧度姿态生成米制 21 点；越限姿态报错，不隐式修正。"""
        return self._native.forward(pose, wrist)

    def fit(self, keypoints: np.ndarray) -> np.ndarray:
        """拟合米制观测；不可达目标保留合法近似解和残差，坏帧不更新状态。"""
        return self._native.fit(keypoints)
