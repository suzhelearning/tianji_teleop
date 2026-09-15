#!/usr/bin/env python3
"""数据手套 21 路角度到后端无关 20-DOF 语义目标的纯映射层。"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping, Sequence

from ...domain.hand_target import DOF_ORDER


CHANNEL_COUNT = 21
CALIBRATED_CHANNEL_COUNT = 20
DEFAULT_THUMB_METACARPAL_OUTPUT_RAD = math.radians(60.0)


def shortest_angular_delta_deg(value_deg: float, reference_deg: float) -> float:
    """返回编码器角度相对参考角的最短有符号圆周差。"""

    value = float(value_deg)
    reference = float(reference_deg)
    if not math.isfinite(value) or not math.isfinite(reference):
        raise ValueError("手套角度和参考零位必须是有限数值")
    raw_delta = value - reference
    wrapped = (raw_delta + 180.0) % 360.0 - 180.0
    # 在恰好半圈时保留原始方向，避免 +180 被无条件翻成 -180。
    if wrapped == -180.0 and raw_delta > 0.0:
        return 180.0
    return wrapped

# 手套负载是“按关节层分组”，语义目标则是“按手指分组”：
#   [侧摆5路, 屈伸1五路, 屈伸2五路, 屈伸3五路, 小指额外侧摆]
# -> 每指 [屈伸1, 侧摆, 屈伸2, 屈伸3]。
GLOVE_INDEX_BY_FINGER_TARGET: tuple[tuple[int, int, int, int], ...] = (
    (5, 0, 10, 15),
    (6, 1, 11, 16),
    (7, 2, 12, 17),
    (8, 3, 13, 18),
    (9, 4, 14, 19),
)

@dataclass(frozen=True)
class ReferencePoint:
    """一个手套输入端点及其对应的语义输出角。"""

    input_deg: float
    output_rad: float

    @classmethod
    def from_dict(cls, data: Mapping[str, object]) -> "ReferencePoint":
        point = cls(
            input_deg=float(data["input_deg"]),
            output_rad=float(data["output_rad"]),
        )
        if not math.isfinite(point.input_deg) or not math.isfinite(point.output_rad):
            raise ValueError("标定端点必须是有限数值")
        if point.input_deg == 0.0:
            raise ValueError("标定端点不能位于零点")
        if abs(point.input_deg) >= 180.0:
            raise ValueError("标定端点必须位于最短圆周角差 (-180, 180) 内")
        return point

    def to_dict(self) -> dict[str, float]:
        return {
            "input_deg": self.input_deg,
            "output_rad": self.output_rad,
        }


@dataclass(frozen=True)
class AxisCalibration:
    """以标定零位为中心的单轴分段线性尺度标定。"""

    enabled: bool = False
    deadband_deg: float = 0.0
    negative: ReferencePoint | None = None
    positive: ReferencePoint | None = None

    @classmethod
    def from_dict(cls, data: Mapping[str, object]) -> "AxisCalibration":
        negative_raw = data.get("negative")
        positive_raw = data.get("positive")
        axis = cls(
            enabled=bool(data.get("enabled", True)),
            deadband_deg=float(data.get("deadband_deg", 0.0)),
            negative=(
                ReferencePoint.from_dict(negative_raw)
                if isinstance(negative_raw, Mapping)
                else None
            ),
            positive=(
                ReferencePoint.from_dict(positive_raw)
                if isinstance(positive_raw, Mapping)
                else None
            ),
        )
        if not math.isfinite(axis.deadband_deg) or axis.deadband_deg < 0.0:
            raise ValueError("deadband_deg 必须是非负有限数值")
        if axis.negative is not None and axis.negative.input_deg >= 0.0:
            raise ValueError("negative 端点的 input_deg 必须小于 0")
        if axis.positive is not None and axis.positive.input_deg <= 0.0:
            raise ValueError("positive 端点的 input_deg 必须大于 0")
        for point in (axis.negative, axis.positive):
            if point is not None and abs(point.input_deg) <= axis.deadband_deg:
                raise ValueError("标定端点必须位于死区之外")
        if axis.enabled and axis.negative is None and axis.positive is None:
            raise ValueError("启用通道至少需要一个标定端点")
        return axis

    def map(self, angle_deg: float) -> float:
        value = float(angle_deg)
        if not math.isfinite(value):
            raise ValueError("手套角度必须是有限数值")
        if not self.enabled or abs(value) <= self.deadband_deg:
            return 0.0

        point = self.positive if value > 0.0 else self.negative
        if point is None:
            return 0.0

        usable_input = abs(point.input_deg) - self.deadband_deg
        progress = (abs(value) - self.deadband_deg) / usable_input
        return point.output_rad * max(0.0, min(1.0, progress))

    def to_dict(self) -> dict[str, object]:
        data: dict[str, object] = {
            "enabled": self.enabled,
            "deadband_deg": self.deadband_deg,
        }
        if self.negative is not None:
            data["negative"] = self.negative.to_dict()
        if self.positive is not None:
            data["positive"] = self.positive.to_dict()
        return data


@dataclass(frozen=True)
class CalibrationProfile:
    """可持久化的全手尺度标定配置。"""

    hand: str
    cs_by_joint: tuple[int, ...]
    axes: tuple[AxisCalibration, ...]
    zero_offsets_deg: tuple[float, ...] = (0.0,) * CHANNEL_COUNT
    input_source_indices: tuple[int, ...] = tuple(
        range(CALIBRATED_CHANNEL_COUNT)
    )
    version: int = 1
    channels: int = CHANNEL_COUNT

    def __post_init__(self) -> None:
        if self.hand not in ("left", "right"):
            raise ValueError("hand 必须是 left 或 right")
        if self.version != 1:
            raise ValueError(f"不支持的标定版本: {self.version}")
        if self.channels != CHANNEL_COUNT:
            raise ValueError(f"标定通道数必须是 {CHANNEL_COUNT}")
        if (
            len(self.cs_by_joint) != CHANNEL_COUNT
            or sorted(self.cs_by_joint) != list(range(CHANNEL_COUNT))
        ):
            raise ValueError("cs_by_joint 必须是 0..20 的排列")
        if len(self.axes) != CALIBRATED_CHANNEL_COUNT:
            raise ValueError(
                f"标定轴必须包含 {CALIBRATED_CHANNEL_COUNT} 路"
            )
        if (
            len(self.zero_offsets_deg) != CHANNEL_COUNT
            or not all(
                math.isfinite(value)
                for value in self.zero_offsets_deg
            )
        ):
            raise ValueError(
                f"zero_offsets_deg 必须包含 {CHANNEL_COUNT} 个有限数值"
            )
        if len(self.input_source_indices) != CALIBRATED_CHANNEL_COUNT:
            raise ValueError(
                f"输入源必须包含 {CALIBRATED_CHANNEL_COUNT} 路"
            )
        if any(
            index < 0 or index >= CHANNEL_COUNT
            for index in self.input_source_indices
        ):
            raise ValueError(
                f"输入源索引必须位于 0..{CHANNEL_COUNT - 1}"
            )

    @classmethod
    def from_dict(cls, data: Mapping[str, object]) -> "CalibrationProfile":
        version = int(data.get("version", 0))
        if version != 1:
            raise ValueError(f"不支持的标定版本: {version}")
        hand = str(data.get("hand", ""))
        if hand not in ("left", "right"):
            raise ValueError("hand 必须是 left 或 right")
        channels = int(data.get("channels", 0))
        if channels != CHANNEL_COUNT:
            raise ValueError(f"标定通道数必须是 {CHANNEL_COUNT}")
        offsets_raw = data.get(
            "zero_offsets_deg",
            [0.0] * CHANNEL_COUNT,
        )
        if not isinstance(offsets_raw, Sequence) or isinstance(
            offsets_raw,
            (str, bytes),
        ):
            raise ValueError("zero_offsets_deg 必须是数组")
        zero_offsets_deg = tuple(float(value) for value in offsets_raw)
        if (
            len(zero_offsets_deg) != CHANNEL_COUNT
            or not all(math.isfinite(value) for value in zero_offsets_deg)
        ):
            raise ValueError(
                f"zero_offsets_deg 必须包含 {CHANNEL_COUNT} 个有限数值"
            )

        mapping_raw = data.get("cs_by_joint")
        if not isinstance(mapping_raw, Sequence) or isinstance(mapping_raw, (str, bytes)):
            raise ValueError("cs_by_joint 必须是数组")
        cs_by_joint = tuple(int(value) for value in mapping_raw)
        if len(cs_by_joint) != CHANNEL_COUNT or sorted(cs_by_joint) != list(
            range(CHANNEL_COUNT)
        ):
            raise ValueError("cs_by_joint 必须是 0..20 的排列")

        input_sources_raw = data.get("input_sources", {})
        if not isinstance(input_sources_raw, Mapping):
            raise ValueError("input_sources 必须是对象")
        input_source_indices = list(range(CALIBRATED_CHANNEL_COUNT))
        valid_targets = {
            f"J{index}"
            for index in range(1, CALIBRATED_CHANNEL_COUNT + 1)
        }
        valid_sources = {
            f"J{index}"
            for index in range(1, CHANNEL_COUNT + 1)
        }
        for target_raw, source_raw in input_sources_raw.items():
            target = str(target_raw).upper()
            source = str(source_raw).upper()
            if target not in valid_targets:
                raise ValueError(f"input_sources 目标通道无效: {target}")
            if source not in valid_sources:
                raise ValueError(f"input_sources 源通道无效: {source}")
            input_source_indices[int(target[1:]) - 1] = int(source[1:]) - 1

        axes_raw = data.get("axes", {})
        if not isinstance(axes_raw, Mapping):
            raise ValueError("axes 必须是对象")
        axes: list[AxisCalibration] = []
        for index in range(1, CALIBRATED_CHANNEL_COUNT + 1):
            raw = axes_raw.get(f"J{index}", {"enabled": False})
            if not isinstance(raw, Mapping):
                raise ValueError(f"J{index} 标定必须是对象")
            axes.append(AxisCalibration.from_dict(raw))

        return cls(
            hand=hand,
            cs_by_joint=cs_by_joint,
            axes=tuple(axes),
            zero_offsets_deg=zero_offsets_deg,
            input_source_indices=tuple(input_source_indices),
            version=version,
            channels=channels,
        )

    def map_to_finger_target(
        self,
        angles_deg: Sequence[float],
    ) -> list[list[float]]:
        """映射为按拇指到小指排列的四轴语义目标。"""

        if len(angles_deg) != CHANNEL_COUNT:
            raise ValueError(f"手套帧必须包含 {CHANNEL_COUNT} 个角度")
        calibrated = [
            self.axes[index].map(
                shortest_angular_delta_deg(
                    angles_deg[source_index],
                    self.zero_offsets_deg[source_index],
                )
            )
            for index, source_index in enumerate(self.input_source_indices)
        ]
        return [
            [calibrated[index] for index in finger_indices]
            for finger_indices in GLOVE_INDEX_BY_FINGER_TARGET
        ]

    def map_to_dof_values(self, angles_deg: Sequence[float]) -> list[float]:
        target = self.map_to_finger_target(angles_deg)
        values: list[float] = []
        for finger_index in range(1, 5):
            flex1, lateral, flex2, flex3 = target[finger_index]
            values.extend((lateral, flex1, flex2, flex3))
        thumb_flex1, thumb_lateral, thumb_flex2, thumb_flex3 = target[0]
        values.extend((thumb_flex1, thumb_lateral, thumb_flex2, thumb_flex3))
        return values

    def validate_stream(
        self,
        *,
        channels: int,
        cs_by_joint: Iterable[int],
    ) -> None:
        if int(channels) != self.channels:
            raise ValueError(
                f"手套返回 {channels} 路，标定文件要求 {self.channels} 路"
            )
        current_mapping = tuple(int(value) for value in cs_by_joint)
        if current_mapping != self.cs_by_joint:
            raise ValueError(
                "板端 enc_map 与标定文件不一致，请恢复映射或重新标定"
            )

    @classmethod
    def load(cls, filepath: str | Path) -> "CalibrationProfile":
        path = Path(filepath)
        with path.open("r", encoding="utf-8") as file:
            data = json.load(file)
        if not isinstance(data, Mapping):
            raise ValueError("标定文件根节点必须是对象")
        return cls.from_dict(data)

    def save(self, filepath: str | Path) -> None:
        path = Path(filepath)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as file:
            json.dump(self.to_dict(), file, indent=2, ensure_ascii=False)
            file.write("\n")

    def to_dict(self) -> dict[str, object]:
        return {
            "version": self.version,
            "hand": self.hand,
            "channels": self.channels,
            "zero_offsets_deg": list(self.zero_offsets_deg),
            "source_order": "J1..J21",
            "target_order": "finger1..finger5 x [flex1,lateral,flex2,flex3]",
            "ignored_channels": ["J21"],
            "cs_by_joint": list(self.cs_by_joint),
            "input_sources": {
                f"J{index + 1}": f"J{source_index + 1}"
                for index, source_index in enumerate(
                    self.input_source_indices
                )
                if source_index != index
            },
            "axes": {
                f"J{index + 1}": axis.to_dict()
                for index, axis in enumerate(self.axes)
            },
        }


def _axis_with_reused_input(
    source_axis: AxisCalibration,
    *,
    negative_output_rad: float | None,
    positive_output_rad: float | None,
) -> AxisCalibration:
    if not source_axis.enabled:
        raise ValueError("复用源通道尚未完成有效标定")
    negative = (
        ReferencePoint(
            input_deg=source_axis.negative.input_deg,
            output_rad=negative_output_rad,
        )
        if source_axis.negative is not None
        and negative_output_rad is not None
        else None
    )
    positive = (
        ReferencePoint(
            input_deg=source_axis.positive.input_deg,
            output_rad=positive_output_rad,
        )
        if source_axis.positive is not None
        and positive_output_rad is not None
        else None
    )
    if negative is None and positive is None:
        raise ValueError("复用源与目标没有共同的有效输入方向")
    return AxisCalibration(
        enabled=True,
        deadband_deg=source_axis.deadband_deg,
        negative=negative,
        positive=positive,
    )


def _flex_output_or_default(
    axis: AxisCalibration,
    default_output_rad: float,
) -> float:
    points = [
        point
        for point in (axis.negative, axis.positive)
        if point is not None
    ]
    if not points:
        return default_output_rad
    outputs = {point.output_rad for point in points}
    if len(outputs) != 1:
        raise ValueError("屈伸目标同时包含不一致的正负输出方向")
    return points[0].output_rad


def apply_faulty_encoder_reuse(
    profile: CalibrationProfile,
) -> CalibrationProfile:
    """应用临时坏编码器复用，同时保留大拇指 J1 独立侧摆。"""

    axes = list(profile.axes)
    input_sources = list(profile.input_source_indices)

    for target_index, source_index in (
        (9, 8),    # J10 <- J9
        (14, 13),  # J15 <- J14
        (19, 18),  # J20 <- J19
    ):
        source_axis = axes[source_index]
        if not source_axis.enabled:
            raise ValueError(
                f"J{source_index + 1} 尚未有效标定，"
                f"不能供 J{target_index + 1} 复用"
            )
        axes[target_index] = source_axis
        input_sources[target_index] = source_index

    # 无名指侧摆 J4 本身已屏蔽，按现场决策让小指侧摆 J5 保持为 0。
    axes[4] = AxisCalibration(
        enabled=False,
        deadband_deg=axes[4].deadband_deg,
    )
    input_sources[4] = 4

    thumb_tip_axis = axes[15]  # J16
    thumb_metacarpal_output = _flex_output_or_default(
        axes[5],
        DEFAULT_THUMB_METACARPAL_OUTPUT_RAD,
    )

    axes[5] = _axis_with_reused_input(
        thumb_tip_axis,
        negative_output_rad=thumb_metacarpal_output,
        positive_output_rad=thumb_metacarpal_output,
    )
    input_sources[5] = 15

    return CalibrationProfile(
        hand=profile.hand,
        cs_by_joint=profile.cs_by_joint,
        axes=tuple(axes),
        zero_offsets_deg=profile.zero_offsets_deg,
        input_source_indices=tuple(input_sources),
        version=profile.version,
        channels=profile.channels,
    )


def build_profile_from_captures(
    *,
    hand: str,
    cs_by_joint: Iterable[int],
    flex_endpoints_deg: Mapping[str, float],
    flex_targets_rad: Mapping[str, float],
    lateral_ranges_deg: Mapping[str, tuple[float, float]],
    lateral_targets_rad: Mapping[str, tuple[float, float]],
    disabled_channels: Iterable[str] = (),
    deadband_deg: float = 0.0,
    zero_offsets_deg: Iterable[float] = (0.0,) * CHANNEL_COUNT,
) -> CalibrationProfile:
    """由已稳定采样的屈伸端点和侧摆扫角生成全手标定配置。"""

    disabled = {str(channel).upper() for channel in disabled_channels}
    known = {f"J{index}" for index in range(1, CALIBRATED_CHANNEL_COUNT + 1)}
    unknown = disabled - known
    if unknown:
        raise ValueError(f"未知禁用通道: {', '.join(sorted(unknown))}")
    if not math.isfinite(deadband_deg) or deadband_deg < 0.0:
        raise ValueError("deadband_deg 必须是非负有限数值")

    axes: dict[str, dict[str, object]] = {}
    for index in range(1, CALIBRATED_CHANNEL_COUNT + 1):
        joint = f"J{index}"
        if joint in disabled:
            axes[joint] = {
                "enabled": False,
                "deadband_deg": float(deadband_deg),
            }
            continue

        if index <= 5:
            if joint not in lateral_ranges_deg or joint not in lateral_targets_rad:
                raise ValueError(f"{joint} 缺少侧摆扫角或输出目标")
            input_min, input_max = (
                float(value) for value in lateral_ranges_deg[joint]
            )
            output_min, output_max = (
                float(value) for value in lateral_targets_rad[joint]
            )
            if not all(
                math.isfinite(value)
                for value in (input_min, input_max, output_min, output_max)
            ):
                raise ValueError(f"{joint} 侧摆端点必须是有限数值")
            if input_min > input_max:
                raise ValueError(f"{joint} 侧摆最小值不能大于最大值")

            has_negative = input_min < -deadband_deg
            has_positive = input_max > deadband_deg
            if not has_negative and not has_positive:
                raise ValueError(f"{joint} 侧摆至少一个方向必须越过死区")
            axis: dict[str, object] = {
                "enabled": True,
                "deadband_deg": float(deadband_deg),
            }
            if has_negative:
                axis["negative"] = {
                    "input_deg": input_min,
                    "output_rad": output_min,
                }
            if has_positive:
                axis["positive"] = {
                    "input_deg": input_max,
                    "output_rad": output_max,
                }
            axes[joint] = axis
            continue

        if joint not in flex_endpoints_deg or joint not in flex_targets_rad:
            raise ValueError(f"{joint} 缺少屈伸端点或输出目标")
        input_endpoint = float(flex_endpoints_deg[joint])
        output_endpoint = float(flex_targets_rad[joint])
        if not math.isfinite(input_endpoint) or abs(input_endpoint) <= deadband_deg:
            raise ValueError(f"{joint} 屈伸端点必须越过死区")
        point_name = "positive" if input_endpoint > 0.0 else "negative"
        axes[joint] = {
            "enabled": True,
            "deadband_deg": float(deadband_deg),
            point_name: {
                "input_deg": input_endpoint,
                "output_rad": output_endpoint,
            },
        }

    return CalibrationProfile.from_dict(
        {
            "version": 1,
            "hand": hand,
            "channels": CHANNEL_COUNT,
            "zero_offsets_deg": list(zero_offsets_deg),
            "cs_by_joint": list(cs_by_joint),
            "axes": axes,
        }
    )
