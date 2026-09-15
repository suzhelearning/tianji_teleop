"""实物数据手套 21 路关节帧到自身 URDF 关节帧的显式映射。"""

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...adapters.glove.encoder_kinematics import GloveJointFrame


URDF_JOINT_COUNT = 21


@dataclass(frozen=True)
class UrdfJointFrame:
    """按 URDF 声明顺序排列、使用弧度的完整关节帧。"""

    sequence: int
    timestamp_ns: int
    dropped: int
    joint_names: tuple[str, ...]
    positions_rad: tuple[float, ...]


@dataclass(frozen=True)
class _UrdfJointRule:
    joint_name: str
    source_index: int
    direction: int
    scale: float


class DataGloveUrdfMapping:
    """加载并校验 J1～J21 到实物数据手套 URDF 的映射。"""

    def __init__(
        self,
        *,
        hand: str,
        directions_verified: bool,
        expected_cs_by_joint: Sequence[int],
        disabled_joint_names: Sequence[str],
        rules: Sequence[_UrdfJointRule],
    ) -> None:
        self.hand = hand
        self.directions_verified = directions_verified
        self.expected_cs_by_joint = tuple(expected_cs_by_joint)
        self._disabled_joint_names = frozenset(disabled_joint_names)
        self._rules = tuple(rules)

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(rule.joint_name for rule in self._rules)

    @classmethod
    def load(cls, filepath: str | Path) -> "DataGloveUrdfMapping":
        path = Path(filepath)
        with path.open("r", encoding="utf-8") as file:
            data = json.load(file)
        return cls.from_dict(data)

    @classmethod
    def from_dict(cls, data: Mapping[str, object]) -> "DataGloveUrdfMapping":
        if not isinstance(data, Mapping):
            raise ValueError("数据手套 URDF 映射根节点必须是对象")
        if int(data.get("version", 0)) != 1:
            raise ValueError("数据手套 URDF 映射 version 必须是 1")
        hand = str(data.get("hand", ""))
        if hand not in ("left", "right"):
            raise ValueError("数据手套 URDF 映射 hand 必须是 left 或 right")
        if int(data.get("channels", 0)) != URDF_JOINT_COUNT:
            raise ValueError(f"数据手套 URDF 映射必须包含 {URDF_JOINT_COUNT} 路")
        if data.get("input_angle_unit") != "degree":
            raise ValueError("input_angle_unit 必须是 degree")
        if data.get("output_angle_unit") != "radian":
            raise ValueError("output_angle_unit 必须是 radian")
        directions_verified = data.get("directions_verified")
        if type(directions_verified) is not bool:
            raise ValueError("directions_verified 必须是布尔值")

        cs_raw = data.get("expected_cs_by_joint")
        if not isinstance(cs_raw, Sequence) or isinstance(cs_raw, (str, bytes)):
            raise ValueError("expected_cs_by_joint 必须是数组")
        expected_cs_by_joint = tuple(int(value) for value in cs_raw)
        if (
            len(expected_cs_by_joint) != URDF_JOINT_COUNT
            or sorted(expected_cs_by_joint) != list(range(URDF_JOINT_COUNT))
        ):
            raise ValueError("expected_cs_by_joint 必须是 0..20 的排列")

        joints_raw = data.get("joints")
        if not isinstance(joints_raw, Sequence) or isinstance(
            joints_raw, (str, bytes)
        ):
            raise ValueError("joints 必须是数组")

        rules: list[_UrdfJointRule] = []
        for item in joints_raw:
            if not isinstance(item, Mapping):
                raise ValueError("每个 URDF 关节映射必须是对象")
            joint_name = str(item.get("joint_name", "")).strip()
            if not joint_name:
                raise ValueError("joint_name 不能为空")
            channel = str(item.get("source_channel", "")).upper()
            if not channel.startswith("J") or not channel[1:].isdigit():
                raise ValueError(f"source_channel 无效：{channel}")
            source_index = int(channel[1:]) - 1
            if not 0 <= source_index < URDF_JOINT_COUNT:
                raise ValueError(f"source_channel 超出 J1..J21：{channel}")
            direction = item.get("direction")
            if isinstance(direction, bool) or direction not in (-1, 1):
                raise ValueError(f"{joint_name}.direction 只能是 +1 或 -1")
            scale_raw = item.get("scale", 1.0)
            try:
                scale = float(scale_raw)
            except (TypeError, ValueError, OverflowError) as exc:
                raise ValueError(
                    f"{joint_name}.scale 必须是大于 0 的有限数值"
                ) from exc
            if (
                isinstance(scale_raw, bool)
                or not math.isfinite(scale)
                or scale <= 0.0
            ):
                raise ValueError(
                    f"{joint_name}.scale 必须是大于 0 的有限数值"
                )
            rules.append(
                _UrdfJointRule(
                    joint_name=joint_name,
                    source_index=source_index,
                    direction=int(direction),
                    scale=scale,
                )
            )

        if len(rules) != URDF_JOINT_COUNT:
            raise ValueError(f"joints 必须恰好包含 {URDF_JOINT_COUNT} 项")
        names = [rule.joint_name for rule in rules]
        if len(set(names)) != URDF_JOINT_COUNT:
            raise ValueError("URDF 关节名不能重复")
        source_indices = [rule.source_index for rule in rules]
        if sorted(source_indices) != list(range(URDF_JOINT_COUNT)):
            raise ValueError("source_channel 必须完整且不重复地包含 J1..J21")

        disabled_raw = data.get("disabled_joint_names", ())
        if not isinstance(disabled_raw, Sequence) or isinstance(
            disabled_raw,
            (str, bytes),
        ):
            raise ValueError("disabled_joint_names 必须是数组")
        disabled_joint_names = tuple(
            str(value).strip() for value in disabled_raw
        )
        if any(not name for name in disabled_joint_names):
            raise ValueError("disabled_joint_names 不能包含空关节名")
        if len(set(disabled_joint_names)) != len(disabled_joint_names):
            raise ValueError("disabled_joint_names 不能重复")
        unknown_disabled = set(disabled_joint_names) - set(names)
        if unknown_disabled:
            raise ValueError(
                "disabled_joint_names 包含未知关节："
                f"{sorted(unknown_disabled)}"
            )

        return cls(
            hand=hand,
            directions_verified=directions_verified,
            expected_cs_by_joint=expected_cs_by_joint,
            disabled_joint_names=disabled_joint_names,
            rules=rules,
        )

    def require_verified_directions(self) -> None:
        """正式运行前要求全部编码器与 URDF 正方向已通过实物确认。"""

        if not self.directions_verified:
            raise ValueError(
                "数据手套与 URDF 的方向尚未完成实物验证；"
                "请使用 --commission-directions 调试并更新映射配置"
            )

    def require_all_joints_enabled(self) -> None:
        """完整 21 点骨架拟合不允许静默固定任何 URDF 关节。"""

        if self._disabled_joint_names:
            raise ValueError(
                "完整骨架 retarget 不允许禁用 URDF 关节："
                f"{sorted(self._disabled_joint_names)}"
            )

    def validate_stream(
        self,
        *,
        channels: int,
        cs_by_joint: Iterable[int],
    ) -> None:
        """确认当前板端仍按本映射所对应的 21 路接线运行。"""

        if int(channels) != URDF_JOINT_COUNT:
            raise ValueError(
                f"板端返回 {channels} 路，实物 URDF 映射要求 "
                f"{URDF_JOINT_COUNT} 路"
            )
        current = tuple(int(value) for value in cs_by_joint)
        if current != self.expected_cs_by_joint:
            raise ValueError(
                "板端 enc_map 与实物 URDF 映射不一致："
                f"配置={list(self.expected_cs_by_joint)}，"
                f"板端={list(current)}"
            )

    def validate_urdf(self, filepath: str | Path) -> None:
        """要求配置关节顺序与目标 URDF 的全部可动关节严格一致。"""

        path = Path(filepath)
        root = ET.parse(path).getroot()
        urdf_joint_names = tuple(
            joint.get("name", "")
            for joint in root.findall("joint")
            if joint.get("type") != "fixed"
        )
        if urdf_joint_names != self.joint_names:
            raise ValueError(
                "配置关节顺序与目标 URDF 不一致："
                f"配置={self.joint_names}，URDF={urdf_joint_names}"
            )

    def map_frame(self, frame: "GloveJointFrame") -> UrdfJointFrame:
        """将已完成四连杆换算的 J 顺序关节帧映射到 URDF 顺序。"""

        angles = tuple(float(value) for value in frame.joint_angles_deg)
        if len(angles) != URDF_JOINT_COUNT:
            raise ValueError(
                f"实物关节帧必须包含 {URDF_JOINT_COUNT} 路角度"
            )
        if not all(math.isfinite(value) for value in angles):
            raise ValueError("实物关节帧包含非有限角度")
        positions = tuple(
            0.0
            if rule.joint_name in self._disabled_joint_names
            else math.radians(
                rule.direction * rule.scale * angles[rule.source_index]
            )
            for rule in self._rules
        )
        return UrdfJointFrame(
            sequence=int(frame.sequence),
            timestamp_ns=int(frame.timestamp_ns),
            dropped=int(frame.dropped),
            joint_names=self.joint_names,
            positions_rad=positions,
        )
