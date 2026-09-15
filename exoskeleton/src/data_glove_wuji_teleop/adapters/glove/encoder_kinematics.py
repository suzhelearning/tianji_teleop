"""将偏心编码器零位增量换算为数据手套实际关节角。"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import TypedDict

from ..._native import EncoderKinematics as _NativeEncoderKinematics
from ..._native import FourBarLinkage as _NativeFourBarLinkage

from .encoder_stream import EncoderFrame


ENCODER_CHANNEL_COUNT = 21
EXPECTED_CHANNELS_BY_GROUP = {
    "four_finger_pip": frozenset(("J12", "J13", "J14", "J15")),
    "four_finger_dip": frozenset(("J17", "J18", "J19", "J20")),
    "thumb_ip": frozenset(("J16",)),
}

Point = tuple[float, float]


class FourBarGeometry(TypedDict):
    A: Point
    B: Point
    C: Point
    D: Point
    theta_A: float
    theta_D: float
    BD: float


@dataclass(frozen=True)
class FourBarParameters:
    """一套可从 JSON 解析并完整校验的四连杆初始化参数。"""

    L_AD: float
    L_AB: float
    L_BC: float
    L_CD: float
    theta_A0_deg: float
    branch: int

    @classmethod
    def from_mapping(cls, data: Mapping[str, object]) -> "FourBarParameters":
        try:
            lengths_and_angle = (
                float(data["L_AD"]),
                float(data["L_AB"]),
                float(data["L_BC"]),
                float(data["L_CD"]),
                float(data["theta_A0_deg"]),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"四连杆参数字段无效：{exc}") from exc
        branch = data.get("branch", 1)
        if type(branch) is not int or branch not in (-1, 1):
            raise ValueError("四连杆参数 branch 只能是整数 +1 或 -1")
        return cls(*lengths_and_angle, branch=branch)

    def build_linkage(self) -> "FourBarLinkage":
        return FourBarLinkage(
            L_AD=self.L_AD,
            L_AB=self.L_AB,
            L_BC=self.L_BC,
            L_CD=self.L_CD,
            theta_A0_deg=self.theta_A0_deg,
            branch=self.branch,
        )


@dataclass(frozen=True)
class GloveJointFrame:
    """编码器机构换算后的 21 路数据手套实际关节角。"""

    sequence: int
    timestamp_ns: int
    dropped: int
    joint_angles_deg: tuple[float, ...]


class FourBarLinkage:
    """根据输入轴 A 相对硬件零位的变化量计算输出轴 D 的变化量。

    数学坐标取 ``A=(0, 0)``、``D=(L_AD, 0)``，角度逆时针为正。
    ``theta_A0_deg`` 是硬件初始化姿态下从 AD 到 AB 的有向几何角；
    ``branch`` 选择实物固定的两圆交点装配分支。
    """

    def __init__(
        self,
        L_AD: float,
        L_AB: float,
        L_BC: float,
        L_CD: float,
        theta_A0_deg: float,
        branch: int = 1,
    ) -> None:
        lengths = tuple(float(value) for value in (L_AD, L_AB, L_BC, L_CD))
        if not all(math.isfinite(value) and value > 0.0 for value in lengths):
            raise ValueError("所有杆长都必须是大于 0 的有限数值")
        theta_A0 = float(theta_A0_deg)
        if not math.isfinite(theta_A0):
            raise ValueError("theta_A0_deg 必须是有限数值")
        if type(branch) is not int or branch not in (-1, 1):
            raise ValueError("branch 只能是 +1 或 -1")

        self._native = _NativeFourBarLinkage(
            *lengths, theta_A0, branch
        )
        self.L_AD, self.L_AB, self.L_BC, self.L_CD = lengths
        self.theta_A0 = self._native.theta_A0
        self.branch = branch
        self.theta_D0 = self._native.theta_D0
        self.C0 = self._native.C0

    def solve(self, delta_A_deg: float) -> float:
        """返回输出轴 D 相对初始化姿态的角度变化，单位为度。"""

        return self._native.solve(float(delta_A_deg))

    def solve_full(self, delta_A_deg: float) -> dict[str, object]:
        """返回角度变化和当前四连杆坐标，供调试与实物参数验证使用。"""

        return self._native.solve_full(float(delta_A_deg))


@dataclass(frozen=True)
class _LinkageGroup:
    name: str
    channel_directions: tuple[tuple[int, int], ...]
    linkage: FourBarLinkage


class EncoderKinematics:
    """从 JSON 加载编码器侧机构参数，并改写一帧中的四连杆通道。"""

    def __init__(
        self,
        hand: str,
        groups: Sequence[_LinkageGroup],
        unset_input_direction_channels: Sequence[str] = (),
    ) -> None:
        self.hand = hand
        self._groups = tuple(groups)
        self._native = _NativeEncoderKinematics(
            [
                (group.name, group.channel_directions, group.linkage._native)
                for group in self._groups
            ]
        )
        self.unset_input_direction_channels = tuple(
            unset_input_direction_channels
        )

    @classmethod
    def load(
        cls,
        filepath: str | Path,
        *,
        commissioning: bool = False,
    ) -> "EncoderKinematics":
        """加载配置；方向调试模式可把未填写的输入方向临时视为 +1。"""

        path = Path(filepath)
        with path.open("r", encoding="utf-8") as file:
            data = json.load(file)
        return cls.from_dict(data, commissioning=commissioning)

    @classmethod
    def from_dict(
        cls,
        data: Mapping[str, object],
        *,
        commissioning: bool = False,
    ) -> "EncoderKinematics":
        """解析内嵌机构配置，复用文件模式的参数和方向校验。"""
        if not isinstance(data, Mapping):
            raise ValueError("编码器运动学配置根节点必须是对象")
        ready = data.get("ready", False)
        if type(ready) is not bool:
            raise ValueError("编码器运动学配置 ready 必须是布尔值")
        if not ready and not commissioning:
            raise ValueError("编码器运动学配置尚未填写实物参数")
        if int(data.get("version", 0)) != 1:
            raise ValueError("编码器运动学配置 version 必须是 1")
        hand = str(data.get("hand", ""))
        if hand not in ("left", "right"):
            raise ValueError("编码器运动学配置 hand 必须是 left 或 right")
        if data.get("angle_unit") != "degree":
            raise ValueError("编码器运动学配置 angle_unit 必须是 degree")
        if data.get("length_unit") != "millimeter":
            raise ValueError(
                "编码器运动学配置 length_unit 必须是 millimeter"
            )

        groups_raw = data.get("groups")
        if not isinstance(groups_raw, Sequence) or isinstance(
            groups_raw, (str, bytes)
        ):
            raise ValueError("编码器运动学配置 groups 必须是数组")

        groups: list[_LinkageGroup] = []
        seen_names: set[str] = set()
        unset_input_direction_channels: list[str] = []
        for raw in groups_raw:
            if not isinstance(raw, Mapping):
                raise ValueError("每个四连杆参数组都必须是对象")
            name = str(raw.get("name", ""))
            expected_channels = EXPECTED_CHANNELS_BY_GROUP.get(name)
            if expected_channels is None:
                raise ValueError(f"未知的四连杆参数组：{name}")
            if name in seen_names:
                raise ValueError(f"四连杆参数组重复：{name}")
            seen_names.add(name)

            channels_raw = raw.get("channels")
            if not isinstance(channels_raw, Sequence) or isinstance(
                channels_raw, (str, bytes)
            ):
                raise ValueError(f"{name}.channels 必须是数组")
            channels = tuple(str(value).upper() for value in channels_raw)
            if len(channels) != len(set(channels)):
                raise ValueError(f"{name}.channels 不能包含重复通道")
            if frozenset(channels) != expected_channels:
                expected = ", ".join(sorted(expected_channels))
                raise ValueError(f"{name}.channels 必须完整包含：{expected}")

            input_directions_raw = raw.get("input_directions")
            if not isinstance(input_directions_raw, Mapping):
                raise ValueError(f"{name}.input_directions 必须是对象")
            input_directions = {
                str(channel).upper(): value
                for channel, value in input_directions_raw.items()
            }
            if set(input_directions) != set(channels):
                expected = ", ".join(channels)
                raise ValueError(
                    f"{name}.input_directions 必须逐项包含：{expected}"
                )
            channel_directions: list[tuple[int, int]] = []
            for channel in channels:
                direction = input_directions[channel]
                if direction is None and commissioning:
                    direction = 1
                    unset_input_direction_channels.append(channel)
                elif type(direction) is not int or direction not in (-1, 1):
                    raise ValueError(
                        f"{name}.input_directions.{channel} 只能是 +1 或 -1"
                    )
                channel_directions.append(
                    (int(channel[1:]) - 1, direction)
                )
            parameters = raw.get("parameters")
            if not isinstance(parameters, Mapping):
                raise ValueError(f"{name}.parameters 必须是对象")
            try:
                linkage = FourBarParameters.from_mapping(
                    parameters
                ).build_linkage()
            except ValueError as exc:
                raise ValueError(f"{name}.parameters 无效：{exc}") from exc

            groups.append(
                _LinkageGroup(
                    name=name,
                    channel_directions=tuple(channel_directions),
                    linkage=linkage,
                )
            )

        missing_groups = set(EXPECTED_CHANNELS_BY_GROUP) - seen_names
        if missing_groups:
            raise ValueError(
                "编码器运动学配置缺少参数组："
                + ", ".join(sorted(missing_groups))
            )
        return cls(
            hand,
            groups,
            unset_input_direction_channels,
        )

    def convert_angles(self, encoder_angles_deg: Sequence[float]) -> list[float]:
        """把 21 路编码器零位增量转换成等长的数据手套关节角数组。"""

        return self._native.convert_angles(encoder_angles_deg)

    def convert_zeroed_frame(
        self,
        frame: EncoderFrame,
    ) -> GloveJointFrame:
        """将已经硬件校零的编码器帧转换为数据手套实际关节角帧。"""

        return GloveJointFrame(
            sequence=frame.sequence,
            timestamp_ns=frame.timestamp_ns,
            dropped=frame.dropped,
            joint_angles_deg=tuple(self.convert_angles(frame.angles_deg)),
        )
