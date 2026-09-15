"""从官方 Wuji Hand 2 MJCF 读取真机命令限位。"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

from ...profiles.joint_mapping import load_joint_mapping
from ...profiles.wuji_v2.model import full_joint_name
from .command_safety import FIRMWARE_DOF_ORDER


def _parse_ctrlrange(raw: str | None, *, actuator: str) -> tuple[float, float]:
    if raw is None:
        raise ValueError(f"二代官方执行器 {actuator} 缺少 ctrlrange")
    try:
        values = tuple(float(value) for value in raw.split())
    except ValueError as exc:
        raise ValueError(
            f"二代官方执行器 {actuator} 的 ctrlrange 非法"
        ) from exc
    if (
        len(values) != 2
        or not all(math.isfinite(value) for value in values)
        or values[0] > values[1]
    ):
        raise ValueError(
            f"二代官方执行器 {actuator} 的 ctrlrange 必须是有效上下限"
        )
    if not values[0] <= 0.0 <= values[1]:
        raise ValueError(
            f"二代官方执行器 {actuator} 的 ctrlrange 不包含零位"
        )
    return values[0], values[1]


def load_v2_firmware_limits(
    *,
    model_path: str | Path,
    config_path: str | Path,
    hand: str,
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """按 SDK 的拇指到小指顺序返回官方位置执行器限位。"""

    rules = load_joint_mapping(
        config_path,
        expected_hand=hand,
        expected_generation="v2",
    )
    tree = ET.parse(Path(model_path))
    root = tree.getroot()
    by_joint: dict[str, tuple[float, float]] = {}
    for actuator in root.findall("./actuator/position"):
        joint = actuator.get("joint")
        if not joint:
            raise ValueError("二代官方位置执行器缺少 joint")
        if joint in by_joint:
            raise ValueError(f"二代关节重复绑定位置执行器：{joint}")
        by_joint[joint] = _parse_ctrlrange(
            actuator.get("ctrlrange"),
            actuator=actuator.get("name", joint),
        )

    limits_by_dof: dict[str, tuple[float, float]] = {}
    for rule in rules:
        joint = full_joint_name(hand, rule.joint_name)
        try:
            limits_by_dof[rule.dof_name] = by_joint[joint]
        except KeyError as exc:
            raise ValueError(
                f"二代官方模型缺少 {joint} 的位置执行器"
            ) from exc

    lower = tuple(limits_by_dof[name][0] for name in FIRMWARE_DOF_ORDER)
    upper = tuple(limits_by_dof[name][1] for name in FIRMWARE_DOF_ORDER)
    return lower, upper
