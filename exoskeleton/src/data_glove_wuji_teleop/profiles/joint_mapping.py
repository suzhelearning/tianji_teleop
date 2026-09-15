"""语义 20-DOF 到具体 Wuji 模型关节的配置合同。"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

from ..domain.hand_target import DOF_ORDER


@dataclass(frozen=True)
class JointMappingRule:
    """一个语义 DOF 到模型关节的仿射映射。"""

    dof_name: str
    joint_name: str
    scale: float
    offset_rad: float


def load_joint_mapping(
    path: str | Path,
    *,
    expected_hand: str,
    expected_generation: str,
) -> tuple[JointMappingRule, ...]:
    """加载并完整校验一侧、一代际的 20-DOF 映射。"""

    config_path = Path(path)
    with config_path.open(encoding="utf-8") as file:
        data = json.load(file)
    if not isinstance(data, Mapping):
        raise ValueError(f"映射根节点必须是对象：{config_path}")
    if data.get("hand") != expected_hand:
        raise ValueError(
            f"映射 hand 必须是 {expected_hand}：{config_path}"
        )
    if data.get("generation", "v1") != expected_generation:
        raise ValueError(
            f"映射 generation 必须是 {expected_generation}：{config_path}"
        )
    raw_rules = data.get("mapping")
    if not isinstance(raw_rules, list):
        raise ValueError(f"mapping 必须是数组：{config_path}")

    rules: list[JointMappingRule] = []
    try:
        for item in raw_rules:
            semantic_dof = item.get("semantic_dof", item.get("mano_dof"))
            model_joint = item.get("model_joint", item.get("wuji_joint"))
            if semantic_dof is None or model_joint is None:
                raise KeyError("semantic_dof/model_joint")
            scale = float(item["scale"])
            offset_rad = math.radians(float(item["offset_deg"]))
            if not math.isfinite(scale) or not math.isfinite(offset_rad):
                raise ValueError("scale 和 offset_deg 必须是有限数值")
            rules.append(
                JointMappingRule(
                    dof_name=str(semantic_dof),
                    joint_name=str(model_joint),
                    scale=scale,
                    offset_rad=offset_rad,
                )
            )
    except (KeyError, TypeError) as exc:
        raise ValueError(f"mapping 条目字段不完整：{config_path}") from exc

    source_names = [rule.dof_name for rule in rules]
    target_names = [rule.joint_name for rule in rules]
    if len(rules) != len(DOF_ORDER):
        raise ValueError(
            f"映射必须包含 {len(DOF_ORDER)} 路，实际 {len(rules)}"
        )
    if set(source_names) != set(DOF_ORDER):
        raise ValueError("映射输入 DOF 与语义目标 DOF 不一致")
    if len(set(source_names)) != len(source_names):
        raise ValueError("映射输入 DOF 存在重复")
    if len(set(target_names)) != len(target_names):
        raise ValueError("Wuji 目标关节存在重复映射")
    return tuple(rules)
