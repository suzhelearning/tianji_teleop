"""Wuji Hand 2 Beta 1 官方模型命名合同。"""

from __future__ import annotations


MODEL_REVISION = "hand2_beta1"
JOINT_PREFIX = {
    "left": "l_",
    "right": "r_",
}


def full_joint_name(hand: str, base_name: str) -> str:
    try:
        prefix = JOINT_PREFIX[hand]
    except KeyError as exc:
        raise ValueError(f"hand 必须是 left 或 right，实际为 {hand}") from exc
    return f"{prefix}{base_name}"
