"""灵巧手目标的后端无关表示。"""

from __future__ import annotations

from dataclasses import dataclass


DOF_ORDER: tuple[str, ...] = (
    "index1_abd",
    "index1_flex",
    "index2_flex",
    "index3_flex",
    "middle1_abd",
    "middle1_flex",
    "middle2_flex",
    "middle3_flex",
    "ring1_abd",
    "ring1_flex",
    "ring2_flex",
    "ring3_flex",
    "pinky1_abd",
    "pinky1_flex",
    "pinky2_flex",
    "pinky3_flex",
    "thumb1_flex",
    "thumb1_abd",
    "thumb2_flex",
    "thumb3_flex",
)


@dataclass(frozen=True)
class HandTarget:
    """一次完整的灵巧手关节目标及其输入帧元数据。"""

    hand: str
    values: tuple[float, ...]
    source: str
    sequence: int
    timestamp_ns: int
    dropped: int
