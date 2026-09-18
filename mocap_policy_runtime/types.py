"""Transport-independent frames. Poses are metres + quaternion xyzw; joints radians."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class TargetFrame:
    """One sampled target, before robot-specific IK or hardware admission.

    ``space`` identifies the pose frame, never inferred from the file suffix:
    ``mocap_wrist`` uses the capture world, ``robot_tcp`` uses robot base.
    ``arm_joints`` bypasses IK; ``hand_keypoints`` remains wrist-local.
    """

    time_s: float
    index: int
    complete: bool = False
    space: str = "mocap_wrist"
    wrist_poses: dict[str, np.ndarray] = field(default_factory=dict)
    arm_joints: dict[str, np.ndarray] = field(default_factory=dict)
    hand_joints: dict[str, np.ndarray] = field(default_factory=dict)
    hand_keypoints: dict[str, np.ndarray] = field(default_factory=dict)
    object_poses: dict[str, np.ndarray] = field(default_factory=dict)
