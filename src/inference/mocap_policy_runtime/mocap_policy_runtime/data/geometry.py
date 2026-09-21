"""Rigid poses use metres and quaternion xyzw throughout the runtime."""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def _finite_pose(pose: np.ndarray, label: str) -> np.ndarray:
    values = np.asarray(pose, dtype=np.float64)
    if values.shape != (7,) or not np.isfinite(values).all():
        raise ValueError(f"{label} must be seven finite xyzw pose values")
    norm = float(np.linalg.norm(values[3:7]))
    if norm < 1e-8:
        raise ValueError(f"{label} quaternion cannot be zero")
    result = values.copy()
    result[3:7] /= norm
    return result


def compose_pose(parent_from_middle: np.ndarray, middle_from_child: np.ndarray) -> np.ndarray:
    first = _finite_pose(parent_from_middle, "parent_from_middle")
    second = _finite_pose(middle_from_child, "middle_from_child")
    rotation = Rotation.from_quat(first[3:7])
    position = first[:3] + rotation.apply(second[:3])
    orientation = (rotation * Rotation.from_quat(second[3:7])).as_quat()
    return np.concatenate((position, orientation))


def invert_pose(parent_from_child: np.ndarray) -> np.ndarray:
    pose = _finite_pose(parent_from_child, "parent_from_child")
    inverse = Rotation.from_quat(pose[3:7]).inv()
    return np.concatenate((inverse.apply(-pose[:3]), inverse.as_quat()))


def interpolate_pose(first: np.ndarray, second: np.ndarray, fraction: float) -> np.ndarray:
    """Shortest-arc SLERP plus linear translation; inputs are normalized poses."""
    left, right = first[3:7], second[3:7]
    dot = float(np.dot(left, right))
    if dot < 0.0:
        right = -right
        dot = -dot
    dot = float(np.clip(dot, 0.0, 1.0))
    if dot > 0.9995:
        quaternion = (1.0 - fraction) * left + fraction * right
    else:
        theta = np.arccos(dot)
        quaternion = (np.sin((1.0 - fraction) * theta) * left + np.sin(fraction * theta) * right) / np.sin(theta)
    quaternion /= np.linalg.norm(quaternion)
    return np.concatenate(((1.0 - fraction) * first[:3] + fraction * second[:3], quaternion))
