#!/usr/bin/env python3
"""ROS-independent PICO controller-to-palm runtime transform."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


EPS = 1.0e-9


@dataclass(frozen=True)
class TcpTransform:
    translation_m: np.ndarray
    quaternion_xyzw: np.ndarray

    def __post_init__(self) -> None:
        translation = _vector(self.translation_m, "translation_m", 3)
        quaternion = _quaternion(self.quaternion_xyzw, "quaternion_xyzw")
        object.__setattr__(self, "translation_m", translation)
        object.__setattr__(self, "quaternion_xyzw", quaternion)


def load_tcp_transform(path: str | Path, side: str) -> TcpTransform:
    """Load and validate exactly one ``T_controller_palm`` artifact."""

    if side not in ("left", "right"):
        raise ValueError("side must be left or right")
    artifact_path = Path(path).expanduser()
    if not artifact_path.exists():
        raise ValueError(f"TCP artifact does not exist: {artifact_path}")
    try:
        import yaml

        with artifact_path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream) or {}
    except (OSError, ImportError) as error:
        raise ValueError(f"cannot read TCP artifact: {error}") from error
    if not isinstance(document, dict):
        raise ValueError("TCP artifact must be a mapping")
    if document.get("valid") is not True:
        raise ValueError("TCP artifact valid must be true")
    artifact_side = document.get("side")
    if artifact_side != side:
        raise ValueError(f"TCP artifact side mismatch: expected {side}, got {artifact_side!r}")
    if document.get("pose_semantics") != "controller_pose":
        raise ValueError("TCP artifact pose_semantics must be controller_pose")
    if document.get("transform_convention") != "T_controller_palm":
        raise ValueError("TCP artifact transform_convention must be T_controller_palm")
    if document.get("orientation_calibrated") is not True:
        raise ValueError("TCP artifact orientation_calibrated must be true")
    try:
        translation = _mapping_or_sequence(
            document["translation_m"], ("x", "y", "z")
        )
        quaternion = _mapping_or_sequence(
            document["quaternion_xyzw"], ("x", "y", "z", "w")
        )
    except (KeyError, TypeError) as error:
        raise ValueError(f"invalid TCP artifact fields: {error}") from error
    return TcpTransform(translation, quaternion)


def apply_tcp_transform(
    controller_position: np.ndarray,
    controller_quaternion_xyzw: np.ndarray,
    transform: TcpTransform,
) -> tuple[np.ndarray, np.ndarray]:
    """Compose ``T_G_C · T_C_H`` and return palm position/quaternion."""

    position = _vector(controller_position, "controller_position", 3)
    controller_quaternion = _quaternion(
        controller_quaternion_xyzw, "controller_quaternion_xyzw"
    )
    controller_rotation = quaternion_to_matrix(controller_quaternion)
    tcp_rotation = quaternion_to_matrix(transform.quaternion_xyzw)
    palm_position = position + controller_rotation @ transform.translation_m
    palm_quaternion = matrix_to_quaternion(controller_rotation @ tcp_rotation)
    return palm_position, palm_quaternion


def quaternion_to_matrix(quaternion_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = _quaternion(quaternion_xyzw, "quaternion")
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("rotation must be a finite 3x3 matrix")
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        quaternion = np.array(
            [
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = 2.0 * np.sqrt(max(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2], EPS))
            quaternion = np.array([0.25 * scale, (matrix[0, 1] + matrix[1, 0]) / scale, (matrix[0, 2] + matrix[2, 0]) / scale, (matrix[2, 1] - matrix[1, 2]) / scale])
        elif index == 1:
            scale = 2.0 * np.sqrt(max(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2], EPS))
            quaternion = np.array([(matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale, (matrix[1, 2] + matrix[2, 1]) / scale, (matrix[0, 2] - matrix[2, 0]) / scale])
        else:
            scale = 2.0 * np.sqrt(max(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1], EPS))
            quaternion = np.array([(matrix[0, 2] + matrix[2, 0]) / scale, (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale, (matrix[1, 0] - matrix[0, 1]) / scale])
    quaternion = _quaternion(quaternion, "composed quaternion")
    if quaternion[3] < 0.0:
        quaternion *= -1.0
    return quaternion


def _mapping_or_sequence(value, axes: tuple[str, ...]) -> np.ndarray:
    if isinstance(value, dict):
        value = [value[axis] for axis in axes]
    return np.asarray(value, dtype=float)


def _vector(value, name: str, size: int) -> np.ndarray:
    vector = np.asarray(value, dtype=float)
    if vector.shape != (size,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must be a finite {size}-vector")
    return vector.copy()


def _quaternion(value, name: str) -> np.ndarray:
    quaternion = _vector(value, name, 4)
    norm = float(np.linalg.norm(quaternion))
    if norm <= EPS:
        raise ValueError(f"{name} quaternion must be non-zero")
    return quaternion / norm
