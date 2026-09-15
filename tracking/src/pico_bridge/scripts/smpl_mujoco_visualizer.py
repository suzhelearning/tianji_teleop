#!/usr/bin/env python3
"""Render the PICO SMPL PoseArray as a geometric MuJoCo skeleton."""

from __future__ import annotations

import argparse
from collections import deque
import math
from pathlib import Path
import threading
import time
from typing import Sequence

import numpy as np


JOINT_NAMES = (
    "Pelvis", "LEFT_HIP", "RIGHT_HIP", "SPINE1", "LEFT_KNEE", "RIGHT_KNEE",
    "SPINE2", "LEFT_ANKLE", "RIGHT_ANKLE", "SPINE3", "LEFT_FOOT", "RIGHT_FOOT",
    "NECK", "LEFT_COLLAR", "RIGHT_COLLAR", "HEAD", "LEFT_SHOULDER",
    "RIGHT_SHOULDER", "LEFT_ELBOW", "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST",
    "LEFT_HAND", "RIGHT_HAND",
)

_INDEX = {name: index for index, name in enumerate(JOINT_NAMES)}
BONE_EDGES = tuple(
    (_INDEX[parent], _INDEX[child])
    for parent, child in (
        ("Pelvis", "LEFT_HIP"), ("LEFT_HIP", "LEFT_KNEE"),
        ("LEFT_KNEE", "LEFT_ANKLE"), ("LEFT_ANKLE", "LEFT_FOOT"),
        ("Pelvis", "RIGHT_HIP"), ("RIGHT_HIP", "RIGHT_KNEE"),
        ("RIGHT_KNEE", "RIGHT_ANKLE"), ("RIGHT_ANKLE", "RIGHT_FOOT"),
        ("Pelvis", "SPINE1"), ("SPINE1", "SPINE2"), ("SPINE2", "SPINE3"),
        ("SPINE3", "NECK"), ("NECK", "HEAD"),
        ("SPINE3", "LEFT_COLLAR"), ("LEFT_COLLAR", "LEFT_SHOULDER"),
        ("LEFT_SHOULDER", "LEFT_ELBOW"), ("LEFT_ELBOW", "LEFT_WRIST"),
        ("LEFT_WRIST", "LEFT_HAND"),
        ("SPINE3", "RIGHT_COLLAR"), ("RIGHT_COLLAR", "RIGHT_SHOULDER"),
        ("RIGHT_SHOULDER", "RIGHT_ELBOW"), ("RIGHT_ELBOW", "RIGHT_WRIST"),
        ("RIGHT_WRIST", "RIGHT_HAND"),
    )
)

HAND_JOINT_NAMES = ("LEFT_ELBOW", "LEFT_WRIST", "LEFT_HAND", "RIGHT_ELBOW", "RIGHT_WRIST", "RIGHT_HAND")
HAND_JOINT_INDICES = frozenset(_INDEX[name] for name in HAND_JOINT_NAMES)
HAND_BONE_INDICES = frozenset(
    index for index, (parent, child) in enumerate(BONE_EDGES)
    if parent in HAND_JOINT_INDICES and child in HAND_JOINT_INDICES
)
HAND_VIEW_JOINT_NAMES = (
    "Pelvis", "SPINE1", "SPINE2", "SPINE3", "NECK", "HEAD",
    "LEFT_COLLAR", "RIGHT_COLLAR", "LEFT_SHOULDER", "RIGHT_SHOULDER",
    "LEFT_ELBOW", "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST",
    "LEFT_HAND", "RIGHT_HAND",
)
HAND_VIEW_JOINT_INDICES = frozenset(_INDEX[name] for name in HAND_VIEW_JOINT_NAMES)
HAND_VIEW_BONE_INDICES = frozenset(
    index for index, (parent, child) in enumerate(BONE_EDGES)
    if parent in HAND_VIEW_JOINT_INDICES and child in HAND_VIEW_JOINT_INDICES
)
ARM_AXIS_JOINT_NAMES = (
    "LEFT_SHOULDER", "LEFT_ELBOW", "LEFT_WRIST", "LEFT_HAND",
    "RIGHT_SHOULDER", "RIGHT_ELBOW", "RIGHT_WRIST", "RIGHT_HAND",
)
ARM_AXIS_JOINT_INDICES = tuple(_INDEX[name] for name in ARM_AXIS_JOINT_NAMES)

FOOT_HALF_EXTENTS = np.array([0.12, 0.055, 0.025], dtype=float)
RAW_FOOT_HALF_EXTENTS = np.array([0.115, 0.05, 0.022], dtype=float)
RAW_FLOOR_STALE_SEC = 1.0


def _rotation_matrix_from_wxyz(quaternion_wxyz: Sequence[float]) -> np.ndarray:
    quaternion = np.asarray(quaternion_wxyz, dtype=float)
    if (
        quaternion.shape != (4,)
        or not np.all(np.isfinite(quaternion))
        or np.linalg.norm(quaternion) <= 1e-9
    ):
        raise ValueError("orientation must be a finite non-zero wxyz quaternion")
    w, x, y, z = quaternion / np.linalg.norm(quaternion)
    return np.array([
        [1.0 - 2.0 * (y*y + z*z), 2.0 * (x*y - z*w), 2.0 * (x*z + y*w)],
        [2.0 * (x*y + z*w), 1.0 - 2.0 * (x*x + z*z), 2.0 * (y*z - x*w)],
        [2.0 * (x*z - y*w), 2.0 * (y*z + x*w), 1.0 - 2.0 * (x*x + y*y)],
    ])


def foot_sole_height(
    position: Sequence[float],
    quaternion_wxyz: Sequence[float],
    half_extents: Sequence[float] = FOOT_HALF_EXTENTS,
) -> float:
    """Return the lowest world-Z point of an oriented foot box."""
    center = np.asarray(position, dtype=float)
    extents = np.asarray(half_extents, dtype=float)
    if center.shape != (3,) or not np.all(np.isfinite(center)):
        raise ValueError("foot position must contain 3 finite values")
    if extents.shape != (3,) or not np.all(np.isfinite(extents)) or np.any(extents <= 0):
        raise ValueError("foot half extents must contain 3 positive finite values")
    rotation = _rotation_matrix_from_wxyz(quaternion_wxyz)
    vertical_radius = float(np.sum(np.abs(rotation[2, :]) * extents))
    return float(center[2] - vertical_radius)


class GroundPlaneEstimator:
    """Lock one floor height after a stable window of two-foot samples."""

    def __init__(self, window_size: int = 30, stability_tolerance: float = 0.02):
        if window_size <= 0:
            raise ValueError("window_size must be positive")
        if not math.isfinite(stability_tolerance) or stability_tolerance < 0:
            raise ValueError("stability_tolerance must be a non-negative finite number")
        self._window_size = window_size
        self._stability_tolerance = stability_tolerance
        self._samples: deque[tuple[float, float]] = deque(maxlen=window_size)
        self._height: float | None = None
        self._lock = threading.Lock()

    @property
    def height(self) -> float | None:
        with self._lock:
            return self._height

    def observe(self, left_height: float, right_height: float) -> float | None:
        values = np.asarray([left_height, right_height], dtype=float)
        if not np.all(np.isfinite(values)):
            return self.height
        with self._lock:
            if self._height is not None:
                return self._height
            self._samples.append((float(values[0]), float(values[1])))
            if len(self._samples) < self._window_size:
                return None
            flattened = np.asarray(self._samples, dtype=float).reshape(-1)
            if float(np.ptp(flattened)) <= self._stability_tolerance + 1e-12:
                self._height = float(np.median(flattened))
            return self._height

    def reset(self) -> None:
        with self._lock:
            self._samples.clear()
            self._height = None


def transform_position(position: Sequence[float], scale: float = 1.0, yaw: float = 0.0) -> np.ndarray:
    """Apply scale and a world-Z yaw rotation to a PICO position."""
    point = np.asarray(position, dtype=float)
    if point.shape != (3,):
        raise ValueError(f"position must contain 3 values, got shape {point.shape}")
    if not np.all(np.isfinite(point)):
        raise ValueError("position must contain only finite values")
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("scale must be a positive finite number")
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))
    return rotation @ (point * scale)


def _finite_offset(offset: Sequence[float], name: str = "offset") -> np.ndarray:
    value = np.asarray(offset, dtype=float)
    if value.shape != (3,) or not np.all(np.isfinite(value)):
        raise ValueError(f"{name} must contain three finite values")
    return value.copy()


def transform_orientation(quaternion_xyzw: Sequence[float], yaw: float = 0.0) -> np.ndarray:
    """Convert ROS xyzw to MuJoCo wxyz and apply the viewer world yaw."""
    q = np.asarray(quaternion_xyzw, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) <= 1e-9:
        raise ValueError("orientation must be a finite non-zero xyzw quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    c, s = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return np.array((c*w - s*z, c*x - s*y, c*y + s*x, c*z + s*w))


def _quaternion_from_rotation_wxyz(rotation: np.ndarray) -> np.ndarray:
    """Convert a proper rotation matrix to a normalized MuJoCo wxyz quaternion."""
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("rotation must be a finite 3x3 matrix")
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * math.sqrt(trace + 1.0)
        quaternion = np.array(
            [
                0.25 * scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            ]
        )
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        next_index = (index + 1) % 3
        last_index = (index + 2) % 3
        scale = 2.0 * math.sqrt(
            max(
                1.0
                + matrix[index, index]
                - diagonal[next_index]
                - diagonal[last_index],
                1e-12,
            )
        )
        quaternion = np.zeros(4)
        quaternion[index + 1] = 0.25 * scale
        quaternion[0] = (
            matrix[last_index, next_index] - matrix[next_index, last_index]
        ) / scale
        quaternion[next_index + 1] = (
            matrix[next_index, index] + matrix[index, next_index]
        ) / scale
        quaternion[last_index + 1] = (
            matrix[last_index, index] + matrix[index, last_index]
        ) / scale
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[0] < 0.0:
        quaternion *= -1.0
    return quaternion


def _artifact_vector(value, name: str) -> np.ndarray:
    if isinstance(value, dict):
        value = [value[axis] for axis in ("x", "y", "z")]
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain three finite values")
    return vector


def _artifact_quaternion_wxyz(value) -> np.ndarray:
    if isinstance(value, dict):
        value = [value[axis] for axis in ("x", "y", "z", "w")]
    quaternion_xyzw = np.asarray(value, dtype=float)
    if (
        quaternion_xyzw.shape != (4,)
        or not np.all(np.isfinite(quaternion_xyzw))
        or np.linalg.norm(quaternion_xyzw) <= 1e-9
    ):
        raise ValueError("quaternion_xyzw must contain four finite values")
    return transform_orientation(quaternion_xyzw)


def load_visualization_tcp_artifact(
    path: str | Path, side: str
) -> tuple[np.ndarray, np.ndarray] | None:
    """Load a controller-to-palm TCP for visualization-only fallback."""
    if side not in ("left", "right"):
        raise ValueError("side must be left or right")
    artifact_path = Path(str(path)).expanduser()
    if not str(path).strip() or not artifact_path.exists():
        return None
    try:
        import yaml

        with artifact_path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream) or {}
        if document.get("valid") is not True:
            return None
        document_side = document.get("side")
        if document_side is not None and document_side != side:
            return None
        if document.get("pose_semantics") not in (None, "controller_pose"):
            return None
        return (
            _artifact_vector(document["translation_m"], "translation_m"),
            _artifact_quaternion_wxyz(document["quaternion_xyzw"]),
        )
    except (OSError, ImportError, KeyError, TypeError, ValueError):
        return None


def load_visualization_wrist_pivot_artifact(
    path: str | Path, side: str
) -> float | None:
    """Load palm-local +X wrist-to-palm distance for visualization."""
    if side not in ("left", "right"):
        raise ValueError("side must be left or right")
    artifact_path = Path(str(path)).expanduser()
    if not str(path).strip() or not artifact_path.exists():
        return None
    try:
        import yaml

        with artifact_path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream) or {}
        if document.get("valid") is not True:
            return None
        document_side = document.get("side")
        if document_side is not None and document_side != side:
            return None
        if document.get("transform_convention") != "wrist_to_palm":
            return None
        raw_distance = document.get("wrist_to_palm_distance_m")
        if raw_distance is not None:
            distance = float(raw_distance)
        else:
            legacy_vector = _artifact_vector(
                document["wrist_to_palm_m"], "wrist_to_palm_m"
            )
            distance = float(np.linalg.norm(legacy_vector))
        if not math.isfinite(distance) or distance <= 1.0e-9:
            return None
        return distance
    except (OSError, ImportError, KeyError, TypeError, ValueError):
        return None


def derive_tcp_palm_pose(
    controller_position: np.ndarray,
    controller_orientation: np.ndarray,
    tcp_translation: np.ndarray,
    tcp_orientation: np.ndarray,
    scale: float = 1.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Apply ``T_controller_palm`` to a viewer-frame controller pose."""
    if not math.isfinite(scale) or scale <= 0.0:
        raise ValueError("scale must be a positive finite number")
    controller_position = _artifact_vector(controller_position, "controller_position")
    tcp_translation = _artifact_vector(tcp_translation, "tcp_translation")
    controller_rotation = _rotation_matrix_from_wxyz(controller_orientation)
    tcp_rotation = _rotation_matrix_from_wxyz(tcp_orientation)
    palm_position = controller_position + controller_rotation @ (scale * tcp_translation)
    palm_rotation = controller_rotation @ tcp_rotation
    return palm_position, _quaternion_from_rotation_wxyz(palm_rotation)


def derive_wrist_pose(
    palm_position: np.ndarray,
    palm_orientation: np.ndarray,
    wrist_to_palm: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Recover wrist pose with the palm on wrist/palm-local positive X."""
    palm_position = _artifact_vector(palm_position, "palm_position")
    wrist_to_palm = float(wrist_to_palm)
    if not math.isfinite(wrist_to_palm) or wrist_to_palm <= 1.0e-9:
        raise ValueError("wrist_to_palm must be a positive finite distance")
    palm_rotation = _rotation_matrix_from_wxyz(palm_orientation)
    local_offset = np.array([wrist_to_palm, 0.0, 0.0], dtype=float)
    return palm_position - palm_rotation @ local_offset, np.asarray(
        palm_orientation, dtype=float
    ).copy()


def resolve_visualization_wrist_pose(
    explicit_position: np.ndarray | None,
    explicit_orientation: np.ndarray | None,
    explicit_received_at: float,
    now: float,
    timeout: float,
    palm_position: np.ndarray | None,
    palm_orientation: np.ndarray | None,
    palm_received_at: float,
    wrist_to_palm: float | None,
    primary_position: np.ndarray | None,
    primary_orientation: np.ndarray | None,
    primary_received_at: float,
) -> tuple[np.ndarray | None, np.ndarray | None, float, str]:
    """Select one wrist source without allowing a fallback to overwrite pivot data.

    When a wrist-pivot artifact is configured, it is authoritative: the wrist
    is reconstructed from the same TCP palm sample and therefore has exactly
    the palm orientation.  This also prevents an older or differently
    calibrated wrist topic from disagreeing with the TCP result.  Without a
    pivot artifact the priority is a live wrist topic, then the wrist in the
    primary PoseArray.  In particular, once the pivot branch is selected it
    must not be replaced merely because another source is available.
    """
    if (
        wrist_to_palm is not None
        and palm_position is not None
        and palm_orientation is not None
    ):
        wrist_position, wrist_orientation = derive_wrist_pose(
            palm_position, palm_orientation, wrist_to_palm
        )
        return wrist_position, wrist_orientation, palm_received_at, "wrist_pivot"
    explicit_live = (
        explicit_position is not None
        and explicit_orientation is not None
        and now - explicit_received_at <= timeout
    )
    if explicit_live:
        return explicit_position, explicit_orientation, explicit_received_at, "topic"
    if primary_position is not None:
        orientation = (
            primary_orientation
            if primary_orientation is not None
            else np.array([1.0, 0.0, 0.0, 0.0])
        )
        return primary_position, orientation, primary_received_at, "primary_posearray"
    if explicit_position is not None and explicit_orientation is not None:
        return explicit_position, explicit_orientation, explicit_received_at, "stale_topic"
    return None, None, 0.0, "missing"


def format_hand_position_overlay(positions: Sequence[np.ndarray]) -> str:
    """Format hand endpoint positions in the displayed world frame."""
    lines = []
    for name in ("LEFT_HAND", "RIGHT_HAND"):
        position = np.asarray(positions[_INDEX[name]], dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            lines.append(f"{name}: unavailable")
        else:
            lines.append(
                f"{name}: x={position[0]:.3f} m, "
                f"y={position[1]:.3f} m, z={position[2]:.3f} m"
            )
    return "\n".join(lines)


def format_palm_position_overlay(position: np.ndarray | None) -> str:
    if position is None or np.asarray(position).shape != (3,) or not np.all(np.isfinite(position)):
        return "Calibrated palm TCP: unavailable"
    position = np.asarray(position, dtype=float)
    return (
        "Calibrated left palm: "
        f"x={position[0]:.3f} m, y={position[1]:.3f} m, z={position[2]:.3f} m"
    )


def _quat_from_z_axis(direction: np.ndarray) -> np.ndarray:
    """Return a MuJoCo wxyz quaternion rotating +Z onto direction."""
    unit = direction / np.linalg.norm(direction)
    z_axis = np.array([0.0, 0.0, 1.0])
    dot = float(np.dot(z_axis, unit))
    if dot > 1.0 - 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0])
    if dot < -1.0 + 1e-8:
        return np.array([0.0, 1.0, 0.0, 0.0])
    axis = np.cross(z_axis, unit)
    quaternion = np.array([1.0 + dot, axis[0], axis[1], axis[2]])
    return quaternion / np.linalg.norm(quaternion)


def capsule_pose(parent: Sequence[float], child: Sequence[float]) -> tuple[np.ndarray, np.ndarray, float]:
    """Return capsule center, MuJoCo wxyz quaternion, and length."""
    start = np.asarray(parent, dtype=float)
    end = np.asarray(child, dtype=float)
    if start.shape != (3,) or end.shape != (3,):
        raise ValueError("parent and child must each contain 3 values")
    delta = end - start
    length = float(np.linalg.norm(delta))
    center = (start + end) / 2.0
    if length <= 1e-8:
        return center, np.array([1.0, 0.0, 0.0, 0.0]), 0.0
    return center, _quat_from_z_axis(delta), length


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/pico/smpl")
    parser.add_argument("--show-raw", action="store_true")
    parser.add_argument("--raw-topic", default="/pico/smpl")
    parser.add_argument("--palm-topic", default="/pico/palm_left")
    parser.add_argument("--right-palm-topic", default="/pico/palm_right")
    parser.add_argument("--wrist-topic", default="/pico/wrist_left")
    parser.add_argument("--right-wrist-topic", default="/pico/wrist_right")
    parser.add_argument(
        "--left-tcp-artifact",
        default="~/.config/pico_tracker/pico_left_palm_tcp.yaml",
        help="optional TCP artifact used when /pico/palm_left is not being published",
    )
    parser.add_argument(
        "--right-tcp-artifact",
        default="~/.config/pico_tracker/pico_right_palm_tcp.yaml",
        help="optional TCP artifact used when /pico/palm_right is not being published",
    )
    parser.add_argument(
        "--left-wrist-pivot-artifact",
        default="~/.config/pico_tracker/pico_left_wrist_pivot.yaml",
        help="optional wrist-to-palm artifact used when /pico/wrist_left is absent",
    )
    parser.add_argument(
        "--right-wrist-pivot-artifact",
        default="~/.config/pico_tracker/pico_right_wrist_pivot.yaml",
        help="optional wrist-to-palm artifact used when /pico/wrist_right is absent",
    )
    parser.add_argument(
        "--raw-offset",
        nargs=3,
        type=float,
        default=(0.45, 0.0, 0.0),
        metavar=("X", "Y", "Z"),
        help=(
            "display-only offset for the raw overlay in viewer coordinates; "
            "use 0 0 0 for a true geometric overlay"
        ),
    )
    parser.add_argument(
        "--hands-only",
        action="store_true",
        help="show upper body from pelvis through both hands",
    )
    parser.add_argument(
        "--show-arm-axes",
        action="store_true",
        help="show local XYZ axes at shoulder, elbow, wrist, and hand joints on the primary skeleton",
    )
    parser.add_argument(
        "--show-raw-arm-axes",
        action="store_true",
        help="also show local XYZ axes on the raw overlay skeleton",
    )
    parser.add_argument(
        "--show-raw-pelvis-axes",
        action="store_true",
        help="also show the raw overlay pelvis reference axes",
    )
    parser.add_argument(
        "--show-controllers",
        action="store_true",
        help="show raw PICO HMD and left/right controller poses with local XYZ axes",
    )
    parser.add_argument("--head-topic", default="/pico/pose/head")
    parser.add_argument("--left-controller-topic", default="/pico/pose/left_hand")
    parser.add_argument("--right-controller-topic", default="/pico/pose/right_hand")
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--rate", type=float, default=60.0)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    return parser


def _make_xml(
    show_raw: bool = False,
    show_arm_axes: bool = False,
    show_controllers: bool = False,
    show_raw_arm_axes: bool = False,
    show_raw_pelvis_axes: bool = False,
) -> str:
    bodies = [
        '    <body name="ground" pos="0 0 0">'
        '<geom name="ground_geom" type="plane" size="10 10 .01" '
        'material="groundplane" rgba="1 1 1 0"/>'
        '<geom name="world_axis_x" class="world_axis" fromto="0 0 .004 .30 0 .004" '
        'rgba="1 0 0 0"/>'
        '<geom name="world_axis_y" class="world_axis" fromto="0 0 .004 0 .30 .004" '
        'rgba="0 1 0 0"/>'
        '<geom name="world_axis_z" class="world_axis" fromto="0 0 .004 0 0 .30" '
        'rgba="0.1 0.35 1 0"/>'
        '</body>',
        '    <body name="pelvis_axes" pos="0 0 0">'
        '<joint name="pelvis_axes_free" type="free"/>'
        '<geom name="pelvis_axis_x" class="pelvis_axis" fromto="0 0 0 .22 0 0" '
        'rgba="1 0 0 0"/>'
        '<geom name="pelvis_axis_y" class="pelvis_axis" fromto="0 0 0 0 .22 0" '
        'rgba="0 1 0 0"/>'
        '<geom name="pelvis_axis_z" class="pelvis_axis" fromto="0 0 0 0 0 .22" '
        'rgba="0.1 0.35 1 0"/>'
        '</body>',
        '    <body name="palm_left" pos="0 0 0">'
        '<joint name="palm_left_free" type="free"/>'
        '<geom name="palm_left_geom" type="sphere" size=".055" '
        'rgba="0.98 0.65 0.08 0"/> '
        '</body>',
        '    <body name="palm_left_axes" pos="0 0 0">'
        '<joint name="palm_left_axes_free" type="free"/>'
        '<geom name="palm_left_axis_x" class="arm_axis" '
        'fromto="0 0 0 .18 0 0" rgba="1 0 0 0"/>'
        '<geom name="palm_left_axis_y" class="arm_axis" '
        'fromto="0 0 0 0 .18 0" rgba="0 1 0 0"/>'
        '<geom name="palm_left_axis_z" class="arm_axis" '
        'fromto="0 0 0 0 0 .18" rgba="0.1 0.35 1 0"/>'
        '</body>',
        '    <body name="palm_right" pos="0 0 0">'
        '<joint name="palm_right_free" type="free"/>'
        '<geom name="palm_right_geom" type="sphere" size=".055" '
        'rgba="0.98 0.65 0.08 0"/> '
        '</body>',
        '    <body name="palm_right_axes" pos="0 0 0">'
        '<joint name="palm_right_axes_free" type="free"/>'
        '<geom name="palm_right_axis_x" class="arm_axis" '
        'fromto="0 0 0 .18 0 0" rgba="1 0 0 0"/>'
        '<geom name="palm_right_axis_y" class="arm_axis" '
        'fromto="0 0 0 0 .18 0" rgba="0 1 0 0"/>'
        '<geom name="palm_right_axis_z" class="arm_axis" '
        'fromto="0 0 0 0 0 .18" rgba="0.1 0.35 1 0"/>'
        '</body>',
    ]
    for side in ("left", "right"):
        bodies.extend([
            f'    <body name="wrist_{side}" pos="0 0 0">'
            f'<joint name="wrist_{side}_free" type="free"/>'
            f'<geom name="wrist_{side}_geom" type="sphere" size=".038" '
            'rgba="0.72 0.25 0.85 0"/> '
            '</body>',
            f'    <body name="wrist_{side}_axes" pos="0 0 0">'
            f'<joint name="wrist_{side}_axes_free" type="free"/>'
            f'<geom name="wrist_{side}_axis_x" class="arm_axis" '
            'fromto="0 0 0 .16 0 0" rgba="1 0 0 0"/>'
            f'<geom name="wrist_{side}_axis_y" class="arm_axis" '
            'fromto="0 0 0 0 .16 0" rgba="0 1 0 0"/>'
            f'<geom name="wrist_{side}_axis_z" class="arm_axis" '
            'fromto="0 0 0 0 0 .16" rgba="0.1 0.35 1 0"/>'
            '</body>',
            f'    <body name="wrist_{side}_palm_bone" pos="0 0 0">'
            f'<joint name="wrist_{side}_palm_bone_free" type="free"/>'
            f'<geom name="wrist_{side}_palm_bone_geom" type="capsule" '
            'fromto="0 0 -0.5 0 0 0.5" size=".014 .5" '
            'rgba="0.55 0.32 0.68 0"/> '
            '</body>',
        ])
    if show_controllers:
        for name, color, size in (
            ("head", "1 0 0", ".045"),
            ("left_hand", "1 0 0", ".035"),
            ("right_hand", "1 0 0", ".035"),
        ):
            bodies.extend([
                f'    <body name="controller_{name}" pos="0 0 0">'
                f'<joint name="controller_{name}_free" type="free"/>'
                f'<geom name="controller_{name}_geom" type="sphere" size="{size}" '
                f'rgba="{color} 0"/> '
                '</body>',
                f'    <body name="controller_{name}_axes" pos="0 0 0">'
                f'<joint name="controller_{name}_axes_free" type="free"/>'
                f'<geom name="controller_{name}_axis_x" class="controller_axis" '
                'fromto="0 0 0 .16 0 0" rgba="1 0 0 0"/>'
                f'<geom name="controller_{name}_axis_y" class="controller_axis" '
                'fromto="0 0 0 0 .16 0" rgba="0 1 0 0"/>'
                f'<geom name="controller_{name}_axis_z" class="controller_axis" '
                'fromto="0 0 0 0 0 .16" rgba="0.1 0.35 1 0"/>'
                '</body>',
            ])
    for index in range(len(JOINT_NAMES)):
        bodies.append(
            f'    <body name="joint_{index}" pos="0 0 0">'
            f'<joint name="joint_free_{index}" type="free"/>'
            f'<geom name="joint_geom_{index}" type="sphere" size="0.035" rgba="0.1 0.45 0.95 1"/></body>'
        )
    for index, _ in enumerate(BONE_EDGES):
        bodies.append(
            f'    <body name="bone_{index}" pos="0 0 0">'
            f'<joint name="bone_free_{index}" type="free"/>'
            f'<geom name="bone_geom_{index}" type="capsule" fromto="0 0 -0.5 0 0 0.5" '
            f'size="0.018 0.5" rgba="0.65 0.72 0.82 1"/></body>'
        )
    for side in ("left", "right"):
        bodies.append(
            f'    <body name="foot_{side}" pos="0 0 0">'
            f'<joint name="foot_free_{side}" type="free"/>'
            f'<geom name="foot_geom_{side}" type="box" size="0.12 0.055 0.025" '
            f'rgba="0.95 0.45 0.1 0.9"/></body>'
        )
    if show_arm_axes:
        for index in ARM_AXIS_JOINT_INDICES:
            bodies.append(
                f'    <body name="arm_axes_{index}" pos="0 0 0">'
                f'<joint name="arm_axes_free_{index}" type="free"/>'
                f'<geom name="arm_axis_{index}_x" class="arm_axis" '
                'fromto="0 0 0 .18 0 0" rgba="1 0 0 0"/>'
                f'<geom name="arm_axis_{index}_y" class="arm_axis" '
                'fromto="0 0 0 0 .18 0" rgba="0 1 0 0"/>'
                f'<geom name="arm_axis_{index}_z" class="arm_axis" '
                'fromto="0 0 0 0 0 .18" rgba="0.1 0.35 1 0"/>'
                '</body>'
            )
    if show_raw:
        if show_raw_pelvis_axes:
            bodies.append(
                '    <body name="raw_pelvis_axes" pos="0 0 0">'
                '<joint name="raw_pelvis_axes_free" type="free"/>'
                '<geom name="raw_pelvis_axis_x" class="raw_pelvis_axis" '
                'fromto="0 0 0 .18 0 0" rgba="1 0 0 0"/>'
                '<geom name="raw_pelvis_axis_y" class="raw_pelvis_axis" '
                'fromto="0 0 0 0 .18 0" rgba="0 1 0 0"/>'
                '<geom name="raw_pelvis_axis_z" class="raw_pelvis_axis" '
                'fromto="0 0 0 0 0 .18" rgba="0.1 0.35 1 0"/>'
                '</body>'
            )
        for index in range(len(JOINT_NAMES)):
            bodies.append(
                f'    <body name="raw_joint_{index}" pos="0 0 0">'
                f'<joint name="raw_joint_free_{index}" type="free"/>'
                f'<geom name="raw_joint_geom_{index}" type="sphere" size="0.024" '
                f'rgba="1 0.05 0.8 0.82"/></body>'
            )
        for index, _ in enumerate(BONE_EDGES):
            bodies.append(
                f'    <body name="raw_bone_{index}" pos="0 0 0">'
                f'<joint name="raw_bone_free_{index}" type="free"/>'
                f'<geom name="raw_bone_geom_{index}" type="capsule" '
                f'fromto="0 0 -0.5 0 0 0.5" size="0.012 0.5" '
                f'rgba="1 0.05 0.8 0.82"/></body>'
            )
        for side in ("left", "right"):
            bodies.append(
                f'    <body name="raw_foot_{side}" pos="0 0 0">'
                f'<joint name="raw_foot_free_{side}" type="free"/>'
                f'<geom name="raw_foot_geom_{side}" type="box" size="0.115 0.05 0.022" '
                f'rgba="1 0.15 0.55 0.86"/></body>'
            )
        if show_raw_arm_axes:
            for index in ARM_AXIS_JOINT_INDICES:
                bodies.append(
                    f'    <body name="raw_arm_axes_{index}" pos="0 0 0">'
                    f'<joint name="raw_arm_axes_free_{index}" type="free"/>'
                    f'<geom name="raw_arm_axis_{index}_x" class="raw_arm_axis" '
                    'fromto="0 0 0 .15 0 0" rgba="1 0 0 0"/>'
                    f'<geom name="raw_arm_axis_{index}_y" class="raw_arm_axis" '
                    'fromto="0 0 0 0 .15 0" rgba="0 1 0 0"/>'
                    f'<geom name="raw_arm_axis_{index}_z" class="raw_arm_axis" '
                    'fromto="0 0 0 0 0 .15" rgba="0.1 0.35 1 0"/>'
                    '</body>'
                )
    return (
        '<mujoco model="pico_smpl">'
        '<compiler angle="radian" coordinate="local"/>'
        '<option timestep="0.01" gravity="0 0 -9.81"/>'
        '<statistic center="0 0 1.0" extent="0.8"/>'
        '<visual>'
        '<headlight diffuse="0.6 0.6 0.6" ambient="0.1 0.1 0.1" '
        'specular="0.9 0.9 0.9"/>'
        '<rgba haze="0.15 0.25 0.35 1"/>'
        '<global azimuth="-140" elevation="-20"/>'
        '</visual>'
        '<asset>'
        '<texture type="skybox" builtin="gradient" rgb1="1 1 1" rgb2=".6 .7 .8" '
        'width="800" height="800"/>'
        '<texture type="2d" name="groundplane" builtin="checker" mark="edge" '
        'rgb1=".92 .92 .92" rgb2=".25 .25 .25" markrgb="0 0 0" '
        'width="300" height="300"/>'
        '<material name="groundplane" texture="groundplane" texuniform="true" '
        'texrepeat="5 5" reflectance="0"/>'
        '</asset>'
        '<default>'
        '<default class="world_axis"><geom type="capsule" size=".010"/></default>'
        '<default class="pelvis_axis"><geom type="capsule" size=".009"/></default>'
        '<default class="raw_pelvis_axis"><geom type="capsule" size=".006"/></default>'
        '<default class="arm_axis"><geom type="capsule" size=".011"/></default>'
        '<default class="controller_axis"><geom type="capsule" size=".012"/></default>'
        '<default class="raw_arm_axis"><geom type="capsule" size=".008"/></default>'
        '</default>'
        '<worldbody>'
        '<light name="sun" directional="true" diffuse=".5 .5 .5" '
        'pos="-3 -3 5" dir="3 3 -5" castshadow="true"/>'
        + ''.join(bodies)
        + '</worldbody></mujoco>'
    )


class PoseFrameCache:
    """Thread-safe storage for one independently timed skeleton stream."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._positions: list[np.ndarray] | None = None
        self._orientations: list[np.ndarray] | None = None
        self._received_at = 0.0

    def update(
        self,
        positions: list[np.ndarray],
        orientations: list[np.ndarray],
        received_at: float,
    ) -> None:
        with self._lock:
            self._positions = [position.copy() for position in positions]
            self._orientations = [orientation.copy() for orientation in orientations]
            self._received_at = received_at

    def snapshot(
        self,
    ) -> tuple[list[np.ndarray] | None, list[np.ndarray] | None, float]:
        with self._lock:
            positions = (
                None if self._positions is None
                else [position.copy() for position in self._positions]
            )
            orientations = (
                None if self._orientations is None
                else [orientation.copy() for orientation in self._orientations]
            )
            return positions, orientations, self._received_at


class PalmPoseCache:
    """Thread-safe storage for the optional calibrated palm point."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._position: np.ndarray | None = None
        self._orientation: np.ndarray | None = None
        self._received_at = 0.0

    def update(self, position: np.ndarray, orientation: np.ndarray, received_at: float) -> None:
        with self._lock:
            self._position = position.copy()
            self._orientation = orientation.copy()
            self._received_at = received_at

    def snapshot(self) -> tuple[np.ndarray | None, np.ndarray | None, float]:
        with self._lock:
            return (
                None if self._position is None else self._position.copy(),
                None if self._orientation is None else self._orientation.copy(),
                self._received_at,
            )


class SmplMujocoVisualizer:
    """ROS subscriber and MuJoCo renderer for the PICO PoseArray."""

    def __init__(
        self,
        topic: str,
        scale: float,
        rate: float,
        timeout: float,
        yaw: float,
        raw_topic: str = "/pico/smpl",
        palm_topic: str = "/pico/palm_left",
        right_palm_topic: str = "/pico/palm_right",
        wrist_topic: str = "/pico/wrist_left",
        right_wrist_topic: str = "/pico/wrist_right",
        left_tcp_artifact: str = "~/.config/pico_tracker/pico_left_palm_tcp.yaml",
        right_tcp_artifact: str = "~/.config/pico_tracker/pico_right_palm_tcp.yaml",
        left_wrist_pivot_artifact: str = "~/.config/pico_tracker/pico_left_wrist_pivot.yaml",
        right_wrist_pivot_artifact: str = "~/.config/pico_tracker/pico_right_wrist_pivot.yaml",
        raw_offset: Sequence[float] = (0.45, 0.0, 0.0),
        show_raw: bool = False,
        hands_only: bool = False,
        show_arm_axes: bool = False,
        show_raw_arm_axes: bool = False,
        show_raw_pelvis_axes: bool = False,
        head_topic: str = "/pico/pose/head",
        left_controller_topic: str = "/pico/pose/left_hand",
        right_controller_topic: str = "/pico/pose/right_hand",
        show_controllers: bool = False,
    ):
        try:
            import rclpy
            from geometry_msgs.msg import PoseArray, PoseStamped
            from std_msgs.msg import Float32
            from rclpy.node import Node
            from rclpy.executors import ExternalShutdownException
            from rclpy._rclpy_pybind11 import RCLError
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            import mujoco
            import mujoco.viewer
        except ImportError as exc:
            raise RuntimeError(
                "SMPL MuJoCo visualizer requires rclpy, geometry_msgs, numpy, and mujoco; "
                "install the project environment before running it"
            ) from exc
        self.rclpy = rclpy
        self.external_shutdown_exception = ExternalShutdownException
        self.rcl_error = RCLError
        self.mujoco = mujoco
        self.node = Node("smpl_mujoco_visualizer")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scale = scale
        self.rate = rate
        self.timeout = timeout
        self.yaw = yaw
        self.topic = topic
        self.raw_topic = raw_topic
        self.palm_topic = palm_topic
        self.right_palm_topic = right_palm_topic
        self.wrist_topic = wrist_topic
        self.right_wrist_topic = right_wrist_topic
        self._tcp_artifacts = {
            "left": load_visualization_tcp_artifact(left_tcp_artifact, "left"),
            "right": load_visualization_tcp_artifact(right_tcp_artifact, "right"),
        }
        self._wrist_pivot_artifacts = {
            "left": load_visualization_wrist_pivot_artifact(
                left_wrist_pivot_artifact, "left"
            ),
            "right": load_visualization_wrist_pivot_artifact(
                right_wrist_pivot_artifact, "right"
            ),
        }
        self.raw_offset = _finite_offset(raw_offset, "raw_offset")
        self.show_raw = show_raw
        self.hands_only = hands_only
        # Hands-only inspection is specifically intended for upper-limb
        # measurement, so expose the local arm triads automatically there.
        self.show_arm_axes = bool(show_arm_axes or hands_only)
        self.show_raw_arm_axes = bool(show_raw_arm_axes)
        self.show_raw_pelvis_axes = bool(show_raw_pelvis_axes)
        self.show_controllers = bool(show_controllers)
        self.head_topic = head_topic
        self.left_controller_topic = left_controller_topic
        self.right_controller_topic = right_controller_topic
        self._primary_cache = PoseFrameCache()
        self._raw_cache = PoseFrameCache()
        self._palm_cache = PalmPoseCache()
        self._right_palm_cache = PalmPoseCache()
        self._derived_palm_cache = {
            "left": PalmPoseCache(),
            "right": PalmPoseCache(),
        }
        self._wrist_cache = PalmPoseCache()
        self._right_wrist_cache = PalmPoseCache()
        self._controller_caches = {
            "head": PalmPoseCache(),
            "left_hand": PalmPoseCache(),
            "right_hand": PalmPoseCache(),
        }
        self._ground_estimator = GroundPlaneEstimator()
        self._floor_source: str | None = None
        self._ground_reset_at = time.monotonic()
        self._last_warning = {"primary": 0.0, "raw": 0.0}
        self._last_stale_warning = {"primary": 0.0, "raw": 0.0}
        self.subscription = self.node.create_subscription(
            PoseArray, topic, self._pose_callback, qos
        )
        self.raw_subscription = None
        if show_raw:
            self.raw_subscription = self.node.create_subscription(
                PoseArray, raw_topic, self._raw_pose_callback, qos
            )
        self.palm_subscription = self.node.create_subscription(
            PoseStamped, palm_topic, lambda message: self._palm_pose_callback(message, "left"), qos
        )
        self.right_palm_subscription = self.node.create_subscription(
            PoseStamped, right_palm_topic,
            lambda message: self._palm_pose_callback(message, "right"), qos
        )
        self.wrist_subscription = self.node.create_subscription(
            PoseStamped,
            wrist_topic,
            lambda message: self._wrist_pose_callback(message, "left"),
            qos,
        )
        self.right_wrist_subscription = self.node.create_subscription(
            PoseStamped,
            right_wrist_topic,
            lambda message: self._wrist_pose_callback(message, "right"),
            qos,
        )
        self.controller_subscriptions = []
        if self.show_controllers or any(
            artifact is not None for artifact in self._tcp_artifacts.values()
        ):
            for name, controller_topic in (
                ("head", self.head_topic),
                ("left_hand", self.left_controller_topic),
                ("right_hand", self.right_controller_topic),
            ):
                self.controller_subscriptions.append(
                    self.node.create_subscription(
                        PoseStamped,
                        controller_topic,
                        lambda message, name=name: self._controller_pose_callback(
                            message, name
                        ),
                        qos,
                    )
                )
        self.world_reset_subscription = self.node.create_subscription(
            Float32, "/pico/world_reset", self._world_reset_callback, 10
        )

        self.model = mujoco.MjModel.from_xml_string(
            _make_xml(
                show_raw,
                self.show_arm_axes,
                self.show_controllers,
                self.show_raw_arm_axes,
                self.show_raw_pelvis_axes,
            )
        )
        self.data = mujoco.MjData(self.model)
        self.ground_body_id = self.model.body("ground").id
        self.ground_geom_id = self.model.geom("ground_geom").id
        self.world_axis_geom_ids = [
            self.model.geom(f"world_axis_{axis}").id for axis in ("x", "y", "z")
        ]
        self._world_axis_colors = (
            np.array([1.0, 0.0, 0.0, 1.0]),
            np.array([0.0, 1.0, 0.0, 1.0]),
            np.array([0.1, 0.35, 1.0, 1.0]),
        )
        self.pelvis_axes_qposadr = int(
            self.model.jnt_qposadr[self.model.joint("pelvis_axes_free").id]
        )
        self.pelvis_axis_geom_ids = [
            self.model.geom(f"pelvis_axis_{axis}").id for axis in ("x", "y", "z")
        ]
        self._pelvis_axis_colors = self._world_axis_colors
        self.palm_qposadr = int(
            self.model.jnt_qposadr[self.model.joint("palm_left_free").id]
        )
        self.palm_axes_qposadr = int(
            self.model.jnt_qposadr[self.model.joint("palm_left_axes_free").id]
        )
        self.palm_geom_id = self.model.geom("palm_left_geom").id
        self.palm_axis_geom_ids = [
            self.model.geom(f"palm_left_axis_{axis}").id for axis in ("x", "y", "z")
        ]
        self.right_palm_qposadr = int(
            self.model.jnt_qposadr[self.model.joint("palm_right_free").id]
        )
        self.right_palm_axes_qposadr = int(
            self.model.jnt_qposadr[self.model.joint("palm_right_axes_free").id]
        )
        self.right_palm_geom_id = self.model.geom("palm_right_geom").id
        self.right_palm_axis_geom_ids = [
            self.model.geom(f"palm_right_axis_{axis}").id for axis in ("x", "y", "z")
        ]
        self.wrist_qposadr = int(
            self.model.jnt_qposadr[self.model.joint("wrist_left_free").id]
        )
        self.wrist_axes_qposadr = int(
            self.model.jnt_qposadr[self.model.joint("wrist_left_axes_free").id]
        )
        self.wrist_bone_qposadr = int(
            self.model.jnt_qposadr[
                self.model.joint("wrist_left_palm_bone_free").id
            ]
        )
        self.wrist_geom_id = self.model.geom("wrist_left_geom").id
        self.wrist_axis_geom_ids = [
            self.model.geom(f"wrist_left_axis_{axis}").id for axis in ("x", "y", "z")
        ]
        self.wrist_bone_geom_id = self.model.geom("wrist_left_palm_bone_geom").id
        self.right_wrist_qposadr = int(
            self.model.jnt_qposadr[self.model.joint("wrist_right_free").id]
        )
        self.right_wrist_axes_qposadr = int(
            self.model.jnt_qposadr[self.model.joint("wrist_right_axes_free").id]
        )
        self.right_wrist_bone_qposadr = int(
            self.model.jnt_qposadr[
                self.model.joint("wrist_right_palm_bone_free").id
            ]
        )
        self.right_wrist_geom_id = self.model.geom("wrist_right_geom").id
        self.right_wrist_axis_geom_ids = [
            self.model.geom(f"wrist_right_axis_{axis}").id
            for axis in ("x", "y", "z")
        ]
        self.right_wrist_bone_geom_id = self.model.geom(
            "wrist_right_palm_bone_geom"
        ).id
        self._palm_color = np.array([0.98, 0.65, 0.08, 1.0])
        self._wrist_color = np.array([0.72, 0.25, 0.85, 1.0])
        self._wrist_bone_color = np.array([0.55, 0.32, 0.68, 1.0])
        self._controller_colors = {
            "head": np.array([1.0, 0.10, 0.05, 1.0]),
            "left_hand": np.array([1.0, 0.10, 0.05, 1.0]),
            "right_hand": np.array([1.0, 0.10, 0.05, 1.0]),
        }
        self.controller_qposadr: dict[str, int] = {}
        self.controller_axes_qposadr: dict[str, int] = {}
        self.controller_geom_ids: dict[str, int] = {}
        self.controller_axis_geom_ids: dict[str, list[int]] = {}
        if self.show_controllers:
            for name in self._controller_caches:
                self.controller_qposadr[name] = int(
                    self.model.jnt_qposadr[
                        self.model.joint(f"controller_{name}_free").id
                    ]
                )
                self.controller_axes_qposadr[name] = int(
                    self.model.jnt_qposadr[
                        self.model.joint(f"controller_{name}_axes_free").id
                    ]
                )
                self.controller_geom_ids[name] = self.model.geom(
                    f"controller_{name}_geom"
                ).id
                self.controller_axis_geom_ids[name] = [
                    self.model.geom(f"controller_{name}_axis_{axis}").id
                    for axis in ("x", "y", "z")
                ]
        self.arm_axes_qposadr: dict[int, int] = {}
        self.arm_axis_geom_ids: dict[int, list[int]] = {}
        self._arm_axis_colors = self._world_axis_colors
        if self.show_arm_axes:
            self.arm_axes_qposadr = {
                index: int(
                    self.model.jnt_qposadr[
                        self.model.joint(f"arm_axes_free_{index}").id
                    ]
                )
                for index in ARM_AXIS_JOINT_INDICES
            }
            self.arm_axis_geom_ids = {
                index: [
                    self.model.geom(f"arm_axis_{index}_{axis}").id
                    for axis in ("x", "y", "z")
                ]
                for index in ARM_AXIS_JOINT_INDICES
            }
        self.joint_qposadr = [
            int(self.model.jnt_qposadr[self.model.joint(f"joint_free_{i}").id])
            for i in range(len(JOINT_NAMES))
        ]
        self.bone_qposadr = [
            int(self.model.jnt_qposadr[self.model.joint(f"bone_free_{i}").id])
            for i in range(len(BONE_EDGES))
        ]
        self.joint_geom_ids = [self.model.geom(f"joint_geom_{i}").id for i in range(len(JOINT_NAMES))]
        self.bone_geom_ids = [self.model.geom(f"bone_geom_{i}").id for i in range(len(BONE_EDGES))]
        self.foot_qposadr = {
            side: int(self.model.jnt_qposadr[self.model.joint(f"foot_free_{side}").id])
            for side in ("left", "right")
        }
        self.foot_geom_ids = {side: self.model.geom(f"foot_geom_{side}").id for side in ("left", "right")}
        self._joint_color = np.array([0.1, 0.45, 0.95, 1.0])
        self._bone_color = np.array([0.65, 0.72, 0.82, 1.0])
        self._foot_color = np.array([0.95, 0.45, 0.1, 0.9])
        self._stale_color = np.array([0.35, 0.35, 0.35, 1.0])
        self.raw_joint_qposadr: list[int] = []
        self.raw_bone_qposadr: list[int] = []
        self.raw_joint_geom_ids: list[int] = []
        self.raw_bone_geom_ids: list[int] = []
        self.raw_foot_qposadr: dict[str, int] = {}
        self.raw_foot_geom_ids: dict[str, int] = {}
        self.raw_pelvis_axes_qposadr: int | None = None
        self.raw_pelvis_axis_geom_ids: list[int] = []
        self.raw_arm_axes_qposadr: dict[int, int] = {}
        self.raw_arm_axis_geom_ids: dict[int, list[int]] = {}
        self._raw_joint_color = np.array([1.0, 0.05, 0.8, 0.82])
        self._raw_bone_color = np.array([1.0, 0.05, 0.8, 0.82])
        self._raw_foot_color = np.array([1.0, 0.15, 0.55, 0.86])
        self._raw_stale_color = np.array([0.35, 0.15, 0.3, 0.45])
        if show_raw:
            if self.show_raw_pelvis_axes:
                self.raw_pelvis_axes_qposadr = int(
                    self.model.jnt_qposadr[
                        self.model.joint("raw_pelvis_axes_free").id
                    ]
                )
                self.raw_pelvis_axis_geom_ids = [
                    self.model.geom(f"raw_pelvis_axis_{axis}").id
                    for axis in ("x", "y", "z")
                ]
            self.raw_joint_qposadr = [
                int(self.model.jnt_qposadr[self.model.joint(f"raw_joint_free_{i}").id])
                for i in range(len(JOINT_NAMES))
            ]
            self.raw_bone_qposadr = [
                int(self.model.jnt_qposadr[self.model.joint(f"raw_bone_free_{i}").id])
                for i in range(len(BONE_EDGES))
            ]
            self.raw_joint_geom_ids = [
                self.model.geom(f"raw_joint_geom_{i}").id
                for i in range(len(JOINT_NAMES))
            ]
            self.raw_bone_geom_ids = [
                self.model.geom(f"raw_bone_geom_{i}").id
                for i in range(len(BONE_EDGES))
            ]
            self.raw_foot_qposadr = {
                side: int(
                    self.model.jnt_qposadr[
                        self.model.joint(f"raw_foot_free_{side}").id
                    ]
                )
                for side in ("left", "right")
            }
            self.raw_foot_geom_ids = {
                side: self.model.geom(f"raw_foot_geom_{side}").id
                for side in ("left", "right")
            }
            if self.show_raw_arm_axes:
                self.raw_arm_axes_qposadr = {
                    index: int(
                        self.model.jnt_qposadr[
                            self.model.joint(f"raw_arm_axes_free_{index}").id
                        ]
                    )
                    for index in ARM_AXIS_JOINT_INDICES
                }
                self.raw_arm_axis_geom_ids = {
                    index: [
                        self.model.geom(f"raw_arm_axis_{index}_{axis}").id
                        for axis in ("x", "y", "z")
                    ]
                    for index in ARM_AXIS_JOINT_INDICES
                }
            for geom_id in (
                self.raw_joint_geom_ids
                + self.raw_bone_geom_ids
                + list(self.raw_foot_geom_ids.values())
            ):
                self.model.geom_rgba[geom_id, 3] = 0.0
        if self.hands_only:
            self._configure_hands_only_visibility()
        self._camera_initialized = False

    def _configure_hands_only_visibility(self) -> None:
        """Hide all geometry except the two arm chains in the primary/raw streams."""
        for index, geom_id in enumerate(self.joint_geom_ids):
            self.model.geom_rgba[geom_id, 3] = 1.0 if index in HAND_VIEW_JOINT_INDICES else 0.0
        for index, geom_id in enumerate(self.bone_geom_ids):
            self.model.geom_rgba[geom_id, 3] = 1.0 if index in HAND_VIEW_BONE_INDICES else 0.0
        for geom_id in list(self.foot_geom_ids.values()) + self.pelvis_axis_geom_ids:
            self.model.geom_rgba[geom_id, 3] = 0.0
        if self.show_arm_axes:
            for geom_ids in self.arm_axis_geom_ids.values():
                for geom_id in geom_ids:
                    self.model.geom_rgba[geom_id, 3] = 1.0
        if self.show_raw:
            for index, geom_id in enumerate(self.raw_joint_geom_ids):
                self.model.geom_rgba[geom_id, 3] = 0.75 if index in HAND_VIEW_JOINT_INDICES else 0.0
            for index, geom_id in enumerate(self.raw_bone_geom_ids):
                self.model.geom_rgba[geom_id, 3] = 0.75 if index in HAND_VIEW_BONE_INDICES else 0.0
            for geom_id in list(self.raw_foot_geom_ids.values()) + self.raw_pelvis_axis_geom_ids:
                self.model.geom_rgba[geom_id, 3] = 0.0
            if self.show_raw_arm_axes:
                for geom_ids in self.raw_arm_axis_geom_ids.values():
                    for geom_id in geom_ids:
                        self.model.geom_rgba[geom_id, 3] = 0.75

    def _update_arm_axes(
        self,
        cache: PoseFrameCache,
        axes_qposadr: dict[int, int],
        axis_geom_ids: dict[int, list[int]],
        stale_color: np.ndarray,
        position_offset: Sequence[float] = (0.0, 0.0, 0.0),
    ) -> None:
        """Place local XYZ triads on upper-limb joints only."""
        if not axes_qposadr:
            return
        positions, orientations, received_at = cache.snapshot()
        if positions is None or orientations is None:
            for geom_ids in axis_geom_ids.values():
                for geom_id in geom_ids:
                    self.model.geom_rgba[geom_id, 3] = 0.0
            return
        offset = _finite_offset(position_offset, "position_offset")
        stale = time.monotonic() - received_at > self.timeout
        for index, qposadr in axes_qposadr.items():
            self._set_free_joint(qposadr, positions[index] + offset, orientations[index])
            for geom_id, color in zip(axis_geom_ids[index], self._world_axis_colors):
                self.model.geom_rgba[geom_id] = (
                    color * np.array([1.0, 1.0, 1.0, 0.35])
                    if stale else color
                )

    def _convert_pose_array(
        self, message, stream: str
    ) -> tuple[list[np.ndarray], list[np.ndarray]] | None:
        if len(message.poses) != len(JOINT_NAMES):
            now = time.monotonic()
            if now - self._last_warning[stream] > 2.0:
                self.node.get_logger().warning(
                    f"Ignoring {stream} PoseArray with {len(message.poses)} poses; "
                    f"expected {len(JOINT_NAMES)}"
                )
                self._last_warning[stream] = now
            return None
        try:
            positions = [
                transform_position(
                    (pose.position.x, pose.position.y, pose.position.z),
                    self.scale,
                    self.yaw,
                )
                for pose in message.poses
            ]
            orientations = [transform_orientation(
                (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w), self.yaw)
                for pose in message.poses]
        except ValueError as exc:
            now = time.monotonic()
            if now - self._last_warning[stream] > 2.0:
                self.node.get_logger().warning(f"Ignoring {stream} PoseArray: {exc}")
                self._last_warning[stream] = now
            return None
        return positions, orientations

    def _pose_callback(self, message) -> None:
        frame = self._convert_pose_array(message, "primary")
        if frame is not None:
            self._primary_cache.update(*frame, received_at=time.monotonic())
            if not self.show_raw or not self._raw_floor_is_preferred():
                self._feed_floor("primary", *frame)

    def _raw_pose_callback(self, message) -> None:
        frame = self._convert_pose_array(message, "raw")
        if frame is not None:
            self._raw_cache.update(*frame, received_at=time.monotonic())
            if self.show_raw:
                self._feed_floor("raw", *frame)

    def _palm_pose_callback(self, message, side: str = "left") -> None:
        try:
            position = transform_position(
                (message.pose.position.x, message.pose.position.y, message.pose.position.z),
                self.scale,
                self.yaw,
            )
            orientation = transform_orientation(
                (
                    message.pose.orientation.x,
                    message.pose.orientation.y,
                    message.pose.orientation.z,
                    message.pose.orientation.w,
                ),
                self.yaw,
            )
        except ValueError as exc:
            self.node.get_logger().warning(f"Ignoring calibrated palm pose: {exc}")
            return
        cache = self._palm_cache if side == "left" else self._right_palm_cache
        cache.update(position, orientation, time.monotonic())

    def _wrist_pose_callback(self, message, side: str = "left") -> None:
        try:
            position = transform_position(
                (message.pose.position.x, message.pose.position.y, message.pose.position.z),
                self.scale,
                self.yaw,
            )
            orientation = transform_orientation(
                (
                    message.pose.orientation.x,
                    message.pose.orientation.y,
                    message.pose.orientation.z,
                    message.pose.orientation.w,
                ),
                self.yaw,
            )
        except ValueError as exc:
            self.node.get_logger().warning(f"Ignoring calibrated wrist pose: {exc}")
            return
        cache = self._wrist_cache if side == "left" else self._right_wrist_cache
        cache.update(position, orientation, time.monotonic())

    def _controller_pose_callback(self, message, name: str) -> None:
        """Store one raw PICO HMD/controller pose in viewer coordinates."""
        try:
            position = transform_position(
                (message.pose.position.x, message.pose.position.y, message.pose.position.z),
                self.scale,
                self.yaw,
            )
            orientation = transform_orientation(
                (
                    message.pose.orientation.x,
                    message.pose.orientation.y,
                    message.pose.orientation.z,
                    message.pose.orientation.w,
                ),
                self.yaw,
            )
        except ValueError as exc:
            self.node.get_logger().warning(f"Ignoring controller pose {name}: {exc}")
            return
        received_at = time.monotonic()
        self._controller_caches[name].update(position, orientation, received_at)
        if name == "head":
            return
        side = "left" if name == "left_hand" else "right"
        artifact = self._tcp_artifacts.get(side)
        if artifact is None:
            return
        try:
            palm_position, palm_orientation = derive_tcp_palm_pose(
                position,
                orientation,
                artifact[0],
                artifact[1],
                scale=self.scale,
            )
        except ValueError as exc:
            self.node.get_logger().warning(
                f"Ignoring derived {name} TCP palm pose: {exc}"
            )
            return
        self._derived_palm_cache[side].update(
            palm_position, palm_orientation, received_at
        )

    def _palm_snapshot(
        self, side: str
    ) -> tuple[np.ndarray | None, np.ndarray | None, float, str]:
        """Prefer live topic data, then derive from controller + TCP artifact."""
        explicit_cache = self._palm_cache if side == "left" else self._right_palm_cache
        explicit = explicit_cache.snapshot()
        derived = self._derived_palm_cache[side].snapshot()
        now = time.monotonic()
        explicit_live = explicit[0] is not None and now - explicit[2] <= self.timeout
        derived_live = derived[0] is not None and now - derived[2] <= self.timeout
        if explicit_live or not derived_live:
            return (*explicit, "topic")
        return (*derived, "tcp_artifact")

    def _raw_floor_is_preferred(self) -> bool:
        _, _, received_at = self._raw_cache.snapshot()
        return (
            received_at >= self._ground_reset_at
            and time.monotonic() - received_at <= RAW_FLOOR_STALE_SEC
        )

    def _feed_floor(
        self,
        source: str,
        positions: list[np.ndarray],
        orientations: list[np.ndarray],
    ) -> None:
        if self._ground_estimator.height is not None:
            return
        if source != self._floor_source:
            self._ground_estimator.reset()
            self._floor_source = source
        half_extents = (
            RAW_FOOT_HALF_EXTENTS if source == "raw" else FOOT_HALF_EXTENTS
        )
        self._observe_floor(positions, orientations, half_extents)

    def _observe_floor(
        self,
        positions: list[np.ndarray],
        orientations: list[np.ndarray],
        half_extents: Sequence[float],
    ) -> None:
        left_height = foot_sole_height(
            positions[10], orientations[10], half_extents
        )
        right_height = foot_sole_height(
            positions[11], orientations[11], half_extents
        )
        self._ground_estimator.observe(left_height, right_height)

    def _world_reset_callback(self, _message) -> None:
        self._ground_estimator.reset()
        self._floor_source = None
        self._ground_reset_at = time.monotonic()
        self.node.get_logger().info(
            "PICO world reset received; waiting for 30 stable foot frames to lock ground"
        )

    def _apply_ground_plane(self) -> bool:
        height = self._ground_estimator.height
        if height is None:
            self.model.geom_rgba[self.ground_geom_id, 3] = 0.0
            for geom_id in self.world_axis_geom_ids:
                self.model.geom_rgba[geom_id, 3] = 0.0
            return False
        self.model.body_pos[self.ground_body_id, 2] = height
        self.model.geom_rgba[self.ground_geom_id] = (1.0, 1.0, 1.0, 1.0)
        for geom_id, color in zip(
            self.world_axis_geom_ids, self._world_axis_colors
        ):
            self.model.geom_rgba[geom_id] = color
        return True

    def _set_free_joint(self, qposadr: int, position: np.ndarray, quaternion: np.ndarray) -> None:
        self.data.qpos[qposadr:qposadr + 7] = (
            position[0], position[1], position[2],
            quaternion[0], quaternion[1], quaternion[2], quaternion[3],
        )

    def _update_skeleton(
        self,
        cache: PoseFrameCache,
        stream: str,
        topic: str,
        joint_qposadr: list[int],
        bone_qposadr: list[int],
        joint_geom_ids: list[int],
        bone_geom_ids: list[int],
        foot_qposadr: dict[str, int],
        foot_geom_ids: dict[str, int],
        joint_color: np.ndarray,
        bone_color: np.ndarray,
        foot_color: np.ndarray,
        stale_color: np.ndarray,
        pelvis_axes_qposadr: int | None,
        pelvis_axis_geom_ids: list[int],
        pelvis_axis_colors: Sequence[np.ndarray],
        position_offset: Sequence[float] = (0.0, 0.0, 0.0),
    ) -> bool:
        positions, orientations, received_at = cache.snapshot()
        if positions is None:
            return False
        offset = _finite_offset(position_offset, "position_offset")
        display_positions = [position + offset for position in positions]
        stale = time.monotonic() - received_at > self.timeout
        for index, position in enumerate(display_positions):
            self._set_free_joint(
                joint_qposadr[index], position, np.array([1.0, 0.0, 0.0, 0.0])
            )
            self.model.geom_rgba[joint_geom_ids[index]] = (
                stale_color if stale else joint_color
            )
        for index, (parent_index, child_index) in enumerate(BONE_EDGES):
            center, quaternion, length = capsule_pose(
                display_positions[parent_index], display_positions[child_index]
            )
            self._set_free_joint(bone_qposadr[index], center, quaternion)
            geom_id = bone_geom_ids[index]
            self.model.geom_size[geom_id, 1] = max(length / 2.0, 1e-5)
            self.model.geom_rgba[geom_id] = stale_color if stale else bone_color
        if orientations is not None and pelvis_axes_qposadr is not None and pelvis_axis_geom_ids:
            self._set_free_joint(
                pelvis_axes_qposadr, display_positions[0], orientations[0]
            )
            for geom_id, color in zip(pelvis_axis_geom_ids, pelvis_axis_colors):
                self.model.geom_rgba[geom_id] = color
            for side, index in (("left", 10), ("right", 11)):
                self._set_free_joint(
                    foot_qposadr[side], display_positions[index], orientations[index]
                )
                self.model.geom_rgba[foot_geom_ids[side]] = (
                    stale_color if stale else foot_color
                )
        if stale:
            now = time.monotonic()
            if now - self._last_stale_warning[stream] > 2.0:
                self.node.get_logger().warning(f"{topic} data is stale")
                self._last_stale_warning[stream] = now
        return True

    def _update_scene(self) -> bool:
        self._apply_ground_plane()
        primary_updated = self._update_skeleton(
            self._primary_cache,
            "primary",
            self.topic,
            self.joint_qposadr,
            self.bone_qposadr,
            self.joint_geom_ids,
            self.bone_geom_ids,
            self.foot_qposadr,
            self.foot_geom_ids,
            self._joint_color,
            self._bone_color,
            self._foot_color,
            self._stale_color,
            self.pelvis_axes_qposadr,
            self.pelvis_axis_geom_ids,
            self._pelvis_axis_colors,
            position_offset=(0.0, 0.0, 0.0),
        )
        if self.show_raw:
            self._update_skeleton(
                self._raw_cache,
                "raw",
                self.raw_topic,
                self.raw_joint_qposadr,
                self.raw_bone_qposadr,
                self.raw_joint_geom_ids,
                self.raw_bone_geom_ids,
                self.raw_foot_qposadr,
                self.raw_foot_geom_ids,
                self._raw_joint_color,
                self._raw_bone_color,
                self._raw_foot_color,
                self._raw_stale_color,
                self.raw_pelvis_axes_qposadr,
                self.raw_pelvis_axis_geom_ids,
                tuple(color * np.array([1.0, 1.0, 1.0, 0.45])
                      for color in self._world_axis_colors),
                position_offset=self.raw_offset,
            )
        if self.show_arm_axes:
            self._update_arm_axes(
                self._primary_cache,
                self.arm_axes_qposadr,
                self.arm_axis_geom_ids,
                self._stale_color,
                position_offset=(0.0, 0.0, 0.0),
            )
        if self.show_raw_arm_axes:
            self._update_arm_axes(
                self._raw_cache,
                self.raw_arm_axes_qposadr,
                self.raw_arm_axis_geom_ids,
                self._raw_stale_color,
                position_offset=self.raw_offset,
            )
        for side, qposadr, axes_qposadr, geom_id, axis_geom_ids in (
            ("left", self.palm_qposadr, self.palm_axes_qposadr,
             self.palm_geom_id, self.palm_axis_geom_ids),
            ("right", self.right_palm_qposadr, self.right_palm_axes_qposadr,
             self.right_palm_geom_id, self.right_palm_axis_geom_ids),
        ):
            palm_position, palm_orientation, palm_received_at, _ = self._palm_snapshot(side)
            if palm_position is None or palm_orientation is None:
                self.model.geom_rgba[geom_id, 3] = 0.0
                for axis_geom_id in axis_geom_ids:
                    self.model.geom_rgba[axis_geom_id, 3] = 0.0
                continue
            self._set_free_joint(qposadr, palm_position, palm_orientation)
            self._set_free_joint(axes_qposadr, palm_position, palm_orientation)
            palm_stale = time.monotonic() - palm_received_at > self.timeout
            self.model.geom_rgba[geom_id] = self._stale_color if palm_stale else self._palm_color
            for axis_geom_id, color in zip(axis_geom_ids, self._world_axis_colors):
                self.model.geom_rgba[axis_geom_id] = (
                    color * np.array([1.0, 1.0, 1.0, 0.35]) if palm_stale else color
                )
        # Prefer an explicitly published calibrated wrist pose.  The current
        # skeleton filter also carries the reconstructed wrist in the primary
        # PoseArray, so fall back to that joint when the optional wrist topic
        # is not running; this keeps the TCP-to-wrist relationship visible in
        # either launch mode.
        primary_positions, primary_orientations, primary_received_at = (
            self._primary_cache.snapshot()
        )
        for (
            side,
            wrist_cache,
            wrist_qposadr,
            wrist_axes_qposadr,
            wrist_bone_qposadr,
            wrist_geom_id,
            wrist_axis_geom_ids,
            wrist_bone_geom_id,
            joint_name,
        ) in (
            (
                "left",
                self._wrist_cache,
                self.wrist_qposadr,
                self.wrist_axes_qposadr,
                self.wrist_bone_qposadr,
                self.wrist_geom_id,
                self.wrist_axis_geom_ids,
                self.wrist_bone_geom_id,
                "LEFT_WRIST",
            ),
            (
                "right",
                self._right_wrist_cache,
                self.right_wrist_qposadr,
                self.right_wrist_axes_qposadr,
                self.right_wrist_bone_qposadr,
                self.right_wrist_geom_id,
                self.right_wrist_axis_geom_ids,
                self.right_wrist_bone_geom_id,
                "RIGHT_WRIST",
            ),
        ):
            wrist_position, wrist_orientation, wrist_received_at = wrist_cache.snapshot()
            palm_position, palm_orientation, palm_received_at, _ = self._palm_snapshot(side)
            index = JOINT_NAMES.index(joint_name)
            primary_position = (
                primary_positions[index] if primary_positions is not None else None
            )
            primary_orientation = (
                primary_orientations[index]
                if primary_orientations is not None
                else None
            )
            wrist_position, wrist_orientation, wrist_received_at, _ = (
                resolve_visualization_wrist_pose(
                    wrist_position,
                    wrist_orientation,
                    wrist_received_at,
                    now=time.monotonic(),
                    timeout=self.timeout,
                    palm_position=palm_position,
                    palm_orientation=palm_orientation,
                    palm_received_at=palm_received_at,
                    wrist_to_palm=self._wrist_pivot_artifacts[side],
                    primary_position=primary_position,
                    primary_orientation=primary_orientation,
                    primary_received_at=primary_received_at,
                )
            )
            if (
                wrist_position is None
                or wrist_orientation is None
                or palm_position is None
            ):
                self.model.geom_rgba[wrist_geom_id, 3] = 0.0
                self.model.geom_rgba[wrist_bone_geom_id, 3] = 0.0
                for axis_geom_id in wrist_axis_geom_ids:
                    self.model.geom_rgba[axis_geom_id, 3] = 0.0
                continue
            self._set_free_joint(wrist_qposadr, wrist_position, wrist_orientation)
            self._set_free_joint(
                wrist_axes_qposadr, wrist_position, wrist_orientation
            )
            center, quaternion, length = capsule_pose(wrist_position, palm_position)
            self._set_free_joint(wrist_bone_qposadr, center, quaternion)
            self.model.geom_size[wrist_bone_geom_id, 1] = max(length / 2.0, 1e-5)
            stale = (
                time.monotonic() - wrist_received_at > self.timeout
                or time.monotonic() - palm_received_at > self.timeout
            )
            self.model.geom_rgba[wrist_geom_id] = (
                self._stale_color if stale else self._wrist_color
            )
            self.model.geom_rgba[wrist_bone_geom_id] = (
                self._stale_color if stale else self._wrist_bone_color
            )
            for axis_geom_id, color in zip(wrist_axis_geom_ids, self._world_axis_colors):
                self.model.geom_rgba[axis_geom_id] = (
                    color * np.array([1.0, 1.0, 1.0, 0.35]) if stale else color
                )
        if self.show_controllers:
            for name, cache in self._controller_caches.items():
                position, orientation, received_at = cache.snapshot()
                geom_id = self.controller_geom_ids[name]
                axis_geom_ids = self.controller_axis_geom_ids[name]
                if position is None or orientation is None:
                    self.model.geom_rgba[geom_id, 3] = 0.0
                    for axis_geom_id in axis_geom_ids:
                        self.model.geom_rgba[axis_geom_id, 3] = 0.0
                    continue
                self._set_free_joint(self.controller_qposadr[name], position, orientation)
                self._set_free_joint(
                    self.controller_axes_qposadr[name], position, orientation
                )
                stale = time.monotonic() - received_at > self.timeout
                self.model.geom_rgba[geom_id] = (
                    self._stale_color if stale else self._controller_colors[name]
                )
                for axis_geom_id, color in zip(axis_geom_ids, self._world_axis_colors):
                    self.model.geom_rgba[axis_geom_id] = (
                        color * np.array([1.0, 1.0, 1.0, 0.35]) if stale else color
                    )
        if self.hands_only:
            self._configure_hands_only_visibility()
        self.mujoco.mj_forward(self.model, self.data)
        return True

    @staticmethod
    def _initialize_camera(viewer, target: np.ndarray) -> None:
        with viewer.lock():
            viewer.cam.lookat[:] = target
            viewer.cam.distance = 2.5
            viewer.cam.azimuth = 180.0
            viewer.cam.elevation = -15.0

    def _update_viewer_scene(self, viewer) -> bool:
        with viewer.lock():
            return self._update_scene()

    def _cache_status(self, cache: PalmPoseCache) -> str:
        position, _, received_at = cache.snapshot()
        if position is None:
            return "missing"
        return "stale" if time.monotonic() - received_at > self.timeout else "live"

    def _palm_status(self, side: str) -> str:
        position, _, received_at, source = self._palm_snapshot(side)
        if position is None:
            return "missing"
        state = "stale" if time.monotonic() - received_at > self.timeout else "live"
        return f"{state}/{source}"

    def _wrist_status(self, side: str) -> str:
        explicit_cache = self._wrist_cache if side == "left" else self._right_wrist_cache
        position, _, received_at = explicit_cache.snapshot()
        now = time.monotonic()
        palm_position, _, palm_received_at, _ = self._palm_snapshot(side)
        if (
            palm_position is not None
            and self._wrist_pivot_artifacts[side] is not None
            and now - palm_received_at <= self.timeout
        ):
            return "live/wrist_pivot"
        if position is not None and now - received_at <= self.timeout:
            return "live/topic"
        primary_positions, _, primary_received_at = self._primary_cache.snapshot()
        if primary_positions is not None:
            joint_name = "LEFT_WRIST" if side == "left" else "RIGHT_WRIST"
            index = JOINT_NAMES.index(joint_name)
            if index < len(primary_positions):
                state = "stale" if now - primary_received_at > self.timeout else "live"
                return f"{state}/primary_posearray"
        if position is not None:
            return "stale/topic"
        return "missing"

    def _update_hand_overlay(self, viewer) -> None:
        if not self.hands_only and not self.show_raw and not self.show_controllers:
            viewer.clear_texts()
            return
        positions, _, _ = self._primary_cache.snapshot()
        if positions is None and self.hands_only:
            viewer.set_texts([])
            return
        if not self.hands_only:
            status_lines = []
            if self.show_controllers:
                status_lines.append(
                    "raw poses: "
                    f"head={self._cache_status(self._controller_caches['head'])}, "
                    f"left_hand={self._cache_status(self._controller_caches['left_hand'])}, "
                    f"right_hand={self._cache_status(self._controller_caches['right_hand'])}"
                )
            status_lines.append(
                "TCP palms: "
                f"left={self._palm_status('left')}, "
                f"right={self._palm_status('right')}"
            )
            status_lines.append(
                "wrists: "
                f"left={self._wrist_status('left')}, "
                f"right={self._wrist_status('right')}"
            )
            viewer.set_texts(
                (
                    self.mujoco.mjtFontScale.mjFONTSCALE_150,
                    self.mujoco.mjtGridPos.mjGRID_TOPLEFT,
                    "PICO skeleton comparison",
                    "primary: blue/gray\n"
                    f"raw: magenta, offset=[{self.raw_offset[0]:.2f}, "
                    f"{self.raw_offset[1]:.2f}, {self.raw_offset[2]:.2f}] m"
                    + "\n"
                    + "\n".join(status_lines),
                )
            )
            return
        palm_position, _, palm_received_at = self._palm_cache.snapshot()
        palm_text = format_palm_position_overlay(
            palm_position if palm_position is not None and time.monotonic() - palm_received_at <= self.timeout else None
        )
        right_palm_position, _, right_palm_received_at = self._right_palm_cache.snapshot()
        right_palm_text = format_palm_position_overlay(
            right_palm_position
            if right_palm_position is not None
            and time.monotonic() - right_palm_received_at <= self.timeout
            else None
        ).replace("left palm", "right palm")
        viewer.set_texts((
            self.mujoco.mjtFontScale.mjFONTSCALE_150,
            self.mujoco.mjtGridPos.mjGRID_TOPLEFT,
            "PICO hand endpoints (pico_ground)",
            "primary: blue/gray\n"
            f"raw: magenta, offset=[{self.raw_offset[0]:.2f}, "
            f"{self.raw_offset[1]:.2f}, {self.raw_offset[2]:.2f}] m\n"
            + format_hand_position_overlay(positions) + "\n" + palm_text + "\n" + right_palm_text,
        ))

    def run(self) -> None:
        import mujoco.viewer

        def spin_ros() -> None:
            try:
                self.rclpy.spin(self.node)
            except (self.external_shutdown_exception, self.rcl_error):
                pass

        spin_thread = threading.Thread(target=spin_ros, daemon=True)
        spin_thread.start()
        try:
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                while viewer.is_running() and self.rclpy.ok():
                    updated = self._update_viewer_scene(viewer)
                    with viewer.lock():
                        self._update_hand_overlay(viewer)
                    if updated and not self._camera_initialized:
                        positions, _, _ = self._primary_cache.snapshot()
                        target = None if positions is None else positions[0]
                        if target is not None:
                            self._initialize_camera(viewer, target)
                            self._camera_initialized = True
                    viewer.sync()
                    time.sleep(max(1.0 / self.rate, 0.001))
        finally:
            if self.rclpy.ok():
                self.rclpy.shutdown()
            spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    arguments = build_parser().parse_args()
    if arguments.rate <= 0:
        raise SystemExit("--rate must be positive")
    if arguments.timeout < 0:
        raise SystemExit("--timeout must be non-negative")
    try:
        import rclpy
        rclpy.init()
        SmplMujocoVisualizer(
            arguments.topic,
            arguments.scale,
            arguments.rate,
            arguments.timeout,
            arguments.yaw,
            raw_topic=arguments.raw_topic,
            palm_topic=arguments.palm_topic,
            right_palm_topic=arguments.right_palm_topic,
            wrist_topic=arguments.wrist_topic,
            right_wrist_topic=arguments.right_wrist_topic,
            left_tcp_artifact=arguments.left_tcp_artifact,
            right_tcp_artifact=arguments.right_tcp_artifact,
            left_wrist_pivot_artifact=arguments.left_wrist_pivot_artifact,
            right_wrist_pivot_artifact=arguments.right_wrist_pivot_artifact,
            raw_offset=arguments.raw_offset,
            show_raw=arguments.show_raw,
            hands_only=arguments.hands_only,
            show_arm_axes=arguments.show_arm_axes,
            show_raw_arm_axes=arguments.show_raw_arm_axes,
            show_raw_pelvis_axes=arguments.show_raw_pelvis_axes,
            head_topic=arguments.head_topic,
            left_controller_topic=arguments.left_controller_topic,
            right_controller_topic=arguments.right_controller_topic,
            # Controller/HMD markers are an explicit overlay.  Keeping this
            # independent from --show-raw prevents a raw skeleton comparison
            # from unexpectedly adding six more endpoint/axis markers.
            show_controllers=arguments.show_controllers,
        ).run()
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc
