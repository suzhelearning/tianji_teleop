#!/usr/bin/env python3
"""Publish a PICO skeleton corrected by calibrated TCP palm observations."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import copy
import json
import math
from pathlib import Path
from typing import Deque

import numpy as np

from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import String, UInt64

from pico_calibration_artifact import (
    file_sha256 as calibration_file_sha256,
    validate_artifact as validate_calibration_artifact,
)

from pico_palm_skeleton_filter_core import (
    DEFAULT_LEFT_SHOULDER_LOCAL_X_OFFSET_RAD,
    DEFAULT_RIGHT_SHOULDER_LOCAL_X_OFFSET_RAD,
    IK_ELBOW_FRAME_SEMANTICS,
    IK_SHOULDER_FRAME_SEMANTICS,
    JOINT_COUNT,
    SIDE_INDICES,
    SideCalibration,
    adapt_arm_frames_to_ik_frames,
    correct_side,
    fit_side_calibration,
    quat_to_matrix,
)


@dataclass(frozen=True)
class PalmSample:
    stamp_ns: int
    position: np.ndarray
    orientation_xyzw: np.ndarray
    frame_id: str
    tracking_epoch: int = 0


@dataclass(frozen=True)
class LoadedArmGeometry:
    side: str
    upper_arm_length_m: float
    forearm_length_m: float
    calibration_revision: int
    source_path: str


@dataclass
class _SideState:
    calibration: SideCalibration | None = None
    positions: Deque[np.ndarray] | None = None
    orientations: Deque[np.ndarray] | None = None
    palm_positions: Deque[np.ndarray] | None = None
    palm_orientations: Deque[np.ndarray] | None = None
    sample_stamps_ns: Deque[int] | None = None
    previous_elbow_position: np.ndarray | None = None
    previous_corrected_positions: np.ndarray | None = None
    previous_corrected_orientations_xyzw: np.ndarray | None = None
    wrist_to_palm_distance_m: float | None = None
    wrist_pivot_status: str = "not_configured"


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def pose_array_to_arrays(message: PoseArray) -> tuple[np.ndarray, np.ndarray]:
    if message.header.frame_id != "pico":
        raise ValueError(
            f"raw skeleton frame must be pico, got {message.header.frame_id!r}"
        )
    if len(message.poses) != JOINT_COUNT:
        raise ValueError(f"raw skeleton must contain 24 poses, got {len(message.poses)}")
    positions = np.zeros((JOINT_COUNT, 3), dtype=float)
    orientations = np.zeros((JOINT_COUNT, 4), dtype=float)
    for index, pose in enumerate(message.poses):
        positions[index] = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
        ]
        orientations[index] = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
    if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(orientations)):
        raise ValueError("raw skeleton contains non-finite pose values")
    return positions, orientations


def pose_array_with_ik_arm_frames(
    message: PoseArray,
    *,
    left_offset_rad: float,
    right_offset_rad: float,
    previous_orientations_xyzw: np.ndarray | None = None,
) -> PoseArray:
    """Copy a corrected skeleton and adapt its shoulder/elbow frame bases."""

    positions, orientations = pose_array_to_arrays(message)
    adapted_orientations = adapt_arm_frames_to_ik_frames(
        positions,
        orientations,
        left_shoulder_offset_rad=left_offset_rad,
        right_shoulder_offset_rad=right_offset_rad,
        previous_orientations_xyzw=previous_orientations_xyzw,
    )
    output = copy.deepcopy(message)
    for side in SIDE_INDICES:
        shoulder, elbow, _, _ = SIDE_INDICES[side]
        for index in (shoulder, elbow):
            orientation = output.poses[index].orientation
            orientation.x = float(adapted_orientations[index, 0])
            orientation.y = float(adapted_orientations[index, 1])
            orientation.z = float(adapted_orientations[index, 2])
            orientation.w = float(adapted_orientations[index, 3])
    return output


def pose_stamped_to_sample(message: PoseStamped) -> PalmSample:
    position = np.array(
        [
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        ],
        dtype=float,
    )
    orientation = np.array(
        [
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        ],
        dtype=float,
    )
    if not np.all(np.isfinite(position)) or not np.all(np.isfinite(orientation)):
        raise ValueError("palm pose contains non-finite values")
    # Validate the quaternion now so a malformed palm never enters the cache.
    quat_to_matrix(orientation)
    return PalmSample(
        stamp_ns=stamp_to_ns(message.header.stamp),
        position=position,
        orientation_xyzw=orientation,
        frame_id=message.header.frame_id,
    )


def load_wrist_pivot_artifact(
    path: str | Path, side: str
) -> tuple[float | None, str]:
    """Load wrist-to-palm distance for the palm-local positive-X contract.

    New artifacts may store ``wrist_to_palm_distance_m`` directly.  Existing
    artifacts store a three-dimensional ``wrist_to_palm_m`` vector; those are
    migrated deterministically by taking the vector norm.  Its legacy XYZ
    direction is deliberately not reused because runtime geometry fixes the
    palm on wrist/palm-local positive X.
    """

    raw_path = str(path).strip()
    if not raw_path:
        return None, "not_configured"
    artifact_path = Path(raw_path).expanduser()
    if not artifact_path.exists():
        return None, "missing"
    try:
        import yaml

        validate_calibration_artifact(artifact_path, "wrist", side)
        with artifact_path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream) or {}
        raw_distance = document.get("wrist_to_palm_distance_m")
        if raw_distance is not None:
            distance = float(raw_distance)
            status = "loaded_scalar_distance"
        else:
            raw_vector = document.get("wrist_to_palm_m")
            if isinstance(raw_vector, dict):
                vector = np.asarray(
                    [raw_vector[axis] for axis in ("x", "y", "z")], dtype=float
                )
            else:
                vector = np.asarray(raw_vector, dtype=float)
            if vector.shape != (3,) or not np.all(np.isfinite(vector)):
                raise ValueError("wrist_to_palm_m must be a finite 3-vector")
            distance = float(np.linalg.norm(vector))
            status = "loaded_legacy_vector_norm"
        if not np.isfinite(distance) or distance <= 1.0e-9:
            raise ValueError("wrist-to-palm distance must be finite and positive")
    except (OSError, ImportError, KeyError, TypeError, ValueError) as error:
        return None, f"invalid:{error}"
    return distance, status


def load_tcp_calibration_revision(path: str | Path, side: str) -> int:
    """Read the TCP revision that produced the palm observation exactly once."""
    return validate_calibration_artifact(path, "tcp", side).calibration_revision


def file_sha256(path: str | Path) -> str:
    return calibration_file_sha256(path)


def load_arm_geometry_artifact(
    path: str | Path,
    *,
    expected_side: str,
    expected_tcp_revision: int,
    expected_tcp_sha256: str,
    expected_wrist_pivot_sha256: str,
    tcp_path: str | Path | None = None,
    wrist_path: str | Path | None = None,
) -> LoadedArmGeometry:
    """Load an accepted side-specific geometry artifact with exact lineage."""
    if tcp_path is not None or wrist_path is not None:
        if tcp_path is None or wrist_path is None:
            raise ValueError("TCP and wrist artifact paths must be provided together")
        summary = validate_calibration_artifact(
            path,
            "geometry",
            expected_side,
            tcp_path=tcp_path,
            wrist_path=wrist_path,
        )
    else:
        summary = validate_calibration_artifact(
            path,
            "geometry",
            expected_side,
            expected_tcp_revision=expected_tcp_revision,
            expected_tcp_sha256=expected_tcp_sha256,
            expected_wrist_pivot_sha256=expected_wrist_pivot_sha256,
        )
    import yaml

    artifact_path = Path(path).expanduser()
    try:
        document = yaml.safe_load(artifact_path.read_text(encoding="utf-8")) or {}
        upper = float(document["upper_arm_length_m"])
        forearm = float(document["forearm_length_m"])
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        raise ValueError(
            f"{expected_side} geometry artifact payload unreadable: {error}"
        ) from error
    return LoadedArmGeometry(
        side=expected_side,
        upper_arm_length_m=upper,
        forearm_length_m=forearm,
        calibration_revision=summary.calibration_revision,
        source_path=summary.path,
    )


def load_left_arm_geometry_artifact(
    path: str | Path,
    *,
    expected_tcp_revision: int,
    expected_tcp_sha256: str,
    expected_wrist_pivot_sha256: str,
) -> LoadedArmGeometry:
    """Compatibility loader for the accepted left-arm v3 artifact."""
    return load_arm_geometry_artifact(
        path,
        expected_side="left",
        expected_tcp_revision=expected_tcp_revision,
        expected_tcp_sha256=expected_tcp_sha256,
        expected_wrist_pivot_sha256=expected_wrist_pivot_sha256,
    )


class PicoPalmSkeletonFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("pico_palm_skeleton_filter")
        self._raw_topic = self.declare_parameter("raw_topic", "/pico/smpl_raw").value
        self._left_palm_topic = self.declare_parameter(
            "left_palm_topic", "/pico/palm_left"
        ).value
        self._right_palm_topic = self.declare_parameter(
            "right_palm_topic", "/pico/palm_right"
        ).value
        self._output_topic = self.declare_parameter(
            "output_topic", "/pico/smpl_palm_corrected"
        ).value
        self._ik_output_topic = self.declare_parameter(
            "ik_output_topic", "/pico/smpl_palm_corrected_ik"
        ).value
        self._left_shoulder_local_x_offset_rad = float(
            self.declare_parameter(
                "left_shoulder_local_x_offset_rad",
                DEFAULT_LEFT_SHOULDER_LOCAL_X_OFFSET_RAD,
            ).value
        )
        self._right_shoulder_local_x_offset_rad = float(
            self.declare_parameter(
                "right_shoulder_local_x_offset_rad",
                DEFAULT_RIGHT_SHOULDER_LOCAL_X_OFFSET_RAD,
            ).value
        )
        self._status_topic = self.declare_parameter(
            "status_topic", "/pico/smpl_palm_corrected/status"
        ).value
        self._tracking_epoch_topic = self.declare_parameter(
            "tracking_epoch_topic", "/pico/tracking_epoch"
        ).value
        self._tracking_epoch_status_topic = self.declare_parameter(
            "tracking_epoch_status_topic", "/pico/tracking_epoch/status"
        ).value
        self._max_skew_s = float(self.declare_parameter("max_skew_s", 0.03).value)
        self._min_calibration_samples = int(
            self.declare_parameter("min_calibration_samples", 60).value
        )
        self._cache_size = int(self.declare_parameter("palm_cache_size", 120).value)
        self._ratio_max_stretch = float(
            self.declare_parameter("ratio_max_stretch", 0.995).value
        )
        self._require_wrist_pivot_artifact = bool(
            self.declare_parameter("require_wrist_pivot_artifact", False).value
        )
        geometry_paths = {
            "left": self.declare_parameter(
                "left_arm_geometry_artifact", ""
            ).value,
            "right": self.declare_parameter(
                "right_arm_geometry_artifact", ""
            ).value,
        }
        require_geometry = {
            "left": bool(
                self.declare_parameter(
                    "require_left_arm_geometry_artifact", False
                ).value
            ),
            "right": bool(
                self.declare_parameter(
                    "require_right_arm_geometry_artifact", False
                ).value
            ),
        }
        wrist_pivot_paths = {
            "left": self.declare_parameter(
                "left_wrist_pivot_artifact",
                "~/.config/pico_tracker/pico_left_wrist_pivot.yaml",
            ).value,
            "right": self.declare_parameter(
                "right_wrist_pivot_artifact",
                "~/.config/pico_tracker/pico_right_wrist_pivot.yaml",
            ).value,
        }
        tcp_artifact_paths = {
            "left": self.declare_parameter(
                "left_tcp_artifact",
                "~/.config/pico_tracker/pico_left_palm_tcp.yaml",
            ).value,
            "right": self.declare_parameter(
                "right_tcp_artifact",
                "~/.config/pico_tracker/pico_right_palm_tcp.yaml",
            ).value,
        }
        self._tcp_calibration_revisions = {side: 0 for side in SIDE_INDICES}
        for side in SIDE_INDICES:
            try:
                self._tcp_calibration_revisions[side] = (
                    load_tcp_calibration_revision(tcp_artifact_paths[side], side)
                )
            except ValueError as error:
                if str(geometry_paths[side]).strip():
                    raise
                self.get_logger().warning(
                    f"{side} TCP artifact unavailable to skeleton filter ({error}); "
                    "that side remains on raw-SMPL baseline until its palm publisher is valid"
                )
        loaded_geometry: dict[str, LoadedArmGeometry | None] = {
            side: None for side in SIDE_INDICES
        }
        for side in SIDE_INDICES:
            geometry_path = str(geometry_paths[side]).strip()
            if geometry_path:
                try:
                    loaded_geometry[side] = load_arm_geometry_artifact(
                        geometry_path,
                        expected_side=side,
                        expected_tcp_revision=self._tcp_calibration_revisions[side],
                        expected_tcp_sha256=file_sha256(tcp_artifact_paths[side]),
                        expected_wrist_pivot_sha256=file_sha256(
                            wrist_pivot_paths[side]
                        ),
                        tcp_path=tcp_artifact_paths[side],
                        wrist_path=wrist_pivot_paths[side],
                    )
                except ValueError as error:
                    if require_geometry[side]:
                        raise
                    self.get_logger().warning(
                        f"{side} arm geometry artifact rejected ({error}); "
                        "using raw-SMPL baseline learning"
                    )
            elif require_geometry[side]:
                raise ValueError(f"{side}_arm_geometry_artifact is required")
        if self._max_skew_s <= 0.0:
            raise ValueError("max_skew_s must be positive")
        if self._min_calibration_samples < 1:
            raise ValueError("min_calibration_samples must be positive")
        if self._cache_size < self._min_calibration_samples:
            raise ValueError("palm_cache_size must cover calibration samples")
        if not self._ik_output_topic:
            raise ValueError("ik_output_topic must not be empty")
        if self._ik_output_topic == self._output_topic:
            raise ValueError("ik_output_topic must differ from output_topic")
        if not math.isfinite(self._left_shoulder_local_x_offset_rad):
            raise ValueError("left_shoulder_local_x_offset_rad must be finite")
        if not math.isfinite(self._right_shoulder_local_x_offset_rad):
            raise ValueError("right_shoulder_local_x_offset_rad must be finite")

        self._palm_cache: dict[str, Deque[PalmSample]] = {
            "left": deque(maxlen=self._cache_size),
            "right": deque(maxlen=self._cache_size),
        }
        self._tracking_epoch = 0
        self._tracking_epoch_numeric = 0
        self._tracking_epoch_source = "unknown"
        self._source_frame_id = ""
        self._previous_ik_orientations_xyzw: np.ndarray | None = None
        self._ik_frame_valid = False
        self._ik_frame_failure_reason = "not_received"
        self._side_state: dict[str, _SideState] = {}
        for side in SIDE_INDICES:
            wrist_to_palm_distance, pivot_status = load_wrist_pivot_artifact(
                wrist_pivot_paths[side], side
            )
            if loaded_geometry[side] is not None and wrist_to_palm_distance is None:
                raise ValueError(
                    f"{side} quick geometry requires its valid wrist pivot artifact: "
                    f"{pivot_status}"
                )
            if self._require_wrist_pivot_artifact and wrist_to_palm_distance is None:
                raise ValueError(
                    f"{side} wrist pivot artifact unavailable: {pivot_status}"
                )
            if wrist_to_palm_distance is None:
                self.get_logger().warning(
                    f"{side} wrist pivot artifact unavailable ({pivot_status}); "
                    "using paired-skeleton position fallback"
                )
            else:
                self.get_logger().info(
                    f"{side} wrist pivot artifact loaded: "
                    f"{Path(str(wrist_pivot_paths[side])).expanduser()} "
                    f"distance={wrist_to_palm_distance:.4f} m "
                    "axis=palm_local_positive_x"
                )
            self._side_state[side] = _SideState(
                calibration=(
                    SideCalibration(
                        upper_length_m=loaded_geometry[side].upper_arm_length_m,
                        forearm_length_m=loaded_geometry[side].forearm_length_m,
                        palm_to_wrist_offset_m=np.zeros(3, dtype=float),
                        palm_to_wrist_quat_xyzw=np.array(
                            [0.0, 0.0, 0.0, 1.0], dtype=float
                        ),
                    )
                    if loaded_geometry[side] is not None
                    else None
                ),
                positions=deque(maxlen=self._min_calibration_samples),
                orientations=deque(maxlen=self._min_calibration_samples),
                palm_positions=deque(maxlen=self._min_calibration_samples),
                palm_orientations=deque(maxlen=self._min_calibration_samples),
                sample_stamps_ns=deque(maxlen=self._min_calibration_samples),
                wrist_to_palm_distance_m=wrist_to_palm_distance,
                wrist_pivot_status=pivot_status,
            )
        self._side_status = {
            side: {
                "baseline_ready": False,
                "corrected": False,
                "fallback_reason": "baseline_collecting",
                "output_mode": "raw_smpl",
                "time_skew_ms": None,
                "reach_clamped": False,
                "wrist_palm_mode": (
                    "positive_x_distance"
                    if self._side_state[side].wrist_to_palm_distance_m is not None
                    else "paired_skeleton_position_fallback"
                ),
                "wrist_palm_axis": "palm_local_positive_x",
                "wrist_to_palm_distance_m": (
                    self._side_state[side].wrist_to_palm_distance_m
                ),
                "wrist_pivot_status": self._side_state[side].wrist_pivot_status,
                "wrist_orientation_mode": "palm_exact",
                "wrist_pivot_position_residual_m": None,
                "geometry_source": (
                    "quick_arm_artifact"
                    if loaded_geometry[side] is not None
                    else "raw_smpl_baseline"
                ),
                "geometry_revision": (
                    loaded_geometry[side].calibration_revision
                    if loaded_geometry[side] is not None
                    else 0
                ),
                "upper_length_m": (
                    loaded_geometry[side].upper_arm_length_m
                    if loaded_geometry[side] is not None
                    else None
                ),
                "upper_arm_length_m": (
                    loaded_geometry[side].upper_arm_length_m
                    if loaded_geometry[side] is not None
                    else None
                ),
                "forearm_length_m": (
                    loaded_geometry[side].forearm_length_m
                    if loaded_geometry[side] is not None
                    else None
                ),
            }
            for side in SIDE_INDICES
        }
        for side, geometry in loaded_geometry.items():
            if geometry is None:
                continue
            self._side_status[side]["baseline_ready"] = True
            self._side_status[side]["fallback_reason"] = ""
            self.get_logger().info(
                f"{side} quick geometry loaded: "
                f"upper={geometry.upper_arm_length_m:.3f} m "
                f"forearm={geometry.forearm_length_m:.3f} m "
                f"revision={geometry.calibration_revision}"
            )

        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._output_publisher = self.create_publisher(
            PoseArray, self._output_topic, qos_profile_sensor_data
        )
        self._ik_output_publisher = self.create_publisher(
            PoseArray, self._ik_output_topic, qos_profile_sensor_data
        )
        self._status_publisher = self.create_publisher(
            String, self._status_topic, status_qos
        )
        self._raw_subscription = self.create_subscription(
            PoseArray, self._raw_topic, self._raw_callback, qos_profile_sensor_data
        )
        epoch_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        epoch_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._tracking_epoch_subscription = self.create_subscription(
            UInt64,
            self._tracking_epoch_topic,
            self._tracking_epoch_callback,
            epoch_qos,
        )
        self._tracking_epoch_status_subscription = self.create_subscription(
            String,
            self._tracking_epoch_status_topic,
            self._tracking_epoch_status_callback,
            epoch_qos,
        )
        self._left_palm_subscription = self.create_subscription(
            PoseStamped,
            self._left_palm_topic,
            lambda message: self._palm_callback(message, "left"),
            qos_profile_sensor_data,
        )
        self._right_palm_subscription = self.create_subscription(
            PoseStamped,
            self._right_palm_topic,
            lambda message: self._palm_callback(message, "right"),
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "PICO palm skeleton filter ready: "
            f"raw={self._raw_topic} output={self._output_topic} "
            f"max_skew_s={self._max_skew_s:.3f} "
            f"min_calibration_samples={self._min_calibration_samples}"
        )

    def _palm_callback(self, message: PoseStamped, side: str) -> None:
        if not self._epoch_consistent():
            return
        try:
            sample = pose_stamped_to_sample(message)
        except ValueError as error:
            self.get_logger().warning(f"Rejecting {side} palm sample: {error}")
            return
        self._palm_cache[side].append(
            PalmSample(
                stamp_ns=sample.stamp_ns,
                position=sample.position,
                orientation_xyzw=sample.orientation_xyzw,
                frame_id=sample.frame_id,
                tracking_epoch=self._tracking_epoch,
            )
        )

    def _tracking_epoch_callback(self, message: UInt64) -> None:
        previous = self._tracking_epoch_numeric
        self._tracking_epoch_numeric = int(message.data)
        self._handle_epoch_transition(previous, self._tracking_epoch_numeric)

    def _tracking_epoch_status_callback(self, message: String) -> None:
        previous = self._tracking_epoch
        try:
            payload = json.loads(message.data)
            epoch = int(payload["tracking_epoch"])
            source = str(payload["tracking_epoch_source"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            self._tracking_epoch = 0
            self._tracking_epoch_source = "unknown"
            self._handle_epoch_transition(previous, 0)
            return
        self._tracking_epoch = epoch
        self._tracking_epoch_source = source
        self._handle_epoch_transition(previous, epoch)

    def _epoch_consistent(self) -> bool:
        return (
            self._tracking_epoch > 0
            and self._tracking_epoch_numeric == self._tracking_epoch
            and self._tracking_epoch_source in {"tcp_connection", "wire_world_reset"}
        )

    def _handle_epoch_transition(self, previous: int, current: int) -> None:
        if current == previous:
            return
        self._previous_ik_orientations_xyzw = None
        self._ik_frame_valid = False
        self._ik_frame_failure_reason = "tracking_epoch_transition"
        if previous <= 0:
            for state in getattr(self, "_side_state", {}).values():
                state.previous_corrected_positions = None
                state.previous_corrected_orientations_xyzw = None
            for status in getattr(self, "_side_status", {}).values():
                status["corrected"] = False
                status["fallback_reason"] = "tracking_epoch_transition"
                status["output_mode"] = "raw_smpl"
            return
        for cache in self._palm_cache.values():
            cache.clear()
        for state in getattr(self, "_side_state", {}).values():
            for name in (
                "positions",
                "orientations",
                "palm_positions",
                "palm_orientations",
                "sample_stamps_ns",
            ):
                values = getattr(state, name, None)
                if values is not None:
                    values.clear()
            state.previous_elbow_position = None
            state.previous_corrected_positions = None
            state.previous_corrected_orientations_xyzw = None
        for status in getattr(self, "_side_status", {}).values():
            status["corrected"] = False
            status["fallback_reason"] = "tracking_epoch_transition"
            status["output_mode"] = "raw_smpl"

    def _nearest_palm(
        self, stamp_ns: int, side: str, tracking_epoch: int | None = None
    ) -> PalmSample | None:
        candidates = [
            sample
            for sample in self._palm_cache[side]
            if sample.frame_id == "pico"
            and (
                tracking_epoch is None
                or sample.tracking_epoch == tracking_epoch
            )
        ]
        if not candidates:
            return None
        selected = min(candidates, key=lambda sample: abs(sample.stamp_ns - stamp_ns))
        if abs(selected.stamp_ns - stamp_ns) > self._max_skew_s * 1.0e9:
            return None
        return selected

    def _raw_callback(self, message: PoseArray) -> None:
        try:
            positions, orientations = pose_array_to_arrays(message)
        except ValueError as error:
            self.get_logger().warning(f"Rejecting raw PICO skeleton: {error}")
            return
        stamp_ns = stamp_to_ns(message.header.stamp)
        self._source_frame_id = message.header.frame_id
        output_positions = positions.copy()
        output_orientations = orientations.copy()
        for side in SIDE_INDICES:
            self._process_side(
                side,
                stamp_ns,
                positions,
                orientations,
                output_positions,
                output_orientations,
            )
        output = copy.deepcopy(message)
        for index in range(JOINT_COUNT):
            output.poses[index].position.x = float(output_positions[index, 0])
            output.poses[index].position.y = float(output_positions[index, 1])
            output.poses[index].position.z = float(output_positions[index, 2])
            output.poses[index].orientation.x = float(output_orientations[index, 0])
            output.poses[index].orientation.y = float(output_orientations[index, 1])
            output.poses[index].orientation.z = float(output_orientations[index, 2])
            output.poses[index].orientation.w = float(output_orientations[index, 3])
        self._output_publisher.publish(output)
        try:
            ik_output = pose_array_with_ik_arm_frames(
                output,
                left_offset_rad=self._left_shoulder_local_x_offset_rad,
                right_offset_rad=self._right_shoulder_local_x_offset_rad,
                previous_orientations_xyzw=self._previous_ik_orientations_xyzw,
            )
        # IK-frame conversion is a derived shadow output.  Any ordinary
        # per-frame adapter failure must remain isolated from the canonical
        # corrected skeleton stream, which has already been published above.
        # Do not catch BaseException: shutdown and keyboard interrupts must
        # still propagate normally.
        except Exception as error:  # noqa: BLE001
            failure_reason = str(error)
            if (
                self._ik_frame_valid
                or self._ik_frame_failure_reason != failure_reason
            ):
                self.get_logger().warning(
                    f"Skipping invalid IK-frame skeleton: {failure_reason}"
                )
            self._ik_frame_valid = False
            self._ik_frame_failure_reason = failure_reason
        else:
            _, self._previous_ik_orientations_xyzw = pose_array_to_arrays(ik_output)
            self._ik_frame_valid = True
            self._ik_frame_failure_reason = ""
            self._ik_output_publisher.publish(ik_output)
        self._publish_status(stamp_ns)

    def _process_side(
        self,
        side: str,
        stamp_ns: int,
        input_positions: np.ndarray,
        input_orientations: np.ndarray,
        output_positions: np.ndarray,
        output_orientations: np.ndarray,
    ) -> None:
        state = self._side_state[side]
        status = self._side_status[side]
        status["corrected"] = False
        status["output_mode"] = "raw_smpl"
        status["reach_clamped"] = False
        status["wrist_pivot_position_residual_m"] = None
        if not self._epoch_consistent():
            status["fallback_reason"] = "tracking_epoch_invalid"
            status["time_skew_ms"] = None
            return
        palm = self._nearest_palm(
            stamp_ns, side, tracking_epoch=self._tracking_epoch
        )
        if palm is None:
            if (
                state.calibration is not None
                and state.previous_corrected_positions is not None
                and state.previous_corrected_orientations_xyzw is not None
            ):
                indices = SIDE_INDICES[side]
                output_positions[list(indices)] = state.previous_corrected_positions
                output_orientations[list(indices)] = (
                    state.previous_corrected_orientations_xyzw
                )
                status["fallback_reason"] = "palm_stale_hold"
                status["output_mode"] = "hold_last_corrected"
            else:
                status["fallback_reason"] = (
                    "palm_stale"
                    if state.calibration is not None
                    else "baseline_collecting"
                )
            status["time_skew_ms"] = None
            return
        status["time_skew_ms"] = abs(palm.stamp_ns - stamp_ns) / 1.0e6
        if state.calibration is None:
            if state.sample_stamps_ns and state.sample_stamps_ns[-1] == palm.stamp_ns:
                status["fallback_reason"] = "baseline_collecting"
                return
            state.positions.append(input_positions.copy())
            state.orientations.append(input_orientations.copy())
            state.palm_positions.append(palm.position.copy())
            state.palm_orientations.append(palm.orientation_xyzw.copy())
            state.sample_stamps_ns.append(palm.stamp_ns)
            if len(state.sample_stamps_ns) < self._min_calibration_samples:
                status["fallback_reason"] = "baseline_collecting"
                return
            try:
                calibration = fit_side_calibration(
                    np.asarray(state.positions),
                    np.asarray(state.orientations),
                    np.asarray(state.palm_positions),
                    np.asarray(state.palm_orientations),
                    side,
                )
                if not 0.10 <= calibration.upper_length_m <= 0.50:
                    raise ValueError("upper arm length outside [0.10, 0.50] m")
                if not 0.10 <= calibration.forearm_length_m <= 0.50:
                    raise ValueError("forearm length outside [0.10, 0.50] m")
                state.calibration = calibration
                status["baseline_ready"] = True
                status["upper_length_m"] = float(calibration.upper_length_m)
                status["upper_arm_length_m"] = float(calibration.upper_length_m)
                status["forearm_length_m"] = float(calibration.forearm_length_m)
                self.get_logger().info(
                    f"{side} PICO arm baseline ready: "
                    f"upper={calibration.upper_length_m:.3f} m "
                    f"forearm={calibration.forearm_length_m:.3f} m"
                )
            except ValueError as error:
                status["fallback_reason"] = f"baseline_invalid:{error}"
                return

        try:
            result = correct_side(
                input_positions,
                input_orientations,
                palm.position,
                palm.orientation_xyzw,
                state.calibration,
                side,
                previous_elbow_position=state.previous_elbow_position,
                ratio_max_stretch=self._ratio_max_stretch,
                wrist_to_palm_distance_m=state.wrist_to_palm_distance_m,
            )
        except ValueError as error:
            status["fallback_reason"] = f"geometry_invalid:{error}"
            return
        indices = SIDE_INDICES[side]
        output_positions[list(indices)] = result.positions[list(indices)]
        output_orientations[list(indices)] = result.orientations_xyzw[list(indices)]
        state.previous_corrected_positions = result.positions[list(indices)].copy()
        state.previous_corrected_orientations_xyzw = (
            result.orientations_xyzw[list(indices)].copy()
        )
        state.previous_elbow_position = result.positions[indices[1]].copy()
        status["corrected"] = True
        status["output_mode"] = "corrected"
        status["fallback_reason"] = ""
        status["reach_clamped"] = bool(result.reach_clamped)
        status["wrist_pivot_position_residual_m"] = float(
            result.wrist_palm_position_residual_m
        )

    def _status_payload(self) -> dict:
        payload = {
            side: dict(values)
            for side, values in self._side_status.items()
        }
        source = getattr(self, "_tracking_epoch_source", "unknown")
        epoch = int(getattr(self, "_tracking_epoch", 0))
        numeric_epoch = int(getattr(self, "_tracking_epoch_numeric", epoch))
        source_frame_id = getattr(self, "_source_frame_id", "")
        payload.update({
            "tracking_epoch": epoch,
            "tracking_epoch_source": source,
            "stream_valid": (
                epoch > 0
                and numeric_epoch == epoch
                and source in {"tcp_connection", "wire_world_reset"}
                and source_frame_id == "pico"
            ),
            "source_frame_id": source_frame_id,
            "tcp_calibration_revision_left": self._tcp_calibration_revisions["left"],
            "tcp_calibration_revision_right": self._tcp_calibration_revisions["right"],
            "ratio_max_stretch": self._ratio_max_stretch,
            "ik_output_topic": getattr(
                self, "_ik_output_topic", "/pico/smpl_palm_corrected_ik"
            ),
            "ik_shoulder_frame_semantics": IK_SHOULDER_FRAME_SEMANTICS,
            "ik_elbow_frame_semantics": IK_ELBOW_FRAME_SEMANTICS,
            "ik_frame_valid": bool(getattr(self, "_ik_frame_valid", False)),
            "ik_frame_failure_reason": str(
                getattr(self, "_ik_frame_failure_reason", "not_received")
            ),
            "left_shoulder_local_x_offset_rad": float(
                getattr(
                    self,
                    "_left_shoulder_local_x_offset_rad",
                    DEFAULT_LEFT_SHOULDER_LOCAL_X_OFFSET_RAD,
                )
            ),
            "right_shoulder_local_x_offset_rad": float(
                getattr(
                    self,
                    "_right_shoulder_local_x_offset_rad",
                    DEFAULT_RIGHT_SHOULDER_LOCAL_X_OFFSET_RAD,
                )
            ),
        })
        return payload

    def _publish_status(self, stamp_ns: int) -> None:
        payload = self._status_payload()
        payload["source_stamp_ns"] = stamp_ns
        message = String()
        message.data = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        self._status_publisher.publish(message)


def main() -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init()
    node = PicoPalmSkeletonFilterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
