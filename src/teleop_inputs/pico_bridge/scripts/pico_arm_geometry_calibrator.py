#!/usr/bin/env python3
"""One-person, side-parameterized PICO arm geometry calibration."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import json
import os
from pathlib import Path
import sys
import tempfile
import termios
import time
from typing import Deque
import tty

import numpy as np
import yaml

from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String, UInt64

from pico_calibration_artifact import validate_artifact
from pico_arm_geometry_core import (
    ArmSide,
    ArmStaticPoseResult,
    MAX_NEUTRAL_PAIR_POSITION_ERROR_M,
    MAX_STATIC_TRANSITION_DIRECTION_ERROR_RAD,
    solve_arm_static_pose_geometry,
)
from pico_palm_skeleton_filter_core import JOINT_COUNT, quat_to_matrix
from pico_palm_skeleton_filter_node import (
    PalmSample,
    file_sha256,
    load_wrist_pivot_artifact,
    pose_stamped_to_sample,
    stamp_to_ns,
)


MIN_VALIDATION_SAMPLES = 50
STABLE_MAX_MOTION_RMS_M = 0.015


_REJECTION_HELP = {
    "neutral_pair_position_error_exceeded": (
        "开始和结束的自然站立位置相差太大；标定期间不要挪动身体，"
        "最后请回到开始时相同的位置和姿势"
    ),
    "static_transition_direction_error_exceeded": (
        "动作方向与要求不一致；伸直动作请向正前方平举，"
        "屈肘动作请让上臂贴身、前臂向前"
    ),
    "length_closure_error_exceeded": (
        "不同动作算出的臂长不一致；平举时请让肘部完全伸直，"
        "屈肘时保持约 90 度且身体不要移动"
    ),
    "upper_arm_length_out_of_range": (
        "算出的上臂长度超出合理范围；通常是肩部移动、平举方向错误或腕部标定不准"
    ),
    "forearm_length_out_of_range": (
        "算出的前臂长度超出合理范围；请检查腕部标定，并保持 90 度屈肘姿势"
    ),
    "static_pose_motion_exceeded": "静止阶段移动过多；听到“开始”后请保持完全静止",
    "straight_pair_position_error_exceeded": (
        "多次伸直平举的位置不一致；每次都请向正前方伸直到相同高度"
    ),
    "length_uncertainty_exceeded": "骨长结果波动过大；请放慢动作并在采集阶段保持静止",
}


def human_readable_rejection_reasons(reasons, *, quality=None) -> list[str]:
    messages = [
        _REJECTION_HELP.get(str(reason), f"未通过检查：{reason}")
        for reason in reasons
    ]
    if quality is None:
        return messages
    for index, reason in enumerate(reasons):
        if str(reason) != "static_transition_direction_error_exceeded":
            continue
        try:
            measured_degrees = float(
                np.rad2deg(quality["transition_direction_error_rad"])
            )
        except (KeyError, TypeError, ValueError):
            continue
        allowed_degrees = float(
            np.rad2deg(MAX_STATIC_TRANSITION_DIRECTION_ERROR_RAD)
        )
        messages[index] += (
            f"（本次 {measured_degrees:.1f}°，要求不超过 {allowed_degrees:.1f}°）"
        )
    return messages


@dataclass(frozen=True)
class CaptureSample:
    stamp_ns: int
    tracking_epoch: int
    shoulder_pico_m: np.ndarray
    raw_elbow_pico_m: np.ndarray
    raw_wrist_pico_m: np.ndarray
    palm_position_pico_m: np.ndarray
    palm_orientation_xyzw: np.ndarray
    wrist_pico_m: np.ndarray


@dataclass(frozen=True)
class PendingRawSample:
    stamp_ns: int
    tracking_epoch: int
    shoulder_pico_m: np.ndarray
    raw_elbow_pico_m: np.ndarray
    raw_wrist_pico_m: np.ndarray


@dataclass(frozen=True)
class StageDefinition:
    name: str
    group: str
    instruction: str
    duration_s: float


def _left_stage_sequence() -> tuple[StageDefinition, ...]:
    return (
        StageDefinition(
            "neutral_start",
            "neutral_start",
            "自然站立，左臂放松下垂并保持静止",
            3.0,
        ),
        StageDefinition(
            "straight_reach_1",
            "straight_1",
            "倒计时内将左臂伸直向前抬至近水平；开始后保持静止",
            6.0,
        ),
        StageDefinition(
            "straight_reach_2",
            "straight_2",
            "先放下，再在倒计时内将左臂伸直向前抬至近水平；开始后保持静止",
            6.0,
        ),
        StageDefinition(
            "elbow_right_angle",
            "right_angle",
            "倒计时内让左上臂下垂贴近身体、肘弯至约90度、前臂向前近水平；开始后保持静止",
            7.0,
        ),
        StageDefinition(
            "straight_validation",
            "validation",
            "倒计时内再次将左臂伸直向前抬至近水平；开始后保持静止用于独立验证",
            6.0,
        ),
        StageDefinition(
            "neutral_return",
            "neutral_return",
            "恢复自然站立，左臂放松下垂并保持静止",
            3.0,
        ),
    )


@dataclass(frozen=True)
class ArmIndices:
    shoulder: int
    elbow: int
    wrist: int

    @staticmethod
    def for_side(side: ArmSide | str) -> "ArmIndices":
        try:
            active_side = ArmSide(side)
        except ValueError as error:
            raise ValueError(f"arm_side_invalid:{side}") from error
        return {
            ArmSide.LEFT: ArmIndices(16, 18, 20),
            ArmSide.RIGHT: ArmIndices(17, 19, 21),
        }[active_side]


def default_palm_topic(side: ArmSide | str) -> str:
    try:
        active_side = ArmSide(side)
    except ValueError as error:
        raise ValueError(f"arm_side_invalid:{side}") from error
    return f"/pico/palm_{active_side.value}"


def artifact_type_for_side(side: ArmSide | str) -> str:
    try:
        active_side = ArmSide(side)
    except ValueError as error:
        raise ValueError(f"arm_side_invalid:{side}") from error
    return f"pico_{active_side.value}_arm_geometry_quick_v3"


def default_stage_sequence(
    side: ArmSide | str = ArmSide.LEFT,
) -> tuple[StageDefinition, ...]:
    try:
        active_side = ArmSide(side)
    except ValueError as error:
        raise ValueError(f"arm_side_invalid:{side}") from error
    side_label = "左" if active_side is ArmSide.LEFT else "右"
    return tuple(
        StageDefinition(
            stage.name,
            stage.group,
            stage.instruction.replace("左", side_label),
            stage.duration_s,
        )
        for stage in _left_stage_sequence()
    )


def wrist_from_palm(
    palm_position: np.ndarray,
    palm_orientation_xyzw: np.ndarray,
    wrist_to_palm_distance_m: float,
) -> np.ndarray:
    position = np.asarray(palm_position, dtype=float)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError("palm_position must be a finite 3-vector")
    distance = float(wrist_to_palm_distance_m)
    if not np.isfinite(distance) or distance <= 0.0:
        raise ValueError("wrist_to_palm_distance_m must be positive")
    rotation = quat_to_matrix(np.asarray(palm_orientation_xyzw, dtype=float))
    return position - rotation @ np.array([distance, 0.0, 0.0], dtype=float)


class ArmCaptureBuffer:
    def __init__(self) -> None:
        self._groups: dict[str, list[CaptureSample]] = {}
        self._stage_ranges: list[dict] = []
        self._active_name: str | None = None
        self._active_group: str | None = None
        self._active_epoch = 0
        self._active_start_wall_ns = 0
        self._active_group_start_index = 0
        self._last_stamp_ns: int | None = None

    def start_stage(self, name: str, group: str, tracking_epoch: int) -> None:
        if self._active_name is not None:
            raise RuntimeError("capture_stage_already_active")
        if tracking_epoch <= 0:
            raise RuntimeError("tracking_epoch_unavailable")
        self._active_name = name
        self._active_group = group
        self._active_epoch = int(tracking_epoch)
        self._active_start_wall_ns = time.time_ns()
        self._last_stamp_ns = None
        self._groups.setdefault(group, [])
        self._active_group_start_index = len(self._groups[group])

    def append(self, sample: CaptureSample) -> bool:
        if self._active_name is None or self._active_group is None:
            return False
        if sample.tracking_epoch != self._active_epoch:
            raise RuntimeError("tracking_epoch_changed_during_capture")
        if self._last_stamp_ns == sample.stamp_ns:
            return False
        if self._last_stamp_ns is not None and sample.stamp_ns < self._last_stamp_ns:
            raise RuntimeError("source_timestamp_out_of_order")
        self._groups[self._active_group].append(sample)
        self._last_stamp_ns = sample.stamp_ns
        return True

    def finish_stage(self, name: str) -> None:
        if name != self._active_name or self._active_group is None:
            raise RuntimeError("capture_stage_finish_mismatch")
        samples = self._groups[self._active_group]
        stage_samples = samples[self._active_group_start_index :]
        self._stage_ranges.append(
            {
                "name": name,
                "group": self._active_group,
                "tracking_epoch": self._active_epoch,
                "wall_start_ns": self._active_start_wall_ns,
                "wall_end_ns": time.time_ns(),
                "source_start_ns": stage_samples[0].stamp_ns if stage_samples else 0,
                "source_end_ns": stage_samples[-1].stamp_ns if stage_samples else 0,
                "accepted_sample_count": len(stage_samples),
            }
        )
        self._active_name = None
        self._active_group = None
        self._last_stamp_ns = None

    @property
    def stage_ranges(self) -> tuple[dict, ...]:
        return tuple(dict(value) for value in self._stage_ranges)

    @property
    def active_epoch(self) -> int | None:
        return self._active_epoch if self._active_name is not None else None

    def group_samples(self, group: str) -> tuple[CaptureSample, ...]:
        return tuple(self._groups.get(group, []))

    def archive_arrays(self) -> dict[str, np.ndarray]:
        output: dict[str, np.ndarray] = {}
        for group, samples in self._groups.items():
            output[f"{group}_stamp_ns"] = np.asarray(
                [sample.stamp_ns for sample in samples], dtype=np.int64
            )
            output[f"{group}_tracking_epoch"] = np.asarray(
                [sample.tracking_epoch for sample in samples], dtype=np.uint64
            )
            for field in (
                "shoulder_pico_m",
                "raw_elbow_pico_m",
                "raw_wrist_pico_m",
                "palm_position_pico_m",
                "palm_orientation_xyzw",
                "wrist_pico_m",
            ):
                width = 4 if field == "palm_orientation_xyzw" else 3
                output[f"{group}_{field}"] = np.asarray(
                    [getattr(sample, field) for sample in samples], dtype=float
                ).reshape((-1, width))
        return output


def _commit_new_file(temporary_name: str, path: Path) -> None:
    """Atomically create *path* without ever replacing prior evidence."""
    os.link(temporary_name, path)
    os.unlink(temporary_name)
    directory_fd = os.open(path.parent, os.O_RDONLY)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)


def _atomic_bytes(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        _commit_new_file(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except OSError:
            pass
        raise


def atomic_write_bundle(
    output_dir: str | Path,
    capture: ArmCaptureBuffer,
    candidate: dict,
    gate_report: dict,
) -> None:
    root = Path(output_dir).expanduser()
    root.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=".capture.", suffix=".npz", dir=root)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            np.savez_compressed(stream, **capture.archive_arrays())
            stream.flush()
            os.fsync(stream.fileno())
        _commit_new_file(temporary_name, root / "capture.npz")
    except BaseException:
        try:
            os.unlink(temporary_name)
        except OSError:
            pass
        raise
    _atomic_bytes(
        root / "stage_ranges.yaml",
        yaml.safe_dump(
            {"stages": list(capture.stage_ranges)}, sort_keys=False
        ).encode("utf-8"),
    )
    _atomic_bytes(
        root / f"pico_{candidate['side']}_arm_geometry_candidate.yaml",
        yaml.safe_dump(candidate, sort_keys=False).encode("utf-8"),
    )
    _atomic_bytes(
        root / "gate_report.json",
        json.dumps(gate_report, indent=2, sort_keys=True).encode("utf-8") + b"\n",
    )


def _upper_triangle(matrix: tuple[tuple[float, ...], ...]) -> list[float]:
    values = np.asarray(matrix, dtype=float)
    return [
        float(values[row, column])
        for row in range(values.shape[0])
        for column in range(row, values.shape[1])
    ]


def candidate_from_result(
    result: ArmStaticPoseResult,
    *,
    side: ArmSide | str,
    tcp_revision: int,
    tcp_artifact_hash: str,
    tcp_translation_revision: int,
    tcp_translation_fingerprint: str,
    wrist_pivot_hash: str,
    tracking_epoch: int,
    tracking_epoch_source: str,
    output_dir: Path,
) -> tuple[dict, dict]:
    try:
        active_side = ArmSide(side)
    except ValueError as error:
        raise ValueError(f"arm_side_invalid:{side}") from error
    rejection_reasons = list(result.rejection_reasons)
    if tracking_epoch <= 0:
        rejection_reasons.append("tracking_epoch_invalid")
    if tracking_epoch_source not in {"tcp_connection", "wire_world_reset"}:
        rejection_reasons.append("tracking_epoch_source_invalid")
    raw_gate_results = {
        "straight_1_sample_count": result.straight_1_sample_count >= 50,
        "straight_2_sample_count": result.straight_2_sample_count >= 50,
        "validation_sample_count": result.validation_sample_count >= 50,
        "neutral_start_sample_count": result.neutral_start_sample_count >= 50,
        "neutral_return_sample_count": result.neutral_return_sample_count >= 50,
        "right_angle_sample_count": result.right_angle_sample_count >= 50,
        "straight_pair_position_error": result.straight_pair_position_error_m <= 0.10,
        "upper_length_repeat_error": result.upper_repeat_error_m <= 0.06,
        "neutral_pair_position_error": (
            result.neutral_pair_position_error_m
            <= MAX_NEUTRAL_PAIR_POSITION_ERROR_M
        ),
        "forearm_length_repeat_error": result.forearm_repeat_error_m <= 0.04,
        "static_pose_motion": result.static_motion_rms_m <= 0.015,
        "transition_direction_error": (
            result.transition_direction_error_rad
            <= MAX_STATIC_TRANSITION_DIRECTION_ERROR_RAD
        ),
        "length_closure_error": result.length_closure_error_m <= 0.025,
        "length_uncertainty": result.length_std_m <= 0.030,
        "upper_arm_length": 0.15 <= result.upper_arm_length_m <= 0.45,
        "forearm_length": 0.15 <= result.forearm_length_m <= 0.40,
    }
    gate_results = {
        name: bool(value) for name, value in raw_gate_results.items()
    }
    valid = not rejection_reasons and all(gate_results.values())
    candidate = {
        "artifact_type": artifact_type_for_side(side),
        "schema_version": 3,
        "valid": valid,
        "candidate_status": "accepted" if valid else "rejected",
        "side": side.value,
        "calibration_revision": int(time.time_ns()),
        "shoulder_anchor_pico_m": list(result.shoulder_anchor_pico_m),
        "elbow_center_pico_m": list(result.elbow_center_pico_m),
        "upper_arm_length_m": result.upper_arm_length_m,
        "forearm_length_m": result.forearm_length_m,
        "covariance_tangent_order": [
            "shoulder_x_m", "shoulder_y_m", "shoulder_z_m",
            "elbow_x_m", "elbow_y_m", "elbow_z_m",
            "upper_arm_length_m", "forearm_length_m",
        ],
        "covariance_upper_triangle_8x8": _upper_triangle(result.covariance_8x8),
        "quality": {
            "length_std_m": result.length_std_m,
            "selected_straight_groups": list(result.selected_straight_groups),
            "discarded_straight_group": result.discarded_straight_group,
            "straight_pair_position_error_m": result.straight_pair_position_error_m,
            "upper_repeat_error_m": result.upper_repeat_error_m,
            "neutral_pair_position_error_m": result.neutral_pair_position_error_m,
            "forearm_repeat_error_m": result.forearm_repeat_error_m,
            "static_motion_rms_m": result.static_motion_rms_m,
            "right_angle_error_rad": result.right_angle_error_rad,
            "right_angle_error_role": "diagnostic_only",
            "raw_smpl_diagnostic_available": result.raw_smpl_diagnostic_available,
            "transition_direction_error_rad": result.transition_direction_error_rad,
            "length_closure_error_m": result.length_closure_error_m,
            "shoulder_motion_rms_m": result.shoulder_motion_rms_m,
            "straight_1_sample_count": result.straight_1_sample_count,
            "straight_2_sample_count": result.straight_2_sample_count,
            "validation_sample_count": result.validation_sample_count,
            "neutral_start_sample_count": result.neutral_start_sample_count,
            "neutral_return_sample_count": result.neutral_return_sample_count,
            "right_angle_sample_count": result.right_angle_sample_count,
        },
        "gate_results": gate_results,
        "tcp_calibration_revision": tcp_revision,
        "tcp_artifact_sha256": tcp_artifact_hash,
        "tcp_translation_revision": tcp_translation_revision,
        "tcp_translation_fingerprint_sha256": tcp_translation_fingerprint,
        "wrist_pivot_sha256": wrist_pivot_hash,
        "tracking_epoch": tracking_epoch,
        "tracking_epoch_source": tracking_epoch_source,
        "source_recording": str((output_dir / "capture.npz").resolve()),
        "rejection_reasons": rejection_reasons,
        "lineage": [
            "/pico/smpl_raw",
            default_palm_topic(active_side),
            "palm_local_positive_x_wrist",
            "shoulder_free_three_static_pose_geometry",
        ],
    }
    report = {
        "valid": valid,
        "candidate_status": candidate["candidate_status"],
        "rejection_reasons": rejection_reasons,
        "quality": candidate["quality"],
        "gate_results": gate_results,
    }
    return candidate, report


class PicoArmGeometryCalibrator(Node):
    def __init__(
        self,
        *,
        max_skew_s: float,
        side: ArmSide | str,
        wrist_to_palm_distance_m: float,
        raw_topic: str = "/pico/smpl_raw",
        palm_topic: str | None = None,
        tracking_epoch_topic: str = "/pico/tracking_epoch",
        tracking_epoch_status_topic: str = "/pico/tracking_epoch/status",
    ) -> None:
        try:
            self._side = ArmSide(side)
        except ValueError as error:
            raise ValueError(f"arm_side_invalid:{side}") from error
        self._indices = ArmIndices.for_side(self._side)
        super().__init__(f"pico_{self._side.value}_arm_geometry_calibrator")
        self._max_skew_ns = int(max_skew_s * 1.0e9)
        self._wrist_to_palm_distance_m = wrist_to_palm_distance_m
        self.capture = ArmCaptureBuffer()
        self._palm_cache: Deque[tuple[PalmSample, int]] = deque(maxlen=240)
        self._pending_raw: Deque[PendingRawSample] = deque(maxlen=240)
        self._tracking_epoch = 0
        self._tracking_epoch_numeric = 0
        self._tracking_epoch_source = "unknown"
        self._latest_raw_stamp_ns = 0
        self._latest_palm_stamp_ns = 0
        self._capture_error: RuntimeError | None = None

        epoch_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        epoch_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(PoseArray, raw_topic, self._raw_callback, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, palm_topic or default_palm_topic(self._side), self._palm_callback, qos_profile_sensor_data)
        self.create_subscription(UInt64, tracking_epoch_topic, self._epoch_callback, epoch_qos)
        self.create_subscription(String, tracking_epoch_status_topic, self._epoch_status_callback, epoch_qos)

    @property
    def tracking_epoch(self) -> int:
        return self._tracking_epoch

    @property
    def tracking_epoch_source(self) -> str:
        return self._tracking_epoch_source

    @property
    def ready(self) -> bool:
        return (
            self._latest_raw_stamp_ns > 0
            and self._latest_palm_stamp_ns > 0
            and self._tracking_epoch > 0
            and self._tracking_epoch_numeric == self._tracking_epoch
            and self._tracking_epoch_source in {"tcp_connection", "wire_world_reset"}
        )

    def raise_capture_error(self) -> None:
        if self._capture_error is not None:
            raise self._capture_error

    def _palm_callback(self, message: PoseStamped) -> None:
        try:
            sample = pose_stamped_to_sample(message)
        except ValueError:
            return
        if sample.frame_id != "pico" or not self._epoch_consistent():
            return
        self._latest_palm_stamp_ns = sample.stamp_ns
        self._palm_cache.append((sample, self._tracking_epoch))
        self._drain_pairs()

    def _epoch_callback(self, message: UInt64) -> None:
        previous = self._tracking_epoch_numeric
        self._tracking_epoch_numeric = int(message.data)
        self._handle_epoch_transition(previous, self._tracking_epoch_numeric)
        self._latch_epoch_mismatch()

    def _epoch_status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            previous = self._tracking_epoch
            self._tracking_epoch = int(payload["tracking_epoch"])
            self._tracking_epoch_source = str(payload["tracking_epoch_source"])
            self._handle_epoch_transition(previous, self._tracking_epoch)
            self._latch_epoch_mismatch()
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            self._tracking_epoch = 0
            self._tracking_epoch_source = "unknown"
            self._latch_epoch_mismatch()

    def _epoch_consistent(self) -> bool:
        return (
            self._tracking_epoch > 0
            and self._tracking_epoch_numeric == self._tracking_epoch
            and self._tracking_epoch_source in {"tcp_connection", "wire_world_reset"}
        )

    def _handle_epoch_transition(self, previous: int, current: int) -> None:
        if previous > 0 and current != previous:
            self._palm_cache.clear()
            self._pending_raw.clear()
            if self.capture.active_epoch is not None:
                self._capture_error = RuntimeError(
                    "tracking_epoch_changed_during_capture"
                )

    def _latch_epoch_mismatch(self) -> None:
        if (
            self.capture.active_epoch is not None
            and not self._epoch_consistent()
        ):
            self._capture_error = RuntimeError(
                "tracking_epoch_mismatch_during_capture"
            )

    def _nearest_palm(self, stamp_ns: int, epoch: int) -> PalmSample | None:
        if not self._palm_cache:
            return None
        candidates = [
            sample for sample, sample_epoch in self._palm_cache
            if sample_epoch == epoch
        ]
        if not candidates:
            return None
        sample = min(candidates, key=lambda value: abs(value.stamp_ns - stamp_ns))
        if abs(sample.stamp_ns - stamp_ns) > self._max_skew_ns:
            return None
        return sample

    def _drain_pairs(self) -> None:
        retained: Deque[PendingRawSample] = deque(maxlen=240)
        for raw in self._pending_raw:
            palm = self._nearest_palm(raw.stamp_ns, raw.tracking_epoch)
            if palm is None:
                if self._latest_palm_stamp_ns - raw.stamp_ns <= self._max_skew_ns:
                    retained.append(raw)
                continue
            try:
                self.capture.append(
                    CaptureSample(
                        stamp_ns=raw.stamp_ns,
                        tracking_epoch=raw.tracking_epoch,
                        shoulder_pico_m=raw.shoulder_pico_m,
                        raw_elbow_pico_m=raw.raw_elbow_pico_m,
                        raw_wrist_pico_m=raw.raw_wrist_pico_m,
                        palm_position_pico_m=np.array(palm.position, copy=True),
                        palm_orientation_xyzw=np.array(
                            palm.orientation_xyzw, copy=True
                        ),
                        wrist_pico_m=wrist_from_palm(
                            palm.position,
                            palm.orientation_xyzw,
                            self._wrist_to_palm_distance_m,
                        ),
                    )
                )
            except RuntimeError as error:
                self._capture_error = error
        self._pending_raw = retained

    def _raw_callback(self, message: PoseArray) -> None:
        if message.header.frame_id != "pico" or len(message.poses) != JOINT_COUNT:
            return
        stamp_ns = stamp_to_ns(message.header.stamp)
        self._latest_raw_stamp_ns = stamp_ns
        if not self._epoch_consistent():
            self._latch_epoch_mismatch()
            return
        shoulder_pose = message.poses[self._indices.shoulder]
        elbow_pose = message.poses[self._indices.elbow]
        wrist_pose = message.poses[self._indices.wrist]
        shoulder = np.asarray(
            [shoulder_pose.position.x, shoulder_pose.position.y, shoulder_pose.position.z]
        )
        elbow = np.asarray(
            [elbow_pose.position.x, elbow_pose.position.y, elbow_pose.position.z]
        )
        wrist = np.asarray(
            [wrist_pose.position.x, wrist_pose.position.y, wrist_pose.position.z]
        )
        if not all(np.all(np.isfinite(value)) for value in (shoulder, elbow, wrist)):
            return
        self._pending_raw.append(
            PendingRawSample(
                stamp_ns, self._tracking_epoch, shoulder, elbow, wrist
            )
        )
        self._drain_pairs()


def _spin_for(node: PicoArmGeometryCalibrator, duration_s: float) -> None:
    import rclpy

    deadline = time.monotonic() + duration_s
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.02, max(deadline - time.monotonic(), 0.0)))
        node.raise_capture_error()


def _countdown(node: PicoArmGeometryCalibrator, seconds: int = 3) -> None:
    for remaining in range(seconds, 0, -1):
        print(f"开始倒计时: {remaining}", flush=True)
        _spin_for(node, 1.0)


def _wait_for_space() -> None:
    """Wait for one Space key while always restoring the caller's terminal."""
    if not sys.stdin.isatty():
        raise RuntimeError("space_start_requires_interactive_tty")
    descriptor = sys.stdin.fileno()
    original = termios.tcgetattr(descriptor)
    try:
        tty.setcbreak(descriptor)
        while True:
            key = sys.stdin.read(1)
            if key == " ":
                print(flush=True)
                return
            if key.lower() == "q":
                raise KeyboardInterrupt
    finally:
        termios.tcsetattr(descriptor, termios.TCSADRAIN, original)


def _array(samples: tuple[CaptureSample, ...], field: str) -> np.ndarray:
    return np.asarray([getattr(sample, field) for sample in samples], dtype=float)


def stable_suffix(samples: tuple[CaptureSample, ...]) -> tuple[CaptureSample, ...]:
    """Return the longest sufficiently static suffix of a prompted pose.

    A participant may still be settling after ``开始``.  A fixed fraction can
    therefore include motion even when the final pose is steady.  The search
    remains fail-closed: if no 50-sample suffix satisfies the motion limit,
    the last 50 samples are returned and the solver's static-motion gate will
    reject them.
    """

    if not samples:
        return ()
    minimum = min(MIN_VALIDATION_SAMPLES, len(samples))
    wrists = np.asarray([sample.wrist_pico_m for sample in samples], dtype=float)
    for count in range(len(samples), minimum - 1, -1):
        suffix = wrists[-count:]
        center = np.median(suffix, axis=0)
        motion = float(np.sqrt(np.mean(np.sum((suffix - center) ** 2, axis=1))))
        if motion <= STABLE_MAX_MOTION_RMS_M:
            return samples[-count:]
    return samples[-minimum:]


def _parse_arguments(default_side: str | None = None) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--side", choices=("left", "right"), default=default_side,
        required=default_side is None,
    )
    parser.add_argument("--tcp-artifact", "--left-tcp-artifact", dest="tcp_artifact", required=True)
    parser.add_argument(
        "--wrist-pivot-artifact", "--left-wrist-pivot-artifact",
        dest="wrist_pivot_artifact", required=True,
    )
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--max-skew-s", type=float, default=0.03)
    parser.add_argument("--readiness-timeout-s", type=float, default=20.0)
    parser.add_argument("--countdown-s", type=int, default=3)
    parser.add_argument("--duration-scale", type=float, default=1.0)
    parser.add_argument("--start-mode", choices=("space", "auto"), default="space")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--raw-topic", default="/pico/smpl_raw")
    parser.add_argument("--palm-topic", default="")
    parser.add_argument("--tracking-epoch-topic", default="/pico/tracking_epoch")
    parser.add_argument(
        "--tracking-epoch-status-topic", default="/pico/tracking_epoch/status"
    )
    return parser.parse_known_args()


def main_for_side(default_side: str | None = None) -> None:
    import rclpy

    arguments, ros_arguments = _parse_arguments(default_side)
    if arguments.duration_scale <= 0.0:
        raise SystemExit("--duration-scale must be positive")
    side = ArmSide(arguments.side)
    tcp_summary = validate_artifact(arguments.tcp_artifact, "tcp", side.value)
    tcp_revision = tcp_summary.calibration_revision
    tcp_hash = file_sha256(arguments.tcp_artifact)
    wrist_distance, wrist_status = load_wrist_pivot_artifact(
        arguments.wrist_pivot_artifact, side.value
    )
    if wrist_distance is None:
        raise SystemExit(f"{side.value} wrist pivot artifact invalid: {wrist_status}")
    wrist_hash = file_sha256(arguments.wrist_pivot_artifact)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    default_run_id = f"{timestamp}_{os.getpid()}-{time.time_ns() % 1_000_000_000:09d}"
    output_dir = Path(
        arguments.output_dir
        or f"recordings/pico_{side.value}_arm_geometry_{default_run_id}"
    ).expanduser()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise SystemExit(f"output directory is not empty: {output_dir}")

    rclpy.init(args=ros_arguments)
    node = PicoArmGeometryCalibrator(
        side=side,
        max_skew_s=arguments.max_skew_s,
        wrist_to_palm_distance_m=wrist_distance,
        raw_topic=arguments.raw_topic,
        palm_topic=arguments.palm_topic or default_palm_topic(side),
        tracking_epoch_topic=arguments.tracking_epoch_topic,
        tracking_epoch_status_topic=arguments.tracking_epoch_status_topic,
    )
    candidate = {
        "artifact_type": artifact_type_for_side(side),
        "schema_version": 3,
        "valid": False,
        "candidate_status": "rejected",
        "side": side.value,
        "rejection_reasons": ["capture_incomplete"],
    }
    report = {"valid": False, "rejection_reasons": ["capture_incomplete"]}
    try:
        deadline = time.monotonic() + arguments.readiness_timeout_s
        while rclpy.ok() and not node.ready and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not node.ready:
            raise RuntimeError("required PICO raw/palm/tracking epoch unavailable")
        side_label = "左" if side is ArmSide.LEFT else "右"
        if arguments.start_mode == "space":
            print(
                f"预检通过：只标定{side_label}臂。摆好后按空格开始，q 取消；开始后无需操作终端。",
                flush=True,
            )
            _wait_for_space()
        else:
            print(f"预检通过：3秒后自动开始{side_label}臂标定。", flush=True)
            _countdown(node, 3)
        capture_epoch = node.tracking_epoch
        for stage in default_stage_sequence(side):
            print(flush=True)
            print(stage.instruction, flush=True)
            print(flush=True)
            _countdown(node, arguments.countdown_s)
            print(f"{stage.name}: 开始", flush=True)
            node.capture.start_stage(stage.name, stage.group, capture_epoch)
            _spin_for(node, stage.duration_s * arguments.duration_scale)
            node.capture.finish_stage(stage.name)
            accepted = node.capture.stage_ranges[-1]["accepted_sample_count"]
            print(f"{stage.name}: 结束，accepted={accepted}", flush=True)
            print(flush=True)

        neutral_start = stable_suffix(node.capture.group_samples("neutral_start"))
        neutral_return = stable_suffix(node.capture.group_samples("neutral_return"))
        straight_1 = stable_suffix(node.capture.group_samples("straight_1"))
        straight_2 = stable_suffix(node.capture.group_samples("straight_2"))
        right_angle = stable_suffix(node.capture.group_samples("right_angle"))
        validation = stable_suffix(node.capture.group_samples("validation"))
        result = solve_arm_static_pose_geometry(
            _array(neutral_start, "wrist_pico_m"),
            _array(neutral_return, "wrist_pico_m"),
            _array(straight_1, "wrist_pico_m"),
            _array(straight_2, "wrist_pico_m"),
            _array(validation, "wrist_pico_m"),
            _array(right_angle, "wrist_pico_m"),
            _array(right_angle, "shoulder_pico_m"),
            _array(right_angle, "raw_elbow_pico_m"),
            _array(right_angle, "raw_wrist_pico_m"),
            side=side,
        )
        candidate, report = candidate_from_result(
            result,
            side=side,
            tcp_revision=tcp_revision,
            tcp_artifact_hash=tcp_hash,
            tcp_translation_revision=tcp_summary.translation_revision,
            tcp_translation_fingerprint=tcp_summary.translation_fingerprint_sha256,
            wrist_pivot_hash=wrist_hash,
            tracking_epoch=capture_epoch,
            tracking_epoch_source=node.tracking_epoch_source,
            output_dir=output_dir,
        )
    except (EOFError, KeyboardInterrupt, RuntimeError, ValueError) as error:
        candidate["rejection_reasons"] = [str(error)]
        report = {"valid": False, "rejection_reasons": [str(error)]}
    finally:
        try:
            atomic_write_bundle(output_dir, node.capture, candidate, report)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    capture_path = output_dir / "capture.npz"
    candidate_path = output_dir / f"pico_{side.value}_arm_geometry_candidate.yaml"
    if report.get("valid", False):
        print("\n✓ 上臂/前臂骨长标定成功")
        print(f"  上臂长度: {candidate['upper_arm_length_m']:.3f} m")
        print(f"  前臂长度: {candidate['forearm_length_m']:.3f} m")
    else:
        print("\n✗ 上臂/前臂骨长标定未通过")
        print("  请按下面提示调整后重新录制：")
        for message in human_readable_rejection_reasons(
            report.get("rejection_reasons", []), quality=report.get("quality")
        ):
            print(f"  - {message}")
    print(f"  采集数据: {capture_path}")
    print(f"  标定候选: {candidate_path}")
    if not report.get("valid", False):
        print("  当前有效配置未被修改")
    if arguments.verbose:
        print("  详细诊断:")
        print(json.dumps(report, ensure_ascii=False, sort_keys=True))
    if not report.get("valid", False):
        raise SystemExit(2)


def main() -> None:
    main_for_side()


if __name__ == "__main__":
    main()
