#!/usr/bin/env python3
"""Record and audit the PICO palm-constrained M0 skeleton streams."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
from typing import Any

import numpy as np

from pico_palm_skeleton_filter_core import SideCalibration, correct_side


REQUIRED_STREAMS = (
    "/pico/smpl_raw",
    "/pico/palm_left",
    "/pico/palm_right",
    "/pico/smpl_palm_corrected",
    "/pico/smpl_palm_corrected/status",
    "/pico/tracking_epoch",
    "/pico/tracking_epoch/status",
)

REQUIRED_ARRAYS = (
    "raw_pose",
    "raw_timestamp_ns",
    "corrected_pose",
    "corrected_timestamp_ns",
    "left_palm_pose",
    "left_palm_timestamp_ns",
    "right_palm_pose",
    "right_palm_timestamp_ns",
    "status_json",
    "tracking_epoch",
    "tracking_epoch_status_json",
)

SIDE_INDICES = {
    "left": (16, 18, 20, 22),
    "right": (17, 19, 21, 23),
}


def _load_capture(path: str | Path) -> dict[str, np.ndarray]:
    capture_path = Path(path).expanduser().resolve()
    if not capture_path.is_file():
        raise ValueError(f"capture_missing:{capture_path}")
    try:
        with np.load(capture_path, allow_pickle=False) as archive:
            missing = [name for name in REQUIRED_ARRAYS if name not in archive.files]
            if missing:
                raise ValueError(f"required_stream_missing:{','.join(missing)}")
            arrays = {name: np.asarray(archive[name]) for name in REQUIRED_ARRAYS}
    except OSError as error:
        raise ValueError(f"capture_unreadable:{error}") from error
    lengths = {name: int(value.shape[0]) for name, value in arrays.items()}
    if len(set(lengths.values())) != 1:
        raise ValueError(f"stream_length_mismatch:{json.dumps(lengths, sort_keys=True)}")
    if not lengths or next(iter(lengths.values())) < 2:
        raise ValueError("capture_sample_count_insufficient")
    for name in ("raw_pose", "corrected_pose"):
        if arrays[name].shape[1:] != (24, 7):
            raise ValueError(f"pose_shape_invalid:{name}:{arrays[name].shape}")
    for name in ("left_palm_pose", "right_palm_pose"):
        if arrays[name].shape[1:] != (7,):
            raise ValueError(f"pose_shape_invalid:{name}:{arrays[name].shape}")
    for name, value in arrays.items():
        if value.dtype.kind in "f" and not np.all(np.isfinite(value)):
            raise ValueError(f"non_finite_stream:{name}")
    return arrays


def _parse_json_stream(values: np.ndarray, name: str) -> list[dict[str, Any]]:
    result = []
    for index, value in enumerate(values):
        try:
            item = json.loads(str(value))
        except (json.JSONDecodeError, TypeError) as error:
            raise ValueError(f"json_stream_invalid:{name}:{index}") from error
        if not isinstance(item, dict):
            raise ValueError(f"json_stream_invalid:{name}:{index}")
        result.append(item)
    return result


def _quat_angle(first: np.ndarray, second: np.ndarray) -> float:
    first_norm = float(np.linalg.norm(first))
    second_norm = float(np.linalg.norm(second))
    if first_norm < 1.0e-12 or second_norm < 1.0e-12:
        raise ValueError("quaternion_norm_invalid")
    dot = abs(float(np.dot(first / first_norm, second / second_norm)))
    return 2.0 * math.acos(float(np.clip(dot, -1.0, 1.0)))


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=float), percentile))


def _validate_geometry_context(source: str, revision: int, side: str) -> None:
    valid = (source == "quick_arm_artifact" and revision > 0) or (
        source == "raw_smpl_baseline" and revision == 0
    )
    if not valid:
        raise ValueError(f"geometry_context_invalid:{side}:{source}:{revision}")


def _side_report(
    side: str,
    raw: np.ndarray,
    corrected: np.ndarray,
    palm: np.ndarray,
    statuses: list[dict[str, Any]],
) -> dict[str, Any]:
    side_statuses: list[dict[str, Any]] = []
    contexts: set[tuple[str, int]] = set()
    for index, status in enumerate(statuses):
        try:
            item = status[side]
            source = str(item["geometry_source"])
            revision = int(item["geometry_revision"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"status_geometry_missing:{side}:{index}") from error
        _validate_geometry_context(source, revision, side)
        contexts.add((source, revision))
        side_statuses.append(item)
    if len(contexts) != 1:
        raise ValueError(f"geometry_context_changed:{side}")
    source, revision = next(iter(contexts))
    shoulder, elbow, wrist, hand = SIDE_INDICES[side]
    corrected_flags = [bool(item.get("corrected", False)) for item in side_statuses]
    clamped_flags = [bool(item.get("reach_clamped", False)) for item in side_statuses]
    upper_residuals: list[float] = []
    forearm_residuals: list[float] = []
    endpoint_position_residuals: list[float] = []
    endpoint_orientation_residuals: list[float] = []
    raw_corrected_endpoint: list[float] = []
    elbow_branch_discontinuities = 0
    previous_elbow_direction: np.ndarray | None = None
    for index, item in enumerate(side_statuses):
        upper_length = item.get("upper_arm_length_m", item.get("upper_length_m"))
        forearm_length = item.get("forearm_length_m")
        if upper_length is not None:
            upper_residuals.append(
                abs(
                    float(np.linalg.norm(corrected[index, elbow, :3] - corrected[index, shoulder, :3]))
                    - float(upper_length)
                )
            )
        if forearm_length is not None:
            forearm_residuals.append(
                abs(
                    float(np.linalg.norm(corrected[index, wrist, :3] - corrected[index, elbow, :3]))
                    - float(forearm_length)
                )
            )
        endpoint_position_residuals.append(
            float(np.linalg.norm(corrected[index, hand, :3] - palm[index, :3]))
        )
        endpoint_orientation_residuals.append(
            _quat_angle(corrected[index, hand, 3:7], palm[index, 3:7])
        )
        raw_corrected_endpoint.append(
            float(np.linalg.norm(corrected[index, hand, :3] - raw[index, hand, :3]))
        )
        elbow_direction = corrected[index, elbow, :3] - corrected[index, shoulder, :3]
        norm = float(np.linalg.norm(elbow_direction))
        if norm > 1.0e-9:
            elbow_direction = elbow_direction / norm
            if previous_elbow_direction is not None:
                angle = math.acos(
                    float(np.clip(np.dot(previous_elbow_direction, elbow_direction), -1.0, 1.0))
                )
                if angle > 0.8:
                    elbow_branch_discontinuities += 1
            previous_elbow_direction = elbow_direction
    count = len(side_statuses)
    return {
        "geometry_source": source,
        "geometry_revision": revision,
        "geometry_context_stable": True,
        "corrected_coverage": float(sum(corrected_flags) / count),
        "fallback_coverage": float((count - sum(corrected_flags)) / count),
        "reach_clamp_rate": float(sum(clamped_flags) / count),
        "upper_length_residual_p95_m": _percentile(upper_residuals, 95.0),
        "forearm_length_residual_p95_m": _percentile(forearm_residuals, 95.0),
        "palm_endpoint_position_residual_p95_m": _percentile(
            endpoint_position_residuals, 95.0
        ),
        "palm_endpoint_orientation_residual_p95_rad": _percentile(
            endpoint_orientation_residuals, 95.0
        ),
        "raw_corrected_endpoint_difference_p95_m": _percentile(
            raw_corrected_endpoint, 95.0
        ),
        "elbow_branch_discontinuities": elbow_branch_discontinuities,
        "fallback_reasons": sorted(
            {str(item.get("fallback_reason", "")) for item in side_statuses if item.get("fallback_reason")}
        ),
    }


def _replay_corrected_skeleton(
    raw: np.ndarray,
    left_palms: np.ndarray,
    right_palms: np.ndarray,
    statuses: list[dict[str, Any]],
) -> np.ndarray:
    """Deterministically rerun the pure M0 arm geometry from recorded inputs."""
    if len(statuses) != raw.shape[0]:
        raise ValueError("replay_stream_length_mismatch")
    ratios = set()
    for index, status in enumerate(statuses):
        try:
            ratio = float(status["ratio_max_stretch"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"replay_ratio_missing:{index}") from error
        if not math.isfinite(ratio) or ratio <= 0.0:
            raise ValueError(f"replay_ratio_invalid:{index}")
        ratios.add(ratio)
    if len(ratios) != 1:
        raise ValueError("replay_ratio_changed")
    ratio = next(iter(ratios))

    replay = raw.copy()
    previous_elbows: dict[str, np.ndarray | None] = {"left": None, "right": None}
    palms = {"left": left_palms, "right": right_palms}
    for frame_index, status in enumerate(statuses):
        raw_positions = raw[frame_index, :, :3]
        raw_orientations = raw[frame_index, :, 3:7]
        for side in ("left", "right"):
            side_status = status[side]
            indices = SIDE_INDICES[side]
            if not bool(side_status.get("corrected", False)):
                # Runtime starts from an exact raw copy on every frame.
                replay[frame_index, list(indices)] = raw[frame_index, list(indices)]
                continue
            try:
                upper = float(side_status["upper_arm_length_m"])
                forearm = float(side_status["forearm_length_m"])
                wrist_distance = float(side_status["wrist_to_palm_distance_m"])
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(
                    f"replay_geometry_missing:{side}:{frame_index}"
                ) from error
            if not all(math.isfinite(value) and value > 0.0 for value in (
                upper, forearm, wrist_distance
            )):
                raise ValueError(f"replay_geometry_invalid:{side}:{frame_index}")
            calibration = SideCalibration(
                upper_length_m=upper,
                forearm_length_m=forearm,
                palm_to_wrist_offset_m=np.zeros(3, dtype=np.float64),
                palm_to_wrist_quat_xyzw=np.asarray(
                    [0.0, 0.0, 0.0, 1.0], dtype=np.float64
                ),
            )
            palm = palms[side][frame_index]
            result = correct_side(
                raw_positions,
                raw_orientations,
                palm[:3],
                palm[3:7],
                calibration,
                side,
                previous_elbow_position=previous_elbows[side],
                ratio_max_stretch=ratio,
                wrist_to_palm_distance_m=wrist_distance,
            )
            replay[frame_index, list(indices), :3] = result.positions[list(indices)]
            replay[frame_index, list(indices), 3:7] = result.orientations_xyzw[list(indices)]
            previous_elbows[side] = result.positions[indices[1]].copy()
    return replay


def build_report(path: str | Path) -> dict[str, Any]:
    arrays = _load_capture(path)
    statuses = _parse_json_stream(arrays["status_json"], "status")
    epoch_statuses = _parse_json_stream(
        arrays["tracking_epoch_status_json"], "tracking_epoch_status"
    )
    status_epochs = {int(item.get("tracking_epoch", 0)) for item in statuses}
    numeric_epochs = {int(value) for value in arrays["tracking_epoch"]}
    explicit_epochs = {int(item.get("tracking_epoch", 0)) for item in epoch_statuses}
    if len(status_epochs | numeric_epochs | explicit_epochs) != 1:
        raise ValueError("tracking_epoch_changed")
    epoch = next(iter(status_epochs | numeric_epochs | explicit_epochs))
    if epoch <= 0:
        raise ValueError("tracking_epoch_invalid")
    if not all(bool(item.get("stream_valid", False)) for item in statuses):
        raise ValueError("status_stream_invalid")
    stamps = arrays["corrected_timestamp_ns"].astype(np.int64)
    gaps = np.diff(stamps).astype(np.float64) / 1.0e9
    monotonic = bool(np.all(gaps >= 0.0))
    if not monotonic:
        raise ValueError("corrected_timestamp_non_monotonic")
    positive_gaps = gaps[gaps > 0.0]
    frequency = float(1.0 / np.median(positive_gaps)) if positive_gaps.size else 0.0
    left_skew = np.abs(stamps - arrays["left_palm_timestamp_ns"].astype(np.int64))
    right_skew = np.abs(stamps - arrays["right_palm_timestamp_ns"].astype(np.int64))
    replay_first = _replay_corrected_skeleton(
        arrays["raw_pose"], arrays["left_palm_pose"], arrays["right_palm_pose"], statuses
    )
    replay_second = _replay_corrected_skeleton(
        arrays["raw_pose"], arrays["left_palm_pose"], arrays["right_palm_pose"], statuses
    )
    deterministic_difference = float(np.max(np.abs(replay_first - replay_second)))
    recorded_difference = float(np.max(np.abs(replay_first - arrays["corrected_pose"])))
    if deterministic_difference > 1.0e-10:
        raise ValueError(
            f"deterministic_replay_mismatch:{deterministic_difference:.17g}"
        )
    if recorded_difference > 1.0e-10:
        raise ValueError(f"recorded_replay_mismatch:{recorded_difference:.17g}")
    report = {
        "schema_version": 1,
        "valid": True,
        "sample_count": int(stamps.size),
        "tracking_epoch": epoch,
        "timestamp_monotonic": monotonic,
        "output_frequency_hz": frequency,
        "maximum_gap_s": float(np.max(gaps)) if gaps.size else 0.0,
        "maximum_pairing_skew_s": float(max(np.max(left_skew), np.max(right_skew)) / 1.0e9),
        "left": _side_report(
            "left",
            arrays["raw_pose"],
            arrays["corrected_pose"],
            arrays["left_palm_pose"],
            statuses,
        ),
        "right": _side_report(
            "right",
            arrays["raw_pose"],
            arrays["corrected_pose"],
            arrays["right_palm_pose"],
            statuses,
        ),
        "deterministic_replay_max_difference": deterministic_difference,
        "recorded_replay_max_difference": recorded_difference,
    }
    return report


def _stamp_ns(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def _pose_array(message: Any) -> np.ndarray:
    if len(message.poses) != 24:
        raise ValueError(f"pose_count_invalid:{len(message.poses)}")
    return np.asarray(
        [
            (
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
            for pose in message.poses
        ],
        dtype=np.float64,
    )


def _pose_stamped(message: Any) -> np.ndarray:
    pose = message.pose
    return np.asarray(
        (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
        dtype=np.float64,
    )


def record_capture(output: str | Path) -> None:
    import rclpy
    from geometry_msgs.msg import PoseArray, PoseStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from std_msgs.msg import String, UInt64

    class Recorder(Node):
        def __init__(self) -> None:
            super().__init__("pico_m0_comparison_recorder")
            self.raw: dict[int, np.ndarray] = {}
            self.corrected: dict[int, np.ndarray] = {}
            self.palms: dict[str, tuple[int, np.ndarray] | None] = {"left": None, "right": None}
            self.epoch = 0
            self.epoch_status = "{}"
            self.rows: list[dict[str, Any]] = []
            self.create_subscription(PoseArray, REQUIRED_STREAMS[0], self.on_raw, qos_profile_sensor_data)
            self.create_subscription(PoseStamped, REQUIRED_STREAMS[1], lambda msg: self.on_palm("left", msg), qos_profile_sensor_data)
            self.create_subscription(PoseStamped, REQUIRED_STREAMS[2], lambda msg: self.on_palm("right", msg), qos_profile_sensor_data)
            self.create_subscription(PoseArray, REQUIRED_STREAMS[3], self.on_corrected, qos_profile_sensor_data)
            reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(String, REQUIRED_STREAMS[4], self.on_status, reliable)
            transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
            transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(UInt64, REQUIRED_STREAMS[5], lambda msg: setattr(self, "epoch", int(msg.data)), transient)
            self.create_subscription(String, REQUIRED_STREAMS[6], lambda msg: setattr(self, "epoch_status", msg.data), transient)

        def on_raw(self, message: Any) -> None:
            stamp = _stamp_ns(message)
            self.raw[stamp] = _pose_array(message)
            self._trim(self.raw)

        def on_corrected(self, message: Any) -> None:
            stamp = _stamp_ns(message)
            self.corrected[stamp] = _pose_array(message)
            self._trim(self.corrected)

        def on_palm(self, side: str, message: Any) -> None:
            self.palms[side] = (_stamp_ns(message), _pose_stamped(message))

        @staticmethod
        def _trim(cache: dict[int, np.ndarray]) -> None:
            while len(cache) > 120:
                del cache[min(cache)]

        def on_status(self, message: Any) -> None:
            try:
                payload = json.loads(message.data)
                stamp = int(payload["source_stamp_ns"])
                raw = self.raw[stamp]
                corrected = self.corrected[stamp]
                left = self.palms["left"]
                right = self.palms["right"]
                if left is None or right is None or self.epoch <= 0:
                    return
            except (json.JSONDecodeError, KeyError, TypeError, ValueError):
                return
            self.rows.append(
                {
                    "raw_pose": raw.copy(),
                    "raw_timestamp_ns": stamp,
                    "corrected_pose": corrected.copy(),
                    "corrected_timestamp_ns": stamp,
                    "left_palm_pose": left[1].copy(),
                    "left_palm_timestamp_ns": left[0],
                    "right_palm_pose": right[1].copy(),
                    "right_palm_timestamp_ns": right[0],
                    "status_json": message.data,
                    "tracking_epoch": self.epoch,
                    "tracking_epoch_status_json": self.epoch_status,
                }
            )

    output_path = Path(output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_path.exists():
        raise SystemExit(f"capture output already exists: {output_path}")
    rclpy.init()
    recorder = Recorder()
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        rclpy.shutdown()
    if len(recorder.rows) < 2:
        raise SystemExit("capture_sample_count_insufficient")
    temporary = output_path.with_name(f".{output_path.name}.incomplete-{os.getpid()}")
    arrays = {
        key: np.asarray([row[key] for row in recorder.rows]) for key in REQUIRED_ARRAYS
    }
    with temporary.open("wb") as stream:
        np.savez_compressed(stream, **arrays)
    os.replace(temporary, output_path)
    print(f"PICO M0 capture: {output_path}")


def main() -> None:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    record_parser = subparsers.add_parser("record")
    record_parser.add_argument("--output", required=True)
    report_parser = subparsers.add_parser("report")
    report_parser.add_argument("capture")
    report_parser.add_argument("--output")
    arguments = parser.parse_args()
    if arguments.command == "record":
        record_capture(arguments.output)
        return
    report = build_report(arguments.capture)
    serialized = json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if arguments.output:
        output = Path(arguments.output).expanduser().resolve()
        output.write_text(serialized, encoding="utf-8")
        print(output)
    else:
        print(serialized, end="")


if __name__ == "__main__":
    main()
