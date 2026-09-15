#!/usr/bin/env python3
"""Publish one calibrated palm and own its orientation-only recalibration."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import tempfile
import threading
import time

import numpy as np
import yaml

from pico_calibration_artifact import validate_artifact
from pico_palm_orientation_core import (
    OrientationCaptureBuffer,
    OrientationGates,
    atomic_replace_if_unchanged,
    build_orientation_only_update,
    file_sha256,
    solve_orientation,
)
from pico_palm_tcp_runtime import (
    TcpTransform,
    apply_tcp_transform,
    load_tcp_transform,
    quaternion_to_matrix,
)


def default_artifact(side: str) -> Path:
    return Path(f"~/.config/pico_tracker/pico_{side}_palm_tcp.yaml").expanduser()


def build_palm_message(controller_message, transform: TcpTransform, pose_stamped_type):
    """Apply one TCP transform while preserving the controller source stamp."""

    if controller_message.header.frame_id != "pico":
        raise ValueError(
            "controller pose frame must be pico, "
            f"got {controller_message.header.frame_id!r}"
        )
    controller_position = np.array(
        [
            controller_message.pose.position.x,
            controller_message.pose.position.y,
            controller_message.pose.position.z,
        ],
        dtype=float,
    )
    controller_quaternion = np.array(
        [
            controller_message.pose.orientation.x,
            controller_message.pose.orientation.y,
            controller_message.pose.orientation.z,
            controller_message.pose.orientation.w,
        ],
        dtype=float,
    )
    palm_position, palm_quaternion = apply_tcp_transform(
        controller_position, controller_quaternion, transform
    )
    output = pose_stamped_type()
    output.header.stamp = controller_message.header.stamp
    output.header.frame_id = "pico"
    output.pose.position.x = float(palm_position[0])
    output.pose.position.y = float(palm_position[1])
    output.pose.position.z = float(palm_position[2])
    output.pose.orientation.x = float(palm_quaternion[0])
    output.pose.orientation.y = float(palm_quaternion[1])
    output.pose.orientation.z = float(palm_quaternion[2])
    output.pose.orientation.w = float(palm_quaternion[3])
    return output


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _pose_rotation(message) -> np.ndarray:
    if message.header.frame_id != "pico":
        raise ValueError(f"pose frame must be pico, got {message.header.frame_id!r}")
    return quaternion_to_matrix(
        np.array(
            [
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w,
            ],
            dtype=float,
        )
    )


class PicoPalmTcpPublisher:
    def __init__(
        self,
        side: str,
        artifact: Path,
        transform: TcpTransform,
        *,
        gates: OrientationGates = OrientationGates(),
        capture_timeout_s: float = 5.0,
        max_pair_skew_s: float = 0.03,
    ) -> None:
        from geometry_msgs.msg import PoseStamped
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from std_msgs.msg import String, UInt64
        from std_srvs.srv import Trigger

        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        self.side = side
        self.artifact = Path(artifact).expanduser()
        self._transform = transform
        self._transform_lock = threading.Lock()
        self._capture_condition = threading.Condition()
        self._capture = OrientationCaptureBuffer(
            max_pair_skew_ns=int(max_pair_skew_s * 1.0e9)
        )
        self._gates = gates
        self._capture_timeout_s = float(capture_timeout_s)
        self._capture_active = False
        self._tracking_epoch = 0
        self._tracking_epoch_source = "unknown"
        self._pose_stamped_type = PoseStamped
        self.node = Node(f"pico_{side}_palm_tcp_publisher")
        callback_group = ReentrantCallbackGroup()
        self.controller_topic = f"/pico/pose/{side}_hand"
        self.palm_topic = f"/pico/palm_{side}"
        output_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.node.create_publisher(
            PoseStamped, self.palm_topic, output_qos
        )
        self.subscription = self.node.create_subscription(
            PoseStamped,
            self.controller_topic,
            self._controller_callback,
            qos_profile_sensor_data,
            callback_group=callback_group,
        )
        self._head_subscription = self.node.create_subscription(
            PoseStamped,
            "/pico/pose/head",
            self._head_callback,
            qos_profile_sensor_data,
            callback_group=callback_group,
        )
        self._epoch_subscription = self.node.create_subscription(
            UInt64,
            "/pico/tracking_epoch",
            self._epoch_callback,
            output_qos,
            callback_group=callback_group,
        )
        self._epoch_status_subscription = self.node.create_subscription(
            String,
            "/pico/tracking_epoch/status",
            self._epoch_status_callback,
            output_qos,
            callback_group=callback_group,
        )
        self.orientation_service_name = f"/pico/palm_orientation/{side}/calibrate"
        self._orientation_service = self.node.create_service(
            Trigger,
            self.orientation_service_name,
            self._calibrate_orientation,
            callback_group=callback_group,
        )
        self.node.get_logger().info(
            f"[{side}] TCP runtime publisher ready: "
            f"{self.controller_topic} -> {self.palm_topic}; "
            f"orientation service={self.orientation_service_name}"
        )

    def _controller_callback(self, message) -> None:
        try:
            with self._transform_lock:
                transform = self._transform
            output = build_palm_message(
                message, transform, self._pose_stamped_type
            )
        except ValueError as error:
            self.node.get_logger().warning(
                f"Rejecting {self.side} controller pose: {error}"
            )
            return
        self.publisher.publish(output)
        with self._capture_condition:
            if not self._capture_active:
                return
            try:
                accepted = self._capture.add_controller(
                    _pose_rotation(message),
                    _stamp_ns(message.header.stamp),
                    self._tracking_epoch,
                )
            except ValueError:
                accepted = False
            if accepted or self._capture.error is not None:
                self._capture_condition.notify_all()

    def _head_callback(self, message) -> None:
        with self._capture_condition:
            if not self._capture_active:
                return
            try:
                self._capture.add_head(
                    _pose_rotation(message), _stamp_ns(message.header.stamp)
                )
            except ValueError:
                return

    def _epoch_callback(self, message) -> None:
        with self._capture_condition:
            self._tracking_epoch = int(message.data)
            if self._capture_active:
                self._capture.update_epoch(
                    self._tracking_epoch, self._tracking_epoch_source
                )
                self._capture_condition.notify_all()

    def _epoch_status_callback(self, message) -> None:
        try:
            payload = json.loads(message.data)
            source = str(payload["tracking_epoch_source"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            source = "unknown"
        with self._capture_condition:
            self._tracking_epoch_source = source
            if self._capture_active:
                self._capture.update_epoch(self._tracking_epoch, source)
                self._capture_condition.notify_all()

    def _validate_candidate(self, document: dict) -> None:
        descriptor, name = tempfile.mkstemp(
            prefix=f".{self.artifact.name}.orientation.",
            suffix=".yaml",
            dir=self.artifact.parent,
        )
        path = Path(name)
        try:
            with open(descriptor, "w", encoding="utf-8", closefd=True) as stream:
                yaml.safe_dump(document, stream, sort_keys=False)
            validate_artifact(path, "tcp", self.side)
        finally:
            path.unlink(missing_ok=True)

    def _calibrate_orientation(self, _request, response):
        with self._capture_condition:
            if self._capture_active:
                response.success = False
                response.message = "orientation_calibration_busy"
                return response
            try:
                self._capture.begin(
                    self._tracking_epoch, self._tracking_epoch_source
                )
            except ValueError as error:
                response.success = False
                response.message = str(error)
                return response
            self._capture_active = True
            deadline = time.monotonic() + self._capture_timeout_s
            while (
                len(self._capture.samples) < self._gates.min_samples
                and self._capture.error is None
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._capture_condition.wait(timeout=remaining)
            samples = self._capture.samples
            capture_error = self._capture.error
            self._capture_active = False

        if capture_error is not None:
            response.success = False
            response.message = capture_error
            return response
        if len(samples) < self._gates.min_samples:
            response.success = False
            response.message = (
                f"orientation sample_count {len(samples)} is below "
                f"{self._gates.min_samples}"
            )
            return response

        try:
            summary = validate_artifact(self.artifact, "tcp", self.side)
            old_sha = file_sha256(self.artifact)
            document = yaml.safe_load(self.artifact.read_text(encoding="utf-8"))
            with self._transform_lock:
                current_rotation = quaternion_to_matrix(
                    self._transform.quaternion_xyzw
                )
            solution = solve_orientation(samples, current_rotation, self._gates)
            updated = build_orientation_only_update(document, solution, old_sha)
            self._validate_candidate(updated)
            atomic_replace_if_unchanged(self.artifact, old_sha, updated)
            new_transform = load_tcp_transform(self.artifact, self.side)
            with self._transform_lock:
                self._transform = new_transform
            result = {
                "side": self.side,
                "artifact_path": str(self.artifact.resolve()),
                "calibration_revision": summary.calibration_revision + 1,
                "translation_revision": summary.translation_revision,
                "orientation_revision": int(updated["orientation_revision"]),
                "tracking_epoch": solution.tracking_epoch,
                "sample_count": solution.sample_count,
                "orientation_rms_rad": solution.orientation_rms_rad,
                "correction_angle_rad": solution.correction_angle_rad,
            }
        except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
            response.success = False
            response.message = str(error)
            return response
        response.success = True
        response.message = json.dumps(result, sort_keys=True)
        return response


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--artifact", default=None)
    arguments, ros_arguments = parser.parse_known_args()
    artifact = (
        Path(arguments.artifact).expanduser()
        if arguments.artifact
        else default_artifact(arguments.side)
    )
    try:
        transform = load_tcp_transform(artifact, arguments.side)
    except ValueError as error:
        raise SystemExit(f"invalid {arguments.side} TCP artifact: {error}") from error

    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=ros_arguments)
    publisher = PicoPalmTcpPublisher(arguments.side, artifact, transform)
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(publisher.node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    # RCLError is not exported from rclpy.exceptions by every Humble build.
    # The pybind RCLError used by those builds derives from RuntimeError, so
    # only suppress it after another process has already shut this context
    # down.  Runtime errors raised while the context is healthy still fail.
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        executor.shutdown()
        publisher.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
