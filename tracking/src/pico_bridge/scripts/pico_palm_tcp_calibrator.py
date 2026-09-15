#!/usr/bin/env python3
"""Calibrate and publish a glove palm pose from the matching PICO controller pose."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import select
import sys
import threading
import time
import termios
import tty

import numpy as np

from pico_palm_orientation_core import (
    OrientationCaptureBuffer,
    OrientationGates,
    OrientationSolution,
    gravity_leveled_heading_rotation,
    solve_orientation,
    translation_fingerprint,
)


def default_output(side: str) -> str:
    return f"~/.config/pico_tracker/pico_{side}_palm_tcp.yaml"


def tcp_position_prompt(side: str, sample_count: int) -> str:
    label = "左侧" if side == "left" else "右侧"
    return (
        f"[{label}] TCP 位置标定：保持掌心参考点不动，明显改变手柄朝向，"
        f"每摆好一个姿态按空格采集 {sample_count} 次；按 q 取消"
    )


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def tcp_orientation_prompt(side: str) -> str:
    label = "左侧" if side == "left" else "右侧"
    return (
        "TCP 位置检查已通过。请双臂向正前方水平伸直、左右掌心相对，"
        f"摆好后再按一次空格启动{label} TCP 姿态采集，并保持约 1～2 秒；"
        "按 q 取消"
    )


def rotation_from_xyzw(quaternion: np.ndarray) -> np.ndarray:
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError("quaternion must contain four finite values")
    norm = np.linalg.norm(q)
    if norm <= 1e-9:
        raise ValueError("quaternion must be non-zero")
    x, y, z, w = q / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quaternion_from_rotation(rotation: np.ndarray) -> np.ndarray:
    """Return a normalized xyzw quaternion from a proper rotation matrix."""
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("rotation must be a finite 3x3 matrix")
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        quaternion = np.array([
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        next_index = (index + 1) % 3
        last_index = (index + 2) % 3
        scale = 2.0 * np.sqrt(max(1.0 + matrix[index, index] - diagonal[next_index] - diagonal[last_index], 1e-12))
        vector = np.zeros(4)
        vector[index] = 0.25 * scale
        vector[3] = (matrix[last_index, next_index] - matrix[next_index, last_index]) / scale
        vector[next_index] = (matrix[next_index, index] + matrix[index, next_index]) / scale
        vector[last_index] = (matrix[last_index, index] + matrix[index, last_index]) / scale
        quaternion = vector
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[3] < 0.0:
        quaternion *= -1.0
    return quaternion


def quaternion_values(value) -> np.ndarray:
    """Normalize YAML quaternion forms to an xyzw NumPy vector.

    Calibration files written by this project use a mapping (``x/y/z/w``),
    while older files and hand-edited configs may use a four-element list.
    Accept both forms so loading a previously saved calibration never crashes
    the ROS node during startup.
    """
    if isinstance(value, dict):
        value = [value.get(axis) for axis in ("x", "y", "z", "w")]
    values = np.asarray(value, dtype=float)
    if values.shape != (4,) or not np.all(np.isfinite(values)):
        raise ValueError("quaternion_xyzw must contain four finite values")
    return values


def solve_tcp_rotation_for_gravity_leveled_hmd_heading(
    controller_rotation: np.ndarray,
    head_rotation: np.ndarray,
) -> np.ndarray:
    """Return T_controller_palm with HMD yaw and gravity-level pitch/roll."""
    controller = np.asarray(controller_rotation, dtype=float)
    head = np.asarray(head_rotation, dtype=float)
    if controller.shape != (3, 3) or head.shape != (3, 3):
        raise ValueError("controller and head rotations must be 3x3 matrices")
    return controller.T @ gravity_leveled_heading_rotation(head)


def build_tcp_artifact(
    side: str,
    source_topic: str,
    translation: np.ndarray,
    rotation: np.ndarray,
    sample_count: int,
    position_rms: float,
    calibration_revision: int,
    sample_matrix_rank: int,
    sample_matrix_condition: float,
    orientation_solution: OrientationSolution | None = None,
) -> dict:
    """Build the versioned, exactly-once controller-to-palm artifact."""
    if side not in ("left", "right"):
        raise ValueError("side must be left or right")
    if orientation_solution is None:
        raise ValueError("orientation_solution is required for a calibrated TCP artifact")
    translation = np.asarray(translation, dtype=float)
    if translation.shape != (3,) or not np.all(np.isfinite(translation)):
        raise ValueError("translation must contain three finite values")
    quaternion = quaternion_from_rotation(rotation)
    covariance_matrix = np.zeros((6, 6))
    covariance_matrix[0:3, 0:3] = np.eye(3) * 1.0e-4
    covariance_matrix[3:6, 3:6] = np.asarray(
        orientation_solution.covariance, dtype=float
    )
    covariance = [
        float(covariance_matrix[row, column])
        for row in range(6)
        for column in range(row, 6)
    ]
    orientation_sample_count = int(orientation_solution.sample_count)
    document = {
        "schema_version": 2,
        "valid": True,
        "side": side,
        "pose_semantics": "controller_pose",
        "transform_convention": "T_controller_palm",
        "source_topic": source_topic,
        "orientation_reference": "gravity_leveled_hmd_heading",
        "translation_m": [float(value) for value in translation],
        "quaternion_xyzw": [float(value) for value in quaternion],
        "covariance_upper_triangle_6x6": covariance,
        "orientation_calibrated": True,
        "calibration_revision": int(calibration_revision),
        "translation_revision": int(calibration_revision),
        "orientation_revision": 1,
        "orientation_only_ancestor_sha256": [],
        "source_recording": "interactive_tcp_calibration",
        "lineage": [source_topic, "/pico/pose/head", "gravity_leveled_hmd_heading"],
        "quality": {
            "sample_count": int(sample_count),
            "position_sample_count": int(sample_count),
            "orientation_sample_count": orientation_sample_count,
            "orientation_capture_separate": True,
            "position_rms_m": float(position_rms),
            "sample_matrix_rank": int(sample_matrix_rank),
            "sample_matrix_condition": float(sample_matrix_condition),
        },
    }
    document["orientation_calibration"] = {
        "method": "gravity_leveled_hmd_heading_bilateral_forward_palms_facing",
        "tracking_epoch": int(orientation_solution.tracking_epoch),
        "sample_count": int(orientation_solution.sample_count),
        "orientation_rms_rad": float(orientation_solution.orientation_rms_rad),
        "correction_angle_rad": float(orientation_solution.correction_angle_rad),
    }
    document["translation_fingerprint_sha256"] = translation_fingerprint(document)
    return document


def _tcp_sample_system(samples: list[tuple[np.ndarray, np.ndarray]]):
    if len(samples) < 4:
        raise ValueError("four samples are required for the reference TCP position solver")
    identity = np.eye(3)
    rf = np.zeros((12, 6), dtype=float)
    pf = np.zeros((12, 1), dtype=float)
    for i, (rotation, position) in enumerate(samples[:4]):
        rf[3 * i:3 * i + 3, 0:3] = rotation
        rf[3 * i:3 * i + 3, 3:6] = -identity
        pf[3 * i:3 * i + 3, 0] = -position
    return rf, pf


def tcp_sample_excitation(samples: list[tuple[np.ndarray, np.ndarray]]) -> tuple[int, float]:
    rf, _ = _tcp_sample_system(samples)
    singular_values = np.linalg.svd(rf, compute_uv=False)
    rank = int(np.linalg.matrix_rank(rf))
    if rank < 6 or singular_values[-1] <= 1.0e-9:
        raise ValueError(
            "四次采样的手柄姿态变化不足；请保持掌心参考点不动，"
            "让手柄分别绕不同方向转动后重新采样"
        )
    condition = float(singular_values[0] / singular_values[-1])
    if not np.isfinite(condition) or condition > 1.0e3:
        raise ValueError(
            "四次采样方向过于接近；请增大手柄姿态变化并覆盖至少两个旋转方向"
        )
    return rank, condition


def solve_reference_tcp_position(samples: list[tuple[np.ndarray, np.ndarray]]) -> tuple[np.ndarray, float]:
    """Use controller_pub.py's first-four-pose Rf/Pf/pinv position solver."""
    rf, pf = _tcp_sample_system(samples)
    tcp_sample_excitation(samples)
    solution = np.linalg.pinv(rf) @ pf
    tcp_position = solution[:3, 0]
    residual = float(np.sqrt(np.mean((rf @ solution - pf) ** 2)))
    return tcp_position, residual


def translation_values(document: dict) -> np.ndarray:
    translation = document["translation_m"]
    if isinstance(translation, dict):
        translation = [translation[axis] for axis in ("x", "y", "z")]
    position = np.asarray(translation, dtype=float)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError("translation_m must contain three finite values")
    return position


def load_tcp(path: Path) -> dict | None:
    if not path.exists():
        return None
    try:
        import yaml
        with path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream) or {}
        if not document.get("valid"):
            return None
        translation_values(document)
        return document
    except (OSError, KeyError, TypeError, ValueError, ImportError):
        return None


class PalmTcpCalibrator:
    def __init__(self, output: Path, side: str = "left", sample_count: int = 4):
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from std_msgs.msg import String, UInt64

        self.rclpy = rclpy
        self._pose_stamped_type = PoseStamped
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        self.side = side
        self.node = Node(f"pico_{side}_palm_tcp_calibrator")
        self.controller_topic = f"/pico/pose/{side}_hand"
        self.palm_topic = f"/pico/palm_{side}"
        self.output = output.expanduser()
        self.sample_count = sample_count
        self._lock = threading.Lock()
        self._latest_controller: tuple[np.ndarray, np.ndarray, float, object] | None = None
        self._latest_head: tuple[np.ndarray, object] | None = None
        self._samples: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
        self._stage = "position"
        self._tcp_position: np.ndarray | None = None
        self._tcp_rotation = np.eye(3)
        self._residual: float | None = None
        self._sample_matrix_rank = 0
        self._sample_matrix_condition = math.inf
        self._orientation_calibrated = False
        self._orientation_gates = OrientationGates()
        self._orientation_capture = OrientationCaptureBuffer(
            max_pair_skew_ns=30_000_000
        )
        self._orientation_capture_timeout_s = 5.0
        self._orientation_capture_deadline = 0.0
        self._orientation_capture_active = False
        self._orientation_solution: OrientationSolution | None = None
        self._tracking_epoch = 0
        self._tracking_epoch_source = "unknown"
        self._stop_keyboard = threading.Event()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(
            PoseStamped, self.controller_topic, self._controller_callback, qos
        )
        self.node.create_subscription(
            PoseStamped, "/pico/pose/head", self._head_callback, qos
        )
        output_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.node.create_publisher(PoseStamped, self.palm_topic, output_qos)
        self.node.create_subscription(
            UInt64, "/pico/tracking_epoch", self._epoch_callback, output_qos
        )
        self.node.create_subscription(
            String, "/pico/tracking_epoch/status", self._epoch_status_callback, output_qos
        )

        existing = load_tcp(self.output)
        if existing is not None:
            self._tcp_position = translation_values(existing)
            quaternion = quaternion_values(existing.get("quaternion_xyzw", [0.0, 0.0, 0.0, 1.0]))
            try:
                self._tcp_rotation = rotation_from_xyzw(quaternion)
                self._orientation_calibrated = bool(existing.get("orientation_calibrated", False))
            except ValueError:
                self._tcp_rotation = np.eye(3)
            self._residual = float(existing.get("quality", {}).get("position_rms_m", 0.0))
            self.node.get_logger().info(f"已加载掌心 TCP 标定文件：{self.output}")
        self.node.get_logger().info(tcp_position_prompt(self.side, self.sample_count))

    def _controller_callback(self, message) -> None:
        position = np.array([
            message.pose.position.x, message.pose.position.y, message.pose.position.z
        ], dtype=float)
        quaternion = np.array([
            message.pose.orientation.x, message.pose.orientation.y,
            message.pose.orientation.z, message.pose.orientation.w,
        ], dtype=float)
        try:
            rotation_from_xyzw(quaternion)
        except ValueError:
            return
        controller_rotation = rotation_from_xyzw(quaternion)
        with self._lock:
            self._latest_controller = (position, quaternion, time.monotonic(), message.header.stamp)
            if self._orientation_capture_active:
                try:
                    self._orientation_capture.add_controller(
                        controller_rotation,
                        _stamp_ns(message.header.stamp),
                        self._tracking_epoch,
                    )
                except ValueError:
                    pass
        self._publish_pose()

    def _head_callback(self, message) -> None:
        position = np.array([
            message.pose.position.x, message.pose.position.y, message.pose.position.z
        ], dtype=float)
        quaternion = np.array([
            message.pose.orientation.x, message.pose.orientation.y,
            message.pose.orientation.z, message.pose.orientation.w,
        ], dtype=float)
        try:
            rotation_from_xyzw(quaternion)
        except ValueError:
            return
        head_rotation = rotation_from_xyzw(quaternion)
        with self._lock:
            self._latest_head = (quaternion, message.header.stamp)
            if self._orientation_capture_active:
                try:
                    self._orientation_capture.add_head(
                        head_rotation, _stamp_ns(message.header.stamp)
                    )
                except ValueError:
                    pass

    def _epoch_callback(self, message) -> None:
        with self._lock:
            self._tracking_epoch = int(message.data)
            if self._orientation_capture_active:
                self._orientation_capture.update_epoch(
                    self._tracking_epoch, self._tracking_epoch_source
                )

    def _epoch_status_callback(self, message) -> None:
        try:
            payload = json.loads(message.data)
            source = str(payload["tracking_epoch_source"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            source = "unknown"
        with self._lock:
            self._tracking_epoch_source = source
            if self._orientation_capture_active:
                self._orientation_capture.update_epoch(
                    self._tracking_epoch, self._tracking_epoch_source
                )

    def capture(self) -> None:
        with self._lock:
            latest = self._latest_controller
            head = self._latest_head
        if latest is None or head is None or time.monotonic() - latest[2] > 0.25:
            self.node.get_logger().warning(
                f"未收到新鲜的 {self.controller_topic} 数据，请确认 PICO 正在发布"
            )
            return
        position, quaternion, _, _ = latest
        try:
            rotation = rotation_from_xyzw(quaternion)
        except ValueError as exc:
            self.node.get_logger().warning(str(exc))
            return
        head_rotation = rotation_from_xyzw(head[0])

        if self._stage == "orientation":
            with self._lock:
                if self._orientation_capture_active:
                    self.node.get_logger().warning("TCP 姿态采集正在进行，请保持当前姿势")
                    return
                try:
                    self._orientation_capture.begin(
                        self._tracking_epoch, self._tracking_epoch_source
                    )
                    self._orientation_capture.add_head(
                        head_rotation, _stamp_ns(head[1])
                    )
                except ValueError as exc:
                    self.node.get_logger().error(
                        f"无法开始 TCP 姿态采集：{exc}；请确认 tracking epoch 有效"
                    )
                    return
                self._orientation_capture_active = True
                self._orientation_capture_deadline = (
                    time.monotonic() + self._orientation_capture_timeout_s
                )
            self.node.get_logger().info(
                f"开始采集 TCP 姿态，目标至少 {self._orientation_gates.min_samples} "
                "个有效样本；请保持双臂水平伸直、掌心相对"
            )
            return

        self._samples.append((rotation, position.copy(), head_rotation))
        self.node.get_logger().info(
            f"已采集 TCP 位置样本 {len(self._samples)}/{self.sample_count}"
        )
        if len(self._samples) == self.sample_count:
            try:
                tcp, residual = solve_reference_tcp_position(
                    [(sample[0], sample[1]) for sample in self._samples]
                )
                rank, condition = tcp_sample_excitation(
                    [(sample[0], sample[1]) for sample in self._samples]
                )
                if residual > 0.02:
                    raise ValueError(f"position residual too large: {residual:.4f} m")
                tcp_distance = float(np.linalg.norm(tcp))
                if not 0.02 <= tcp_distance <= 0.25:
                    raise ValueError(
                        "手柄到掌心距离不合理；请确认手套固定牢靠，并按要求改变手柄姿态"
                    )
                self._tcp_position = tcp
                self._residual = residual
                self._sample_matrix_rank = rank
                self._sample_matrix_condition = condition
                self._orientation_calibrated = False
                self._stage = "orientation"
                self.node.get_logger().info(
                    f"TCP 位置检查通过：[{tcp[0]:.4f}, {tcp[1]:.4f}, "
                    f"{tcp[2]:.4f}] m，位置 RMS={residual:.5f} m"
                )
                self.node.get_logger().info(tcp_orientation_prompt(self.side))
            except ValueError as exc:
                self.node.get_logger().error(str(exc))
            finally:
                self._samples.clear()

    def _finalize_orientation_capture(self) -> bool:
        with self._lock:
            if not self._orientation_capture_active:
                return False
            samples = self._orientation_capture.samples
            capture_error = self._orientation_capture.error
            enough_samples = len(samples) >= self._orientation_gates.min_samples
            timed_out = time.monotonic() >= self._orientation_capture_deadline
            if capture_error is None and not enough_samples and not timed_out:
                return False
            self._orientation_capture_active = False

        if capture_error is not None:
            self.node.get_logger().error(
                f"TCP 姿态采集失败：{capture_error}；请重新摆好后按空格重试"
            )
            return False
        if not enough_samples:
            self.node.get_logger().error(
                f"TCP 姿态有效样本不足：{len(samples)}/"
                f"{self._orientation_gates.min_samples}；请保持 PICO 数据连续后重试"
            )
            return False
        try:
            solution = solve_orientation(
                samples, self._tcp_rotation, self._orientation_gates
            )
        except ValueError as exc:
            self.node.get_logger().error(f"TCP 姿态质量检查未通过：{exc}")
            return False

        self._orientation_solution = solution
        self._tcp_rotation = solution.rotation.copy()
        self._orientation_calibrated = True
        self._save()
        self.node.get_logger().info(
            f"掌心 TCP 已保存：姿态使用 {solution.sample_count} 个重力水平样本，"
            f"RMS={solution.orientation_rms_rad:.5f} rad"
        )
        self._stop_keyboard.set()
        self.rclpy.shutdown()
        return True

    def _save(self) -> None:
        import yaml
        self.output.parent.mkdir(parents=True, exist_ok=True)
        document = build_tcp_artifact(
            side=self.side,
            source_topic=self.controller_topic,
            translation=self._tcp_position,
            rotation=self._tcp_rotation,
            sample_count=self.sample_count,
            position_rms=float(self._residual),
            calibration_revision=1,
            sample_matrix_rank=self._sample_matrix_rank,
            sample_matrix_condition=self._sample_matrix_condition,
            orientation_solution=self._orientation_solution,
        )
        temporary = self.output.with_suffix(self.output.suffix + ".tmp")
        with temporary.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(document, stream, sort_keys=False)
        temporary.replace(self.output)

    def _publish_pose(self) -> None:
        if self._tcp_position is None or not self._orientation_calibrated:
            return
        with self._lock:
            latest = self._latest_controller
        if latest is None or time.monotonic() - latest[2] > 0.5:
            return
        controller_position, quaternion, _, controller_stamp = latest
        rotation = rotation_from_xyzw(quaternion)
        palm = controller_position + rotation @ self._tcp_position
        palm_rotation = rotation @ self._tcp_rotation
        palm_quaternion = quaternion_from_rotation(palm_rotation)
        message = self._pose_stamped_type()
        message.header.stamp = controller_stamp
        message.header.frame_id = "pico"
        message.pose.position.x, message.pose.position.y, message.pose.position.z = palm
        message.pose.orientation.x, message.pose.orientation.y = palm_quaternion[0], palm_quaternion[1]
        message.pose.orientation.z, message.pose.orientation.w = palm_quaternion[2], palm_quaternion[3]
        self.publisher.publish(message)

    def keyboard_loop(self) -> None:
        if not sys.stdin.isatty():
            self.node.get_logger().warning("当前终端不是 TTY，无法使用键盘进行标定")
            return
        fd = sys.stdin.fileno()
        original = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while not self._stop_keyboard.is_set() and self.rclpy.ok():
                self._finalize_orientation_capture()
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not readable:
                    continue
                key = sys.stdin.read(1)
                if key == " ":
                    self.capture()
                elif key.lower() == "q":
                    self.rclpy.shutdown()
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, original)

    def run(self) -> None:
        keyboard = threading.Thread(target=self.keyboard_loop, daemon=True)
        keyboard.start()
        try:
            self.rclpy.spin(self.node)
        finally:
            self._stop_keyboard.set()
            keyboard.join(timeout=1.0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--output", default=None)
    arguments = parser.parse_args()
    import rclpy
    rclpy.init()
    try:
        output = arguments.output or default_output(arguments.side)
        PalmTcpCalibrator(Path(output), side=arguments.side).run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
