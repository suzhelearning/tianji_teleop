#!/usr/bin/env python3
"""Calibrate and publish a wrist pivot from a calibrated palm pose stream."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import select
import sys
import threading
import time
import termios
import tty

import numpy as np


def default_output(side: str) -> str:
    return f"~/.config/pico_tracker/pico_{side}_wrist_pivot.yaml"


def wrist_calibration_prompt(side: str, sample_count: int, motion: str) -> str:
    label = "左侧" if side == "left" else "右侧"
    motion_instruction = (
        "缓慢左右转动手掌；局部 z 偏移固定为 0"
        if motion == "yaw"
        else "缓慢进行左右转动，并加入小幅俯仰和横滚"
    )
    return (
        f"[{label}] 掌心到手腕标定：按空格开始采集 {sample_count} 个样本，"
        f"全程保持手腕不动，{motion_instruction}；成功后自动结束，按 q 取消"
    )


def rotation_from_xyzw(quaternion) -> np.ndarray:
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) <= 1e-9:
        raise ValueError("orientation must be a finite non-zero xyzw quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def palm_sample_from_message(message, expected_frame: str = "pico"):
    """Extract a canonical, finite palm sample or reject it fail-closed."""
    if message.header.frame_id != expected_frame:
        raise ValueError(
            f"palm frame must be {expected_frame!r}, got {message.header.frame_id!r}"
        )
    position = np.array(
        [message.pose.position.x, message.pose.position.y, message.pose.position.z],
        dtype=float,
    )
    if not np.all(np.isfinite(position)):
        raise ValueError("palm position must be finite")
    quaternion = np.array(
        [
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        ],
        dtype=float,
    )
    return position, rotation_from_xyzw(quaternion), quaternion


def quaternion_from_rotation(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("rotation must be a finite 3x3 matrix")
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        result = np.array([
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        next_index, last_index = (index + 1) % 3, (index + 2) % 3
        scale = 2.0 * np.sqrt(max(
            1.0 + matrix[index, index] - diagonal[next_index] - diagonal[last_index],
            1e-12,
        ))
        result = np.zeros(4)
        result[index] = 0.25 * scale
        result[3] = (matrix[last_index, next_index] - matrix[next_index, last_index]) / scale
        result[next_index] = (matrix[next_index, index] + matrix[index, next_index]) / scale
        result[last_index] = (matrix[last_index, index] + matrix[index, last_index]) / scale
    result /= np.linalg.norm(result)
    if result[3] < 0.0:
        result *= -1.0
    return result


@dataclass(frozen=True)
class WristPivotSolution:
    wrist_world: np.ndarray
    wrist_to_palm: np.ndarray


def solve_wrist_pivot(
    samples: list[tuple[np.ndarray, np.ndarray]],
    motion: str = "yaw",
) -> tuple[WristPivotSolution, float, float]:
    """Solve p_palm = p_wrist + R_palm * r_wrist_to_palm."""
    if len(samples) < 4:
        raise ValueError("at least four wrist-pivot samples are required")
    if motion not in ("yaw", "full"):
        raise ValueError("motion must be yaw or full")
    offset_columns = 2 if motion == "yaw" else 3
    matrix = np.zeros((3 * len(samples), 3 + offset_columns), dtype=float)
    vector = np.zeros(3 * len(samples), dtype=float)
    for index, (position, rotation) in enumerate(samples):
        position = np.asarray(position, dtype=float)
        rotation = np.asarray(rotation, dtype=float)
        if position.shape != (3,) or rotation.shape != (3, 3):
            raise ValueError("invalid palm sample shape")
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(rotation)):
            raise ValueError("palm samples must contain only finite values")
        matrix[3 * index:3 * index + 3, :3] = np.eye(3)
        matrix[3 * index:3 * index + 3, 3:] = rotation[:, :offset_columns]
        vector[3 * index:3 * index + 3] = position
    solution, _, rank, singular_values = np.linalg.lstsq(matrix, vector, rcond=None)
    expected_rank = 5 if motion == "yaw" else 6
    if rank < expected_rank:
        if motion == "yaw":
            raise ValueError("wrist-pivot yaw excitation is degenerate; rotate through a wide yaw range")
        raise ValueError("wrist-pivot excitation is degenerate; rotate about at least two axes")
    condition = float(singular_values[0] / singular_values[-1])
    residual = float(np.sqrt(np.mean((matrix @ solution - vector) ** 2)))
    if not np.all(np.isfinite(solution)) or not np.isfinite(condition) or not np.isfinite(residual):
        raise ValueError("wrist-pivot solution must contain only finite values")
    offset = np.zeros(3, dtype=float)
    offset[:offset_columns] = solution[3:]
    return WristPivotSolution(solution[:3], offset), residual, condition


def wrist_position_from_palm_pose(
    palm_position: np.ndarray, palm_rotation: np.ndarray, wrist_to_palm: np.ndarray
) -> np.ndarray:
    return np.asarray(palm_position, dtype=float) - np.asarray(palm_rotation, dtype=float) @ np.asarray(wrist_to_palm, dtype=float)


def load_pivot(path: Path) -> dict | None:
    if not path.exists():
        return None
    try:
        import yaml
        with path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream) or {}
        if not document.get("valid"):
            return None
        vector = np.array([
            document["wrist_to_palm_m"][axis] for axis in ("x", "y", "z")
        ], dtype=float)
        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            return None
        return document
    except (OSError, KeyError, TypeError, ValueError, ImportError):
        return None


class PalmWristCalibrator:
    def __init__(self, output: Path, side: str, sample_count: int = 60, motion: str = "yaw"):
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String, UInt64

        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        self.rclpy = rclpy
        self.side = side
        self.output = output.expanduser()
        self.sample_count = sample_count
        self.motion = motion
        self.node = Node(f"pico_{side}_palm_wrist_calibrator")
        self.palm_topic = f"/pico/palm_{side}"
        self.wrist_topic = f"/pico/wrist_{side}"
        self._lock = threading.Lock()
        self._capturing = False
        self._samples: list[tuple[np.ndarray, np.ndarray]] = []
        self._wrist_to_palm: np.ndarray | None = None
        self._stop_keyboard = threading.Event()
        self._tracking_epoch = 0
        self._tracking_epoch_numeric = 0
        self._tracking_epoch_source = "unknown"
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        output_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pose_type = PoseStamped
        self.publisher = self.node.create_publisher(PoseStamped, self.wrist_topic, output_qos)
        self.node.create_subscription(PoseStamped, self.palm_topic, self._palm_callback, qos)
        self.node.create_subscription(
            UInt64, "/pico/tracking_epoch", self._epoch_callback, output_qos
        )
        self.node.create_subscription(
            String, "/pico/tracking_epoch/status", self._epoch_status_callback, output_qos
        )
        existing = load_pivot(self.output)
        if existing is not None:
            self._wrist_to_palm = np.array([
                existing["wrist_to_palm_m"][axis] for axis in ("x", "y", "z")
            ], dtype=float)
            self.node.get_logger().info(f"已加载掌心到手腕标定文件：{self.output}")
        self.node.get_logger().info(
            wrist_calibration_prompt(side, sample_count, motion)
        )

    def _palm_callback(self, message) -> None:
        try:
            position, rotation, quaternion = palm_sample_from_message(message)
        except ValueError:
            return
        with self._lock:
            if not self._epoch_consistent():
                return
            capturing = self._capturing
            if capturing:
                self._samples.append((position, rotation))
                if len(self._samples) >= self.sample_count:
                    samples = self._samples[:]
                    self._samples.clear()
                    self._capturing = False
                else:
                    samples = None
            else:
                samples = None
            wrist_to_palm = None if self._wrist_to_palm is None else self._wrist_to_palm.copy()
        if samples is not None:
            self._finish_calibration(samples)
        if wrist_to_palm is not None:
            wrist = wrist_position_from_palm_pose(position, rotation, wrist_to_palm)
            self._publish_wrist(message, wrist, quaternion)

    def _epoch_callback(self, message) -> None:
        with self._lock:
            previous = self._tracking_epoch_numeric
            self._tracking_epoch_numeric = int(message.data)
            self._handle_epoch_transition(previous, self._tracking_epoch_numeric)

    def _epoch_status_callback(self, message) -> None:
        with self._lock:
            previous = self._tracking_epoch
            try:
                payload = json.loads(message.data)
                self._tracking_epoch = int(payload["tracking_epoch"])
                self._tracking_epoch_source = str(payload["tracking_epoch_source"])
            except (json.JSONDecodeError, KeyError, TypeError, ValueError):
                self._tracking_epoch = 0
                self._tracking_epoch_source = "unknown"
            self._handle_epoch_transition(previous, self._tracking_epoch)

    def _epoch_consistent(self) -> bool:
        return (
            self._tracking_epoch > 0
            and self._tracking_epoch_numeric == self._tracking_epoch
            and self._tracking_epoch_source in {"tcp_connection", "wire_world_reset"}
        )

    def _handle_epoch_transition(self, previous: int, current: int) -> None:
        if previous > 0 and current != previous:
            self._samples.clear()
            self._capturing = False

    def _publish_wrist(self, palm_message, wrist: np.ndarray, quaternion: np.ndarray) -> None:
        message = self._pose_type()
        message.header = palm_message.header
        message.pose.position.x, message.pose.position.y, message.pose.position.z = wrist
        message.pose.orientation.x, message.pose.orientation.y = quaternion[:2]
        message.pose.orientation.z, message.pose.orientation.w = quaternion[2:]
        self.publisher.publish(message)

    def _finish_calibration(self, samples) -> None:
        try:
            solution, residual, condition = solve_wrist_pivot(samples, motion=self.motion)
            wrist_distance = float(np.linalg.norm(solution.wrist_to_palm))
            if wrist_distance < 0.01:
                raise ValueError(
                    "掌心到手腕距离过短；请保持手腕位置不动，并让手掌绕手腕充分左右转动"
                )
            if wrist_distance > 0.16:
                raise ValueError(
                    "掌心到手腕距离过长；请确认手腕固定、手套未滑动后重新标定"
                )
            if solution.wrist_to_palm[0] <= 0.0:
                raise ValueError(
                    "掌心没有位于手腕局部 +X 前方；请检查 TCP 朝向并重新标定"
                )
            if residual > 0.015:
                raise ValueError(f"wrist-pivot residual too large: {residual:.4f} m")
            if condition > 1e4:
                raise ValueError(f"wrist-pivot excitation is ill-conditioned: {condition:.1f}")
            self._wrist_to_palm = solution.wrist_to_palm.copy()
            self._save(solution, residual, condition)
            self.node.get_logger().info(
                f"掌心到手腕标定已保存：wrist-to-palm=[{self._wrist_to_palm[0]:.4f}, "
                f"{self._wrist_to_palm[1]:.4f}, {self._wrist_to_palm[2]:.4f}] m, "
                f"RMS={residual:.5f} m，条件数={condition:.1f}"
            )
            self._stop_keyboard.set()
        except ValueError as exc:
            self.node.get_logger().error(str(exc))

    def _save(self, solution: WristPivotSolution, residual: float, condition: float) -> None:
        import yaml
        self.output.parent.mkdir(parents=True, exist_ok=True)
        document = {
            "schema_version": 1,
            "valid": True,
            "side": self.side,
            "transform_convention": "wrist_to_palm",
            "source_topic": self.palm_topic,
            "source_frame": "pico",
            "motion_mode": self.motion,
            "tracking_epoch": self._tracking_epoch,
            "tracking_epoch_source": self._tracking_epoch_source,
            "wrist_to_palm_m": {
                axis: float(value) for axis, value in zip("xyz", solution.wrist_to_palm)
            },
            "quality": {
                "sample_count": self.sample_count,
                "position_rms_m": float(residual),
                "condition_number": float(condition),
            },
        }
        temporary = self.output.with_suffix(self.output.suffix + ".tmp")
        with temporary.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(document, stream, sort_keys=False)
        temporary.replace(self.output)

    def keyboard_loop(self) -> None:
        if not sys.stdin.isatty():
            self.node.get_logger().warning("当前终端不是 TTY，无法使用键盘进行标定")
            return
        fd = sys.stdin.fileno()
        original = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while not self._stop_keyboard.is_set():
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not ready:
                    continue
                key = sys.stdin.read(1)
                if key == " ":
                    with self._lock:
                        if self._capturing:
                            continue
                        if not self._epoch_consistent():
                            self.node.get_logger().error(
                                "tracking_epoch 不可用；请重置或重新连接 PICO 后再采集"
                            )
                            continue
                        self._samples.clear()
                        self._capturing = True
                    self.node.get_logger().info(
                        f"正在采集 {self.sample_count} 个掌心姿态，请保持手腕不动"
                    )
                elif key.lower() == "q":
                    self._stop_keyboard.set()
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, original)

    def run(self) -> None:
        keyboard = threading.Thread(target=self.keyboard_loop, daemon=True)
        keyboard.start()
        try:
            # A successful solution is produced inside a subscription callback.
            # Shutting down rclpy from that callback can deadlock the executor,
            # so the callback only sets the completion event and this owning
            # thread leaves the executor cooperatively.
            while self.rclpy.ok() and not self._stop_keyboard.is_set():
                self.rclpy.spin_once(self.node, timeout_sec=0.1)
        finally:
            self._stop_keyboard.set()
            keyboard.join(timeout=1.0)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--sample-count", type=int, default=60)
    parser.add_argument("--motion", choices=("yaw", "full"), default="yaw")
    parser.add_argument("--output", default=None)
    arguments = parser.parse_args()
    if arguments.sample_count < 60:
        raise SystemExit("--sample-count must be at least 60")
    output = arguments.output or default_output(arguments.side)
    import rclpy
    rclpy.init()
    try:
        PalmWristCalibrator(
            Path(output), arguments.side, arguments.sample_count, arguments.motion
        ).run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
