#!/usr/bin/env python3
"""Official Wuji Hand 2 retargeter to the TJ MuJoCo hand UDP bridge.

The retargeting implementation remains ``wuji_retargeting.Retargeter``.  This
file only adapts the ROS2 ``/hand_input`` contract and serializes the resulting
official 20-joint Hand 2 command for the C++ MuJoCo process.
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np


_REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(_REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPOSITORY_ROOT))


HAND2_JOINT_COUNT = 20
HAND2_JOINT_NAMES = {
    "left": (
        "l_thumb_cmc_flex", "l_thumb_cmc_abd", "l_thumb_mcp", "l_thumb_ip",
        "l_index_finger_mcp_flex", "l_index_finger_mcp_abd",
        "l_index_finger_pip", "l_index_finger_dip",
        "l_middle_finger_mcp_flex", "l_middle_finger_mcp_abd",
        "l_middle_finger_pip", "l_middle_finger_dip",
        "l_ring_finger_mcp_flex", "l_ring_finger_mcp_abd",
        "l_ring_finger_pip", "l_ring_finger_dip",
        "l_pinky_mcp_flex", "l_pinky_mcp_abd",
        "l_pinky_pip", "l_pinky_dip",
    ),
    "right": (
        "r_thumb_cmc_flex", "r_thumb_cmc_abd", "r_thumb_mcp", "r_thumb_ip",
        "r_index_finger_mcp_flex", "r_index_finger_mcp_abd",
        "r_index_finger_pip", "r_index_finger_dip",
        "r_middle_finger_mcp_flex", "r_middle_finger_mcp_abd",
        "r_middle_finger_pip", "r_middle_finger_dip",
        "r_ring_finger_mcp_flex", "r_ring_finger_mcp_abd",
        "r_ring_finger_pip", "r_ring_finger_dip",
        "r_pinky_mcp_flex", "r_pinky_mcp_abd",
        "r_pinky_pip", "r_pinky_dip",
    ),
}

PACKET_MAGIC = b"TJH2"
PACKET_VERSION = 1
PACKET_FLAGS_LEFT_VALID = 1 << 0
PACKET_FLAGS_RIGHT_VALID = 1 << 1
PACKET_FLAGS_MASK = PACKET_FLAGS_LEFT_VALID | PACKET_FLAGS_RIGHT_VALID
_PACKET_BODY = struct.Struct("<4sBBHQq40d")
_PACKET_CRC = struct.Struct("<I")
HAND_COMMAND_PACKET_SIZE = _PACKET_BODY.size + _PACKET_CRC.size


@dataclass(frozen=True)
class HandCommandFrame:
    sequence: int
    source_timestamp_ns: int
    left: np.ndarray
    right: np.ndarray
    left_valid: bool
    right_valid: bool


def sequence_gap_requires_reset(
    previous_sequence: int | None,
    sequence: int,
) -> bool:
    """Return whether a missing/rollback frame should reset LP filter state."""

    return previous_sequence is not None and int(sequence) != int(previous_sequence) + 1


def _as_joint_vector(value: Sequence[float], field: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (HAND2_JOINT_COUNT,):
        raise ValueError(
            f"{field} must have shape ({HAND2_JOINT_COUNT},), got {result.shape}"
        )
    if not np.isfinite(result).all():
        raise ValueError(f"{field} must contain finite values")
    return result


def _joint_reorder_indices(
    source_names: Sequence[str], target_names: Sequence[str]
) -> np.ndarray:
    """Return indices that reorder a joint vector from source to target names."""

    source = tuple(str(name) for name in source_names)
    target = tuple(str(name) for name in target_names)
    if len(source) != HAND2_JOINT_COUNT or len(set(source)) != len(source):
        raise ValueError("Hand 2 source joint names must contain 20 unique entries")
    if len(target) != HAND2_JOINT_COUNT or len(set(target)) != len(target):
        raise ValueError("Hand 2 target joint names must contain 20 unique entries")
    source_index = {name: index for index, name in enumerate(source)}
    if set(source_index) != set(target):
        missing = sorted(set(target) - set(source_index))
        unexpected = sorted(set(source_index) - set(target))
        raise ValueError(
            "Hand 2 joint-name mismatch: "
            f"missing={missing}, unexpected={unexpected}"
        )
    return np.asarray([source_index[name] for name in target], dtype=np.int64)


def split_hand_input(
    values: Sequence[float],
    single_hand_side: str = "right",
) -> dict[str, np.ndarray]:
    """Split the established ROS2 63/126-float MediaPipe payload."""

    side = str(single_hand_side).strip().lower()
    if side not in {"left", "right"}:
        raise ValueError("single_hand_side must be left or right")
    array = np.asarray(values, dtype=np.float64).reshape(-1)
    if not np.isfinite(array).all():
        raise ValueError("hand input must contain finite values")
    if array.size == 63:
        return {side: array.reshape(21, 3).copy()}
    if array.size == 126:
        return {
            "right": array[:63].reshape(21, 3).copy(),
            "left": array[63:].reshape(21, 3).copy(),
        }
    raise ValueError(f"hand input must contain 63 or 126 floats, got {array.size}")


def clamp_hand_qpos(
    side: str,
    qpos: Sequence[float],
    limits: Sequence[Sequence[float]],
) -> np.ndarray:
    """Clamp official retarget output to the selected official URDF limits."""

    side = str(side).strip().lower()
    if side not in {"left", "right"}:
        raise ValueError("side must be left or right")
    vector = _as_joint_vector(qpos, f"{side} qpos")
    bound_array = np.asarray(limits, dtype=np.float64)
    if bound_array.shape != (HAND2_JOINT_COUNT, 2):
        raise ValueError(
            f"limits must have shape ({HAND2_JOINT_COUNT}, 2), got {bound_array.shape}"
        )
    if not np.isfinite(bound_array).all() or np.any(bound_array[:, 1] <= bound_array[:, 0]):
        raise ValueError("limits must be finite with upper greater than lower")
    return np.clip(vector, bound_array[:, 0], bound_array[:, 1])


def build_hand_command(
    sequence: int,
    timestamp_ns: int,
    values: Mapping[str, Sequence[float]],
) -> HandCommandFrame:
    """Build a command frame while preserving the official side/order names."""

    unknown = set(values) - {"left", "right"}
    if unknown:
        raise ValueError(f"unknown hand sides: {sorted(unknown)}")
    left_valid = "left" in values
    right_valid = "right" in values
    zero = np.zeros(HAND2_JOINT_COUNT, dtype=np.float64)
    left = _as_joint_vector(values["left"], "left qpos") if left_valid else zero
    right = _as_joint_vector(values["right"], "right qpos") if right_valid else zero
    if not left_valid and not right_valid:
        raise ValueError("at least one hand command is required")
    return HandCommandFrame(
        sequence=int(sequence),
        source_timestamp_ns=int(timestamp_ns),
        left=left.copy(),
        right=right.copy(),
        left_valid=left_valid,
        right_valid=right_valid,
    )


def encode_hand_command(frame: HandCommandFrame) -> bytes:
    """Encode a ``TJH2`` little-endian packet with CRC32."""

    if int(frame.sequence) <= 0 or int(frame.source_timestamp_ns) <= 0:
        raise ValueError("invalid hand command metadata")
    if not frame.left_valid and not frame.right_valid:
        raise ValueError("invalid hand command metadata: no valid side")
    left = _as_joint_vector(frame.left, "left qpos")
    right = _as_joint_vector(frame.right, "right qpos")
    flags = (
        (PACKET_FLAGS_LEFT_VALID if frame.left_valid else 0)
        | (PACKET_FLAGS_RIGHT_VALID if frame.right_valid else 0)
    )
    body = _PACKET_BODY.pack(
        PACKET_MAGIC,
        PACKET_VERSION,
        flags,
        HAND_COMMAND_PACKET_SIZE,
        int(frame.sequence),
        int(frame.source_timestamp_ns),
        *(left.tolist() + right.tolist()),
    )
    return body + _PACKET_CRC.pack(zlib.crc32(body) & 0xFFFFFFFF)


def decode_hand_command(packet: bytes) -> HandCommandFrame:
    """Decode and validate one ``TJH2`` packet."""

    if len(packet) != HAND_COMMAND_PACKET_SIZE:
        raise ValueError(f"wrong packet size: {len(packet)}")
    body = packet[:-_PACKET_CRC.size]
    received_crc = _PACKET_CRC.unpack(packet[-_PACKET_CRC.size :])[0]
    if zlib.crc32(body) & 0xFFFFFFFF != received_crc:
        raise ValueError("CRC mismatch")
    (
        magic,
        version,
        flags,
        declared_size,
        sequence,
        source_timestamp_ns,
        *values,
    ) = _PACKET_BODY.unpack(body)
    if magic != PACKET_MAGIC:
        raise ValueError("wrong magic")
    if version != PACKET_VERSION:
        raise ValueError("wrong version")
    if declared_size != HAND_COMMAND_PACKET_SIZE:
        raise ValueError("wrong declared size")
    if flags & ~PACKET_FLAGS_MASK or not flags:
        raise ValueError("invalid flags")
    left = np.asarray(values[:HAND2_JOINT_COUNT], dtype=np.float64)
    right = np.asarray(values[HAND2_JOINT_COUNT:], dtype=np.float64)
    _as_joint_vector(left, "left qpos")
    _as_joint_vector(right, "right qpos")
    return HandCommandFrame(
        sequence=sequence,
        source_timestamp_ns=source_timestamp_ns,
        left=left,
        right=right,
        left_valid=bool(flags & PACKET_FLAGS_LEFT_VALID),
        right_valid=bool(flags & PACKET_FLAGS_RIGHT_VALID),
    )


class OfficialWujiHand2Bridge:
    """Run the official Hand 2 retargeter for one or both input sides."""

    def __init__(
        self,
        repository_root: str | Path,
        single_hand_side: str = "right",
        left_config: str | Path | None = None,
        right_config: str | Path | None = None,
    ) -> None:
        self.repository_root = Path(repository_root).resolve()
        self.single_hand_side = str(single_hand_side).strip().lower()
        if self.single_hand_side not in {"left", "right"}:
            raise ValueError("single_hand_side must be left or right")
        self._retargeters = {}
        self._limits = {}
        self._qpos_permutations = {}
        self._last_sequences: dict[str, int] = {}
        config_paths = {
            "left": left_config
            or self.repository_root
            / "example/config/adaptive_analytical_wuji_glove_wuji_hand_2_left.yaml",
            "right": right_config
            or self.repository_root
            / "example/config/adaptive_analytical_wuji_glove_wuji_hand_2_right.yaml",
        }
        # Import lazily so protocol/unit tests do not require Pinocchio/NLopt.
        from wuji_retargeting import Retargeter

        for side in ("left", "right"):
            config = Path(config_paths[side]).resolve()
            self._retargeters[side] = Retargeter.from_yaml(str(config), hand_side=side)
            robot = self._retargeters[side].optimizer.robot
            self._limits[side] = np.asarray(
                robot.joint_limits,
                dtype=np.float64,
            )
            if self._limits[side].shape != (HAND2_JOINT_COUNT, 2):
                raise ValueError(
                    f"official Hand 2 {side} model exposes {self._limits[side].shape[0]} "
                    f"joint limits, expected {HAND2_JOINT_COUNT}"
                )
            self._qpos_permutations[side] = _joint_reorder_indices(
                robot.dof_joint_names,
                HAND2_JOINT_NAMES[side],
            )

    def retarget(
        self,
        values: Sequence[float],
        sequence: int,
        timestamp_ns: int,
    ) -> HandCommandFrame:
        points_by_side = split_hand_input(values, self.single_hand_side)
        qpos_by_side = {}
        for side, points in points_by_side.items():
            if sequence_gap_requires_reset(self._last_sequences.get(side), sequence):
                self._retargeters[side].reset()
            qpos = self._retargeters[side].retarget(points)
            qpos = clamp_hand_qpos(side, qpos, self._limits[side])
            qpos_by_side[side] = qpos[self._qpos_permutations[side]]
            self._last_sequences[side] = int(sequence)
        return build_hand_command(sequence, timestamp_ns, qpos_by_side)


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Manus /hand_input to TJ Wuji Hand 2 UDP")
    default_root = Path(__file__).resolve().parents[1]
    parser.add_argument("--repository-root", default=str(default_root))
    parser.add_argument("--input-topic", default="/hand_input")
    parser.add_argument("--single-hand-side", choices=("left", "right"), default="right")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=16000)
    parser.add_argument("--left-config")
    parser.add_argument("--right-config")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from rclpy.utilities import remove_ros_args
    from std_msgs.msg import Float32MultiArray

    raw_argv = sys.argv if argv is None else [sys.argv[0], *argv]
    args = _parse_args(remove_ros_args(raw_argv)[1:])
    bridge = OfficialWujiHand2Bridge(
        args.repository_root,
        single_hand_side=args.single_hand_side,
        left_config=args.left_config,
        right_config=args.right_config,
    )
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    destination = (args.host, args.port)

    rclpy.init(args=raw_argv)

    class BridgeNode(Node):
        def __init__(self) -> None:
            super().__init__("tj_wuji2_hand_bridge")
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.sequence = 0
            self.subscription = self.create_subscription(
                Float32MultiArray, args.input_topic, self._callback, qos
            )

        def _callback(self, message: Float32MultiArray) -> None:
            self.sequence += 1
            try:
                frame = bridge.retarget(
                    message.data,
                    self.sequence,
                    time.monotonic_ns(),
                )
                udp.sendto(encode_hand_command(frame), destination)
            except (ValueError, RuntimeError) as exc:
                self.get_logger().error(f"rejecting hand frame: {exc}")

    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        udp.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
