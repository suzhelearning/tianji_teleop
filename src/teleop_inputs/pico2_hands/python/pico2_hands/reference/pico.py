"""PICO_2 TCP decoding and pure coordinate/mapping helpers."""
from __future__ import annotations

import struct
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from .models import (
    PICO_JOINT_NAMES,
    PicoRawFrame,
    PicoRawHand,
    PicoRawJoint,
)


MAGIC = 0xAB
MESSAGE_TYPE = 0x40
PROTOCOL_VERSION = 1
JOINT_COUNT = 26
HEADER = struct.Struct("<BBqI")
PAYLOAD_HEADER = struct.Struct("<BBBB")
POSE = struct.Struct("<7f")
JOINT_BLOCK = struct.Struct("<BBBB7ff")
PAYLOAD_BYTES = 1968
PACKET_BYTES = HEADER.size + PAYLOAD_BYTES
PICO_TO_MEDIAPIPE = [
    1,
    2, 3, 4, 5,
    7, 8, 9, 10,
    12, 13, 14, 15,
    17, 18, 19, 20,
    22, 23, 24, 25,
]


def _validate_pose(values: Sequence[float], field: str, *, allow_zero_quaternion: bool = False) -> np.ndarray:
    pose = np.asarray(values, dtype=np.float64)
    if pose.shape != (7,) or not np.isfinite(pose).all():
        raise ValueError(f"{field} must contain seven finite values")
    if not allow_zero_quaternion and float(np.linalg.norm(pose[3:])) < 1.0e-12:
        raise ValueError(f"{field} quaternion must be non-zero")
    return pose


def _read_pose(view: memoryview, offset: int, field: str) -> tuple[np.ndarray, int]:
    end = offset + POSE.size
    if end > len(view):
        raise ValueError(f"truncated {field}")
    values = POSE.unpack_from(view, offset)
    return _validate_pose(values, field, allow_zero_quaternion=True), end


def parse_pico_packet(
    packet: bytes,
    *,
    receiver_instance_id: str,
    connection_generation: int,
    receiver_frame_sequence: int,
    received_timestamp_ns: int,
) -> PicoRawFrame:
    """Decode one complete PICO_2 packet.

    The caller owns TCP framing.  This function accepts exactly one known v1
    packet and retains the original packet bytes in the returned frame.
    """
    if not isinstance(packet, (bytes, bytearray)) or len(packet) < HEADER.size:
        raise ValueError("PICO packet is shorter than its header")
    magic, message_type, timestamp_ms, payload_length = HEADER.unpack_from(packet, 0)
    if magic != MAGIC:
        raise ValueError(f"unexpected PICO packet magic: {magic:#x}")
    if message_type != MESSAGE_TYPE:
        raise ValueError(f"unexpected PICO message type: {message_type}")
    if timestamp_ms < 0:
        raise ValueError("PICO source timestamp must be non-negative")
    if payload_length != PAYLOAD_BYTES or len(packet) != HEADER.size + payload_length:
        raise ValueError("PICO packet has an unsupported payload length")

    view = memoryview(packet)[HEADER.size:]
    version, flags, joint_count, reserved = PAYLOAD_HEADER.unpack_from(view, 0)
    if version != PROTOCOL_VERSION:
        raise ValueError(f"unsupported PICO protocol version: {version}")
    if joint_count != JOINT_COUNT:
        raise ValueError(f"unsupported PICO joint count: {joint_count}")
    if reserved != 0:
        raise ValueError("PICO payload header reserved byte is non-zero")
    if flags & ~0x07:
        raise ValueError("PICO v1 contains unknown flags")

    offset = PAYLOAD_HEADER.size
    head_pose, offset = _read_pose(view, offset, "head_pose")
    head_valid = bool(flags & 0x01)
    hands: dict[str, PicoRawHand] = {}
    for side, flag in (("left", 0x02), ("right", 0x04)):
        if offset + 4 > len(view):
            raise ValueError(f"truncated {side} hand header")
        hand_valid_byte, r0, r1, r2 = struct.unpack_from("<BBBB", view, offset)
        offset += 4
        if hand_valid_byte not in (0, 1) or (r0, r1, r2) != (0, 0, 0):
            raise ValueError(f"invalid {side} hand header")
        wrist_pose, offset = _read_pose(view, offset, f"{side}_wrist_pose")
        wrist_valid = bool(hand_valid_byte) and bool(flags & flag)
        joints: list[PicoRawJoint] = []
        for index, name in enumerate(PICO_JOINT_NAMES):
            end = offset + JOINT_BLOCK.size
            if end > len(view):
                raise ValueError(f"truncated {side} joint {index}")
            joint_valid_byte, r0, r1, r2, *values = JOINT_BLOCK.unpack_from(view, offset)
            offset = end
            if joint_valid_byte not in (0, 1) or (r0, r1, r2) != (0, 0, 0):
                raise ValueError(f"invalid {side} joint {index} validity/reserved bytes")
            pose = _validate_pose(values[:7], f"{side}_joint_{index}", allow_zero_quaternion=True)
            radius = float(values[7])
            if not np.isfinite(radius) or radius < 0.0:
                raise ValueError(f"invalid {side} joint {index} radius")
            joints.append(PicoRawJoint(index, name, bool(joint_valid_byte), pose, radius))
        hands[side] = PicoRawHand(
            valid=bool(hand_valid_byte) and bool(flags & flag),
            wrist_valid=wrist_valid,
            wrist_pose=wrist_pose,
            joints=tuple(joints),
        )
    if offset != len(view):
        raise ValueError("PICO v1 packet contains trailing bytes")
    return PicoRawFrame(
        source_timestamp_ms=int(timestamp_ms),
        protocol_version=version,
        flags=flags,
        joint_count=joint_count,
        head_valid=head_valid,
        head_pose=head_pose,
        hands=hands,
        raw_packet=bytes(packet),
        received_timestamp_ns=received_timestamp_ns,
        receiver_instance_id=receiver_instance_id,
        connection_generation=connection_generation,
        receiver_frame_sequence=receiver_frame_sequence,
    )


def pico_to_mediapipe(
    joint_positions_m: Sequence[Sequence[float]],
    joint_valid: Sequence[bool],
) -> tuple[np.ndarray, np.ndarray]:
    """Select the 21 canonical points from a PICO 26-point hand."""
    points = np.asarray(joint_positions_m, dtype=np.float64)
    if points.shape != (26, 3) or not np.isfinite(points).all():
        raise ValueError("PICO joint positions must be finite with shape (26, 3)")
    valid = np.asarray(joint_valid, dtype=np.bool_)
    if valid.shape != (26,):
        raise ValueError("PICO joint validity must have shape (26,)")
    selected_valid = valid[np.asarray(PICO_TO_MEDIAPIPE)]
    selected = points[np.asarray(PICO_TO_MEDIAPIPE)].copy()
    selected[~selected_valid] = 0.0
    return selected, selected_valid.copy()


def tracking_pose_to_current_head(head_pose: Sequence[float], wrist_pose: Sequence[float]) -> np.ndarray:
    """Express a tracking-space wrist pose in the current head frame."""
    head = _validate_pose(head_pose, "head_pose")
    wrist = _validate_pose(wrist_pose, "wrist_pose")
    head_rotation = Rotation.from_quat(head[3:] / np.linalg.norm(head[3:]))
    wrist_rotation = Rotation.from_quat(wrist[3:] / np.linalg.norm(wrist[3:]))
    relative_position = head_rotation.inv().apply(wrist[:3] - head[:3])
    relative_rotation = head_rotation.inv() * wrist_rotation
    return np.concatenate((relative_position, relative_rotation.as_quat()))


__all__ = [
    "HEADER",
    "JOINT_COUNT",
    "MAGIC",
    "MESSAGE_TYPE",
    "PACKET_BYTES",
    "PAYLOAD_BYTES",
    "PICO_TO_MEDIAPIPE",
    "PROTOCOL_VERSION",
    "parse_pico_packet",
    "pico_to_mediapipe",
    "tracking_pose_to_current_head",
]
