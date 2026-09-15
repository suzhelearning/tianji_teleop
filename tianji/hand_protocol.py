"""TJH2 v2 hand packet encoding shared by local input producers."""
from __future__ import annotations

import struct
import zlib

import numpy as np


PACKET_SIZE = 364
LEFT_VALID = 1
RIGHT_VALID = 2
# mujoco_robot.cpp::handJointStems(), not Pinocchio's traversal order.
JOINT_STEMS = (
    "thumb_cmc_flex", "thumb_cmc_abd", "thumb_mcp", "thumb_ip",
    "index_finger_mcp_flex", "index_finger_mcp_abd", "index_finger_pip", "index_finger_dip",
    "middle_finger_mcp_flex", "middle_finger_mcp_abd", "middle_finger_pip", "middle_finger_dip",
    "ring_finger_mcp_flex", "ring_finger_mcp_abd", "ring_finger_pip", "ring_finger_dip",
    "pinky_mcp_flex", "pinky_mcp_abd", "pinky_pip", "pinky_dip",
)
_PACKET_BODY = struct.Struct("<4sBBHQqqq40d")
_CRC = struct.Struct("<I")
_ZERO_JOINTS = (0.0,) * 20


def encode_packet(sequence: int, timestamp_ns: int, *, left=None, right=None,
                  left_timestamp_ns: int = 0, right_timestamp_ns: int = 0) -> bytes:
    """Encode TJH2 v2 with publication time and independently aged source poses."""
    if not isinstance(sequence, int) or not 0 < sequence < 2**64:
        raise ValueError("TJH2 sequence must be a positive uint64")
    if not isinstance(timestamp_ns, int) or not 0 < timestamp_ns < 2**63:
        raise ValueError("TJH2 timestamp must be a positive int64")
    flags = (LEFT_VALID if left is not None else 0) | (RIGHT_VALID if right is not None else 0)
    if not flags:
        raise ValueError("TJH2 requires at least one valid hand")
    values = []
    for hand, source_ns in ((left, left_timestamp_ns), (right, right_timestamp_ns)):
        if not isinstance(source_ns, int):
            raise ValueError("TJH2 side timestamp must be an int64")
        if hand is None:
            if source_ns != 0:
                raise ValueError("TJH2 absent side timestamp must be zero")
            values.extend(_ZERO_JOINTS)
        else:
            if not 0 < source_ns <= timestamp_ns:
                raise ValueError("TJH2 side timestamp must be positive and no later than publication")
            joints = np.asarray(hand, dtype=np.float64)
            if joints.shape != (20,) or not np.isfinite(joints).all():
                raise ValueError("TJH2 hand must contain 20 finite joint angles")
            values.extend(joints)
    body = _PACKET_BODY.pack(b"TJH2", 2, flags, PACKET_SIZE, sequence, timestamp_ns,
                             left_timestamp_ns, right_timestamp_ns, *values)
    return body + _CRC.pack(zlib.crc32(body) & 0xFFFFFFFF)
