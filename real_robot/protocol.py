"""Loopback-only final-joint command format emitted by the existing controller."""
from dataclasses import dataclass
import math
import struct
import zlib

BODY = struct.Struct("<4sBBHQqQ54d")
CRC = struct.Struct("<I")
PACKET_SIZE = BODY.size + CRC.size
ARMS_READY = 1
RIGHT_HAND_READY = 2
LEFT_HAND_READY = 4
DEVICE_READY_FLAGS = {"arms": ARMS_READY, "left_hand": LEFT_HAND_READY, "right_hand": RIGHT_HAND_READY}


@dataclass(frozen=True)
class CommandFrame:
    sequence: int
    timestamp_ns: int
    tracking_epoch: int
    flags: int
    left_arm: tuple[float, ...]
    right_arm: tuple[float, ...]
    left_hand: tuple[float, ...]
    right_hand: tuple[float, ...]

    def positions(self, device: str) -> tuple[float, ...]:
        if device == "arms":
            return self.left_arm + self.right_arm
        if device == "left_hand":
            return self.left_hand
        if device == "right_hand":
            return self.right_hand
        raise ValueError(f"unsupported hardware device: {device}")


def decode_packet(packet: bytes) -> CommandFrame:
    if len(packet) != PACKET_SIZE:
        raise ValueError(f"expected {PACKET_SIZE} command bytes, got {len(packet)}")
    if CRC.unpack_from(packet, BODY.size)[0] != zlib.crc32(packet[:BODY.size]) & 0xFFFFFFFF:
        raise ValueError("command CRC mismatch")
    magic, version, flags, size, sequence, timestamp, epoch, *positions = BODY.unpack_from(packet)
    if magic != b"TJRC" or version != 2 or size != PACKET_SIZE or flags & ~7:
        raise ValueError("unsupported command protocol header")
    if sequence <= 0 or timestamp <= 0 or not all(map(math.isfinite, positions)):
        raise ValueError("invalid command sequence, timestamp or joint position")
    return CommandFrame(sequence, timestamp, epoch, flags,
                        tuple(positions[:7]), tuple(positions[7:14]),
                        tuple(positions[14:34]), tuple(positions[34:54]))
