"""Loopback TJVR fixtures for native DLS viewer integration."""
import math
import socket
import struct
import time
import zlib

PACKET_SIZE = 656

def reserve_udp_port():
    reservation = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    reservation.bind(("127.0.0.1", 0))
    port = reservation.getsockname()[1]
    reservation.close()
    return port


def quaternion(axis, angle):
    half = 0.5 * angle
    sine = math.sin(half)
    return (axis[0] * sine, axis[1] * sine, axis[2] * sine, math.cos(half))


def encode_packet(
    sequence,
    source_timestamp_ns,
    phase,
    orientation_amplitude,
    position_offset=0.0,
    button_pressed=False,
):
    left_position = (
        0.35 + position_offset + 0.015 * math.sin(phase),
        0.35 + 0.008 * math.cos(phase),
        1.25 + 0.010 * math.sin(0.5 * phase),
    )
    right_position = (
        0.35 + position_offset + 0.015 * math.sin(phase),
        -0.35 - 0.008 * math.cos(phase),
        1.25 + 0.010 * math.sin(0.5 * phase),
    )
    left_quaternion = quaternion(
        (0.0, 0.0, 1.0), orientation_amplitude * math.sin(phase)
    )
    right_quaternion = quaternion(
        (0.0, 1.0, 0.0), -orientation_amplitude * math.sin(phase)
    )
    bridge_send_ns = time.monotonic_ns()
    flags = 0xFF | (0x100 if button_pressed else 0)
    header = struct.pack(
        "<4sHHQQqqI",
        b"TJVR",
        4,
        PACKET_SIZE,
        sequence,
        9,
        source_timestamp_ns,
        bridge_send_ns,
        flags,
    )
    payload = struct.pack(
        "<14d",
        *left_position,
        *left_quaternion,
        *right_position,
        *right_quaternion,
    )
    # Keep the synthetic elbow directions away from world-down so an epoch
    # resynchronization exposes any accidental reset of the filtered arm-angle
    # reference back to the fallback posture.
    arm_directions = struct.pack("<6d", 0.0, 1.0, 0.0, 0.0, -1.0, 0.0)
    skeleton_points = (
        (0.0, 0.22, 1.121), (0.20, 0.30, 0.90),
        (0.42, 0.34, 1.02), (0.50, 0.35, 1.06),
        (0.0, -0.22, 1.121), (0.20, -0.30, 0.90),
        (0.42, -0.34, 1.02), (0.50, -0.35, 1.06),
    )
    skeleton = struct.pack("<24d", *(value for point in skeleton_points for value in point))
    skeleton_rotations = struct.pack(
        "<32d", *(value for _ in range(8) for value in (0.0, 0.0, 0.0, 1.0))
    )
    without_crc = header + payload + arm_directions + skeleton + skeleton_rotations
    assert len(without_crc) == 652
    packet = without_crc + struct.pack("<I", zlib.crc32(without_crc))
    assert len(packet) == PACKET_SIZE
    return packet
