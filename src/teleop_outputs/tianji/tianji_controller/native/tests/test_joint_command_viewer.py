"""TJRC and hand-packet fixtures for the DLS viewer integration."""
import math
import struct
import time
import zlib

def hand_packet(sequence, left_valid, right_valid):
    left = tuple(-0.005 * (i + 1) for i in range(20))
    right = tuple(0.01 * (i + 1) for i in range(20))
    flags = int(left_valid) | (int(right_valid) << 1)
    stamp = time.monotonic_ns()
    body = struct.pack("<4sBBHQqqq40d", b"TJH2", 2, flags,
                       364, sequence, stamp, stamp if left_valid else 0,
                       stamp if right_valid else 0, *left, *right)
    return body + struct.pack("<I", zlib.crc32(body)), left, right


def decode(packet):
    assert len(packet) == 468, len(packet)
    assert zlib.crc32(packet[:-4]) == struct.unpack_from("<I", packet, 464)[0]
    magic, version, flags, size, sequence, stamp, epoch, *q = struct.unpack(
        "<4sBBHQqQ54d", packet[:-4])
    assert (magic, version, size) == (b"TJRC", 2, 468)
    assert flags & ~7 == 0 and sequence > 0 and stamp > 0
    assert all(math.isfinite(value) for value in q)
    assert 0 <= time.monotonic_ns() - stamp < 1_000_000_000
    return flags, sequence, stamp, epoch, tuple(q)


