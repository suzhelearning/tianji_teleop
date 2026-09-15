#!/usr/bin/env python3
"""Exercise the real headless controller using loopback-only synthetic inputs."""

import argparse
import ast
import math
from pathlib import Path
import select
import signal
import socket
import struct
import subprocess
import time
import zlib

from test_pico_viewer_integration import encode_packet, reserve_udp_port


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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--viewer", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--continuous", action="store_true")
    args = parser.parse_args()
    initial = {}
    for line in Path(args.config).read_text().splitlines():
        for side in ("left", "right"):
            key = f"initial_{side}_q_rad:"
            if line.strip().startswith(key):
                initial[side] = ast.literal_eval(line.split(":", 1)[1].strip())
    expected_initial = tuple(initial["left"] + initial["right"])
    pico_port, hand_port = reserve_udp_port(), reserve_udp_port()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver, \
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        receiver.bind(("127.0.0.1", 0))
        command = [args.viewer, "--config", args.config, "--model", args.model,
                   "--headless", "--pico-port", str(pico_port), "--hand-teleop",
                   "--hand-port", str(hand_port), "--joint-command-port",
                   str(receiver.getsockname()[1])]
        command += ["--continuous"] if args.continuous else ["--duration", "3.0"]
        process = subprocess.Popen(command, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, text=True,
                                   cwd=Path(args.config).resolve().parents[1])
        records = []
        try:
            receiver.settimeout(10)
            first = decode(receiver.recv(512))
            assert first[0] == 0, first
            # Export occurs after the first real QP solve, whose numerical
            # residue is not bit-identical to the seed (observed max 1.75e-9 rad).
            # One microradian is negligible physically but still detects an
            # incorrect configured/measured startup or a joint-order mistake.
            assert all(abs(a - b) < 1e-6 for a, b in zip(first[4][:14], expected_initial)), {
                "configured": expected_initial, "exported": first[4][:14],
                "delta": tuple(a - b for a, b in zip(first[4][:14], expected_initial)),
            }
            records.append((0.0, first))
            start = time.monotonic()
            next_send = start
            sequence = 1
            expected_left = expected_right = None
            while time.monotonic() - start < 2.35:
                now = time.monotonic()
                elapsed = now - start
                if now >= next_send:
                    # PICO first goes live, then stops. Hands independently
                    # transition LEFT-only -> both -> LEFT-only -> RIGHT-only
                    # -> neither, without changing either side's joint values.
                    if 0.15 < elapsed < 1.5:
                        # The continuous case also pauses/resumes in the SAME
                        # tracking epoch: export must remain inhibited after the
                        # session resets, even though fresh PICO data resumes.
                        pico_packet = bytearray(encode_packet(
                            sequence, time.monotonic_ns(), 0.0, 0.0,
                            button_pressed=args.continuous and 1.0 < elapsed < 1.2))
                        if not args.continuous and 0.9 < elapsed < 1.15:
                            # Valid new sequence/fresh receive time must not
                            # launder an old local bridge timestamp.
                            struct.pack_into("<q", pico_packet, 32,
                                             time.monotonic_ns() - 500_000_000)
                            struct.pack_into("<I", pico_packet, len(pico_packet) - 4,
                                             zlib.crc32(pico_packet[:-4]))
                        sender.sendto(pico_packet, ("127.0.0.1", pico_port))
                    if elapsed < 1.8:
                        packet, expected_left, expected_right = hand_packet(
                            sequence, elapsed < 1.2,
                            0.3 < elapsed < 0.8 or elapsed > 1.2)
                        sender.sendto(packet, ("127.0.0.1", hand_port))
                    sequence += 1
                    next_send = now + 0.01
                if select.select([receiver], [], [], 0.003)[0]:
                    frame = decode(receiver.recv(512))
                    assert frame[1] > records[-1][1][1]
                    assert frame[2] > records[-1][1][2]
                    records.append((time.monotonic() - start, frame))
            assert any(frame[0] & 1 for _, frame in records), "arms never became ready"
            assert any(frame[0] == 7 and frame[4][14:34] == expected_left
                       and frame[4][34:54] == expected_right
                       for _, frame in records), "all-ready mask/dual-hand order/payload mismatch"
            left_only_start = [frame for elapsed, frame in records if 0.1 < elapsed < 0.25]
            assert left_only_start and any(frame[0] & 4 for frame in left_only_start), \
                "LEFT-only input never became ready"
            assert all(not frame[0] & 2 for frame in left_only_start), \
                "missing RIGHT input was marked ready"
            left_only_later = [frame for elapsed, frame in records if 1.05 < elapsed < 1.2]
            assert left_only_later and all(not frame[0] & 2 for frame in left_only_later), \
                "LEFT-only traffic resurrected RIGHT hand readiness"
            right_only = [frame for elapsed, frame in records if 1.5 < elapsed < 1.8]
            assert right_only and any(frame[0] & 2 for frame in right_only), \
                "RIGHT-only input never became ready"
            assert all(not frame[0] & 4 for frame in right_only), \
                "RIGHT-only traffic resurrected LEFT hand readiness"
            expired = [frame for elapsed, frame in records if elapsed > 2.05]
            assert expired and all(frame[0] == 0 for frame in expired), \
                "stale inputs remained ready"
            assert any(frame[3] == 9 for _, frame in records), "PICO epoch missing"
            if not args.continuous:
                assert all(not frame[0] & 1 for elapsed, frame in records
                           if 0.97 < elapsed < 1.1), "delayed bridge data marked arms ready"
                assert any(frame[0] & 1 for elapsed, frame in records
                           if 1.25 < elapsed < 1.45), "fresh bridge input did not recover"
            if args.continuous:
                assert all(not frame[0] & 1 for elapsed, frame in records if elapsed > 1.3), \
                    "same-epoch resume restored hardware arms without controller restart"
                assert process.poll() is None, "continuous controller stopped at finite default"
                process.send_signal(signal.SIGTERM)
            stdout, stderr = process.communicate(timeout=10)
            assert process.returncode == 0, (stdout, stderr)
            print(f"joint-command loopback smoke: {len(records)} packets, initial q, "
                  "live arms/both hands, independent hand expiry, PICO expiry, epoch and CRC verified")
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate()


if __name__ == "__main__":
    main()
