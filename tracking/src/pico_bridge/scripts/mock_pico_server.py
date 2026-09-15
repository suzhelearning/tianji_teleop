#!/usr/bin/env python3
"""
Mock PicoStreamingServer for integration testing.

Listens on 127.0.0.1:9999 and pushes fake frames in the official wire format.
Useful when the real PICO isn't connected.

Frame layout: [0xAB][type:1B][ts_ms:8B LE i64][payload_len:4B LE u32][payload]
"""
import argparse
import math
import socket
import struct
import time


XR_POSE_TYPES = ((0x03, 0.0), (0x04, 1.0), (0x05, 2.0))
BODY_POSE_TYPES = tuple((0x20 + index, 3.0 + index) for index in range(24))
POSE_TYPES = XR_POSE_TYPES + BODY_POSE_TYPES


def make_header(frame_type: int, ts_ms: int, payload_len: int) -> bytes:
    return struct.pack("<BBqI", 0xAB, frame_type, ts_ms, payload_len)


def tiny_jpeg() -> bytes:
    # Minimal valid-looking "JPEG" — a real JPEG SOI+EOI so cv2.imdecode returns None cleanly.
    # (Integration tests don't need a decodable image, just bytes that travel end-to-end.)
    return b"\xFF\xD8\xFF\xD9" + b"FAKE_JPEG"


def pose_payload(t: float, offset: float) -> bytes:
    return struct.pack(
        "<7f",
        math.sin(t),       math.cos(t),       offset,
        0.0, 0.0, 0.0, 1.0,
    )


def ble_payload(ts_ms: int, payload_byte: int) -> bytes:
    return struct.pack("<I", ts_ms) + bytes([payload_byte, payload_byte ^ 0xFF])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=9999)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--duration", type=float, default=0.0,
                    help="Run for this many seconds (0 = forever)")
    ap.add_argument("--record-cycle", type=float, default=0.0,
                    help="Toggle the 0x09 record flag every N seconds (0 = never send)")
    args = ap.parse_args()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", args.port))
    srv.listen(1)
    print(f"[mock] listening on 127.0.0.1:{args.port}")

    while True:
        conn, addr = srv.accept()
        print(f"[mock] client connected: {addr}")
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        start = time.time()
        frame_idx = 0
        record_flag = False
        last_toggle = start
        try:
            while args.duration == 0 or (time.time() - start) < args.duration:
                ts_ms = int((time.time() - start) * 1000)
                t = ts_ms / 1000.0

                # Cameras (0x01, 0x02)
                for ftype in (0x01, 0x02):
                    jp = tiny_jpeg()
                    conn.sendall(make_header(ftype, ts_ms, len(jp)) + jp)

                # Poses: controllers + head (0x03-0x05), complete BodyTrackerRole
                # frame in canonical order (0x20-0x37).
                for ftype, off in POSE_TYPES:
                    p = pose_payload(t, off)
                    conn.sendall(make_header(ftype, ts_ms, len(p)) + p)

                # Record flag (0x09)
                if args.record_cycle > 0 and time.time() - last_toggle >= args.record_cycle:
                    record_flag = not record_flag
                    last_toggle = time.time()
                    conn.sendall(make_header(0x09, ts_ms, 1) + bytes([1 if record_flag else 0]))
                    print(f"[mock] record_flag -> {int(record_flag)}")

                # BLE (0x10, 0x11)
                for ftype, byte in ((0x10, 0xA0), (0x11, 0xB0)):
                    p = ble_payload(ts_ms, byte + (frame_idx & 0x0F))
                    conn.sendall(make_header(ftype, ts_ms, len(p)) + p)

                frame_idx += 1
                time.sleep(1.0 / args.fps)
        except (BrokenPipeError, ConnectionResetError):
            print("[mock] client disconnected")
        finally:
            conn.close()
            if args.duration > 0:
                break

    srv.close()


if __name__ == "__main__":
    main()
