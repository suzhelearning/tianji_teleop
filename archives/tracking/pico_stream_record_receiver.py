#!/usr/bin/env python3
"""
PC receiver for PicoStreamingServer.cs.

Usage:
  adb forward tcp:9999 tcp:9999
  python pico_stream_record_receiver.py --out pc_stream_records

Press A on the right PICO controller:
  record_flag=1 -> start a new local recording session
  record_flag=0 -> stop and close the current session
"""

from __future__ import annotations

import argparse
import csv
import socket
import struct
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


HOST = "127.0.0.1"
PORT = 9999
HEADER_SIZE = 14
MAX_PAYLOAD_LEN = 64 * 1024 * 1024

TYPE_CAM_LEFT = 0x01
TYPE_CAM_RIGHT = 0x02
TYPE_POSE_LEFT = 0x03
TYPE_POSE_RIGHT = 0x04
TYPE_POSE_HEAD = 0x05
TYPE_WORLD_RESET = 0x06
TYPE_CTRL_LEFT = 0x07
TYPE_CTRL_RIGHT = 0x08
TYPE_RECORD_FLAG = 0x09
TYPE_BLE1 = 0x10
TYPE_BLE2 = 0x11
TYPE_BODY_BASE = 0x20
TYPE_BODY_LAST = 0x37

BODY_JOINT_NAMES = [
    "Pelvis",
    "LEFT_HIP",
    "RIGHT_HIP",
    "SPINE1",
    "LEFT_KNEE",
    "RIGHT_KNEE",
    "SPINE2",
    "LEFT_ANKLE",
    "RIGHT_ANKLE",
    "SPINE3",
    "LEFT_FOOT",
    "RIGHT_FOOT",
    "NECK",
    "LEFT_COLLAR",
    "RIGHT_COLLAR",
    "HEAD",
    "LEFT_SHOULDER",
    "RIGHT_SHOULDER",
    "LEFT_ELBOW",
    "RIGHT_ELBOW",
    "LEFT_WRIST",
    "RIGHT_WRIST",
    "LEFT_HAND",
    "RIGHT_HAND",
]

CAM_TYPES = {
    TYPE_CAM_LEFT: "cam0",
    TYPE_CAM_RIGHT: "cam1",
}

XR_POSE_TYPES = {
    TYPE_POSE_LEFT: "imu0",
    TYPE_POSE_RIGHT: "imu1",
    TYPE_POSE_HEAD: "imu4",
}

BODY_POSE_TYPES = {
    TYPE_BODY_BASE + index: name
    for index, name in enumerate(BODY_JOINT_NAMES)
}

POSE_TYPES = {**XR_POSE_TYPES, **BODY_POSE_TYPES}

BLE_TYPES = {
    TYPE_BLE1: "imu2",
    TYPE_BLE2: "imu3",
}

POSE_HEADER = ["TimestampMs", "PosX", "PosY", "PosZ", "RotQx", "RotQy", "RotQz", "RotQw"]


def recv_exact(sock: socket.socket, nbytes: int) -> bytes:
    data = bytearray()
    while len(data) < nbytes:
        chunk = sock.recv(nbytes - len(data))
        if not chunk:
            raise ConnectionError("socket closed")
        data.extend(chunk)
    return bytes(data)


def read_frame(sock: socket.socket) -> tuple[int, int, bytes]:
    while True:
        magic = recv_exact(sock, 1)
        if magic == b"\xAB":
            break

    header_tail = recv_exact(sock, HEADER_SIZE - 1)
    frame_type = header_tail[0]
    timestamp_ms = struct.unpack_from("<q", header_tail, 1)[0]
    payload_len = struct.unpack_from("<I", header_tail, 9)[0]
    if payload_len > MAX_PAYLOAD_LEN:
        raise ValueError(f"payload too large: {payload_len}")
    payload = recv_exact(sock, payload_len) if payload_len else b""
    return frame_type, timestamp_ms, payload


class RecordingSession:
    def __init__(self, output_root: Path):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.root = output_root / f"pico_stream_{timestamp}"
        self.mav0 = self.root / "mav0"
        self.csv_files = {}
        self.csv_writers = {}
        self.frame_counts = {name: 0 for name in CAM_TYPES.values()}
        self.pose_counts = {name: 0 for name in POSE_TYPES.values()}
        self.ble_counts = {name: 0 for name in BLE_TYPES.values()}
        self._create_layout()

    def _create_layout(self) -> None:
        self.root.mkdir(parents=True, exist_ok=True)
        self.mav0.mkdir(parents=True, exist_ok=True)

        for cam_name in CAM_TYPES.values():
            (self.mav0 / cam_name / "data").mkdir(parents=True, exist_ok=True)
            self._open_csv(cam_name, self.mav0 / cam_name / "data.csv", ["TimestampMs", "Filename"])

        for pose_name in POSE_TYPES.values():
            (self.mav0 / pose_name).mkdir(parents=True, exist_ok=True)
            self._open_csv(pose_name, self.mav0 / pose_name / "data.csv", POSE_HEADER)

        for ble_name in BLE_TYPES.values():
            (self.mav0 / ble_name).mkdir(parents=True, exist_ok=True)
            self._open_csv(
                ble_name,
                self.mav0 / ble_name / "data.csv",
                ["TimestampMs", "Esp32TimestampMs", "PayloadHex"],
            )

        self._open_csv("events", self.root / "events.csv", ["TimestampMs", "Event", "Value"])
        (self.root / "recording_info.txt").write_text(
            "PICO TCP streaming recording\n"
            "Protocol: [0xAB][type:1B][timestamp_ms:int64][payload_len:uint32][payload]\n"
            "Pose payload: pos.xyz + rot.xyzw, 7 little-endian float32 values\n"
            "Body frame types: 0x20 + BodyTrackerRole enum value\n"
            f"Body folders: {', '.join(BODY_JOINT_NAMES)}\n",
            encoding="utf-8",
        )

    def _open_csv(self, key: str, path: Path, header: list[str]) -> None:
        file_obj = path.open("w", newline="", encoding="utf-8", buffering=1)
        writer = csv.writer(file_obj)
        writer.writerow(header)
        self.csv_files[key] = file_obj
        self.csv_writers[key] = writer

    def save_camera(self, cam_name: str, timestamp_ms: int, jpeg_payload: bytes) -> None:
        index = self.frame_counts[cam_name]
        filename = f"frame_{index:06d}.jpg"
        rel_path = f"data/{filename}"
        image_path = self.mav0 / cam_name / rel_path
        image_path.write_bytes(jpeg_payload)
        self.csv_writers[cam_name].writerow([timestamp_ms, rel_path])
        self.frame_counts[cam_name] = index + 1

    def save_pose(self, pose_name: str, timestamp_ms: int, payload: bytes) -> None:
        if len(payload) < 28:
            return
        values = struct.unpack_from("<7f", payload, 0)
        self.csv_writers[pose_name].writerow([timestamp_ms, *[f"{value:.6f}" for value in values]])
        self.pose_counts[pose_name] += 1

    def save_ble(self, ble_name: str, timestamp_ms: int, payload: bytes) -> None:
        esp32_timestamp = ""
        data = payload
        if len(payload) >= 4:
            esp32_timestamp = struct.unpack_from("<I", payload, 0)[0]
            data = payload[4:]
        self.csv_writers[ble_name].writerow([timestamp_ms, esp32_timestamp, data.hex()])
        self.ble_counts[ble_name] += 1

    def save_event(self, timestamp_ms: int, event: str, value: str | int | float) -> None:
        self.csv_writers["events"].writerow([timestamp_ms, event, value])

    def close(self) -> None:
        for file_obj in self.csv_files.values():
            file_obj.flush()
            file_obj.close()


class Receiver:
    def __init__(self, host: str, port: int, output_root: Path):
        self.host = host
        self.port = port
        self.output_root = output_root
        self.recording = False
        self.session: RecordingSession | None = None
        self.last_stats_time = time.monotonic()
        self.frame_total = 0

    def start_session(self, timestamp_ms: int) -> None:
        if self.recording:
            return
        self.session = RecordingSession(self.output_root)
        self.recording = True
        self.session.save_event(timestamp_ms, "record_flag", 1)
        print(f"[REC] start -> {self.session.root}")

    def stop_session(self, timestamp_ms: int) -> None:
        if not self.recording:
            return
        assert self.session is not None
        self.session.save_event(timestamp_ms, "record_flag", 0)
        root = self.session.root
        self.session.close()
        self.session = None
        self.recording = False
        print(f"[REC] stop  -> {root}")

    def handle_frame(self, frame_type: int, timestamp_ms: int, payload: bytes) -> None:
        self.frame_total += 1

        if frame_type == TYPE_RECORD_FLAG:
            flag = 1 if payload and payload[0] != 0 else 0
            if flag:
                self.start_session(timestamp_ms)
            else:
                self.stop_session(timestamp_ms)
            return

        if not self.recording or self.session is None:
            return

        if frame_type in CAM_TYPES:
            self.session.save_camera(CAM_TYPES[frame_type], timestamp_ms, payload)
        elif frame_type in POSE_TYPES:
            self.session.save_pose(POSE_TYPES[frame_type], timestamp_ms, payload)
        elif frame_type in BLE_TYPES:
            self.session.save_ble(BLE_TYPES[frame_type], timestamp_ms, payload)
        elif frame_type == TYPE_WORLD_RESET:
            yaw = struct.unpack_from("<f", payload, 0)[0] if len(payload) >= 4 else ""
            self.session.save_event(timestamp_ms, "world_reset_yaw", yaw)
        elif frame_type in (TYPE_CTRL_LEFT, TYPE_CTRL_RIGHT):
            side = "left" if frame_type == TYPE_CTRL_LEFT else "right"
            self.session.save_event(timestamp_ms, f"controller_{side}", payload.hex())

        self.print_stats_periodically()

    def print_stats_periodically(self) -> None:
        now = time.monotonic()
        if now - self.last_stats_time < 2.0:
            return
        self.last_stats_time = now
        if not self.session:
            return
        cam = ", ".join(f"{k}:{v}" for k, v in self.session.frame_counts.items())
        body = ", ".join(f"{k}:{self.session.pose_counts[k]}" for k in BODY_JOINT_NAMES)
        print(f"[Stats] frames={self.frame_total} cam=({cam}) body=({body})")

    def run(self) -> None:
        self.output_root.mkdir(parents=True, exist_ok=True)
        while True:
            try:
                print(f"[Net] connecting {self.host}:{self.port} ...")
                with socket.create_connection((self.host, self.port), timeout=5) as sock:
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    sock.settimeout(None)
                    print("[Net] connected. Waiting for A-button record flag...")
                    while True:
                        frame_type, timestamp_ms, payload = read_frame(sock)
                        self.handle_frame(frame_type, timestamp_ms, payload)
            except KeyboardInterrupt:
                print("\n[Main] interrupted")
                self.stop_session(0)
                return
            except Exception as exc:
                print(f"[Net] disconnected/error: {exc}. retry in 2s")
                self.stop_session(0)
                time.sleep(2)


def maybe_adb_forward(port: int) -> None:
    cmd = ["adb", "forward", f"tcp:{port}", f"tcp:{port}"]
    print("[ADB]", " ".join(cmd))
    subprocess.run(cmd, check=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Receive and record PICO TCP stream with body tracking.")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--out", default="pc_stream_records", help="output root directory")
    parser.add_argument("--adb-forward", action="store_true", help="run adb forward before connecting")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.adb_forward:
        maybe_adb_forward(args.port)
    receiver = Receiver(args.host, args.port, Path(args.out))
    receiver.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
