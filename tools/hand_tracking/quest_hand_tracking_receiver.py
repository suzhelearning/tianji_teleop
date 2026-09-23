#!/usr/bin/env python3
"""Receive Quest hand tracking frames over an ADB-forwarded TCP socket.

The receiver creates ``adb forward tcp:10002 tcp:10002`` automatically by
default, then connects to the forwarded local port. Use ``--no-adb-forward``
when another process owns the ADB forwarding lifecycle.

    python tools/hand_tracking/quest_hand_tracking_receiver.py --visualize

The receiver uses only the Python standard library until ``--visualize`` is
requested. Visualization dependencies are in requirements-visualize.txt.
Coordinates are FLU: X forward, Y left and Z up.
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

MAGIC = 0xAB
TYPE_HAND_FRAME = 0x40
PROTOCOL_VERSION = 1
HEADER = struct.Struct("<BBqI")
POSE = struct.Struct("<7f")
JOINT_COUNT = 26
JOINT_RECORD_BYTES = 4 + POSE.size + 4
HAND_BLOCK_BYTES = 4 + POSE.size + JOINT_COUNT * JOINT_RECORD_BYTES
PAYLOAD_BYTES = 4 + POSE.size + 2 * HAND_BLOCK_BYTES

# OpenXR XRHand joint order (0 = palm, 1 = wrist).  These indices are also the order in every frame.
JOINT_NAMES = (
    "palm", "wrist",
    "thumb_metacarpal", "thumb_proximal", "thumb_distal", "thumb_tip",
    "index_metacarpal", "index_proximal", "index_intermediate", "index_distal", "index_tip",
    "middle_metacarpal", "middle_proximal", "middle_intermediate", "middle_distal", "middle_tip",
    "ring_metacarpal", "ring_proximal", "ring_intermediate", "ring_distal", "ring_tip",
    "little_metacarpal", "little_proximal", "little_intermediate", "little_distal", "little_tip",
)

BONE_PAIRS = (
    (1, 0), (1, 2), (1, 6), (1, 11), (1, 16), (1, 21),
    (2, 3), (3, 4), (4, 5),
    (6, 7), (7, 8), (8, 9), (9, 10),
    (11, 12), (12, 13), (13, 14), (14, 15),
    (16, 17), (17, 18), (18, 19), (19, 20),
    (21, 22), (22, 23), (23, 24), (24, 25),
)


def recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = sock.recv(size - len(chunks))
        if not chunk:
            raise ConnectionError("Quest closed the TCP connection")
        chunks.extend(chunk)
    return bytes(chunks)


def parse_pose(payload: memoryview, offset: int) -> Tuple[Dict[str, Any], int]:
    values = POSE.unpack_from(payload, offset)
    pose = {
        "pos": [float(values[0]), float(values[1]), float(values[2])],
        "rot": [float(values[3]), float(values[4]), float(values[5]), float(values[6])],
    }
    return pose, offset + POSE.size


def parse_hand_frame(timestamp_ms: int, payload: bytes) -> Dict[str, Any]:
    if len(payload) < PAYLOAD_BYTES:
        raise ValueError(f"hand frame is {len(payload)} bytes, expected {PAYLOAD_BYTES}")

    view = memoryview(payload)
    version, flags, joint_count, _reserved = struct.unpack_from("<BBBB", view, 0)
    if version != PROTOCOL_VERSION:
        raise ValueError(f"unsupported hand frame version {version}")
    if joint_count != JOINT_COUNT:
        raise ValueError(f"unsupported joint count {joint_count}")

    offset = 4
    head, offset = parse_pose(view, offset)
    head["valid"] = bool(flags & 0x01)

    hands: Dict[str, Any] = {}
    for side, bit in (("left", 0x02), ("right", 0x04)):
        hand_valid = bool(view[offset])
        offset += 4  # valid + three reserved bytes
        wrist, offset = parse_pose(view, offset)
        wrist["valid"] = hand_valid
        joints = []
        for index, name in enumerate(JOINT_NAMES):
            joint_valid = bool(view[offset])
            offset += 4
            joint, offset = parse_pose(view, offset)
            radius = struct.unpack_from("<f", view, offset)[0]
            offset += 4
            joint["index"] = index
            joint["name"] = name
            joint["valid"] = joint_valid
            joint["radius"] = float(radius)
            joints.append(joint)
        hands[side] = {
            "valid": hand_valid and bool(flags & bit),
            "wrist": wrist,
            "joints": joints,
        }

    if offset != len(payload):
        # Future protocol extensions may append fields.  Parsing remains safe,
        # but expose the size so a caller can decide whether to reject it.
        extra_bytes = len(payload) - offset
    else:
        extra_bytes = 0

    return {
        "timestamp_ms": int(timestamp_ms),
        "flags": int(flags),
        "head": head,
        "hands": hands,
        "extra_bytes": extra_bytes,
    }


class HandTrackingReceiver:
    def __init__(self, host: str, port: int, reconnect_seconds: float,
                 save_jsonl: Optional[Path], print_frames: bool,
                 adb_path: str = "adb", adb_serial: Optional[str] = None,
                 adb_device_port: Optional[int] = None,
                 auto_adb_forward: bool = True) -> None:
        self.host = host
        self.port = port
        self.reconnect_seconds = reconnect_seconds
        self.save_jsonl = save_jsonl
        self.print_frames = print_frames
        self.adb_path = adb_path
        self.adb_serial = adb_serial
        self.adb_device_port = adb_device_port if adb_device_port is not None else port
        self.auto_adb_forward = auto_adb_forward
        self._adb_forward_announced = False
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest: Optional[Dict[str, Any]] = None
        self.frames_received = 0
        self.rx_rate_hz = 0.0
        self.last_error = ""
        self._last_print = 0.0
        self._rate_window_start = time.monotonic()
        self._rate_window_frames = 0
        self._last_frame_monotonic = 0.0
        self._json_file = None

    def start(self) -> threading.Thread:
        thread = threading.Thread(target=self.run, name="quest-hand-receiver", daemon=True)
        thread.start()
        return thread

    def stop(self) -> None:
        self.stop_event.set()
        if self._json_file is not None:
            self._json_file.close()
            self._json_file = None

    def ensure_adb_forward(self) -> None:
        """Create or refresh the USB ADB TCP forward before connecting.

        ``adb forward`` is idempotent for the same local/device port. It is
        intentionally re-run after every failed socket connection so a Quest
        USB reconnect does not require a manual shell command.
        """
        if not self.auto_adb_forward:
            return

        command = [self.adb_path]
        if self.adb_serial:
            command.extend(("-s", self.adb_serial))
        command.extend((
            "forward",
            f"tcp:{self.port}",
            f"tcp:{self.adb_device_port}",
        ))
        try:
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=5.0,
                check=False,
            )
        except FileNotFoundError as exc:
            raise RuntimeError(
                f"ADB executable not found: {self.adb_path!r}; "
                "install Android platform-tools or pass --adb-path"
            ) from exc
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError("adb forward timed out; check the USB connection") from exc

        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            raise RuntimeError(
                f"adb forward failed (exit {result.returncode})"
                + (f": {detail}" if detail else "")
            )

        if not self._adb_forward_announced:
            print(
                f"[ADB] forwarding tcp:{self.port} -> tcp:{self.adb_device_port}",
                flush=True,
            )
            self._adb_forward_announced = True

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                self.ensure_adb_forward()
                print(f"[Quest] connecting to {self.host}:{self.port} ...", flush=True)
                with socket.create_connection((self.host, self.port), timeout=3.0) as sock:
                    sock.settimeout(None)
                    print("[Quest] connected", flush=True)
                    self.receive_socket(sock)
            except (OSError, ConnectionError, ValueError, RuntimeError) as exc:
                self.last_error = str(exc)
                if not self.stop_event.is_set():
                    print(f"[Quest] {exc}; retrying in {self.reconnect_seconds:g}s", flush=True)
                    self.stop_event.wait(self.reconnect_seconds)

    def receive_socket(self, sock: socket.socket) -> None:
        while not self.stop_event.is_set():
            # Read the magic byte separately so a bad byte can be discarded and
            # the next valid frame can still be found without restarting ADB.
            magic = recv_exact(sock, 1)[0]
            if magic != MAGIC:
                continue
            tail = recv_exact(sock, HEADER.size - 1)
            frame_type, timestamp_ms, payload_len = struct.unpack("<BqI", tail)
            if payload_len > 4 * 1024 * 1024:
                raise ValueError(f"payload too large: {payload_len}")
            payload = recv_exact(sock, payload_len)
            if frame_type != TYPE_HAND_FRAME:
                continue
            frame = parse_hand_frame(timestamp_ms, payload)
            now = time.monotonic()
            with self.lock:
                self.latest = frame
                self.frames_received += 1
                self._rate_window_frames += 1
                self._last_frame_monotonic = now
                elapsed = now - self._rate_window_start
                if elapsed >= 0.5:
                    self.rx_rate_hz = self._rate_window_frames / elapsed
                    self._rate_window_frames = 0
                    self._rate_window_start = now
            self.write_json(frame)
            self.maybe_print(frame)

    def write_json(self, frame: Dict[str, Any]) -> None:
        if self.save_jsonl is None:
            return
        if self._json_file is None:
            self.save_jsonl.parent.mkdir(parents=True, exist_ok=True)
            self._json_file = self.save_jsonl.open("a", encoding="utf-8")
        self._json_file.write(json.dumps(frame, separators=(",", ":")) + "\n")
        self._json_file.flush()

    def maybe_print(self, frame: Dict[str, Any]) -> None:
        if not self.print_frames:
            return
        now = time.monotonic()
        if now - self._last_print < 0.25:
            return
        self._last_print = now
        head = frame["head"]
        left = frame["hands"]["left"]["wrist"]
        right = frame["hands"]["right"]["wrist"]
        with self.lock:
            rx_rate_hz = self.rx_rate_hz
            frames_received = self.frames_received
        def fmt(pose: Dict[str, Any]) -> str:
            p = pose["pos"]
            return "(%+.3f,%+.3f,%+.3f)" % (p[0], p[1], p[2])
        print(
            f"[{frame['timestamp_ms']:8d} ms] head={fmt(head)} "
            f"left_wrist={fmt(left)} right_wrist={fmt(right)} "
            f"hands=L{int(frame['hands']['left']['valid'])}/R{int(frame['hands']['right']['valid'])} "
            f"rx={rx_rate_hz:5.1f}Hz frames={frames_received}",
            flush=True,
        )


def run_qt_visualizer(receiver: HandTrackingReceiver) -> None:
    """Show a dark, dashboard-style 3D viewer in a fixed FLU rear view."""
    try:
        import numpy as np
        import pyqtgraph as pg
        import pyqtgraph.opengl as gl
        from PyQt5 import QtCore, QtGui, QtWidgets
    except ImportError as exc:
        raise ImportError(
            "the dashboard visualizer requires PyQt5, pyqtgraph and numpy"
        ) from exc

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    app.setApplicationName("Quest Hand Tracking")
    app.setStyle("Fusion")

    window = QtWidgets.QMainWindow()
    window.setWindowTitle("Quest Hand Tracking  ·  FLU Dashboard")
    window.resize(1280, 820)
    window.setMinimumSize(980, 680)

    root = QtWidgets.QWidget()
    root.setObjectName("root")
    window.setCentralWidget(root)
    outer = QtWidgets.QVBoxLayout(root)
    outer.setContentsMargins(24, 20, 24, 18)
    outer.setSpacing(14)

    style = """
        QMainWindow, QWidget#root { background: #07111f; color: #e2e8f0; }
        QLabel#brand { color: #f8fafc; font-size: 22px; font-weight: 700; letter-spacing: 1px; }
        QLabel#subtitle { color: #8da4b8; font-size: 11px; letter-spacing: 1.4px; }
        QLabel#chip { color: #a7f3d0; background: #102b2a; border: 1px solid #1f7668;
                     border-radius: 10px; padding: 7px 11px; font-size: 10px; letter-spacing: 1px; }
        QLabel#status { color: #dbeafe; background: #0d1b2a; border: 1px solid #21405a;
                        border-radius: 10px; padding: 8px 12px; font-size: 11px; }
        QFrame#dataCard { background: #0d1b2a; border: 1px solid #21405a; border-radius: 12px; }
        QLabel#cardTitle { font-size: 11px; font-weight: 700; letter-spacing: 1.2px; }
        QLabel#cardValue { color: #e2e8f0; font-family: monospace; font-size: 12px; }
        QLabel#cardDetail { color: #7890a5; font-size: 10px; }
        QFrame#sceneFrame { background: #07111f; border: 1px solid #183247; border-radius: 14px; }
        QLabel#legend { color: #8da4b8; font-size: 10px; letter-spacing: .8px; }
        QLabel#legendLeft { color: #69e6b1; font-size: 11px; font-weight: 600; }
        QLabel#legendRight { color: #66c9ff; font-size: 11px; font-weight: 600; }
        QLabel#legendHead { color: #ffb36b; font-size: 11px; font-weight: 600; }
    """
    window.setStyleSheet(style)

    header = QtWidgets.QHBoxLayout()
    header.setSpacing(12)
    heading = QtWidgets.QVBoxLayout()
    heading.setSpacing(2)
    brand = QtWidgets.QLabel("Quest HAND TRACKING")
    brand.setObjectName("brand")
    subtitle = QtWidgets.QLabel("POSE · WRISTS · 26-JOINT SKELETON")
    subtitle.setObjectName("subtitle")
    heading.addWidget(brand)
    heading.addWidget(subtitle)
    header.addLayout(heading)
    header.addStretch(1)
    world_chip = QtWidgets.QLabel("WORLD FLU  ·  INITIAL ORIGIN")
    world_chip.setObjectName("chip")
    header.addWidget(world_chip, alignment=QtCore.Qt.AlignVCenter)
    status_label = QtWidgets.QLabel("CONNECTING · ADB TCP")
    status_label.setObjectName("status")
    header.addWidget(status_label, alignment=QtCore.Qt.AlignVCenter)
    outer.addLayout(header)

    cards = QtWidgets.QHBoxLayout()
    cards.setSpacing(12)

    def make_card(title: str, accent: str) -> Tuple[QtWidgets.QLabel, QtWidgets.QLabel, QtWidgets.QLabel]:
        card = QtWidgets.QFrame()
        card.setObjectName("dataCard")
        layout = QtWidgets.QVBoxLayout(card)
        layout.setContentsMargins(14, 11, 14, 10)
        layout.setSpacing(4)
        title_label = QtWidgets.QLabel(title)
        title_label.setObjectName("cardTitle")
        title_label.setStyleSheet(f"color: {accent};")
        value_label = QtWidgets.QLabel("XYZ   —")
        value_label.setObjectName("cardValue")
        detail_label = QtWidgets.QLabel("WAITING FOR TRACKING")
        detail_label.setObjectName("cardDetail")
        layout.addWidget(title_label)
        layout.addWidget(value_label)
        layout.addWidget(detail_label)
        cards.addWidget(card, 1)
        return title_label, value_label, detail_label

    _head_title, head_value, head_detail = make_card("HEADSET", "#ffb36b")
    _left_title, left_value, left_detail = make_card("LEFT HAND", "#69e6b1")
    _right_title, right_value, right_detail = make_card("RIGHT HAND", "#66c9ff")
    outer.addLayout(cards)

    scene_frame = QtWidgets.QFrame()
    scene_frame.setObjectName("sceneFrame")
    scene_layout = QtWidgets.QVBoxLayout(scene_frame)
    scene_layout.setContentsMargins(1, 1, 1, 1)
    scene_layout.setSpacing(0)
    view = gl.GLViewWidget()
    view.setBackgroundColor("#07111f")
    view.opts["distance"] = 1.45
    view.setCameraPosition(distance=1.45, elevation=16, azimuth=180)
    # Keep the requested rear camera fixed; the target point follows the head.
    view.mouseEnabled = [False, False, False]
    scene_layout.addWidget(view, 1)
    outer.addWidget(scene_frame, 1)

    footer = QtWidgets.QHBoxLayout()
    footer.setContentsMargins(4, 0, 4, 0)
    footer.setSpacing(18)
    footer_label = QtWidgets.QLabel("FIXED REAR CAMERA  ·  -X BEHIND HEAD  ·  +16° UP")
    footer_label.setObjectName("legend")
    footer.addWidget(footer_label)
    footer.addStretch(1)
    for name, object_name in (("HEAD", "legendHead"), ("LEFT", "legendLeft"), ("RIGHT", "legendRight")):
        label = QtWidgets.QLabel(f"●  {name}")
        label.setObjectName(object_name)
        footer.addWidget(label)
    flu_label = QtWidgets.QLabel("X FORWARD  ·  Y LEFT  ·  Z UP")
    flu_label.setObjectName("legend")
    footer.addWidget(flu_label)
    outer.addLayout(footer)

    grid = gl.GLGridItem()
    grid.setSize(3.0, 3.0)
    grid.setSpacing(0.1, 0.1)
    grid.setColor((0.10, 0.30, 0.34, 0.72))
    view.addItem(grid)
    ground_grid = gl.GLLinePlotItem(
        pos=np.zeros((2, 3), dtype=float), color=(0.16, 0.46, 0.48, 0.70),
        width=1.2, antialias=True, mode="lines",
    )
    view.addItem(ground_grid)
    axis = gl.GLAxisItem()
    axis.setSize(0.60, 0.60, 0.60)
    view.addItem(axis)
    # Add a brighter, labelled FLU triad on top of the default axis item.  The
    # long grid remains the world reference while these lines stay readable at
    # the fixed rear camera distance.
    axis_length = 0.62
    for endpoint, color in (
        ([axis_length, 0.0, 0.0], (0.95, 0.35, 0.35, 1.0)),  # X forward
        ([0.0, axis_length, 0.0], (0.35, 0.95, 0.55, 1.0)),  # Y left
        ([0.0, 0.0, axis_length], (0.35, 0.70, 1.0, 1.0)),  # Z up
    ):
        axis_line = gl.GLLinePlotItem(
            pos=np.asarray([[0.0, 0.0, 0.0], endpoint], dtype=float),
            color=color, width=4.0, antialias=True,
        )
        view.addItem(axis_line)
    origin = gl.GLScatterPlotItem(pos=np.asarray([[0.0, 0.0, 0.0]]), color=(1.0, 0.85, 0.35, 1.0), size=9, pxMode=True)
    view.addItem(origin)
    axis_font = QtGui.QFont("DejaVu Sans", 9, QtGui.QFont.Bold)
    for label, position, color in (
        ("X  forward", (axis_length + 0.035, 0.0, 0.0), (1.0, 0.55, 0.55, 1.0)),
        ("Y  left", (0.0, axis_length + 0.035, 0.0), (0.55, 1.0, 0.70, 1.0)),
        ("Z  up", (0.0, 0.0, axis_length + 0.035), (0.55, 0.78, 1.0, 1.0)),
    ):
        axis_label = gl.GLTextItem(pos=position, text=label, color=color, font=axis_font)
        view.addItem(axis_label)
    world_label = gl.GLTextItem(pos=(0.02, 0.02, 0.025), text="WORLD 0", color=(1.0, 0.85, 0.35, 0.9), font=axis_font)
    view.addItem(world_label)
    dynamic_items = []
    head_rgba = (1.0, 0.56, 0.25, 1.0)
    left_rgba = (0.20, 0.90, 0.60, 1.0)
    right_rgba = (0.20, 0.70, 1.0, 1.0)

    def pose_text(pose: Dict[str, Any]) -> str:
        if not pose.get("valid"):
            return "XYZ   —"
        x, y, z = pose["pos"]
        return f"XYZ   {x:+.3f}  {y:+.3f}  {z:+.3f} m"

    def clear_dynamic() -> None:
        while dynamic_items:
            view.removeItem(dynamic_items.pop())

    def add_line(points: Any, color: Tuple[float, float, float, float], width: float = 3.0) -> None:
        item = gl.GLLinePlotItem(pos=np.asarray(points, dtype=float), color=color, width=width, antialias=True)
        view.addItem(item)
        dynamic_items.append(item)

    def add_marker(points: Any, color: Tuple[float, float, float, float], size: float) -> None:
        item = gl.GLScatterPlotItem(pos=np.asarray(points, dtype=float), color=color, size=size, pxMode=True)
        view.addItem(item)
        dynamic_items.append(item)

    def pose_basis(rotation: Any) -> Optional[Tuple[Tuple[float, float, float], ...]]:
        """Return the local FLU X/Y/Z axes for an [x,y,z,w] quaternion."""
        if not isinstance(rotation, (list, tuple)) or len(rotation) != 4:
            return None
        qx, qy, qz, qw = (float(value) for value in rotation)
        length = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
        if length < 1e-5:
            return None
        qx, qy, qz, qw = qx / length, qy / length, qz / length, qw / length
        return (
            (1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy + qz * qw), 2.0 * (qx * qz - qy * qw)),
            (2.0 * (qx * qy - qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz + qx * qw)),
            (2.0 * (qx * qz + qy * qw), 2.0 * (qy * qz - qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)),
        )

    def add_pose_axes(position: Any, rotation: Any, scale: float = 0.075) -> None:
        basis = pose_basis(rotation)
        if basis is None:
            return
        for direction, color in zip(
            basis,
            ((0.95, 0.35, 0.35, 1.0), (0.35, 0.95, 0.55, 1.0), (0.35, 0.70, 1.0, 1.0)),
        ):
            endpoint = [position[i] + direction[i] * scale for i in range(3)]
            add_line([position, endpoint], color, 2.2)

    def refresh() -> None:
        with receiver.lock:
            frame = receiver.latest
            frames_received = receiver.frames_received
            rx_rate_hz = receiver.rx_rate_hz
            last_frame_monotonic = receiver._last_frame_monotonic

        clear_dynamic()
        head = frame["head"] if frame is not None else {"valid": False, "pos": [0.0, 0.0, 0.0], "rot": [0, 0, 0, 1]}
        head_valid = bool(head.get("valid"))
        if head_valid:
            head_position = [float(value) for value in head["pos"]]
            center = [head_position[0] + 0.12, head_position[1], head_position[2] + 0.03]
            add_marker([head_position], head_rgba, 17)
            rotation = head.get("rot", [0, 0, 0, 1])
            add_pose_axes(head_position, rotation, 0.085)
            basis = pose_basis(rotation)
            if basis is not None:
                qx, qy, qz, qw = (float(value) for value in rotation)
                add_line(
                    [head_position, [head_position[i] + basis[0][i] * 0.14 for i in range(3)]],
                    head_rgba, 2.8,
                )
        else:
            head_position = [0.0, 0.0, 0.0]
            center = [0.0, 0.0, 0.0]

        # Keep a perspective ground grid below the tracked objects, similar to
        # the trajectory viewer, while the world-origin triad remains fixed.
        grid.resetTransform()
        floor_z = center[2] - 0.55
        grid.translate(0.0, 0.0, floor_z)
        # Explicit line segments make the floor grid visible across OpenGL
        # implementations where GLGridItem's transparent shader is too faint.
        extent, spacing = 1.8, 0.10
        segments = []
        for index in range(-int(extent / spacing), int(extent / spacing) + 1):
            coordinate = index * spacing
            segments.extend(([-extent, coordinate, floor_z], [extent, coordinate, floor_z]))
            segments.extend(([coordinate, -extent, floor_z], [coordinate, extent, floor_z]))
        ground_grid.setData(
            pos=np.asarray(segments, dtype=float), color=(0.16, 0.46, 0.48, 0.70),
            width=1.2, antialias=True, mode="lines",
        )

        head_value.setText(pose_text(head))
        head_detail.setText("VALID · HEAD POSE" if head_valid else "WAITING FOR HEAD POSE")

        for side, color, value_label, detail_label in (
            ("left", left_rgba, left_value, left_detail),
            ("right", right_rgba, right_value, right_detail),
        ):
            hand = frame["hands"][side] if frame is not None else {"valid": False, "wrist": {"valid": False, "pos": [0, 0, 0]}, "joints": []}
            valid = bool(hand.get("valid"))
            wrist = hand["wrist"]
            value_label.setText(pose_text(wrist))
            detail_label.setText("VALID · 26 JOINTS" if valid else "NO HAND DETECTED")
            if not valid:
                continue
            wrist_position = [float(value) for value in wrist["pos"]]
            add_marker([wrist_position], color, 13)
            add_pose_axes(wrist_position, wrist.get("rot"), 0.075)
            joints = hand["joints"]
            valid_points = [[float(value) for value in joint["pos"]] for joint in joints if joint.get("valid")]
            if valid_points:
                add_marker(valid_points, color, 7)
            for a, b in BONE_PAIRS:
                if not (joints[a].get("valid") and joints[b].get("valid")):
                    continue
                add_line([joints[a]["pos"], joints[b]["pos"]], color, 3.0)

        view.opts["center"] = pg.Vector(*center)
        view.update()
        stale = bool(last_frame_monotonic and time.monotonic() - last_frame_monotonic > 1.0)
        shown_rate = 0.0 if stale else rx_rate_hz
        status_label.setText(f"RX  {shown_rate:04.1f} Hz   ·   {frames_received:,} FRAMES   ·   ADB TCP")

    timer = QtCore.QTimer(window)
    timer.timeout.connect(refresh)
    timer.start(33)
    refresh()
    window.show()
    app.exec_()


def run_mpl_visualizer(receiver: HandTrackingReceiver) -> None:
    try:
        # Some Ubuntu installations expose the distro's ``mpl_toolkits``
        # package ahead of the pip-installed Matplotlib package.  That makes
        # Matplotlib's initial Axes3D import fail with a misleading warning.
        # Put the toolkit directory belonging to the loaded Matplotlib first,
        # then explicitly register the projection after importing it.
        import matplotlib
        import mpl_toolkits
        from pathlib import Path as _Path

        matching_toolkits = _Path(matplotlib.__file__).resolve().parent.parent / "mpl_toolkits"
        if matching_toolkits.is_dir() and hasattr(mpl_toolkits, "__path__"):
            matching_path = str(matching_toolkits)
            toolkit_paths = [path for path in mpl_toolkits.__path__ if path != matching_path]
            mpl_toolkits.__path__[:] = [matching_path] + toolkit_paths

        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        from matplotlib.projections import register_projection

        register_projection(Axes3D)
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            "--visualize requires a working Matplotlib 3D install. "
            "The current Python environment has incompatible matplotlib/mpl_toolkits "
            "versions; use a clean virtual environment or reinstall Matplotlib."
        ) from exc
    except Exception as exc:
        raise SystemExit(f"Unable to initialize Matplotlib 3D visualization: {exc}") from exc

    plt.ion()
    figure = plt.figure("Quest Hand Tracking · FLU", figsize=(10, 8), facecolor="#07111f")
    axes = figure.add_subplot(111, projection="3d")
    background = "#07111f"
    panel = "#0d1b2a"
    grid = "#36506a"
    text = "#e2e8f0"
    head_color = "#fb923c"
    left_color = "#34d399"
    right_color = "#38bdf8"
    muted = "#94a3b8"
    camera_elevation = 16
    camera_azimuth = 180  # camera is behind the head (-X in FLU)

    def head_forward(rotation: Any) -> Optional[Tuple[float, float, float]]:
        """Return the head's +X (forward) axis from an [x,y,z,w] quaternion."""
        if not isinstance(rotation, (list, tuple)) or len(rotation) != 4:
            return None
        qx, qy, qz, qw = (float(value) for value in rotation)
        length = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
        if length < 1e-5:
            return None
        qx, qy, qz, qw = qx / length, qy / length, qz / length, qw / length
        # First column of the quaternion rotation matrix: Unity/FLU +X.
        return (
            1.0 - 2.0 * (qy * qy + qz * qz),
            2.0 * (qx * qy + qz * qw),
            2.0 * (qx * qz - qy * qw),
        )

    while not receiver.stop_event.is_set() and plt.fignum_exists(figure.number):
        with receiver.lock:
            frame = receiver.latest
            frames_received = receiver.frames_received
            rx_rate_hz = receiver.rx_rate_hz
            last_frame_monotonic = receiver._last_frame_monotonic
        axes.clear()
        axes.set_facecolor(background)
        for pane in (axes.xaxis.pane, axes.yaxis.pane, axes.zaxis.pane):
            pane.set_facecolor(panel)
            pane.set_edgecolor(grid)
            pane.set_alpha(0.55)
        axes.grid(True, color=grid, alpha=0.35, linewidth=0.7)
        axes.tick_params(colors=text, labelsize=8, pad=2)
        axes.set_xlabel("X · forward (m)", color=text, labelpad=8)
        axes.set_ylabel("Y · left (m)", color=text, labelpad=8)
        axes.set_zlabel("Z · up (m)", color=text, labelpad=8)
        axes.set_title(
            "Quest Hand Tracking  ·  FLU rear view",
            color=text, pad=18, fontsize=15, fontweight="bold",
        )
        # Keep the camera stable so the scene is easy to read while the hands move:
        # -X is behind the user in FLU, with a small upward tilt.
        axes.view_init(elev=camera_elevation, azim=camera_azimuth)
        try:
            axes.set_proj_type("persp")
        except (AttributeError, NotImplementedError):
            pass
        try:
            axes.set_box_aspect((1, 1, 1))
        except (AttributeError, NotImplementedError):
            pass

        center = [0.0, 0.0, 0.0]
        points = []
        has_labels = False
        if frame is not None:
            if frame["head"]["valid"]:
                head_position = frame["head"]["pos"][:]
                # Aim just in front of and above the head, leaving room for both hands.
                center = [head_position[0] + 0.12, head_position[1], head_position[2] + 0.03]
                points.append(head_position)
                axes.scatter(
                    *head_position, color=head_color, edgecolors="#fff7ed", linewidths=1.3,
                    s=115, depthshade=True, label="Head",
                )
                axes.text(
                    head_position[0], head_position[1], head_position[2] + 0.035,
                    "H", color="#fed7aa", fontsize=10, fontweight="bold",
                )
                forward = head_forward(frame["head"].get("rot"))
                if forward is not None:
                    axes.quiver(
                        head_position[0], head_position[1], head_position[2],
                        forward[0] * 0.16, forward[1] * 0.16, forward[2] * 0.16,
                        color=head_color, linewidth=2.0, arrow_length_ratio=0.22,
                    )
                has_labels = True
            for side, color in (("left", left_color), ("right", right_color)):
                hand = frame["hands"][side]
                if not hand["valid"]:
                    continue
                wrist = hand["wrist"]["pos"]
                points.append(wrist)
                axes.scatter(
                    *wrist, color=color, edgecolors="#f8fafc", linewidths=1.0,
                    s=62, depthshade=True, label=f"{side.title()} wrist",
                )
                has_labels = True
                joints = hand["joints"]
                valid_points = [joint["pos"] for joint in joints if joint["valid"]]
                points.extend(valid_points)
                for a, b in BONE_PAIRS:
                    if not (joints[a]["valid"] and joints[b]["valid"]):
                        continue
                    pa, pb = joints[a]["pos"], joints[b]["pos"]
                    axes.plot(
                        [pa[0], pb[0]], [pa[1], pb[1]], [pa[2], pb[2]],
                        color=color, linewidth=3.0, alpha=0.88, solid_capstyle="round",
                    )
                if valid_points:
                    axes.scatter(
                        [p[0] for p in valid_points], [p[1] for p in valid_points], [p[2] for p in valid_points],
                        color=color, edgecolors="#0f172a", linewidths=0.35, alpha=0.95, s=18,
                    )

            if not frame["head"]["valid"] and points:
                center = points[0][:]
            if points:
                max_distance = max(
                    (max(abs(point[i] - center[i]) for i in range(3)) for point in points),
                    default=0.0,
                )
                span = max(0.55, min(1.8, max_distance * 1.45))
                axes.set_xlim(center[0] - span, center[0] + span)
                axes.set_ylim(center[1] - span, center[1] + span)
                axes.set_zlim(center[2] - span, center[2] + span)
            else:
                axes.set_xlim(-1, 1); axes.set_ylim(-1, 1); axes.set_zlim(-1, 1)
        else:
            axes.set_xlim(-1, 1); axes.set_ylim(-1, 1); axes.set_zlim(-1, 1)

        if has_labels:
            legend = axes.legend(loc="upper right", fontsize=9, frameon=True, borderpad=0.8)
            legend.get_frame().set_facecolor(panel)
            legend.get_frame().set_edgecolor(grid)
            legend.get_frame().set_alpha(0.92)
            for legend_text in legend.get_texts():
                legend_text.set_color(text)

        if last_frame_monotonic and time.monotonic() - last_frame_monotonic > 1.0:
            rx_rate_hz = 0.0
        status = f"RX {rx_rate_hz:5.1f} Hz   ·   frames {frames_received:,}"
        if frame is None:
            status += "   ·   waiting for Quest frame"
        axes.text2D(
            0.03, 0.955, status, transform=axes.transAxes, color=text, fontsize=10,
            bbox=dict(boxstyle="round,pad=0.45", facecolor=panel, edgecolor=grid, alpha=0.92),
        )
        axes.text2D(
            0.03, 0.895, "FLU  X forward  ·  Y left  ·  Z up  ·  rear camera +16°",
            transform=axes.transAxes, color=muted, fontsize=9,
        )

        figure.canvas.draw_idle()
        figure.canvas.flush_events()
        time.sleep(1.0 / 60.0)


def run_visualizer(receiver: HandTrackingReceiver, use_matplotlib: bool = False) -> None:
    """Use the dashboard window by default, with the old viewer as an explicit fallback."""
    if use_matplotlib:
        run_mpl_visualizer(receiver)
        return
    try:
        run_qt_visualizer(receiver)
    except ImportError as exc:
        print(f"[visualizer] {exc}; falling back to Matplotlib", file=sys.stderr, flush=True)
        run_mpl_visualizer(receiver)


def main() -> None:
    parser = argparse.ArgumentParser(description="Receive Quest FLU hand tracking over ADB forward TCP")
    parser.add_argument("--host", default="127.0.0.1", help="ADB-forwarded host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=10002, help="must match QuestHandTrackingAdbStreamer.tcpPort")
    parser.add_argument("--device-port", type=int, help="Quest device TCP port; defaults to --port")
    parser.add_argument("--adb-path", default="adb", help="adb executable path (default: adb)")
    parser.add_argument("--adb-serial", help="ADB device serial when multiple devices are connected")
    parser.add_argument(
        "--no-adb-forward",
        action="store_true",
        help="do not run adb forward; use an externally managed TCP mapping",
    )
    parser.add_argument("--reconnect", type=float, default=2.0, help="reconnect delay in seconds")
    parser.add_argument("--visualize", action="store_true", help="show the dashboard-style 3D FLU window")
    parser.add_argument("--visualize-mpl", action="store_true", help="use the legacy Matplotlib 3D view")
    parser.add_argument("--print", dest="print_frames", action="store_true", help="periodically print head and wrist positions")
    parser.add_argument("--save-jsonl", type=Path, help="also append decoded frames to this JSONL file")
    args = parser.parse_args()

    receiver = HandTrackingReceiver(
        args.host,
        args.port,
        max(args.reconnect, 0.1),
        args.save_jsonl,
        args.print_frames,
        adb_path=args.adb_path,
        adb_serial=args.adb_serial,
        adb_device_port=args.device_port,
        auto_adb_forward=not args.no_adb_forward,
    )
    thread = receiver.start()
    try:
        if args.visualize or args.visualize_mpl:
            run_visualizer(receiver, use_matplotlib=args.visualize_mpl)
        else:
            while thread.is_alive():
                thread.join(timeout=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()
        thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
