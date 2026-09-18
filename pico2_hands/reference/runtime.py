"""Extracted receive-only PICO2 classes from dexhand_deploy.

No Zenoh, ROS, IK, executors or vendor SDK imports. APK/ADB protocol unchanged.
The standalone observation entry disables automatic ADB forwarding.
"""
from __future__ import annotations
from collections.abc import Callable
from typing import Any
import logging
import socket
import subprocess
import threading
import time
import numpy as np
from .models import (PicoRawFrame, PicoRawHand, HandObservation, ArmInputObservation,
                     PICO_HEAD_CURRENT_FRAME, PICO_HEAD_MAPPING_VERSION, PICO_MAPPING_VERSION)
from .pico import (MAGIC, MESSAGE_TYPE, HEADER, PAYLOAD_BYTES, parse_pico_packet,
                   pico_to_mediapipe, tracking_pose_to_current_head)
LOG = logging.getLogger(__name__)
MAX_PACKET_BYTES = HEADER.size + PAYLOAD_BYTES

def _valid_pose(pose: np.ndarray) -> bool:
    return bool(np.isfinite(pose).all() and np.linalg.norm(pose[3:]) >= 1.0e-12)


def _canonical_hand_observation(frame: PicoRawFrame, side: str, hand: PicoRawHand) -> HandObservation:
    positions = np.asarray([joint.pose[:3] for joint in hand.joints], dtype=np.float64)
    validity = np.asarray([joint.valid for joint in hand.joints], dtype=np.bool_)
    points, point_valid = pico_to_mediapipe(positions, validity)
    if not hand.valid:
        point_valid[:] = False
        points[:] = 0.0
    elif bool(point_valid[0]):
        points[point_valid] -= points[0]
    else:
        points[:] = 0.0
    wrist_pose = hand.wrist_pose.copy() if hand.wrist_valid and _valid_pose(hand.wrist_pose) else None
    valid = bool(hand.valid and wrist_pose is not None and point_valid.all())
    return HandObservation(
        source="pico",
        side=side,
        source_instance_id=frame.receiver_instance_id,
        source_sequence=None,
        source_timestamp_ns=frame.source_timestamp_ns,
        received_timestamp_ns=frame.received_timestamp_ns,
        receiver_instance_id=frame.receiver_instance_id,
        receiver_frame_sequence=frame.receiver_frame_sequence,
        coordinate_frame="pico_tracking_initial_wrist_relative",
        mapping_version=PICO_MAPPING_VERSION,
        keypoints_m=points,
        joint_valid=point_valid,
        valid=valid,
        wrist_pose=wrist_pose,
        frame_association_id=frame.association_id,
    )


def _pico_arm_observation(frame: PicoRawFrame, side: str, hand: PicoRawHand) -> ArmInputObservation:
    pose: np.ndarray | None = None
    if frame.head_valid and hand.wrist_valid and _valid_pose(frame.head_pose) and _valid_pose(hand.wrist_pose):
        pose = tracking_pose_to_current_head(frame.head_pose, hand.wrist_pose)
    return ArmInputObservation(
        source="pico",
        side=side,
        tracked_frame="wrist",
        reference_frame=PICO_HEAD_CURRENT_FRAME,
        pose=pose,
        valid=pose is not None,
        source_timestamp_ns=frame.source_timestamp_ns,
        received_timestamp_ns=frame.received_timestamp_ns,
        receiver_instance_id=frame.receiver_instance_id,
        receiver_frame_sequence=frame.receiver_frame_sequence,
        mapping_version=PICO_HEAD_MAPPING_VERSION,
        frame_association_id=frame.association_id,
        source_sequence=None,
        source_instance_id=frame.receiver_instance_id,
    )


def pico_frame_observations(frame: PicoRawFrame) -> dict[str, tuple[HandObservation, ArmInputObservation]]:
    """Convert one raw PICO frame into one pair per hand side."""
    if not isinstance(frame, PicoRawFrame):
        raise TypeError("frame must be PicoRawFrame")
    return {
        side: (
            _canonical_hand_observation(frame, side, frame.hands[side]),
            _pico_arm_observation(frame, side, frame.hands[side]),
        )
        for side in ("left", "right")
    }


class PicoPacketStream:
    """Incrementally frame PICO TCP bytes without assuming recv boundaries."""

    def __init__(self, *, receiver_instance_id: str, connection_generation: int) -> None:
        if not receiver_instance_id:
            raise ValueError("receiver_instance_id is required")
        if isinstance(connection_generation, bool) or connection_generation < 0:
            raise ValueError("connection_generation must be non-negative")
        self.receiver_instance_id = receiver_instance_id
        self.connection_generation = int(connection_generation)
        self._buffer = bytearray()
        self._next_frame_sequence = 0

    def reset(self, *, connection_generation: int | None = None) -> None:
        self._buffer.clear()
        self._next_frame_sequence = 0
        if connection_generation is not None:
            if isinstance(connection_generation, bool) or connection_generation < 0:
                raise ValueError("connection_generation must be non-negative")
            self.connection_generation = int(connection_generation)

    def feed(self, data: bytes, *, received_timestamp_ns: int | None = None) -> list[PicoRawFrame]:
        if not isinstance(data, (bytes, bytearray, memoryview)):
            raise TypeError("PICO TCP data must be bytes-like")
        self._buffer.extend(bytes(data))
        frames: list[PicoRawFrame] = []
        while True:
            if len(self._buffer) < 1:
                break
            if self._buffer[0] != MAGIC:
                try:
                    index = self._buffer.index(MAGIC)
                except ValueError:
                    self._buffer.clear()
                    break
                del self._buffer[:index]
            if len(self._buffer) < HEADER.size:
                break
            _magic, message_type, _timestamp_ms, payload_length = HEADER.unpack_from(self._buffer, 0)
            if message_type != MESSAGE_TYPE or payload_length != PAYLOAD_BYTES:
                raise ValueError(
                    f"unsupported PICO TCP frame header: type={message_type}, payload_length={payload_length}"
                )
            packet_length = HEADER.size + payload_length
            if packet_length > MAX_PACKET_BYTES:
                raise ValueError("PICO packet exceeds configured maximum")
            if len(self._buffer) < packet_length:
                break
            packet = bytes(self._buffer[:packet_length])
            del self._buffer[:packet_length]
            timestamp = time.monotonic_ns() if received_timestamp_ns is None else received_timestamp_ns
            frame = parse_pico_packet(
                packet,
                receiver_instance_id=self.receiver_instance_id,
                connection_generation=self.connection_generation,
                receiver_frame_sequence=self._next_frame_sequence,
                received_timestamp_ns=timestamp,
            )
            self._next_frame_sequence += 1
            frames.append(frame)
        return frames



class PicoTcpReceiver:
    """Reconnectable PICO TCP/ADB receiver that feeds ``ObservationRuntime``."""

    def __init__(
        self,
        *,
        host: str,
        port: int,
        receiver_instance_id: str,
        reconnect_seconds: float = 1.0,
        adb_path: str = "adb",
        adb_serial: str | None = None,
        adb_device_port: int | None = None,
        auto_adb_forward: bool = True,
        socket_factory: Callable[..., Any] = socket.create_connection,
        on_error: Callable[[Exception], None] | None = None,
    ) -> None:
        if not host or not receiver_instance_id:
            raise ValueError("host and receiver_instance_id are required")
        if isinstance(port, bool) or not 1 <= int(port) <= 65535:
            raise ValueError("port must be in 1..65535")
        if reconnect_seconds <= 0.0 or not np.isfinite(reconnect_seconds):
            raise ValueError("reconnect_seconds must be finite and positive")
        self.host = host
        self.port = int(port)
        self.receiver_instance_id = receiver_instance_id
        self.reconnect_seconds = float(reconnect_seconds)
        self.adb_path = adb_path
        self.adb_serial = adb_serial
        self.adb_device_port = self.port if adb_device_port is None else int(adb_device_port)
        self.auto_adb_forward = bool(auto_adb_forward)
        self.socket_factory = socket_factory
        self.on_error = on_error
        self.stop_event = threading.Event()
        self._connection_generation = 0
        self._socket: Any = None

    def stop(self) -> None:
        self.stop_event.set()
        current_socket = self._socket
        if current_socket is not None:
            try:
                current_socket.close()
            except Exception:
                pass

    def ensure_adb_forward(self) -> None:
        if not self.auto_adb_forward:
            return
        command = [self.adb_path]
        if self.adb_serial:
            command.extend(("-s", self.adb_serial))
        command.extend(("forward", f"tcp:{self.port}", f"tcp:{self.adb_device_port}"))
        try:
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=5.0, check=False)
        except FileNotFoundError as exc:
            raise RuntimeError(f"ADB executable not found: {self.adb_path!r}") from exc
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError("adb forward timed out") from exc
        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            raise RuntimeError(f"adb forward failed (exit {result.returncode})" + (f": {detail}" if detail else ""))

    def run(self, on_frame: Callable[[PicoRawFrame], None]) -> None:
        if not callable(on_frame):
            raise TypeError("on_frame must be callable")
        while not self.stop_event.is_set():
            try:
                self.ensure_adb_forward()
                LOG.info("connecting to PICO at %s:%s", self.host, self.port)
                with self.socket_factory((self.host, self.port), timeout=3.0) as sock:
                    self._socket = sock
                    self._connection_generation += 1
                    stream = PicoPacketStream(
                        receiver_instance_id=self.receiver_instance_id,
                        connection_generation=self._connection_generation,
                    )
                    try:
                        sock.settimeout(1.0)
                    except AttributeError:
                        pass
                    while not self.stop_event.is_set():
                        try:
                            data = sock.recv(64 * 1024)
                        except socket.timeout:
                            continue
                        if not data:
                            raise ConnectionError("PICO closed the TCP connection")
                        for frame in stream.feed(data):
                            on_frame(frame)
            except (OSError, ConnectionError, RuntimeError, ValueError) as exc:
                if self.on_error is not None and not self.stop_event.is_set():
                    self.on_error(exc)
                if not self.stop_event.is_set():
                    self.stop_event.wait(self.reconnect_seconds)
            finally:
                self._socket = None
