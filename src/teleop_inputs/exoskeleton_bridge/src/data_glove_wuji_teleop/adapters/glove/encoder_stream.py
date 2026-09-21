#!/usr/bin/env python3
"""Explicit DGST v3 or encoder v1 receiver delivering only raw encoder scans."""

from __future__ import annotations

import json
import math
import socket
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import TYPE_CHECKING

from .dgst_protocol import ProtocolError, StreamMetadata, fetch_metadata, json_object

if TYPE_CHECKING:
    from .encoder_kinematics import EncoderKinematics, GloveJointFrame


MAGIC = b"DGST"
VERSION = 3
HEADER = struct.Struct(">4sBBHIQQIII")
ENCODER_HEADER = struct.Struct("!4sBBHIQI")
HELLO, VIDEO, IMU0, IMU900, ENCODER, END, ERROR = 0, 1, 2, 3, 4, 254, 255
FIRST, LAST, KEY = 1, 2, 4
MAX_JSON_BYTES = 64 * 1024
MAX_VIDEO_FRAGMENT = 16 * 1024
MAX_VIDEO_FRAME = 8 * 1024 * 1024
ENCODER_QUEUE_SIZE = 256


@dataclass
class EncoderFrame:
    """One atomic raw scan; sequence gaps are errors, not dropped samples."""

    sequence: int
    timestamp_ns: int
    dropped: int
    angles_deg: list[float]


class EncoderConnection:
    """One reader continuously validates all sensors, with bounded encoder delivery.

    The queue may discard old encoder scans when the consumer falls behind. This
    never skips wire validation and never hides a failed stream behind cached data.
    STOP must receive a stopped END (DGST) or a frame-boundary EOF (encoder v1).
    """

    def __init__(self, sock: socket.socket | None):
        self.sock = sock
        self.protocol = "dgst-v3"
        self.channels = 0
        self.cs_by_joint: list[int] = []
        self.range_min = 0.0
        self.range_max = 360.0
        self.metadata: StreamMetadata | None = None
        self.complete = False
        self.stop_confirmation: str | None = None
        self.counts: dict[int, int] = {}
        self._condition = threading.Condition()
        self._frames: deque[EncoderFrame] = deque(maxlen=ENCODER_QUEUE_SIZE)
        self._error: Exception | None = None
        self._reader: threading.Thread | None = None
        self._timeout = 15.0
        self._hello = False
        self._done = False
        self._closed = False
        self._stop_requested = False
        self._stream_id: int | None = None
        self._available: set[int] = set()
        self._next_sequence: dict[int, int] = {}
        # Only five integers are retained for a video frame, never its payload.
        self._video: tuple[int, int, int, int, int] | None = None

    @property
    def zeroed(self) -> bool:
        """Samples are always raw, regardless of firmware calibration status."""
        return False

    @classmethod
    def connect(
        cls, host: str, port: int = 5580, timeout: float = 15.0,
        source_address: str | None = None, *, http_port: int = 5570,
        expected_device_id: str | None = None, expected_hand: str | None = None,
        protocol: str = "dgst-v3",
    ) -> EncoderConnection:
        if protocol not in ("dgst-v3", "encoder-v1"):
            raise ValueError(f"unsupported encoder protocol: {protocol!r}")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be finite and positive")
        metadata = fetch_metadata(
            host, port=http_port, timeout=timeout, source_address=source_address,
            expected_device_id=expected_device_id, expected_hand=expected_hand,
        )
        sock = socket.create_connection(
            (host, port), timeout=timeout,
            source_address=(source_address, 0) if source_address is not None else None,
        )
        connection = cls(sock)
        connection.protocol = protocol
        connection.metadata = metadata
        connection.channels = metadata.sensors["encoder"]["channels"]
        connection.cs_by_joint = list(metadata.sensors["encoder"]["cs_by_joint"])
        connection._timeout = timeout
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(min(timeout, 5.0))
            start = (
                {"command": "start", "version": VERSION, "metadata_sha256": metadata.sha256}
                if protocol == "dgst-v3"
                else {"cmd": "start", "stream": "encoder", "version": 1}
            )
            sock.sendall(json.dumps(start, separators=(",", ":")).encode("utf-8") + b"\n")
            # A short socket poll lets shutdown interrupt an otherwise long read.
            sock.settimeout(min(timeout, 0.25))
            connection._reader = threading.Thread(
                target=connection._read_stream, name=f"{protocol}-reader", daemon=True,
            )
            connection._reader.start()
            deadline = time.monotonic() + timeout
            with connection._condition:
                while not connection._hello:
                    connection._raise_error()
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError(f"{protocol} HELLO deadline exceeded")
                    connection._condition.wait(remaining)
                connection._raise_error()
            return connection
        except BaseException:
            connection._abort()
            raise

    def _raise_error(self) -> None:
        if self._error is not None:
            raise self._error

    def _read_exact(self, size: int, deadline: float, *, allow_stop_eof: bool = False) -> bytes:
        sock = self.sock
        if sock is None:
            raise EOFError(f"{self.protocol} socket is closed")
        data = bytearray(size)
        view = memoryview(data)
        offset = 0
        while offset < size:
            if time.monotonic() >= deadline:
                raise TimeoutError(f"{self.protocol} message deadline exceeded")
            try:
                received = sock.recv_into(view[offset:])
            except socket.timeout:
                continue
            if not received:
                with self._condition:
                    if (allow_stop_eof and offset == 0 and self.protocol == "encoder-v1"
                            and self._hello and self._stop_requested):
                        self._raise_error()
                        return b""
                raise EOFError(f"{self.protocol} EOF before complete STOP response")
            offset += received
        return bytes(data)

    def _consume_encoder_hello(self, deadline: float) -> None:
        # Read only through LF: the first binary frame can share this TCP packet.
        line = bytearray()
        while len(line) < MAX_JSON_BYTES:
            byte = self._read_exact(1, deadline)
            if byte == b"\n":
                break
            line.extend(byte)
        else:
            raise ProtocolError("encoder-v1 START response exceeds JSON limit")
        hello = json_object(bytes(line), limit=MAX_JSON_BYTES)
        if (hello.get("ok") is not True
                or type(hello.get("version")) is not int or hello["version"] != 1
                or type(hello.get("channels")) is not int or hello["channels"] != self.channels
                or hello.get("unit") != "deg"
                or hello.get("clock") != "CLOCK_MONOTONIC_ns"
                or hello.get("order") != f"J1..J{self.channels}"):
            raise ProtocolError("invalid encoder-v1 START response")
        mapping = hello.get("cs_by_joint")
        if (not isinstance(mapping, list) or any(type(cs) is not int for cs in mapping)
                or mapping != self.cs_by_joint):
            raise ProtocolError("encoder-v1 cs_by_joint differs from metadata")
        if (("zeroed" in hello and hello["zeroed"] is not False)
                or ("range" in hello and hello["range"] != [0, 360])):
            raise ProtocolError("encoder-v1 samples must be raw absolute degrees [0,360)")
        self._available = {ENCODER}
        self._publish(HELLO, 0)

    def _read_encoder_message(self, deadline: float) -> bool:
        data = self._read_exact(ENCODER_HEADER.size, deadline, allow_stop_eof=True)
        if not data:
            with self._condition:
                self._raise_error()
                self.complete = True
                self.stop_confirmation = "eof"
                self._condition.notify_all()
            return True
        magic, version, flags, channels, seq, timestamp, dropped = ENCODER_HEADER.unpack(data)
        if magic != b"DGEC" or version != 1 or flags != 0:
            raise ProtocolError("invalid encoder-v1 magic, version or flags")
        if channels != self.channels:
            raise ProtocolError("encoder channel count changed")
        if seq != self._next_sequence.get(ENCODER, 0):
            raise ProtocolError("sequence gap for encoder-v1")
        if dropped != 0:
            raise ProtocolError("encoder-v1 dropped samples")
        payload = self._read_exact(channels * 4, deadline)
        angles = list(struct.unpack(f"!{channels}f", payload))
        self._publish(ENCODER, seq, self._encoder_frame(seq, timestamp, dropped, angles))
        return False

    @staticmethod
    def _encoder_frame(seq: int, timestamp: int, dropped: int, angles: list[float]) -> EncoderFrame:
        if any(not math.isfinite(value) or not 0 <= value < 360 for value in angles):
            raise ProtocolError("encoder angle outside raw [0,360) degrees")
        return EncoderFrame(seq, timestamp, dropped, angles)

    def _check_header(self, values: tuple) -> tuple[int, int, int, int, int, int, int]:
        magic, version, kind, flags, size, seq, timestamp, stream, offset, total = values
        if magic != MAGIC or version != VERSION:
            raise ProtocolError("invalid DGST magic or version")
        if kind not in (HELLO, VIDEO, IMU0, IMU900, ENCODER, END, ERROR):
            raise ProtocolError(f"unknown DGST kind: {kind}")
        if flags & ~(FIRST | LAST | KEY) or size == 0:
            raise ProtocolError("invalid DGST flags or payload length")
        if self._stream_id is not None and stream != self._stream_id:
            raise ProtocolError("DGST stream_id changed")
        if not self._hello and kind not in (HELLO, ERROR):
            raise ProtocolError("DGST stream must begin with HELLO")
        if kind == VIDEO:
            if size > MAX_VIDEO_FRAGMENT or not 0 < total <= MAX_VIDEO_FRAME or offset + size > total:
                raise ProtocolError("invalid VIDEO fragment size")
        else:
            if flags != FIRST | LAST or offset != 0 or total != size:
                raise ProtocolError("non-video messages cannot be fragmented or KEY marked")
            expected = {IMU0: 48, IMU900: 72, ENCODER: 2 + self.channels * 8}.get(kind)
            if expected is not None and size != expected:
                raise ProtocolError(f"invalid payload length for kind {kind}")
            if kind in (HELLO, END, ERROR) and (size > MAX_JSON_BYTES or seq != 0 or timestamp != 0):
                raise ProtocolError("invalid control message header")
        if kind in (VIDEO, IMU0, IMU900, ENCODER):
            if kind not in self._available:
                raise ProtocolError("sensor kind absent from HELLO")
            if seq != self._next_sequence.get(kind, 0):
                raise ProtocolError(f"sequence gap for kind {kind}")
        if self._stream_id is None:
            self._stream_id = stream
        return kind, flags, size, seq, timestamp, offset, total

    def _consume_video(self, flags: int, size: int, seq: int, timestamp: int, offset: int, total: int) -> bool:
        key = flags & KEY
        if self._video is None:
            if not flags & FIRST or offset != 0:
                raise ProtocolError("VIDEO frame must start at offset zero with FIRST")
        else:
            old_seq, old_time, old_key, old_total, next_offset = self._video
            if flags & FIRST or (seq, timestamp, key, total, offset) != (
                old_seq, old_time, old_key, old_total, next_offset,
            ):
                raise ProtocolError("VIDEO fragment offset or frame identity changed")
        end = offset + size
        if bool(flags & LAST) != (end == total):
            raise ProtocolError("VIDEO LAST flag does not match frame length")
        self._video = None if flags & LAST else (seq, timestamp, key, total, end)
        return self._video is None

    def _consume_hello(self, payload: bytes) -> None:
        if self._hello:
            raise ProtocolError("duplicate HELLO")
        hello = json_object(payload, limit=MAX_JSON_BYTES)
        metadata = self.metadata
        if metadata is None:
            raise ProtocolError("metadata required before START")
        if hello.get("boot_id") != metadata.device["boot_id"]:
            raise ProtocolError("HELLO boot_id differs from metadata")
        if hello.get("metadata_sha256") != metadata.sha256:
            raise ProtocolError("HELLO metadata SHA256 mismatch")
        if hello.get("clock") != "CLOCK_MONOTONIC_ns":
            raise ProtocolError("unsupported HELLO clock")
        kinds = hello.get("available_kinds")
        if (not isinstance(kinds, list) or not kinds
                or any(type(kind) is not int or kind not in (VIDEO, IMU0, IMU900, ENCODER) for kind in kinds)
                or len(set(kinds)) != len(kinds) or ENCODER not in kinds):
            raise ProtocolError("invalid available_kinds or encoder missing")
        self._available = set(kinds)

    def _consume(self, header: tuple, payload: bytes) -> bool:
        kind, flags, size, seq, timestamp, offset, total = header
        frame = None
        if kind == VIDEO:
            if not self._consume_video(flags, size, seq, timestamp, offset, total):
                return False
        elif kind in (IMU0, IMU900):
            if not all(math.isfinite(value) for value in struct.unpack("<6d" if kind == IMU0 else "<9d", payload)):
                raise ProtocolError("non-finite IMU sample")
        elif kind == ENCODER:
            if struct.unpack_from("<H", payload)[0] != self.channels:
                raise ProtocolError("encoder channel count changed")
            angles = list(struct.unpack_from(f"<{self.channels}d", payload, 2))
            frame = self._encoder_frame(seq, timestamp, 0, angles)
        elif kind == HELLO:
            self._consume_hello(payload)
        elif kind == END:
            end = json_object(payload, limit=MAX_JSON_BYTES)
            if end.get("reason") != "stopped" or not self._stop_requested or self._video is not None:
                raise ProtocolError("END is not a complete response to STOP")
        else:
            error = json_object(payload, limit=MAX_JSON_BYTES)
            if any(not isinstance(error.get(key), str) or not error[key] for key in ("code", "message")):
                raise ProtocolError("invalid ERROR payload")
            with self._condition:
                self.counts[kind] = self.counts.get(kind, 0) + 1
            raise ProtocolError(f"DGST {error['code']}: {error['message']}")
        self._publish(kind, seq, frame)
        return kind == END

    def _publish(self, kind: int, seq: int, frame: EncoderFrame | None = None) -> None:
        with self._condition:
            self._raise_error()
            self.counts[kind] = self.counts.get(kind, 0) + 1
            if kind in self._available:
                self._next_sequence[kind] = (seq + 1) & 0xFFFFFFFF if self.protocol == "encoder-v1" else seq + 1
            if frame is not None:
                self._frames.append(frame)
            if kind == HELLO:
                self._hello = True
            if kind == END:
                self.complete = True
                self.stop_confirmation = "end"
            self._condition.notify_all()

    def _read_stream(self) -> None:
        try:
            if self.protocol == "encoder-v1":
                self._consume_encoder_hello(time.monotonic() + self._timeout)
            while True:
                deadline = time.monotonic() + self._timeout
                if self.protocol == "encoder-v1":
                    if self._read_encoder_message(deadline):
                        break
                    continue
                header = self._check_header(HEADER.unpack(self._read_exact(HEADER.size, deadline)))
                payload = self._read_exact(header[2], deadline)
                if self._consume(header, payload):
                    break
        except Exception as exc:
            with self._condition:
                if self._error is None:
                    self._error = exc
                self.complete = False
                self.stop_confirmation = None
        finally:
            self._shutdown_socket()
            with self._condition:
                self._done = True
                self._condition.notify_all()

    def _take_frame(self, latest: bool, deadline_monotonic: float | None) -> EncoderFrame:
        deadline = time.monotonic() + self._timeout if deadline_monotonic is None else deadline_monotonic
        with self._condition:
            while True:
                self._raise_error()
                if self._closed:
                    raise EOFError(f"{self.protocol} connection is closed")
                if time.monotonic() >= deadline:
                    raise TimeoutError("encoder frame deadline exceeded")
                if self._frames:
                    if latest:
                        frame = self._frames[-1]
                        self._frames.clear()
                        return frame
                    return self._frames.popleft()
                if self._done:
                    raise EOFError(f"{self.protocol} stream ended")
                self._condition.wait(deadline - time.monotonic())

    def read_frame(self, *, deadline_monotonic: float | None = None) -> EncoderFrame:
        return self._take_frame(False, deadline_monotonic)

    def read_latest_frame(self, *, deadline_monotonic: float | None = None) -> EncoderFrame:
        """Atomically take the newest validated encoder sample without socket draining."""
        return self._take_frame(True, deadline_monotonic)

    def _require_zeroed_or_trusted(self, trust_hardware_zero: bool) -> None:
        if not self.zeroed and trust_hardware_zero is not True:
            raise ProtocolError(
                "编码器尚未完成零位初始化，不能执行四连杆换算；"
                "确认硬件已校零后，必须显式传入布尔值 True 才能绕过"
            )

    def read_joint_frame(
        self, kinematics: EncoderKinematics, *, trust_hardware_zero: bool = False,
    ) -> GloveJointFrame:
        """Keep the explicit safety gate; neither protocol supplies calibrated angles."""
        self._require_zeroed_or_trusted(trust_hardware_zero)
        return kinematics.convert_zeroed_frame(self.read_frame())

    def read_latest_joint_frame(
        self, kinematics: EncoderKinematics, *, trust_hardware_zero: bool = False,
    ) -> GloveJointFrame:
        self._require_zeroed_or_trusted(trust_hardware_zero)
        return kinematics.convert_zeroed_frame(self.read_latest_frame())

    def _shutdown_socket(self) -> None:
        sock = self.sock
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()

    def _abort(self) -> None:
        self._shutdown_socket()
        if self._reader is not None and self._reader is not threading.current_thread():
            self._reader.join()
        self.sock = None
        self._closed = True

    def close(self) -> None:
        if self._closed:
            return
        deadline = time.monotonic() + self._timeout
        try:
            with self._condition:
                self._raise_error()
                if not self._done and self.sock is not None:
                    self._stop_requested = True
                    stop = b'{"command":"stop"}\n' if self.protocol == "dgst-v3" else b'{"cmd":"stop"}\n'
                    self.sock.sendall(stop)
                while not self._done and self._reader is not None:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError(f"{self.protocol} STOP drain exceeded {self._timeout:g} seconds")
                    self._condition.wait(remaining)
                self._raise_error()
                if self._reader is not None and not self.complete:
                    raise ProtocolError(f"{self.protocol} stream ended without complete STOP response")
        except Exception as exc:
            with self._condition:
                self.complete = False
                self.stop_confirmation = None
                if self._error is None:
                    self._error = exc
            raise
        finally:
            self._abort()

    def __enter__(self) -> EncoderConnection:
        return self

    def __exit__(self, exc_type, _exc_value, _traceback) -> None:
        if exc_type is None:
            self.close()
        else:
            try:
                self.close()
            except Exception:
                # Preserve the body failure while still draining or aborting safely.
                pass
