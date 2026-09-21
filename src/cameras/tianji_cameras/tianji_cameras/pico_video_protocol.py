"""Existing PICO camera control bytes and ownership-safe wired ADB routing."""

from __future__ import annotations

from dataclasses import dataclass
import os
import socket
import struct
import subprocess
import time

CONTROL_PORT = 13579
VIDEO_PORT = 12345
_MAX_MESSAGE_BYTES = 65536
_MAX_COMMAND_BYTES = 256
_MESSAGE_TIMEOUT = 2.0


@dataclass(frozen=True)
class CameraRequest:
    width: int
    height: int
    fps: int
    bitrate: int
    render_mode: int
    port: int
    ip: str
    camera: str


def read_message(sock: socket.socket) -> tuple[str | None, bytes | None]:
    """Read one XRoboToolkit message; its big-endian total includes the header.

    An idle timeout leaves the socket usable. A timeout after any bytes were
    consumed closes it and raises ValueError: callers must never retry a partly
    consumed message as a new header. EOF returns (None, None).
    """
    original_timeout = sock.gettimeout()
    deadline = time.monotonic() + _MESSAGE_TIMEOUT
    consumed = 0
    closed = False

    def receive(size: int) -> bytes | None:
        nonlocal consumed
        data = bytearray(size)
        view = memoryview(data)
        offset = 0
        while offset < size:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("PICO control message deadline expired")
            timeout = remaining if original_timeout is None else min(original_timeout, remaining)
            sock.settimeout(timeout)
            count = sock.recv_into(view[offset:])
            if not count:
                return None
            consumed += count
            offset += count
        return bytes(data)

    try:
        header = receive(4)
        if header is None:
            return None, None
        total, = struct.unpack(">I", header)
        if not 12 <= total <= _MAX_MESSAGE_BYTES:
            raise ValueError(f"Invalid PICO control message length: {total}")
        body = receive(total - 4)
        if body is None:
            return None, None
        command_size, = struct.unpack_from("<I", body)
        if not 1 <= command_size <= min(_MAX_COMMAND_BYTES, total - 12):
            raise ValueError("Invalid PICO control command length")
        payload_size, = struct.unpack_from("<I", body, 4 + command_size)
        if payload_size != total - 12 - command_size:
            raise ValueError("PICO control payload length does not match framing")
        try:
            command = body[4:4 + command_size].decode("utf-8")
        except UnicodeDecodeError as error:
            raise ValueError("PICO control command is not UTF-8") from error
        return command, body[8 + command_size:]
    except TimeoutError as error:
        if consumed:
            sock.close()
            closed = True
            raise ValueError("Timed out inside a PICO control message; connection closed") from error
        raise
    except (ValueError, OSError):
        sock.close()
        closed = True
        raise
    finally:
        if not closed:
            sock.settimeout(original_timeout)


def parse_camera_request(payload: bytes) -> CameraRequest:
    """Decode CA FE 01 plus seven int32s and two byte-length UTF-8 strings.

    The peer's IP is descriptive only: wired video always uses localhost.
    Multiview HEVC is deliberately unsupported; the video path is H.264 SBS.
    """
    if not 33 <= len(payload) <= 543 or payload[:3] != b"\xca\xfe\x01":
        raise ValueError("Invalid PICO camera request header or length")
    width, height, fps, bitrate, enable_mv_hevc, render_mode, port = struct.unpack_from(
        "<7i", payload, 3)
    camera_size = payload[31]
    ip_offset = 32 + camera_size
    if ip_offset >= len(payload):
        raise ValueError("Truncated PICO camera name")
    ip_size = payload[ip_offset]
    if ip_offset + 1 + ip_size != len(payload):
        raise ValueError("PICO camera IP length does not match request")
    try:
        camera = payload[32:ip_offset].decode("utf-8")
        ip = payload[ip_offset + 1:].decode("utf-8")
    except UnicodeDecodeError as error:
        raise ValueError("PICO camera name/IP is not UTF-8") from error
    if not 4 <= width <= 4096 or width % 4:
        raise ValueError("PICO SBS width must be a multiple of 4 in [4, 4096]")
    if not 2 <= height <= 2160 or height % 2:
        raise ValueError("PICO video height must be even and in [2, 2160]")
    if not 1 <= fps <= 60:
        raise ValueError("PICO requested FPS must be in [1, 60]")
    if not 1 <= bitrate <= 100_000_000:
        raise ValueError("PICO bitrate must be in [1, 100000000] bits/second")
    if enable_mv_hevc != 0:
        raise ValueError("PICO multiview HEVC is unsupported; request H.264")
    if port != VIDEO_PORT:
        raise ValueError(f"Wired PICO video requires port {VIDEO_PORT}")
    return CameraRequest(width, height, fps, bitrate, render_mode, port, ip, camera)


class AdbVideoBridge:
    """Own only newly created camera routes; never alter the hand-input route.

    Reverse listings are device-scoped and their first column may be a transport
    label rather than a serial, so it is normalized to the selected serial.
    Device transport IDs and the complete rule are
    rechecked before removal. ADB provides no compare-and-delete operation, so
    callers must not concurrently reconfigure these same two camera endpoints.
    """

    def __init__(self) -> None:
        self.serial: str | None = None
        self._identity: tuple[str, str] | None = None
        self._owned: list[tuple[str, tuple[str, str, str]]] = []
        self._active = False

    @staticmethod
    def _adb(*arguments: str) -> str:
        try:
            return subprocess.run(
                ["adb", *arguments], check=True, capture_output=True,
                text=True, timeout=5).stdout
        except FileNotFoundError as error:
            raise RuntimeError("ADB is required for wired PICO video; install adb first") from error
        except subprocess.TimeoutExpired as error:
            raise RuntimeError("ADB video setup/cleanup timed out; check the USB connection") from error
        except subprocess.CalledProcessError as error:
            detail = (error.stderr or error.stdout or str(error)).strip()
            raise RuntimeError(f"ADB video setup/cleanup failed: {detail}") from error

    def _devices(self) -> dict[str, tuple[str, str]]:
        devices = {}
        for line in self._adb("devices", "-l").splitlines():
            fields = line.split()
            if len(fields) < 2 or line.startswith("List of devices"):
                continue
            transport = next((field.split(":", 1)[1] for field in fields[2:]
                              if field.startswith("transport_id:")), "")
            devices[fields[0]] = (fields[1], transport)
        return devices

    def _rules(self, direction: str) -> list[tuple[str, str, str]]:
        arguments = ("forward", "--list") if direction == "forward" else (
            "-s", self.serial, "reverse", "--list")
        rows = []
        for line in self._adb(*arguments).splitlines():
            fields = line.split()
            if not fields:
                continue
            if len(fields) != 3:
                raise RuntimeError(f"Unrecognized ADB {direction} rule: {line!r}")
            rows.append((self.serial if direction == "reverse" else fields[0],
                         fields[1], fields[2]))
        return rows

    def _ensure(self, direction: str, port: int) -> None:
        endpoint = f"tcp:{port}"
        owners = [row for row in self._rules(direction) if row[1] == endpoint]
        if owners:
            if (len(owners) != 1 or owners[0][2] != endpoint
                    or (direction == "forward" and owners[0][0] != self.serial)):
                raise RuntimeError(f"ADB {direction} {endpoint} already maps to another device/port; "
                                   "refusing to replace it")
            return
        self._adb("-s", self.serial, direction, "--no-rebind", endpoint, endpoint)
        # Record successful creation before verification so later setup failures
        # can roll it back, but never assume the reverse transport-label value.
        provisional = (self.serial, endpoint, endpoint)
        self._owned.append((direction, provisional))
        owners = [row for row in self._rules(direction) if row[1] == endpoint]
        if (len(owners) != 1 or owners[0][2] != endpoint
                or (direction == "forward" and owners[0][0] != self.serial)):
            raise RuntimeError(f"ADB did not retain the requested {direction} {endpoint} rule")
        self._owned[-1] = (direction, owners[0])

    def start(self) -> AdbVideoBridge:
        if self._active:
            return self
        if self._owned:
            raise RuntimeError("Previous ADB cleanup failed; call close() before restarting")
        devices = self._devices()
        serial = os.environ.get("ANDROID_SERIAL")
        if not serial:
            if len(devices) != 1:
                raise RuntimeError("Connect one PICO headset and authorize USB debugging; "
                                   "with multiple devices set ANDROID_SERIAL explicitly")
            serial = next(iter(devices))
        if serial not in devices or devices[serial][0] != "device":
            raise RuntimeError(f"PICO {serial} is not authorized/online; "
                               "accept USB debugging in the headset")
        if not devices[serial][1]:
            raise RuntimeError("ADB did not report a device transport_id; update adb for safe video cleanup")
        self.serial = serial
        self._identity = devices[serial]
        try:
            self._ensure("reverse", CONTROL_PORT)
            self._ensure("forward", VIDEO_PORT)
        except Exception as error:
            try:
                self.close()
            except RuntimeError as cleanup_error:
                raise RuntimeError(f"{error}; rollback also failed: {cleanup_error}") from error
            raise
        self._active = True
        return self

    def close(self) -> None:
        self._active = False
        if not self._owned:
            return
        if self._devices().get(self.serial) != self._identity:
            # A replacement/reconnected transport is not ours to modify.
            self._owned.clear()
            return
        errors = []
        for direction, expected in list(reversed(self._owned)):
            try:
                owners = [row for row in self._rules(direction) if row[1] == expected[1]]
                if owners == [expected]:
                    self._adb("-s", self.serial, direction, "--remove", expected[1])
                self._owned.remove((direction, expected))
            except RuntimeError as error:
                errors.append(str(error))
        if errors:
            raise RuntimeError("; ".join(errors))

    def __enter__(self) -> AdbVideoBridge:
        return self.start()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
