"""Fresh, read-only RGB-to-PICO transport using one owned FFmpeg process.

The source callable must be nonblocking and return its latest immutable frame.
Only raw frames may be skipped. Once submitted to x264, an access unit is sent
in order or the whole connection is closed; encoded P-frames are never dropped.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import errno
import os
import re
import select
import shutil
import socket
import struct
import subprocess
import threading
import time
from typing import Callable

from tianji_runtime.constants import CAMERA_FPS, IMAGE_HEIGHT, IMAGE_WIDTH


FRESHNESS_NS = 250_000_000
FUTURE_TOLERANCE_NS = 5_000_000
SOURCE_TIMEOUT_NS = 2_000_000_000
_MAX_PACKET = 16 * 1024 * 1024
_MAX_PENDING = 2
_POLL_SECONDS = 0.02
_START_CODE = re.compile(b"\x00\x00(?:\x00)?\x01")


@dataclass(frozen=True)
class RgbFrame:
    sequence: int
    received_ns: int
    stamp_ns: int
    data: bytes


class _AccessUnits:
    """Incremental AnnexB packets, delimited by FFmpeg's packet-size sidecar.

    Waiting for the *next* AUD would add a whole frame of latency, and would
    leave a final frame stuck forever. The tee framecrc muxer reports the exact
    size of each SAME encoded packet, so arbitrary pipe fragmentation is safe,
    including at 1 fps. CRC text is diagnostic only, not sent to the headset.
    """

    def __init__(self) -> None:
        self.data = bytearray()
        self.metadata = bytearray()
        self.sizes: deque[int] = deque()
        self.first = True

    def append_data(self, chunk: bytes) -> None:
        if len(self.data) + len(chunk) > _MAX_PACKET * _MAX_PENDING:
            raise RuntimeError("FFmpeg H264 output exceeded bounded packet storage")
        self.data.extend(chunk)

    def append_metadata(self, chunk: bytes) -> None:
        self.metadata.extend(chunk)
        while True:
            end = self.metadata.find(b"\n")
            if end < 0:
                break
            line = bytes(self.metadata[:end]).strip()
            del self.metadata[:end + 1]
            if not line or line.startswith(b"#"):
                continue
            fields = line.split(b",")
            if len(fields) < 6 or int(fields[0]) != 0:
                raise RuntimeError("Unexpected FFmpeg packet metadata")
            size = int(fields[4])
            if not 0 < size <= _MAX_PACKET or len(self.sizes) >= _MAX_PENDING:
                raise RuntimeError("FFmpeg packet size/count exceeded bounds")
            self.sizes.append(size)
        if len(self.metadata) > 16_384:
            raise RuntimeError("Unterminated FFmpeg packet metadata")

    def pop(self) -> bytes | None:
        if not self.sizes or len(self.data) < self.sizes[0]:
            return None
        size = self.sizes.popleft()
        packet = bytes(self.data[:size])
        del self.data[:size]
        starts = list(_START_CODE.finditer(packet))
        if not starts or starts[0].start() != 0:
            raise RuntimeError("FFmpeg emitted non-AnnexB H264")
        types = []
        for index, match in enumerate(starts):
            end = starts[index + 1].start() if index + 1 < len(starts) else size
            if match.end() >= end:
                raise RuntimeError("FFmpeg emitted an empty H264 NAL")
            types.append(packet[match.end()] & 0x1f)
        if types[0] != 9 or types.count(9) != 1 or not any(t in (1, 5) for t in types):
            raise RuntimeError("FFmpeg packet is not one AUD-delimited access unit")
        if self.first:
            if not all(t in types for t in (7, 8, 5)):
                raise RuntimeError("First H264 access unit lacks SPS/PPS/IDR")
            self.first = False
        return packet


class H264StreamError(RuntimeError):
    """The current video session failed; its resources still need closing."""


class H264CleanupError(RuntimeError):
    """Owned resources could not be released safely; do not reopen a session."""


class H264Sender:
    """Owned nonblocking encoder/socket worker; call check() while streaming.

    start() launches the worker (connection/encoder errors surface via check()).
    close() is bounded and reports cleanup and unhandled stream failures.
    Instances are single-use. Frame stamps are Unix wall-clock nanoseconds;
    receive stamps are monotonic nanoseconds. No files or recordings are made.
    """

    def __init__(
        self, source: Callable[[], RgbFrame | None], width: int, height: int,
        fps: int, bitrate: int, host: str = "127.0.0.1", port: int = 12345,
        *, video_transform: Callable[[bytes], bytes | bytearray] | None = None,
    ) -> None:
        if not 4 <= width <= 4096 or width % 4:
            raise ValueError("H264 width must be a multiple of four in [4, 4096]")
        if not 2 <= height <= 2160 or height % 2:
            raise ValueError("H264 height must be even in [2, 2160]")
        if not 1 <= fps <= CAMERA_FPS:
            raise ValueError(f"H264 output fps must be in [1, {CAMERA_FPS}]")
        if not 0 < bitrate <= 100_000_000:
            raise ValueError("H264 bitrate must be in (0, 100000000]")
        if host != "127.0.0.1" or port != 12345:
            raise ValueError("PICO video is wired-only at 127.0.0.1:12345")
        self._source = source
        self._video_transform = video_transform
        self._width = width
        self._height = height
        self._fps = fps
        self._bitrate = bitrate
        self._host = host
        self._port = port
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._closed = False
        self._stream_error: Exception | None = None
        self._cleanup_errors: list[Exception] = []
        self._frames_sent = 0

    @property
    def frames_sent(self) -> int:
        with self._lock:
            return self._frames_sent

    def start(self) -> None:
        with self._lock:
            if self._closed or self._thread is not None:
                raise RuntimeError("H264Sender cannot be started twice or after close")
            self._thread = threading.Thread(target=self._run, name="pico-h264")
            self._thread.start()

    def check(self) -> None:
        self._check_errors(stream_error_handled=False)

    def _check_errors(self, *, stream_error_handled: bool) -> None:
        with self._lock:
            cleanup_errors = tuple(self._cleanup_errors)
            stream_error = self._stream_error
        if cleanup_errors:
            raise H264CleanupError("PICO video cleanup failed: " + "; ".join(
                str(error) for error in cleanup_errors)) from cleanup_errors[0]
        if stream_error is not None and not stream_error_handled:
            raise H264StreamError(f"PICO video failed: {stream_error}") from stream_error

    def close(self, *, stream_error_handled: bool = False) -> None:
        """Release once; only an already-reported stream failure may be acknowledged.

        Cleanup errors and a worker that cannot stop always remain fatal.
        """
        with self._lock:
            self._closed = True
            thread = self._thread
        self._stop.set()
        if thread is not None:
            thread.join(timeout=4.0)
            if thread.is_alive():
                raise H264CleanupError("PICO worker did not stop; source must be nonblocking")
        self._check_errors(stream_error_handled=stream_error_handled)

    def _record_error(self, error: Exception, *, cleanup: bool = False) -> None:
        with self._lock:
            if cleanup:
                self._cleanup_errors.append(error)
            else:
                self._stream_error = error

    def _command(self, ffmpeg: str, metadata_fd: int) -> list[str]:
        graph = (
            f"[0:v]scale={self._width // 2}:{self._height}:flags=bilinear,"
            "format=yuv420p,split=2[left][right];"
            "[left][right]hstack=inputs=2[sbs]"
        )
        return [
            ffmpeg, "-hide_banner", "-loglevel", "error", "-nostdin",
            "-filter_complex_threads", "1", "-probesize", "32", "-analyzeduration", "0",
            "-f", "rawvideo", "-pixel_format", "rgb24",
            "-video_size", f"{IMAGE_WIDTH}x{IMAGE_HEIGHT}",
            "-framerate", str(self._fps), "-i", "pipe:0",
            "-filter_complex", graph, "-map", "[sbs]", "-an", "-vsync", "0",
            "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
            "-profile:v", "baseline", "-pix_fmt", "yuv420p", "-threads", "2",
            "-b:v", str(self._bitrate), "-maxrate", str(self._bitrate),
            "-bufsize", str(self._bitrate), "-g", str(self._fps),
            "-keyint_min", str(self._fps), "-bf", "0",
            "-x264-params", "scenecut=0:repeat-headers=1:aud=1:rc-lookahead=0:sync-lookahead=0",
            "-f", "tee",
            f"[f=h264:flush_packets=1]pipe:1|[f=framecrc:flush_packets=1]pipe:{metadata_fd}",
        ]

    @staticmethod
    def _fresh(received_ns: int, stamp_ns: int, stage: str = "source") -> None:
        receive_age = time.monotonic_ns() - received_ns
        stamp_age = time.time_ns() - stamp_ns
        if (not 0 <= receive_age <= FRESHNESS_NS or
                not -FUTURE_TOLERANCE_NS <= stamp_age <= FRESHNESS_NS):
            raise TimeoutError(
                f"Camera frame is stale or has a future timestamp at {stage} "
                f"(receive_age={receive_age / 1_000_000:.3f} ms, "
                f"stamp_age={stamp_age / 1_000_000:.3f} ms; "
                "250 ms age / 5 ms future limit)")

    def _connect(self, sock: socket.socket) -> None:
        result = sock.connect_ex((self._host, self._port))
        if result not in (0, errno.EINPROGRESS, errno.EWOULDBLOCK, errno.EALREADY):
            raise OSError(result, os.strerror(result))
        deadline = time.monotonic() + 2.0
        while result and not self._stop.is_set():
            if time.monotonic() >= deadline:
                raise TimeoutError("PICO video connection timed out")
            _, writable, exceptional = select.select([], [sock], [sock], _POLL_SECONDS)
            if writable or exceptional:
                result = sock.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
                if result:
                    raise OSError(result, os.strerror(result))
                return

    def _run(self) -> None:
        process: subprocess.Popen | None = None
        sock: socket.socket | None = None
        metadata_read: int | None = None
        metadata_write: int | None = None
        stderr = bytearray()
        try:
            ffmpeg = shutil.which("ffmpeg")
            if ffmpeg is None:
                raise RuntimeError("ffmpeg is required (with the libx264 encoder)")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setblocking(False)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 64 * 1024)
            self._connect(sock)
            if self._stop.is_set():
                return
            metadata_read, metadata_write = os.pipe()
            process = subprocess.Popen(
                self._command(ffmpeg, metadata_write), stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0,
                pass_fds=(metadata_write,),
            )
            os.close(metadata_write)
            metadata_write = None
            for stream in (process.stdin, process.stdout, process.stderr):
                os.set_blocking(stream.fileno(), False)
            os.set_blocking(metadata_read, False)
            self._stream(process, sock, metadata_read, stderr)
        except Exception as error:
            if not self._stop.is_set():
                detail = stderr.decode("utf-8", errors="replace").strip()
                self._record_error(RuntimeError(f"{error}; ffmpeg: {detail}") if detail else error)
        finally:
            self._stop.set()
            if sock is not None:
                try:
                    # Do not drain queued video during shutdown.
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError as error:
                    if error.errno not in (errno.ENOTCONN, errno.EBADF):
                        self._record_error(error, cleanup=True)
                try:
                    sock.close()
                except OSError as error:
                    self._record_error(error, cleanup=True)
            if process is not None:
                try:
                    if process.poll() is None:
                        process.terminate()
                    try:
                        process.wait(timeout=1.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=1.0)
                except Exception as error:
                    self._record_error(error, cleanup=True)
                finally:
                    for stream in (process.stdin, process.stdout, process.stderr):
                        try:
                            stream.close()
                        except Exception as error:
                            self._record_error(error, cleanup=True)
            for fd in (metadata_read, metadata_write):
                if fd is not None:
                    try:
                        os.close(fd)
                    except OSError as error:
                        self._record_error(error, cleanup=True)

    def _stream(self, process: subprocess.Popen, sock: socket.socket,
                metadata_fd: int, stderr: bytearray) -> None:
        parser = _AccessUnits()
        # Metadata only: raw bytes live solely in the current input memoryview.
        pending: deque[tuple[int, int]] = deque()
        raw: memoryview | None = None
        output: deque[memoryview] = deque()
        input_fd = process.stdin.fileno()
        output_fd = process.stdout.fileno()
        error_fd = process.stderr.fileno()
        read_fds = [output_fd, metadata_fd, error_fd, sock]
        started_ns = time.monotonic_ns()
        last_sequence: int | None = None
        last_source: tuple[int, int] | None = None
        next_input_ns = started_ns
        interval_ns = 1_000_000_000 // self._fps
        while not self._stop.is_set():
            now = time.monotonic_ns()
            if pending:
                self._fresh(*pending[0], stage="pending encoded delivery")
            frame = self._source()
            if frame is not None:
                self._fresh(frame.received_ns, frame.stamp_ns)
                last_source = (frame.received_ns, frame.stamp_ns)
            elif last_source is not None:
                self._fresh(*last_source)
            elif now - started_ns > SOURCE_TIMEOUT_NS:
                raise TimeoutError("No camera frame arrived within two seconds")
            if (frame is not None and raw is None and len(pending) < _MAX_PENDING
                    and now >= next_input_ns
                    and (last_sequence is None or frame.sequence > last_sequence)):
                if not isinstance(frame.data, bytes) or len(frame.data) != IMAGE_WIDTH * IMAGE_HEIGHT * 3:
                    raise ValueError("Camera source must provide tightly packed immutable RGB8 bytes")
                # HUD work happens only after frame admission, never in the poll loop
                # or DDS callback. Source bytes/stamps and delivery deadlines stay intact.
                video = self._video_transform(frame.data) if self._video_transform else frame.data
                raw = memoryview(video)
                video = None
                pending.append((frame.received_ns, frame.stamp_ns))
                last_sequence = frame.sequence
                # Never catch up by producing a burst after a delayed iteration.
                next_input_ns = now + interval_ns
            frame = None
            if not output:
                packet = parser.pop()
                if packet is not None:
                    if not pending:
                        raise RuntimeError("FFmpeg produced an access unit without a source frame")
                    self._fresh(*pending[0], stage="encoded packet")
                    output.extend((memoryview(struct.pack(">I", len(packet))), memoryview(packet)))
            writable = ([input_fd] if raw is not None else []) + ([sock] if output else [])
            readable, writable, _ = select.select(read_fds, writable, [], _POLL_SECONDS)
            for fd in readable:
                if fd is sock:
                    if not sock.recv(1024):
                        raise ConnectionError("PICO video receiver disconnected")
                    raise RuntimeError("Unexpected data on PICO's receive-only video socket")
                chunk = os.read(fd, 65_536)
                if not chunk:
                    if fd == error_fd:
                        read_fds.remove(error_fd)
                        continue
                    raise RuntimeError(f"FFmpeg closed its output (exit={process.poll()})")
                if fd == output_fd:
                    parser.append_data(chunk)
                elif fd == metadata_fd:
                    parser.append_metadata(chunk)
                else:
                    stderr.extend(chunk)
                    del stderr[:-8192]
            if input_fd in writable and raw is not None:
                self._fresh(*pending[-1], stage="encoder input")
                try:
                    written = os.write(input_fd, raw)
                except BlockingIOError:
                    written = 0
                raw = raw[written:]
                if not raw:
                    raw = None
            if sock in writable and output:
                self._fresh(*pending[0], stage="socket send")
                try:
                    sent = sock.send(output[0])
                except BlockingIOError:
                    continue
                if sent <= 0:
                    raise ConnectionError("PICO video send made no progress")
                output[0] = output[0][sent:]
                if not output[0]:
                    output.popleft()
                if not output:
                    self._fresh(*pending.popleft(), stage="completed delivery")
                    with self._lock:
                        self._frames_sent += 1
            if process.poll() is not None:
                raise RuntimeError(f"FFmpeg exited unexpectedly with status {process.returncode}")
