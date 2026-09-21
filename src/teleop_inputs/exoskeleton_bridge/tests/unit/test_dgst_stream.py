"""DGST protocol regressions using local HTTP and TCP only, never glove hardware."""

from __future__ import annotations

import hashlib
import io
import json
import socket
import struct
import tarfile
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from unittest.mock import patch

from data_glove_wuji_teleop.adapters.glove.dgst_protocol import parse_metadata
from data_glove_wuji_teleop.adapters.glove.encoder_stream import (
    ENCODER, END, ERROR, FIRST, HEADER, HELLO, IMU900, KEY, LAST,
    MAGIC, VERSION, VIDEO, EncoderConnection, ProtocolError,
)


def encoded(value: dict) -> bytes:
    return json.dumps(value, separators=(",", ":")).encode("utf-8")


def metadata_files() -> dict[str, bytes]:
    files = {
        "calibration.json": encoded({"status": "uncalibrated"}),
        # Deliberately contains a zero and direction that must never affect samples.
        "encoder_calibration.json": encoded({
            "joints": [{"joint": "J1", "cs": 0, "zero_deg": 120, "direction": -1}],
        }),
    }
    files["device.json"] = encoded({
        "schema": "dataglove.stream_device", "schema_version": 1,
        "device_id": "0x000000e04c1a0399", "hand": "left",
        "boot_id": "33344e97-ac9c-4ac6-b4e7-c67ce71c2c8d",
        **{name: {"file": f"{name}.json", "status": "uncalibrated",
                  "sha256": hashlib.sha256(files[f"{name}.json"]).hexdigest()}
           for name in ("calibration", "encoder_calibration")},
    })
    files["sensors.json"] = encoded({
        "schema": "dataglove.stream_sensors", "schema_version": 1,
        "timestamp": {"field": "timestamp_ns", "unit": "ns", "clock": "CLOCK_MONOTONIC",
                      "scope": "same_device_boot_only"},
        "encoder": {
            "channels": 21, "cs_by_joint": list(range(21)), "unit": "deg",
            "angle_convention": "raw_absolute_0_360", "range": [0, 360],
            "range_upper_exclusive": True, "joint_order": "J1..Jn",
            "calibration_applied_to_samples": False,
        },
    })
    return files


def metadata_tar(files: dict[str, bytes] | None = None, extra: tarfile.TarInfo | None = None) -> bytes:
    output = io.BytesIO()
    with tarfile.open(fileobj=output, mode="w", format=tarfile.USTAR_FORMAT) as archive:
        for name, data in (metadata_files() if files is None else files).items():
            member = tarfile.TarInfo(name)
            member.size = len(data)
            archive.addfile(member, io.BytesIO(data))
        if extra is not None:
            archive.addfile(extra, io.BytesIO(b"{}") if extra.size else None)
    return output.getvalue()


def packet(kind: int, payload: bytes, *, sequence: int = 0, timestamp: int = 0,
           flags: int = FIRST | LAST, stream: int = 17, offset: int = 0,
           total: int | None = None) -> bytes:
    return HEADER.pack(MAGIC, VERSION, kind, flags, len(payload), sequence,
                       timestamp, stream, offset, len(payload) if total is None else total) + payload


def encoder(sequence: int, angle: float = 12.5, **kwargs) -> bytes:
    return packet(ENCODER, struct.pack("<H21d", 21, *([angle] * 21)),
                  sequence=sequence, timestamp=1_000_000_000 + sequence, **kwargs)


def line(sock: socket.socket) -> dict:
    data = bytearray()
    while not data.endswith(b"\n"):
        chunk = sock.recv(1)
        if not chunk:
            raise EOFError("fixture client closed before command")
        data.extend(chunk)
        if len(data) > 4096:
            raise AssertionError("client command is not bounded")
    return json.loads(data)


class DgstServer:
    """Real loopback fixture, releasing samples only after connect returns."""

    def __init__(self, script=None, *, body: bytes | None = None,
                 etag: str | None = None, hello_changes: dict | None = None,
                 expect_tcp: bool = True):
        self.body = metadata_tar() if body is None else body
        self.digest = hashlib.sha256(self.body).hexdigest()
        self.etag = f'"{self.digest}"' if etag is None else etag
        self.script = script
        self.hello_changes = hello_changes or {}
        self.proceed = threading.Event()
        self.errors: list[BaseException] = []
        self.http_peers: list[str] = []
        self.tcp_peer = None
        self.start = None
        self.stop = None
        self.expect_tcp = expect_tcp
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(1)
        self.listener.settimeout(3)
        self.port = self.listener.getsockname()[1]
        owner = self

        class MetadataHandler(BaseHTTPRequestHandler):
            def do_GET(self):
                owner.http_peers.append(self.client_address[0])
                if self.path != "/api/stream/metadata":
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "application/x-tar")
                self.send_header("Content-Length", str(len(owner.body)))
                self.send_header("ETag", owner.etag)
                self.end_headers()
                self.wfile.write(owner.body)

            def log_message(self, *_args):
                pass

        self.http = ThreadingHTTPServer(("127.0.0.1", 0), MetadataHandler)
        self.http_port = self.http.server_port
        self.http_thread = threading.Thread(target=self.http.serve_forever, daemon=True)
        self.tcp_thread = threading.Thread(target=self.serve_tcp, daemon=True)

    def serve_tcp(self):
        try:
            sock, peer = self.listener.accept()
            self.tcp_peer = peer[0]
            with sock:
                sock.settimeout(3)
                self.start = line(sock)
                hello = {
                    "boot_id": "33344e97-ac9c-4ac6-b4e7-c67ce71c2c8d",
                    "metadata_sha256": self.digest,
                    "available_kinds": [VIDEO, IMU900, ENCODER],
                    "clock": "CLOCK_MONOTONIC_ns", **self.hello_changes,
                }
                # Fragment both header and JSON, independent of TCP recv boundaries.
                raw = packet(HELLO, encoded(hello))
                for start in range(0, len(raw), 7):
                    sock.sendall(raw[start:start + 7])
                if self.script is not None:
                    if not self.proceed.wait(3):
                        raise AssertionError("fixture not released")
                    self.script(sock, self)
        except BaseException as exc:
            self.errors.append(exc)

    def stopped(self, sock: socket.socket):
        self.stop = line(sock)
        sock.sendall(packet(END, encoded({"reason": "stopped"})))

    def connect(self, **kwargs) -> EncoderConnection:
        return EncoderConnection.connect("127.0.0.1", port=self.port, http_port=self.http_port,
                                         timeout=2, **kwargs)

    def __enter__(self):
        self.http_thread.start()
        if self.expect_tcp:
            self.tcp_thread.start()
        return self

    def __exit__(self, exc_type, _value, _traceback):
        self.proceed.set()
        if self.expect_tcp:
            self.tcp_thread.join(4)
        self.http.shutdown()
        self.http.server_close()
        self.http_thread.join(2)
        self.listener.close()
        if exc_type is None and self.errors:
            raise self.errors[0]
        if exc_type is None and self.expect_tcp and self.tcp_thread.is_alive():
            raise AssertionError("TCP fixture did not terminate")


class DgstStreamTests(unittest.TestCase):
    def test_fragmented_video_interleaving_raw_angles_source_binding_and_stop(self):
        def send(sock, server):
            data = (
                packet(VIDEO, b"abc", flags=FIRST | KEY, timestamp=55, total=6)
                + encoder(0)
                + packet(IMU900, struct.pack("<9d", *range(9)), timestamp=66)
                + packet(VIDEO, b"def", flags=LAST | KEY, timestamp=55, offset=3, total=6)
                + encoder(1, 359.999)
            )
            for start in range(0, len(data), 11):
                sock.sendall(data[start:start + 11])
            server.stopped(sock)

        with DgstServer(send) as server, patch.dict("os.environ", {
            "http_proxy": "http://127.0.0.1:1", "HTTP_PROXY": "http://127.0.0.1:1",
        }):
            with server.connect(source_address="127.0.0.2", expected_hand="left",
                                expected_device_id="0x000000e04c1a0399") as connection:
                server.proceed.set()
                self.assertEqual(connection.read_frame().angles_deg, [12.5] * 21)
                second = connection.read_frame()
                self.assertEqual((second.sequence, second.timestamp_ns), (1, 1_000_000_001))
                self.assertEqual(second.angles_deg, [359.999] * 21)
                self.assertFalse(connection.zeroed)
                self.assertFalse(connection.complete)
            self.assertTrue(connection.complete)
            self.assertEqual(connection.counts, {HELLO: 1, VIDEO: 1, IMU900: 1, ENCODER: 2, END: 1})
            self.assertEqual(server.start, {"command": "start", "version": 3, "metadata_sha256": server.digest})
            self.assertEqual(server.stop, {"command": "stop"})
            self.assertEqual(server.http_peers, ["127.0.0.2"])
            self.assertEqual(server.tcp_peer, "127.0.0.2")

    def test_identity_and_http_digest_rejected_before_tcp_start(self):
        cases = [
            ({"expected_device_id": "0x0000000000000001"}, None),
            ({"expected_hand": "right"}, None),
            ({}, '"' + "0" * 64 + '"'),
        ]
        for options, etag in cases:
            with self.subTest(options=options, etag=etag), DgstServer(etag=etag, expect_tcp=False) as server:
                with self.assertRaises(ProtocolError):
                    server.connect(**options)
                server.listener.settimeout(0.05)
                with self.assertRaises(TimeoutError):
                    server.listener.accept()

    def test_no_camera_and_consumer_deadline_can_resume_then_stop_normally(self):
        release_sample = threading.Event()

        def send(sock, server):
            if not release_sample.wait(2):
                raise AssertionError("sample was never released")
            sock.sendall(encoder(0))
            server.stopped(sock)

        with DgstServer(send, hello_changes={"available_kinds": [IMU900, ENCODER]}) as server:
            with server.connect() as connection:
                server.proceed.set()
                try:
                    with self.assertRaises(TimeoutError):
                        connection.read_latest_frame(deadline_monotonic=time.monotonic() + 0.03)
                finally:
                    release_sample.set()
                self.assertEqual(connection.read_frame().angles_deg, [12.5] * 21)
            self.assertTrue(connection.complete)
            self.assertEqual(connection.counts, {HELLO: 1, ENCODER: 1, END: 1})

    def test_hello_rejects_boot_digest_clock_and_missing_encoder(self):
        for changes in ({"boot_id": "different"}, {"metadata_sha256": "0" * 64},
                        {"clock": "UTC"}, {"available_kinds": [VIDEO, IMU900]},
                        {"available_kinds": [ENCODER, ENCODER]}):
            with self.subTest(changes=changes), DgstServer(hello_changes=changes) as server:
                with self.assertRaises(ProtocolError):
                    server.connect()

    def test_wire_faults_are_not_hidden_by_cached_encoder_frames(self):
        bad_messages = {
            "sequence_gap": encoder(2),
            "stream_change": encoder(1, stream=18),
            "angle_nan": encoder(1, float("nan")),
            "angle_upper_bound": encoder(1, 360),
            "imu_nonfinite": packet(IMU900, struct.pack("<9d", *([float("inf")] * 9))),
            "video_offset": packet(VIDEO, b"abc", flags=FIRST, total=6)
                            + packet(VIDEO, b"de", flags=LAST, offset=4, total=6),
            "video_key_change": packet(VIDEO, b"abc", flags=FIRST | KEY, total=6)
                                + packet(VIDEO, b"def", flags=LAST, offset=3, total=6),
            "video_missing_last": packet(VIDEO, b"abc", flags=FIRST, total=3),
            "unsolicited_end": packet(END, encoded({"reason": "stopped"})),
            "error": packet(ERROR, encoded({"code": "source_stalled", "message": "stalled"})),
        }
        for label, bad in bad_messages.items():
            def send(sock, _server):
                sock.sendall(encoder(0) + bad)

            with self.subTest(label=label), DgstServer(send) as server:
                connection = server.connect()
                server.proceed.set()
                # Waiting for EOF/error is deterministic; do not rely on recv grouping.
                with self.assertRaises(ProtocolError):
                    while True:
                        connection.read_frame()
                with self.assertRaises(ProtocolError):
                    connection.read_latest_frame()
                with self.assertRaises(ProtocolError):
                    connection.close()
                self.assertFalse(connection.complete)
                self.assertIsNone(connection.sock)

    def test_early_eof_is_incomplete_and_context_preserves_body_error(self):
        def send(sock, _server):
            sock.sendall(encoder(0))

        with DgstServer(send) as server:
            connection = server.connect()
            with self.assertRaisesRegex(ValueError, "consumer failed"):
                with connection:
                    server.proceed.set()
                    raise ValueError("consumer failed")
            self.assertFalse(connection.complete)
            self.assertIsNone(connection.sock)

    def test_stop_cannot_hide_unfinished_video_or_early_eof(self):
        for video in (False, True):
            def send(sock, server):
                sock.sendall(encoder(0))
                if video:
                    sock.sendall(packet(VIDEO, b"abc", flags=FIRST, total=6))
                    server.stopped(sock)
                else:
                    server.stop = line(sock)

            with self.subTest(video=video), DgstServer(send) as server:
                connection = server.connect()
                server.proceed.set()
                connection.read_frame()
                with self.assertRaises((EOFError, ProtocolError)):
                    connection.close()
                self.assertFalse(connection.complete)

    def test_latest_delivery_skips_backlog_without_skipping_wire_validation(self):
        def send(sock, server):
            for seq in range(600):
                sock.sendall(encoder(seq, seq % 360))
            # STOP is a deterministic barrier: all prior frames must be consumed
            # before END can be validated; complete means every sequence was checked.
            server.stopped(sock)

        with DgstServer(send) as server:
            connection = server.connect()
            server.proceed.set()
            with connection._condition:
                self.assertTrue(connection._condition.wait_for(
                    lambda: connection.counts.get(ENCODER, 0) == 600, timeout=2,
                ))
            latest = connection.read_latest_frame()
            self.assertEqual((latest.sequence, latest.angles_deg), (599, [239.0] * 21))
            connection.close()
            self.assertTrue(connection.complete)
            self.assertEqual(connection.counts[ENCODER], 600)

    def test_metadata_rejects_duplicate_unsafe_and_nonordinary_members(self):
        for name, kind in (("device.json", tarfile.REGTYPE), ("../device.json", tarfile.REGTYPE),
                           ("other.json", tarfile.SYMTYPE), ("folder", tarfile.DIRTYPE)):
            extra = tarfile.TarInfo(name)
            extra.type = kind
            extra.size = 2 if kind == tarfile.REGTYPE else 0
            raw = metadata_tar(extra=extra)
            with self.subTest(name=name), self.assertRaises(ProtocolError):
                parse_metadata(raw, f'"{hashlib.sha256(raw).hexdigest()}"')

    def test_metadata_rejects_calibrated_samples_and_duplicate_json_keys(self):
        for duplicate in (False, True):
            files = metadata_files()
            sensors = json.loads(files["sensors.json"])
            sensors["encoder"]["calibration_applied_to_samples"] = True
            files["sensors.json"] = b'{"schema":1,"schema":2}' if duplicate else encoded(sensors)
            raw = metadata_tar(files)
            with self.subTest(duplicate=duplicate), self.assertRaises(ProtocolError):
                parse_metadata(raw, f'"{hashlib.sha256(raw).hexdigest()}"')


if __name__ == "__main__":
    unittest.main()
