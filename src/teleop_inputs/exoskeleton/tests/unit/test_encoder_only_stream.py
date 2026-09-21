"""Encoder-v1 regressions using real loopback HTTP/TCP, never glove hardware."""

from __future__ import annotations

import hashlib
import json
import socket
import struct
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from data_glove_wuji_teleop.adapters.glove.encoder_stream import (
    ENCODER, EncoderConnection, ProtocolError,
)
from tests.unit.test_dgst_stream import encoded, line, metadata_files, metadata_tar


CS_BY_JOINT = [18, 3, 7, 11, 15, 20, 14, 10, 6, 2, 17, 13, 9, 5, 1, 16, 12, 8, 4, 0, 19]
TIMESTAMP_NS = 9_007_199_254_740_993
RAW_ANGLES = [347.75, 0.0, 359.875] + [float(joint) for joint in range(3, 21)]


def encoder_packet(sequence: int, *, dropped: int = 0,
                   angles: list[float] | None = None) -> bytes:
    values = RAW_ANGLES if angles is None else angles
    return struct.pack("!4sBBHIQI", b"DGEC", 1, 0, 21, sequence,
                       TIMESTAMP_NS + sequence, dropped) + struct.pack("!21f", *values)


class EncoderOnlyServer:
    """Release binary samples explicitly, after the client's handshake finishes."""

    def __init__(self, script=None, *, hello_changes: dict | None = None):
        files = metadata_files()
        # The reused archive identifies 0x000000e04c1a0399 / left. Its default
        # sequential CS mapping is replaced with the real encoder-v1 mapping.
        sensors = json.loads(files["sensors.json"])
        sensors["encoder"]["cs_by_joint"] = CS_BY_JOINT
        files["sensors.json"] = encoded(sensors)
        self.body = metadata_tar(files)
        self.etag = f'"{hashlib.sha256(self.body).hexdigest()}"'
        self.script = script
        self.hello_changes = hello_changes or {}
        self.proceed = threading.Event()
        self.errors: list[BaseException] = []
        self.http_peers: list[str] = []
        self.tcp_peer = None
        self.start = None
        self.stop = None
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
                    "ok": True, "version": 1, "channels": 21, "unit": "deg",
                    "clock": "CLOCK_MONOTONIC_ns", "order": "J1..J21",
                    "cs_by_joint": CS_BY_JOINT, **self.hello_changes,
                }
                raw = encoded(hello) + b"\n"
                for offset in range(0, len(raw), 7):
                    sock.sendall(raw[offset:offset + 7])
                if self.script is None:
                    # Keep the stream open: rejection must be caused by the
                    # invalid handshake, not an unrelated early EOF.
                    if sock.recv(1):
                        raise AssertionError("rejected handshake sent another command")
                else:
                    if not self.proceed.wait(3):
                        raise AssertionError("fixture was not released")
                    self.script(sock, self)
        except BaseException as exc:
            self.errors.append(exc)

    def stopped(self, sock: socket.socket, tail: bytes):
        self.stop = line(sock)
        if self.stop != {"cmd": "stop"}:
            raise AssertionError(f"unexpected STOP: {self.stop!r}")
        sock.sendall(tail)
        # encoder-v1 confirms STOP by EOF, not a JSON response or DGST END.

    def connect(self, **kwargs) -> EncoderConnection:
        return EncoderConnection.connect(
            "127.0.0.1", port=self.port, http_port=self.http.server_port,
            timeout=2, protocol="encoder-v1", **kwargs,
        )

    def __enter__(self):
        self.http_thread.start()
        self.tcp_thread.start()
        return self

    def __exit__(self, exc_type, _value, _traceback):
        self.proceed.set()
        self.tcp_thread.join(4)
        self.http.shutdown()
        self.http.server_close()
        self.http_thread.join(2)
        self.listener.close()
        if exc_type is None and self.errors:
            raise self.errors[0]
        if exc_type is None and self.tcp_thread.is_alive():
            raise AssertionError("TCP fixture did not terminate")


class EncoderOnlyStreamTests(unittest.TestCase):
    def test_raw_angles_timestamp_source_binding_and_stop_drain(self):
        def send(sock, server):
            raw = encoder_packet(0)
            for offset in range(0, len(raw), 11):
                sock.sendall(raw[offset:offset + 11])
            server.stopped(sock, encoder_packet(1))

        with EncoderOnlyServer(send) as server:
            with server.connect(source_address="127.0.0.2", expected_hand="left",
                                expected_device_id="0x000000e04c1a0399") as connection:
                server.proceed.set()
                frame = connection.read_latest_frame()
                self.assertEqual(frame.angles_deg, RAW_ANGLES)
                self.assertEqual(frame.timestamp_ns, TIMESTAMP_NS)
                self.assertEqual(frame.sequence, 0)
                self.assertEqual(frame.dropped, 0)
                self.assertEqual(connection.protocol, "encoder-v1")
                self.assertEqual(connection.cs_by_joint, CS_BY_JOINT)
                self.assertFalse(connection.zeroed)
                self.assertFalse(connection.complete)
                self.assertIsNone(connection.stop_confirmation)
            self.assertTrue(connection.complete)
            self.assertEqual(connection.stop_confirmation, "eof")
            self.assertEqual(connection.counts[ENCODER], 2)
            self.assertEqual(server.start, {"cmd": "start", "stream": "encoder", "version": 1})
            self.assertEqual(server.stop, {"cmd": "stop"})
            self.assertEqual(server.http_peers, ["127.0.0.2"])
            self.assertEqual(server.tcp_peer, "127.0.0.2")

    def test_eof_without_stop_is_a_persistent_stream_failure(self):
        release_eof = threading.Event()

        def send(sock, _server):
            sock.sendall(encoder_packet(0))
            if not release_eof.wait(3):
                raise AssertionError("EOF was not released")

        with EncoderOnlyServer(send) as server:
            connection = server.connect()
            try:
                server.proceed.set()
                self.assertEqual(connection.read_frame().sequence, 0)
                release_eof.set()
                with self.assertRaises(EOFError):
                    connection.read_frame()
                with self.assertRaises(EOFError):
                    connection.read_latest_frame()
                with self.assertRaises(EOFError):
                    connection.close()
                self.assertFalse(connection.complete)
                self.assertIsNone(connection.stop_confirmation)
            finally:
                release_eof.set()
                try:
                    connection.close()
                except EOFError:
                    pass

    def test_stop_does_not_accept_truncated_header_or_payload(self):
        for label, tail in (("header", encoder_packet(1)[:12]),
                            ("payload", encoder_packet(1)[:-1])):
            with self.subTest(boundary=label):
                self.assert_stop_tail_rejected(tail, EOFError)

    def test_stop_drain_rejects_lost_or_invalid_scans(self):
        faults = {
            "sequence_gap": encoder_packet(2),
            "dropped_scan": encoder_packet(1, dropped=1),
            "nonfinite_angle": encoder_packet(1, angles=[float("nan")] + RAW_ANGLES[1:]),
        }
        for label, tail in faults.items():
            with self.subTest(fault=label):
                self.assert_stop_tail_rejected(tail)

    def assert_stop_tail_rejected(self, tail: bytes, error_type=ProtocolError):
        def send(sock, server):
            sock.sendall(encoder_packet(0))
            server.stopped(sock, tail)

        with EncoderOnlyServer(send) as server:
            connection = server.connect()
            server.proceed.set()
            self.assertEqual(connection.read_frame().sequence, 0)
            with self.assertRaises(error_type):
                connection.close()
            self.assertEqual(server.stop, {"cmd": "stop"})
            self.assertFalse(connection.complete)
            self.assertIsNone(connection.stop_confirmation)
            with self.assertRaises(error_type):
                connection.read_frame()
            with self.assertRaises(error_type):
                connection.read_latest_frame()

    def test_handshake_cs_mapping_must_match_http_metadata(self):
        wrong_cs = list(CS_BY_JOINT)
        wrong_cs[0], wrong_cs[1] = wrong_cs[1], wrong_cs[0]
        with EncoderOnlyServer(hello_changes={"cs_by_joint": wrong_cs}) as server:
            with self.assertRaises(ProtocolError):
                server.connect()


if __name__ == "__main__":
    unittest.main()
