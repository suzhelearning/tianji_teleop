import os
import socket
import struct
import subprocess
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from tianji_cameras.pico_video_protocol import (
    AdbVideoBridge, CameraRequest, parse_camera_request, read_message,
)


def message(command, payload=b""):
    command = command.encode("utf-8")
    return (struct.pack(">I", 12 + len(command) + len(payload))
            + struct.pack("<I", len(command)) + command
            + struct.pack("<I", len(payload)) + payload)


def request(**changes):
    values = dict(width=1280, height=720, fps=60, bitrate=8_000_000,
                  hevc=0, render_mode=9, port=12345)
    values.update(changes)
    camera = "相机".encode("utf-8")
    ip = b"192.0.2.5"
    return (b"\xca\xfe\x01" + struct.pack("<7i", *values.values())
            + bytes([len(camera)]) + camera + bytes([len(ip)]) + ip)


class ProtocolTest(unittest.TestCase):
    def sockets(self):
        reader, writer = socket.socketpair()
        self.addCleanup(reader.close)
        self.addCleanup(writer.close)
        reader.settimeout(0.02)
        return reader, writer

    def test_upstream_framing_keeps_consecutive_messages_separate(self):
        reader, writer = self.sockets()
        payload = request()
        writer.sendall(message("OPEN_CAMERA", payload) + message("CLOSE_CAMERA"))
        self.assertEqual(read_message(reader), ("OPEN_CAMERA", payload))
        self.assertEqual(read_message(reader), ("CLOSE_CAMERA", b""))
        writer.close()
        self.assertEqual(read_message(reader), (None, None))

    def test_idle_timeout_is_retryable_but_partial_frame_timeout_closes(self):
        reader, writer = self.sockets()
        with self.assertRaises(TimeoutError):
            read_message(reader)
        writer.sendall(message("CLOSE_CAMERA"))
        self.assertEqual(read_message(reader), ("CLOSE_CAMERA", b""))
        writer.sendall(b"\x00")
        with self.assertRaisesRegex(ValueError, "connection closed"):
            read_message(reader)
        self.assertEqual(reader.fileno(), -1)

    def test_malformed_or_oversized_framing_is_rejected(self):
        for wire in (struct.pack(">I", 65537),
                     struct.pack(">I", 13) + struct.pack("<I", 100) + b"abcde",
                     message("X")[:-4] + struct.pack("<I", 1),
                     message("X")[:8] + b"\xff" + struct.pack("<I", 0)):
            with self.subTest(wire=wire):
                reader, writer = self.sockets()
                writer.sendall(wire)
                with self.assertRaises(ValueError):
                    read_message(reader)
                self.assertEqual(reader.fileno(), -1)

    def test_camera_request_decodes_utf8_and_preserves_requested_fps_and_mode(self):
        self.assertEqual(parse_camera_request(request()), CameraRequest(
            1280, 720, 60, 8_000_000, 9, 12345, "192.0.2.5", "相机"))

    def test_camera_request_rejects_unsafe_profiles_and_bad_string_boundaries(self):
        invalid = [request(**fields) for fields in (
            {"width": 1278}, {"height": 721}, {"fps": 61},
            {"bitrate": 100_000_001}, {"hevc": 1}, {"port": 10002})]
        invalid.extend((request()[:-1], request() + b"\x00", b"{}"))
        for payload in invalid:
            with self.subTest(payload=payload):
                with self.assertRaises(ValueError):
                    parse_camera_request(payload)


class AdbDevice:
    def __init__(self):
        self.devices = {"pico": ("device", "1")}
        self.forwards = [("pico", "tcp:10002", "tcp:10002")]
        self.reverses = []
        self.fail_forward = False
        self.fail_reverse_verification = False
        self.reverse_created = False

    def __call__(self, command, **kwargs):
        args = command[1:]
        if args == ["devices", "-l"]:
            text = "List of devices attached\n" + "\n".join(
                f"{serial} {state} transport_id:{transport}"
                for serial, (state, transport) in self.devices.items())
        elif args == ["forward", "--list"]:
            text = "\n".join(" ".join(row) for row in self.forwards) + "\n"
        else:
            if args[:2] != ["-s", "pico"]:
                raise AssertionError(f"Unexpected ADB operation: {command}")
            direction, operation = args[2:4]
            rows = self.forwards if direction == "forward" else self.reverses
            if operation == "--list":
                if self.fail_reverse_verification and self.reverse_created:
                    self.fail_reverse_verification = False
                    raise subprocess.CalledProcessError(1, command, stderr="list failed")
                text = "\n".join(" ".join(row) for row in rows) + "\n"
            elif operation == "--no-rebind":
                endpoint, remote = args[4:]
                if self.fail_forward and direction == "forward":
                    raise subprocess.CalledProcessError(1, command, stderr="forward failed")
                if any(row[1] == endpoint for row in rows):
                    raise subprocess.CalledProcessError(1, command, stderr="cannot rebind")
                rows.append(("pico" if direction == "forward" else "UsbFfs", endpoint, remote))
                self.reverse_created = direction == "reverse"
                text = ""
            elif operation == "--remove":
                endpoint, = args[4:]
                if endpoint == "tcp:10002":
                    raise AssertionError("Hand-input mapping must not be removed")
                rows[:] = [row for row in rows if row[1] != endpoint]
                text = ""
            else:
                raise AssertionError(f"Unexpected ADB operation: {command}")
        return SimpleNamespace(stdout=text)


class AdbBridgeTest(unittest.TestCase):
    def setUp(self):
        self.environment = patch.dict(os.environ, {"ANDROID_SERIAL": ""})
        self.environment.start()
        self.addCleanup(self.environment.stop)
        self.adb = AdbDevice()
        self.runner = patch("tianji_cameras.pico_video_protocol.subprocess.run", side_effect=self.adb)
        self.runner.start()
        self.addCleanup(self.runner.stop)

    def test_repeated_contexts_remove_only_created_video_routes(self):
        bridge = AdbVideoBridge()
        for _ in range(2):
            with bridge:
                self.assertIn(("pico", "tcp:12345", "tcp:12345"), self.adb.forwards)
                self.assertEqual(self.adb.reverses, [("UsbFfs", "tcp:13579", "tcp:13579")])
            bridge.close()
            self.assertEqual(self.adb.forwards, [("pico", "tcp:10002", "tcp:10002")])
            self.assertEqual(self.adb.reverses, [])

    def test_preexisting_video_routes_remain(self):
        self.adb.forwards.append(("pico", "tcp:12345", "tcp:12345"))
        self.adb.reverses.append(("UsbFfs", "tcp:13579", "tcp:13579"))
        original = (self.adb.forwards[:], self.adb.reverses[:])
        with AdbVideoBridge():
            pass
        self.assertEqual((self.adb.forwards, self.adb.reverses), original)

    def test_external_rule_replacement_is_not_removed(self):
        bridge = AdbVideoBridge().start()
        replacement = ("other", "tcp:12345", "tcp:9000")
        self.adb.forwards[-1] = replacement
        self.adb.reverses[-1] = ("UsbFfs", "tcp:13579", "tcp:9999")
        bridge.close()
        self.assertIn(replacement, self.adb.forwards)
        self.assertEqual(self.adb.reverses, [("UsbFfs", "tcp:13579", "tcp:9999")])

    def test_reconnected_transport_is_not_modified(self):
        bridge = AdbVideoBridge().start()
        self.adb.devices["pico"] = ("device", "2")
        original = (self.adb.forwards[:], self.adb.reverses[:])
        bridge.close()
        self.assertEqual((self.adb.forwards, self.adb.reverses), original)

    def test_partial_setup_rolls_back_even_if_reverse_verification_failed(self):
        for failure in ("fail_forward", "fail_reverse_verification"):
            with self.subTest(failure=failure):
                setattr(self.adb, failure, True)
                with self.assertRaises(RuntimeError):
                    AdbVideoBridge().start()
                setattr(self.adb, failure, False)
                self.assertEqual(self.adb.forwards, [("pico", "tcp:10002", "tcp:10002")])
                self.assertEqual(self.adb.reverses, [])
                self.adb.reverse_created = False

    def test_conflicting_owner_fails_without_clobbering_or_leaking_reverse(self):
        self.adb.forwards.append(("other", "tcp:12345", "tcp:12345"))
        original = self.adb.forwards[:]
        with self.assertRaisesRegex(RuntimeError, "refusing to replace"):
            AdbVideoBridge().start()
        self.assertEqual(self.adb.forwards, original)
        self.assertEqual(self.adb.reverses, [])

    def test_authorization_and_explicit_device_selection(self):
        self.adb.devices["pico"] = ("unauthorized", "1")
        with self.assertRaisesRegex(RuntimeError, "USB debugging"):
            AdbVideoBridge().start()
        self.adb.devices = {"pico": ("device", "1"), "other": ("device", "2")}
        with self.assertRaisesRegex(RuntimeError, "ANDROID_SERIAL"):
            AdbVideoBridge().start()
        with patch.dict(os.environ, {"ANDROID_SERIAL": "pico"}), AdbVideoBridge():
            self.assertIn(("pico", "tcp:12345", "tcp:12345"), self.adb.forwards)
        self.assertEqual(self.adb.reverses, [])
