import os
import subprocess
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from pico2_hands.run_sim import ensure_adb_forward


class AdbDevice:
    def __init__(self, devices=None, forwards=None):
        self.devices = devices if devices is not None else {"pico": "device"}
        self.forwards = list(forwards or [])
        self.changes = 0

    def __call__(self, command, **kwargs):
        args = command[1:]
        if args == ["devices"]:
            text = "List of devices attached\n" + "\n".join(
                f"{serial}\t{state}" for serial, state in self.devices.items())
        elif args == ["forward", "--list"]:
            text = "\n".join(" ".join(row) for row in self.forwards)
        elif args[:4] == ["-s", "pico", "forward", "--no-rebind"]:
            local, remote = args[4:]
            if any(row[1] == local for row in self.forwards):
                raise subprocess.CalledProcessError(1, command, stderr="cannot rebind")
            self.forwards.append(["pico", local, remote])
            self.changes += 1
            text = ""
        else:
            raise AssertionError(f"Unexpected or unsafe ADB operation: {command}")
        return SimpleNamespace(stdout=text)


class AdbPreflightTest(unittest.TestCase):
    def setUp(self):
        environment = patch.dict(os.environ, {"ANDROID_SERIAL": ""})
        environment.start()
        self.addCleanup(environment.stop)

    def test_missing_forward_created_once_then_reused(self):
        adb = AdbDevice()
        with patch("pico2_hands.run_sim.subprocess.run", side_effect=adb):
            ensure_adb_forward(10002)
            ensure_adb_forward(10002)
        self.assertEqual(adb.forwards, [["pico", "tcp:10002", "tcp:10002"]])
        self.assertEqual(adb.changes, 1)

    def test_conflicting_forward_is_not_replaced(self):
        original = [["pico", "tcp:10002", "tcp:15000"]]
        adb = AdbDevice(forwards=original)
        with patch("pico2_hands.run_sim.subprocess.run", side_effect=adb):
            with self.assertRaisesRegex(RuntimeError, "refusing to replace"):
                ensure_adb_forward(10002)
        self.assertEqual(adb.forwards, original)
        self.assertEqual(adb.changes, 0)

    def test_multiple_devices_require_explicit_serial(self):
        adb = AdbDevice(devices={"pico": "device", "other": "device"})
        with patch("pico2_hands.run_sim.subprocess.run", side_effect=adb):
            with self.assertRaisesRegex(RuntimeError, "ANDROID_SERIAL"):
                ensure_adb_forward(10002)
            self.assertEqual(adb.changes, 0)
            with patch.dict(os.environ, {"ANDROID_SERIAL": "pico"}):
                ensure_adb_forward(10002)
        self.assertEqual(adb.forwards, [["pico", "tcp:10002", "tcp:10002"]])

    def test_unauthorized_device_cannot_create_forward(self):
        adb = AdbDevice(devices={"pico": "unauthorized"})
        with patch("pico2_hands.run_sim.subprocess.run", side_effect=adb):
            with self.assertRaisesRegex(RuntimeError, "USB debugging"):
                ensure_adb_forward(10002)
        self.assertEqual(adb.changes, 0)
