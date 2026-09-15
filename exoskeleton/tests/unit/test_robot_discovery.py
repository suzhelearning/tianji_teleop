"""身份预检的原子选择和只读 SDK 生命周期。外部设备均为内存替身。"""

from __future__ import annotations

from types import SimpleNamespace
import unittest
from unittest.mock import patch

from data_glove_wuji_teleop.adapters.hardware.discovery import resolve_robot_serials


class Hand2:
    def __init__(self, serial, side, online=20):
        self.serial_number = serial
        self.side = side
        self.online = online

    def handedness(self):
        return SimpleNamespace(get=lambda: self.side)

    def online_joints_count(self):
        return SimpleNamespace(get=lambda: self.online)


class Manager:
    def __init__(self, hands):
        self.hands = hands
        self.active = set()
        self.disconnected = []
        self.connected = []
        self.fail_serial = None
        self.fail_disconnect = False

    def scan(self):
        return [SimpleNamespace(
            sn=serial, device_type="hand2", transport_type="usb", address=f"usb:{serial}",
        ) for serial in self.hands]

    def connect(self, *, sn, device_name, options):
        self.active.add(device_name)
        self.connected.append(sn)
        if sn == self.fail_serial:
            raise RuntimeError("connect failed after registration")
        return self.hands[sn]

    def disconnect(self, name):
        self.active.remove(name)
        self.disconnected.append(name)
        if self.fail_disconnect:
            raise RuntimeError("disconnect failed")


class RobotDiscoveryTest(unittest.TestCase):
    def setUp(self):
        self.manager = Manager({
            "actual-A": Hand2("actual-A", "right"),
            "actual-B": Hand2("actual-B", "left"),
        })
        sdk = SimpleNamespace(
            SdkManager=SimpleNamespace(instance=lambda: self.manager),
            DeviceType=SimpleNamespace(WujiHand2="hand2"),
            TransportType=SimpleNamespace(Udp="udp", Usb="usb"),
            ConnectOptions=lambda **kwargs: SimpleNamespace(**kwargs),
            WujiHand2=Hand2,
        )
        self.patch = patch("data_glove_wuji_teleop.adapters.hardware.discovery._load_sdk", return_value=sdk)
        self.patch.start()
        self.addCleanup(self.patch.stop)

    def test_auto_uses_real_handedness_not_serial_convention(self):
        result = resolve_robot_serials({"left": None, "right": ""})
        self.assertEqual(result, {"left": "actual-B", "right": "actual-A"})
        self.assertEqual(self.manager.active, set())
        self.assertEqual(len(self.manager.disconnected), 2)

    def test_explicit_serial_cannot_be_replaced(self):
        with self.assertRaises(RuntimeError):
            resolve_robot_serials({"left": "missing"})
        self.assertEqual(self.manager.connected, [])
        with self.assertRaises(ValueError):
            resolve_robot_serials({"left": "actual-A"})
        self.assertEqual(self.manager.active, set())

    def test_explicit_serial_resolves_even_when_same_side_has_multiple_candidates(self):
        self.manager.hands["actual-C"] = Hand2("actual-C", "left")
        self.assertEqual(resolve_robot_serials({"left": "actual-B"}), {"left": "actual-B"})
        self.assertEqual(self.manager.connected, ["actual-B"])

    def test_missing_side_never_returns_partial_result(self):
        del self.manager.hands["actual-A"]
        with self.assertRaises(RuntimeError):
            resolve_robot_serials({"left": None, "right": None})
        self.assertEqual(self.manager.active, set())

    def test_ambiguous_side_never_returns_partial_result(self):
        self.manager.hands["actual-C"] = Hand2("actual-C", "left")
        with self.assertRaises(RuntimeError):
            resolve_robot_serials({"left": None, "right": None})
        self.assertEqual(self.manager.active, set())
        self.assertEqual(len(self.manager.disconnected), 3)

    def test_same_serial_cannot_bind_both_sides(self):
        with self.assertRaises(ValueError):
            resolve_robot_serials({"left": "actual-A", "right": "actual-A"})
        self.assertEqual(self.manager.connected, [])

    def test_connection_failure_cleans_even_partially_registered_connection(self):
        self.manager.fail_serial = "actual-B"
        with self.assertRaisesRegex(RuntimeError, "connect failed"):
            resolve_robot_serials({"left": None, "right": None})
        self.assertEqual(self.manager.active, set())
        self.assertEqual(len(self.manager.disconnected), 2)

    def test_cleanup_error_does_not_hide_connection_error(self):
        self.manager.fail_serial = "actual-A"
        self.manager.fail_disconnect = True
        with self.assertRaises(ExceptionGroup) as captured:
            resolve_robot_serials({"right": "actual-A"})
        self.assertEqual(len(captured.exception.exceptions), 2)
        self.assertIn("connect failed", str(captured.exception.exceptions[0]))
        self.assertIn("disconnect failed", str(captured.exception.exceptions[1]))

    def test_offline_joint_refuses_complete_mapping(self):
        self.manager.hands["actual-A"].online = 19
        with self.assertRaisesRegex(RuntimeError, "19/20"):
            resolve_robot_serials({"left": None, "right": None})
        self.assertEqual(self.manager.active, set())

    def test_connected_identity_must_match_scan(self):
        self.manager.hands["actual-A"].serial_number = "other"
        with self.assertRaises(RuntimeError):
            resolve_robot_serials({"right": "actual-A"})
        self.assertEqual(self.manager.active, set())

    def test_network_failure_prevents_any_sdk_connection(self):
        self.manager.scan = lambda: [SimpleNamespace(
            sn="actual-A", device_type="hand2", transport_type="udp", address="192.168.100.2:8888",
        )]
        with patch("data_glove_wuji_teleop.adapters.hardware.discovery.prepare_robot_network", side_effect=RuntimeError("unsafe route")):
            with self.assertRaisesRegex(RuntimeError, "unsafe route"):
                resolve_robot_serials({"right": None})
        self.assertEqual(self.manager.connected, [])
