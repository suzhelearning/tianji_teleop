"""自动标定发现：候选范围、独立网络分配、唯一手侧和已有零位保护。"""

from __future__ import annotations

import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from data_glove_wuji_teleop.application import glove_discovery as discovery
from data_glove_wuji_teleop.application.glove_profiles import STANDARD_GLOVE_MAC, command_register_glove
from data_glove_wuji_teleop.cli import build_parser
from data_glove_wuji_teleop.profiles.dataglove.device import GloveDeviceProfile
from data_glove_wuji_teleop.profiles.dataglove.urdf_zero import URDF_ZERO_GROUPS, UrdfZeroProfile
from data_glove_wuji_teleop.project import PROJECT_ROOT


class ProbeStream:
    channels = 21

    def __init__(self, metadata):
        self.metadata = metadata
        self.cs_by_joint = metadata.sensors["encoder"]["cs_by_joint"]

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        pass

    def read_frame(self):
        return SimpleNamespace(angles_deg=[10.0] * 21)


class GloveDiscoveryTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.registry = self.root / "devices"
        self.registry.mkdir()
        self.sys = self.root / "net"
        self.sys.mkdir()
        self.prepared = []
        self.metadata = {}
        self.addresses = [{"ifname": "wifi", "addr_info": [{"family": "inet", "local": "192.168.7.1"}]}]
        self.routes = [{"table": 17000, "dst": "10.0.0.0/24", "dev": "robot"}]
        self.rules = [{"priority": 17001, "src": "all", "table": "main"}]
        self.parser = build_parser()
        patches = [
            patch.object(discovery, "device_directory", return_value=self.registry),
            patch("data_glove_wuji_teleop.application.glove_profiles.device_directory", return_value=self.registry),
            patch.object(discovery, "_SYS_CLASS_NET", self.sys),
            patch.object(discovery, "_ip_json", side_effect=self.ip_rows),
            patch.object(discovery, "_table_aliases", return_value={"main": 254, "local": 255}),
            patch.object(discovery, "prepare_glove_network", side_effect=self.prepare),
            patch.object(discovery, "fetch_metadata", side_effect=self.fetch),
            patch("data_glove_wuji_teleop.application.glove_profiles.prepare_glove_network", side_effect=self.prepare),
            patch("data_glove_wuji_teleop.application.glove_profiles.fetch_metadata", side_effect=self.fetch),
            patch("data_glove_wuji_teleop.application.glove_profiles._registration_mac", return_value=STANDARD_GLOVE_MAC),
            patch("data_glove_wuji_teleop.application.glove_profiles.socket.create_connection",
                  return_value=SimpleNamespace(close=lambda: None)),
            patch("data_glove_wuji_teleop.application.glove_profiles.EncoderConnection.connect", side_effect=self.connect),
            patch("sys.stdout"),
        ]
        for active in patches:
            active.start()
            self.addCleanup(active.stop)

    def ip_rows(self, *arguments):
        return {"address": self.addresses, "route": self.routes, "rule": self.rules}[arguments[0]]

    def prepare(self, args, *, check_only=False):
        self.prepared.append((args, check_only))

    def fetch(self, host, *, source_address, **_kwargs):
        interface = next(args.glove_interface for args, _ in reversed(self.prepared)
                         if args.glove_host_address.split("/")[0] == source_address)
        result = self.metadata[interface]
        if isinstance(result, Exception):
            raise result
        return result

    def connect(self, host, **kwargs):
        return ProbeStream(self.fetch(host, source_address=kwargs["source_address"]))

    def candidate(self, interface, hand="left", identity=1, mac=STANDARD_GLOVE_MAC):
        directory = self.sys / interface
        directory.mkdir()
        (directory / "address").write_text(mac)
        cs = json.loads((PROJECT_ROOT / f"config/dataglove/urdf/{hand}.json").read_text())["expected_cs_by_joint"]
        self.metadata[interface] = SimpleNamespace(
            device={"hand": hand, "device_id": f"0x{identity:016x}"},
            sensors={"encoder": {"channels": 21, "cs_by_joint": cs}}, sha256=f"test-{identity}",
        )

    def existing(self, identity=1, hand="left", source="192.168.7.3/24", table=17003, interface="usb0"):
        args = self.parser.parse_args([
            "register-glove", "--hand", hand, "--device-id", f"0x{identity:016x}",
            "--protocol", "encoder-v1", "--mac", STANDARD_GLOVE_MAC, "--interface", interface,
            "--host-address", source, "--route-table", str(table),
        ])
        command_register_glove(args)
        profile = GloveDeviceProfile.load(self.registry / f"{hand}-{identity:016x}.json")
        cs = profile.mapping_data["expected_cs_by_joint"]
        zero = UrdfZeroProfile.empty(hand, cs)
        for group in URDF_ZERO_GROUPS:
            zero, _ = zero.capture_group(group.name, [[12.0] * 21], max_motion_deg=1.0)
        return profile.store_zero("initial", zero, activate=True)

    def configured(self, interface, source, table):
        self.addresses.append({"ifname": interface, "addr_info": [{"family": "inet", "local": source}]})
        self.routes.append({"table": table, "dst": "192.168.7.2/32", "dev": interface, "prefsrc": source})
        self.rules.append({"priority": table, "src": f"{source}/32", "table": table})

    def test_unconfigured_candidates_get_unique_unoccupied_network_and_only_selected_side_is_registered(self):
        historical = self.existing(9, source="192.168.7.3/24", table=17003)
        original = {historical.path: historical.path.read_bytes()}
        self.candidate("usb0", "left", 1)
        self.candidate("usb1", "right", 2)
        self.candidate("wifi", mac="aa:bb:cc:dd:ee:ff")
        self.candidate("robot", mac="11:22:33:44:55:66")
        profile = discovery.discover_glove_for_calibration("left")
        self.assertEqual(profile.device_id, "0x0000000000000001")
        self.assertIsNone(profile.load_zero())
        self.assertFalse((self.registry / "right-0000000000000002.json").exists())
        first_two = self.prepared[:2]
        self.assertEqual({args.glove_interface for args, _ in first_two}, {"usb0", "usb1"})
        self.assertEqual(len({args.glove_host_address for args, _ in first_two}), 2)
        self.assertEqual(len({args.glove_route_table for args, _ in first_two}), 2)
        for args, _ in self.prepared:
            self.assertNotIn(args.glove_interface, ("wifi", "robot"))
            self.assertNotIn(args.glove_host_address, ("192.168.7.1/24", "192.168.7.3/24"))
            self.assertNotIn(args.glove_route_table, (17000, 17001, 17003))
        for path, content in original.items():
            self.assertEqual(path.read_bytes(), content)

    def test_two_same_side_ids_are_reported_and_neither_is_registered(self):
        self.candidate("usb0", "left", 1)
        self.candidate("usb1", "left", 2)
        with self.assertRaisesRegex(ValueError, "发现多个 left") as caught:
            discovery.discover_glove_for_calibration("left")
        self.assertIn("0000000000000001", str(caught.exception))
        self.assertIn("0000000000000002", str(caught.exception))
        self.assertEqual(list(self.registry.glob("*.json")), [])

    def test_missing_requested_side_does_not_register_other_hand(self):
        self.candidate("usb0", "right", 2)
        with self.assertRaisesRegex(ValueError, "未发现 left"):
            discovery.discover_glove_for_calibration("left")
        self.assertEqual(list(self.registry.glob("*.json")), [])

    def test_unknown_candidate_failure_prevents_false_unique_selection(self):
        self.candidate("usb0", "left", 1)
        self.candidate("usb1", "right", 2)
        self.metadata["usb1"] = TimeoutError("metadata timeout")
        with self.assertRaisesRegex(ValueError, "不能保证手侧唯一"):
            discovery.discover_glove_for_calibration("left")
        self.assertEqual(list(self.registry.glob("*.json")), [])

    def test_existing_profile_reuses_zero_and_check_only_never_prepares_network(self):
        profile = self.existing()
        before = {profile.path: profile.path.read_bytes()}
        self.candidate("usb0")
        self.configured("usb0", "192.168.7.3", 17003)
        result = discovery.discover_glove_for_calibration("left", prepare_network=False)
        self.assertEqual(result.zeros, profile.zeros)
        self.assertTrue(all(check_only for _, check_only in self.prepared))
        for path, content in before.items():
            self.assertEqual(path.read_bytes(), content)

    def test_wiring_mismatch_does_not_change_existing_zero_or_profile(self):
        profile = self.existing()
        before = {profile.path: profile.path.read_bytes()}
        self.candidate("usb0")
        self.metadata["usb0"].sensors["encoder"]["cs_by_joint"].reverse()
        with self.assertRaisesRegex(ValueError, "enc_map"):
            discovery.discover_glove_for_calibration("left")
        for path, content in before.items():
            self.assertEqual(path.read_bytes(), content)

    def test_hotplug_new_id_does_not_inherit_historical_source_or_zero(self):
        old = self.existing()
        original = old.path.read_bytes()
        self.candidate("usb0", "left", 2)
        self.configured("usb0", "192.168.7.3", 17003)
        new = discovery.discover_glove_for_calibration("left")
        self.assertEqual(new.network["interface"], old.network["interface"])
        self.assertNotEqual(new.network["host_address"], old.network["host_address"])
        self.assertNotEqual(new.network["route_table"], old.network["route_table"])
        self.assertNotEqual(new.path, old.path)
        self.assertIsNone(new.load_zero())
        self.assertEqual(old.path.read_bytes(), original)

    def test_runtime_matches_ids_after_usb_names_are_reused_and_preserves_calibrations(self):
        left = self.existing(interface="usb0")
        right = self.existing(2, "right", "192.168.7.4/24", 17004, "usb0")
        original = {profile.path: (profile.mapping_data, profile.zeros) for profile in (left, right)}
        self.candidate("usb0", "right", 2)
        self.candidate("usb1", "left", 1)
        self.configured("usb0", "192.168.7.3", 17003)
        self.configured("usb0", "192.168.7.4", 17004)
        self.configured("usb1", "192.168.7.5", 17005)
        resolved = discovery.resolve_glove_profiles((left, right))
        self.assertEqual([(p.device_id, p.network["interface"]) for p in resolved],
                         [(left.device_id, "usb1"), (right.device_id, "usb0")])
        self.assertNotEqual(resolved[0].network["host_address"], right.network["host_address"])
        self.assertNotEqual(resolved[0].network["route_table"], right.network["route_table"])
        for profile in resolved:
            self.assertEqual(GloveDeviceProfile.load(profile.path).network, profile.network)
        for path, (mapping, zeros) in original.items():
            loaded = GloveDeviceProfile.load(path)
            self.assertEqual(loaded.mapping_data, mapping)
            self.assertEqual(loaded.zeros, zeros)

    def test_label_filename_and_identity_can_index_the_same_independent_bundle(self):
        original = self.existing()
        moved = replace(original, path=self.registry / "custom.slot.json", name="bench-left")
        moved.save()
        original.path.unlink()
        with patch("data_glove_wuji_teleop.profiles.dataglove.device.device_directory", return_value=self.registry):
            for selector in ("bench-left", "custom.slot", "custom.slot.json", original.device_id):
                loaded = GloveDeviceProfile.load(selector)
                self.assertEqual(loaded.path, moved.path)
                self.assertEqual(loaded.zeros, original.zeros)
                self.assertEqual(loaded.device_id, original.device_id)

    def test_duplicate_labels_require_a_unique_identity_or_explicit_path(self):
        left = self.existing()
        right = self.existing(2, "right", "192.168.7.4/24", 17004)
        for profile in (left, right):
            replace(profile, name="bench").save(overwrite=True)
        with patch("data_glove_wuji_teleop.profiles.dataglove.device.device_directory", return_value=self.registry):
            with self.assertRaisesRegex(ValueError, "不唯一"):
                GloveDeviceProfile.load("bench")
            self.assertEqual(GloveDeviceProfile.load(left.device_id).hand, "left")

    def test_runtime_missing_selected_id_never_uses_another_same_side_device(self):
        profile = self.existing()
        before = profile.path.read_bytes()
        self.candidate("usb0", "left", 99)
        with self.assertRaisesRegex(ValueError, "发现 0 个匹配端点") as caught:
            discovery.resolve_glove_profiles((profile,))
        self.assertIn(profile.device_id, str(caught.exception))
        self.assertIn("0000000000000063", str(caught.exception))
        self.assertEqual(profile.path.read_bytes(), before)

    def test_runtime_duplicate_id_endpoints_never_pick_the_first_interface(self):
        profile = self.existing()
        before = profile.path.read_bytes()
        self.candidate("usb0", "left", 1)
        self.candidate("usb1", "left", 1)
        with self.assertRaisesRegex(ValueError, "发现 2 个匹配端点"):
            discovery.resolve_glove_profiles((profile,))
        self.assertEqual(profile.path.read_bytes(), before)

    def test_runtime_identity_change_does_not_commit_any_profile(self):
        left = self.existing(interface="old0")
        right = self.existing(2, "right", "192.168.7.4/24", 17004, "old1")
        before = {profile.path: profile.path.read_bytes() for profile in (left, right)}
        self.candidate("usb0", "left", 1)
        self.candidate("usb1", "right", 2)
        calls = 0
        def changed_identity(host, **kwargs):
            nonlocal calls
            calls += 1
            return self.metadata["usb0"] if calls == 4 else self.fetch(host, **kwargs)
        with patch.object(discovery, "fetch_metadata", side_effect=changed_identity):
            with self.assertRaisesRegex(ValueError, "身份/元数据变化"):
                discovery.resolve_glove_profiles((left, right))
        for path, content in before.items():
            self.assertEqual(path.read_bytes(), content)


if __name__ == "__main__":
    unittest.main()
