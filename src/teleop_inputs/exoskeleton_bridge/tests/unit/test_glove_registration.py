"""实物身份绑定、注册期协议选择和 no-clobber 保存事务。"""

from __future__ import annotations

import json
from dataclasses import replace
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from data_glove_wuji_teleop.application.glove_profiles import _discover_interface, command_register_glove
from data_glove_wuji_teleop.cli import build_parser
from data_glove_wuji_teleop.profiles.dataglove.device import GloveDeviceProfile
from data_glove_wuji_teleop.profiles.dataglove.urdf_zero import URDF_ZERO_GROUPS, UrdfZeroProfile
from data_glove_wuji_teleop.project import PROJECT_ROOT


class RegistrationStream:
    channels = 21

    def __init__(self, hand="left", device_id="0x0000000000000001", *, fail_read=False, fail_stop=False):
        self.cs_by_joint = list(reversed(range(21)))
        self.metadata = SimpleNamespace(
            device={"hand": hand, "device_id": device_id}, sha256="synthetic-metadata-sha",
            sensors={"encoder": {"channels": 21, "cs_by_joint": self.cs_by_joint}},
        )
        self.fail_read = fail_read
        self.fail_stop = fail_stop
        self.stopped = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, *_args):
        self.stopped = True
        if self.fail_stop and exc_type is None:
            raise RuntimeError("STOP failed")

    def read_frame(self):
        if self.fail_read:
            raise RuntimeError("first frame failed")
        return SimpleNamespace(angles_deg=[5.0] * 21)


class GloveRegistrationTest(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.parser = build_parser()
        for target, kwargs in (
            ("device_directory", {"return_value": self.root}),
            ("_registration_mac", {"return_value": "02:11:22:33:44:55"}),
            ("prepare_glove_network", {}),
            ("fetch_metadata", {"return_value": RegistrationStream().metadata}),
        ):
            active = patch(f"data_glove_wuji_teleop.application.glove_profiles.{target}", **kwargs)
            active.start()
            self.addCleanup(active.stop)
        quiet = patch("sys.stdout")
        quiet.start()
        self.addCleanup(quiet.stop)

    def args(self, hand="left", index=0, extra=()):
        return self.parser.parse_args([
            "register-glove", "--protocol", "dgst-v3",
            "--interface", f"glove{index}", "--host-address", f"192.168.7.{10 + index}/24",
            "--route-table", str(17010 + index),
            *(["--hand", hand] if hand is not None else []), *extra,
        ])

    def discover(self, args, stream):
        with (
            patch("data_glove_wuji_teleop.application.glove_profiles.fetch_metadata", return_value=stream.metadata),
            patch("data_glove_wuji_teleop.application.glove_profiles.EncoderConnection.connect", return_value=stream),
        ):
            command_register_glove(args)

    def complete_profile(self):
        command_register_glove(self.args(extra=("--device-id", "0x0000000000000001",)))
        profile = GloveDeviceProfile.load(self.root / "left-0000000000000001.json")
        profile = replace(profile, mapping_data={
            **profile.mapping_data, "ready": True, "directions_verified": True,
        })
        profile.save(overwrite=True)
        zero = UrdfZeroProfile.empty(profile.hand, profile.mapping_data["expected_cs_by_joint"])
        for group in URDF_ZERO_GROUPS:
            zero, _ = zero.capture_group(group.name, [[12.5] * 21], max_motion_deg=0.0)
        profile = profile.store_zero("initial", zero, activate=True)
        partial = UrdfZeroProfile.empty(profile.hand, zero.expected_cs_by_joint)
        partial, _ = partial.capture_group(URDF_ZERO_GROUPS[0].name, [[23.0] * 21], max_motion_deg=0.0)
        return profile.store_zero("second", partial)

    def test_wrong_identity_or_side_never_starts_stream_or_saves(self):
        for stream in (RegistrationStream(device_id="0x0000000000000002"), RegistrationStream(hand="right")):
            with self.subTest(device=stream.metadata.device):
                with (
                    patch("data_glove_wuji_teleop.application.glove_profiles.fetch_metadata", return_value=stream.metadata),
                    patch("data_glove_wuji_teleop.application.glove_profiles.EncoderConnection.connect") as connect,
                ):
                    with self.assertRaisesRegex(ValueError, "hand/device_id"):
                        command_register_glove(self.args(extra=("--discover", "--device-id", "0x0000000000000001")))
                    connect.assert_not_called()
                self.assertEqual(list(self.root.rglob("*.json")), [])

    def test_first_frame_and_stop_failures_never_register_device(self):
        for stream in (RegistrationStream(fail_read=True), RegistrationStream(fail_stop=True)):
            with self.subTest(read=stream.fail_read):
                with self.assertRaises(RuntimeError):
                    self.discover(self.args(extra=("--discover",)), stream)
                self.assertEqual(list(self.root.rglob("*.json")), [])
                self.assertTrue(stream.stopped)

    def test_four_devices_have_independent_calibration_and_discovered_hands(self):
        devices = []
        for index, hand in enumerate(("left", "right", "left", "right")):
            identity = f"{index + 1:016x}"
            stream = RegistrationStream(hand=hand, device_id=f"0x{identity}")
            self.discover(self.args(None, index, ("--discover", "--protocol", "encoder-v1")), stream)
            profile = GloveDeviceProfile.load(self.root / f"{hand}-{identity}.json")
            mapping = profile.load_mapping()
            mapping.validate_stream(channels=21, cs_by_joint=stream.cs_by_joint)
            with self.assertRaises(ValueError):
                mapping.require_verified_directions()
            self.assertFalse(profile.mapping_data["ready"])
            self.assertEqual(profile.zeros, {})
            self.assertIsNone(profile.active_zero)
            with self.assertRaises(ValueError):
                profile.validate_resources()
            devices.append(profile)
        self.assertEqual(set(self.root.rglob("*.json")), {device.path for device in devices})
        self.assertFalse((self.root / "mapping").exists())
        self.assertFalse((self.root / "zero").exists())
        siblings = {device.path: device.path.read_bytes() for device in devices[1:]}
        first = devices[0]
        data = {**first.mapping_data, "directions_verified": True}
        first = replace(first, mapping_data=data)
        first.save(overwrite=True)
        partial = UrdfZeroProfile.empty(first.hand, data["expected_cs_by_joint"])
        first = first.store_zero("initial", partial)
        with self.assertRaises(ValueError):
            first.validate_resources()
        for path, before in siblings.items():
            self.assertEqual(path.read_bytes(), before)

    def test_explicit_mapping_is_never_silently_rewired(self):
        template = self.root / "explicit-mapping.json"
        original = (PROJECT_ROOT / "config/dataglove/urdf/left.json").read_bytes()
        template.write_bytes(original)
        with self.assertRaisesRegex(ValueError, "enc_map"):
            self.discover(self.args(extra=("--discover", "--glove-config", str(template))), RegistrationStream())
        self.assertEqual(template.read_bytes(), original)
        self.assertFalse((self.root / "left-0000000000000001.json").exists())

    def test_explicit_mapping_is_embedded_without_modification(self):
        template = self.root / "explicit-mapping.json"
        data = json.loads((PROJECT_ROOT / "config/dataglove/urdf/left.json").read_text())
        data["expected_cs_by_joint"] = RegistrationStream().cs_by_joint
        data["ready"] = True
        data["directions_verified"] = True
        template.write_text(json.dumps(data))
        before = template.read_bytes()
        self.discover(self.args(extra=("--discover", "--glove-config", str(template))), RegistrationStream())
        path = self.root / "left-0000000000000001.json"
        profile = GloveDeviceProfile.load(path)
        self.assertEqual(profile.mapping_data, data)
        self.assertEqual(template.read_bytes(), before)
        self.assertEqual(profile.zeros, {})
        self.assertIsNone(profile.active_zero)
        self.assertEqual(set(self.root.rglob("*.json")), {template, path})

    def test_profile_commit_failure_leaves_no_partial_registration(self):
        command_register_glove(self.args(extra=("--device-id", "0x0000000000000002",)))
        existing = GloveDeviceProfile.load(self.root / "left-0000000000000002.json")
        before = existing.path.read_bytes()
        with patch.object(GloveDeviceProfile, "save", side_effect=OSError("disk full")):
            with self.assertRaisesRegex(OSError, "disk full"):
                self.discover(self.args(index=1, extra=("--discover",)), RegistrationStream())
        self.assertEqual(existing.path.read_bytes(), before)
        self.assertEqual(set(self.root.rglob("*.json")), {existing.path})
        self.assertFalse((self.root / "mapping").exists())
        self.assertFalse((self.root / "zero").exists())

    def test_complete_profile_survives_failed_overwrite(self):
        profile = self.complete_profile()
        before = profile.path.read_bytes()
        with self.assertRaises(FileExistsError):
            command_register_glove(self.args(extra=("--device-id", profile.device_id)))
        with self.assertRaises(ValueError):
            command_register_glove(self.args(extra=(
                "--device-id", "0x0000000000000002", "--overwrite", "--output", str(profile.path),
            )))
        with patch("data_glove_wuji_teleop.profiles.dataglove.device.os.replace", side_effect=OSError("disk full")):
            with self.assertRaisesRegex(OSError, "disk full"):
                command_register_glove(self.args(extra=("--device-id", profile.device_id, "--overwrite")))
        self.assertEqual(profile.path.read_bytes(), before)
        self.assertEqual(set(self.root.rglob("*.json")), {profile.path})

    def test_overwrite_preserves_all_embedded_calibration(self):
        profile = self.complete_profile()
        before = profile.to_dict()
        command_register_glove(self.args(extra=(
            "--device-id", profile.device_id, "--overwrite", "--http-port", "5571",
            "--name", "bench-left", "--output", str(profile.path),
        )))
        updated = GloveDeviceProfile.load(profile.path)
        self.assertEqual(updated.mapping_data, before["mapping"])
        self.assertEqual(updated.zeros, before["zeros"])
        self.assertEqual(updated.active_zero, before["active_zero"])
        self.assertEqual(updated.network["http_port"], 5571)
        self.assertEqual(updated.name, "bench-left")
        self.assertEqual(set(self.root.rglob("*.json")), {profile.path})

    def test_discovered_overwrite_stop_failure_preserves_complete_profile(self):
        profile = self.complete_profile()
        before = profile.path.read_bytes()
        stream = RegistrationStream(fail_stop=True)
        stream.cs_by_joint[:] = profile.mapping_data["expected_cs_by_joint"]
        with self.assertRaisesRegex(RuntimeError, "STOP failed"):
            self.discover(self.args(extra=(
                "--discover", "--device-id", profile.device_id, "--overwrite",
            )), stream)
        self.assertEqual(profile.path.read_bytes(), before)
        self.assertTrue(stream.stopped)

    def test_overwrite_cannot_replace_existing_mapping_with_template(self):
        profile = self.complete_profile()
        before = profile.path.read_bytes()
        template = PROJECT_ROOT / "config/dataglove/urdf/left.json"
        with self.assertRaises(ValueError):
            command_register_glove(self.args(extra=(
                "--device-id", profile.device_id, "--overwrite", "--glove-config-template", str(template),
            )))
        self.assertEqual(profile.path.read_bytes(), before)

    def test_shared_source_is_rejected_before_network_preparation(self):
        command_register_glove(self.args(extra=("--device-id", "0x0000000000000002",)))
        with patch("data_glove_wuji_teleop.application.glove_profiles.prepare_glove_network") as network:
            with self.assertRaises(ValueError):
                command_register_glove(self.args(index=1, extra=("--discover", "--host-address", "192.168.7.10/24")))
            network.assert_not_called()

    def test_v4_rejects_short_identity_and_legacy_file_schema(self):
        command_register_glove(self.args(extra=("--device-id", "0x000000ABCDEF1234",)))
        path = self.root / "left-000000abcdef1234.json"
        profile = GloveDeviceProfile.load(path)
        for changed in (
            {"device_id": "0x1234"}, {"glove_config": "mapping/another-device.json"},
            {"zero_file": "zero/another-device/initial.json"}, {"version": 2}, {"version": 3},
        ):
            with self.subTest(changed=changed):
                with self.assertRaises(ValueError):
                    GloveDeviceProfile.from_dict({**profile.to_dict(), **changed}, path=path)
        legacy = profile.to_dict()
        for key in ("mapping", "zeros", "active_zero"):
            legacy.pop(key)
        legacy.update(version=3, glove_config="mapping/left-000000abcdef1234.json",
                      zero_file="zero/left-000000abcdef1234/initial.json")
        with self.assertRaises(ValueError):
            GloveDeviceProfile.from_dict(legacy, path=path)
        custom_path = self.root / "left.json"
        profile.save(custom_path)
        self.assertEqual(GloveDeviceProfile.load(custom_path).device_id, profile.device_id)

    def test_custom_name_and_independent_filename_keep_hardware_identity(self):
        output = self.root / "operator-device.json"
        self.discover(self.args(extra=(
            "--discover", "--name", "bench-left", "--output", str(output),
        )), RegistrationStream())
        profile = GloveDeviceProfile.load(output)
        self.assertEqual(profile.name, "bench-left")
        self.assertEqual(profile.device_id, "0x0000000000000001")
        self.assertEqual(profile.hand, "left")
        self.assertEqual(set(self.root.rglob("*.json")), {output})
        before = output.read_bytes()
        with self.assertRaises(ValueError):
            self.discover(self.args(extra=(
                "--discover", "--name", "bench-left", "--output", str(output), "--overwrite",
            )), RegistrationStream(device_id="0x0000000000000002"))
        self.assertEqual(output.read_bytes(), before)

    def test_auto_prefers_encoder_and_selects_dgst_only_for_refused_endpoint(self):
        for rejected, expected in ((False, "encoder-v1"), (True, "dgst-v3")):
            with self.subTest(rejected=rejected):
                stream = RegistrationStream(device_id=f"0x{3 + int(rejected):016x}")
                probe = SimpleNamespace(close=lambda: None)
                effects = [ConnectionRefusedError("not listening"), probe] if rejected else [probe]
                with patch("data_glove_wuji_teleop.application.glove_profiles.socket.create_connection", side_effect=effects):
                    self.discover(self.args(index=3 + int(rejected), extra=("--discover", "--protocol", "auto")), stream)
                profile = GloveDeviceProfile.load(self.root / f"left-{3 + int(rejected):016x}.json")
                self.assertEqual(profile.network["protocol"], expected)

    def test_auto_never_retries_data_stop_or_timeout_errors(self):
        for stream in (RegistrationStream(fail_read=True), RegistrationStream(fail_stop=True)):
            with self.subTest(read=stream.fail_read):
                with patch("data_glove_wuji_teleop.application.glove_profiles.socket.create_connection",
                           return_value=SimpleNamespace(close=lambda: None)) as probe:
                    with self.assertRaises(RuntimeError):
                        self.discover(self.args(extra=("--discover", "--protocol", "auto")), stream)
                    self.assertEqual(probe.call_count, 1)
                self.assertEqual(list(self.root.rglob("*.json")), [])
        with patch("data_glove_wuji_teleop.application.glove_profiles.socket.create_connection",
                   side_effect=TimeoutError("unreachable")) as probe:
            with self.assertRaises(TimeoutError):
                self.discover(self.args(extra=("--discover", "--protocol", "auto")), RegistrationStream())
            self.assertEqual(probe.call_count, 1)

    def test_ambiguous_same_mac_interfaces_are_not_guessed(self):
        files = []
        for name in ("usb0", "usb1"):
            address = self.root / name / "address"
            address.parent.mkdir()
            address.write_text("02:11:22:33:44:55\n")
            files.append(address)
        with patch("data_glove_wuji_teleop.application.glove_profiles.Path.glob", return_value=files):
            with self.assertRaisesRegex(ValueError, "无法唯一识别"):
                _discover_interface("02:11:22:33:44:55")
        with patch("data_glove_wuji_teleop.application.glove_profiles.Path.glob", return_value=files[:1]):
            self.assertEqual(_discover_interface("02:11:22:33:44:55"), "usb0")


if __name__ == "__main__":
    unittest.main()
