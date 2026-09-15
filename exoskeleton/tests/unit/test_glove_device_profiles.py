"""单文件设备登记、具名零位续采及失败不切换的消费侧合同。"""

from __future__ import annotations

import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

import numpy as np

from data_glove_wuji_teleop.adapters.glove.encoder_stream import EncoderFrame
from data_glove_wuji_teleop.adapters.retargeting.glove_pipeline import GloveRetargetPipeline
from data_glove_wuji_teleop.adapters.simulation import mujoco_wuji_official as official
from data_glove_wuji_teleop.application.glove_profiles import command_register_glove
from data_glove_wuji_teleop.application.urdf_zero_calibration import command_calibrate_urdf_zero
from data_glove_wuji_teleop.cli import build_parser
from data_glove_wuji_teleop.profiles.dataglove.device import GloveDeviceProfile, apply_glove_profile
from data_glove_wuji_teleop.profiles.dataglove.urdf_mapping import DataGloveUrdfMapping
from data_glove_wuji_teleop.profiles.dataglove.urdf_zero import URDF_ZERO_GROUPS, UrdfZeroProfile
from data_glove_wuji_teleop.project import PROJECT_ROOT


class StableConnection:
    channels = 21
    cs_by_joint = list(range(21))
    zeroed = False
    range_min = 0.0
    range_max = 360.0

    def __init__(self, angle):
        self.angle = angle
        self.sequence = 0

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    def read_frame(self, **_kwargs):
        self.sequence += 1
        return EncoderFrame(
            sequence=self.sequence,
            timestamp_ns=self.sequence * 10_000_000,
            dropped=0,
            angles_deg=[self.angle] * 21,
        )


class GloveDeviceConsumptionTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.devices = self.root / "devices"
        self.devices.mkdir()
        self.parser = build_parser()
        registry = patch("data_glove_wuji_teleop.application.glove_profiles.device_directory", return_value=self.devices)
        registry.start()
        self.addCleanup(registry.stop)

    def register(self, hand="right", source="192.168.7.1/24", table=10001):
        device_id = f"{table:016x}"
        name = f"{hand}-bench-{table}"
        device_path = self.devices / f"slot-{table}.json"
        mapping_data = json.loads((PROJECT_ROOT / "config/dataglove/urdf/right.json").read_text())
        # 合成左右手资源仅用于隔离合同，不作为可发布的左手标定。
        mapping_data["hand"] = hand
        if hand == "left":
            mapping_data["expected_cs_by_joint"].reverse()
        mapping_path = self.root / f"{name}-mapping.json"
        mapping_path.write_text(json.dumps(mapping_data))
        mapping = DataGloveUrdfMapping.from_dict(mapping_data)
        joints = "".join(f'<joint name="{joint}" type="revolute"/>' for joint in mapping.joint_names)
        urdf_path = self.root / f"{name}.urdf"
        urdf_path.write_text(f'<robot name="synthetic-{hand}">{joints}</robot>')
        args = self.parser.parse_args([
            "register-glove", "--name", name, "--hand", hand,
            "--device-id", device_id, "--protocol", "dgst-v3",
            "--mac", "02:11:22:33:44:55", "--interface", f"usb{table}",
            "--host", "192.168.7.2", "--port", "5580", "--http-port", "5570", "--mtu", "8000",
            "--host-address", source, "--route-table", str(table),
            "--glove-urdf", str(urdf_path), "--glove-config", str(mapping_path),
            "--output", str(device_path),
        ])
        with patch("sys.stdout"):
            command_register_glove(args)
        return GloveDeviceProfile.load(device_path)

    def calibration_args(self, profile, name="morning"):
        argv = ["calibrate-urdf-zero", "--glove-profile", str(profile.path),
                "--sample-seconds", "0.01"]
        if name is not None:
            argv += ["--name", name]
        return self.parser.parse_args(argv)

    def connection(self, device, angle=20.0):
        connection = StableConnection(angle)
        connection.cs_by_joint = device.load_mapping().expected_cs_by_joint
        return connection

    def complete_zero(self, device, angle=7.0, name="initial", activate=True):
        zero = UrdfZeroProfile.empty(device.hand, device.load_mapping().expected_cs_by_joint)
        for group in URDF_ZERO_GROUPS:
            zero, _ = zero.capture_group(group.name, [[angle] * 21], max_motion_deg=1.0)
        return device.store_zero(name, zero, activate=activate)

    def test_two_devices_keep_independent_groups_in_single_json(self):
        first = self.register()
        second = self.register("left", "192.168.7.3/24", 10002)
        originals = {device.name: device.to_dict() for device in (first, second)}

        def connect(args):
            device, angle = (first, 12.0) if args.hand == "right" else (second, 34.0)
            self.assertEqual(args.glove_interface, device.network["interface"])
            self.assertEqual(args.glove_host_address, device.network["host_address"])
            self.assertEqual(args.glove_route_table, device.network["route_table"])
            return self.connection(device, angle)

        with (
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.prepare_glove_network"),
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.connect_glove", side_effect=connect),
            patch("builtins.input", return_value=""), patch("sys.stdout"),
        ):
            command_calibrate_urdf_zero(self.calibration_args(first))
            command_calibrate_urdf_zero(self.calibration_args(second))
        for device, angle in ((first, 12.0), (second, 34.0)):
            loaded = GloveDeviceProfile.load(device.path)
            self.assertEqual(loaded.active_zero, "morning")
            self.assertEqual(loaded.load_zero().apply_angles([angle] * 21), [0.0] * 21)
            for field in ("network", "mapping", "glove_urdf", "device_id"):
                self.assertEqual(loaded.to_dict()[field], originals[device.name][field])
            loaded.validate_resources()
        self.assertEqual(set(self.devices.iterdir()), {first.path, second.path})

    def test_registration_is_non_destructive_and_runtime_requires_zero(self):
        profile = self.register()
        before = profile.path.read_bytes()
        self.assertEqual(profile.zeros, {})
        self.assertIsNone(profile.active_zero)
        with self.assertRaises(FileExistsError):
            profile.save()
        self.assertEqual(profile.path.read_bytes(), before)
        with self.assertRaises(ValueError):
            apply_glove_profile(self.calibration_args(profile))

    def test_communication_profile_does_not_require_resources_but_enforces_hand(self):
        profile = self.register()
        profile.glove_urdf.unlink()
        args = self.calibration_args(profile)
        apply_glove_profile(args, require_resources=False)
        self.assertEqual(args.hand, profile.hand)
        self.assertEqual(args.glove_device_id, profile.device_id)
        with self.assertRaises(ValueError):
            apply_glove_profile(args)
        args.hand = "left"
        with self.assertRaises(ValueError):
            apply_glove_profile(args, require_resources=False)

    def test_invalid_name_side_and_missing_resources_fail_before_network(self):
        profile = self.register()
        cases = [
            ["--name", "../escape"], ["--name", "bad/name"], ["--hand", "left"],
            ["--host", "192.168.7.2"], ["--port", "5580"],
            ["--output", str(self.root / "override.json")],
        ]
        with patch("data_glove_wuji_teleop.application.urdf_zero_calibration.prepare_glove_network") as network:
            for extra in cases:
                with self.subTest(extra=extra):
                    args = self.parser.parse_args([
                        "calibrate-urdf-zero", "--glove-profile", str(profile.path), *extra,
                    ])
                    with self.assertRaises(ValueError):
                        command_calibrate_urdf_zero(args)
            profile.glove_urdf.unlink()
            with self.assertRaisesRegex(ValueError, "glove_urdf"):
                command_calibrate_urdf_zero(self.calibration_args(profile))
            network.assert_not_called()

    def test_wrong_side_mapping_or_zero_is_rejected(self):
        profile = self.complete_zero(self.register())
        for field in ("mapping", "zeros"):
            with self.subTest(field=field):
                data = profile.to_dict()
                if field == "mapping":
                    data["mapping"]["hand"] = "left"
                else:
                    data["zeros"]["initial"]["hand"] = "left"
                with self.assertRaises(ValueError):
                    GloveDeviceProfile.from_dict(data, path=profile.path).validate_resources()

    def test_active_zero_with_different_wiring_is_rejected_before_network(self):
        profile = self.complete_zero(self.register())
        data = profile.to_dict()
        data["zeros"]["initial"]["expected_cs_by_joint"].reverse()
        profile.path.write_text(json.dumps(data))
        with patch("data_glove_wuji_teleop.application.urdf_zero_calibration.prepare_glove_network") as network:
            with self.assertRaises(ValueError):
                command_calibrate_urdf_zero(self.calibration_args(profile, name=None))
            network.assert_not_called()

    def test_interruption_keeps_progress_previous_active_and_default(self):
        for index, interruption in enumerate((KeyboardInterrupt(), RuntimeError("采样连接失败"))):
            with self.subTest(interruption=type(interruption).__name__):
                profile = self.complete_zero(self.register(
                    table=10001 + index, source=f"192.168.7.{1 + index * 2}/24",
                ))
                original = profile.to_dict()
                args = self.parser.parse_args([
                    "calibrate-urdf-zero", "--hand", "right", "--name", "morning",
                    "--sample-seconds", "0.01",
                ])
                with (
                    patch("data_glove_wuji_teleop.application.urdf_zero_calibration.discover_glove_for_calibration", return_value=profile),
                    patch("data_glove_wuji_teleop.application.urdf_zero_calibration.prepare_glove_network"),
                    patch("data_glove_wuji_teleop.application.urdf_zero_calibration.connect_glove",
                          side_effect=[self.connection(profile, 40.0), interruption]),
                    patch("data_glove_wuji_teleop.application.urdf_zero_calibration.bind_default_glove") as bind,
                    patch("builtins.input", return_value=""), patch("sys.stdout"),
                ):
                    with self.assertRaises(type(interruption)):
                        command_calibrate_urdf_zero(args)
                    bind.assert_not_called()
                saved = GloveDeviceProfile.load(profile.path)
                self.assertEqual(saved.active_zero, "initial")
                self.assertEqual(saved.zeros["initial"], original["zeros"]["initial"])
                self.assertEqual(saved.load_zero("morning").captured_groups, ("thumb",))
                for field in ("network", "mapping", "glove_urdf", "device_id"):
                    self.assertEqual(saved.to_dict()[field], original[field])

    def test_failed_atomic_write_keeps_previous_saved_progress(self):
        profile = self.complete_zero(self.register())
        partial = UrdfZeroProfile.empty(profile.hand, profile.load_mapping().expected_cs_by_joint)
        partial, _ = partial.capture_group("thumb", [[8.0] * 21], max_motion_deg=1.0)
        profile = profile.store_zero("morning", partial)
        original = profile.path.read_bytes()
        with (
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.prepare_glove_network"),
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.connect_glove", return_value=self.connection(profile)),
            patch("data_glove_wuji_teleop.profiles.dataglove.device.os.replace", side_effect=OSError("磁盘写入失败")),
            patch("builtins.input", return_value=""), patch("sys.stdout"),
        ):
            with self.assertRaisesRegex(OSError, "磁盘写入失败"):
                command_calibrate_urdf_zero(self.calibration_args(profile))
        self.assertEqual(profile.path.read_bytes(), original)
        saved = GloveDeviceProfile.load(profile.path)
        self.assertEqual(saved.active_zero, "initial")
        self.assertEqual(saved.load_zero("morning").captured_groups, ("thumb",))
        self.assertEqual(set(self.devices.iterdir()), {profile.path})

    def test_invalid_existing_group_is_never_replaced(self):
        profile = self.register()
        data = profile.to_dict()
        data["zeros"]["morning"] = {"hand": "已有设备的资料"}
        profile.path.write_text(json.dumps(data))
        original = profile.path.read_bytes()
        with patch("data_glove_wuji_teleop.application.urdf_zero_calibration.prepare_glove_network") as network:
            with self.assertRaises(ValueError):
                command_calibrate_urdf_zero(self.calibration_args(profile))
            network.assert_not_called()
        self.assertEqual(profile.path.read_bytes(), original)

    def test_named_resume_preserves_finished_groups_and_activates_only_at_completion(self):
        profile = self.complete_zero(self.register())
        partial = UrdfZeroProfile.empty(profile.hand, profile.load_mapping().expected_cs_by_joint)
        partial, _ = partial.capture_group("thumb", [[8.0] * 21], max_motion_deg=1.0)
        profile = profile.store_zero("morning", partial)
        states = []

        def connect(_args):
            current = GloveDeviceProfile.load(profile.path)
            states.append(current.load_zero("morning").captured_groups)
            self.assertEqual(current.active_zero, "initial")
            self.assertEqual(current.load_zero().apply_angles([7.0] * 21), [0.0] * 21)
            return self.connection(profile)

        with (
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.prepare_glove_network"),
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.connect_glove", side_effect=connect),
            patch("builtins.input", return_value=""), patch("sys.stdout"),
        ):
            command_calibrate_urdf_zero(self.calibration_args(profile))
        active = GloveDeviceProfile.load(profile.path)
        names = tuple(group.name for group in URDF_ZERO_GROUPS)
        self.assertEqual(states, [names[:count] for count in range(1, 6)])
        self.assertEqual(active.active_zero, "morning")
        self.assertEqual(active.load_zero().captured_groups, names)
        angles = [20.0] * 21
        for index in URDF_ZERO_GROUPS[0].channel_indices:
            angles[index] = 8.0
        self.assertEqual(active.load_zero().apply_angles(angles), [0.0] * 21)
        self.assertEqual(active.zeros["initial"], profile.zeros["initial"])
        self.assertEqual(set(self.devices.iterdir()), {profile.path})

    def test_complete_group_activation_failure_can_retry_without_sampling(self):
        profile = self.complete_zero(self.register())
        profile = self.complete_zero(profile, angle=23.0, name="morning", activate=False)
        original = profile.path.read_bytes()
        with (
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.prepare_glove_network") as network,
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.connect_glove") as connect,
            patch("data_glove_wuji_teleop.profiles.dataglove.device.os.replace", side_effect=OSError("档案写入失败")),
        ):
            with self.assertRaisesRegex(OSError, "档案写入失败"):
                command_calibrate_urdf_zero(self.calibration_args(profile))
            network.assert_not_called()
            connect.assert_not_called()
        self.assertEqual(profile.path.read_bytes(), original)
        with (
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.prepare_glove_network") as network,
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.connect_glove") as connect,
            patch("sys.stdout"),
        ):
            command_calibrate_urdf_zero(self.calibration_args(profile))
            network.assert_not_called()
            connect.assert_not_called()
        active = GloveDeviceProfile.load(profile.path)
        self.assertEqual(active.active_zero, "morning")
        self.assertEqual(active.zeros, profile.zeros)

    def test_hand_only_uses_discovered_device_and_binds_default_after_completion(self):
        profile = self.register()
        args = self.parser.parse_args(["calibrate-urdf-zero", "--hand", "right", "--sample-seconds", "0.01"])
        bound = []

        def bind(device):
            saved = GloveDeviceProfile.load(device.path)
            self.assertTrue(saved.load_zero().complete)
            self.assertEqual(saved.device_id, profile.device_id)
            bound.append(saved.name)

        with (
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.discover_glove_for_calibration", return_value=profile),
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.prepare_glove_network"),
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.connect_glove", return_value=self.connection(profile)),
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.bind_default_glove", side_effect=bind),
            patch("builtins.input", return_value=""), patch("sys.stdout"),
        ):
            command_calibrate_urdf_zero(args)
        self.assertEqual(bound, [profile.name])
        completed = GloveDeviceProfile.load(profile.path)
        self.assertEqual(completed.active_zero, "initial")
        original = completed.path.read_bytes()
        with (
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.discover_glove_for_calibration", return_value=completed),
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.prepare_glove_network") as network,
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.connect_glove") as connect,
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.bind_default_glove", side_effect=bind),
            patch("sys.stdout"),
        ):
            command_calibrate_urdf_zero(self.parser.parse_args(["calibrate-urdf-zero", "--hand", "right"]))
            network.assert_not_called()
            connect.assert_not_called()
        self.assertEqual(completed.path.read_bytes(), original)
        self.assertEqual(bound, [profile.name, profile.name])

    def test_wrong_stream_mapping_fails_before_saving_progress(self):
        profile = self.complete_zero(self.register())
        original = profile.path.read_bytes()
        with (
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.prepare_glove_network"),
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.connect_glove", return_value=StableConnection(40.0)),
            patch("builtins.input", return_value=""), patch("sys.stdout"),
        ):
            with self.assertRaisesRegex(ValueError, "enc_map"):
                command_calibrate_urdf_zero(self.calibration_args(profile))
        self.assertEqual(profile.path.read_bytes(), original)

    def test_resume_with_wrong_mapping_fails_before_network(self):
        profile = self.register()
        partial = UrdfZeroProfile.empty(profile.hand, range(21))
        partial, _ = partial.capture_group("thumb", [[8.0] * 21], max_motion_deg=1.0)
        data = profile.to_dict()
        data["zeros"]["morning"] = partial.to_dict()
        profile.path.write_text(json.dumps(data))
        original = profile.path.read_bytes()
        with patch("data_glove_wuji_teleop.application.urdf_zero_calibration.prepare_glove_network") as network:
            with self.assertRaises(ValueError):
                command_calibrate_urdf_zero(self.calibration_args(profile))
            network.assert_not_called()
        self.assertEqual(profile.path.read_bytes(), original)

    def test_device_profile_cannot_override_shared_retargeting(self):
        profile = self.register()
        data = profile.to_dict()
        data["wuji_config"] = "pico-retarget.yaml"
        with self.assertRaises(ValueError):
            GloveDeviceProfile.from_dict(data, path=profile.path)

    def test_pico_and_tracker_calibrate_to_the_same_urdf_and_mano(self):
        pico = self.register()
        tracker = self.register(source="192.168.7.3/24", table=10002)
        trajectories = []
        for device, zero_angle in ((pico, 7.0), (tracker, 34.0)):
            device = replace(device, glove_urdf=official.DEFAULT_URDF)
            device.save(overwrite=True)
            device = self.complete_zero(device, angle=zero_angle)
            args = official.build_parser().parse_args(["--glove-profile", str(device.path)])
            selected = apply_glove_profile(args)
            pipeline = GloveRetargetPipeline.from_config(
                glove_urdf=args.glove_urdf, mapping_data=selected.mapping_data,
                zero_profile=selected.load_zero(), morphology_file=args.mano_morphology,
                commissioning=True,
            )
            trajectory = []
            for index, delta in enumerate((0.0, 2.0, 5.0, 2.0, 0.0)):
                points = pipeline.process_frame(
                    EncoderFrame(index, index * 10_000_000, 0, [zero_angle + delta] * 21),
                    zero_profile=pipeline.zero_profile,
                )
                trajectory.append((pipeline.data.qpos.copy(), points.copy()))
            trajectories.append(trajectory)
        for pico_frame, tracker_frame in zip(*trajectories):
            np.testing.assert_allclose(pico_frame[0], tracker_frame[0], atol=1e-12)
            np.testing.assert_allclose(pico_frame[1], tracker_frame[1], atol=1e-12)

    def test_mapping_and_urdf_must_describe_same_joints(self):
        profile = self.register()
        profile.glove_urdf.write_text('<robot name="wrong-resource"/>')
        with patch("data_glove_wuji_teleop.application.urdf_zero_calibration.prepare_glove_network") as network:
            with self.assertRaisesRegex(ValueError, "URDF"):
                command_calibrate_urdf_zero(self.calibration_args(profile))
            network.assert_not_called()

    def test_standalone_resume_preserves_finished_channels_without_network_setup(self):
        output = self.root / "standalone.json"
        zero = UrdfZeroProfile.empty("right", range(21))
        zero, _ = zero.capture_group("thumb", [[8.0] * 21], max_motion_deg=1.0)
        zero.save(output)
        args = self.parser.parse_args([
            "calibrate-urdf-zero", "--hand", "right", "--output", str(output),
            "--sample-seconds", "0.01",
        ])
        with (
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.prepare_glove_network") as network,
            patch("data_glove_wuji_teleop.application.encoder_capture.EncoderConnection.connect",
                  side_effect=lambda *a, **kw: StableConnection(20.0)),
            patch("builtins.input", return_value=""), patch("sys.stdout"),
        ):
            command_calibrate_urdf_zero(args)
            network.assert_not_called()
        saved = UrdfZeroProfile.load(output)
        angles = [20.0] * 21
        for index in URDF_ZERO_GROUPS[0].channel_indices:
            angles[index] = 8.0
        self.assertTrue(saved.complete)
        self.assertEqual(saved.apply_angles(angles), [0.0] * 21)

    def test_complete_standalone_zero_is_untouched_without_connecting(self):
        output = self.root / "complete.json"
        zero = UrdfZeroProfile.empty("right", range(21))
        for group in URDF_ZERO_GROUPS:
            zero, _ = zero.capture_group(group.name, [[8.0] * 21], max_motion_deg=1.0)
        zero.save(output)
        original = output.read_bytes()
        args = self.parser.parse_args([
            "calibrate-urdf-zero", "--hand", "right", "--output", str(output),
        ])
        with (
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.prepare_glove_network") as network,
            patch("data_glove_wuji_teleop.application.encoder_capture.EncoderConnection.connect") as connect,
            patch("sys.stdout"),
        ):
            command_calibrate_urdf_zero(args)
            network.assert_not_called()
            connect.assert_not_called()
        self.assertEqual(output.read_bytes(), original)


if __name__ == "__main__":
    unittest.main()
