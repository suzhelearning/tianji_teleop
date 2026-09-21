"""具名任务的设备隔离、错误绑定和标定入口合同。"""

from __future__ import annotations

import argparse
import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

from data_glove_wuji_teleop.cli import build_parser
from data_glove_wuji_teleop.application.teleop_tasks import command_bind_task
from data_glove_wuji_teleop.application.urdf_zero_calibration import command_calibrate_urdf_zero
from data_glove_wuji_teleop.official_teleop import build_parser as teleop_parser, select_gloves
from data_glove_wuji_teleop.profiles.dataglove.device import GloveDeviceProfile
from data_glove_wuji_teleop.profiles.teleop_task import TeleopTask, apply_task_glove
from tests.integration.test_official_dual_launcher import _synthetic_profile


class TeleopTaskTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.paths = {hand: _synthetic_profile(self.root, hand) for hand in ("left", "right")}

    def task(self, **changes):
        data = {"version": 1, "name": "bench", "gloves": {hand: str(path) for hand, path in self.paths.items()}}
        data.update(changes)
        return TeleopTask.from_dict(data, path=self.root / "bench.json")

    def test_wrong_side_is_rejected_before_binding_is_written(self):
        task = self.task(gloves={"left": str(self.paths["right"])})
        with self.assertRaises(ValueError):
            task.save()
        self.assertFalse(task.path.exists())

    def test_duplicate_identity_and_source_routes_cannot_form_a_pair(self):
        original = GloveDeviceProfile.load(self.paths["right"])
        left = GloveDeviceProfile.load(self.paths["left"])
        for kind in ("device", "source", "table"):
            with self.subTest(kind=kind):
                network = dict(original.network)
                if kind == "source":
                    network["host_address"] = left.network["host_address"]
                elif kind == "table":
                    network["route_table"] = left.network["route_table"]
                changed = replace(original, network=network,
                                  device_id=left.device_id if kind == "device" else original.device_id)
                changed.save(overwrite=True)
                with self.assertRaises(ValueError):
                    self.task().save()
                self.assertFalse((self.root / "bench.json").exists())
        original.save(overwrite=True)

    def test_existing_task_is_not_replaced_without_explicit_permission(self):
        task = self.task()
        task.save()
        before = task.path.read_bytes()
        changed = self.task(gloves={"left": str(self.paths["left"])})
        with self.assertRaises(FileExistsError):
            changed.save()
        self.assertEqual(task.path.read_bytes(), before)
        changed.save(overwrite=True)
        with self.assertRaises(ValueError):
            TeleopTask.load(task.path).profiles("right")

    def test_missing_side_is_not_silently_dropped_from_both_request(self):
        task = self.task(gloves={"left": str(self.paths["left"])})
        task.save()
        args = teleop_parser().parse_args(["--task", str(task.path), "--hand", "both", "--commission-directions"])
        with self.assertRaises(ValueError):
            select_gloves(args)

    def test_dual_calibration_requires_a_side_and_rejects_profile_override(self):
        task = self.task()
        task.save()
        args = build_parser().parse_args(["calibrate-urdf-zero", "--task", str(task.path)])
        with self.assertRaises(ValueError):
            apply_task_glove(args)
        args = build_parser().parse_args(["calibrate-urdf-zero", "--task", str(task.path),
                                         "--hand", "left", "--glove-profile", str(self.paths["right"])])
        with self.assertRaises(ValueError):
            apply_task_glove(args)
        args = teleop_parser().parse_args(["--task", str(task.path), "--left-profile", str(self.paths["left"]),
                                          "--commission-directions"])
        with self.assertRaises(ValueError):
            select_gloves(args)

    def test_calibration_task_does_not_require_an_existing_zero(self):
        profile = GloveDeviceProfile.load(self.paths["left"])
        replace(profile, zeros={}, active_zero=None).save(overwrite=True)
        task = self.task(gloves={"left": str(self.paths["left"])})
        task.save()
        args = build_parser().parse_args(["calibrate-urdf-zero", "--task", str(task.path)])
        apply_task_glove(args)
        GloveDeviceProfile.load(args.glove_profile).validate_resources(require_zero=False)
        runtime = teleop_parser().parse_args(["--task", str(task.path), "--commission-directions"])
        with self.assertRaises(ValueError):
            select_gloves(runtime)

    def test_default_binding_cannot_fall_back_when_its_zero_is_missing(self):
        task = replace(self.task(gloves={"left": str(self.paths["left"])}), path=self.root / "default.json")
        task.save()
        profile = GloveDeviceProfile.load(self.paths["left"])
        replace(profile, zeros={}, active_zero=None).save(overwrite=True)
        args = teleop_parser().parse_args(["--commission-directions"])
        with patch("data_glove_wuji_teleop.profiles.teleop_task.task_directory", return_value=self.root):
            with self.assertRaises(ValueError):
                select_gloves(args)

    def test_inferred_side_rejects_two_profiles_for_the_same_hand(self):
        output = self.root / "binding.json"
        args = build_parser().parse_args([
            "bind-task", "--glove-profile", str(self.paths["left"]),
            "--glove-profile", str(self.paths["left"]), "--output", str(output),
        ])
        with self.assertRaises(ValueError):
            command_bind_task(args)
        self.assertFalse(output.exists())

    def test_robot_identity_bindings_cannot_cross_unbound_sides(self):
        for serials in ({"right": "SN-right"}, {"left": ""}):
            with self.subTest(serials=serials), self.assertRaises(ValueError):
                self.task(gloves={"left": str(self.paths["left"])}, robot_serials=serials)
        with self.assertRaises(ValueError):
            self.task(robot_serials={"left": "same", "right": "same"})

    def test_side_only_calibration_binds_completed_device_without_losing_other_side(self):
        task = replace(
            self.task(gloves={"right": str(self.paths["right"])},
                      robot_serials={"right": "SN-right"}, robot_network={"interface": "robot0"}),
            path=self.root / "default.json", name="default",
        )
        task.save()
        profile = GloveDeviceProfile.load(self.paths["left"])
        previous = GloveDeviceProfile.load(self.paths["right"])
        profile = replace(profile, network={**profile.network, "interface": previous.network["interface"]})
        profile.save(overwrite=True)
        original_zero = profile.zeros
        args = build_parser().parse_args(["calibrate-urdf-zero", "--hand", "left"])
        with (
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.discover_glove_for_calibration",
                  return_value=profile),
            patch("data_glove_wuji_teleop.profiles.teleop_task.task_directory", return_value=self.root),
            patch("sys.stdout"),
        ):
            self.assertEqual(command_calibrate_urdf_zero(args), 0)
        saved = TeleopTask.load(task.path)
        self.assertEqual(saved.gloves, self.paths)
        self.assertEqual(saved.robot_serials, {"right": "SN-right"})
        self.assertEqual(saved.robot_network, {"interface": "robot0"})
        self.assertEqual(GloveDeviceProfile.load(profile.path).zeros, original_zero)

    def test_interrupted_side_only_calibration_does_not_change_default_binding(self):
        task = replace(self.task(gloves={"right": str(self.paths["right"])}),
                       path=self.root / "default.json", name="default")
        task.save()
        original_task = task.path.read_bytes()
        profile = GloveDeviceProfile.load(self.paths["left"])
        profile = replace(profile, zeros={}, active_zero=None)
        profile.save(overwrite=True)
        args = build_parser().parse_args(["calibrate-urdf-zero", "--hand", "left"])
        with (
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.discover_glove_for_calibration",
                  return_value=profile),
            patch("data_glove_wuji_teleop.application.urdf_zero_calibration.prepare_glove_network"),
            patch("data_glove_wuji_teleop.profiles.teleop_task.task_directory", return_value=self.root),
            patch("builtins.input", side_effect=EOFError),
            patch("sys.stdout"),
        ):
            with self.assertRaises(EOFError):
                command_calibrate_urdf_zero(args)
        self.assertEqual(task.path.read_bytes(), original_task)
        self.assertIsNone(GloveDeviceProfile.load(profile.path).load_zero())


if __name__ == "__main__":
    unittest.main()
