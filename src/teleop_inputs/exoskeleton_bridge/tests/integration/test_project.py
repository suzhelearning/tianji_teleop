"""独立项目配置、第三方模型与 MuJoCo 资源合同测试。"""

from __future__ import annotations

import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from dataclasses import replace
from unittest.mock import patch

from data_glove_wuji_teleop.application.urdf_zero_calibration import command_calibrate_urdf_zero
from data_glove_wuji_teleop.cli import build_parser
from data_glove_wuji_teleop.profiles.dataglove.device import GloveDeviceProfile
from data_glove_wuji_teleop.profiles.dataglove.urdf_mapping import DataGloveUrdfMapping

from data_glove_wuji_teleop.profiles.joint_mapping import load_joint_mapping
from data_glove_wuji_teleop.project import (
    PROJECT_ROOT,
    get_hand_resources,
    validate_project,
)


class StandaloneProjectTest(unittest.TestCase):
    def test_pico_left_can_enter_calibration_without_an_existing_zero(self) -> None:
        with TemporaryDirectory() as directory:
            root = Path(directory)
            original = GloveDeviceProfile.load("left-000000e04c1a0399")
            profile = replace(
                original,
                path=root / f"{original.name}.json",
                zeros={}, active_zero=None,
            )
            profile.save()
            args = build_parser().parse_args([
                "calibrate-urdf-zero", "--glove-profile", str(profile.path), "--name", "initial",
            ])
            with (
                patch("data_glove_wuji_teleop.application.urdf_zero_calibration.prepare_glove_network"),
                patch("builtins.input", side_effect=KeyboardInterrupt),
            ):
                with self.assertRaises(KeyboardInterrupt):
                    command_calibrate_urdf_zero(args)
            self.assertIsNone(GloveDeviceProfile.load(profile.path).load_zero())
            mapping = profile.load_mapping()
            mapping.validate_stream(
                channels=21,
                cs_by_joint=[18, 3, 7, 11, 15, 20, 14, 10, 6, 2, 17, 13, 9, 5, 1, 16, 12, 8, 4, 0, 19],
            )
            with self.assertRaises(ValueError):
                mapping.require_verified_directions()

    def test_default_left_project_is_self_contained(self) -> None:
        report = validate_project(load_mujoco=True)

        self.assertEqual(report.calibrated_hand, "left")
        self.assertEqual(report.mapping_count, 20)
        self.assertEqual(report.mesh_count, 26)
        self.assertEqual(report.mujoco_joint_count, 20)

    def test_right_project_model_and_mapping_are_complete(self) -> None:
        report = validate_project(
            hand="right",
            load_mujoco=True,
            require_calibration=False,
        )

        self.assertIsNone(report.calibrated_hand)
        self.assertEqual(report.mapping_count, 20)
        self.assertEqual(report.mesh_count, 26)
        self.assertEqual(report.mujoco_joint_count, 20)

    def test_wuji_v2_left_and_right_control_resources_are_complete(self) -> None:
        for hand in ("left", "right"):
            resources = get_hand_resources(hand, generation="v2")
            self.assertEqual(resources.generation, "v2")
            self.assertTrue(resources.mjcf.is_file())
            self.assertTrue(resources.mapping.is_file())

            report = validate_project(
                generation="v2",
                hand=hand,
                load_mujoco=True,
                require_calibration=False,
            )
            self.assertEqual(report.generation, "v2")
            self.assertEqual(report.mapping_count, 20)
            self.assertEqual(report.mesh_count, 21)
            self.assertEqual(report.mujoco_joint_count, 20)
            self.assertEqual(report.mujoco_actuator_count, 20)

    def test_wuji_models_come_from_managed_assets_checkout(self) -> None:
        self.assertFalse((PROJECT_ROOT / "asset").exists())
        for hand in ("left", "right"):
            self.assertTrue(
                (
                    PROJECT_ROOT
                    / "assets"
                    / "wuji-description"
                    / "hand"
                    / "body"
                    / "urdf"
                    / f"{hand}.urdf"
                ).is_file()
            )

    def test_wuji_v2_simulation_assets_are_present_and_loadable(self) -> None:
        import mujoco

        body = (
            PROJECT_ROOT
            / "assets"
            / "wuji-description"
            / "hand2"
            / "hand2_beta1"
            / "body"
        )
        for hand in ("left", "right"):
            urdf_model = mujoco.MjModel.from_xml_path(
                str(body / "urdf" / f"{hand}.urdf")
            )
            mjcf_model = mujoco.MjModel.from_xml_path(
                str(body / "mjcf" / f"{hand}.xml")
            )
            self.assertEqual(urdf_model.nq, 20)
            self.assertEqual(urdf_model.nu, 0)
            self.assertEqual(mjcf_model.nq, 20)
            self.assertEqual(mjcf_model.nu, 20)






    def test_legacy_joint_mapping_field_names_remain_readable(self) -> None:
        current_path = (
            PROJECT_ROOT / "config/hands/wuji_v1/left_mapping.json"
        )
        legacy = json.loads(current_path.read_text(encoding="utf-8"))
        for item in legacy["mapping"]:
            item["mano_dof"] = item.pop("semantic_dof")
            item["wuji_joint"] = item.pop("model_joint")

        with TemporaryDirectory() as directory:
            legacy_path = Path(directory) / "left_mapping.json"
            legacy_path.write_text(
                json.dumps(legacy),
                encoding="utf-8",
            )
            rules = load_joint_mapping(
                legacy_path,
                expected_hand="left",
                expected_generation="v1",
            )

        self.assertEqual(len(rules), 20)



if __name__ == "__main__":
    unittest.main()
