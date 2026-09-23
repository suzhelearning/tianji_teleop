import hashlib
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from tianji_runtime import controller_profile
from tianji_runtime.resources import controller_resource
import yaml
from ..dls_session import launch
from ..run_sim import main


class DlsSessionTests(unittest.TestCase):
    def test_retired_routes_and_invalid_options_fail_before_launch(self):
        cases = (
            ["--ik-backend", "ceres"],
            ["--ik-backend", "spark"],
            ["--ik-backend", "mapped-palm"],
            ["--simulation-mode", "dynamics"],
            ["--mapped-palm-xz-calibration"],
            ["--headless"],
            ["--pico-port", "25000"],
            ["--hand-teleop", "--no-hand-teleop"],
            ["--hand-port", "0"],
            ["--hand-port", "65536"],
            ["--duration", "nan"],
            ["--duration", "-1"],
        )
        for options in cases:
            with self.subTest(options=options), \
                    patch("simulation.dls_session.launch") as start, \
                    self.assertRaises(SystemExit) as raised:
                main(options)
            self.assertEqual(raised.exception.code, 2)
            start.assert_not_called()

    def test_standard_dls_configuration_starts_the_actual_ros_core_without_export(self):
        import os
        import subprocess
        from tianji_runtime import native_executable
        from tianji_runtime.resources import ResourceNotFound
        try:
            native_executable("tianji_arm_ros")
        except ResourceNotFound:
            self.skipTest("build-arm-ros is required for native startup verification")
        args = SimpleNamespace(config=None, model=None, duration=2.0,
                               ik_backend="franka-dls", hand_teleop=False)
        results = []

        def execute(_executable, command):
            results.append(subprocess.run(
                [*command, "--headless"], capture_output=True, text=True, timeout=20,
                env={**os.environ, "ROS_DOMAIN_ID": "121"}))

        with tempfile.TemporaryDirectory() as directory, \
             patch("simulation.dls_session.tempfile.mkdtemp", return_value=directory), \
             patch("simulation.dls_session.os.execv", side_effect=execute):
            launch(args)
        self.assertEqual(results[0].returncode, 0, results[0].stderr)
        self.assertIn("DLS_SIM: WAITING", results[0].stdout)
        self.assertNotIn("joint_command_state=ready", results[0].stdout)

    def test_missing_standard_binary_does_not_fallback_to_old_build(self):
        args = SimpleNamespace(ik_backend="franka-dls")
        with patch("simulation.dls_session.Path.is_file", return_value=False), \
             patch("simulation.dls_session.os.execv") as execute:
            with self.assertRaises(RuntimeError):
                launch(args)
            execute.assert_not_called()

    def test_installed_launch_preserves_artifacts_and_custom_home(self):
        from tianji_runtime.resources import config_path
        source = controller_profile("qp_ik_pico_shared_root_dls.yaml")
        robot_config = config_path("robot.json").read_bytes()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "pixi.toml").touch()
            (root / "config").mkdir()
            (root / "config" / "robot.json").write_bytes(robot_config)
            installed = root / "install/control/share/tianji_controller/config"
            installed.mkdir(parents=True)
            description = root / "install/default/share/tianji_description/models"
            description.mkdir(parents=True)
            config = yaml.safe_load(source.read_text())
            shared = config["shared_root"]
            for key in ("input_contract_artifact", "robot_geometry_artifact"):
                original = source.parent / shared[key]
                (installed / original.name).write_bytes(original.read_bytes())
            geometry_path = installed / shared["robot_geometry_artifact"]
            geometry_bytes = geometry_path.read_bytes()
            geometry = yaml.safe_load(geometry_bytes)["robot_geometry"]
            for key in ("urdf_path", "mujoco_xml_path"):
                original = controller_resource(source.parent / shared["robot_geometry_artifact"], geometry[key])
                (description / original.name).write_bytes(original.read_bytes())
            controller = config["controller"]
            controller.pop("initial_left_q_rad")
            controller.pop("initial_right_q_rad")
            controller["home_config"] = "custom-home.yaml"
            home = installed / "custom-home.yaml"
            home.write_text("left_home_rad: [0, 0, 0, 0, 0, 0, 0]\nright_home_rad: [0, 0, 0, 0, 0, 0, 0]\n")
            profile = installed / source.name
            profile.write_text(yaml.safe_dump(config))
            original_profile = profile.read_bytes()
            runtime_dir = root / "runtime"
            runtime_dir.mkdir()
            args = SimpleNamespace(config=profile, model=description / Path(geometry["mujoco_xml_path"]).name,
                                   duration=0, hand_teleop=False)
            with patch.dict("os.environ", {"TIANJI_WORKSPACE": str(root), "TIANJI_ENVIRONMENT": "default", "AMENT_PREFIX_PATH": ""}), \
                 patch("simulation.dls_session.native_executable", return_value=root / "viewer"), \
                 patch("simulation.dls_session.tempfile.mkdtemp", return_value=str(runtime_dir)), \
                 patch("simulation.dls_session.os.execv"):
                launch(args)
                runtime = yaml.safe_load((runtime_dir / "runtime.yaml").read_text())
                self.assertEqual(Path(runtime["controller"]["home_config"]).read_bytes(), home.read_bytes())
                self.assertEqual(Path(runtime["controller"]["pico_ee_dls_kinematics_urdf_path"]),
                                 description / Path(geometry["urdf_path"]).name)
                for key in ("urdf_path", "mujoco_xml_path"):
                    self.assertEqual(controller_resource(geometry_path, geometry[key]),
                                     description / Path(geometry[key]).name)
                    fingerprint = "urdf_sha256" if key == "urdf_path" else "mujoco_xml_sha256"
                    self.assertEqual(hashlib.sha256(controller_resource(geometry_path, geometry[key]).read_bytes()).hexdigest(),
                                     geometry[fingerprint])
            self.assertEqual(geometry_path.read_bytes(), geometry_bytes)
            self.assertEqual(profile.read_bytes(), original_profile)


if __name__ == "__main__":
    unittest.main()
