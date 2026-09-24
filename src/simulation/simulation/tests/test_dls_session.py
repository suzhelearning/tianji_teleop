import csv
import hashlib
import json
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
            ["--hand-source", "unknown"],
            ["--hand-port", "16000"],
            ["--hand-source", "exoskeleton", "--hand-port", "0"],
            ["--hand-source", "exoskeleton", "--hand-port", "65536"],
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

    def test_actual_ros_simulation_holds_initial_arms_without_input(self):
        import os
        import subprocess
        from tianji_runtime import native_executable
        from tianji_runtime.resources import ResourceNotFound
        try:
            native_executable("tianji_arm_ros")
        except ResourceNotFound:
            self.skipTest("build-arm-ros is required for native startup verification")

        def execute(_executable, command):
            result = subprocess.run(
                [*command, "--headless"], capture_output=True, text=True, timeout=20,
                env={**os.environ, "ROS_DOMAIN_ID": "121"})
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

        for options in ([], ["--no-hand-teleop"]):
            with self.subTest(options=options), \
                 tempfile.TemporaryDirectory() as directory, \
                 patch("simulation.dls_session.tempfile.mkdtemp", return_value=directory), \
                 patch("simulation.dls_session.os.execv", side_effect=execute):
                main(["--duration", "2", *options])
                runtime = yaml.safe_load((Path(directory) / "runtime.yaml").read_text())
                with (Path(directory) / "joints.csv").open() as stream:
                    samples = list(csv.DictReader(stream))
                self.assertGreater(float(samples[-1]["control_time_seconds"]), 0.1)
                for side in ("left", "right"):
                    initial = runtime["controller"][f"initial_{side}_q_rad"]
                    for joint, expected in enumerate(initial, start=1):
                        positions = [float(sample[f"{side}_j{joint}_actual_q"]) for sample in samples]
                        self.assertAlmostEqual(min(positions), expected, places=8)
                        self.assertAlmostEqual(max(positions), expected, places=8)

    def test_manus_configuration_rejects_missing_invalid_or_shared_topics(self):
        left, right = "/hands/left", "/hands/right"
        invalid = (
            None,
            {"left_hand": left},
            {"left_hand": 3, "right_hand": right},
            {"left_hand": "/hands/left bad", "right_hand": right},
            {"left_hand": "hands/left", "right_hand": right},
            {"left_hand": left, "right_hand": left},
            {"left_hand": "/pico/arm_input", "right_hand": right},
        )
        with tempfile.TemporaryDirectory() as directory:
            robot_path = Path(directory) / "robot.json"
            for topics in invalid:
                robot_path.write_text(json.dumps({
                    "pico_input_topic": "/pico/arm_input",
                    "hand_command_topics": topics,
                }))
                with self.subTest(topics=topics), \
                     patch("simulation.dls_session.native_executable", return_value=Path("/unused/viewer")), \
                     patch("simulation.dls_session.config_path", return_value=robot_path), \
                     patch("simulation.dls_session.os.execv") as execute:
                    with self.assertRaises(ValueError):
                        launch(SimpleNamespace(config=None))
                    execute.assert_not_called()

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
        robot_config = json.loads(config_path("robot.json").read_text())
        robot_config.pop("hand_command_topics")
        robot_config["hand_port"] = 0
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "pixi.toml").touch()
            (root / "config").mkdir()
            (root / "config" / "robot.json").write_text(json.dumps(robot_config))
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
