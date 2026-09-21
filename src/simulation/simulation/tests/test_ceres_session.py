import hashlib
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from tianji_runtime import controller_profile
from tianji_runtime.resources import controller_resource
import yaml
import io
from ..ceres_session import launch
from ..run_sim import main


class CeresSessionTests(unittest.TestCase):
    def test_hands_default_on_with_explicit_arms_only(self):
        for backend in ('franka-dls', 'ceres'):
            with patch('simulation.ceres_session.launch', return_value=0) as launch_mock:
                self.assertEqual(main(['--ik-backend', backend]), 0)
                self.assertTrue(launch_mock.call_args.args[0].hand_teleop)
                self.assertEqual(main(['--ik-backend', backend, '--no-hand-teleop']), 0)
                self.assertFalse(launch_mock.call_args.args[0].hand_teleop)
        with self.assertRaises(SystemExit):
            main(['--hand-teleop', '--no-hand-teleop'])
        with self.assertRaises(SystemExit):
            main(['--hand-port', '15000'])
        with self.assertRaises(SystemExit):
            main(['--ik-backend', 'spark', '--no-hand-teleop'])
    def test_optional_hand_input_and_port_validation(self):
        with patch("simulation.ceres_session.launch", return_value=0) as mocked:
            self.assertEqual(main(["--user", "person", "--hand-teleop", "--hand-port", "25003"]), 0)
            self.assertTrue(mocked.call_args.args[0].hand_teleop)
            self.assertEqual(mocked.call_args.args[0].hand_port, 25003)
        for extra in (["--hand-port", "0"], ["--hand-port", "65536"],
                      ["--hand-port", "15000"], ["--ik-backend", "spark"]):
            with self.subTest(extra=extra), self.assertRaises(SystemExit):
                main(["--hand-teleop", *extra])

    def test_hands_launch_uses_native_receiver_without_export(self):
        args = SimpleNamespace(config=None, model=None, duration=0, pico_port=25002,
                               ik_backend="franka-dls", hand_teleop=True, hand_port=25003)
        with tempfile.TemporaryDirectory() as directory, \
             patch("simulation.ceres_session.tempfile.mkdtemp", return_value=directory), \
             patch("simulation.ceres_session.os.execv") as execute:
            launch(args)
        command = execute.call_args.args[1]
        self.assertIn("--hand-teleop", command)
        self.assertNotIn("--no-hand-teleop", command)
        self.assertEqual(command[command.index("--hand-bind")+1], "127.0.0.1")
        self.assertEqual(command[command.index("--hand-port")+1], "25003")
        self.assertEqual(command[command.index("--joint-command-port")+1], "0")

    def test_jump_bypass_is_explicit_and_direct_backend_only(self):
        with patch("simulation.ceres_session.launch", return_value=0) as mocked:
            self.assertEqual(main(["--sim-allow-pico-jumps"]), 0)
            self.assertTrue(mocked.call_args.args[0].sim_allow_pico_jumps)
        with self.assertRaises(SystemExit):
            main(["--ik-backend", "spark", "--sim-allow-pico-jumps"])

    def test_named_user_is_forwarded(self):
        with patch("simulation.ceres_session.launch", return_value=0) as mocked:
            self.assertEqual(main(["--user", "person"]), 0)
            self.assertEqual(mocked.call_args.args[0].user, "person")

    def test_named_user_rejects_unsupported_input_wiring(self):
        for args in (["--ik-backend", "spark"], ["--pico-port", "25000"]):
            with self.subTest(args=args), self.assertRaises(SystemExit):
                main(["--user", "person", *args])

    def test_input_failure_prevents_native_viewer(self):
        args = SimpleNamespace(config=None, model=None, duration=0, pico_port=15000,
                               ik_backend="franka-dls", user="person")
        with tempfile.TemporaryDirectory() as directory, \
             patch("simulation.ceres_session.tempfile.mkdtemp", return_value=directory), \
             patch("simulation.ceres_session.os.execv") as execute, \
             patch("sys.stderr", new_callable=io.StringIO) as errors, \
             patch("simulation.pico_owned_session.run_with_owned_pico", return_value=2) as start:
            self.assertEqual(launch(args), 2)
            self.assertNotIn("Traceback", errors.getvalue())
            self.assertEqual(start.call_args.args[1], "person")
            execute.assert_not_called()

    def test_missing_standard_binary_does_not_fallback_to_old_build(self):
        args = SimpleNamespace(ik_backend="franka-dls")
        with patch("simulation.ceres_session.Path.is_file", return_value=False), \
             patch("simulation.ceres_session.os.execv") as execute:
            with self.assertRaisesRegex(RuntimeError, "pixi run build"):
                launch(args)
            execute.assert_not_called()

    def test_default_launch_selects_franka_dls_without_devices(self):
        with patch("simulation.ceres_session.launch", return_value=0) as launch_mock:
            self.assertEqual(main([]), 0)
            self.assertEqual(launch_mock.call_args.args[0].ik_backend, "franka-dls")

    def test_explicit_ceres_still_selectable(self):
        with patch("simulation.ceres_session.launch", return_value=0) as launch_mock:
            self.assertEqual(main(["--ik-backend", "ceres"]), 0)
            self.assertEqual(launch_mock.call_args.args[0].ik_backend, "ceres")

    def test_default_does_not_silently_fallback_for_unsupported_mode(self):
        with patch("simulation.ceres_session.launch") as launch_mock:
            with self.assertRaises(SystemExit):
                main(["--simulation-mode", "dynamics"])
            launch_mock.assert_not_called()

    def test_dls_selects_its_own_profile(self):
        args = SimpleNamespace(config=None, model=None, duration=0, pico_port=25002,
                               ik_backend="franka-dls")
        with tempfile.TemporaryDirectory() as directory:
            with patch("simulation.ceres_session.tempfile.mkdtemp", return_value=directory), \
                 patch("simulation.ceres_session.os.execv"):
                launch(args)
            runtime = yaml.safe_load((Path(directory)/"runtime.yaml").read_text())
            self.assertEqual(runtime["ik"]["algorithm"], "pico_ee_franka_dls")
            self.assertEqual(runtime["pico_ee_franka_dls"]["post_smoothing"]["mode"], "ruckig")

    def test_native_ownership_and_runtime_only_enable(self):
        source = controller_profile("qp_ik_pico_shared_root_ceres.yaml")
        before = source.read_bytes()
        args = SimpleNamespace(config=source, model=None, duration=2, pico_port=25001,
                               sim_allow_pico_jumps=True, hand_teleop=False)
        with tempfile.TemporaryDirectory() as directory:
            with patch("simulation.ceres_session.tempfile.mkdtemp", return_value=directory), \
                 patch("simulation.ceres_session.os.execv") as execute:
                launch(args)
            command = execute.call_args.args[1]
            # Resolved through the control install prefix rather than a build path.
            self.assertEqual(Path(command[0]).name, "tianji_qp_ik_viewer")
            self.assertIn("--simulation-recovery", command)
            self.assertIn("--sim-allow-pico-jumps", command)
            self.assertIn("--no-hand-teleop", command)
            self.assertEqual(command[command.index("--joint-command-port")+1], "0")
            runtime = yaml.safe_load((Path(directory)/"runtime.yaml").read_text())
            self.assertTrue(runtime["spark_shared_root"]["enabled"])
            self.assertTrue(Path(runtime["spark_shared_root"]["robot_geometry_artifact"]).is_absolute())
            self.assertTrue(Path(runtime["controller"]["pico_ee_dls_kinematics_urdf_path"]).is_absolute())
        self.assertEqual(source.read_bytes(), before)

    def test_installed_launch_preserves_artifacts_and_custom_home(self):
        source = controller_profile("qp_ik_pico_shared_root_ceres.yaml")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "pixi.toml").touch()
            installed = root / "install/control/share/tianji_controller/config"
            installed.mkdir(parents=True)
            description = root / "install/default/share/tianji_description/models"
            description.mkdir(parents=True)
            config = yaml.safe_load(source.read_text())
            shared = config["spark_shared_root"]
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
                                   duration=0, pico_port=25001, hand_teleop=False)
            with patch.dict("os.environ", {"TIANJI_WORKSPACE": str(root), "TIANJI_ENVIRONMENT": "default", "AMENT_PREFIX_PATH": ""}), \
                 patch("simulation.ceres_session.native_executable", return_value=root / "viewer"), \
                 patch("simulation.ceres_session.tempfile.mkdtemp", return_value=str(runtime_dir)), \
                 patch("simulation.ceres_session.os.execv") as execute:
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
            command = execute.call_args.args[1]
            self.assertEqual(command[command.index("--model") + 1], str(args.model))
            self.assertEqual(geometry_path.read_bytes(), geometry_bytes)
            self.assertEqual(profile.read_bytes(), original_profile)

    def test_reject_unsupported_modes(self):
        for options in (["--headless"], ["--simulation-mode", "dynamics"],
                        ["--mapped-palm-xz-calibration"], ["--duration", "nan"],
                        ["--pico-port", "0"]):
            with self.subTest(options=options), self.assertRaises(SystemExit):
                main(["--ik-backend", "ceres"]+options)

if __name__ == "__main__":
    unittest.main()
