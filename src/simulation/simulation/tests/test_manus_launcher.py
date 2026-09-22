"""Manus identity selection and calibration checks without opening a device."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from tianji_runtime import workspace


class ManusLauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        package = self.root / "src/teleop_inputs/manus/manus_bridge"
        (self.root / "bash").mkdir(parents=True)
        package.mkdir(parents=True)
        (self.root / "pixi.toml").touch()
        source = workspace()
        self.script = self.root / "bash/run_manus.sh"
        self.script.write_bytes((source / "bash/run_manus.sh").read_bytes())
        (self.root / "bash/pixi.bash").write_text("cd -- \"" + str(self.root) + "\"\n")
        self.calibration = self.root / "profiles/person/manus"
        self.calibration.mkdir(parents=True)
        (package / "__init__.py").write_text("")
        modules = source / "src/teleop_inputs/manus/manus_bridge"
        for name in ("paths.py", "list_calibrations.py", "retarget_worker.py"):
            (package / name).write_bytes((modules / name).read_bytes())
        (package / "launcher.py").write_bytes((modules / "start_hand_teleop.py").read_bytes())
        (self.root / "bash/environment.sh").write_text(
            'export TIANJI_PYTHON="'"$TEST_PYTHON"'"\n'
            'export TIANJI_WORKSPACE="'"$TEST_ROOT"'"\n'
            'export PYTHONPATH="'"$TEST_ROOT"'/src/teleop_inputs/manus:$PYTHONPATH"\n')
        # Run the real launcher until its first native-resource lookup. Reaching
        # this boundary proves identity and pair validation accepted the input;
        # no SDK, ROS, or worker process is needed for these tests.
        (package / "start_hand_teleop.py").write_text(
            "import os\nfrom pathlib import Path\n"
            "from manus_bridge import launcher\n"
            "def runtime_boundary(*args):\n"
            "    Path(os.environ['TEST_ROOT'], 'runtime-reached').touch()\n"
            "    raise SystemExit(0)\n"
            "launcher.vendor_path = runtime_boundary\n"
            "raise SystemExit(launcher.main())\n")

    def run_launcher(self, *args):
        return subprocess.run(
            ["bash", str(self.script), *args],
            cwd=self.root,
            env={**os.environ, "TEST_PYTHON": sys.executable, "TEST_ROOT": str(self.root),
                 "TIANJI_PIXI_ACTIVE": "1", "CONDA_PREFIX": str(self.root),
                 "ROS_DISTRO": "jazzy", "PIXI_ENVIRONMENT_NAME": "default"},
            capture_output=True, text=True, timeout=60)

    def test_rejects_more_than_one_identity_flag(self):
        result = self.run_launcher("--user", "a", "--calibration-user", "b")
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse((self.root / "runtime-reached").exists())

    def test_missing_calibration_fails_before_runtime_checks(self):
        (self.calibration / "personLeftMetaglovePro.mcal").write_text("fixture")
        for flag, user in (("--user", "person"), ("--calibration-user", "person"), ("--user", "kj")):
            with self.subTest(flag=flag, user=user):
                result = self.run_launcher(flag, user)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertFalse((self.root / "runtime-reached").exists())

    def test_both_identity_flags_accept_pair_without_profile_yaml(self):
        for side in ("Left", "Right"):
            (self.calibration / f"person{side}MetaglovePro.mcal").write_text("fixture")
        for flag in ("--user", "--calibration-user"):
            with self.subTest(flag=flag):
                result = self.run_launcher(flag, "person")
                self.assertEqual(result.returncode, 0, result.stderr)
                marker = self.root / "runtime-reached"
                self.assertTrue(marker.exists())
                marker.unlink()

    def test_list_aliases_only_report_complete_pairs(self):
        for side in ("Left", "Right"):
            (self.calibration / f"person{side}MetaglovePro.mcal").write_text("fixture")
        partial = self.root / "profiles/partial/manus"
        partial.mkdir(parents=True)
        (partial / "partialLeftMetaglovePro.mcal").write_text("fixture")
        for flag in ("--list-users", "--list-calibration-users"):
            with self.subTest(flag=flag):
                result = self.run_launcher(flag)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stdout.splitlines(), ["person"])
                self.assertFalse((self.root / "runtime-reached").exists())


if __name__ == "__main__":
    unittest.main()
