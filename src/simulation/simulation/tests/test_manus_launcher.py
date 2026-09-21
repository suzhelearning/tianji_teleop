"""Manus launcher contract, checked without opening a Manus device.

The launcher must refuse conflicting personnel profiles and fail before starting
input when either hand's calibration is missing.
"""

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
        package = self.root / "src/teleop_inputs/manus_bridge/manus_bridge"
        (self.root / "bash").mkdir(parents=True)
        package.mkdir(parents=True)
        # A stand-in launcher instance: the entry script is copied verbatim and a
        # fake module tree records what it would have started.
        self.script = self.root / "bash/run_manus.sh"
        self.script.write_bytes((workspace() / "bash/run_manus.sh").read_bytes())
        (self.root / "bash/pixi.bash").write_text("cd -- \"" + str(self.root) + "\"\n")
        # The fake package tree shadows the installed one, and calibration is
        # resolved into the fixture so the launcher's own check decides.
        self.calibration = self.root / "calibration"
        self.calibration.mkdir(parents=True)
        (package / "__init__.py").write_text("")
        (package / "paths.py").write_text(
            "import os\n"
            "from pathlib import Path\n"
            "\n"
            "def calibration_dir() -> Path:\n"
            "    return Path(os.environ['TEST_ROOT']) / 'calibration'\n")
        (self.root / "bash/environment.sh").write_text(
            'export TIANJI_PYTHON="'"$TEST_PYTHON"'"\n'
            'export TIANJI_WORKSPACE="'"$TEST_ROOT"'"\n'
            'export PYTHONPATH="'"$TEST_ROOT"'/src/teleop_inputs/manus_bridge:$PYTHONPATH"\n')
        (package / "start_hand_teleop.py").write_text(
            "import os\nfrom pathlib import Path\n"
            "Path(os.environ['TEST_ROOT'], 'input-started').touch()\n")
        (package / "list_calibrations.py").write_text(
            "def main():\n    print('person')\n    return 0\n")

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
        self.assertFalse((self.root / "input-started").exists())

    def test_missing_calibration_fails_before_starting_input(self):
        calibration = self.calibration
        calibration.mkdir(exist_ok=True)
        (calibration / "personLeftMetaglovePro.mcal").write_text("fixture")
        # Right side is absent, so nothing may start.
        result = self.run_launcher("--calibration-user", "person")
        self.assertEqual(result.returncode, 2)
        self.assertFalse((self.root / "input-started").exists())


if __name__ == "__main__":
    unittest.main()
