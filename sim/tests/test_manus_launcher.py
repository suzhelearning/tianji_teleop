"""Launcher tests use an isolated fake SDK tree; never open a Manus device."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
import tomllib


class ManusLauncherTests(unittest.TestCase):
    def test_pixi_manus_uses_own_interpreter_and_ros(self):
        root = Path(__file__).resolve().parents[2]
        manifest = tomllib.loads((root/'pixi.toml').read_text())
        self.assertIn('-e manus manus-input', manifest['tasks']['manus'])
        task = manifest['feature']['manus']['tasks']['manus-input']
        self.assertIn('TIANJI_PYTHON=$CONDA_PREFIX/bin/python', task)
        self.assertIn('ROS_SETUP=$PIXI_PROJECT_ROOT/.pixi/envs/tracking/setup.bash', task)
        self.assertNotIn('.venv', task)
        self.assertTrue(manifest['environments']['manus']['no-default-feature'])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        for directory in ('scripts', 'tracking/scripts', 'manus/calibration'):
            (self.root/directory).mkdir(parents=True)
        shutil.copyfile(Path(__file__).resolve().parents[2]/'manus.sh', self.root/'manus.sh')
        (self.root/'scripts/environment.sh').write_text('export TIANJI_PYTHON="${TEST_PYTHON}"\n')
        (self.root/'tracking/scripts/environment.sh').write_text('touch tracking_sourced\n')
        (self.root/'manus/start_hand_teleop.py').write_text('import json,sys\nprint(json.dumps(sys.argv[1:]))\n')
        (self.root/'tianji.py').write_text('print("mapped_person")\n')
        for side in ('Left', 'Right'):
            (self.root/f'manus/calibration/person{side}MetaglovePro.mcal').write_text('fixture')

    def run_launcher(self, *args):
        return subprocess.run(['bash', 'manus.sh', *args], cwd=self.root,
                              env={**os.environ, 'TEST_PYTHON': sys.executable},
                              capture_output=True, text=True, timeout=5)

    def test_explicit_calibration_without_person_profile(self):
        result = self.run_launcher('--calibration-user', 'person', '--port', '25003')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout), ['--user', 'person', '--port', '25003'])

    def test_missing_calibration_fails_before_ros_or_input(self):
        result = self.run_launcher('--calibration-user', 'missing')
        self.assertEqual(result.returncode, 2)
        self.assertFalse((self.root/'tracking_sourced').exists())

    def test_reject_bad_or_duplicate_identity(self):
        for args in [('--calibration-user', '../person'),
                     ('--calibration-user', 'person', '--user', 'other'),
                     ('--user', 'person', '--calibration-user', 'other')]:
            with self.subTest(args=args):
                self.assertEqual(self.run_launcher(*args).returncode, 2)
                self.assertFalse((self.root/'tracking_sourced').exists())

    def test_list_calibrations_does_not_source_ros(self):
        result = self.run_launcher('--list-calibration-users')
        self.assertEqual(result.returncode, 0)
        self.assertEqual(result.stdout.strip(), 'person')
        self.assertFalse((self.root/'tracking_sourced').exists())

    def test_old_profile_mapping_still_used(self):
        result = self.run_launcher('--user', 'person')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout), ['--user', 'mapped_person'])
