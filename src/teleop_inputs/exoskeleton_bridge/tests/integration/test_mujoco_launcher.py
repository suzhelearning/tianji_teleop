"""一代手 MuJoCo 启动脚本的进程和安全边界集成测试。"""

from __future__ import annotations

import os
import subprocess
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory


PROJECT_ROOT = Path(__file__).resolve().parents[2]
LAUNCHER = PROJECT_ROOT / "scripts" / "launch_mujoco.sh"


class MujocoLauncherTest(unittest.TestCase):

    def test_rejects_unknown_hand_generation(self) -> None:
        result = subprocess.run(
            [str(LAUNCHER), "--generation", "v3", "--hand", "left"],
            cwd=PROJECT_ROOT,
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )

        self.assertEqual(result.returncode, 2)
        self.assertIn("--generation 必须是 v1 或 v2", result.stderr)

    def test_rejects_missing_calibration_with_valid_recovery_command(self) -> None:
        with TemporaryDirectory() as directory:
            missing = Path(directory) / "missing.json"
            env = os.environ.copy()
            env["DATAGLOVE_CAL_FILE"] = str(missing)
            result = subprocess.run(
                [str(LAUNCHER), "--hand", "left"],
                cwd=PROJECT_ROOT,
                env=env,
                capture_output=True,
                text=True,
                timeout=5,
                check=False,
            )

        self.assertEqual(result.returncode, 1)
        self.assertIn(f"标定文件不存在：{missing}", result.stderr)
        self.assertIn("pixi run calibrate-left", result.stderr)

    def test_both_mode_requires_two_explicit_glove_hosts(self) -> None:
        env = os.environ.copy()
        env.pop("DATAGLOVE_LEFT_HOST", None)
        env.pop("DATAGLOVE_RIGHT_HOST", None)
        result = subprocess.run(
            [str(LAUNCHER), "--hand", "both"],
            cwd=PROJECT_ROOT,
            env=env,
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )

        self.assertEqual(result.returncode, 1)
        self.assertIn("DATAGLOVE_LEFT_HOST", result.stderr)
        self.assertIn("DATAGLOVE_RIGHT_HOST", result.stderr)

    def test_stops_bridge_when_v2_simulator_exits(self) -> None:
        with TemporaryDirectory() as directory:
            temp_dir = Path(directory)
            fake_pixi = temp_dir / "fake-pixi"
            trace = temp_dir / "bridge"
            fake_pixi.write_text(
                """#!/usr/bin/env python3
import os
import signal
import sys
import time
from pathlib import Path

trace = Path(os.environ["FAKE_PIXI_TRACE"])
command = " ".join(sys.argv)
if "data_glove_wuji_teleop.cli" in command:
    trace.with_suffix(".started").write_text("started", encoding="utf-8")

    def stop(_signum, _frame):
        trace.with_suffix(".terminated").write_text("terminated", encoding="utf-8")
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, stop)
    while True:
        time.sleep(0.05)

if "data_glove_wuji_teleop.adapters.simulation.mujoco_wuji_" in command:
    trace.with_suffix(".simulator").write_text(command, encoding="utf-8")
    deadline = time.monotonic() + 2.0
    while not trace.with_suffix(".started").exists():
        if time.monotonic() >= deadline:
            raise SystemExit(9)
        time.sleep(0.01)
    raise SystemExit(7)

raise SystemExit(8)
""",
                encoding="utf-8",
            )
            fake_pixi.chmod(0o755)
            env = os.environ.copy()
            env["DATAGLOVE_PIXI_BIN"] = str(fake_pixi)
            env["FAKE_PIXI_TRACE"] = str(trace)
            result = subprocess.run(
                [
                    str(LAUNCHER),
                    "--generation",
                    "v2",
                    "--hand",
                    "left",
                ],
                cwd=PROJECT_ROOT,
                env=env,
                capture_output=True,
                text=True,
                timeout=5,
                check=False,
            )

            self.assertEqual(result.returncode, 7)
            self.assertTrue(trace.with_suffix(".started").is_file())
            self.assertTrue(trace.with_suffix(".terminated").is_file())
            simulator_command = trace.with_suffix(".simulator").read_text(
                encoding="utf-8"
            )
            self.assertIn("mujoco_wuji_v2", simulator_command)
            self.assertIn(
                "config/hands/wuji_v2/left_mapping.json",
                simulator_command,
            )


if __name__ == "__main__":
    unittest.main()
