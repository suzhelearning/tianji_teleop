import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys

import pytest


REPO_ROOT = Path(__file__).parents[3]


@pytest.fixture
def calibration_menu(tmp_path):
    project = tmp_path / "project"
    tracking = project / "tracking"
    scripts = tracking / "scripts"
    scripts.mkdir(parents=True)
    for name in ("calibrate_pico_arm.sh", "environment.sh"):
        shutil.copy2(REPO_ROOT / "scripts" / name, scripts / name)
    package = tracking / "src" / "pico_bridge"
    package.mkdir(parents=True)
    (package / "scripts").symlink_to(REPO_ROOT / "src" / "pico_bridge" / "scripts", target_is_directory=True)
    python = project / ".venv" / "bin" / "python"
    python.parent.mkdir(parents=True)
    # Execute the test runner's environment, not a developer checkout's .venv.
    python.write_text(f'#!/bin/sh\nexec {shlex.quote(sys.executable)} "$@"\n', encoding="utf-8")
    python.chmod(0o755)
    home = tmp_path / "home"
    home.mkdir()
    ros_started = tmp_path / "ros-started"
    ros_setup = tmp_path / "ros_setup.bash"
    ros_setup.write_text(f"touch {shlex.quote(str(ros_started))}\nreturn 99\n", encoding="utf-8")
    environment = os.environ.copy()
    for name in (
        "PYTHONHOME", "PYTHONPATH", "VIRTUAL_ENV", "PICO_CALIBRATION_DIR",
        "PICO_CALIBRATION_RECORDINGS_DIR", "PICO_CALIBRATION_VERBOSE",
        "EXO_REQUESTED_ROS_DOMAIN_ID",
    ):
        environment.pop(name, None)
    environment.update(
        HOME=str(home), ROS_DOMAIN_ID="120", ROS_LOCALHOST_ONLY="1", ROS_SETUP=str(ros_setup),
    )

    def run_script(*args):
        return subprocess.run(
            [str(scripts / "calibrate_pico_arm.sh"), *args],
            text=True,
            capture_output=True,
            env=environment,
            timeout=15,
            check=False,
        )

    return run_script, home, ros_started


def test_wrist_requires_tcp_before_starting_ros(calibration_menu):
    run_script, home, ros_started = calibration_menu
    result = run_script("left", "wrist")
    assert result.returncode == 2
    assert not ros_started.exists()
    assert not (home / ".config").exists()


def test_geometry_requires_tcp_before_starting_ros(calibration_menu):
    run_script, home, ros_started = calibration_menu
    result = run_script("right", "geometry")
    assert result.returncode == 2
    assert not ros_started.exists()
    assert not (home / ".config").exists()


def test_status_menu_is_read_only_without_ros(calibration_menu):
    run_script, home, ros_started = calibration_menu
    result = run_script("status")
    assert result.returncode == 0, result.stderr
    assert not ros_started.exists()
    assert not (home / ".config").exists()
