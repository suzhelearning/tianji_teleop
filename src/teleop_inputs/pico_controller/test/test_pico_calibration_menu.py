import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys

import pytest


WORKSPACE = Path(__file__).resolve().parents[4]
BASH_SCRIPTS = WORKSPACE / "bash"
PICO_BRIDGE_SCRIPTS = WORKSPACE / "src" / "teleop_inputs" / "pico_controller" / "scripts"


@pytest.fixture
def calibration_menu(tmp_path):
    project = tmp_path / "project"
    scripts = project / "bash"
    scripts.mkdir(parents=True)
    shutil.copy2(BASH_SCRIPTS / "calibrate_pico_arm.sh", scripts)
    (scripts / "environment.sh").write_text(
        f"export TIANJI_PYTHON={shlex.quote(sys.executable)}\n", encoding="utf-8"
    )
    package = project / "src" / "teleop_inputs" / "pico_controller"
    package.mkdir(parents=True)
    (package / "scripts").symlink_to(PICO_BRIDGE_SCRIPTS, target_is_directory=True)
    home = tmp_path / "home"
    home.mkdir()
    ros_started = tmp_path / "ros-started"
    binaries = tmp_path / "bin"
    binaries.mkdir()
    ros2 = binaries / "ros2"
    ros2.write_text(f"#!/bin/sh\ntouch {shlex.quote(str(ros_started))}\nexit 99\n", encoding="utf-8")
    ros2.chmod(0o755)
    environment = os.environ.copy()
    for name in (
        "PYTHONHOME", "PYTHONPATH", "VIRTUAL_ENV", "PICO_CALIBRATION_DIR",
        "PICO_CALIBRATION_RECORDINGS_DIR", "PICO_CALIBRATION_VERBOSE",
        "EXO_REQUESTED_ROS_DOMAIN_ID",
    ):
        environment.pop(name, None)
    environment.update(
        HOME=str(home), ROS_DOMAIN_ID="120", PATH=str(binaries) + ":" + os.environ["PATH"],
    )

    def run_script(*args):
        return subprocess.run(
            ["bash", str(scripts / "calibrate_pico_arm.sh"), *args],
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
