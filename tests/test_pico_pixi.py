"""Pixi entry contracts; no devices, ROS nodes or tmux sessions are started."""
import os
from pathlib import Path
import subprocess
import tomllib

ROOT = Path(__file__).resolve().parents[1]


def test_tracking_tasks_pin_matching_python_and_sdk():
    manifest = tomllib.loads((ROOT / "pixi.toml").read_text())
    tasks = manifest["feature"]["tracking"]["tasks"]
    scripts = {
        "ensure-pico-user": "scripts/ensure_pico_user.sh",
        "setup-pico": "scripts/setup_pico.sh",
        "build-tracking": "tracking/scripts/build.sh",
        "pico-driver": "tracking/scripts/start_pico_driver.sh",
        "calibrate-pico": "scripts/calibrate_pico_simple.sh",
        "pico-simple": "scripts/start_simple_pico.sh",
    }
    for name, script in scripts.items():
        command = tasks[name]
        assert "TIANJI_PYTHON=$CONDA_PREFIX/bin/python" in command
        assert "ROS_SETUP=$CONDA_PREFIX/setup.bash" in command
        assert command.endswith("bash " + script)
        assert (ROOT / script).is_file()
    assert manifest["environments"]["tracking"]["no-default-feature"]


def test_stop_task_and_displayed_commands_use_tracking_environment():
    manifest = tomllib.loads((ROOT / "pixi.toml").read_text())
    assert manifest["feature"]["tracking"]["tasks"]["stop-pico"] == "bash tracking/scripts/start_tianji_pico_teleop.sh --stop"
    for path in ("scripts/setup_pico.py", "tracking/scripts/start_tianji_pico_teleop.sh",
                 "docs/pico-simple-calibration.md"):
        assert "pixi run -e tracking stop-pico" in (ROOT / path).read_text()


def test_simple_help_does_not_require_environment_or_device():
    result = subprocess.run(
        ["bash", str(ROOT / "scripts/start_simple_pico.sh"), "--help"],
        env={**os.environ, "TIANJI_PYTHON": "/missing/python"},
        capture_output=True, text=True, timeout=5,
    )
    assert result.returncode == 0, result.stderr
    assert "not an executor" in result.stdout


def test_tmux_children_pin_environment_with_shell_safe_paths():
    source = (ROOT / "tracking/scripts/start_tianji_pico_teleop.sh").read_text()
    # Execute only command construction, never session/device operations.
    fragment = source.split("printf -v repo_quoted", 1)[1].split('window_name="driver"', 1)[0]
    fragment = "printf -v repo_quoted" + fragment
    env = {**os.environ, "TIANJI_PYTHON": "/tmp/sdk space/py'thon",
           "CONDA_PREFIX": "/tmp/sdk space"}
    script = '''set -eu
repo_root=/tmp/not-a-checkout
ros_setup='/tmp/sdk space/setup.bash'
calibration_dir=''
pico_world_x_offset=''
''' + fragment + '''
for inner in "$driver_inner" "$m0_inner" "$bridge_inner"; do
  [[ "$inner" == *"$ros_environment"* ]]
done
eval "$ros_environment"
printf '%s\\n' "$TIANJI_PYTHON" "$ROS_SETUP" "$CONDA_PREFIX"
'''
    result = subprocess.run(["bash", "-c", script], env=env,
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [env["TIANJI_PYTHON"],
                                         "/tmp/sdk space/setup.bash", env["CONDA_PREFIX"]]
