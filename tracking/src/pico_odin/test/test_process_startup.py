import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid

from ament_index_python.packages import get_package_prefix


def _executable(name):
    return Path(get_package_prefix("pico_odin")) / "lib" / "pico_odin" / name


def _isolated_environment():
    environment = os.environ.copy()
    unique = uuid.uuid4().int
    environment["ROS_DOMAIN_ID"] = str(100 + unique % 100)
    return environment


def test_calibrator_rejects_non_raw_feedback_topics():
    result = subprocess.run(
        [
            str(_executable("odin_pelvis_calibrator")),
            "--ros-args",
            "-p", "pico_topic:=/pico/smpl_odin",
        ],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=5,
        env=_isolated_environment(),
        check=False,
    )

    assert result.returncode != 0
    assert "requires canonical /pico/smpl" in result.stdout


def test_runtime_stays_disabled_when_calibration_file_is_missing():
    with tempfile.TemporaryDirectory() as directory:
        missing = Path(directory) / "missing.yaml"
        process = subprocess.Popen(
            [
                str(_executable("odin_pelvis_runtime")),
                "--ros-args",
                "-p", f"extrinsics_file:={missing}",
            ],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env=_isolated_environment(),
        )
        try:
            time.sleep(0.30)
            assert process.poll() is None
            process.send_signal(signal.SIGINT)
            output, _ = process.communicate(timeout=5)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=5)

    assert process.returncode == 0
    assert "Pelvis correction disabled" in output
