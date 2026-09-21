from __future__ import annotations

import argparse
import socket
import struct
import subprocess
import time
import zlib
import os
from pathlib import Path
import sys


from _workspace import native_binary, workspace

# The helper sits beside this script; ctest invokes it by path.
sys.path.insert(0, str(Path(__file__).resolve().parent))

PROJECT_ROOT = workspace()
VIEWER = native_binary("tianji_qp_ik_viewer")
MODEL = PROJECT_ROOT / "src/teleop_outputs/tianji/tianji_description/models/marvin_m6_wuji2.xml"
CONFIG = (PROJECT_ROOT / "src/teleop_outputs/tianji/tianji_controller/native"
          / "config" / "qp_ik_pico_teleop.yaml")
PACKET_SIZE = 364


def _packet(sequence: int, right_joint0: float, left_joint0: float) -> bytes:
    values = [0.0] * 40
    values[0] = left_joint0
    values[20] = right_joint0
    stamp = time.monotonic_ns()
    body = struct.pack(
        "<4sBBHQqqq40d",
        b"TJH2",
        2,
        (1 << 0) | (1 << 1),
        PACKET_SIZE,
        sequence,
        stamp,
        stamp,
        stamp,
        *values,
    )
    return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


def _free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def test_full_model_viewer_applies_bimanual_hand_packet_without_arm_failure(
    viewer: Path = VIEWER,
) -> None:
    assert viewer.exists(), f"build viewer first: {viewer}"
    port = _free_udp_port()
    command = [
        str(viewer),
        "--config",
        str(CONFIG),
        "--model",
        str(MODEL),
        "--no-pico-teleop",
        "--hand-teleop",
        "--hand-bind",
        "127.0.0.1",
        "--hand-port",
        str(port),
        "--headless",
        "--duration",
        "4.0",
        "--algorithm",
        "hierarchical_qp",
    ]
    environment = {
        "LD_LIBRARY_PATH": (
            f"{PROJECT_ROOT / '.pixi/envs/default/lib'}:"
            + os.environ.get("LD_LIBRARY_PATH", "")
        )
    }
    process = subprocess.Popen(
        command,
        cwd=PROJECT_ROOT,
        env={**os.environ, **environment},
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sequence = 1
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline and process.poll() is None:
            sock.sendto(_packet(sequence, 0.3, -0.2), ("127.0.0.1", port))
            sequence += 1
            time.sleep(0.005)
    stdout, stderr = process.communicate(timeout=5.0)

    # The existing no-PICO headless harness returns 2 when its scripted stage
    # sequence has not observed the final reset stage before the duration.  The
    # hand integration assertions below are independent of that harness result.
    assert process.returncode in (0, 2), f"stdout={stdout}\nstderr={stderr}"
    assert "hand_accepted=" in stdout
    assert "hand_right_q0=" in stdout
    assert "hand_accepted=0" not in stdout
    assert "hand_left_q0=-0.2" in stdout
    assert "hand_right_q0=0.3" in stdout
    assert "control_failures=0" in stdout


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--viewer", type=Path, default=VIEWER)
    args = parser.parse_args()
    test_full_model_viewer_applies_bimanual_hand_packet_without_arm_failure(args.viewer)
