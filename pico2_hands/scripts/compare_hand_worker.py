"""Offline pinned Python/C++ Hand2 comparison; no sockets or devices."""
import json
from pathlib import Path
import struct
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OFFICIAL = ROOT / "third_party/wuji_hand_retargeting"
sys.path.insert(0, str(OFFICIAL / "example"))
from tj_wuji2_hand_bridge import OfficialWujiHand2Bridge

REQUEST = struct.Struct("<4sBBHQQII126d")
RESPONSE = struct.Struct("<4sBBHQQ40d")


def main():
    worker = ROOT.parent / "build/pico2-hand/tianji_hand_native_worker"
    maximum = 0.0
    for side, flag in (("left", 1), ("right", 2)):
        bridge = OfficialWujiHand2Bridge(
            OFFICIAL, single_hand_side=side,
            left_config=OFFICIAL / "example/config/adaptive_analytical_manus_wuji_hand_2_left.yaml",
            right_config=OFFICIAL / "example/config/adaptive_analytical_manus_wuji_hand_2_right.yaml")
        messages, expected = [], []
        # Include a sequence gap to verify original optimizer/filter reset.
        for seq in (1, 2, 3, 4, 100, 101, 102, 103):
            points = np.zeros((21, 3))
            for finger in range(5):
                for joint in range(4):
                    points[1 + finger * 4 + joint] = [
                        (finger - 2) * .02, .025 * (joint + 1),
                        .001 * (joint + 1) ** 2 * np.sin(seq * .1)]
            if side == "left":
                points[:, 0] *= -1
            values = points.flatten().tolist()
            timestamp = seq * 10_000_000
            result = bridge.retarget(values, seq, timestamp)
            expected.append(np.r_[result.left, result.right])
            messages.append(REQUEST.pack(b"TJWI", 1, flag, REQUEST.size, seq,
                                         timestamp, 63, 0, *(values + [0.] * 63)))
        child = subprocess.run([
            sys.executable, str(ROOT / "scripts/wuji_hand_native_launcher.py"),
            "--native-worker", str(worker), "--single-hand-side", side,
            "--startup-handshake"], input=b"".join(messages),
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=30)
        if child.returncode:
            raise RuntimeError(child.stderr.decode(errors="replace"))
        header, payload = child.stdout.split(b"\n", 1)
        if json.loads(header).get("kind") != "wuji_worker_ready":
            raise AssertionError("missing startup handshake")
        if len(payload) != len(expected) * RESPONSE.size:
            raise AssertionError("truncated/extra native worker response")
        for index, values in enumerate(expected):
            row = RESPONSE.unpack_from(payload, index * RESPONSE.size)
            seq = (1, 2, 3, 4, 100, 101, 102, 103)[index]
            if row[:6] != (b"TJHR", 1, flag, RESPONSE.size, seq, seq * 10_000_000):
                raise AssertionError("native response identity mismatch")
            error = float(np.max(np.abs(np.asarray(row[6:]) - values)))
            maximum = max(maximum, error)
            np.testing.assert_allclose(row[6:], values, rtol=0, atol=1e-6)
    print(json.dumps({"scope": "offline_synthetic_hand_worker_comparison",
                      "frames": 16, "max_joint_error_rad": maximum,
                      "hardware_acceptance_complete": False}))


if __name__ == "__main__":
    main()
