"""Offline pinned Python/C++ Hand2 comparison; no sockets or devices."""
import argparse
import json
from pathlib import Path
import struct
import subprocess
import sys

import numpy as np

REQUEST = struct.Struct("<4sBBHQQII126d")
RESPONSE = struct.Struct("<4sBBHQQ40d")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--official-root", type=Path, required=True)
    parser.add_argument("--native-worker", type=Path, required=True)
    args = parser.parse_args()
    official = args.official_root.resolve(strict=True)
    sys.path.insert(0, str(official / "example"))
    from tj_wuji2_hand_bridge import OfficialWujiHand2Bridge

    worker = args.native_worker.resolve(strict=True)
    maximum = 0.0
    for side, flag in (("left", 1), ("right", 2)):
        bridge = OfficialWujiHand2Bridge(
            official, single_hand_side=side,
            left_config=official / "example/config/adaptive_analytical_manus_wuji_hand_2_left.yaml",
            right_config=official / "example/config/adaptive_analytical_manus_wuji_hand_2_right.yaml")
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
            sys.executable, str(Path(__file__).with_name("wuji_hand_native_launcher.py")),
            "--official-root", str(official),
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
