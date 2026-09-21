"""Private request/reply worker, executed by Manus Python with -I (no ROS)."""
from __future__ import annotations

import json
import os
import sys


def main() -> int:
    # Keep the protocol on its own descriptor: even native solver stdout belongs
    # on inherited stderr, never in the bounded request/reply pipe.
    with os.fdopen(os.dup(sys.stdout.fileno()), "wb", buffering=0) as output:
        os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
        side = sys.argv[1] if len(sys.argv) == 2 else None
        sequence = 0

        def reply(kind, **payload):
            message = dict(kind=kind, sequence=sequence, side=side, **payload)
            output.write((json.dumps(message, allow_nan=False, separators=(",", ":")) + "\n").encode("ascii"))

        def request():
            raw = sys.stdin.buffer.readline(8193)
            if not raw:
                return None
            if len(raw) > 8192 or not raw.endswith(b"\n"):
                raise ValueError("hand request exceeds bound or is truncated")
            value = json.loads(raw)
            if not isinstance(value, dict) or value.get("sequence") != sequence:
                raise ValueError("hand request sequence mismatch")
            return value

        try:
            if side not in ("left", "right") or not sys.flags.isolated:
                raise ValueError("hand worker requires isolated Python and a left/right side")
            root = os.environ["TIANJI_WORKSPACE"]
            expected = os.path.realpath(os.path.join(root, ".pixi", "envs", "manus"))
            if os.path.realpath(sys.prefix) != expected:
                raise RuntimeError("hand worker must use the workspace Manus interpreter")
            initial = request()
            if initial is None:
                return 0
            if initial.get("kind") != "init" or initial.get("side") != side:
                raise ValueError("hand worker init identity mismatch")
            import numpy as np
            from example.tj_wuji2_hand_bridge import HandRetargeter

            # Preserve the official model, optimizer history, joint permutation,
            # coordinate calibration, limits and filter in this one interpreter.
            solver = HandRetargeter(side)
            reply("ready")
            while True:
                sequence += 1
                value = request()
                if value is None:
                    return 0
                if value.get("kind") != "retarget":
                    raise ValueError("unknown hand worker request")
                points = np.asarray(value.get("points"), dtype=np.float64)
                if points.shape != (21, 3) or not np.isfinite(points).all():
                    raise ValueError("hand keypoints must be finite 21x3")
                joints = np.asarray(solver.retarget(points), dtype=np.float64)
                if joints.shape != (20,) or not np.isfinite(joints).all():
                    raise ValueError("hand solver returned invalid joint angles")
                reply("joints", joints=joints.tolist())
        except Exception as error:
            reply("error", error=f"{type(error).__name__}: {error}"[:2048])
            return 1


if __name__ == "__main__":
    raise SystemExit(main())
