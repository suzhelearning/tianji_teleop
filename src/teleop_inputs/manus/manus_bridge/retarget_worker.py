"""ROS-free Manus Hand2 worker; private pipe input, original TJH2 UDP output.

Only this process imports the Pinocchio/NLopt retargeter. The pipe carries the
ROS callback's monotonic receive time unchanged, not a refreshed worker time.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import select
import socket
import struct
import sys
import threading
import time


# One side plus callback time and 21 XYZ landmarks. Smaller than PIPE_BUF, so
# nonblocking writes are atomic: a busy solver drops a whole input, never half.
FRAME = struct.Struct("<BQ63d")
SIDES = ("left", "right")
READY = b"MANUS_RETARGET_READY\n"


def worker_command(root: Path, *arguments: str) -> list[str]:
    return ["pixi", "run", "--locked", "--manifest-path", str(root / "pixi.toml"),
            "-e", "manus", "python", "-I", "-m", "manus_bridge.retarget_worker",
            *arguments]


def worker_environment() -> dict[str, str]:
    """Cross the process boundary without borrowing another environment's ABI."""
    environment = os.environ.copy()
    for key in ("PYTHONPATH", "PYTHONHOME", "LD_LIBRARY_PATH", "LD_PRELOAD",
                "AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "CMAKE_PREFIX_PATH",
                "TIANJI_PYTHON", "ROS_DISTRO", "ROS_VERSION"):
        environment.pop(key, None)
    environment["TIANJI_ENVIRONMENT"] = "manus"
    environment["PYTHONUNBUFFERED"] = "1"
    return environment


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=16000)
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("--port must be in [1, 65535]")
    if os.environ.get("PIXI_ENVIRONMENT_NAME") != "manus":
        parser.error("run this worker with pixi run --locked -e manus")

    import numpy as np
    from wuji_retargeting import _native
    from example.tj_wuji2_hand_bridge import (
        HandRetargeter, HandTargetCache, publish_targets,
    )

    # Exactly the existing independent optimizers, transforms, limits, named
    # joint permutation and filters; no alternate model or algorithm fallback.
    models = {side: HandRetargeter(side) for side in SIDES}
    if args.check:
        print("Manus native retargeter and both Hand2 models OK (no ROS or UDP).")
        return 0

    address = (socket.gethostbyname(args.host), args.port)
    cache = HandTargetCache()
    stop = threading.Event()
    publisher_errors: list[BaseException] = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
        udp.setblocking(False)

        def publish():
            try:
                publish_targets(cache, udp, address, stop)
            except BaseException as error:
                publisher_errors.append(error)
                stop.set()

        publisher = threading.Thread(target=publish, name="tjh2-publisher")
        publisher.start()
        try:
            sys.stdout.buffer.write(READY)
            sys.stdout.buffer.flush()
            # The pipe is only a startup handshake. Solver diagnostics belong
            # on stderr once the parent has consumed the ready record.
            os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
            pending = bytearray()
            while not stop.is_set():
                readable, _, _ = select.select([sys.stdin.fileno()], [], [], 0.1)
                if not readable:
                    continue
                chunk = os.read(sys.stdin.fileno(), FRAME.size * 16)
                if not chunk:
                    if pending:
                        raise RuntimeError("truncated Manus retarget pipe frame")
                    break
                pending.extend(chunk)
                offset = 0
                while len(pending) - offset >= FRAME.size:
                    side_index, received_ns, *values = FRAME.unpack_from(pending, offset)
                    offset += FRAME.size
                    if side_index >= len(SIDES) or not 0 < received_ns <= time.monotonic_ns():
                        raise RuntimeError("invalid Manus retarget pipe identity/time")
                    side = SIDES[side_index]
                    points = np.asarray(values, dtype=np.float64).reshape(21, 3)
                    try:
                        cache.retarget(side, models[side], points, received_ns)
                    except (ValueError, RuntimeError, np.linalg.LinAlgError) as error:
                        print(f"Rejected {side} solve: {error}", file=sys.stderr)
                del pending[:offset]
        except KeyboardInterrupt:
            pass
        finally:
            stop.set()
            publisher.join()
        if publisher_errors:
            raise RuntimeError("TJH2 publisher failed") from publisher_errors[0]
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
