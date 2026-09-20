#!/usr/bin/env python3
"""Run the local Manus -> ROS 2 -> TJH2 simulation input pipeline."""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parent


def check_collector(collector, sdk):
    """Inspect trusted local ELF files and loader dependencies; never run SDK main."""
    if not os.access(collector, os.X_OK):
        raise ValueError(f"collector is not executable: {collector}")
    for path in (collector, sdk):
        with path.open('rb') as stream:
            if stream.read(4) != b'\x7fELF':
                raise ValueError(f"not an ELF binary (possibly Git LFS pointer): {path}")
    try:
        result = subprocess.run(['ldd', str(collector)], capture_output=True, text=True,
                                timeout=5, env={**os.environ, 'LC_ALL': 'C'})
    except subprocess.TimeoutExpired as error:
        raise RuntimeError('collector dependency check timed out') from error
    details = result.stdout + result.stderr
    if result.returncode or 'not found' in details:
        raise RuntimeError(f'collector dynamic dependencies unavailable: {details.strip()}')
    # Ensure the intended bundled SDK was resolved, not a different installed SDK.
    matches = [line.split('=>', 1)[1].strip().split(' (', 1)[0]
               for line in result.stdout.splitlines()
               if line.strip().startswith('libManusSDK_Integrated.so =>')]
    if len(matches) != 1 or Path(matches[0]).resolve() != sdk.resolve():
        raise RuntimeError('collector does not resolve the bundled Manus SDK; rebuild with pixi run prepare-manus')


def calibration_users() -> list[str]:
    suffix = "LeftMetaglovePro.mcal"
    return sorted(
        path.name[:-len(suffix)]
        for path in (ROOT / "calibration").glob(f"*{suffix}")
        if (ROOT / "calibration" / f"{path.name[:-len(suffix)]}RightMetaglovePro.mcal").is_file()
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--user", help="Manus calibration user (see --list-users)")
    parser.add_argument("--list-users", action="store_true")
    parser.add_argument("--check", action="store_true", help="validate runtime and models without starting devices, ROS nodes or UDP")
    parser.add_argument("--host", default="127.0.0.1", help="MuJoCo hand UDP receiver")
    parser.add_argument("--port", type=int, default=16000)
    parser.add_argument("--ros-domain-id", type=int, default=120)
    parser.add_argument("--topic", default="/hand_input")
    args = parser.parse_args()
    if args.list_users:
        print("\n".join(calibration_users()))
        return 0
    if args.user not in calibration_users():
        parser.error("choose an existing calibration user with --user; list them with --list-users")
    if not 1 <= args.port <= 65535:
        parser.error("--port must be in [1, 65535]")
    if not 0 <= args.ros_domain_id <= 232:
        parser.error("--ros-domain-id must be in [0, 232]")

    hand_root = ROOT.parent / "retargeting"
    python = Path(os.environ.get("TIANJI_PYTHON", sys.executable))
    collector = ROOT / "build/manus_raw"
    adapter = ROOT / "manus_hand_input.py"
    bridge = hand_root / "example/tj_wuji2_hand_bridge.py"
    for required in (python, collector, adapter, bridge):
        if not required.is_file():
            parser.error(f"missing {required}; run pixi run prepare-manus (or provision the explicit legacy environment)")

    # Fail before spawning the SDK collector if the Python/native path is incomplete.
    try:
        if not os.access(python, os.X_OK):
            raise ValueError(f'Python is not executable: {python}')
        check_collector(collector, ROOT / 'ManusSDK/lib/libManusSDK_Integrated.so')
        import rclpy
        from std_msgs.msg import Float32MultiArray
        from retargeting.wuji_retargeting import _native
        from retargeting.example.tj_wuji2_hand_bridge import HandRetargeter
        if args.check:
            for side in ("left", "right"):
                HandRetargeter(hand_root, side)
    except (ImportError, OSError, ValueError, RuntimeError) as error:
        parser.error(f"Manus runtime check failed: {error}; run pixi run prepare-manus")
    if args.check:
        print("Manus runtime/models OK; no devices, ROS nodes or UDP started.")
        return 0

    environment = os.environ.copy()
    # Keep the sourced ROS 2 runtime environment; all Python project code comes
    # from the root editable installation, never a separate hand environment.
    environment.pop("PYTHONHOME", None)
    environment.update(
        ROS_DOMAIN_ID=str(args.ros_domain_id),
        EXO_REQUESTED_ROS_DOMAIN_ID=str(args.ros_domain_id),
        ROS_LOCALHOST_ONLY="1",
        ROS2CLI_DISABLE_DAEMON="1",
        PYTHONUNBUFFERED="1",
    )
    processes: list[tuple[str, subprocess.Popen]] = []
    stopped = False

    def request_stop(signum, frame):
        nonlocal stopped
        stopped = True

    old_handlers = {sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        print(f"Manus user={args.user}; ROS domain={args.ros_domain_id}; hand UDP={args.host}:{args.port}", flush=True)
        output = subprocess.Popen(
            [str(python), "-m", "retargeting.example.tj_wuji2_hand_bridge", "--repository-root", str(hand_root),
             "--host", args.host, "--port", str(args.port), "--topic", args.topic],
            cwd=ROOT, env=environment, start_new_session=True,
        )
        processes.append(("Hand2 bridge", output))
        raw = subprocess.Popen(
            [str(collector), "--user", args.user, "--calibration-dir", str(ROOT / "calibration")],
            cwd=ROOT, env=environment, stdout=subprocess.PIPE, start_new_session=True,
        )
        processes.append(("Manus collector", raw))
        input_node = subprocess.Popen(
            [str(python), "-m", "manus.manus_hand_input", "--topic", args.topic],
            cwd=ROOT, env=environment, stdin=raw.stdout, start_new_session=True,
        )
        processes.append(("Manus ROS input", input_node))
        raw.stdout.close()
        while not stopped:
            for label, process in processes:
                if process.poll() is not None:
                    print(f"{label} exited ({process.returncode}); stopping hand pipeline", file=sys.stderr)
                    return process.returncode if process.returncode and process.returncode > 0 else 1
            time.sleep(0.1)
        return 0
    finally:
        for _, process in reversed(processes):
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + 5.0
        for _, process in processes:
            try:
                process.wait(timeout=max(0.0, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                pass
            # A pipeline child can outlive its process-group leader.
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)


if __name__ == "__main__":
    raise SystemExit(main())
