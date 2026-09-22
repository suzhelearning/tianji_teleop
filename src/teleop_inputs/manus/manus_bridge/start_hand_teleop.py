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

from tianji_runtime.resources import ResourceNotFound, package_share, vendor_path, workspace

from manus_bridge.list_calibrations import calibration_users
from manus_bridge.paths import calibration_dir
from manus_bridge.retarget_worker import worker_command, worker_environment


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
        for user in calibration_users():
            print(user)
        return 0
    if args.user is None:
        parser.error("choose a calibration user with --user (list them with --list-users)")
    try:
        calibrations = calibration_dir(args.user)
    except ValueError as error:
        parser.error(f"{error}; no Manus input started (list users with --list-users)")
    if not 1 <= args.port <= 65535:
        parser.error("--port must be in [1, 65535]")
    if not 0 <= args.ros_domain_id <= 232:
        parser.error("--ros-domain-id must be in [0, 232]")

    python = Path(sys.executable)
    sdk = vendor_path("manus_sdk", "lib", "libManusSDK_Integrated.so")
    # The installed collector resolves the vendored SDK through the dynamic loader.
    # Exporting its directory keeps `ldd` below and the collector itself on exactly
    # this SDK, whatever the install prefix, without a baked-in absolute rpath.
    os.environ["LD_LIBRARY_PATH"] = os.pathsep.join(
        [str(sdk.parent), *filter(None, [os.environ.get("LD_LIBRARY_PATH")])]
    )
    try:
        collector = package_share("manus_bridge", "rawviz")
    except ResourceNotFound as error:
        parser.error(f"{error}; run pixi run prepare-manus, then pixi run build-workspace")
    adapter = ROOT / "manus_hand_input.py"
    for required in (python, collector, adapter):
        if not required.is_file():
            parser.error(f"missing {required}; run pixi run prepare-manus, then pixi run build-workspace")

    # Fail before spawning the SDK collector if the Python/native path is incomplete.
    try:
        if not os.access(python, os.X_OK):
            raise ValueError(f'Python is not executable: {python}')
        check_collector(collector, sdk)
        import rclpy
        from std_msgs.msg import Float32MultiArray
        # ROS belongs to this default interpreter; native hand retargeting is
        # checked by the manus interpreter, never imported across environments.
        from example.tj_wuji2_hand_bridge import interpret_hand_input
        subprocess.run(worker_command(workspace(), "--check"), env=worker_environment(),
                       check=True, timeout=60)
    except (ImportError, OSError, ValueError, RuntimeError,
            subprocess.SubprocessError) as error:
        parser.error(f"Manus runtime check failed: {error}; run pixi run prepare-manus and pixi run build-workspace")
    if args.check:
        print("Manus runtime/models OK; no devices, ROS nodes or UDP started.")
        return 0

    environment = os.environ.copy()
    # ROS nodes stay in default. The bridge launches its private ROS-free
    # worker in manus, carrying only landmarks and receive timestamps by pipe.
    environment.update(
        ROS_DOMAIN_ID=str(args.ros_domain_id),
        EXO_REQUESTED_ROS_DOMAIN_ID=str(args.ros_domain_id),
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
            [str(python), "-m", "manus_bridge.ros_retarget_bridge",
             "--host", args.host, "--port", str(args.port), "--topic", args.topic],
            cwd=ROOT, env=environment, start_new_session=True,
        )
        processes.append(("Hand2 bridge", output))
        raw = subprocess.Popen(
            [str(collector), "--user", args.user, "--calibration-dir", str(calibrations)],
            cwd=ROOT, env=environment, stdout=subprocess.PIPE, start_new_session=True,
        )
        processes.append(("Manus collector", raw))
        input_node = subprocess.Popen(
            [str(python), "-m", "manus_bridge.manus_hand_input", "--topic", args.topic],
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
