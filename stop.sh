#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
python3 - <<'PY'
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

root = Path.cwd()


def stop_launcher(relative_path):
    target = root / relative_path
    handles = []
    try:
        for entry in Path("/proc").iterdir():
            if not entry.name.isdecimal():
                continue
            descriptor = None
            try:
                if entry.stat().st_uid != os.getuid():
                    continue
                descriptor = os.pidfd_open(int(entry.name))
                args = (entry / "cmdline").read_bytes().split(b"\0")
                if not args or not args[0]:
                    continue
                if Path(os.fsdecode(args[0])).name.startswith("python"):
                    if len(args) < 2 or not args[1]:
                        continue
                    script = Path(os.fsdecode(args[1]))
                    if not script.is_absolute():
                        script = (entry / "cwd").resolve() / script
                else:
                    if Path(os.fsdecode(args[0])).name != target.name:
                        continue
                    script = (entry / "exe").resolve()
                if script.resolve() != target:
                    continue
                signal.pidfd_send_signal(descriptor, signal.SIGTERM)
                print(f"Stopping {relative_path} (PID {entry.name})", flush=True)
                handles.append(descriptor)
                descriptor = None
            except (FileNotFoundError, ProcessLookupError):
                continue
            finally:
                if descriptor is not None:
                    os.close(descriptor)
        if not handles:
            print(f"Not running: {relative_path}", flush=True)
            return
        pending = set(handles)
        deadline = time.monotonic() + 15.0
        while pending:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError(
                    f"{relative_path}: shutdown timed out; verify physical emergency stop "
                    "and original terminal cleanup logs before restarting"
                )
            ready, _, _ = select.select(list(pending), [], [], remaining)
            pending.difference_update(ready)
        print(f"Exited: {relative_path}; check original terminal for cleanup warnings", flush=True)
    finally:
        for descriptor in handles:
            os.close(descriptor)


try:
    stop_launcher("real_robot/run_teleop.py")
    stop_launcher("sim/run_sim.py")
    stop_launcher("control/build/tianji_qp_ik_viewer")
    stop_launcher("manus/start_hand_teleop.py")
    subprocess.run(
        [str(root / "tracking/scripts/stop_tianji_pico_teleop.sh")],
        check=True,
    )
except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
    print(f"STOP FAILED: {error}; verify hardware is stopped before restarting", file=sys.stderr)
    sys.exit(1)
print("Teleop shutdown commands completed; verify hardware is physically stopped.", flush=True)
PY