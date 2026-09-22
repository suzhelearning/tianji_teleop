#!/usr/bin/env bash
# Stop the teleoperation programs owned by this checkout.
#
# Only processes whose executable or resolved script path is inside this
# workspace are signalled; unrelated programs on the same ports are left alone.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"

# Ubuntu's system Python exposes pidfd APIs; some conda Python builds do not.
# This stdlib-only owner check must not depend on a ROS/native Python ABI.
exec /usr/bin/python3 - "$root" <<'PY'
from __future__ import annotations

import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

root = Path(sys.argv[1])

TARGETS = {
    "tianji_controller.run_teleop": "src/teleop_outputs/tianji/tianji_controller/tianji_controller/run_teleop.py",
    "simulation.run_sim": "src/simulation/simulation/run_sim.py",
    "manus_adapter": "src/teleop_inputs/manus/scripts/manus_adapter",
    "manus_hand2_retarget": "src/teleop_inputs/manus/scripts/manus_hand2_retarget",
}
# Native viewers are matched by executable name inside the workspace install tree.
TARGET_NAMES = {
    "tianji_qp_ik_viewer": "install/control/bin/tianji_qp_ik_viewer",
    "mapped_palm_native_worker": "install/control/lib/mapped_palm/mapped_palm_native_worker",
    "manus_data_publisher": "install/default/manus_bridge/lib/manus_bridge/manus_data_publisher",
}


def _matches(entry: Path, target_name: str) -> bool:
    """Match a specific entrypoint and its checkout, never just a Python process."""
    try:
        args = (entry / "cmdline").read_bytes().split(b"\0")
    except (FileNotFoundError, PermissionError):
        return False
    if not args or not args[0]:
        return False
    argv0 = os.fsdecode(args[0])
    base = Path(argv0).name
    try:
        if base.startswith("python") and target_name in TARGETS:
            if len(args) < 2 or not args[1]:
                return False
            if args[1] == b"-m":
                if len(args) < 3 or os.fsdecode(args[2]) != target_name:
                    return False
                environment = dict(
                    item.split(b"=", 1) for item in (entry / "environ").read_bytes().split(b"\0")
                    if b"=" in item
                )
                checkout = environment.get(b"TIANJI_WORKSPACE")
                return (
                    checkout is not None
                    and Path(os.fsdecode(checkout)).resolve() == root
                    and (entry / "exe").resolve().is_relative_to(root / ".pixi")
                )
            script = Path(os.fsdecode(args[1]))
            if not script.is_absolute():
                script = (entry / "cwd").resolve() / script
            return script.resolve() == (root / TARGETS[target_name]).resolve()
        if base == target_name and target_name in TARGET_NAMES:
            return (entry / "exe").resolve() == (root / TARGET_NAMES[target_name]).resolve()
    except (OSError, ValueError):
        return False
    return False


def stop(target_name: str) -> None:
    handles: list[int] = []
    try:
        for entry in Path("/proc").iterdir():
            if not entry.name.isdecimal():
                continue
            descriptor = None
            try:
                if entry.stat().st_uid != os.getuid():
                    continue
                descriptor = os.pidfd_open(int(entry.name))
                if not _matches(entry, target_name):
                    continue
                signal.pidfd_send_signal(descriptor, signal.SIGTERM)
                print(f"Stopping {target_name} (PID {entry.name})", flush=True)
                handles.append(descriptor)
                descriptor = None
            except (FileNotFoundError, ProcessLookupError, PermissionError):
                continue
            finally:
                if descriptor is not None:
                    os.close(descriptor)
        if not handles:
            print(f"Not running: {target_name}", flush=True)
            return
        pending = set(handles)
        deadline = time.monotonic() + 15.0
        while pending:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError(
                    f"{target_name}: shutdown timed out; verify physical emergency stop "
                    "and the original terminal logs before restarting"
                )
            ready, _, _ = select.select(list(pending), [], [], remaining)
            pending.difference_update(ready)
        print(f"Exited: {target_name}; check the original terminal for cleanup warnings", flush=True)
    finally:
        for descriptor in handles:
            os.close(descriptor)


def main() -> int:
    try:
        for module in TARGETS:
            stop(module)
        for name in TARGET_NAMES:
            stop(name)
        subprocess.run(
            [str(root / "bash" / "run_stop_pico.sh")], check=True, cwd=str(root)
        )
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"STOP FAILED: {error}; verify hardware is stopped before restarting", file=sys.stderr)
        return 1
    print("Teleop shutdown commands completed; verify hardware is physically stopped.", flush=True)
    return 0


raise SystemExit(main())
PY
