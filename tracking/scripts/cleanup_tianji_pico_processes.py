#!/usr/bin/env python3
"""Stop historical processes owned by the Tianji PICO teleoperation chain."""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time
from pathlib import Path
from typing import NamedTuple


TARGET_EXECUTABLES = frozenset(
    {
        "pico_bridge_node",
        "pico_smpl_ground",
        "pico_palm_tcp_publisher",
        "pico_palm_skeleton_filter",
        "smpl_mujoco_visualizer",
        "tianji_mujoco_teleop_bridge",
    }
)
TARGET_LAUNCH_FILES = frozenset(
    {
        "start_pico_bridge.launch.py",
        "start_pico_palm_skeleton_filter.launch.py",
        "start_tianji_mujoco_teleop.launch.py",
    }
)


class TargetProcess(NamedTuple):
    pid: int
    arguments: tuple[str, ...]


def _read_cmdline(process_dir: Path) -> tuple[str, ...]:
    raw = process_dir.joinpath("cmdline").read_bytes()
    return tuple(
        item.decode(errors="surrogateescape") for item in raw.split(b"\0") if item
    )


def _is_target(arguments: tuple[str, ...]) -> bool:
    basenames = tuple(Path(argument).name for argument in arguments)
    if basenames[0] in TARGET_EXECUTABLES:
        return True
    interpreter = basenames[0]
    if (
        interpreter.startswith("python")
        and len(basenames) > 1
        and basenames[1] in TARGET_EXECUTABLES
    ):
        return True

    ros2_index = 1 if interpreter.startswith("python") else 0
    if ros2_index + 3 < len(arguments) and basenames[ros2_index] == "ros2":
        if (
            arguments[ros2_index + 1] == "launch"
            and arguments[ros2_index + 2] == "pico_bridge"
            and Path(arguments[ros2_index + 3]).name in TARGET_LAUNCH_FILES
        ):
            return True
    return False


def find_target_processes(
    proc_root: Path = Path("/proc"), excluded_pids: set[int] | None = None
) -> list[TargetProcess]:
    excluded = excluded_pids or set()
    targets: list[TargetProcess] = []
    for process_dir in proc_root.glob("[0-9]*"):
        try:
            pid = int(process_dir.name)
            if pid in excluded:
                continue
            arguments = _read_cmdline(process_dir)
        except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError):
            continue
        if arguments and _is_target(arguments):
            targets.append(TargetProcess(pid=pid, arguments=arguments))
    return sorted(targets, key=lambda process: process.pid)


def _read_parent_pid(proc_root: Path, pid: int) -> int | None:
    try:
        stat = proc_root.joinpath(str(pid), "stat").read_text(encoding="utf-8")
        remainder = stat.rsplit(")", 1)[1].split()
        return int(remainder[1])
    except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError, IndexError):
        return None


def current_process_ancestry(proc_root: Path = Path("/proc")) -> set[int]:
    ancestry: set[int] = set()
    pid = os.getpid()
    while pid > 1 and pid not in ancestry:
        ancestry.add(pid)
        parent = _read_parent_pid(proc_root, pid)
        if parent is None:
            break
        pid = parent
    ancestry.add(1)
    return ancestry


def _wait_until_clear(proc_root: Path, excluded_pids: set[int], timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not find_target_processes(proc_root, excluded_pids):
            return True
        time.sleep(0.05)
    return not find_target_processes(proc_root, excluded_pids)


def cleanup_processes(
    proc_root: Path = Path("/proc"),
    dry_run: bool = False,
    interrupt_timeout_s: float = 2.0,
    terminate_timeout_s: float = 1.0,
    kill_timeout_s: float = 1.0,
) -> int:
    excluded_pids = current_process_ancestry(proc_root)
    targets = find_target_processes(proc_root, excluded_pids)
    if not targets:
        print("No historical Tianji PICO processes found")
        return 0

    for process in targets:
        print(f"historical pid={process.pid} command={process.arguments[0]}")
    if dry_run:
        return 0

    stages = (
        (signal.SIGINT, interrupt_timeout_s),
        (signal.SIGTERM, terminate_timeout_s),
        (signal.SIGKILL, kill_timeout_s),
    )
    for process_signal, timeout_s in stages:
        targets = find_target_processes(proc_root, excluded_pids)
        if not targets:
            return 0
        for process in targets:
            try:
                os.kill(process.pid, process_signal)
            except (ProcessLookupError, PermissionError):
                continue
        if _wait_until_clear(proc_root, excluded_pids, timeout_s):
            return 0

    survivors = find_target_processes(proc_root, excluded_pids)
    if survivors:
        print(
            "Failed to stop historical Tianji PICO pid(s): "
            + ",".join(str(process.pid) for process in survivors),
            file=sys.stderr,
        )
        return 2
    return 0


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--proc-root", type=Path, default=Path("/proc"))
    parser.add_argument("--dry-run", action="store_true")
    arguments = parser.parse_args()
    raise SystemExit(cleanup_processes(arguments.proc_root, arguments.dry_run))


if __name__ == "__main__":
    main()
