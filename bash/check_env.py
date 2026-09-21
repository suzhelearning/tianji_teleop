#!/usr/bin/env python3
"""Verify the running environment is the one this workspace expects.

Run as `pixi run check-env`. Contacts no devices and starts no nodes.
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

EXPECTED_ROS_DISTRO = "jazzy"
EXPECTED_RMW = "rmw_fastrtps_cpp"
EXPECTED_PYTHON = (3, 12)


class CheckError(RuntimeError):
    pass


def check_python() -> str:
    if sys.version_info[:2] != EXPECTED_PYTHON:
        raise CheckError(
            f"expected Python {EXPECTED_PYTHON[0]}.{EXPECTED_PYTHON[1]}, "
            f"running {sys.version_info.major}.{sys.version_info.minor}"
        )
    return f"python {sys.version.split()[0]}"


def check_ros() -> str:
    distro = os.environ.get("ROS_DISTRO")
    if distro != EXPECTED_ROS_DISTRO:
        raise CheckError(f"ROS_DISTRO is {distro!r}, expected {EXPECTED_ROS_DISTRO!r}")
    try:
        from rclpy.utilities import get_rmw_implementation_identifier
    except ImportError as error:  # pragma: no cover - environment probe
        raise CheckError(f"rclpy is not importable: {error}") from error
    rmw = get_rmw_implementation_identifier()
    if rmw != EXPECTED_RMW:
        raise CheckError(f"RMW implementation is {rmw!r}, expected {EXPECTED_RMW!r}")
    return f"ROS {distro} / {rmw}"


def check_domain() -> str:
    domain = os.environ.get("ROS_DOMAIN_ID")
    if not domain:
        raise CheckError("ROS_DOMAIN_ID is unset")
    if os.environ.get("ROS_LOCALHOST_ONLY"):
        raise CheckError(
            "ROS_LOCALHOST_ONLY is set; it conflicts with Jazzy's "
            "ROS_AUTOMATIC_DISCOVERY_RANGE and must be cleared"
        )
    return f"domain {domain}, discovery {os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE', '-')}"


def check_no_humble() -> str:
    """No path from a different ROS distribution may leak into this one."""
    offenders: list[str] = []
    for variable in ("AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "PYTHONPATH",
                     "COLCON_PREFIX_PATH", "LD_LIBRARY_PATH"):
        for entry in os.environ.get(variable, "").split(os.pathsep):
            if not entry:
                continue
            lowered = entry.lower()
            if "humble" in lowered or "tracking/install" in lowered:
                offenders.append(f"{variable}={entry}")
    if offenders:
        raise CheckError("foreign ROS paths present: " + "; ".join(offenders))
    return "no foreign ROS/overlay paths"


def check_workspace() -> str:
    workspace = os.environ.get("TIANJI_WORKSPACE")
    if not workspace:
        raise CheckError("TIANJI_WORKSPACE is unset")
    for relative in ("pixi.toml", "config/collect_real.json", "config/robot.json"):
        if not (Path(workspace) / relative).exists():
            raise CheckError(f"workspace is missing {relative}")
    return f"workspace {workspace}"


def check_imports() -> str:
    """Import what the workspace actually needs, reporting every failure."""
    failures: list[str] = []
    for module in ("numpy", "h5py", "yaml", "cv2", "mujoco", "pyrealsense2"):
        try:
            __import__(module)
        except ImportError as error:
            failures.append(f"{module}: {error}")
    if failures:
        raise CheckError("missing modules: " + "; ".join(failures))
    return "numpy, h5py, yaml, cv2, mujoco, pyrealsense2"


def check_tools() -> str:
    missing = [tool for tool in ("colcon", "cmake") if not shutil.which(tool)]
    if missing:
        raise CheckError("missing tools: " + ", ".join(missing))
    if shutil.which("cmake"):
        version = subprocess.run(
            ["cmake", "--version"], capture_output=True, text=True, check=False
        ).stdout.splitlines()[0]
        return f"colcon, {version}"
    return "colcon"


CHECKS = (
    ("python", check_python),
    ("ros", check_ros),
    ("domain", check_domain),
    ("isolation", check_no_humble),
    ("workspace", check_workspace),
    ("imports", check_imports),
    ("tools", check_tools),
)


def main() -> int:
    failed = 0
    for name, check in CHECKS:
        try:
            detail = check()
        except CheckError as error:
            print(f"FAIL {name}: {error}")
            failed += 1
        else:
            print(f"ok   {name}: {detail}")
    if failed:
        print(f"\n{failed} check(s) failed.", file=sys.stderr)
        return 1
    print("\nEnvironment verified. No devices were contacted.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
