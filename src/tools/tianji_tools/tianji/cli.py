"""One command surface for acquisition, calibration and offline dataset tools.

Commands replace this process so signals, terminal ownership and executor exit
codes keep the same semantics as the standalone programs. Importing the CLI
never loads a hardware SDK and never opens a device.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

from tianji_runtime import native_executable, package_share
from .paths import ROOT

COMMANDS = {
    "collect": "Run the independent observation-only ROS collector",
    "real": "Run the real executor (explicit --confirm-real required)",
    "sim": "Run actuator-driven simulation without hardware",
    "view": "View reference motion without hardware",
    "profile": "List, resolve or calibrate a named operator profile",
    "calibrate": "Run the project-owned PICO calibration workflow",
    "pico": "Start PICO input for an explicitly selected operator",
    "manus": "Start Manus input for an explicitly selected operator",
    "exoskeleton": "Start both gloves and send local TJH2 (use --hand left/right for one side)",
    "compress": "Compress recorded datasets offline",
    "visualize": "Browse recorded datasets",
    "preview": "Preview the three RGB camera streams without recording",
    "cameras": "Start the official RealSense ROS driver",
    "inspect-cameras": "Enumerate RealSense devices and validate the configured profiles",
    "home": "Return arms home (explicit --confirm-real required)",
    "mocap": "Replay mocap trajectories and run reference-conditioned RL policies",
}

# Modules reachable as `python -m <module>` in this environment.
MODULE_COMMANDS = {
    "profile": "teleop_profile",
    "compress": "data_collector.compress",
    "visualize": "data_collector.visualize",
    "preview": "tianji_cameras.preview",
    "inspect-cameras": "tianji_cameras.preflight",
    "sim": "simulation.run_sim",
}

# Shell entries under bash/.
SCRIPT_COMMANDS = {
    "calibrate": "run_setup_pico.sh",
    "pico": "run_pico.sh",
    "manus": "run_manus.sh",
    "exoskeleton": "run_exoskeleton.sh",
    "cameras": "run_cameras.sh",
    "collect": "run_data_collector.sh",
    "mocap": "run_mocap.sh",
}


def command_line(command: str, arguments: list[str]) -> tuple[list[str], Path]:
    """Build a process invocation without contacting hardware."""
    python = sys.executable
    if command in MODULE_COMMANDS:
        return [python, "-m", MODULE_COMMANDS[command], *arguments], ROOT
    if command in SCRIPT_COMMANDS:
        return (["/bin/bash", str(ROOT / "bash" / SCRIPT_COMMANDS[command]), *arguments],
                ROOT)
    if command == "view":
        # Directed-interaction viewer: the native binary owns the terminal.
        return [str(native_executable("tianji_qp_ik_viewer")),
                "--model", str(package_share("tianji_description", "models", "marvin_m6_wuji2.xml")),
                "--model-state-only", "--pico-teleop", "--hand-teleop",
                "--pico-port", "15000", "--hand-port", "16000", *arguments], ROOT
    if command in {"real", "home"}:
        module = ("tianji_controller.return_home" if command == "home"
                  else "tianji_controller.run_teleop")
        child = [python, "-m", module, *arguments]
        if any(argument in {"-h", "--help"} for argument in arguments):
            return child, ROOT
        log_name = "home" if command == "home" else "real"
        # Absolute: the logged child may change its own working directory, and a
        # relative log root would then land somewhere else entirely.
        return [python, "-m", "tianji_controller.run_logging", "--log-root",
                str(ROOT / "logs" / log_name), "--", *child], ROOT
    raise ValueError(f"unknown command: {command}")


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(
        prog="tianji", description=__doc__.split("\n\n", 1)[0],
        epilog="Use tianji COMMAND --help for command-specific arguments.")
    parser.add_argument("command", choices=COMMANDS, help="; ".join(
        f"{name}: {description}" for name, description in COMMANDS.items()))
    # Forward child flags unchanged, including --help, rather than parsing twice.
    if not args or args[0] in {"-h", "--help"}:
        parser.print_help()
        return 0 if args else 2
    command = parser.parse_args(args[:1]).command
    invocation, cwd = command_line(command, args[1:])
    try:
        os.chdir(cwd)
        os.execv(invocation[0], invocation)
    except OSError as error:
        print(f"Cannot start {command}: {error}", file=sys.stderr)
        return 1
    return 0
