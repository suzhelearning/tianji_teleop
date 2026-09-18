"""One command surface for acquisition, calibration and offline dataset tools.

Commands replace this process so signals, terminal ownership and executor exit
codes retain the same semantics as the standalone programs. Importing the CLI
never loads a hardware SDK or opens a device.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

from .paths import DATASET, ROOT
COMMANDS = {
    "collect": "Record operator-triggered episodes with the real safety executor",
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
    "preview": "Preview the live collection stream",
    "home": "Return arms home (explicit --confirm-real required)",
    "mocap": "Replay mocap trajectories and run reference-conditioned RL policies",
}


def command_line(command: str, arguments: list[str]) -> tuple[list[str], Path]:
    """Build a process invocation without contacting hardware."""
    python = sys.executable
    modules = {
        "profile": "teleop_profile",
        "compress": "data_collection.compress",
        "visualize": "data_collection.visualize",
        "preview": "data_collection.live_preview",
        "mocap": "mocap_policy_runtime",
    }
    if command in modules:
        return [python, "-m", modules[command], *arguments], ROOT
    scripts = {
        "calibrate": "tracking/scripts/calibrate_pico_arm.sh",
        "pico": "pico.sh",
        "manus": "manus.sh",
        "exoskeleton": "exo.sh",
    }
    if command in scripts:
        return ["/bin/bash", str(ROOT / scripts[command]), *arguments], ROOT
    if command == "sim":
        return [python, str(ROOT / "sim/run_sim.py"), *arguments], ROOT
    if command == "view":
        return [str(ROOT / "control/build/tianji_qp_ik_viewer"),
                "--model", "models/marvin_m6_wuji2.xml", "--model-state-only",
                "--pico-teleop", "--hand-teleop", "--pico-port", "15000",
                "--hand-port", "16000", *arguments], ROOT / "control"
    if command in {"collect", "real", "home"}:
        script = "return_home.py" if command == "home" else "run_teleop.py"
        options = []
        if command == "collect":
            # The executor still requires a TTY, task, all devices, camera preflight
            # and physical operator confirmation before enabling anything.
            options = ["--devices", "all", "--confirm-real", "--dataset",
                       str(DATASET)]
        child = [python, str(ROOT / "real_robot" / script), *options, *arguments]
        if any(arg in {"-h", "--help"} for arg in arguments):
            return child, ROOT
        log_name = "home" if command == "home" else "real"
        return [python, str(ROOT / "real_robot/run_logging.py"), "--log-root",
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
