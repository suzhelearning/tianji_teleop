"""Explicit execution selection; no hardware imports before --real dispatch."""
from __future__ import annotations

import argparse
import sys


def main(argv: list[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(description=__doc__, add_help=False)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sim", action="store_true", help="run trajectory through native IK and MuJoCo")
    mode.add_argument("--real", action="store_true", help="guarded hardware mode; additionally requires --confirm-real")
    if not arguments or arguments in (["--help"], ["-h"]):
        parser.print_help()
        print("Simulation: run --sim --trajectory FILE [replay options].\n"
              "Hardware: run --real --help. Real devices are never opened by simulation or shadow inference.")
        return 0 if arguments else 2
    args, remaining = parser.parse_known_args(arguments)
    if args.real:
        from .real import main as real_main
        return real_main(remaining)
    trajectory_parser = argparse.ArgumentParser(description="Native IK / MuJoCo trajectory playback", add_help=False)
    trajectory_parser.add_argument("--trajectory", required=not any(x in remaining for x in ("--help", "-h")))
    selection, remaining = trajectory_parser.parse_known_args(remaining)
    from ..replay.cli import main as replay_main
    if selection.trajectory is None:
        return replay_main(["--help"])
    return replay_main(["replay", selection.trajectory, "--robot", *remaining])
