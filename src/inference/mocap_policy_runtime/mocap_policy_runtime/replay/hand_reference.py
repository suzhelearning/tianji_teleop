"""Regrind reference hand/object visualization, never policy inference or control."""
from __future__ import annotations

import argparse
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--start-time", type=float, default=0.0)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-realtime", action="store_true")
    parser.add_argument("--snapshot", type=Path)
    parser.add_argument("--object-mesh", type=Path)
    args = parser.parse_args(argv)
    from .cli import main as replay_main

    options = ["replay", str(args.reference), "--format", "regrind", "--reference-only",
               "--speed", str(args.speed), "--duration", str(args.duration),
               "--start-time", str(args.start_time)]
    for flag in ("loop", "headless", "no_realtime"):
        if getattr(args, flag):
            options.append("--" + flag.replace("_", "-"))
    for flag in ("snapshot", "object_mesh"):
        value = getattr(args, flag)
        if value is not None:
            options.extend(("--" + flag.replace("_", "-"), str(value)))
    return replay_main(options)
