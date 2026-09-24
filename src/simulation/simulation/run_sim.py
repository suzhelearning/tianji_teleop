#!/usr/bin/env python3
"""MuJoCo simulation without hardware: Franka DLS + Ruckig with optional hand input.

The direct viewer uses in-process IK/Ruckig with S/H/P recovery.
Missing or stale input holds the last setpoint. No joint commands are exported.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--user", help="start/reuse this person's published ROS pico-simple input")
    parser.add_argument("--simulation-mode", choices=("direct",), default="direct",
                        help="direct joint-state viewer (no actuator physics)")
    parser.add_argument("--ik-backend", choices=("franka-dls",), default="franka-dls",
                        help="Franka DLS with Ruckig arm smoothing")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="stop after N wall-clock seconds (0: until stopped)")
    parser.add_argument("--hand-source", choices=("manus", "exoskeleton"), default="manus",
                        help="Manus ROS hand commands (default), or explicit legacy TJH2 exoskeleton input")
    parser.add_argument("--hand-port", type=int,
                        help="exoskeleton loopback UDP input port (default: 16000)")
    hands = parser.add_mutually_exclusive_group()
    hands.add_argument("--hand-teleop", dest="hand_teleop", action="store_true", default=True,
                       help="receive independently started input from the selected hand source")
    hands.add_argument("--no-hand-teleop", dest="hand_teleop", action="store_false",
                       help="arms only, do not subscribe to hand topics or bind a hand input port")
    parser.add_argument("--config", type=Path,
                        help="controller YAML, also used for the initial arm pose")
    parser.add_argument("--model", type=Path, help="dual-arm and both Hand2 MuJoCo XML")
    parser.add_argument("--sim-allow-pico-jumps", action="store_true",
                        help="simulation only: skip PICO pose jump rejection; retain freshness and motion limits")
    args = parser.parse_args(argv)
    if args.hand_port is not None:
        if args.hand_source != "exoskeleton":
            parser.error("--hand-port requires --hand-source exoskeleton")
        if not 1 <= args.hand_port <= 65535:
            parser.error("hand port must be in [1,65535]")
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("duration must be finite and non-negative")
    from .dls_session import launch
    return launch(args)


if __name__ == "__main__":
    raise SystemExit(main())
