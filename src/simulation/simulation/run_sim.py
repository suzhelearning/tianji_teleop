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
    parser.add_argument("--hand-port", type=int, default=16000,
                        help="loopback hand input port (default: 16000)")
    hands = parser.add_mutually_exclusive_group()
    hands.add_argument("--hand-teleop", dest="hand_teleop", action="store_true", default=True,
                       help="receive independently started legacy TJH2 hand input")
    hands.add_argument("--no-hand-teleop", dest="hand_teleop", action="store_false",
                       help="arms only, do not bind the hand input port")
    parser.add_argument("--config", type=Path,
                        help="controller YAML, also used for the initial arm pose")
    parser.add_argument("--model", type=Path, help="dual-arm and both Hand2 MuJoCo XML")
    parser.add_argument("--sim-allow-pico-jumps", action="store_true",
                        help="simulation only: skip PICO pose jump rejection; retain freshness and motion limits")
    args = parser.parse_args(argv)
    if args.hand_teleop and not 1 <= args.hand_port <= 65535:
        parser.error("hand port must be in [1,65535]")
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("duration must be finite and non-negative")
    from .dls_session import launch
    return launch(args)


if __name__ == "__main__":
    raise SystemExit(main())
