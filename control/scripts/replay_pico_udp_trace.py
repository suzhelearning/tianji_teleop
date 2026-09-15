#!/usr/bin/env python3
"""Replay a recorded TJVT trace to the simulator without rewriting TJVR packets."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import socket
import time

from run_pico_trace_algorithm_benchmark import _read_trace


def replay_trace(path: Path, host: str, port: int, lead: float) -> int:
    if not math.isfinite(lead) or lead < 0.0:
        raise ValueError("lead must be finite and nonnegative")
    if not 1 <= port <= 65535:
        raise ValueError("port must be in [1, 65535]")
    _, records = _read_trace(path)
    if not records:
        raise ValueError("empty TJVR trace")
    previous_ns = 0
    for relative_ns, _, _ in records:
        if relative_ns < previous_ns:
            raise ValueError("non-monotonic TJVR receive timestamps")
        previous_ns = relative_ns
    # Retain the recorder's receive-time spacing, including source-clock resets.
    # The original bridge timestamp and CRC are deliberately untouched: recorded
    # input must not masquerade as a fresh live bridge for physical commands.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        sender.connect((host, port))
        start_ns = time.monotonic_ns() + round(lead * 1_000_000_000)
        for relative_ns, packet, _ in records:
            deadline_ns = start_ns + relative_ns
            remaining_ns = deadline_ns - time.monotonic_ns()
            while remaining_ns > 0:
                time.sleep(remaining_ns * 1.0e-9)
                remaining_ns = deadline_ns - time.monotonic_ns()
            sender.send(packet)
    return len(records)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=15000)
    parser.add_argument("--lead", type=float, default=0.5)
    args = parser.parse_args()
    count = replay_trace(args.input, args.host, args.port, args.lead)
    print(f"replayed_frames={count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
