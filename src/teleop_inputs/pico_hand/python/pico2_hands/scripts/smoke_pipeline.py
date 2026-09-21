"""Offline raw PICO -> shared-root C -> DLS/Ruckig + native Hand2 smoke.

Uses the production simulation owner and recorder; never contacts a device.
Pass --height-m explicitly and optionally --record for the complete session.
"""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from pico2_hands.run_sim import main as run_sim


def main(argv=None):
    return run_sim(["--self-test", "--headless", *(sys.argv[1:] if argv is None else argv)])


if __name__ == "__main__":
    raise SystemExit(main())
