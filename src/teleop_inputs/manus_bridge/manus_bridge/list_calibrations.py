#!/usr/bin/env python3
"""List the Manus calibration users this package can drive.

Read-only: inspects the installed `.mcal` profiles and imports neither ROS nor
the Manus SDK. A user counts only when both the left and the right profile are
present, because the collector loads a matched pair.
"""

from __future__ import annotations

from manus_bridge.paths import calibration_dir

LEFT = "LeftMetaglovePro.mcal"
RIGHT = "RightMetaglovePro.mcal"


def calibration_users() -> list[str]:
    """Names with a complete left/right profile pair, sorted."""
    directory = calibration_dir()
    return sorted(
        path.name[: -len(LEFT)]
        for path in directory.glob(f"*{LEFT}")
        if (directory / f"{path.name[: -len(LEFT)]}{RIGHT}").is_file()
    )


def main() -> int:
    for user in calibration_users():
        print(user)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
