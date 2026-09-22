#!/usr/bin/env python3
"""List the Manus calibration users this package can drive.

Read-only: inspects personnel `.mcal` profiles and imports neither ROS nor
the Manus SDK. A user counts only when both the left and the right profile are
present, because the collector loads a matched pair.
"""

from __future__ import annotations

from manus_bridge.paths import calibration_dir, profiles_root


def calibration_users() -> list[str]:
    """Names with a complete left/right profile pair, sorted."""
    directory = profiles_root()
    if not directory.is_dir():
        return []
    users = []
    for person in directory.iterdir():
        try:
            calibration_dir(person.name)
        except ValueError:
            continue
        users.append(person.name)
    return sorted(users)


def main() -> int:
    for user in calibration_users():
        print(user)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
