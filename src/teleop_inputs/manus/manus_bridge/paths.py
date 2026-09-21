"""Locate this package's Manus calibration directory.

The ament install is authoritative: `share/manus_bridge/calibration` carries the
per-user `<user>LeftMetaglovePro.mcal` / `<user>RightMetaglovePro.mcal` profiles.
A source checkout that has not been built yet falls back to the `calibration/`
directory next to this package so listing and pre-checks keep working.
"""

from __future__ import annotations

from pathlib import Path

from tianji_runtime.resources import ResourceNotFound, package_share

SOURCE_CALIBRATION = Path(__file__).resolve().parent.parent / "calibration"


def calibration_dir() -> Path:
    """Directory holding the per-user glove calibration profiles."""
    try:
        return package_share("manus_bridge", "calibration")
    except ResourceNotFound:
        if not SOURCE_CALIBRATION.is_dir():
            raise
        return SOURCE_CALIBRATION
