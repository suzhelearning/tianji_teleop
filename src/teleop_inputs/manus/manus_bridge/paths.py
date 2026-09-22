"""Locate and validate Manus calibration pairs in personnel storage."""

from __future__ import annotations

from pathlib import Path
import re

from tianji_runtime.resources import workspace

_USER_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]*")


def profiles_root() -> Path:
    """Workspace personnel storage, independent of package installation."""
    return workspace() / "profiles"


def calibration_dir(user: str) -> Path:
    """Return a complete calibration pair's directory, contained in this person."""
    if not isinstance(user, str) or _USER_NAME.fullmatch(user) is None:
        raise ValueError("user must be a single name using letters, digits, '_' or '-'")
    profiles = profiles_root()
    person = profiles / user
    directory = person / "manus"
    try:
        for path in (profiles, person, directory):
            resolved = path.resolve(strict=True)
            if not resolved.is_relative_to(path):
                raise ValueError(f"path escapes selected Manus calibration: {path}")
            if not resolved.is_dir():
                raise ValueError(f"expected Manus calibration directory: {path}")
        directory = directory.resolve(strict=True)
        for side in ("Left", "Right"):
            path = directory / f"{user}{side}MetaglovePro.mcal"
            resolved = path.resolve(strict=True)
            if not resolved.is_relative_to(directory):
                raise ValueError(f"path escapes selected Manus calibration: {path}")
            if not resolved.is_file():
                raise ValueError(f"expected Manus calibration file: {path}")
    except (OSError, RuntimeError) as error:
        raise ValueError(f"missing or inaccessible Manus calibration for {user}: {directory}") from error
    return directory
