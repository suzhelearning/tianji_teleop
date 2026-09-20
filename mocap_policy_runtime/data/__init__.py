"""Standalone, read-only replay loading for four explicitly distinct schemas."""
from __future__ import annotations

from pathlib import Path

import h5py

from ._h5 import reject_links, text
from .acquisition import load_acquisition
from .collection import load_collection
from .reference import REFERENCE_DATASETS, reference_trajectory
from .session import load_session
from .trajectory import Trajectory

FORMATS = ("auto", "acquisition", "regrind", "session", "data_collection")


def load_trajectory(
    path: str | Path, format: str = "auto", mode: str = "target", rate_hz: float = 50.0,
) -> Trajectory:
    """Load without torch, transports, controller imports or retained HDF5 handles.

    Acquisition and Regrind yield mocap wrist targets; mode='joint' is reserved
    for legacy session commands and data_collection measured observations.
    Regrind alone uses rate_hz; all other formats retain their stored timestamps.
    """
    if format not in FORMATS:
        raise ValueError(f"format must be one of {FORMATS}, got {format!r}")
    if mode not in ("target", "joint"):
        raise ValueError("mode must be 'target' or 'joint'")
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(path)
    with h5py.File(path, "r") as stream:
        reject_links(stream)
        candidates = []
        if "h5_version" in stream.attrs or text(stream.attrs.get("schema_name", "")) == "mocap-acquisition":
            candidates.append("acquisition")
        if text(stream.attrs.get("schema_name", "")) == "tianji-teleop-session":
            candidates.append("session")
        if "observations" in stream and "schema_version" in stream.attrs:
            candidates.append("data_collection")
        if any(name in stream for name in REFERENCE_DATASETS[:3]):
            candidates.append("regrind")
        if len(candidates) != 1:
            raise ValueError(f"unknown or ambiguous HDF5 schema: detected {candidates}; expected acquisition, Regrind reference, legacy session or data_collection")
        detected = candidates[0]
        if format != "auto" and format != detected:
            raise ValueError(f"requested format={format!r}, but file schema is {detected!r}")
        if detected in ("acquisition", "regrind") and mode != "target":
            raise ValueError(f"{detected} stores wrist references, not recorded arm commands; use mode='target'")
        if detected == "acquisition":
            return load_acquisition(stream, path)
        if detected == "regrind":
            return reference_trajectory(stream, path, rate_hz)
        if detected == "session":
            return load_session(stream, path, mode)
        return load_collection(stream, path, mode)


__all__ = ["FORMATS", "Trajectory", "load_trajectory"]
