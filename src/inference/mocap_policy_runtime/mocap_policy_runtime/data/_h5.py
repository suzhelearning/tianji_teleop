"""Read-only HDF5 guards shared by the supported recording formats."""
from __future__ import annotations

import h5py
import numpy as np

SIDES = ("left", "right")
ARM_JOINT_NAMES = {
    side: tuple(f"Joint{i}_{'L' if side == 'left' else 'R'}" for i in range(1, 8))
    for side in SIDES
}
_HAND_NAMES = (
    "thumb_cmc_flex", "thumb_cmc_abd", "thumb_mcp", "thumb_ip",
    "index_mcp_flex", "index_mcp_abd", "index_pip", "index_dip",
    "middle_mcp_flex", "middle_mcp_abd", "middle_pip", "middle_dip",
    "ring_mcp_flex", "ring_mcp_abd", "ring_pip", "ring_dip",
    "pinky_mcp_flex", "pinky_mcp_abd", "pinky_pip", "pinky_dip",
)
HAND_JOINT_NAMES = {
    side: tuple(f"{'l' if side == 'left' else 'r'}_{name}" for name in _HAND_NAMES)
    for side in SIDES
}


def text(value: object) -> str:
    return bytes(value).decode("utf-8") if isinstance(value, (bytes, np.bytes_)) else str(value)


def reject_links(stream: h5py.File) -> None:
    """Never dereference soft/external links, external storage or virtual datasets."""
    visited = set()

    def visit(group: h5py.Group) -> None:
        address = h5py.h5o.get_info(group.id).addr
        if address in visited:
            return
        visited.add(address)
        for name in group:
            if not isinstance(group.get(name, getlink=True), h5py.HardLink):
                raise ValueError(f"linked HDF5 objects are not allowed: {group.name}/{name}")
            item = group[name]
            if isinstance(item, h5py.Group):
                visit(item)
            elif isinstance(item, h5py.Dataset) and (item.is_virtual or item.external):
                raise ValueError(f"external/virtual HDF5 storage is not allowed: {item.name}")

    visit(stream)


def array(group: h5py.Group, name: str, shape: tuple[int, ...] | None = None) -> np.ndarray:
    if name not in group or not isinstance(group[name], h5py.Dataset):
        raise ValueError(f"missing dataset: {group.name}/{name}")
    dataset = group[name]
    if dataset.dtype.kind not in "biuf":
        raise ValueError(f"expected numeric dataset: {dataset.name}")
    if shape is not None and dataset.shape != shape:
        raise ValueError(f"{dataset.name}: expected shape {shape}, got {dataset.shape}")
    return np.asarray(dataset[:])


def flags(group: h5py.Group, name: str, count: int) -> np.ndarray:
    values = array(group, name, (count,))
    if not np.isin(values, (0, 1)).all():
        raise ValueError(f"{group.name}/{name}: validity must contain only booleans or 0/1")
    return values.astype(bool)


def times_ns(group: h5py.Group, name: str = "time_ns", *, strict: bool = False) -> np.ndarray:
    values = array(group, name)
    if values.ndim != 1 or values.dtype.kind not in "iu":
        raise ValueError(f"{group.name}/{name}: expected integer [N] timestamps")
    if values.size and (values.min() < 0 or values.max() > np.iinfo(np.int64).max):
        raise ValueError(f"{group.name}/{name}: timestamps outside nonnegative int64 range")
    values = values.astype(np.int64)
    delta = np.diff(values)
    if np.any(delta <= 0 if strict else delta < 0):
        raise ValueError(f"{group.name}/{name}: timestamps must be {'strictly increasing' if strict else 'non-decreasing'}")
    return values


def pose_valid(values: np.ndarray, *, unit_tolerance: float | None = None) -> np.ndarray:
    norms = np.linalg.norm(values[:, 3:7], axis=1)
    valid = np.isfinite(values).all(axis=1) & (norms > 1e-8)
    if unit_tolerance is not None:
        valid &= np.abs(norms - 1.0) < unit_tolerance
    return valid
