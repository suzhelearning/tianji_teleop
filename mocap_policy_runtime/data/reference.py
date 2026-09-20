"""Regrind reference arrays, independent of the optional torch actor runtime."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import h5py
import numpy as np

from ._h5 import array, reject_links
from .trajectory import Track, Trajectory

REFERENCE_DATASETS = (
    "regrind_retargeting_root_pos", "regrind_retargeting_root_quat",
    "regrind_retargeting_joints", "object_pos", "object_quat",
)


@dataclass(frozen=True)
class RegrindReference:
    wrist_pos: np.ndarray
    wrist_quat_wxyz: np.ndarray
    joints: np.ndarray
    object_pos: np.ndarray
    object_quat_wxyz: np.ndarray

    @property
    def frame_count(self) -> int:
        return len(self.wrist_pos)


def _read_reference(stream: h5py.File) -> RegrindReference:
    root = array(stream, REFERENCE_DATASETS[0])
    if root.ndim != 2 or root.shape[1] != 3 or len(root) < 2:
        raise ValueError("Regrind reference root position must have shape [N,3] with at least two frames")
    count = len(root)
    arrays = [root.astype(np.float64)]
    for name, width in zip(REFERENCE_DATASETS[1:], (4, 20, 3, 4)):
        arrays.append(array(stream, name, (count, width)).astype(np.float64))
    for name, value in zip(REFERENCE_DATASETS, arrays):
        if not np.isfinite(value).all():
            raise ValueError(f"Regrind reference {name} contains non-finite values")
    for index in (1, 4):
        norms = np.linalg.norm(arrays[index], axis=1)
        if np.any(norms < 1e-8):
            raise ValueError(f"Regrind reference {REFERENCE_DATASETS[index]} contains a zero quaternion")
        arrays[index] /= norms[:, None]
    return RegrindReference(*arrays)


def load_reference(path: str | Path) -> RegrindReference:
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(path)
    with h5py.File(path, "r") as stream:
        reject_links(stream)
        return _read_reference(stream)


def reference_trajectory(stream: h5py.File, path: Path, rate_hz: float) -> Trajectory:
    if not np.isfinite(rate_hz) or rate_hz <= 0:
        raise ValueError("reference rate_hz must be positive and finite")
    reference = _read_reference(stream)
    time = np.arange(reference.frame_count, dtype=np.float64) / rate_hz
    valid = np.ones(reference.frame_count, dtype=bool)
    wrist = np.column_stack((reference.wrist_pos, reference.wrist_quat_wxyz[:, (1, 2, 3, 0)]))
    hammer = np.column_stack((reference.object_pos, reference.object_quat_wxyz[:, (1, 2, 3, 0)]))
    return Trajectory(path, "regrind", time, {
        "wrist_poses": {"right": Track(time, wrist, valid, "pose")},
        "hand_joints": {"right": Track(time, reference.joints, valid, "linear")},
        "object_poses": {"hammer": Track(time, hammer, valid, "pose")},
    }, space="mocap_wrist", metadata={
        "rate_hz": float(rate_hz), "time_origin": "frame_zero",
        "source_quaternion_order": "wxyz", "output_quaternion_order": "xyzw",
        "hand_joint_order": "finger-major thumb/index/middle/ring/pinky, four joints each",
    })
