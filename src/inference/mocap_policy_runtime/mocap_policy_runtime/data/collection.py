"""Measured-joint playback of target data_collection schema-v1 episodes.

These are observations, NOT historical commands or policy actions. Camera data
is not decoded or presented as validated by this state-only reader.
"""
from __future__ import annotations

import json
from pathlib import Path

import h5py
import numpy as np

from ._h5 import ARM_JOINT_NAMES, HAND_JOINT_NAMES, SIDES, array, text, times_ns
from .trajectory import Trajectory, from_timestamp_tracks


def _metadata(path: Path) -> tuple[Path, dict]:
    for directory in path.resolve().parents:
        config_path = directory / "dataset_config.json"
        if config_path.is_file():
            try:
                config = json.loads(config_path.read_text(encoding="utf-8"))
            except (OSError, ValueError) as exc:
                raise ValueError(f"invalid collection metadata: {config_path}") from exc
            if not isinstance(config, dict):
                raise ValueError(f"collection metadata must be an object: {config_path}")
            return config_path, config
    raise ValueError("data_collection requires an ancestor dataset_config.json to establish joint order")


def load_collection(stream: h5py.File, path: Path, mode: str) -> Trajectory:
    if mode != "joint":
        raise ValueError("data_collection contains measured joint observations, not targets or actions; use mode='joint' for observation playback")
    if set(stream) != {"observations", "images"}:
        raise ValueError("data_collection schema-v1 requires observations and images root groups only")
    version = stream.attrs.get("schema_version")
    if not isinstance(version, (int, np.integer)) or isinstance(version, (bool, np.bool_)) or version != 1:
        raise ValueError("unsupported data_collection schema_version; expected integer 1")
    if path.name.endswith(".partial.h5") or "success" not in stream.attrs:
        raise ValueError("unfinished data_collection episode cannot be replayed")
    for attr in ("robot_config", "task"):
        if not text(stream.attrs.get(attr, "")).strip():
            raise ValueError(f"data_collection is missing {attr}")
    config_path, config = _metadata(path)
    if config.get("schema_version") != 1 or config.get("joint_unit") != "rad":
        raise ValueError("data_collection metadata must declare schema_version=1 and joint_unit='rad'")
    if config.get("robot_config") != text(stream.attrs["robot_config"]):
        raise ValueError("data_collection robot_config does not match dataset metadata")
    expected = list(ARM_JOINT_NAMES["left"] + ARM_JOINT_NAMES["right"])
    for side in SIDES:
        for name in HAND_JOINT_NAMES[side]:
            for finger in ("index", "middle", "ring"):
                name = name.replace(f"_{finger}_", f"_{finger}_finger_")
            expected.append(name)
    if config.get("joint_names") != expected:
        raise ValueError("data_collection joint_names do not match Tianji/Wuji model order (left arm, right arm, left hand, right hand)")
    observations = stream["observations"]
    if not isinstance(observations, h5py.Group) or set(observations) != {"arms", "hands"}:
        raise ValueError("data_collection observations must contain arms and hands")
    tracks = {"arm_joints": {}, "hand_joints": {}}
    for name, width, field in (("arms", 14, "arm_joints"), ("hands", 40, "hand_joints")):
        group = observations[name]
        if not isinstance(group, h5py.Group) or set(group) != {"timestamp_ns", "qpos"}:
            raise ValueError(f"invalid data_collection state group: {name}")
        time = times_ns(group, "timestamp_ns")
        if time.size == 0 or group["timestamp_ns"].dtype != np.dtype("int64"):
            raise ValueError(f"{name}/timestamp_ns must be nonempty int64")
        values = array(group, "qpos", (len(time), width))
        if values.dtype != np.dtype("float32") or not np.isfinite(values).all():
            raise ValueError(f"{name}/qpos must be finite float32")
        valid = np.ones(len(time), dtype=bool)
        for index, side in enumerate(SIDES):
            value = values[:, index * (width // 2):(index + 1) * (width // 2)].astype(np.float64)
            tracks[field][side] = (time, value, valid, "hold")
    return from_timestamp_tracks(path, "data_collection", tracks, space="robot_tcp", metadata={
        "schema_version": 1, "mode": "joint", "provenance": "measured_joint_observations_not_commands",
        "joint_names": expected, "dataset_config": str(config_path),
        "task": text(stream.attrs["task"]), "success": bool(stream.attrs["success"]),
        "camera_validation": "not_performed_state_only_reader", "interpolation": "zero_order_hold",
    })
