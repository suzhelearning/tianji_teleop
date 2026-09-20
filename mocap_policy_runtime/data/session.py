"""Minimal session-v1 replay reader; no writer, transport or hardware imports."""
from __future__ import annotations

import json
from pathlib import Path

import h5py
import numpy as np

from ._h5 import ARM_JOINT_NAMES, HAND_JOINT_NAMES, SIDES, array, flags, pose_valid, text, times_ns
from .trajectory import Trajectory, from_timestamp_tracks


def _group(stream: h5py.File, name: str, side: str) -> h5py.Group:
    if name not in stream or not isinstance(stream[name], h5py.Group):
        raise ValueError(f"missing session group {name}")
    group = stream[name]
    if text(group.attrs.get("side", "")) != side:
        raise ValueError(f"invalid side attribute: {name}")
    return group


def _joint_order(group: h5py.Group, expected: tuple[str, ...]) -> None:
    try:
        names = json.loads(text(group.attrs["joint_names"]))
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid joint_names attribute: {group.name}") from exc
    if not isinstance(names, list) or tuple(names) != expected:
        raise ValueError(f"invalid canonical joint order: {group.name}; expected {expected}")
    if not text(group.attrs.get("logical_id", "")):
        raise ValueError(f"missing logical_id attribute: {group.name}")


def load_session(stream: h5py.File, path: Path, mode: str) -> Trajectory:
    version = text(stream.attrs.get("schema_version", ""))
    if text(stream.attrs.get("schema_name", "")) != "tianji-teleop-session" or version not in ("1.0", "1.1", "1.2"):
        raise ValueError("unsupported tianji-teleop-session schema; expected version 1.0, 1.1 or 1.2")
    complete = stream.attrs.get("complete")
    if not isinstance(complete, (bool, np.bool_)) or not complete:
        raise ValueError("session must have boolean complete=true; interrupted sessions cannot be replayed")
    tracks = {"wrist_poses": {}, "hand_keypoints": {}, "arm_joints": {}, "hand_joints": {}}
    for side in SIDES:
        for domain, width in (("arm", 7), ("hand", 20)):
            name = f"target/{domain}/{side}" if mode == "target" else f"joint/command/{domain}/{side}"
            group = _group(stream, name, side)
            time = times_ns(group)
            count = len(time)
            if mode == "joint":
                value = array(group, "position_rad", (count, width)).astype(np.float64)
                if count:
                    _joint_order(group, ARM_JOINT_NAMES[side] if domain == "arm" else HAND_JOINT_NAMES[side])
                valid = np.isfinite(value).all(axis=1)
                if not valid.all():
                    raise ValueError(f"non-finite commanded joints: {name}")
                field = "arm_joints" if domain == "arm" else "hand_joints"
            else:
                frame = "wrist_relative_mediapipe" if domain == "hand" else ("Base_L" if side == "left" else "Base_R")
                if text(group.attrs.get("frame_id", "")) != frame:
                    raise ValueError(f"invalid target frame_id: {name}; expected {frame}")
                if count and not text(group.attrs.get("source", "")):
                    raise ValueError(f"missing target source attribute: {name}")
                valid = flags(group, "tracking_valid", count) if "tracking_valid" in group else np.ones(count, dtype=bool)
                if domain == "arm":
                    value = array(group, "pose", (count, 7)).astype(np.float64)
                    numeric = pose_valid(value, unit_tolerance=0.05)
                    if np.any(valid & ~numeric):
                        raise ValueError(f"invalid tracked target pose: {name}")
                    value[valid, 3:7] /= np.linalg.norm(value[valid, 3:7], axis=1)[:, None]
                    field = "wrist_poses"
                else:
                    value = array(group, "keypoints_m", (count, 21, 3)).astype(np.float64)
                    numeric = np.isfinite(value).all(axis=(1, 2))
                    if np.any(valid & ~numeric):
                        raise ValueError(f"non-finite hand target: {name}")
                    if np.any(valid & (np.linalg.norm(value[:, 0], axis=1) > 1e-5)):
                        raise ValueError(f"hand keypoints must be wrist-relative: {name}")
                    field = "hand_keypoints"
            tracks[field][side] = (time, value, valid, "hold")
    return from_timestamp_tracks(path, "session", tracks, space="robot_tcp", metadata={
        "schema_version": version, "mode": mode, "interpolation": "zero_order_hold",
        "source_type": text(stream.attrs.get("source_type", "")),
        "arm_joint_names": ARM_JOINT_NAMES, "hand_joint_names": HAND_JOINT_NAMES,
        "arm_pose_frames": {"left": "Base_L", "right": "Base_R"},
    })
