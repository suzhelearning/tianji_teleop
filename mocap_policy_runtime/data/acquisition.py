"""mocap-acquisition v4/v5 compact aligned recordings."""
from __future__ import annotations

from pathlib import Path

import h5py
import numpy as np
from scipy.spatial.transform import Rotation

from ._h5 import SIDES, array, flags, pose_valid, text, times_ns
from .trajectory import Trajectory, from_timestamp_tracks


def load_acquisition(stream: h5py.File, path: Path) -> Trajectory:
    version = text(stream.attrs.get("h5_version", ""))
    if version not in ("4.0", "5.0"):
        raise ValueError(f"unsupported acquisition h5_version={version!r}; expected 4.0 or 5.0")
    layout = text(stream.attrs.get("schema_layout", ""))
    if layout and layout != "compact-aligned-60hz-v1":
        raise ValueError(f"unsupported acquisition schema_layout={layout!r}")
    schema_name = text(stream.attrs.get("schema_name", "mocap-acquisition"))
    if schema_name != "mocap-acquisition":
        raise ValueError(f"invalid acquisition schema_name={schema_name!r}")
    time = times_ns(stream, strict=True)
    count = len(time)
    if count < 2:
        raise ValueError("acquisition time_ns must contain at least two frames")
    hz = float(stream.attrs.get("output_hz", 60.0))
    if not np.isfinite(hz) or hz <= 0:
        raise ValueError("acquisition output_hz must be positive and finite")
    tracks = {"wrist_poses": {}, "hand_keypoints": {}, "hand_joints": {}, "object_poses": {}}
    for side in SIDES:
        name = f"hands/{side}"
        if name not in stream or not isinstance(stream[name], h5py.Group):
            raise ValueError(f"missing acquisition hand group {name}")
        group = stream[name]
        position = array(group, "wrist_position", (count, 3)).astype(np.float64)
        quaternion = array(group, "wrist_quaternion_xyzw", (count, 4)).astype(np.float64)
        keypoints = array(group, "keypoints_world", (count, 21, 3)).astype(np.float64)
        wrist = np.concatenate((position, quaternion), axis=1)
        valid = flags(group, "valid", count) & pose_valid(wrist, unit_tolerance=0.05)
        valid &= np.isfinite(keypoints).all(axis=(1, 2))
        valid &= np.linalg.norm(keypoints[:, 0] - position, axis=1) <= 1e-5
        # Row-vector inverse rotation: (world_point - wrist_position) @ R.
        local = np.full_like(keypoints, np.nan)
        if np.any(valid):
            rotations = Rotation.from_quat(quaternion[valid]).as_matrix()
            local[valid] = np.einsum("nki,nij->nkj", keypoints[valid] - position[valid, None, :], rotations)
        tracks["wrist_poses"][side] = (time, wrist, valid, "pose")
        tracks["hand_keypoints"][side] = (time, local, valid, "linear")
        if "wuji2_joints" in group:
            joints = array(group, "wuji2_joints", (count, 20)).astype(np.float64)
            joint_valid = valid & np.isfinite(joints).all(axis=1)
            tracks["hand_joints"][side] = (time, joints, joint_valid, "linear")
    if "objects" in stream:
        objects = stream["objects"]
        if not isinstance(objects, h5py.Group):
            raise ValueError("acquisition objects must be a group")
        for name, group in objects.items():
            if not isinstance(group, h5py.Group):
                raise ValueError(f"acquisition objects/{name} must be a group")
            position = array(group, "object_position", (count, 3)).astype(np.float64)
            quaternion = array(group, "object_quaternion_xyzw", (count, 4)).astype(np.float64)
            pose = np.concatenate((position, quaternion), axis=1)
            valid = flags(group, "valid", count) & pose_valid(pose, unit_tolerance=0.05)
            tracks["object_poses"][name] = (time, pose, valid, "pose")
    return from_timestamp_tracks(path, "acquisition", tracks, space="mocap_wrist", metadata={
        "h5_version": version, "output_hz": hz,
        "take_id": int(stream.attrs["take_id"]) if "take_id" in stream.attrs else None,
        "object_pose_frame": text(stream.attrs.get("object_pose_frame", "unspecified")),
        "invalid_tracking": "omitted; interpolation requires two adjacent valid rows",
        "hand_joint_order": "finger-major thumb/index/middle/ring/pinky, four joints each",
    })
