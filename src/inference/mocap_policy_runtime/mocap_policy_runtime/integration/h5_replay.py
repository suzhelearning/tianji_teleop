"""Right-only source-time replay of acquisition and solved Regrind recordings."""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from ..data import load_trajectory
from ..data.geometry import compose_pose
from ..data.trajectory import Track
from ..types import TargetFrame
from .native import finite_vector, pose_xyzw


# Acquisition wrist W -> WuJi wrist B, not the Manus node axis_transform.
MANUS_TO_WUJI_WRIST = np.array([0., 0., 0., np.sqrt(0.5), 0., -np.sqrt(0.5), 0.])


def motive_world_transform(robot_home_wrist, measured_home_wrist, rotation=None):
    """Map capture world to robot world from the same calibrated wrist at Home.

    Default: T_robot_mocap = T_robot_wrist_home * inverse(T_mocap_wrist_home).
    The measured pose must include the rigid->marker->mount->wrist chain, not
    raw rigid-body or Manus palm axes, and the physical robot must be at Home.
    ``rotation`` optionally overrides the fitted rotation with an externally
    calibrated xyzw quaternion or 3x3 matrix; Home then fits translation only.
    """
    robot = pose_xyzw(robot_home_wrist, "robot Home wrist")
    measured = pose_xyzw(measured_home_wrist, "measured Home wrist")
    if rotation is None:
        world_rotation = Rotation.from_quat(robot[3:]) * Rotation.from_quat(measured[3:]).inv()
    else:
        value = np.asarray(rotation, dtype=float)
        if value.shape == (4,):
            world_rotation = Rotation.from_quat(pose_xyzw(np.r_[np.zeros(3), value])[3:])
        elif value.shape == (3, 3) and np.isfinite(value).all():
            if not np.allclose(value.T @ value, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(value), 1., atol=1e-6):
                raise ValueError("world rotation must be a proper orthogonal matrix")
            world_rotation = Rotation.from_matrix(value)
        else:
            raise ValueError("world rotation must be a finite xyzw quaternion or 3x3 matrix")
    return np.r_[robot[:3] - world_rotation.apply(measured[:3]), world_rotation.as_quat()]


def motive_rigid_to_wrist(settings):
    """Use the shared calibrated right_wrist chain, with explicit YAML overrides."""
    from ..policies.regrind.tracking import WRIST_RIGID_TO_MARKER, MARKER_TO_MOUNT, MOUNT_TO_WRIST

    def configured(prefix, default):
        return pose_xyzw(np.r_[
            settings.get(prefix + "_translation_m", default[:3]),
            settings.get(prefix + "_quaternion_xyzw", default[3:]),
        ], prefix)

    rigid_to_marker = configured("right_rigid_to_marker_mocap", WRIST_RIGID_TO_MARKER)
    marker_to_mount = configured("right_marker_to_mount", MARKER_TO_MOUNT)
    return compose_pose(compose_pose(rigid_to_marker, marker_to_mount), MOUNT_TO_WRIST)


class H5Replay:
    """Prepare a complete right-hand target before any replay or device access.

    Wrist gaps bridge valid rows; acquisition preview holds the last valid hand
    shape. Missing acquisition joints are solved once, chronologically, at valid
    right-hand frames and interpolate across the same gaps. Existing canonical
    joints retain finite-row forward-fill, even during wrist tracking dropouts.
    An unavailable initial joint row or failed retargeting aborts construction.

    Speed belongs to the hold clock owner: sample times and duration remain in
    source seconds. Regrind alone uses ``rate_hz`` to construct its source clock.
    ``trajectory`` retains the original recording for metadata, whereas
    ``hand_joint_track`` is the complete executable, replay-relative joint track.
    """

    def __init__(self, path, *, speed=1.0, yaw_deg=0.0, format="auto", rate_hz=50.0):
        self.speed = float(speed)
        self.yaw_deg = float(yaw_deg)
        if not np.isfinite(self.speed) or self.speed <= 0:
            raise ValueError("speed must be positive and finite")
        if not np.isfinite(self.yaw_deg):
            raise ValueError("yaw_deg must be finite")
        self.trajectory = load_trajectory(path, format=format, rate_hz=rate_hz)
        self.format = self.trajectory.format
        if self.format not in ("acquisition", "regrind"):
            raise ValueError("H5 replay requires acquisition or Regrind wrist targets")
        wrist = self.trajectory.tracks["wrist_poses"]["right"]
        self.valid_indices = np.flatnonzero(wrist.valid)
        if not self.valid_indices.size:
            raise ValueError("recording contains no valid right wrist")
        self.start_frame_index = int(self.valid_indices[0])
        self.end_frame_index = int(self.valid_indices[-1])
        origin = wrist.time_s[self.start_frame_index]
        self._frame_times = wrist.time_s - origin
        self.duration_s = float(self._frame_times[self.end_frame_index])
        self.interpolated_frame_count = int(np.count_nonzero(~wrist.valid[self.start_frame_index:self.end_frame_index + 1]))
        times = self._frame_times[self.valid_indices]
        valid = np.ones(len(times), dtype=bool)
        yaw = Rotation.from_euler("z", self.yaw_deg, degrees=True)
        poses = wrist.values[self.valid_indices].copy()
        poses[:, :3] = yaw.apply(poses[:, :3])
        poses[:, 3:] = (yaw * Rotation.from_quat(poses[:, 3:])).as_quat()
        self._wrist = Track(times, poses, valid, "pose")
        self._points = None
        if self.format == "acquisition":
            points = self.trajectory.tracks["hand_keypoints"]["right"]
            # Retain the source skeleton in Manus-local axes, independently of
            # the control wrist's Manus -> Wuji conversion.
            self._points = Track(times, points.values[self.valid_indices].copy(), valid, "hold")
        joint_track = self.trajectory.tracks.get("hand_joints", {}).get("right")
        if joint_track is None:
            from .hand import HandRetargeter

            values = np.empty((len(self.valid_indices), 20), dtype=np.float64)
            retargeter = HandRetargeter("right")
            try:
                for row, index in enumerate(self.valid_indices):
                    try:
                        values[row] = finite_vector(retargeter.retarget(points.values[index]), 20,
                                                    "retargeted right hand joints")
                    except (ValueError, RuntimeError, OSError) as exc:
                        raise ValueError(f"right hand retargeting failed at source frame {index}: {exc}") from exc
            finally:
                retargeter.close()
            self.hand_joint_track = Track(times, values, valid, "linear")
            self.hand_mode = "retargeted"
        else:
            values = joint_track.values.copy()
            finite = np.isfinite(values).all(axis=1)
            previous = None
            for index, is_finite in enumerate(finite):
                if is_finite:
                    previous = values[index]
                elif previous is not None:
                    values[index] = previous
            active = slice(self.start_frame_index, self.end_frame_index + 1)
            values = values[active]
            if not np.isfinite(values).all():
                raise ValueError("canonical right hand joints unavailable at first valid wrist frame")
            self.hand_joint_track = Track(self._frame_times[active], values,
                                          np.ones(len(values), dtype=bool), "linear")
            self.hand_mode = "regrind" if self.format == "regrind" else "canonical"
        self._objects = {}
        for name, track in self.trajectory.tracks.get("object_poses", {}).items():
            values = track.values.copy()
            if track.valid.any():
                values[track.valid, :3] = yaw.apply(values[track.valid, :3])
                values[track.valid, 3:] = (yaw * Rotation.from_quat(values[track.valid, 3:])).as_quat()
            # Objects retain their own validity/gaps; never manufacture poses
            # from an untracked object or apply the wrist's local-axis change.
            self._objects[name] = Track(track.time_s - origin, values, track.valid.copy(), "pose")

    def _time_index(self, time_s):
        requested = float(time_s)
        if not np.isfinite(requested):
            raise ValueError("time_s must be finite")
        elapsed = float(np.clip(requested, 0., self.duration_s))
        index = int(np.clip(np.searchsorted(self._frame_times, elapsed, side="right") - 1,
                            self.start_frame_index, self.end_frame_index))
        return elapsed, index, elapsed >= self.duration_s - 1e-9

    def _object_poses(self, elapsed):
        poses = {}
        for name, track in self._objects.items():
            pose = track.sample(elapsed)
            if pose is not None:
                poses[name] = pose
        return poses

    def sample(self, time_s):
        elapsed, index, complete = self._time_index(time_s)
        wrist = self._wrist.sample(elapsed)
        if self.format == "acquisition":
            wrist = compose_pose(wrist, MANUS_TO_WUJI_WRIST)
        return TargetFrame(elapsed, index, complete,
                           wrist_poses={"right": wrist},
                           hand_joints={"right": self.hand_joint_track.sample(elapsed)},
                           object_poses=self._object_poses(elapsed))

    def preview(self, time_s):
        """Display the actual source skeleton, or solved wrist/joints without one."""
        elapsed, index, complete = self._time_index(time_s)
        return TargetFrame(elapsed, index, complete,
                           wrist_poses={"right": self._wrist.sample(elapsed)},
                           hand_keypoints={} if self._points is None else {"right": self._points.sample(elapsed)},
                           hand_joints={"right": self.hand_joint_track.sample(elapsed)},
                           object_poses=self._object_poses(elapsed))
