"""Recorded capture poses / direct joints to the target robot's canonical q54."""
from __future__ import annotations

from contextlib import ExitStack

import numpy as np
from scipy.spatial.transform import Rotation

from ..data.geometry import compose_pose, invert_pose
from ..types import TargetFrame
from .native import SIDES, finite_vector, pose_xyzw


def model_wrist_pose(model, data, side: str) -> np.ndarray:
    """Recover the original Wuji wrist link frame, even after MJCF body fusion.

    MuJoCo folds a mesh's compiler centering/principal-axis transform into its
    geom frame. Undo that transform; raw geom_xpos/xmat are NOT the wrist pose.
    This function performs no simulation step or device operation.
    """
    import mujoco

    if side not in SIDES:
        raise ValueError("wrist side must be left/right")
    name = f"wuji2_{side[0]}_wrist"
    mesh = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MESH, name)
    if mesh < 0:
        raise ValueError(f"model has no {name} mesh")
    geoms = np.flatnonzero((model.geom_type == int(mujoco.mjtGeom.mjGEOM_MESH)) & (model.geom_dataid == mesh))
    if geoms.size != 1:
        raise ValueError(f"model must contain exactly one {name} mesh geom")
    geom = int(geoms[0])
    geom_rotation = Rotation.from_matrix(data.geom_xmat[geom].reshape(3, 3))
    mesh_rotation = Rotation.from_quat(np.roll(model.mesh_quat[mesh], -1))
    wrist_rotation = geom_rotation * mesh_rotation.inv()
    return np.concatenate((data.geom_xpos[geom] - wrist_rotation.apply(model.mesh_pos[mesh]),
                           wrist_rotation.as_quat()))


class TargetAdapter:
    """Convert a target on every 200 Hz tick; retarget new hand samples only.

    Recorded mocap defaults to an independent first-frame home alignment for
    each hand. ``set_world_transform`` instead installs ONE fixed capture-world
    to robot-world transform, preserving absolute Regrind motion and object
    coordinates. ``robot_tcp`` means Base_L/Base_R TCP coordinates and bypasses
    mocap alignment and wrist extrinsics; ``robot_world_tcp`` bypasses base too.
    Direct arm and hand joint vectors are already in canonical physical order.
    """

    def __init__(self, ik, initial_positions=None, alignment="first-frame", *, legacy_tcp=None,
                 conditioning_settings=None):
        if alignment not in ("first-frame", "absolute"):
            raise ValueError("alignment must be first-frame or absolute")
        if legacy_tcp not in (None, "flange", "hand"):
            raise ValueError("legacy_tcp must be explicitly flange or hand")
        self.legacy_tcp = legacy_tcp
        self.ik = ik
        self.alignment = alignment
        self.lower = finite_vector(ik.lower, 54, "lower limits")
        self.upper = finite_vector(ik.upper, 54, "upper limits")
        self.current_positions = (np.concatenate((ik.home_positions, np.zeros(40)))
                                  if initial_positions is None else
                                  finite_vector(initial_positions, 54, "initial positions").copy())
        self._check_limits(self.current_positions)
        if not np.allclose(self.current_positions[:14], ik.current_positions, rtol=0, atol=1e-12):
            ik.synchronize(self.current_positions[:14])
        self.home_wrist = {side: pose_xyzw(ik.home_wrist[side], f"{side} home wrist") for side in SIDES}
        self._wrist_to_tcp = {side: invert_pose(ik.tcp_to_wrist[side]) for side in SIDES}
        self._alignment = {}
        self._world_transform = None
        self._direct_sides = set()
        self._retargeters = {}
        self._closed = False
        self._hand_cache = {}
        self.last_tcp_targets = {}
        self._conditioners = {}
        if conditioning_settings is not None:
            from .conditioning import TargetConditioner, TargetConditioningSettings
            settings = TargetConditioningSettings(
                rate_hz=200.0,
                translation_gain=np.asarray(conditioning_settings.get("translation_gain", [1., 1., 1.])),
                rotation_gain=float(conditioning_settings.get("rotation_gain", 1.0)),
                **{name: conditioning_settings[name] for name in (
                    "workspace_relative_radii_m", "workspace_soft_zone_ratio",
                    "maximum_linear_speed_m_s", "maximum_angular_speed_rad_s",
                    "maximum_linear_acceleration_m_s2", "maximum_angular_acceleration_rad_s2")},
            )
            for side in SIDES:
                home = compose_pose(invert_pose(ik.base_poses[side]), ik.home_tcp[side])
                self._conditioners[side] = TargetConditioner(home[:3], home[3:], settings)

    def close(self) -> None:
        self._closed = True
        with ExitStack() as cleanup:
            for retargeter in self._retargeters.values():
                cleanup.callback(retargeter.close)
            self._retargeters.clear()
            self._hand_cache.clear()

    def __enter__(self):
        if self._closed:
            raise RuntimeError("target adapter is closed")
        return self

    def __exit__(self, *_):
        self.close()

    def _check_limits(self, q):
        if np.any(q < self.lower) or np.any(q > self.upper):
            index = int(np.flatnonzero((q < self.lower) | (q > self.upper))[0])
            raise ValueError(f"joint {index} target {q[index]:.9g} outside [{self.lower[index]:.9g}, {self.upper[index]:.9g}] rad")

    def reset_alignment(self) -> None:
        """Discard first-frame anchors and any explicitly installed calibration."""
        self._alignment.clear()
        self._world_transform = None
        self._hand_cache.clear()
        self.last_tcp_targets = {}

    def set_world_transform(self, pose_xyzw_value) -> None:
        """Fix capture/training-world -> model-world; do not reanchor frame zero."""
        self._world_transform = pose_xyzw(pose_xyzw_value, "world transform")
        self._alignment.clear()

    def hold(self, positions) -> None:
        """Freeze native/limiter history at the last speed-bounded command."""
        values = finite_vector(positions, 54, "held command")
        self._check_limits(values)
        self.ik.synchronize(values[:14])
        self.current_positions = values.copy()
        self.last_tcp_targets = {}
        for side, conditioner in self._conditioners.items():
            pose = compose_pose(invert_pose(self.ik.base_poses[side]), self.ik.current_tcp[side])
            conditioner.synchronize(pose[:3], pose[3:])

    def _tcp_target(self, side, value, space):
        pose = pose_xyzw(value, f"{side} pose")
        if space == "robot_world_tcp":
            return pose
        if space == "robot_tcp":
            if self.legacy_tcp is None:
                raise ValueError("legacy Base-frame TCP replay requires --legacy-tcp flange|hand")
            world = compose_pose(self.ik.base_poses[side], pose)
            return compose_pose(world, self.ik.flange_to_tcp[side]) if self.legacy_tcp == "flange" else world
        if self._world_transform is not None:
            transform = self._world_transform
        elif self.alignment == "first-frame":
            if side not in self._alignment:
                self._alignment[side] = compose_pose(self.home_wrist[side], invert_pose(pose))
            transform = self._alignment[side]
        else:
            transform = np.array([0., 0., 0., 0., 0., 0., 1.])
        return compose_pose(compose_pose(transform, pose), self._wrist_to_tcp[side])

    def _retarget(self, side, points, target):
        points = np.asarray(points, dtype=np.float64)
        if points.shape != (21, 3) or not np.isfinite(points).all():
            raise ValueError(f"{side} keypoints must be finite 21x3 wrist-local metres")
        if np.linalg.norm(points[0]) > 1e-8:
            raise ValueError(f"{side} keypoint zero must be the wrist-local origin")
        key = target.index
        previous = self._hand_cache.get(side)
        if previous is not None and previous[0] == key and np.array_equal(points, previous[1]):
            return previous[2]
        if side not in self._retargeters:
            # Reuse the target's exact official Hand2 solver, native joint-name
            # permutation, coordinate calibration, NLopt failure checks/filter.
            from .hand import HandRetargeter
            self._retargeters[side] = HandRetargeter(side)
        joints = finite_vector(self._retargeters[side].retarget(points), 20, f"{side} retargeted joints")
        self._hand_cache[side] = (key, points.copy(), joints.copy())
        return joints

    def frame(self, target: TargetFrame) -> np.ndarray:
        if self._closed:
            raise RuntimeError("target adapter is closed")
        if target.space not in ("mocap_wrist", "robot_tcp", "robot_world_tcp"):
            raise ValueError(f"unknown pose space {target.space!r}")
        if not np.isfinite(target.time_s) or target.index < 0:
            raise ValueError("target time/index must be finite and nonnegative")
        for mapping in (target.wrist_poses, target.arm_joints, target.hand_joints, target.hand_keypoints):
            if set(mapping) - set(SIDES):
                raise ValueError("target sides must be left/right")
        for name, pose in target.object_poses.items():
            pose_xyzw(pose, f"object {name}")
        # Validate all direct data BEFORE advancing native IK. No arbitrary
        # replay target is clipped or silently converted to a held command.
        result = self.current_positions.copy()
        for i, side in enumerate(SIDES):
            if side in target.arm_joints:
                result[i * 7:(i + 1) * 7] = finite_vector(target.arm_joints[side], 7, f"{side} arm joints")
            hand_slice = slice(14 + i * 20, 34 + i * 20)
            if side in target.hand_joints:
                result[hand_slice] = finite_vector(target.hand_joints[side], 20, f"{side} hand joints")
            elif side in target.hand_keypoints:
                result[hand_slice] = self._retarget(side, target.hand_keypoints[side], target)
        self._check_limits(result)
        tcp = {side: self._tcp_target(side, pose, target.space)
               for side, pose in target.wrist_poses.items() if side not in target.arm_joints}
        for side, pose in tcp.items():
            if side in self._conditioners:
                base_pose = self.ik.base_poses[side]
                local = compose_pose(invert_pose(base_pose), pose)
                position, quaternion, _ = self._conditioners[side].condition(local[:3], local[3:])
                tcp[side] = compose_pose(base_pose, np.concatenate((position, quaternion)))
        if self._direct_sides.intersection(tcp):
            self.ik.synchronize(self.current_positions[:14])
            self._direct_sides.clear()
        if tcp or self.last_tcp_targets:
            q = finite_vector(self.ik.step(tcp), 14, "IK result")
            for i, side in enumerate(SIDES):
                if side in tcp:
                    result[i * 7:(i + 1) * 7] = q[i * 7:(i + 1) * 7]
        self._check_limits(result)
        self._direct_sides.update(target.arm_joints)
        self.last_tcp_targets = tcp
        self.current_positions = result
        return result.copy()
