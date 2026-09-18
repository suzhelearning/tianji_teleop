"""Configurable human pose to robot TCP mapping backends.

This module only maps geometry.  It does not subscribe to input, condition a
target, call IK, or publish a robot command.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any, Mapping, Protocol

import numpy as np
from scipy.spatial.transform import Rotation

from .models import ArmInputObservation
from .control_geometry import IK_TCP_TO_HAND_CONTROL_TRANSLATION_M


SIDES = ("left", "right")


def _value(config: Any, name: str, default: Any = None) -> Any:
    if isinstance(config, Mapping):
        return config.get(name, default)
    return getattr(config, name, default)


def _side_value(config: Any, name: str, side: str, default: Any) -> Any:
    values = _value(config, name, None)
    if values is None:
        return default
    if isinstance(values, Mapping):
        return values.get(side, default)
    return values


def _rotation(value: Any, field: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (3, 3) or not np.isfinite(result).all():
        raise ValueError(f"{field} must be a finite 3x3 matrix")
    if not np.allclose(result @ result.T, np.eye(3), atol=1.0e-6) or not np.isclose(np.linalg.det(result), 1.0, atol=1.0e-6):
        raise ValueError(f"{field} must be a proper rotation matrix")
    return result


def _pose(value: Any, field: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (7,) or not np.isfinite(result).all():
        raise ValueError(f"{field} must be a finite 7-vector")
    norm = float(np.linalg.norm(result[3:]))
    if norm < 1.0e-12:
        raise ValueError(f"{field} quaternion must be non-zero")
    result[3:] /= norm
    return result


def _compose(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    first_rotation = Rotation.from_quat(first[3:])
    second_rotation = Rotation.from_quat(second[3:])
    position = first[:3] + first_rotation.apply(second[:3])
    rotation = first_rotation * second_rotation
    return np.concatenate((position, rotation.as_quat()))


@dataclass(frozen=True)
class MappedArmPose:
    side: str
    pose: np.ndarray | None
    valid: bool
    backend: str
    mapping_version: str
    frame_association_id: str
    elbow_reference_direction: tuple[float, float, float] | None = None

    def __post_init__(self) -> None:
        if self.side not in SIDES:
            raise ValueError("side must be left or right")
        if not isinstance(self.valid, (bool, np.bool_)):
            raise ValueError("valid must be boolean")
        value = None if self.pose is None else _pose(self.pose, "mapped pose")
        if self.valid and value is None:
            raise ValueError("valid mapped pose requires pose")
        if not self.backend or not self.mapping_version or not self.frame_association_id:
            raise ValueError("mapped pose metadata is required")
        elbow = None if self.elbow_reference_direction is None else np.asarray(
            self.elbow_reference_direction, dtype=np.float64
        )
        if elbow is not None:
            if elbow.shape != (3,) or not np.isfinite(elbow).all() or np.linalg.norm(elbow) < 1.0e-12:
                raise ValueError("mapped elbow direction must be a finite non-zero 3-vector")
        object.__setattr__(self, "pose", value)
        object.__setattr__(self, "elbow_reference_direction", None if elbow is None else tuple(float(item) for item in elbow))


class ArmPoseMapper(Protocol):
    def reset(self) -> None: ...
    def initialize(self, reference: Any = None) -> None: ...
    def map(self, arm_input: ArmInputObservation) -> MappedArmPose: ...


class DirectPoseMapper:
    def __init__(self, config: Any):
        self._rotations = {
            side: _rotation(_side_value(config, "input_to_base_rotation", side, np.eye(3)), f"{side}.input_to_base_rotation")
            for side in SIDES
        }
        self._origins = {
            side: np.asarray(_side_value(config, "input_origin_in_base_m", side, np.zeros(3)), dtype=np.float64)
            for side in SIDES
        }
        for side, origin in self._origins.items():
            if origin.shape != (3,) or not np.isfinite(origin).all():
                raise ValueError(f"{side}.input_origin_in_base_m must be a finite 3-vector")
        self._tracked_to_tcp = {
            side: _pose(_side_value(config, "tracked_to_tcp_pose", side, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]), f"{side}.tracked_to_tcp_pose")
            for side in SIDES
        }
        self._expected_frames = {
            side: _side_value(config, "expected_reference_frame", side, None)
            for side in SIDES
        }
        self._expected_tracked_frames = {
            side: _side_value(config, "expected_tracked_frame", side, None)
            for side in SIDES
        }
        for side in SIDES:
            for field, value in (
                ("expected_reference_frame", self._expected_frames[side]),
                ("expected_tracked_frame", self._expected_tracked_frames[side]),
            ):
                if value is not None and (not isinstance(value, str) or not value):
                    raise ValueError(f"{side}.{field} must be a non-empty string")

    def reset(self) -> None:
        return None

    def initialize(self, reference: Any = None) -> None:
        return None

    def map(self, arm_input: ArmInputObservation) -> MappedArmPose:
        if not isinstance(arm_input, ArmInputObservation):
            raise TypeError("arm_input must be ArmInputObservation")
        if not arm_input.valid or arm_input.pose is None:
            return MappedArmPose(arm_input.side, None, False, "direct_pose", "direct_pose_v1", arm_input.frame_association_id)
        expected = self._expected_frames.get(arm_input.side)
        if expected is not None and arm_input.reference_frame != expected:
            raise ValueError(f"{arm_input.side} input reference frame must be {expected!r}")
        expected_tracked = self._expected_tracked_frames.get(arm_input.side)
        if expected_tracked is not None and arm_input.tracked_frame != expected_tracked:
            raise ValueError(f"{arm_input.side} tracked frame must be {expected_tracked!r}")
        reference_to_base = np.concatenate((self._origins[arm_input.side], Rotation.from_matrix(self._rotations[arm_input.side]).as_quat()))
        reference_to_tracked = _pose(arm_input.pose, "arm_input.pose")
        base_to_tracked = _compose(reference_to_base, reference_to_tracked)
        base_to_tcp = _compose(base_to_tracked, self._tracked_to_tcp[arm_input.side])
        return MappedArmPose(arm_input.side, base_to_tcp, True, "direct_pose", "direct_pose_v1", arm_input.frame_association_id)


class HeadDirectMapper(DirectPoseMapper):
    """Fixed base<-head<-wrist<-TCP composition, never start-relative."""

    def __init__(self, config: Any):
        config = dict(config)
        for field, expected in (("expected_reference_frame", "pico_head_current"),
                                ("expected_tracked_frame", "wrist")):
            if field in config and config[field] != expected:
                raise ValueError(f"head_direct requires {field}={expected}")
            config[field] = expected
        super().__init__(config)

    def map(self, arm_input: ArmInputObservation) -> MappedArmPose:
        result = super().map(arm_input)
        return replace(result, backend="head_direct", mapping_version="head_direct_v1")


class HeadPalmDirectMapper(HeadDirectMapper):
    """Wrist-driven head mapping with independent local-Z/height calibration.

    Despite the display-oriented name, no skeleton Palm point is consumed.
    """

    def __init__(self, config: Any):
        super().__init__(config)
        def finite(value: Any, name: str) -> float:
            if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.number)) or not np.isfinite(value):
                raise ValueError(f"{name} must be finite numeric")
            return float(value)

        height = finite(_value(config, "head_height_offset_m", 0.0), "head_height_offset_m")
        forward = finite(_value(config, "head_forward_offset_m", 0.0), "head_forward_offset_m")
        self._position_offsets = {side: self._rotations[side] @ np.array([forward, 0.0, height]) for side in SIDES}
        self._height_calibration = {}
        self._local_corrections = {
            side: Rotation.from_euler("z", finite(_side_value(config,
                "tcp_local_z_correction_deg", side, 0.0), f"{side}.tcp_local_z_correction_deg"), degrees=True)
            for side in SIDES
        }

    def calibrate_height(self, human_heights, reference):
        pending = {}
        for side, height in human_heights.items():
            if side not in SIDES or not np.isfinite(height):
                raise ValueError('invalid human calibration height')
            up = self._rotations[side][:,2]
            if not np.allclose(up, reference[side]['world_up_base'], atol=1e-5):
                raise ValueError('virtual head up does not match model world up')
            target = float(up @ reference[side]['control_position_base_m'])
            if not np.isfinite(target):
                raise ValueError('invalid robot horizontal reference')
            pending[side] = (float(height), target)
        if not pending:
            raise ValueError('no calibration sides')
        self._height_calibration = pending

    def map(self, arm_input: ArmInputObservation) -> MappedArmPose:
        result = super().map(arm_input)
        pose = None
        if result.valid:
            pose = result.pose.copy()
            pose[:3] += self._position_offsets[result.side]
            pose[3:] = (Rotation.from_quat(pose[3:]) * self._local_corrections[result.side]).as_quat()
            if result.side in self._height_calibration:
                human_height, robot_height = self._height_calibration[result.side]
                up = self._rotations[result.side][:,2]
                center = pose[:3] + Rotation.from_quat(pose[3:]).apply(IK_TCP_TO_HAND_CONTROL_TRANSLATION_M)
                desired = robot_height + arm_input.pose[2] - human_height
                # Replace the vertical component, including the old -0.10 m.
                pose[:3] += up * (desired - float(up @ center))
        return replace(result, pose=pose, backend="head_palm_direct", mapping_version="head_palm_direct_v1")


class RelativeHomeMapper:
    def __init__(self, config: Any):
        self._home = {side: _pose(_side_value(config, "home_pose", side, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]), f"{side}.home_pose") for side in SIDES}
        # home_pose always describes the original IK TCP. Map motion at the
        # configured control frame, then convert back to that same IK TCP.
        self._control_home = {}
        self._control_to_ik = {}
        for side in SIDES:
            tool = _pose(_side_value(config, "ik_tcp_to_control_pose", side,
                                    [0, 0, 0, 0, 0, 0, 1]), f"{side}.ik_tcp_to_control_pose")
            inverse_rotation = Rotation.from_quat(tool[3:]).inv()
            self._control_to_ik[side] = np.concatenate((inverse_rotation.apply(-tool[:3]), inverse_rotation.as_quat()))
            self._control_home[side] = _compose(self._home[side], tool)
        self._rotations = {side: _rotation(_side_value(config, "input_to_base_rotation", side, np.eye(3)), f"{side}.input_to_base_rotation") for side in SIDES}
        self._expected_frames = {
            side: _side_value(config, "expected_reference_frame", side, None)
            for side in SIDES
        }
        self._expected_tracked_frames = {
            side: _side_value(config, "expected_tracked_frame", side, None)
            for side in SIDES
        }
        for side in SIDES:
            for field, value in (
                ("expected_reference_frame", self._expected_frames[side]),
                ("expected_tracked_frame", self._expected_tracked_frames[side]),
            ):
                if value is not None and (not isinstance(value, str) or not value):
                    raise ValueError(f"{side}.{field} must be a non-empty string")
        self._references: dict[str, np.ndarray] = {}

    def reset(self) -> None:
        self._references.clear()

    def initialize(self, reference: Any = None) -> None:
        if reference is None:
            raise ValueError("relative_home requires an input reference")
        if isinstance(reference, ArmInputObservation):
            values = {reference.side: reference}
        elif isinstance(reference, Mapping):
            values = dict(reference)
        else:
            raise TypeError("relative_home reference must be an ArmInputObservation or side mapping")
        for side, item in values.items():
            if side not in SIDES or not isinstance(item, ArmInputObservation) or item.side != side or not item.valid or item.pose is None:
                raise ValueError(f"invalid {side} relative_home reference")
            expected = self._expected_frames.get(side)
            if expected is not None and item.reference_frame != expected:
                raise ValueError(f"{side} reference frame must be {expected!r}")
            expected_tracked = self._expected_tracked_frames.get(side)
            if expected_tracked is not None and item.tracked_frame != expected_tracked:
                raise ValueError(f"{side} tracked frame must be {expected_tracked!r}")
            self._references[side] = _pose(item.pose, f"{side}.reference.pose")

    def map(self, arm_input: ArmInputObservation) -> MappedArmPose:
        if not isinstance(arm_input, ArmInputObservation):
            raise TypeError("arm_input must be ArmInputObservation")
        reference = self._references.get(arm_input.side)
        if reference is None:
            raise RuntimeError(f"relative_home is not initialized for {arm_input.side}")
        if not arm_input.valid or arm_input.pose is None:
            return MappedArmPose(arm_input.side, None, False, "relative_home", "relative_home_v1", arm_input.frame_association_id)
        expected = self._expected_frames.get(arm_input.side)
        if expected is not None and arm_input.reference_frame != expected:
            raise ValueError(f"{arm_input.side} input reference frame must be {expected!r}")
        expected_tracked = self._expected_tracked_frames.get(arm_input.side)
        if expected_tracked is not None and arm_input.tracked_frame != expected_tracked:
            raise ValueError(f"{arm_input.side} tracked frame must be {expected_tracked!r}")
        current = _pose(arm_input.pose, "arm_input.pose")
        # This is the complete input-reference -> robot-base rotation.  For
        # the historical TJ_arm_control relative mapper, callers configure it
        # as ``R_world_to_chest(side) @ R_input_to_robot``; keeping the full
        # side-specific matrix here makes the old chest-frame behavior an
        # explicit, testable configuration rather than a hidden branch.
        input_rotation = Rotation.from_matrix(self._rotations[arm_input.side])
        reference_rotation = Rotation.from_quat(reference[3:])
        current_rotation = Rotation.from_quat(current[3:])
        delta = input_rotation * (current_rotation * reference_rotation.inv()) * input_rotation.inv()
        home = self._control_home[arm_input.side]
        position = home[:3] + self._rotations[arm_input.side] @ (current[:3] - reference[:3])
        orientation = delta * Rotation.from_quat(home[3:])
        pose = _compose(np.concatenate((position, orientation.as_quat())), self._control_to_ik[arm_input.side])
        return MappedArmPose(arm_input.side, pose, True, "relative_home", "relative_home_v2", arm_input.frame_association_id)


def create_arm_pose_mapper(backend: str, config: Any) -> ArmPoseMapper:
    if backend == "head_palm_direct":
        return HeadPalmDirectMapper(config)
    if backend == "head_direct":
        return HeadDirectMapper(config)
    if backend == "direct_pose":
        return DirectPoseMapper(config)
    if backend == "relative_home":
        return RelativeHomeMapper(config)
    raise ValueError("unknown arm pose mapping backend: " + str(backend))


__all__ = ["ArmPoseMapper", "DirectPoseMapper", "HeadDirectMapper", "HeadPalmDirectMapper", "MappedArmPose", "RelativeHomeMapper", "create_arm_pose_mapper"]
