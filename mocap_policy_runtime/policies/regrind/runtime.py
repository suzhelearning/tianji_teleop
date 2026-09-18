"""Measured-feedback Regrind inference, independent of transport and hardware."""
from __future__ import annotations

from pathlib import Path
import time

import numpy as np

from ...data.reference import RegrindReference, load_reference
from ...types import TargetFrame
from .actor import action_to_targets, build_observation, infer, load_actor


class RegrindPolicy:
    """Run one training-exact 50 Hz policy tick per :meth:`step`.

    Inputs and output wrists are in the reference's capture/training world.
    Calibration into robot world and hardware limits belong to the adapter.
    ``reset`` and every ``step`` require explicit measured hand joints; this
    reusable API never substitutes a previous target for measured feedback.
    """

    rate_hz = 50.0

    def __init__(
        self, model: str | Path, reference: str | Path | RegrindReference,
        device: str = "auto", start_frame: int = 0, reference_speed: float = 1.0,
    ) -> None:
        self.reference = reference if isinstance(reference, RegrindReference) else load_reference(Path(reference))
        if isinstance(start_frame, bool) or not isinstance(start_frame, (int, np.integer)):
            raise ValueError("start_frame must be an integer")
        if not 0 <= start_frame < self.reference.frame_count - 1:
            raise ValueError(f"start_frame must be in [0, {self.reference.frame_count - 2}]")
        if not np.isfinite(reference_speed) or not 0.0 < reference_speed <= 1.0:
            raise ValueError("reference_speed must be finite and in (0, 1]")
        self.start_frame = int(start_frame)
        self.reference_speed = float(reference_speed)
        self.actor, self.mean, self.variance, self.iteration = load_actor(Path(model), device=device)
        self.reference_progress = float(self.start_frame)
        self.last_action = np.zeros(26, dtype=np.float64)
        self.raw_action = np.zeros(26, dtype=np.float64)
        self.inference_ms = 0.0
        self._previous_wrist: np.ndarray | None = None
        self._previous_joints: np.ndarray | None = None
        self._ticks = 0

    @property
    def device(self) -> str:
        return str(self.mean.device)

    @property
    def complete(self) -> bool:
        return self.reference_progress >= self.reference.frame_count - 1

    @staticmethod
    def _pose(value: np.ndarray, label: str) -> np.ndarray:
        pose = np.asarray(value, dtype=np.float64)
        if pose.shape != (7,) or not np.isfinite(pose).all():
            raise ValueError(f"{label} must contain seven finite xyzw pose values")
        if np.linalg.norm(pose[3:]) < 1e-8:
            raise ValueError(f"{label} quaternion cannot be zero")
        return pose

    @staticmethod
    def _joints(value: np.ndarray) -> np.ndarray:
        joints = np.asarray(value, dtype=np.float64)
        if joints.shape != (20,) or not np.isfinite(joints).all():
            raise ValueError("measured hand_joints must contain 20 finite radians")
        return joints

    def reset(self, wrist_xyzw: np.ndarray, joints: np.ndarray) -> None:
        wrist = self._pose(wrist_xyzw, "wrist_xyzw")
        measured = self._joints(joints)
        self._previous_wrist = wrist.copy()
        self._previous_joints = measured.copy()
        self.reference_progress = float(self.start_frame)
        self.last_action.fill(0.0)
        self.raw_action.fill(0.0)
        self.inference_ms = 0.0
        self._ticks = 0

    def observe(
        self, wrist_xyzw: np.ndarray, joints: np.ndarray, *, clear_action: bool = False,
    ) -> None:
        """Refresh measured history during a hold without advancing reference time."""
        if self._previous_wrist is None or self._previous_joints is None:
            raise RuntimeError("reset(wrist_xyzw, measured_joints) is required before observe")
        wrist = self._pose(wrist_xyzw, "wrist_xyzw")
        measured = self._joints(joints)
        self._previous_wrist = wrist.copy()
        self._previous_joints = measured.copy()
        if clear_action:
            self.last_action.fill(0.0)

    def step(
        self, wrist_xyzw: np.ndarray, object_xyzw: np.ndarray, hand_joints: np.ndarray,
    ) -> TargetFrame:
        if self._previous_wrist is None or self._previous_joints is None:
            raise RuntimeError("reset(wrist_xyzw, measured_joints) is required before step")
        if self.complete:
            raise RuntimeError("Regrind reference is complete; reset before stepping again")
        wrist = self._pose(wrist_xyzw, "wrist_xyzw")
        hammer = self._pose(object_xyzw, "object_xyzw")
        joints = self._joints(hand_joints)
        reference = self.reference
        index = int(self.reference_progress)
        observation = build_observation(
            object_pos=hammer[:3], object_quat_wxyz=np.roll(hammer[3:], 1),
            previous_wrist_pos=self._previous_wrist[:3], wrist_pos=wrist[:3],
            previous_wrist_quat_wxyz=np.roll(self._previous_wrist[3:], 1),
            wrist_quat_wxyz=np.roll(wrist[3:], 1),
            previous_joints=self._previous_joints, joints=joints,
            last_action=self.last_action, phase=index / (reference.frame_count - 1),
            base_wrist_pos=reference.wrist_pos[index],
            base_wrist_quat_wxyz=reference.wrist_quat_wxyz[index],
            base_joints=reference.joints[index],
        )
        started_ns = time.perf_counter_ns()
        raw_action = infer(self.actor, self.mean, self.variance, observation)
        self.inference_ms = (time.perf_counter_ns() - started_ns) / 1e6
        position, quaternion, target_joints = action_to_targets(
            raw_action, reference.wrist_pos[index], reference.wrist_quat_wxyz[index],
            reference.joints[index],
        )
        self.raw_action = raw_action
        self.last_action = np.clip(raw_action, -1.0, 1.0)
        self._previous_wrist = wrist.copy()
        self._previous_joints = joints.copy()
        self.reference_progress = min(
            float(reference.frame_count - 1), self.reference_progress + self.reference_speed,
        )
        result = TargetFrame(
            time_s=self._ticks / self.rate_hz, index=index, complete=self.complete,
            wrist_poses={"right": np.concatenate((position, np.roll(quaternion, -1)))},
            hand_joints={"right": target_joints},
            object_poses={"hammer": np.concatenate((
                reference.object_pos[index], np.roll(reference.object_quat_wxyz[index], -1)))},
        )
        self._ticks += 1
        return result
