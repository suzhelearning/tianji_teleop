"""Forward-horizontal TCP X/Z alignment for the bare-hand simulation only."""
from dataclasses import replace
import numpy as np

from .mapping import OptionalHeightMapping


class OptionalXZMapping(OptionalHeightMapping):
    def __init__(self, forward):
        super().__init__()
        self.forward = forward
        self.offsets = {}

    def request_calibration(self, now_ns, *, idle):
        if not idle or self.calibration.state == "collecting":
            return False
        q = np.zeros((2, 7))
        q[:, 1] = -np.pi / 2
        # Query only: never sends this posture to the simulation or hardware.
        poses = self.forward(q)
        self.reference = {}
        for side in ("left", "right"):
            pose = np.asarray(poses[side]["achieved_pose"], dtype=float)
            if pose.shape != (7,) or not np.isfinite(pose).all():
                raise ValueError("invalid horizontal TCP reference")
            self.reference[side] = self.mapper._rotations[side].T @ pose[:3]
        self.calibration.begin(now_ns)
        return True

    def add(self, observation, now_ns):
        # Sample uncalibrated mapped TCP, not already-offset output. This makes
        # repeated C idempotent and retains the fixed head-relative orientation.
        mapped = self.mapper.map(observation)
        pose = None
        if mapped.valid:
            pose = mapped.pose.copy()
            pose[:3] = self.mapper._rotations[observation.side].T @ pose[:3]
        self.calibration.add(replace(observation, pose=pose, valid=mapped.valid), now_ns)

    def tick(self, now_ns):
        means = self.calibration.tick(now_ns)
        if means is None:
            return False
        pending = {}
        for side, samples in self.calibration.samples.items():
            average = np.mean([v[1] for v in samples], axis=0)
            offset = self.reference[side] - average
            offset[1] = 0  # Preserve lateral placement and all orientation axes.
            if not np.isfinite(offset).all() or np.max(np.abs(offset)) > 1:
                self.calibration.fail("X/Z offset exceeds 1 m; check horizontal pose")
                return False
            pending[side] = offset
        self.offsets = pending
        self.calibration.accept(means)
        return True

    def map(self, observation):
        result = self.mapper.map(observation)
        if result.valid and observation.side in self.offsets:
            pose = result.pose.copy()
            pose[:3] += self.mapper._rotations[observation.side] @ self.offsets[observation.side]
            return replace(result, pose=pose, mapping_version="head_palm_xz_v1")
        return result

    def status(self):
        return dict(super().status(), mapping="head_palm_xz", reference_posture_deg=[0,-90,0,0,0,0,0],
                    offsets_head_axes_m={s: v.tolist() for s,v in self.offsets.items()})
