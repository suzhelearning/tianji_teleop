from typing import List
import time
import xml.etree.ElementTree as ET

import numpy as np
import numpy.typing as npt
import pinocchio as pin


class RobotWrapper:
    """Pinocchio robot wrapper for forward kinematics."""

    def __init__(self, urdf_path: str, hand_side: str = None):
        # Create robot model and data
        self.model: pin.Model = pin.buildModelFromUrdf(urdf_path)
        self.data: pin.Data = self.model.createData()

        if self.model.nv != self.model.nq:
            raise NotImplementedError("Cannot handle robot with special joint.")
        self._dof_joint_names = self._load_dof_joint_names(urdf_path, self.model)

        # Store hand side for frame name resolution
        self.hand_side = hand_side.lower() if hand_side else None

        # Timing statistics for FK and Jacobian
        self._timing_enabled = False
        self._fk_time_sum = 0.0
        self._jacobian_time_sum = 0.0
        self._fk_call_count = 0
        self._jacobian_call_count = 0

    def enable_timing(self, enabled: bool = True):
        """Enable or disable timing statistics."""
        self._timing_enabled = enabled
        if enabled:
            self.reset_timing()

    def reset_timing(self):
        """Reset timing statistics."""
        self._fk_time_sum = 0.0
        self._jacobian_time_sum = 0.0
        self._fk_call_count = 0
        self._jacobian_call_count = 0

    def get_timing_stats(self):
        """Get timing statistics.

        Returns:
            dict with keys:
                - fk_avg_us: average FK time in microseconds
                - jacobian_avg_us: average Jacobian time in microseconds
                - fk_call_count: number of FK calls
                - jacobian_call_count: number of Jacobian calls
        """
        fk_avg = (self._fk_time_sum / self._fk_call_count * 1e6) if self._fk_call_count > 0 else 0
        jac_avg = (self._jacobian_time_sum / self._jacobian_call_count * 1e6) if self._jacobian_call_count > 0 else 0
        return {
            'fk_avg_us': fk_avg,
            'jacobian_avg_us': jac_avg,
            'fk_call_count': self._fk_call_count,
            'jacobian_call_count': self._jacobian_call_count,
        }

    @property
    def dof_joint_names(self) -> List[str]:
        """Return names of joints with DOF > 0."""
        return list(self._dof_joint_names)

    @staticmethod
    def _load_dof_joint_names(urdf_path: str, model) -> tuple[str, ...]:
        """Resolve qpos names without accessing Pinocchio vector properties.

        Isaac Sim can load Pinocchio without registering Python converters for
        ``model.nqs`` and ``model.names``. URDF provides the names, while the
        scalar ``getJointId()``, ``joint.nq`` and ``joint.idx_q`` APIs provide
        the authoritative qpos ordering without crossing either vector binding.
        """

        ordered: list[tuple[int, str]] = []
        total_nq = 0
        for joint_spec in ET.parse(urdf_path).getroot().findall(".//joint"):
            if joint_spec.get("type") == "fixed":
                continue
            name = joint_spec.get("name")
            if not name:
                raise ValueError("URDF contains a movable joint without a name")
            joint_id = int(model.getJointId(name))
            if joint_id <= 0 or joint_id >= int(model.njoints):
                raise RuntimeError(f"Movable URDF joint '{name}' is missing from Pinocchio model")
            joint = model.joints[joint_id]
            nq = int(joint.nq)
            if nq <= 0:
                continue
            ordered.append((int(joint.idx_q), name))
            total_nq += nq

        ordered.sort(key=lambda item: item[0])
        names = tuple(name for _, name in ordered)
        if total_nq != int(model.nq) or len(set(names)) != len(names):
            raise RuntimeError(
                f"URDF/Pinocchio DOF mapping mismatch: names={len(names)}, "
                f"resolved_nq={total_nq}, model_nq={model.nq}"
            )
        return names

    @property
    def joint_limits(self):
        """Return joint limits as (lower, upper) pairs."""
        lower = self.model.lowerPositionLimit
        upper = self.model.upperPositionLimit
        return np.stack([lower, upper], axis=1)

    def get_link_index(self, name: str) -> int:
        """Get frame index by name, trying both unprefixed and prefixed variants.

        Args:
            name: Frame name without prefix (e.g., "palm_link", "finger1_tip_link")
        """
        for candidate in [name, f"{self.hand_side}_{name}"]:
            idx = self.model.getFrameId(candidate, pin.BODY)
            if idx < self.model.nframes:
                return idx
        raise RuntimeError(
            f"Frame '{name}' not found. "
            f"Available: {[self.model.frames[i].name for i in range(self.model.nframes)]}"
        )

    def get_actuated_qpos_index(self, link_name: str) -> int:
        """Return the qpos index of the joint that actuates ``link_name``.

        A link's parent joint is the joint directly above it in the kinematic
        chain (the joint whose URDF ``<child>`` is this link). For the wuji
        finger chain ``link1->link2->link3->link4``, ``finger{i}_link3``'s parent
        joint is the PIP joint and ``finger{i}_link4``'s is the DIP joint.

        Resolving the qpos slot this way (instead of hardcoding indices) keeps the
        mapping correct when a custom URDF declares joints in a different order or
        with a non-uniform DOF layout. Raises if the link is unknown or its parent
        joint is not a single-DOF joint.
        """
        frame_id = self.get_link_index(link_name)
        # 本项目固定使用 Pinocchio 3.x；不求值已弃用的 Frame.parent。
        frame = self.model.frames[frame_id]
        joint_id = frame.parentJoint
        joint = self.model.joints[joint_id]
        if joint.nq != 1:
            raise RuntimeError(
                f"Parent joint of '{link_name}' ('{self.model.names[joint_id]}') has "
                f"nq={joint.nq}, expected a single-DOF (revolute/prismatic) joint."
            )
        return int(joint.idx_q)

    def compute_forward_kinematics(self, qpos: npt.NDArray):
        """Compute forward kinematics for all links."""
        pin.forwardKinematics(self.model, self.data, qpos)

    def get_link_pose(self, link_id: int) -> npt.NDArray:
        """Get link pose as 4x4 homogeneous matrix."""
        pose: pin.SE3 = pin.updateFramePlacement(self.model, self.data, link_id)
        return pose.homogeneous

    def compute_single_link_local_jacobian(self, qpos, link_id: int) -> npt.NDArray:
        """Compute Jacobian for a single link."""
        J = pin.computeFrameJacobian(self.model, self.data, qpos, link_id)
        return J

    def compute_all_jacobians_batch(self, qpos: npt.NDArray, link_indices: List[int]) -> npt.NDArray:
        """Batch compute position Jacobians for multiple links.

        This is more efficient than calling compute_single_link_local_jacobian
        multiple times because it uses computeJointJacobians once.

        Args:
            qpos: Joint positions
            link_indices: List of frame indices

        Returns:
            jacobians: (num_links, 3, nq) position Jacobians in world frame
        """
        qpos = np.asarray(qpos, dtype=np.float64)

        # Compute all joint Jacobians at once (updates data.J internally)
        pin.computeJointJacobians(self.model, self.data, qpos)
        # Update all frame placements
        pin.updateFramePlacements(self.model, self.data)

        jacobians = []
        for idx in link_indices:
            # getFrameJacobian reuses computed joint Jacobians (faster than computeFrameJacobian)
            J_local = pin.getFrameJacobian(self.model, self.data, idx, pin.LOCAL)
            # Get rotation to transform to world frame
            R = self.data.oMf[idx].rotation
            # Only take position part (3, nq) and transform to world frame
            J_world_pos = R @ J_local[:3, :]
            jacobians.append(J_world_pos)

        return np.stack(jacobians, axis=0)

    def compute_fk_batch(self, qpos: npt.NDArray, link_indices: List[int]) -> npt.NDArray:
        """Batch compute FK positions for multiple links.

        Args:
            qpos: Joint positions
            link_indices: List of frame indices

        Returns:
            positions: (num_links * 3,) flattened positions
        """
        qpos = np.asarray(qpos, dtype=np.float64)
        pin.forwardKinematics(self.model, self.data, qpos)
        pin.updateFramePlacements(self.model, self.data)

        positions = []
        for idx in link_indices:
            pos = self.data.oMf[idx].translation
            positions.append(pos)

        return np.concatenate(positions)
