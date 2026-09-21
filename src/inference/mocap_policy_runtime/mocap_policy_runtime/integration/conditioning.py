from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation as R


def _finite_vector(values, *, size: int, label: str) -> np.ndarray:
    vector = np.asarray(values, dtype=np.float64)
    if vector.shape != (size,) or not np.isfinite(vector).all():
        raise ValueError(f"{label} must contain {size} finite values")
    return vector.copy()


def _limited_norm(vector: np.ndarray, maximum: float) -> tuple[np.ndarray, bool]:
    norm = float(np.linalg.norm(vector))
    if norm <= maximum or norm < 1.0e-12:
        return vector, False
    return vector * (maximum / norm), True


@dataclass(frozen=True)
class TargetConditioningSettings:
    """末端目标进入 IK 前的尺度、工作空间和动态约束。"""

    rate_hz: float
    translation_gain: np.ndarray
    rotation_gain: float
    workspace_relative_radii_m: np.ndarray
    workspace_soft_zone_ratio: float
    maximum_linear_speed_m_s: float
    maximum_angular_speed_rad_s: float
    maximum_linear_acceleration_m_s2: float
    maximum_angular_acceleration_rad_s2: float

    def __post_init__(self) -> None:
        translation_gain = _finite_vector(
            self.translation_gain, size=3, label="translation_gain"
        )
        workspace_radii = _finite_vector(
            self.workspace_relative_radii_m,
            size=3,
            label="workspace_relative_radii_m",
        )
        if self.rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")
        if np.any(translation_gain <= 0.0):
            raise ValueError("translation_gain must be positive")
        if np.any(workspace_radii <= 0.0):
            raise ValueError("workspace radii must be positive")
        if not 0.0 < self.rotation_gain <= 1.0:
            raise ValueError("rotation_gain must be in (0, 1]")
        if not 0.0 < self.workspace_soft_zone_ratio < 1.0:
            raise ValueError("workspace_soft_zone_ratio must be in (0, 1)")
        for label, value in (
            ("maximum_linear_speed_m_s", self.maximum_linear_speed_m_s),
            ("maximum_angular_speed_rad_s", self.maximum_angular_speed_rad_s),
            (
                "maximum_linear_acceleration_m_s2",
                self.maximum_linear_acceleration_m_s2,
            ),
            (
                "maximum_angular_acceleration_rad_s2",
                self.maximum_angular_acceleration_rad_s2,
            ),
        ):
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{label} must be positive and finite")
        object.__setattr__(self, "translation_gain", translation_gain)
        object.__setattr__(self, "workspace_relative_radii_m", workspace_radii)


@dataclass(frozen=True)
class TargetConditioningDiagnostics:
    requested_workspace_utilization: float
    workspace_utilization: float
    workspace_soft_limited: bool
    requested_linear_speed_m_s: float
    applied_linear_speed_m_s: float
    linear_speed_limited: bool
    linear_acceleration_limited: bool
    requested_angular_speed_rad_s: float
    applied_angular_speed_rad_s: float
    angular_speed_limited: bool
    angular_acceleration_limited: bool

    def as_dict(self) -> dict[str, float | bool]:
        return {
            "requested_workspace_utilization": (
                self.requested_workspace_utilization
            ),
            "workspace_utilization": self.workspace_utilization,
            "workspace_soft_limited": self.workspace_soft_limited,
            "requested_linear_speed_m_s": self.requested_linear_speed_m_s,
            "applied_linear_speed_m_s": self.applied_linear_speed_m_s,
            "linear_speed_limited": self.linear_speed_limited,
            "linear_acceleration_limited": self.linear_acceleration_limited,
            "requested_angular_speed_rad_s": (
                self.requested_angular_speed_rad_s
            ),
            "applied_angular_speed_rad_s": self.applied_angular_speed_rad_s,
            "angular_speed_limited": self.angular_speed_limited,
            "angular_acceleration_limited": (
                self.angular_acceleration_limited
            ),
        }


class TargetConditioner:
    """将相对腕部位姿变成连续、有限且渐近接近边界的 IK 目标。"""

    def __init__(
        self,
        initial_position,
        initial_quaternion,
        settings: TargetConditioningSettings,
    ):
        self._origin_position = _finite_vector(
            initial_position, size=3, label="initial_position"
        )
        initial_quaternion = _finite_vector(
            initial_quaternion, size=4, label="initial_quaternion"
        )
        if np.linalg.norm(initial_quaternion) < 1.0e-8:
            raise ValueError("initial_quaternion must be nonzero")
        self._origin_rotation = R.from_quat(initial_quaternion)
        self._settings = settings
        self.reset()

    def reset(self) -> None:
        self._position = self._origin_position.copy()
        self._rotation = self._origin_rotation
        self._linear_velocity = np.zeros(3, dtype=np.float64)
        self._angular_velocity = np.zeros(3, dtype=np.float64)

    def synchronize(self, position, quaternion) -> None:
        """Resume limiting from a measured hold pose without a target jump."""
        self._position = _finite_vector(position, size=3, label="hold position")
        quaternion = _finite_vector(quaternion, size=4, label="hold quaternion")
        if np.linalg.norm(quaternion) < 1.0e-8:
            raise ValueError("hold quaternion must be nonzero")
        self._rotation = R.from_quat(quaternion)
        self._linear_velocity.fill(0.0)
        self._angular_velocity.fill(0.0)

    def condition(
        self, position, quaternion
    ) -> tuple[np.ndarray, np.ndarray, TargetConditioningDiagnostics]:
        requested_position = _finite_vector(
            position, size=3, label="target position"
        )
        requested_quaternion = _finite_vector(
            quaternion, size=4, label="target quaternion"
        )
        if np.linalg.norm(requested_quaternion) < 1.0e-8:
            raise ValueError("target quaternion must be nonzero")

        settings = self._settings
        relative = (
            requested_position - self._origin_position
        ) * settings.translation_gain
        utilization = float(
            np.linalg.norm(relative / settings.workspace_relative_radii_m)
        )
        requested_utilization = utilization
        workspace_limited = utilization > settings.workspace_soft_zone_ratio
        if workspace_limited and utilization > 1.0e-12:
            soft = settings.workspace_soft_zone_ratio
            mapped_utilization = soft + (1.0 - soft) * (
                1.0
                - np.exp(
                    -(utilization - soft) / (1.0 - soft)
                )
            )
            relative *= mapped_utilization / utilization
            utilization = float(mapped_utilization)
        desired_position = self._origin_position + relative

        desired_rotation = self._origin_rotation * R.from_rotvec(
            (
                self._origin_rotation.inv()
                * R.from_quat(requested_quaternion)
            ).as_rotvec()
            * settings.rotation_gain
        )

        rate = settings.rate_hz
        requested_linear_velocity = (
            desired_position - self._position
        ) * rate
        requested_linear_speed = float(
            np.linalg.norm(requested_linear_velocity)
        )
        desired_linear_velocity, linear_speed_limited = _limited_norm(
            requested_linear_velocity,
            settings.maximum_linear_speed_m_s,
        )
        linear_acceleration = (
            desired_linear_velocity - self._linear_velocity
        ) * rate
        limited_linear_acceleration, linear_acceleration_limited = (
            _limited_norm(
                linear_acceleration,
                settings.maximum_linear_acceleration_m_s2,
            )
        )
        self._linear_velocity += limited_linear_acceleration / rate
        linear_step = self._linear_velocity / rate
        remaining_position = desired_position - self._position
        if np.linalg.norm(linear_step) >= np.linalg.norm(remaining_position):
            linear_step = remaining_position
            self._linear_velocity = linear_step * rate
        self._position += linear_step

        rotation_delta = self._rotation.inv() * desired_rotation
        requested_angular_velocity = rotation_delta.as_rotvec() * rate
        requested_angular_speed = float(
            np.linalg.norm(requested_angular_velocity)
        )
        desired_angular_velocity, angular_speed_limited = _limited_norm(
            requested_angular_velocity,
            settings.maximum_angular_speed_rad_s,
        )
        angular_acceleration = (
            desired_angular_velocity - self._angular_velocity
        ) * rate
        limited_angular_acceleration, angular_acceleration_limited = (
            _limited_norm(
                angular_acceleration,
                settings.maximum_angular_acceleration_rad_s2,
            )
        )
        self._angular_velocity += limited_angular_acceleration / rate
        angular_step = self._angular_velocity / rate
        if np.linalg.norm(angular_step) >= np.linalg.norm(rotation_delta.as_rotvec()):
            angular_step = rotation_delta.as_rotvec()
            self._angular_velocity = angular_step * rate
        self._rotation = self._rotation * R.from_rotvec(angular_step)

        diagnostics = TargetConditioningDiagnostics(
            requested_workspace_utilization=requested_utilization,
            workspace_utilization=utilization,
            workspace_soft_limited=workspace_limited,
            requested_linear_speed_m_s=requested_linear_speed,
            applied_linear_speed_m_s=float(
                np.linalg.norm(self._linear_velocity)
            ),
            linear_speed_limited=linear_speed_limited,
            linear_acceleration_limited=linear_acceleration_limited,
            requested_angular_speed_rad_s=requested_angular_speed,
            applied_angular_speed_rad_s=float(
                np.linalg.norm(self._angular_velocity)
            ),
            angular_speed_limited=angular_speed_limited,
            angular_acceleration_limited=angular_acceleration_limited,
        )
        return (
            self._position.copy(),
            self._rotation.as_quat(),
            diagnostics,
        )
