"""语义目标到真机固件顺序的映射、限位与运动约束。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

from ...domain.hand_target import DOF_ORDER, HandTarget
from ...profiles.joint_mapping import load_joint_mapping


FIRMWARE_DOF_ORDER: tuple[str, ...] = (
    "thumb1_flex",
    "thumb1_abd",
    "thumb2_flex",
    "thumb3_flex",
    "index1_flex",
    "index1_abd",
    "index2_flex",
    "index3_flex",
    "middle1_flex",
    "middle1_abd",
    "middle2_flex",
    "middle3_flex",
    "ring1_flex",
    "ring1_abd",
    "ring2_flex",
    "ring3_flex",
    "pinky1_flex",
    "pinky1_abd",
    "pinky2_flex",
    "pinky3_flex",
)


def _finite_vector(
    values,
    *,
    name: str,
) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != 20:
        raise ValueError(f"{name} 必须包含 20 个关节，实际 {len(result)}")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} 必须全部是有限数值")
    return result


@dataclass(frozen=True)
class FirmwareCommandMapper:
    """将 HandTarget 映射成拇指到小指的固件 20 轴顺序。"""

    generation: str
    hand: str
    scales: tuple[float, ...]
    offsets: tuple[float, ...]
    lower: tuple[float, ...]
    upper: tuple[float, ...]

    @classmethod
    def from_config(
        cls,
        config_path: str | Path,
        *,
        generation: str,
        hand: str,
        lower,
        upper,
    ) -> "FirmwareCommandMapper":
        rules = load_joint_mapping(
            config_path,
            expected_hand=hand,
            expected_generation=generation,
        )
        by_dof = {rule.dof_name: rule for rule in rules}
        lower_values = _finite_vector(lower, name="lower")
        upper_values = _finite_vector(upper, name="upper")
        if any(lo > hi for lo, hi in zip(lower_values, upper_values)):
            raise ValueError("关节 lower 不能大于 upper")
        if any(
            not lo <= 0.0 <= hi
            for lo, hi in zip(lower_values, upper_values)
        ):
            raise ValueError("全部真机关节限位都必须包含零位")
        return cls(
            generation=generation,
            hand=hand,
            scales=tuple(by_dof[name].scale for name in FIRMWARE_DOF_ORDER),
            offsets=tuple(
                by_dof[name].offset_rad for name in FIRMWARE_DOF_ORDER
            ),
            lower=lower_values,
            upper=upper_values,
        )

    def map_target(self, target: HandTarget) -> tuple[float, ...]:
        if target.hand != self.hand:
            raise ValueError(
                f"{self.hand} 真机不能接收 {target.hand} 目标"
            )
        semantic_values = _finite_vector(target.values, name="HandTarget")
        by_dof = dict(zip(DOF_ORDER, semantic_values))
        requested = (
            by_dof[name] * scale + offset
            for name, scale, offset in zip(
                FIRMWARE_DOF_ORDER,
                self.scales,
                self.offsets,
            )
        )
        return tuple(
            min(hi, max(lo, value))
            for value, lo, hi in zip(
                requested,
                self.lower,
                self.upper,
            )
        )


class AngleLowPassFilter:
    """对固件顺序的 20 路目标角执行一阶低通滤波。"""

    def __init__(
        self,
        *,
        initial_positions,
        cutoff_hz: float,
        initial_time: float,
    ):
        self._positions = _finite_vector(
            initial_positions,
            name="filter initial_positions",
        )
        self._cutoff_hz = float(cutoff_hz)
        self._time = float(initial_time)
        if (
            not math.isfinite(self._cutoff_hz)
            or self._cutoff_hz <= 0.0
            or not math.isfinite(self._time)
        ):
            raise ValueError("滤波截止频率和初始时间必须是正有限数值")

    @property
    def positions(self) -> tuple[float, ...]:
        return self._positions

    def step(self, requested, *, now: float) -> tuple[float, ...]:
        target = _finite_vector(requested, name="filter requested")
        current_time = float(now)
        if not math.isfinite(current_time) or current_time <= self._time:
            raise ValueError("滤波时间必须严格递增")
        dt = current_time - self._time
        alpha = 1.0 - math.exp(-2.0 * math.pi * self._cutoff_hz * dt)
        self._positions = tuple(
            current + alpha * (goal - current)
            for current, goal in zip(self._positions, target)
        )
        self._time = current_time
        return self._positions


class AdaptiveTargetFilter:
    """静止时抑制噪声、快速运动时提高带宽的目标角滤波器。"""

    def __init__(
        self,
        *,
        initial_positions,
        deadband_rad: float,
        min_cutoff_hz: float,
        max_cutoff_hz: float,
        velocity_for_max_cutoff: float,
        initial_time: float,
    ):
        self._positions = _finite_vector(
            initial_positions,
            name="adaptive filter initial_positions",
        )
        self._raw_positions = self._positions
        self._deadband_rad = float(deadband_rad)
        self._min_cutoff_hz = float(min_cutoff_hz)
        self._max_cutoff_hz = float(max_cutoff_hz)
        self._velocity_for_max_cutoff = float(velocity_for_max_cutoff)
        self._time = float(initial_time)
        values = (
            self._deadband_rad,
            self._min_cutoff_hz,
            self._max_cutoff_hz,
            self._velocity_for_max_cutoff,
            self._time,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("实时滤波参数必须是有限数值")
        if self._deadband_rad < 0.0:
            raise ValueError("实时滤波死区不能为负数")
        if (
            self._min_cutoff_hz <= 0.0
            or self._max_cutoff_hz < self._min_cutoff_hz
            or self._velocity_for_max_cutoff <= 0.0
        ):
            raise ValueError("实时滤波截止频率和速度阈值无效")

    @property
    def positions(self) -> tuple[float, ...]:
        return self._positions

    def step(self, requested, *, now: float) -> tuple[float, ...]:
        target = _finite_vector(requested, name="adaptive filter requested")
        current_time = float(now)
        if not math.isfinite(current_time) or current_time <= self._time:
            raise ValueError("实时滤波时间必须严格递增")
        dt = current_time - self._time
        filtered: list[float] = []
        for current, previous_raw, goal in zip(
            self._positions,
            self._raw_positions,
            target,
        ):
            if abs(goal - current) <= self._deadband_rad:
                filtered.append(current)
                continue
            speed = abs(goal - previous_raw) / dt
            blend = min(1.0, speed / self._velocity_for_max_cutoff)
            cutoff = self._min_cutoff_hz + blend * (
                self._max_cutoff_hz - self._min_cutoff_hz
            )
            alpha = 1.0 - math.exp(-2.0 * math.pi * cutoff * dt)
            filtered.append(current + alpha * (goal - current))
        self._positions = tuple(filtered)
        self._raw_positions = target
        self._time = current_time
        return self._positions


class RealtimeVelocityLimiter:
    """实时遥操作用单步速度限制；不引入跨帧加速度轨迹。"""

    def __init__(
        self,
        *,
        initial_positions,
        lower,
        upper,
        max_velocity: float,
        initial_time: float,
    ):
        self._lower = _finite_vector(lower, name="realtime lower")
        self._upper = _finite_vector(upper, name="realtime upper")
        if any(lo > hi for lo, hi in zip(self._lower, self._upper)):
            raise ValueError("关节 lower 不能大于 upper")
        self._positions = _finite_vector(
            initial_positions,
            name="realtime initial_positions",
        )
        if any(
            not lo <= value <= hi
            for value, lo, hi in zip(
                self._positions,
                self._lower,
                self._upper,
            )
        ):
            raise ValueError("实时控制初始关节位置超出真机限位")
        self._max_velocity = float(max_velocity)
        self._time = float(initial_time)
        if (
            not math.isfinite(self._max_velocity)
            or self._max_velocity <= 0.0
            or not math.isfinite(self._time)
        ):
            raise ValueError("实时速度限制和初始时间必须是正有限数值")

    @property
    def positions(self) -> tuple[float, ...]:
        return self._positions

    def step(self, requested, *, now: float) -> tuple[float, ...]:
        target = _finite_vector(requested, name="realtime requested")
        current_time = float(now)
        if not math.isfinite(current_time) or current_time <= self._time:
            raise ValueError("实时速度限制时间必须严格递增")
        max_step = self._max_velocity * (current_time - self._time)
        next_positions = []
        for current, goal, lo, hi in zip(
            self._positions,
            target,
            self._lower,
            self._upper,
        ):
            bounded_goal = min(hi, max(lo, goal))
            delta = min(max_step, max(-max_step, bounded_goal - current))
            next_positions.append(current + delta)
        self._positions = tuple(next_positions)
        self._time = current_time
        return self._positions


class MotionLimiter:
    """以梯形积分限制 20 轴位置目标的速度与加速度。"""

    def __init__(
        self,
        *,
        initial_positions,
        lower,
        upper,
        max_velocity: float,
        max_acceleration: float,
        initial_time: float,
    ):
        self._lower = _finite_vector(lower, name="lower")
        self._upper = _finite_vector(upper, name="upper")
        if any(lo > hi for lo, hi in zip(self._lower, self._upper)):
            raise ValueError("关节 lower 不能大于 upper")
        positions = _finite_vector(
            initial_positions,
            name="initial_positions",
        )
        if any(
            not lo <= value <= hi
            for value, lo, hi in zip(
                positions,
                self._lower,
                self._upper,
            )
        ):
            raise ValueError("初始关节位置超出真机限位")
        self._positions = positions
        self._velocities = (0.0,) * 20
        self._max_velocity = float(max_velocity)
        self._max_acceleration = float(max_acceleration)
        self._time = float(initial_time)
        if (
            not math.isfinite(self._max_velocity)
            or self._max_velocity <= 0.0
            or not math.isfinite(self._max_acceleration)
            or self._max_acceleration <= 0.0
            or not math.isfinite(self._time)
        ):
            raise ValueError("运动限制和初始时间必须是正有限数值")

    @property
    def positions(self) -> tuple[float, ...]:
        return self._positions

    def step(self, requested, *, now: float) -> tuple[float, ...]:
        target = _finite_vector(requested, name="requested")
        current_time = float(now)
        if not math.isfinite(current_time) or current_time <= self._time:
            raise ValueError("运动限制时间必须严格递增")
        dt = current_time - self._time
        target = tuple(
            min(hi, max(lo, value))
            for value, lo, hi in zip(
                target,
                self._lower,
                self._upper,
            )
        )

        next_positions: list[float] = []
        next_velocities: list[float] = []
        max_delta_velocity = self._max_acceleration * dt
        for position, velocity, goal, lo, hi in zip(
            self._positions,
            self._velocities,
            target,
            self._lower,
            self._upper,
        ):
            desired_velocity = (goal - position) / dt
            desired_velocity = min(
                self._max_velocity,
                max(-self._max_velocity, desired_velocity),
            )
            velocity_delta = desired_velocity - velocity
            velocity_delta = min(
                max_delta_velocity,
                max(-max_delta_velocity, velocity_delta),
            )
            next_velocity = velocity + velocity_delta
            next_position = position + (velocity + next_velocity) * 0.5 * dt

            direction = goal - position
            if direction == 0.0 or direction * (goal - next_position) <= 0.0:
                next_position = goal
                next_velocity = 0.0
            clipped_position = min(hi, max(lo, next_position))
            if clipped_position != next_position:
                next_velocity = 0.0
            next_positions.append(clipped_position)
            next_velocities.append(next_velocity)

        self._positions = tuple(next_positions)
        self._velocities = tuple(next_velocities)
        self._time = current_time
        return self._positions
