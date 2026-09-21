"""带回零、运动约束、仿真守卫和看门狗的真机发布器。"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Callable

from ...domain.hand_target import HandTarget
from .command_safety import (
    AdaptiveTargetFilter,
    AngleLowPassFilter,
    FirmwareCommandMapper,
    MotionLimiter,
    RealtimeVelocityLimiter,
)
from .wuji_sdk_driver import WujiSdkDriver


class HardwareSafetyError(RuntimeError):
    """真机安全条件失效，发布器已经停止。"""


@dataclass(frozen=True)
class HardwareSafetyOptions:
    """真机运动、回零和看门狗参数。"""

    max_velocity: float = 1.5
    max_acceleration: float = 8.0
    input_filter_hz: float = 5.0
    home_rate_hz: float = 100.0
    home_tolerance: float = 0.01
    home_timeout: float = 8.0
    command_timeout: float = 0.5
    watchdog_period: float = 0.05
    tracking_mode: str = "legacy"
    realtime_max_velocity: float = 4.0

    def __post_init__(self) -> None:
        values = (
            self.max_velocity,
            self.max_acceleration,
            self.input_filter_hz,
            self.home_rate_hz,
            self.home_tolerance,
            self.home_timeout,
            self.command_timeout,
            self.watchdog_period,
            self.realtime_max_velocity,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("真机安全参数必须全部是正有限数值")
        if self.tracking_mode not in ("legacy", "realtime"):
            raise ValueError("tracking_mode 必须是 legacy 或 realtime")


class SafeHardwarePublisher:
    """实现 application.TargetPublisher，并拥有完整真机生命周期。"""

    def __init__(
        self,
        *,
        driver: WujiSdkDriver,
        mapper: FirmwareCommandMapper,
        simulation_check: Callable[[], object],
        options: HardwareSafetyOptions,
        limiter: MotionLimiter | RealtimeVelocityLimiter,
        angle_filter: AngleLowPassFilter | AdaptiveTargetFilter | None,
        clock: Callable[[], float],
        sleep: Callable[[float], None],
    ):
        self._driver = driver
        self._mapper = mapper
        self._simulation_check = simulation_check
        self._options = options
        self._limiter = limiter
        self._angle_filter = angle_filter
        self._clock = clock
        self._sleep = sleep
        self._lock = threading.RLock()
        self._watchdog_stop = threading.Event()
        self._watchdog_thread: threading.Thread | None = None
        self._last_publish_time: float | None = None
        self._closed = False
        self._failure: HardwareSafetyError | None = None

    @classmethod
    def open(
        cls,
        *,
        driver: WujiSdkDriver,
        mapper: FirmwareCommandMapper,
        simulation_check: Callable[[], object],
        options: HardwareSafetyOptions | None = None,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        start_watchdog: bool = True,
        arm_command_timeout: bool = True,
    ) -> "SafeHardwarePublisher":
        """检查仿真，清错使能，从当前位置受限速回零后返回。"""

        safety = options or HardwareSafetyOptions()
        try:
            if (
                mapper.generation != driver.generation
                or mapper.hand != driver.hand
            ):
                raise ValueError(
                    "真机 driver 与命令 mapper 的代际或侧别不一致"
                )
            simulation_check()
            driver.prepare()
            initial_positions = driver.read_positions()
            limiter = MotionLimiter(
                initial_positions=initial_positions,
                lower=mapper.lower,
                upper=mapper.upper,
                max_velocity=safety.max_velocity,
                max_acceleration=safety.max_acceleration,
                initial_time=clock(),
            )
            publisher = cls(
                driver=driver,
                mapper=mapper,
                simulation_check=simulation_check,
                options=safety,
                limiter=limiter,
                angle_filter=None,
                clock=clock,
                sleep=sleep,
            )
            with publisher._lock:
                publisher._move_home_locked()
                reset_time = clock()
                publisher._reset_tracking_locked(
                    initial_positions=(0.0,) * 20,
                    reset_time=reset_time,
                )
                if arm_command_timeout:
                    publisher._last_publish_time = reset_time
            if start_watchdog:
                publisher._start_watchdog()
            return publisher
        except BaseException:
            try:
                driver.emergency_stop()
            finally:
                driver.close()
            raise

    def _move_home_locked(self) -> None:
        zeros = (0.0,) * 20
        period = 1.0 / self._options.home_rate_hz
        deadline = self._clock() + self._options.home_timeout
        while True:
            if max(abs(value) for value in self._limiter.positions) <= (
                self._options.home_tolerance
            ):
                self._driver.send_positions(zeros)
                actual = self._driver.read_positions()
                if max(abs(value) for value in actual) <= (
                    self._options.home_tolerance
                ):
                    return
            if self._clock() >= deadline:
                raise HardwareSafetyError("真机回零超时")
            self._sleep(period)
            if max(abs(value) for value in self._limiter.positions) > (
                self._options.home_tolerance
            ):
                command = self._limiter.step(
                    zeros,
                    now=self._clock(),
                )
                self._driver.send_positions(command)

    def _start_watchdog(self) -> None:
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            name=f"wuji-watchdog-{self._driver.generation}-{self._driver.hand}",
            daemon=True,
        )
        self._watchdog_thread.start()

    def arm_command_timeout(self) -> None:
        """重置运动状态，并从当前时刻开始要求目标帧持续到达。"""

        with self._lock:
            if self._failure is not None:
                raise self._failure
            if self._closed:
                raise HardwareSafetyError("真机发布器已经关闭")
            reset_time = self._clock()
            current_positions = self._limiter.positions
            self._reset_tracking_locked(
                initial_positions=current_positions,
                reset_time=reset_time,
            )
            self._last_publish_time = reset_time

    def _reset_tracking_locked(
        self,
        *,
        initial_positions,
        reset_time: float,
    ) -> None:
        if self._options.tracking_mode == "realtime":
            self._limiter = RealtimeVelocityLimiter(
                initial_positions=initial_positions,
                lower=self._mapper.lower,
                upper=self._mapper.upper,
                max_velocity=self._options.realtime_max_velocity,
                initial_time=reset_time,
            )
            self._angle_filter = AdaptiveTargetFilter(
                initial_positions=initial_positions,
                deadband_rad=math.radians(0.5),
                min_cutoff_hz=2.0,
                max_cutoff_hz=20.0,
                velocity_for_max_cutoff=1.0,
                initial_time=reset_time,
            )
            return
        self._limiter = MotionLimiter(
            initial_positions=initial_positions,
            lower=self._mapper.lower,
            upper=self._mapper.upper,
            max_velocity=self._options.max_velocity,
            max_acceleration=self._options.max_acceleration,
            initial_time=reset_time,
        )
        self._angle_filter = AngleLowPassFilter(
            initial_positions=initial_positions,
            cutoff_hz=self._options.input_filter_hz,
            initial_time=reset_time,
        )

    def _watchdog_loop(self) -> None:
        while not self._watchdog_stop.wait(self._options.watchdog_period):
            reason: str | None = None
            try:
                self._simulation_check()
            except Exception as exc:
                reason = f"仿真守卫失效：{exc}"
            with self._lock:
                if self._closed:
                    return
                if (
                    reason is None
                    and self._last_publish_time is not None
                    and self._clock() - self._last_publish_time
                    > self._options.command_timeout
                ):
                    reason = "手套目标发布超时"
            if reason is not None:
                self._safety_shutdown(reason, try_home=True)
                return

    def _safety_shutdown(self, reason: str, *, try_home: bool) -> None:
        with self._lock:
            if self._closed:
                return
            home_failed = False
            if try_home:
                try:
                    self._move_home_locked()
                except Exception:
                    home_failed = True
            if home_failed or not try_home:
                try:
                    self._driver.emergency_stop()
                except Exception:
                    pass
            try:
                self._driver.close()
            finally:
                self._closed = True
                self._failure = HardwareSafetyError(reason)
                self._watchdog_stop.set()

    def publish(self, target: HandTarget) -> None:
        with self._lock:
            if self._failure is not None:
                raise self._failure
            if self._closed:
                raise HardwareSafetyError("真机发布器已经关闭")
            try:
                self._simulation_check()
            except Exception as exc:
                reason = f"仿真守卫失效：{exc}"
                self._safety_shutdown(reason, try_home=True)
                raise HardwareSafetyError(reason) from exc
            try:
                requested = self._mapper.map_target(target)
                now = self._clock()
                if self._angle_filter is None:
                    raise HardwareSafetyError("真机角度滤波器尚未初始化")
                filtered = self._angle_filter.step(requested, now=now)
                command = self._limiter.step(filtered, now=now)
                self._driver.send_positions(command)
                self._last_publish_time = now
            except HardwareSafetyError:
                raise
            except Exception as exc:
                self._safety_shutdown(
                    f"真机发布失败：{exc}",
                    try_home=False,
                )
                raise HardwareSafetyError(
                    f"真机发布失败：{exc}"
                ) from exc

    def close(self) -> None:
        self._watchdog_stop.set()
        thread = self._watchdog_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        with self._lock:
            if self._closed:
                return
            try:
                self._move_home_locked()
            except Exception:
                try:
                    self._driver.emergency_stop()
                except Exception:
                    pass
            finally:
                try:
                    self._driver.close()
                finally:
                    self._closed = True
