"""对官方 wuji-sdk 一代/二代控制 API 的薄封装。"""

from __future__ import annotations

import math
import threading
import time
from typing import Protocol


DEFAULT_V2_KP = 8.0
DEFAULT_V2_KD = 0.2
MAX_V2_KP = 10.0
MAX_V2_KD = 1.0


def validate_v2_mit_gains(kp: float, kd: float) -> tuple[float, float]:
    """校验项目允许写入 Hand2 的保守 MIT 增益范围。"""

    proportional = float(kp)
    derivative = float(kd)
    if (
        not math.isfinite(proportional)
        or not 0.0 < proportional <= MAX_V2_KP
    ):
        raise ValueError(f"Hand2 Kp 必须在 (0, {MAX_V2_KP:g}] 内")
    if (
        not math.isfinite(derivative)
        or not 0.0 < derivative <= MAX_V2_KD
    ):
        raise ValueError(f"Hand2 Kd 必须在 (0, {MAX_V2_KD:g}] 内")
    return proportional, derivative


def _positions(values, *, name: str = "positions") -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != 20:
        raise ValueError(f"{name} 必须包含 20 个关节，实际 {len(result)}")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} 必须全部是有限数值")
    return result


def _ordered_entries(entries, *, nids, name: str):
    expected = tuple(int(nid) for nid in nids)
    by_nid = {}
    for entry in entries:
        nid = int(entry.nid)
        if nid in by_nid:
            raise RuntimeError(f"{name} 包含重复 nid={nid}")
        by_nid[nid] = entry
    if set(by_nid) != set(expected):
        raise RuntimeError(
            f"{name} 关节 nid 不完整："
            f"expected={list(expected)} actual={sorted(by_nid)}"
        )
    return tuple(by_nid[nid] for nid in expected)


class WujiSdkDriver(Protocol):
    """安全发布层使用的最小真机驱动合同。"""

    generation: str
    hand: str
    serial: str
    lower: tuple[float, ...]
    upper: tuple[float, ...]

    def prepare(self) -> None: ...

    def read_positions(self) -> tuple[float, ...]: ...

    def send_positions(self, positions) -> None: ...

    def emergency_stop(self) -> None: ...

    def close(self) -> None: ...


class _WujiV1Driver:
    generation = "v1"

    def __init__(
        self,
        *,
        hand: str,
        serial: str,
        device_name: str,
        manager,
        sdk_module,
        device,
        effort_limit: float,
        lowpass_cutoff_hz: float,
    ):
        self.hand = hand
        self.serial = serial
        self._device_name = device_name
        self._manager = manager
        self._sdk = sdk_module
        self._device = device
        self._effort_limit = float(effort_limit)
        self._lowpass_cutoff_hz = float(lowpass_cutoff_hz)
        upper, lower = device.get_soft_limits()
        self.upper = _positions(upper, name="v1 upper")
        self.lower = _positions(lower, name="v1 lower")
        if any(lo > hi for lo, hi in zip(self.lower, self.upper)):
            raise ValueError("v1 固件软限位 lower 不能大于 upper")
        self._controller_context = None
        self._controller = None
        self._publisher = None
        self._prepared = False
        self._closed = False

    def prepare(self) -> None:
        if self._prepared:
            return
        self._device.clear_all_faults()
        self._device.set_all_effort_limit(self._effort_limit)
        self._device.enable()
        try:
            self._controller_context = self._device.realtime_controller(
                self._sdk.LowPass(cutoff_hz=self._lowpass_cutoff_hz)
            )
            self._controller = self._controller_context.__enter__()
            self._publisher = self._device.joint_command().publish()
        except Exception:
            if self._controller_context is not None:
                context = self._controller_context
                self._controller_context = None
                self._controller = None
                context.__exit__(None, None, None)
            self._device.disable()
            raise
        self._prepared = True

    def _require_controller(self):
        if (
            not self._prepared
            or self._controller is None
            or self._publisher is None
        ):
            raise RuntimeError("v1 真机驱动尚未 prepare")
        if self._closed:
            raise RuntimeError("v1 真机驱动已经关闭")
        return self._controller

    def read_positions(self) -> tuple[float, ...]:
        controller = self._require_controller()
        return _positions(
            controller.get_actual_position(),
            name="v1 actual_positions",
        )

    def send_positions(self, positions) -> None:
        self._require_controller()
        values = _positions(positions)
        commands = [
            self._sdk.JointCommand(position, 0.0, 0.0)
            for position in values
        ]
        self._publisher.send(commands)

    def emergency_stop(self) -> None:
        if not self._closed:
            self._device.disable()

    def close(self) -> None:
        if self._closed:
            return
        try:
            if self._publisher is not None:
                self._publisher.close()
        finally:
            try:
                if self._controller_context is not None:
                    self._controller_context.__exit__(None, None, None)
            finally:
                try:
                    self._device.disable()
                finally:
                    self._manager.disconnect(self._device_name)
                    self._closed = True


class _WujiV2Driver:
    generation = "v2"

    def __init__(
        self,
        *,
        hand: str,
        serial: str,
        device_name: str,
        manager,
        sdk_module,
        device,
        effort_limit: float,
        kp: float,
        kd: float,
        enable_timeout: float,
        lower,
        upper,
    ):
        self.hand = hand
        self.serial = serial
        self._device_name = device_name
        self._manager = manager
        self._sdk = sdk_module
        self._device = device
        self._effort_limit = float(effort_limit)
        self._kp = float(kp)
        self._kd = float(kd)
        self._enable_timeout = float(enable_timeout)
        self.lower = _positions(lower, name="v2 lower")
        self.upper = _positions(upper, name="v2 upper")
        if any(lo > hi for lo, hi in zip(self.lower, self.upper)):
            raise ValueError("v2 模型限位 lower 不能大于 upper")
        # joint_id_from_bus_node() returns the zero-based command joint_id,
        # while feedback entry.nid uses the one-based bus node encoding.
        self._feedback_nids = tuple(
            bus_id * 5 + node_id
            for bus_id in range(5)
            for node_id in range(1, 5)
        )
        if len(set(self._feedback_nids)) != 20:
            raise RuntimeError("v2 反馈包含重复的关节 nid 映射")
        self._publisher = None
        self._state_subscription = None
        self._state_condition = threading.Condition()
        self._latest_state_frame = None
        self._prepared = False
        self._closed = False

    def _store_latest_state(self, frame) -> None:
        """持续消费 SDK 状态流，并以覆盖方式只保留最新完整帧。"""

        if frame is None or len(frame.joints) != 20:
            return
        with self._state_condition:
            self._latest_state_frame = frame
            self._state_condition.notify_all()

    def prepare(self) -> None:
        if self._prepared:
            return
        self._device.clear_fault()
        online_count = int(self._device.online_joints_count().get())
        if online_count != 20:
            raise RuntimeError(
                f"v2 真机必须 20 个关节全部在线，实际 {online_count}"
            )
        self._device.effort_limit().set(self._effort_limit)
        self._device.mit_params().set((self._kp, self._kd))
        self._device.enable()

        diagnostics = self._device.joint_diagnostics().subscribe()
        enabled = False
        try:
            deadline = time.monotonic() + self._enable_timeout
            while time.monotonic() < deadline:
                frame = diagnostics.recv()
                if (
                    frame is not None
                    and len(frame.joints) == 20
                    and all(
                        entry.status_word.ext_state == 2
                        for entry in _ordered_entries(
                            frame.joints,
                            nids=self._feedback_nids,
                            name="v2 joint_diagnostics",
                        )
                    )
                ):
                    enabled = True
                    break
                time.sleep(0.05)
        finally:
            diagnostics.close()
        if not enabled:
            self._device.disable()
            raise RuntimeError("v2 真机使能超时，未全部进入 Enabled")

        try:
            self._publisher = self._device.joint_command().publish()
            self._state_subscription = (
                self._device.joint_states().subscribe_with_callback(
                    self._store_latest_state
                )
            )
        except Exception:
            try:
                if self._publisher is not None:
                    self._publisher.close()
                    self._publisher = None
            finally:
                self._device.disable()
            raise
        self._prepared = True

    def _require_prepared(self) -> None:
        if (
            not self._prepared
            or self._publisher is None
            or self._state_subscription is None
        ):
            raise RuntimeError("v2 真机驱动尚未 prepare")
        if self._closed:
            raise RuntimeError("v2 真机驱动已经关闭")

    def read_positions(self) -> tuple[float, ...]:
        self._require_prepared()
        deadline = time.monotonic() + self._enable_timeout
        with self._state_condition:
            while self._latest_state_frame is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise RuntimeError("v2 真机状态回读超时")
                self._state_condition.wait(timeout=remaining)
            frame = self._latest_state_frame
        ordered = _ordered_entries(
            frame.joints,
            nids=self._feedback_nids,
            name="v2 joint_states",
        )
        return _positions(
            (entry.position for entry in ordered),
            name="v2 actual_positions",
        )

    def send_positions(self, positions) -> None:
        self._require_prepared()
        values = _positions(positions)
        commands = [
            self._sdk.JointCommand(position, 0.0, 0.0)
            for position in values
        ]
        self._publisher.send(commands)

    def emergency_stop(self) -> None:
        if not self._closed:
            self._device.emergency_stop()

    def close(self) -> None:
        if self._closed:
            return
        try:
            if self._publisher is not None:
                self._publisher.close()
        finally:
            try:
                if self._state_subscription is not None:
                    self._state_subscription.close()
            finally:
                try:
                    self._device.disable()
                finally:
                    self._manager.disconnect(self._device_name)
                    self._closed = True


def _load_sdk():
    try:
        import wuji_sdk
    except ImportError as exc:
        raise RuntimeError(
            "缺少 wuji-sdk，请在项目 Pixi 环境中运行"
        ) from exc
    return wuji_sdk


def _normalized_hand_side(value) -> str:
    candidate = getattr(value, "value", value)
    normalized = str(candidate).strip().lower()
    return normalized.rsplit(".", 1)[-1]


def _validate_side(device, *, generation: str, hand: str) -> None:
    actual_raw = (
        device.handedness_name()
        if generation == "v1"
        else device.handedness().get()
    )
    actual = _normalized_hand_side(actual_raw)
    if actual != hand:
        raise ValueError(
            f"设备左右手不匹配：请求 {hand}，实际 {actual}"
        )


def open_wuji_sdk_driver(
    *,
    generation: str,
    hand: str,
    serial: str,
    manager=None,
    sdk_module=None,
    effort_limit: float = 1.5,
    lowpass_cutoff_hz: float = 5.0,
    kp: float = DEFAULT_V2_KP,
    kd: float = DEFAULT_V2_KD,
    enable_timeout: float = 5.0,
    lower=None,
    upper=None,
) -> WujiSdkDriver:
    """按 SN 连接指定代际和侧别，但在 prepare 前不使能电机。"""

    if generation not in ("v1", "v2"):
        raise ValueError("generation 必须是 v1 或 v2")
    if hand not in ("left", "right"):
        raise ValueError("hand 必须是 left 或 right")
    if not serial:
        raise ValueError("真机控制必须显式提供 serial")
    if generation == "v2":
        kp, kd = validate_v2_mit_gains(kp, kd)

    sdk = sdk_module or _load_sdk()
    sdk_manager = manager or sdk.SdkManager.instance()
    expected_type = (
        sdk.DeviceType.WujiHand
        if generation == "v1"
        else sdk.DeviceType.WujiHand2
    )
    matching = [device for device in sdk_manager.scan() if device.sn == serial]
    if len(matching) != 1:
        raise RuntimeError(f"未唯一发现 SN={serial} 的设备")
    if matching[0].device_type != expected_type:
        raise ValueError(
            f"SN={serial} 不是 Wuji {generation} 设备"
        )

    device_name = f"real_{generation}_{hand}"
    connect_kwargs = {}
    if generation == "v2":
        connect_kwargs["options"] = sdk.ConnectOptions(enable_bridge=False)
    device = sdk_manager.connect(
        sn=serial,
        device_name=device_name,
        **connect_kwargs,
    )
    expected_class = sdk.WujiHand if generation == "v1" else sdk.WujiHand2
    if not isinstance(device, expected_class):
        sdk_manager.disconnect(device_name)
        raise TypeError(f"SDK 返回了错误设备类型：{type(device).__name__}")
    try:
        _validate_side(
            device,
            generation=generation,
            hand=hand,
        )
        if generation == "v1":
            return _WujiV1Driver(
                hand=hand,
                serial=serial,
                device_name=device_name,
                manager=sdk_manager,
                sdk_module=sdk,
                device=device,
                effort_limit=effort_limit,
                lowpass_cutoff_hz=lowpass_cutoff_hz,
            )
        if lower is None or upper is None:
            raise ValueError("v2 真机必须提供官方模型关节限位")
        return _WujiV2Driver(
            hand=hand,
            serial=serial,
            device_name=device_name,
            manager=sdk_manager,
            sdk_module=sdk,
            device=device,
            effort_limit=effort_limit,
            kp=kp,
            kd=kd,
            enable_timeout=enable_timeout,
            lower=lower,
            upper=upper,
        )
    except BaseException:
        sdk_manager.disconnect(device_name)
        raise
