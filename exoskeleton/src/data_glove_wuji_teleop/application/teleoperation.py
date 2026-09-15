"""数据手套到后端无关 Wuji 目标消息的实时遥操作流程。"""

from __future__ import annotations

import math
import signal
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

from ..adapters.glove.encoder_stream import (
    EncoderConnection,
    EncoderFrame,
    ProtocolError,
)
from ..domain.hand_target import HandTarget
from ..profiles.dataglove.mapping import CalibrationProfile


class TargetPublisher(Protocol):
    """仿真或真机目标输出后端需要实现的最小接口。"""

    def publish(self, target: HandTarget) -> None: ...


@dataclass(frozen=True)
class TeleoperationOptions:
    """与具体目标输出后端无关的遥操作运行参数。"""

    calibration_file: Path
    glove_host: str
    glove_port: int
    rate_hz: float
    reconnect_seconds: float
    duration_seconds: float
    print_interval_seconds: float
    dry_run: bool
    stop_on_disconnect: bool = False


def build_hand_target(
    profile: CalibrationProfile,
    frame: EncoderFrame,
) -> HandTarget:
    """把一帧手套角度转换为后端无关的灵巧手目标。"""

    return HandTarget(
        hand=profile.hand,
        values=tuple(profile.map_to_dof_values(frame.angles_deg)),
        source="dataglove",
        sequence=int(frame.sequence),
        timestamp_ns=int(frame.timestamp_ns),
        dropped=int(frame.dropped),
    )


def _zero_target(profile: CalibrationProfile) -> HandTarget:
    return HandTarget(
        hand=profile.hand,
        values=(0.0,) * 20,
        source="dataglove_failsafe",
        sequence=-1,
        timestamp_ns=0,
        dropped=0,
    )


def _publish_zero_burst(
    publisher: TargetPublisher | None,
    profile: CalibrationProfile,
) -> None:
    if publisher is None:
        return
    target = _zero_target(profile)
    for _ in range(5):
        try:
            publisher.publish(target)
        except Exception:
            # 真机看门狗可能已完成回零并关闭后端，退出清理不能覆盖原始错误。
            break
        time.sleep(0.02)


def run_teleoperation(
    options: TeleoperationOptions,
    publisher: TargetPublisher | None,
    *,
    stop_event: threading.Event | None = None,
    install_signal_handlers: bool = True,
) -> int:
    """运行共享遥操作主循环，并将目标交给注入的后端。"""

    profile = CalibrationProfile.load(options.calibration_file)
    if profile.hand not in ("left", "right"):
        raise ValueError(f"不支持的标定 hand：{profile.hand}")
    if not options.dry_run and publisher is None:
        raise ValueError("非 dry-run 模式必须提供目标输出后端")

    stop = stop_event if stop_event is not None else threading.Event()

    def request_stop(_signum=None, _frame=None) -> None:
        stop.set()

    if install_signal_handlers:
        for name in ("SIGINT", "SIGTERM", "SIGHUP"):
            signum = getattr(signal, name, None)
            if signum is not None:
                signal.signal(signum, request_stop)

    period_ns = max(1, int(1_000_000_000 / max(options.rate_hz, 1.0)))
    started_at = time.monotonic()
    had_valid_stream = False
    last_log = started_at
    sent = 0
    received = 0
    try:
        while not stop.is_set():
            if (
                options.duration_seconds > 0.0
                and time.monotonic() - started_at >= options.duration_seconds
            ):
                break
            connection = None
            try:
                connection = EncoderConnection.connect(
                    options.glove_host,
                    port=options.glove_port,
                    timeout=5.0,
                )
                profile.validate_stream(
                    channels=connection.channels,
                    cs_by_joint=connection.cs_by_joint,
                )
                print(
                    f"[手套:{profile.hand}] 已连接 "
                    f"{options.glove_host}:{options.glove_port} "
                    f"zeroed={str(connection.zeroed).lower()} "
                    f"hand={profile.hand}"
                )
                had_valid_stream = True
                next_publish_ns: int | None = None
                while not stop.is_set():
                    if (
                        options.duration_seconds > 0.0
                        and time.monotonic() - started_at
                        >= options.duration_seconds
                    ):
                        stop.set()
                        break
                    frame = connection.read_frame()
                    received += 1
                    if next_publish_ns is None:
                        next_publish_ns = frame.timestamp_ns
                    if frame.timestamp_ns < next_publish_ns:
                        continue
                    while next_publish_ns <= frame.timestamp_ns:
                        next_publish_ns += period_ns
                    target = build_hand_target(profile, frame)
                    if publisher is not None:
                        publisher.publish(target)
                    sent += 1
                    now = time.monotonic()
                    if (
                        options.dry_run
                        and now - last_log >= options.print_interval_seconds
                    ):
                        target = profile.map_to_finger_target(frame.angles_deg)
                        printable = [
                            [round(math.degrees(value), 1) for value in finger]
                            for finger in target
                        ]
                        print(
                            f"[dry] rx={received} sent={sent} "
                            f"dropped={frame.dropped} target_deg={printable}"
                        )
                        last_log = now
            except (EOFError, OSError, ProtocolError, ValueError) as exc:
                if not stop.is_set():
                    action = (
                        "等待后重连"
                        if publisher is None
                        else "发布零目标并重连"
                    )
                    print(
                        f"[手套:{profile.hand}] 连接中断：{exc}；{action}",
                        file=sys.stderr,
                    )
                if had_valid_stream:
                    _publish_zero_burst(publisher, profile)
                if had_valid_stream and options.stop_on_disconnect:
                    stop.set()
                else:
                    stop.wait(options.reconnect_seconds)
            finally:
                if connection is not None:
                    connection.close()
    finally:
        _publish_zero_burst(publisher, profile)
    suffix = "dry-run 未发布数据" if publisher is None else "已发布零目标"
    print(
        f"[结束:{profile.hand}] received={received} sent={sent}，{suffix}"
    )
    return 0
