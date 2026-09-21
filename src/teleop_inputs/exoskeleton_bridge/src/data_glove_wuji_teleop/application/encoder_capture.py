"""数据手套编码器流的有界短窗口采样。"""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass

from ..adapters.glove.encoder_stream import EncoderConnection, EncoderFrame


@dataclass(frozen=True)
class CaptureWindow:
    """一次短连接采到的帧和板端流身份。"""

    frames: tuple[EncoderFrame, ...]
    mapping: tuple[int, ...]
    zeroed: bool
    channels: int = 21
    range_min: float = 0.0
    range_max: float = 360.0


def read_encoder_window(
    host: str,
    port: int,
    duration_s: float,
    *,
    timeout: float = 5.0,
    connection_factory: Callable[[], EncoderConnection] | None = None,
) -> CaptureWindow:
    """用新短连接按板端时戳采样，并以本机单调时钟硬截止。"""

    duration = float(duration_s)
    connection_timeout = float(timeout)
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("采样时间必须大于 0")
    if not math.isfinite(connection_timeout) or connection_timeout <= 0.0:
        raise ValueError("连接超时必须是正有限数值")

    connection = (
        connection_factory()
        if connection_factory is not None
        else EncoderConnection.connect(host, port=port, timeout=connection_timeout)
    )
    with connection:
        hard_deadline = time.monotonic() + max(
            duration * 2.0,
            duration + connection_timeout,
        )
        first = connection.read_frame(deadline_monotonic=hard_deadline)
        frames = [first]
        remote_deadline_ns = (
            first.timestamp_ns + int(duration * 1_000_000_000)
        )
        while frames[-1].timestamp_ns < remote_deadline_ns:
            if time.monotonic() >= hard_deadline:
                raise RuntimeError(
                    "编码器采样未在本机截止时间内达到目标窗口；"
                    "板端时间戳可能停滞"
                )
            current = connection.read_frame(
                deadline_monotonic=hard_deadline,
            )
            previous = frames[-1]
            if current.sequence == previous.sequence:
                raise RuntimeError("编码器帧 sequence 重复")
            if current.timestamp_ns < previous.timestamp_ns:
                raise RuntimeError("编码器帧时间戳倒退")
            frames.append(current)
        return CaptureWindow(
            frames=tuple(frames),
            mapping=tuple(connection.cs_by_joint),
            zeroed=connection.zeroed,
            channels=connection.channels,
            range_min=connection.range_min,
            range_max=connection.range_max,
        )
