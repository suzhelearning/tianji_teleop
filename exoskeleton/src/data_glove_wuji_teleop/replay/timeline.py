"""由录制时间戳驱动的编码器/视频共用回放时钟。"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ReplayPosition:
    """某一墙钟时刻对应的录制位置。"""

    frame_index: int
    recording_timestamp_ns: int
    loop_count: int
    finished: bool


class ReplayTimeline:
    """将经过时间按速度映射到单调录制时间戳。"""

    def __init__(
        self,
        encoder_timestamps_ns: np.ndarray,
        *,
        speed: float = 1.0,
        loop: bool = False,
    ) -> None:
        timestamps = np.asarray(encoder_timestamps_ns, dtype=np.int64)
        if timestamps.ndim != 1 or timestamps.size == 0:
            raise ValueError("回放时间戳必须是非空一维数组")
        if timestamps.size > 1 and np.any(np.diff(timestamps) <= 0):
            raise ValueError("回放时间戳必须严格递增")
        selected_speed = float(speed)
        if not math.isfinite(selected_speed) or selected_speed <= 0.0:
            raise ValueError("回放速度必须是正有限数值")
        self._timestamps = timestamps
        self._speed = selected_speed
        self._loop = bool(loop)
        self._start_ns = int(timestamps[0])
        self._span_ns = int(timestamps[-1] - timestamps[0])
        self._frame_period_ns = (
            max(1, int(np.median(np.diff(timestamps))))
            if timestamps.size > 1
            else 1
        )
        self._duration_ns = self._span_ns + self._frame_period_ns

    @property
    def duration_seconds(self) -> float:
        return self._duration_ns / 1e9

    def position_at(self, elapsed_wall_seconds: float) -> ReplayPosition:
        elapsed = float(elapsed_wall_seconds)
        if not math.isfinite(elapsed) or elapsed < 0.0:
            raise ValueError("墙钟经过时间必须是非负有限数值")
        playback_ns = int(elapsed * self._speed * 1e9)
        if self._loop:
            loop_count, offset_ns = divmod(playback_ns, self._duration_ns)
            finished = False
        else:
            loop_count = 0
            offset_ns = min(playback_ns, self._span_ns)
            finished = playback_ns >= self._duration_ns
        target_ns = self._start_ns + min(offset_ns, self._span_ns)
        index = frame_index_at_timestamp(self._timestamps, target_ns)
        if index is None:
            index = 0
        return ReplayPosition(
            frame_index=index,
            recording_timestamp_ns=target_ns,
            loop_count=int(loop_count),
            finished=finished,
        )


def frame_index_at_timestamp(
    timestamps_ns: np.ndarray,
    target_timestamp_ns: int,
) -> int | None:
    """返回不晚于目标时刻的最新帧，序列尚未开始时返回 None。"""

    timestamps = np.asarray(timestamps_ns, dtype=np.int64)
    if timestamps.ndim != 1:
        raise ValueError("时间戳必须是一维数组")
    if timestamps.size == 0 or int(target_timestamp_ns) < int(timestamps[0]):
        return None
    return int(np.searchsorted(timestamps, int(target_timestamp_ns), side="right") - 1)
