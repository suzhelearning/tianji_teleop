"""Sampling primitives with explicit invalidity and end-of-recording behavior."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ..types import TargetFrame
from .geometry import interpolate_pose


@dataclass
class Track:
    time_s: np.ndarray
    values: np.ndarray
    valid: np.ndarray
    interpolation: str = "hold"

    def __post_init__(self) -> None:
        if self.interpolation == "pose":
            self.values = self.values.copy()
            self.values[self.valid, 3:7] /= np.linalg.norm(self.values[self.valid, 3:7], axis=1)[:, None]

    def sample(self, time_s: float) -> np.ndarray | None:
        index = int(np.searchsorted(self.time_s, time_s, side="right") - 1)
        if index < 0 or not self.valid[index]:
            return None
        if self.interpolation == "hold" or index == len(self.time_s) - 1 or time_s == self.time_s[index]:
            return self.values[index].copy()
        # Never bridge missing tracking, even when a later valid sample exists.
        if not self.valid[index + 1]:
            return None
        fraction = (time_s - self.time_s[index]) / (self.time_s[index + 1] - self.time_s[index])
        if self.interpolation == "pose":
            return interpolate_pose(self.values[index], self.values[index + 1], float(fraction))
        return (1.0 - fraction) * self.values[index] + fraction * self.values[index + 1]


class Trajectory:
    """In-memory trajectory; no file handles remain open after loading.

    Time zero is the first recorded timestamp across replayed streams. Samples
    clamp to [0, duration_s]; index is the last global timeline row at that time.
    Missing/invalid streams are absent from TargetFrame dictionaries, not zeros.
    """

    def __init__(
        self, path: Path, format: str, timeline_s: np.ndarray,
        tracks: dict[str, dict[str, Track]], *, space: str,
        metadata: dict[str, object] | None = None,
    ) -> None:
        if timeline_s.ndim != 1 or timeline_s.size == 0:
            raise ValueError("recording contains no replayable rows")
        self.path = path
        self.format = format
        self.timeline_s = timeline_s
        self.tracks = tracks
        self.space = space
        self.metadata = metadata or {}

    @property
    def duration_s(self) -> float:
        return float(self.timeline_s[-1])

    @property
    def frame_count(self) -> int:
        return int(self.timeline_s.size)

    def summary(self) -> dict[str, object]:
        return {
            "path": str(self.path), "format": self.format, "space": self.space,
            "frame_count": self.frame_count, "duration_s": self.duration_s,
            "time_origin": "first_replayed_timestamp", "index": "preceding_global_timeline_row",
            "streams": {
                field: {side: {"rows": len(track.time_s), "valid_rows": int(track.valid.sum()),
                               "interpolation": track.interpolation}
                        for side, track in streams.items()}
                for field, streams in self.tracks.items()
            },
            **self.metadata,
        }

    def sample(self, time_s: float) -> TargetFrame:
        requested = float(time_s)
        if not np.isfinite(requested):
            raise ValueError("time_s must be finite")
        elapsed = float(np.clip(requested, 0.0, self.duration_s))
        index = int(np.searchsorted(self.timeline_s, elapsed, side="right") - 1)
        fields = {}
        for field, streams in self.tracks.items():
            fields[field] = {}
            for side, track in streams.items():
                value = track.sample(elapsed)
                if value is not None:
                    fields[field][side] = value
        return TargetFrame(time_s=elapsed, index=index, complete=requested >= self.duration_s,
                           space=self.space, **fields)


def from_timestamp_tracks(
    path: Path, format: str, streams: dict[str, dict[str, tuple[np.ndarray, np.ndarray, np.ndarray, str]]],
    *, space: str, metadata: dict[str, object] | None = None,
) -> Trajectory:
    timestamps = [item[0] for fields in streams.values() for item in fields.values() if item[0].size]
    if not timestamps:
        raise ValueError(f"{format} recording contains no replayable rows")
    timeline = np.unique(np.concatenate(timestamps))
    origin = int(timeline[0])
    tracks = {
        field: {side: Track((time - origin).astype(np.float64) / 1e9, values, valid, interpolation)
                for side, (time, values, valid, interpolation) in fields.items() if time.size}
        for field, fields in streams.items()
    }
    return Trajectory(path, format, (timeline - origin).astype(np.float64) / 1e9, tracks,
                      space=space, metadata=metadata)
