"""Observation-only, scale-normalized 21-point gesture baseline.

This is not a learned/native PICO gesture classifier or an emergency stop.
Labels never authorize motion. Temporal stability/release belongs to the
separate operator binding; callers must clear previous labels on tracking loss.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class GestureThresholds:
    pinch_enter: float = .22
    pinch_leave: float = .34
    open_enter: float = .92
    open_leave: float = .85
    fist_enter: float = .55
    fist_leave: float = .65

    def __post_init__(self):
        values = vars(self).values()
        if (any(type(v) not in (int, float) or not math.isfinite(v) for v in values) or
                not 0 < self.pinch_enter < self.pinch_leave or
                not 0 < self.fist_enter < self.fist_leave < self.open_leave < self.open_enter <= 1):
            raise ValueError('invalid gesture hysteresis thresholds')


@dataclass(frozen=True)
class GestureObservation:
    gesture: str
    available: bool
    pinch_ratio: float | None = None
    finger_extension: tuple[float, ...] = ()


def classify_hand(keypoints_m, *, valid, previous='unknown', thresholds=None):
    """Classify finite canonical geometry; input must use wrist + 5×4 ordering.

    Extension is MCP→tip chord divided by the three segment lengths, for
    index/middle/ring/little fingers. Pinch is thumb→index tip distance divided
    by wrist→middle MCP length. Hysteresis is explicit, with no hidden state.
    """
    if type(valid) is not bool or previous not in ('unknown', 'open', 'fist', 'pinch'):
        raise ValueError('validity and previous observation label required')
    t = thresholds if thresholds is not None else GestureThresholds()
    if not isinstance(t, GestureThresholds):
        raise ValueError('validated gesture thresholds required')
    points = np.asarray(keypoints_m, dtype=float)
    unavailable = GestureObservation('unknown', False)
    if not valid or points.shape != (21, 3) or not np.isfinite(points).all():
        return unavailable
    scale = float(np.linalg.norm(points[9] - points[0]))
    if scale < 1e-8:
        return unavailable
    extensions = []
    for base in (5, 9, 13, 17):
        lengths = np.linalg.norm(np.diff(points[base:base+4], axis=0), axis=1)
        if np.any(lengths < scale * 1e-6):
            return unavailable
        extensions.append(float(np.linalg.norm(points[base+3] - points[base]) / lengths.sum()))
    pinch = float(np.linalg.norm(points[4] - points[8]) / scale)
    if pinch <= (t.pinch_leave if previous == 'pinch' else t.pinch_enter):
        label = 'pinch'
    elif min(extensions) >= (t.open_leave if previous == 'open' else t.open_enter):
        label = 'open'
    elif max(extensions) <= (t.fist_leave if previous == 'fist' else t.fist_enter):
        label = 'fist'
    else:
        label = 'unknown'
    return GestureObservation(label, True, pinch, tuple(extensions))
