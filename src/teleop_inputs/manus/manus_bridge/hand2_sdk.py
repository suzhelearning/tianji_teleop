"""Wuji Hand 2 SDK IK only; never discovers, connects, or enables hardware.

Adapted from wuji-hand-teleop's wujihand_output/_internal/sdk_retargeter.py.
Copyright (c) 2025 Wuji Technology Co., Ltd.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import numpy as np


HAND2_JOINT_LABELS = tuple(
    f"{finger}_S{joint}"
    for finger in ("thumb", "index", "middle", "ring", "pinky")
    for joint in range(1, 5)
)
# Official Hand 2 Beta 1 URDF limits, in dense SDK/firmware order, radians.
HAND2_POSITION_LIMITS = np.asarray([
    (-1.187, 1.291), (-1.484, 0.698), (-1.047, 1.570), (-1.047, 1.570),
    (-1.047, 1.570), (-0.698, 0.698), (-1.047, 2.094), (-1.047, 1.570),
    (-1.047, 1.570), (-0.698, 0.698), (-1.047, 2.094), (-1.047, 1.570),
    (-1.047, 1.570), (-0.698, 0.698), (-1.047, 2.094), (-1.047, 1.570),
    (-1.047, 1.570), (-0.698, 0.698), (-1.047, 2.094), (-1.047, 1.570),
], dtype=np.float32)
_POSITION_LIMIT_TOLERANCE = 1.0e-4


class WujiHand2Retargeter:
    """Own exactly one SDK RetargetSession for one side, with explicit reset."""

    def __init__(self, side, *, sdk=None):
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        if sdk is None:
            import wuji_sdk as sdk
        self.side = side
        self._session = sdk.RetargetSession.for_hand(
            sdk.HandModel.WujiHand2,
            side=sdk.Handedness.Left if side == "left" else sdk.Handedness.Right,
        )

    def retarget(self, keypoints):
        points = np.asarray(keypoints, dtype=np.float32)
        if points.shape == (63,):
            points = points.reshape(21, 3)
        if points.shape != (21, 3) or not np.isfinite(points).all():
            raise ValueError("keypoints must be finite (21,3) or (63,) points")
        result = np.asarray(self._session.step(points), dtype=np.float32)
        if result.shape != (20,) or not np.isfinite(result).all():
            raise RuntimeError("Wuji SDK returned an invalid 20-joint command")
        lower, upper = HAND2_POSITION_LIMITS.T
        outside = np.flatnonzero(
            (result < lower - _POSITION_LIMIT_TOLERANCE)
            | (result > upper + _POSITION_LIMIT_TOLERANCE)
        )
        if outside.size:
            details = ", ".join(
                f"{HAND2_JOINT_LABELS[index]}={result[index]:+.4f} outside "
                f"[{lower[index]:+.4f}, {upper[index]:+.4f}]" for index in outside
            )
            raise RuntimeError("Wuji SDK Hand 2 result violates URDF limits: " + details)
        # Only harmless floating-point endpoint overshoot is clipped.
        return np.clip(result, lower, upper)

    def reset(self):
        # Required API: never silently retain solver history after source change.
        self._session.reset()
