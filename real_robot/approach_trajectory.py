"""Bounded, shared-progress fixed-target motion; no hardware dependencies.

Quintic motion is rest-to-rest, not jerk limited. Normal pause switches to a
constant scalar deceleration: velocity is continuous, acceleration may jump.
"""
from __future__ import annotations

import math


class ApproachTrajectory:
    def __init__(self, start, target, speed, acceleration):
        self.start = tuple(start)
        self.target = tuple(target)
        if len(self.start) != len(self.target) or not self.start:
            raise ValueError("trajectory endpoints must have matching nonzero dimensions")
        if not all(math.isfinite(q) for q in self.start + self.target):
            raise ValueError("trajectory endpoints must be finite")
        if not all(math.isfinite(x) and x > 0 for x in (speed, acceleration)):
            raise ValueError("trajectory limits must be positive and finite")
        self.delta = tuple(b - a for a, b in zip(self.start, self.target))
        distance = max(abs(d) for d in self.delta)
        # max s'(u)=15/8; max |s''(u)|=10/sqrt(3).
        self.duration = max(1.875 * distance / speed,
                            math.sqrt((10 / math.sqrt(3)) * distance / acceleration))
        self._scalar_acceleration = acceleration / distance if distance else 0.0
        self.elapsed = 0.0
        self.progress = 0.0 if distance else 1.0
        self.velocity = 0.0
        self.braking = False
        self.held = False

    @property
    def finished(self):
        return self.progress == 1.0 and self.velocity == 0.0

    @property
    def stopped(self):
        return self.velocity == 0.0

    @property
    def position(self):
        if self.progress == 1.0:
            return self.target
        return tuple(a + d * self.progress for a, d in zip(self.start, self.delta))

    def brake(self):
        """Commit to stopping even if the operator resumes before completion.

        Let A be the scalar acceleration bound. Every state of the nominal
        quintic can stop within its remaining path: its remaining integral of
        velocity is >= v²/(2A), because its nominal deceleration never exceeds
        A. Braking at -A therefore cannot overshoot. All joints stay on the
        original segment, with |qddot_i| = |delta_i| A <= acceleration.
        """
        self.braking = not self.stopped
        self.held = True

    def advance(self, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("trajectory timestep must be positive and finite")
        if self.braking:
            duration = min(dt, self.velocity / self._scalar_acceleration)
            self.progress = min(1.0, self.progress + duration * (
                self.velocity - 0.5 * self._scalar_acceleration * duration))
            self.velocity = max(0.0, self.velocity - self._scalar_acceleration * duration)
            if duration < dt or self.velocity == 0.0:
                self.velocity = 0.0
                self.braking = False
        elif not self.held and not self.finished:
            self.elapsed = min(self.duration, self.elapsed + dt)
            u = self.elapsed / self.duration
            # Evaluate close to the destination via the symmetric remainder to
            # avoid cancellation, then return the exact target at completion.
            r = min(u, 1.0 - u)
            s = r * r * r * (10 + r * (-15 + 6 * r))
            self.progress = s if u <= .5 else 1.0 - s
            self.velocity = 30 * u * u * (1 - u) * (1 - u) / self.duration
            if self.elapsed == self.duration:
                self.progress, self.velocity = 1.0, 0.0
        return self.position
