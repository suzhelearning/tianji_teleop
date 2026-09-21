"""Feedback-only rest estimation. No SDK or command authority."""
import math


class FeedbackRest:
    def __init__(self, speed_limit, duration_ns, maximum_gap_ns):
        if not math.isfinite(speed_limit) or speed_limit <= 0:
            raise ValueError('settle_speed_rad_s must be positive and finite')
        self.speed_limit = speed_limit
        self.duration_ns = duration_ns
        self.maximum_gap_ns = maximum_gap_ns
        self.previous = None
        self.since = None
        self.resting = False

    def observe(self, position, stamp, in_position):
        position = tuple(position)
        if stamp <= 0 or not all(math.isfinite(x) for x in position):
            self.previous = None; self.since = None; self.resting = False
            return False
        previous = self.previous
        if previous is not None and stamp == previous[0]:
            # Cached feedback must neither accumulate rest time nor advance phase.
            if position != previous[1] or not in_position:
                self.since = None; self.resting = False
            return False
        self.previous = (stamp, position)
        if previous is None or stamp <= previous[0] or stamp-previous[0] > self.maximum_gap_ns:
            self.since = None; self.resting = False
            return False
        speed = max(abs(a-b) for a,b in zip(position, previous[1])) * 1e9/(stamp-previous[0])
        if not in_position or speed > self.speed_limit:
            self.since = None; self.resting = False
            return False
        if self.since is None:
            self.since = stamp
        self.resting = stamp-self.since >= self.duration_ns
        return self.resting
