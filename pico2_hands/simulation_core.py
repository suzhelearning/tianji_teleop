"""Single-owner simulation state machine. No hardware or network output."""
import numpy as np
from .xz_mapping import OptionalXZMapping
from .reference.runtime import pico_frame_observations
from .reference.official_pico import pico_official_hand_observations, pico_official_hand2_retarget_input
from .reference.gesture_recognition import classify_hand


class SimulationCore:
    def __init__(self, home, ik, hands=None):
        self.home = np.asarray(home, dtype=float).copy()
        if self.home.shape != (54,) or not np.isfinite(self.home).all():
            raise ValueError("54 finite Home joints required")
        self.q = self.home.copy()
        self.ik, self.hands = ik, hands or {}
        self.mapping = OptionalXZMapping(lambda q: self.ik.forward(q))
        self.state, self.reason = "idle", "waiting for fresh PICO input"
        self.frame = None
        self.identity = None
        self.sequence = self.source_timestamp = -1
        self.armed_at = 0
        self.exit_requested = self.done = self.input_hold = False
        self.last_hand_frame = None
        self.hand_sequence = 0
        self.rejections = 0
        self.gestures = dict(left="unknown", right="unknown")
        self.ik.reset(self.q[:14].reshape(2, 7))

    def fresh(self, now, maximum_age=150_000_000):
        if self.frame is None or not 0 <= now - self.frame.received_timestamp_ns <= maximum_age:
            return False
        return all(arm.valid for _, arm in pico_frame_observations(self.frame).values())

    def offer(self, frame, now):
        identity = (frame.receiver_instance_id, frame.connection_generation)
        if self.identity is not None and identity != self.identity:
            self.mapping = OptionalXZMapping(lambda q: self.ik.forward(q))
            if self.state == "teleop":
                self.action("h", now)
            self.sequence = self.source_timestamp = -1
            self.hand_sequence += 2  # explicit discontinuity resets both native hand histories
            self.reason = "source changed; return Home and press S again"
        if frame.receiver_frame_sequence <= self.sequence or frame.source_timestamp_ns <= self.source_timestamp:
            return False
        self.identity = identity
        self.sequence, self.source_timestamp = frame.receiver_frame_sequence, frame.source_timestamp_ns
        self.frame = frame
        for side, observation in pico_official_hand_observations(frame).items():
            self.gestures[side] = classify_hand(observation.keypoints_m, valid=bool(observation.valid),
                                                previous=self.gestures[side]).gesture
        for _, arm in pico_frame_observations(frame).values():
            self.mapping.add(arm, now)
        return True

    def disconnected(self):
        self.frame = None
        self.gestures = dict(left="unknown", right="unknown")
        if self.mapping.calibration.state == "collecting":
            self.mapping.calibration.fail("PICO disconnected during calibration")

    def action(self, key, now):
        key = key.lower()
        if key == "c":
            return self.mapping.request_calibration(now, idle=self.state == "idle")
        if key == "r":
            if self.state != "idle" or not np.allclose(self.q, self.home, atol=1e-9, rtol=0):
                return False
            self.ik.reset(self.q[:14].reshape(2, 7))
            self.armed_at = now
            self.reason = "rearmed; wait for new input then S"
            return True
        if key == "s":
            if (self.state != "idle" or not self.mapping.calibration_allows_start or
                    not self.fresh(now) or self.frame.received_timestamp_ns <= self.armed_at):
                return False
            self.ik.reset(self.q[:14].reshape(2, 7))
            self.state, self.reason = "teleop", "following PICO"
            self.rejections = 0
            self.input_hold = False
            self.hand_sequence += 2
            self.last_hand_frame = None
            return True
        if key in ("h", "q"):
            if self.mapping.calibration.state == "collecting":
                self.mapping.calibration.fail("calibration cancelled by Home")
            self.exit_requested |= key == "q"
            if self.state != "homing":
                self.return_from = self.q.copy()
                self.return_start = now
                # Quintic peak derivative is 1.875; <= 0.4 rad/s for every joint.
                self.return_duration = max(1., 1.875 * np.max(np.abs(self.q-self.home)) / .4)
                self.state, self.reason = "homing", "returning Home; S disabled"
            return True
        return False

    def tick(self, now):
        if self.mapping.tick(now):
            self.ik.reset(self.q[:14].reshape(2, 7))
            self.armed_at = now
            self.reason = "X/Z calibration succeeded; press S"
        if self.state == "homing":
            u = min(1., max(0., (now-self.return_start) / 1e9 / self.return_duration))
            self.q = self.return_from + (10*u**3 - 15*u**4 + 6*u**5)*(self.home-self.return_from)
            if u >= 1:
                self.q = self.home.copy()
                # Direct-state simulation reaches the assigned setpoint exactly.
                # Two subsequent ticks provide a zero-velocity display barrier.
                if now >= self.return_start + int(self.return_duration*1e9) + 10_000_000:
                    self.ik.reset(self.q[:14].reshape(2, 7))
                    self.state, self.reason = "idle", "Home reached; press S"
                    self.armed_at = now
                    self.done = self.exit_requested
            return self.q.copy()
        if self.state != "teleop":
            return self.q.copy()
        # V131 internally expires targets at 50 ms; hold before that boundary
        # rather than accumulating solver failures while input is stale.
        if not self.fresh(now, 45_000_000):
            self.rejections = 0
            self.gestures = dict(left="unknown", right="unknown")
            if not self.input_hold:
                self.ik.reset(self.q[:14].reshape(2, 7))
            self.input_hold = True
            self.reason = "input stale/invalid: holding last pose"
            return self.q.copy()
        self.input_hold = False
        observations = pico_frame_observations(self.frame)
        targets = np.array([self.mapping.map(observations[s][1]).pose for s in ("left", "right")])
        result = self.ik.solve(self.q[:14].reshape(2, 7), targets,
                               source_time=self.frame.source_timestamp_ns/1e9,
                               received_time=self.frame.received_timestamp_ns/1e9, now=now/1e9)
        if not all(result[s]["accepted"] for s in ("left", "right")):
            self.rejections += 1
            self.ik.reset(self.q[:14].reshape(2, 7))
            self.reason = "IK rejected: holding last pose"
            if self.rejections >= 20:
                raise RuntimeError("20 consecutive bilateral IK rejections")
            return self.q.copy()
        self.rejections = 0
        candidate = self.q.copy()
        candidate[:14] = np.r_[result["left"]["joints"], result["right"]["joints"]]
        if self.hands and self.frame.association_id != self.last_hand_frame:
            observations = pico_official_hand_observations(self.frame)
            self.hand_sequence += 1
            for side, region in (("left", slice(14, 34)), ("right", slice(34, 54))):
                if observations[side].valid:
                    candidate[region] = self.hands[side].retarget(
                        pico_official_hand2_retarget_input(observations[side].keypoints_m),
                        self.hand_sequence, self.frame.received_timestamp_ns)
            self.last_hand_frame = self.frame.association_id
        if not np.isfinite(candidate).all():
            raise RuntimeError("nonfinite simulation output")
        self.q = candidate
        self.reason = "following PICO"
        return self.q.copy()
