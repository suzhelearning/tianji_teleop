"""PICO2 shared-root owner. Arm motion always comes from native Ruckig.

No Python Home interpolation or velocity reset while moving. Manual mode retains
Home/hold controls and bounded dropout recovery. Continuous mode uses R calibration,
explicit S activation and stable same-session recovery; reconnection revokes calibration.
"""
from time import perf_counter_ns
import numpy as np
from .reference.runtime import pico_frame_observations
from .reference.gesture_recognition import classify_hand
from .shared_root_mapping import SharedRootMapping
from .reference.official_pico import pico_official_hand_observations, pico_official_hand2_retarget_input


class SharedRootCore:
    dropout_window_ns = 1_000_000_000
    recovery_stable_ns = 100_000_000
    recovery_min_frames = 5

    def __init__(self, home, ik, hands=None, *, height_m, continuous_follow=False):
        self.home = np.asarray(home, dtype=float).copy()
        if self.home.shape != (54,) or not np.isfinite(self.home).all():
            raise ValueError("54 finite Home joints required")
        self.q = self.home.copy()
        self.ik, self.hands = ik, hands or {}
        self.height_m = height_m
        self.continuous_follow = continuous_follow
        self._calibration_pending = False
        self.mapping = SharedRootMapping(height_m, ik.forward,
                                        calibration_key="R" if continuous_follow else "C")
        self.state = "idle"
        self.frame = self.identity = None
        self._frame_valid = None
        self._official_observations = None
        self.sequence = self.source_timestamp = -1
        self.armed_at = 0
        self.exit_requested = self.done = False
        self.last_hand_frame = None
        self.hand_sequence = 0
        self.gestures = dict(left="unknown", right="unknown")
        self.ik.reset(self.q[:14].reshape(2, 7))
        self.phase = "WAITING"
        self.braking_reason = None
        self.last_dls_ns = self.last_hands_ns = 0
        self.reason = ("shared-root + DLS/Ruckig: R calibration required" if continuous_follow
                       else "shared-root + DLS/Ruckig: C calibration required")
        self._auto_resume = None
        self._last_valid_ns = None
        self._source_reset_pending = False
        self._manual_rearm_required = False
        # SPD metadata never changes the numerical controller or grants S.
        self._spd_calibration_generation = 0
        self._spd_disconnect_generation = 0
        self._spd_disconnected = False
        self._spd_prepared = False
        self._spd_hold = False
        self._spd_arms_valid = False
        self._spd_hand_mask = 0
        self._spd_hand_stamps = dict(left=0, right=0)
        self._spd_arm_stamp = 0

    def spd_output(self, now_ns):
        """Return readiness and session identity for the last generated q.

        Tracking references need fresh source data at completion, not just at
        solve entry. Bounded native recovery and deliberate held targets do not
        depend on continuing sensor updates. This accessor does not refresh any
        timestamps or copy q; the caller timestamps the actual producer work.
        """
        session = (self.identity, getattr(self.ik, "epoch", 0),
                   self._spd_calibration_generation, self._spd_disconnect_generation)
        if self.phase == "FAULT" or getattr(self.ik, "failed", False):
            return 0, session
        mask = self._spd_hand_mask
        arms_valid = self._spd_arms_valid
        if self.phase == "TELEOP":
            arms_valid &= (self._spd_prepared and
                           0 <= now_ns-self._spd_arm_stamp <= 45_000_000)
        elif self.phase not in ("BRAKING", "HOMING"):
            arms_valid &= self._spd_prepared and self._spd_hold
        if not self._spd_hold:
            for side, bit in (("right", 2), ("left", 4)):
                if not 0 <= now_ns-self._spd_hand_stamps[side] <= 45_000_000:
                    mask &= ~bit
        return mask | int(arms_valid), session

    def _spd_hold_targets(self, now):
        # A manual hold may preserve known targets, but cannot retrospectively
        # endorse stale finger tracking just because it bypasses future aging.
        if not self._spd_hold:
            for side, bit in (("right", 2), ("left", 4)):
                if not 0 <= now-self._spd_hand_stamps[side] <= 45_000_000:
                    self._spd_hand_mask &= ~bit
        self._spd_hold = self._spd_prepared and self.phase != "FAULT"

    def _cancel_resume(self):
        self._auto_resume = None

    def disconnected(self):
        self.braking_reason = "disconnect"
        if not self._spd_disconnected:
            self._spd_disconnect_generation += 1
            self._spd_disconnected = True
        self._spd_prepared = self._spd_hold = False
        self._spd_hand_mask = 0
        if self.continuous_follow:
            self._spd_arms_valid = False
        self._cancel_resume()  # TCP reconnect always requires a human, even if short.
        self._last_valid_ns = None
        self._manual_rearm_required = True
        self._calibration_pending = False
        self._source_reset_pending = True
        self.frame = None
        self._frame_valid = self._official_observations = None
        self.gestures = dict(left="unknown", right="unknown")
        self.mapping.fail("PICO disconnected; " +
                          ("R" if self.continuous_follow else "C") + " calibration required")

    def _queue_resume(self, deadline=None, *, dropout=False):
        self._auto_resume = dict(identity=self.identity, deadline=deadline,
                                 first=None, last=None, count=0, qualified=False,
                                 dropout=dropout)

    def _begin_dropout(self, now):
        self._cancel_resume()
        if (self._last_valid_ns is not None and
                (self.continuous_follow or
                 0 <= now-self._last_valid_ns <= self.dropout_window_ns) and
                self.mapping.calibration_allows_start and not self.exit_requested):
            self._queue_resume(None if self.continuous_follow else
                               self._last_valid_ns+self.dropout_window_ns, dropout=True)

    def _try_resume(self, now):
        pending = self._auto_resume
        if pending is None:
            return
        allowed_phases = (("BRAKING", "HOLD", "WAITING", "HOME_REACHED")
                          if self.continuous_follow else ("BRAKING", "HOLD"))
        if (pending["identity"] != self.identity or self.exit_requested or
                getattr(self,"pending_home",False) or not self.mapping.calibration_allows_start or
                self.phase not in allowed_phases):
            self._cancel_resume()
            return
        if (pending["deadline"] is not None and not pending["qualified"] and
                now > pending["deadline"]):
            self._cancel_resume()
            self.reason = "dropout recovery expired; fresh input then S"
            return
        try:
            valid = self.fresh(now,45_000_000)
            if valid and self.continuous_follow:
                valid = self.frame.received_timestamp_ns > self.armed_at
            if valid:
                self.mapping.targets(self.frame)  # Mapping rejection never grants rearm.
        except ValueError:
            valid = False
        if not valid:
            if pending["qualified"] and not self.continuous_follow:
                self._cancel_resume()
            else:
                pending.update(first=None,last=None,count=0,qualified=False)
            return
        stamp = self.frame.received_timestamp_ns
        if pending["last"] is None or stamp > pending["last"]:
            if pending["last"] is not None and stamp-pending["last"] > 45_000_000:
                if pending["qualified"] and not self.continuous_follow:
                    self._cancel_resume()
                    return
                pending.update(first=None,last=None,count=0,qualified=False)
            if pending["first"] is None:
                pending["first"] = stamp
            pending["last"] = stamp
            pending["count"] += 1
            pending["qualified"] = (pending["count"] >= self.recovery_min_frames and
                                     stamp-pending["first"] >= self.recovery_stable_ns)
        if (pending["qualified"] and self.state=="idle" and stamp > self.armed_at):
            # Only a short same-session dropout preserves its prior approach
            # progress. Native independently checks freshness, HOLD and age.
            resume = (self.continuous_follow and pending["dropout"] and
                      self._last_valid_ns is not None and
                      0 <= now-self._last_valid_ns <= self.dropout_window_ns)
            if self._start_follow(now, resume=resume):
                self._cancel_resume()
                self.reason = ("stable tracking; continuous follow resumed" if self.continuous_follow
                               else "short dropout recovered; soft-start resumed")
            elif not self.continuous_follow:
                self._cancel_resume()

    def _start_follow(self, now, *, resume=False):
        if (self.exit_requested or self.state!="idle" or not self.mapping.calibration_allows_start or
                not self.fresh(now,45_000_000) or self.frame.received_timestamp_ns <= self.armed_at):
            return False
        if self._source_reset_pending:
            self.ik.reset(self.q[:14].reshape(2,7))
            self._source_reset_pending = False
        # Native may reject preserving progress (e.g. no accepted solve yet).
        # Ordinary start still enforces its at-rest gate and full soft approach.
        if not (resume and self._command(8,now)) and not self._command(4,now):
            return False
        self._manual_rearm_required = False
        self._spd_hold = False
        self._spd_arm_stamp = self.frame.received_timestamp_ns
        self.hand_sequence += 2
        self.last_hand_frame = None
        self._last_valid_ns = self.frame.received_timestamp_ns
        return True

    def _try_calibration(self, now):
        if (self._calibration_pending and
                self.phase in ("WAITING", "HOLD", "HOME_REACHED")):
            if self.mapping.request_calibration(now, idle=True):
                self._calibration_pending = False
                self.reason = "R calibration: hold head and both forward arms steady"

    def _request_continuous_calibration(self, now):
        if self.phase == "FAULT" or self.exit_requested:
            return False
        self._cancel_resume()
        self.pending_home = False
        self._calibration_pending = True
        self._last_valid_ns = None
        self.mapping.fail("R calibration pending; waiting for bounded HOLD")
        self.mapping.samples = []
        self._spd_calibration_generation += 1
        self._spd_prepared = self._spd_hold = self._spd_arms_valid = False
        self._spd_hand_mask = 0
        self.armed_at = now
        # Repeated R restarts collection, but never restarts an active brake.
        self.braking_reason = "recalibration"
        if self.phase in ("TELEOP", "HOMING"):
            self._command(6,now)
        self.reason = "R calibration pending; braking to HOLD"
        self._try_calibration(now)
        return True

    def _consume(self, result):
        if result["left"]["status"] != result["right"]["status"]:
            raise RuntimeError("bilateral recovery phase mismatch")
        self.phase = result["left"]["status"]
        self.q[:14] = np.r_[result["left"]["joints"],result["right"]["joints"]]
        if not np.isfinite(self.q).all():
            raise RuntimeError("nonfinite DLS reference")
        self.state = ("teleop" if self.phase == "TELEOP" else
                      "homing" if self.phase in ("BRAKING","HOMING") else "idle")
        self.reason = "DLS/Ruckig: " + self.phase
        accepted = all(result[s]["accepted"] for s in ("left","right"))
        if self.phase == "TELEOP":
            self._spd_arms_valid = self._spd_prepared and accepted
        elif self.phase in ("BRAKING", "HOMING"):
            # Rejected tracking can still produce bounded native braking.
            self._spd_arms_valid |= self._spd_prepared
        else:
            self._spd_arms_valid = (self._spd_prepared and self._spd_hold and
                                   self.phase in ("WAITING", "HOME_REACHED", "HOLD"))
            if self.phase == "FAULT":
                self._spd_hand_mask = 0
        return accepted

    def _command(self, op, now):
        return self._consume(self.ik.command(op, self.q[:14].reshape(2,7), now/1e9))

    def fresh(self, now, maximum_age=150_000_000):
        if (self.frame is None or
                not 0 <= now-self.frame.received_timestamp_ns <= maximum_age):
            return False
        if self._frame_valid is None:
            self._frame_valid = all(
                arm.valid for _, arm in pico_frame_observations(self.frame).values())
        if not self._frame_valid:
            return False
        try:
            self.mapping.extract(self.frame)
            return True
        except ValueError:
            return False

    def offer(self, frame, now):
        identity = (frame.receiver_instance_id, frame.connection_generation)
        if self.identity is not None and self.identity != identity:
            self._frame_valid = self._official_observations = None
            self._spd_prepared = self._spd_hold = False
            self._spd_hand_mask = 0
            self._cancel_resume()
            self._last_valid_ns = None
            # Defer the numerical epoch barrier until stationary start.
            # Never reset velocity/acceleration during reconnect braking.
            self._source_reset_pending = True
            self.mapping = SharedRootMapping(self.height_m, self.ik.forward,
                                            calibration_key="R" if self.continuous_follow else "C")
            if self.continuous_follow:
                self._spd_arms_valid = False
                self._calibration_pending = False
                self._manual_rearm_required = True
                if self.phase == "TELEOP":
                    self._command(6,now)
                    self.armed_at = now
            elif self.state == "teleop":
                self.action("h", now)
            self.braking_reason = "disconnect"
            self.sequence = self.source_timestamp = -1
            self.hand_sequence += 2
            self.gestures = dict(left="unknown", right="unknown")
            self.reason = ("source changed; R calibration required after HOLD" if self.continuous_follow
                           else "source changed; C calibration required after Home/hold")
        if frame.receiver_frame_sequence <= self.sequence or frame.source_timestamp_ns <= self.source_timestamp:
            return False
        self.identity = identity
        self._spd_disconnected = False
        self.sequence, self.source_timestamp = frame.receiver_frame_sequence, frame.source_timestamp_ns
        self.frame = frame
        self._frame_valid = None
        self._official_observations = pico_official_hand_observations(frame)
        self.mapping.offer_frame(frame, now)
        for side, observation in self._official_observations.items():
            if not observation.valid:
                self._spd_hand_mask &= ~(2 if side == "right" else 4)
            self.gestures[side] = classify_hand(
                observation.keypoints_m, valid=bool(observation.valid),
                previous=self.gestures[side]).gesture
        return True

    def action(self, key, now):
        key = key.lower()
        if self.continuous_follow:
            if key == "r":
                return self._request_continuous_calibration(now)
            if key == "c":
                return False
            if key in ("p", " ", "h", "q"):
                self._calibration_pending = False
        if key in ("c","s","p"," ","h","q"):
            self._cancel_resume()
        if key == "c":
            accepted = self.mapping.request_calibration(now, idle=self.state=="idle")
            if accepted:
                # An accepted C revokes the previous candidate immediately,
                # including when the subsequent sample collection fails.
                self._spd_calibration_generation += 1
                self._spd_prepared = self._spd_hold = self._spd_arms_valid = False
                self._spd_hand_mask = 0
            return accepted
        if key == "s":
            return self._start_follow(now)
        if key in ("h","q"):
            self.braking_reason = "home"
            if self.mapping.state=="collecting":
                self.mapping.fail("calibration cancelled by Home")
            self.exit_requested |= key=="q"
            self._spd_hold_targets(now)
            if self.phase in ("BRAKING","HOMING"):
                # A stale-input brake only requested HOLD. Request Home after
                # it settles; never reset the running trajectory.
                self.pending_home=True
                return True
            self.pending_home=False
            return self._command(5,now)
        if key in ("p"," "):
            # Exit must finish Home; pausing it would strand the shutdown loop.
            if self.exit_requested:
                return False
            self.pending_home = False
            self.braking_reason = "manual_hold"
            if self.phase in ("TELEOP","BRAKING","HOMING"):
                self._spd_hold_targets(now)
                self.armed_at = now
                return self._command(6,now)
            accepted = self.phase in ("HOLD","WAITING","HOME_REACHED")
            if accepted:
                self._spd_hold_targets(now)
                self._spd_arms_valid = self._spd_prepared
            return accepted
        return False

    def tick(self, now):
        self.last_dls_ns = self.last_hands_ns = 0
        if self.mapping.tick(now):
            self.armed_at=now
            self.reason=("shared-root R calibrated; ready, new frame then S to follow" if self.continuous_follow
                         else "shared-root C calibrated; new frame then S")
            self._spd_prepared = self._spd_hold = self._spd_arms_valid = True
            # Calibration endorses the initialized/current commanded pose, not the
            # calibration posture. Disabled or invalid hands are not candidates.
            observations = self._official_observations
            self._spd_hand_mask = sum(
                bit for side, bit in (("right", 2), ("left", 4))
                if side in self.hands and observations[side].valid)
            for side in self._spd_hand_stamps:
                self._spd_hand_stamps[side] = self.frame.received_timestamp_ns
        if not self._spd_hold:
            for side, bit in (("right", 2), ("left", 4)):
                if not 0 <= now-self._spd_hand_stamps[side] <= 45_000_000:
                    self._spd_hand_mask &= ~bit
        # A receiver-level disconnect is stronger than an ordinary stale tick:
        # stop before a fast reconnect can be consumed while still in TELEOP.
        # Keep the rearm latch until calibrated start is accepted.
        if self._manual_rearm_required and self.phase == "TELEOP":
            self.braking_reason = "disconnect"
            self._command(6,now)
            self.armed_at=now
            self.reason=("input disconnected: braking to HOLD; R calibration required" if self.continuous_follow
                         else "input disconnected: braking to HOLD; C calibration then S")
        if self.state=="teleop":
            if not self.fresh(now,45_000_000):
                self.braking_reason = "stale_input"
                self._spd_hold = False
                self.gestures = dict(left="unknown", right="unknown")
                self._begin_dropout(now)
                self._command(6,now)
                self._command(7,now)
                self.armed_at=now
                if self.continuous_follow:
                    self.reason=("input stale/invalid: braking; stable recovery pending" if self._auto_resume
                                 else "input stale/invalid: braking to HOLD; R calibration required")
                else:
                    self.reason=("input stale/invalid: braking; bounded auto-resume pending" if self._auto_resume
                                 else "input stale/invalid: braking to HOLD; fresh input then S")
            else:
                try:
                    targets=self.mapping.targets(self.frame)
                except ValueError:
                    self.braking_reason = "mapping_rejected"
                    self._cancel_resume()
                    if self.continuous_follow:
                        self._begin_dropout(now)
                    self._spd_hold = False
                    self._command(6,now)
                    self._command(7,now)
                    self.armed_at=now
                    return self.q.copy()
                started = perf_counter_ns()
                try:
                    result=self.ik.solve(self.q[:14].reshape(2,7),targets,
                        source_time=self.frame.source_timestamp_ns/1e9,
                        received_time=self.frame.received_timestamp_ns/1e9,now=now/1e9)
                finally:
                    self.last_dls_ns = perf_counter_ns()-started
                accepted=self._consume(result)
                if not accepted:
                    self.braking_reason = "ik_rejected"
                    self._spd_hand_mask = 0
                    self._cancel_resume()
                    if self.continuous_follow:
                        self._begin_dropout(now)
                    self.armed_at=now
                else:
                    self._last_valid_ns=self.frame.received_timestamp_ns
                    self._spd_arm_stamp = self.frame.received_timestamp_ns
                if accepted and self.frame.association_id!=self.last_hand_frame:
                    started = perf_counter_ns()
                    pending = []
                    try:
                        observations=self._official_observations
                        self.hand_sequence+=1
                        stamp = self.frame.received_timestamp_ns
                        # Both independent native solvers start before either
                        # reply is awaited. No queue of old input frames.
                        for side,region in (("left",slice(14,34)),("right",slice(34,54))):
                            if side in self.hands and observations[side].valid:
                                worker = self.hands[side]
                                pending.append((side, region, worker))
                                worker.submit(
                                    pico_official_hand2_retarget_input(observations[side].keypoints_m),
                                    self.hand_sequence, stamp)
                        results = [worker.receive() for _, _, worker in pending]
                        if any(result.shape != (20,) or not np.isfinite(result).all()
                               for result in results):
                            raise RuntimeError("invalid native hand target")
                        # Commit both hands only after all requested results
                        # validate; a failed peer must not expose half a frame.
                        for (side, region, _), result in zip(pending, results):
                            self.q[region] = result
                            self._spd_hand_mask |= 2 if side == "right" else 4
                            self._spd_hand_stamps[side] = stamp
                        self.last_hand_frame=self.frame.association_id
                    except BaseException:
                        self._spd_hand_mask = 0
                        raise
                    finally:
                        self.last_hands_ns = perf_counter_ns()-started if pending else 0
        elif self.state=="homing":
            previous=self.phase
            self._command(7,now)
            if self.phase=="HOLD" and getattr(self,"pending_home",False):
                self.pending_home=False
                self._command(5,now)
            elif self.phase in ("HOME_REACHED","HOLD") and previous!=self.phase:
                self.armed_at=now
                if self.phase=="HOME_REACHED":
                    self.pending_home=False
                self.done=self.exit_requested and self.phase=="HOME_REACHED"
        self._try_calibration(now)
        self._try_resume(now)
        return self.q.copy()
