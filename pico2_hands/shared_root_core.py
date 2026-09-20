"""Opt-in PICO2 shared-root owner. Arm motion always comes from native Ruckig.

No Python Home interpolation or velocity reset while moving. H homes arms and
holds fingers, matching the VR DLS simulation. Short input loss may resume only
after bounded stable recovery and completed braking. Reconnection invalidates C.
"""
import numpy as np
from .simulation_core import SimulationCore
from .shared_root_mapping import SharedRootMapping
from .reference.official_pico import pico_official_hand_observations, pico_official_hand2_retarget_input


class SharedRootCore(SimulationCore):
    dropout_window_ns = 1_000_000_000
    recovery_stable_ns = 100_000_000
    recovery_min_frames = 5

    def __init__(self, home, ik, hands=None, *, height_m):
        super().__init__(home, ik, hands,
                         mapping_factory=lambda: SharedRootMapping(height_m, ik.forward))
        self.phase = "WAITING"
        self.reason = "shared-root + DLS/Ruckig: C calibration required"
        self._auto_resume = None
        self._last_valid_ns = None
        self._source_reset_pending = False
        self._manual_rearm_required = False

    def _cancel_resume(self):
        self._auto_resume = None

    def disconnected(self):
        self._cancel_resume()  # TCP reconnect always requires a human, even if short.
        self._last_valid_ns = None
        self._manual_rearm_required = True
        self._source_reset_pending = True
        super().disconnected()

    def _begin_dropout(self, now):
        self._cancel_resume()
        if (self._last_valid_ns is not None and
                0 <= now-self._last_valid_ns <= self.dropout_window_ns and
                self.mapping.calibration_allows_start and not self.exit_requested):
            self._auto_resume = dict(identity=self.identity,
                deadline=self._last_valid_ns+self.dropout_window_ns,
                first=None, last=None, count=0, qualified=False)

    def _try_resume(self, now):
        pending = self._auto_resume
        if pending is None:
            return
        if (pending["identity"] != self.identity or self.exit_requested or
                getattr(self,"pending_home",False) or not self.mapping.calibration_allows_start or
                self.phase not in ("BRAKING","HOLD")):
            self._cancel_resume()
            return
        if not pending["qualified"] and now > pending["deadline"]:
            self._cancel_resume()
            self.reason = "dropout recovery expired; fresh input then S"
            return
        try:
            valid = self.fresh(now,45_000_000)
            if valid:
                self.mapping.targets(self.frame)  # Mapping rejection never grants rearm.
        except ValueError:
            valid = False
        if not valid:
            if pending["qualified"]:
                self._cancel_resume()
            else:
                pending.update(first=None,last=None,count=0)
            return
        stamp = self.frame.received_timestamp_ns
        if pending["last"] is None or stamp > pending["last"]:
            if pending["last"] is not None and stamp-pending["last"] > 45_000_000:
                if pending["qualified"]:
                    self._cancel_resume()
                    return
                pending.update(first=None,last=None,count=0)
            if pending["first"] is None:
                pending["first"] = stamp
            pending["last"] = stamp
            pending["count"] += 1
            pending["qualified"] = (pending["count"] >= self.recovery_min_frames and
                                     stamp-pending["first"] >= self.recovery_stable_ns)
        if (pending["qualified"] and self.phase=="HOLD" and stamp > self.armed_at):
            # Same guarded S path: native at-rest check, soft start, hand-history
            # reset. Never teleport/reset velocity or resume during braking.
            self._cancel_resume()
            if self.action("s",now):
                self.reason = "short dropout recovered; soft-start resumed"

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
        return all(result[s]["accepted"] for s in ("left","right"))

    def _command(self, op, now):
        return self._consume(self.ik.command(op, self.q[:14].reshape(2,7), now/1e9))

    def fresh(self, now, maximum_age=150_000_000):
        if not super().fresh(now, maximum_age):
            return False
        try:
            self.mapping.extract(self.frame)
            return True
        except ValueError:
            return False

    def offer(self, frame, now):
        if self.identity is not None and self.identity != (frame.receiver_instance_id,frame.connection_generation):
            self._cancel_resume()
            self._last_valid_ns = None
            # Defer the numerical epoch barrier until stationary manual start.
            # Never reset velocity/acceleration during reconnect braking.
            self._source_reset_pending = True
        previous = self.mapping
        accepted = super().offer(frame, now)
        if self.mapping is not previous:
            self.reason = "source changed; C calibration required after Home/hold"
        return accepted

    def action(self, key, now):
        key = key.lower()
        if key in ("c","r","s","p"," ","h","q"):
            self._cancel_resume()
        if key == "c":
            return self.mapping.request_calibration(now, idle=self.state=="idle")
        if key == "s":
            if (self.exit_requested or self.state!="idle" or not self.mapping.calibration_allows_start or
                    not self.fresh(now,45_000_000) or self.frame.received_timestamp_ns <= self.armed_at):
                return False
            if self._source_reset_pending:
                self.ik.reset(self.q[:14].reshape(2,7))
                self._source_reset_pending = False
            if not self._command(4,now):
                return False
            self._manual_rearm_required = False
            self.hand_sequence += 2
            self.last_hand_frame = None
            self.rejections = 0
            self._last_valid_ns = self.frame.received_timestamp_ns
            return True
        if key == "r":
            if self.state!="idle" or not np.allclose(self.q[:14],self.home[:14],atol=1e-6,rtol=0):
                return False
            self.armed_at=now
            return True
        if key in ("h","q"):
            if self.mapping.state=="collecting":
                self.mapping.fail("calibration cancelled by Home")
            self.exit_requested |= key=="q"
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
            if self.phase in ("TELEOP","BRAKING","HOMING"):
                self.armed_at = now
                return self._command(6,now)
            return self.phase in ("HOLD","WAITING","HOME_REACHED")
        return False

    def tick(self, now):
        if self.mapping.tick(now):
            self.armed_at=now
            self.reason="shared-root C calibrated; new frame then S"
        # A receiver-level disconnect is stronger than an ordinary stale tick:
        # stop before a fast reconnect can be consumed while still in TELEOP.
        # Keep the manual-rearm latch until an explicit S is accepted.
        if self._manual_rearm_required and self.phase == "TELEOP":
            self._command(6,now)
            self.armed_at=now
            self.reason="input disconnected: braking to HOLD; fresh input then S"
        if self.state=="teleop":
            if not self.fresh(now,45_000_000):
                self._begin_dropout(now)
                self._command(6,now)
                self._command(7,now)
                self.armed_at=now
                self.reason=("input stale/invalid: braking; bounded auto-resume pending" if self._auto_resume
                             else "input stale/invalid: braking to HOLD; fresh input then S")
            else:
                try:
                    targets=self.mapping.targets(self.frame)
                except ValueError:
                    self._cancel_resume()
                    self._command(6,now)
                    self._command(7,now)
                    self.armed_at=now
                    return self.q.copy()
                result=self.ik.solve(self.q[:14].reshape(2,7),targets,
                    source_time=self.frame.source_timestamp_ns/1e9,
                    received_time=self.frame.received_timestamp_ns/1e9,now=now/1e9)
                accepted=self._consume(result)
                if not accepted:
                    self._cancel_resume()
                    self.armed_at=now
                else:
                    self._last_valid_ns=self.frame.received_timestamp_ns
                if accepted and self.frame.association_id!=self.last_hand_frame:
                    observations=pico_official_hand_observations(self.frame)
                    self.hand_sequence+=1
                    for side,region in (("left",slice(14,34)),("right",slice(34,54))):
                        if side in self.hands and observations[side].valid:
                            self.q[region]=self.hands[side].retarget(
                                pico_official_hand2_retarget_input(observations[side].keypoints_m),
                                self.hand_sequence,self.frame.received_timestamp_ns)
                    self.last_hand_frame=self.frame.association_id
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
        self._try_resume(now)
        return self.q.copy()
