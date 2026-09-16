"""Staged enable/teleop/home state machine on top of the MotionGate guards.

The base gate enables a device and immediately follows the live input. This
staged gate instead authorizes an explicit operator sequence:

    WAITING --arm()--> ALIGNING --settled--> READY --start_teleop()--> TELEOP
                                                              |
                                              start_homing()  v
                                    HOMING --settled--> HOME_REACHED

Every inherited source/feedback/bounds/epoch guard stays active in every armed
phase; this module only changes where setpoints come from and when the operator
is told the robot is ready. It never fabricates a command frame to satisfy a
guard, never substitutes zero for Home, and never homes after a fault.
"""
from __future__ import annotations

import math

from .safety import MotionGate, SafetyFault, _finite_vector, _slew
from .protocol import ARMS_READY
from .settling import FeedbackRest

WAITING = "WAITING"
ALIGNING = "ALIGNING"
READY = "READY"
TELEOP = "TELEOP"
HOMING = "HOMING"
HOME_REACHED = "HOME_REACHED"

# Phases whose setpoints are capped by the slow staged speed.
_SLOW_PHASES = (ALIGNING, READY, HOMING, HOME_REACHED)
# A setpoint this close to its target counts as "the commanded target reached".
_COMMAND_EPSILON = 1e-9


def _home_vector(values, label):
    if not isinstance(values, (list, tuple)):
        raise ValueError(f"staged_motion {label}: expected a list of 7 joint values")
    try:
        return _finite_vector(values, 7, f"staged_motion {label}")
    except (SafetyFault, TypeError) as error:
        raise ValueError(str(error)) from error


class StagedMotionGate(MotionGate):
    def __init__(self, configuration: dict, devices: tuple[str, ...], settings: dict):
        super().__init__(configuration, devices)
        if "arms" not in self.devices:
            raise ValueError("staged motion requires the arms device")
        if not isinstance(settings, dict):
            raise ValueError("staged_motion settings must be a mapping")
        left = _home_vector(settings.get("home_left_rad"), "home_left_rad")
        right = _home_vector(settings.get("home_right_rad"), "home_right_rad")
        home = left + right
        # Validate the supplied Home against the safety bounds here, long before
        # any SDK is loaded; Home is never silently replaced by zero or startup.
        for index, (value, lower, upper) in enumerate(zip(
                home, self.configuration["arms"]["lower_rad"], self.configuration["arms"]["upper_rad"])):
            if not lower <= value <= upper:
                raise ValueError(f"staged_motion home joint {index}: {value:.6f} rad "
                                 f"outside [{lower}, {upper}]")
        speed = settings.get("maximum_speed_rad_s")
        if type(speed) not in (int, float) or not math.isfinite(speed) or speed <= 0:
            raise ValueError("staged_motion maximum_speed_rad_s must be positive and finite")
        timeout = settings.get("timeout_s")
        if type(timeout) not in (int, float) or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("staged_motion timeout_s must be positive and finite")
        settle = settings.get("settle_time_s")
        if type(settle) not in (int, float) or not math.isfinite(settle) or settle < 0:
            raise ValueError("staged_motion settle_time_s must be finite and nonnegative")
        self._home = home
        self._slow_speed = float(speed)
        self._timeout_ns = float(timeout) * 1e9
        self._settle_ns = float(settle) * 1e9
        speed_limit = settings.get('settle_speed_rad_s', .03)
        if type(speed_limit) not in (int, float):
            raise ValueError('settle_speed_rad_s must be positive and finite')
        self._rest_speed = float(speed_limit)
        self._rest = {device: FeedbackRest(self._rest_speed, self._settle_ns, self.feedback_timeout_ns)
                      for device in self.devices}
        # The inherited per-device _phase dict drives the zero/ramp enable
        # sequence; the staged gate overrides arm/step/check_enable_ready, so its
        # single operator-visible phase lives here instead.
        self._staged_phase = WAITING
        self._settle_since_ns = None
        self._targets = {}
        self.bounded_resync = False
        self.dropout_hold = False
        self._dropout_since = None
        self._dropout_ready = None
        self.calibration_guard = None
        self._resync_since = None
        self._resync_generation = None
        self._event_sequence = 0

    def allow_unready_hold(self, device, frame, now_ns):
        if device!='arms' or not self.armed or self.phase!=TELEOP or frame.tracking_epoch!=self._epoch:
            return False
        if self.dropout_hold and getattr(frame,'event_state',None)==9:
            return (self._dropout_since is not None and
                    0<=now_ns-self._dropout_since<=300_000_000)
        return (self.bounded_resync and getattr(frame,'event_state',None)==2 and
                self._resync_since is not None and 0<=now_ns-self._resync_since<=100_000_000)

    def validate_targets(self, frame, now_ns):
        if self.calibration_guard is not None:
            self.calibration_guard.check(frame,now_ns,locked=self.armed)
        if self.bounded_resync or self.dropout_hold:
            from .mapped_events import EventFrame,FAULTS
            if not isinstance(frame,EventFrame):
                raise SafetyFault('hold policy requires matched private native events')
            if frame.event_state in FAULTS:
                raise SafetyFault('native '+FAULTS[frame.event_state])
        if self.dropout_hold:
            self._validate_dropout(frame,now_ns)
        if self.bounded_resync:
            if not self.armed:
                self._resync_since=None
                if frame.event_state==2:
                    raise SafetyFault('cannot enable during source resynchronization')
            if self._resync_since is not None and now_ns-self._resync_since>100_000_000:
                raise SafetyFault('resynchronization hold timeout')
            if frame.sequence>self._event_sequence:
                if frame.event_state==2:
                    if self._resync_since is None:
                        self._resync_since=frame.timestamp_ns
                        self._resync_generation=frame.generation
                    elif frame.generation!=self._resync_generation:
                        raise SafetyFault('generation changed during unconfirmed jump')
                elif self._resync_since is not None:
                    # Either confirmation (generation advances) or return to
                    # original accepted stream (generation unchanged).
                    if frame.generation<self._resync_generation:
                        raise SafetyFault('resynchronization generation rollback')
                    self._resync_since=None
                self._event_sequence=frame.sequence
        super().validate_targets(frame,now_ns)
        if self.dropout_hold:
            # Remember only a fully validated predecessor, never a rejected
            # target or a pending resynchronization.
            if frame.event_state in (1,3):
                self._dropout_ready=(frame.tracking_epoch,frame.generation,frame.input_valid_ns)
                self._dropout_since=None
            elif frame.event_state!=9:
                self._dropout_ready=None

    def _validate_dropout(self, frame, now_ns):
        source=frame.input_valid_ns
        if source<=0 or source>frame.timestamp_ns or source>now_ns:
            raise SafetyFault('invalid PICO input timestamp')
        if self.armed and frame.tracking_epoch!=self._epoch:
            raise SafetyFault('PICO tracking epoch changed during real teleoperation')
        predecessor=self._dropout_ready
        if predecessor is not None and source<predecessor[2]:
            raise SafetyFault('PICO input timestamp rollback')
        # Check the original source deadline BEFORE accepting a recovered frame.
        # Even missing/coalesced hold events cannot allow a late packet to renew it.
        since=self._dropout_since
        if since is None and self.armed and self.phase==TELEOP and predecessor is not None:
            since=predecessor[2]
        if since is not None and not 0<=now_ns-since<=300_000_000:
            raise SafetyFault('PICO dropout hold timeout')
        if frame.event_state==9:
            if not self.armed or self.phase!=TELEOP:
                raise SafetyFault('PICO dropout hold requires armed TELEOP')
            if frame.flags&ARMS_READY or predecessor!=(frame.tracking_epoch,frame.generation,source):
                raise SafetyFault('PICO dropout has no matching validated predecessor')
            if self._resync_since is not None:
                raise SafetyFault('PICO dropout during source resynchronization')
            self._dropout_since=source
        elif self._dropout_since is not None:
            if frame.event_state!=1 or predecessor[:2]!=(frame.tracking_epoch,frame.generation):
                raise SafetyFault('PICO dropout recovery identity/state changed')
            if source<=self._dropout_since:
                raise SafetyFault('PICO dropout recovery requires a new valid input')
        if frame.event_state in (1,3,9) and now_ns-source>300_000_000:
            raise SafetyFault('PICO dropout hold timeout')

    @property
    def phase(self) -> str:
        """Operator-visible phase: WAITING/ALIGNING/READY/TELEOP/HOMING/HOME_REACHED."""
        return self._staged_phase

    @property
    def display_targets(self) -> dict:
        """Snapshot of the active target per device.

        Alignment/READY report the frozen goal, TELEOP the latest input target,
        HOMING/HOME_REACHED the locked Home and the held hand poses, and WAITING
        an empty mapping (no target is active yet). The returned dict and its
        tuples are fresh copies: mutating them cannot change gate state.
        """
        return {device: tuple(self._targets[device]) for device in self.devices
                if device in self._targets}

    def check_enable_ready(self, frame, feedback, now_ns):
        """Pre-Enter gate: fresh sources plus healthy, disabled, in-bounds feedback.

        There is deliberately no start-pose match here: the first Enter authorizes
        bounded alignment to the frozen target instead of direct teleop.
        This bounds setpoint speed; it does not certify a collision-free path.
        """
        self.validate_targets(frame, now_ns)
        for device in self.devices:
            self.check_feedback(device, feedback[device], now_ns)
            if feedback[device].enabled:
                raise SafetyFault(f"{device}: servo already enabled; "
                                  f"refusing to take over another control session")

    def arm(self, frame, feedback, now_ns):
        if self.armed or self.fault:
            raise SafetyFault("session already armed/faulted; restart is required")
        self.check_enable_ready(frame, feedback, now_ns)
        if self.calibration_guard is not None:
            self.calibration_guard.check(frame,now_ns,locked=True)
        self._epoch = frame.tracking_epoch
        self._last_step_ns = now_ns
        self._targets = {device: tuple(float(value) for value in frame.positions(device))
                         for device in self.devices}
        for device in self.devices:
            # Never command a jump: the setpoint starts at the measured pose and
            # only the bounded slew moves it toward the frozen target. Hands go
            # straight there too, without the base class's zero/ramp detour.
            self._last_commands[device] = tuple(float(value) for value in feedback[device].position_rad)
        self.armed = True
        self._enter_phase(ALIGNING, now_ns)


    def start_teleop(self, frame, feedback, now_ns):
        """Switch a settled READY alignment to continuous teleop without a jump."""
        if self.fault:
            raise SafetyFault(self.fault)
        if not self.armed or self._staged_phase != READY:
            raise SafetyFault("continuous teleop requires a settled READY alignment; "
                              "press ENTER only after READY")
        try:
            self.observe_source(frame, now_ns)
            measured = {
                device: self.check_feedback(device, feedback[device], now_ns, require_enabled=True)
                for device in self.devices
            }
        except SafetyFault as error:
            self.fault = str(error)
            raise
        for device in self.devices:
            tolerance = self.configuration[device]["alignment_rad"]
            error = max(abs(value - goal) for value, goal in zip(measured[device], self._targets[device]))
            if error > tolerance:
                self._enter_phase(ALIGNING, now_ns)
                raise SafetyFault(f"{device}: drifted {error:.4f} rad from the frozen "
                                  f"alignment target; wait for READY")
            rest = self._rest[device]
            stamp = feedback[device].received_monotonic_ns
            rest.observe(measured[device], stamp, True)
            if not rest.resting:
                self._enter_phase(ALIGNING, now_ns)
                raise SafetyFault(f'{device}: no longer stationary; alignment required')
        # The last commanded pose is already the frozen target, so the TELEOP
        # setpoint continues from it and the bounded slew provides the handoff.
        self._enter_phase(TELEOP, now_ns)

    def start_homing(self, frame, feedback, now_ns):
        """Lock the configured Home for the arms and hold the hands where they are."""
        if self.fault:
            raise SafetyFault(self.fault)
        if not self.armed or self._staged_phase != TELEOP:
            raise SafetyFault("homing requires an active TELEOP session")
        try:
            self.observe_source(frame, now_ns)
            if self._dropout_since is not None:
                raise SafetyFault('cannot start homing during PICO dropout hold')
            for device in self.devices:
                self.check_feedback(device, feedback[device], now_ns, require_enabled=True)
        except SafetyFault as error:
            self.fault = str(error)
            raise
        self._targets["arms"] = self._home
        for device in self.devices:
            if device != "arms":
                # Hold the exact last commanded hand pose; no open/zero shortcut.
                self._targets[device] = self._last_commands[device]
        self._enter_phase(HOMING, now_ns)

    def step(self, frame, feedback, now_ns):
        if not self.armed or self.fault:
            raise SafetyFault(self.fault or "hardware output requires explicit arming")
        try:
            self.observe_source(frame, now_ns)
            if now_ns <= self._last_step_ns:
                raise SafetyFault("execution monotonic clock did not advance")
            # Cap the tick like the base gate: a stalled loop must not produce a
            # large catch-up step.
            dt = min((now_ns - self._last_step_ns) / 1e9, 1.0 / self.rate_hz)
            phase = self._staged_phase
            measured = {device: self.check_feedback(device, feedback[device], now_ns, require_enabled=True)
                        for device in self.devices}
            result = {}
            for device in self.devices:
                settings = self.configuration[device]
                previous = self._last_commands[device]
                self._check_tracking(device, previous, measured[device], now_ns,
                                     feedback[device].received_monotonic_ns)
                if phase == TELEOP:
                    limit = settings["maximum_speed_rad_s"]
                    holding = ((self.bounded_resync and getattr(frame,'event_state',None)==2) or
                               (self.dropout_hold and self._dropout_since is not None))
                    target = previous if holding else frame.positions(device)
                    self._targets[device] = tuple(float(value) for value in target)
                else:
                    limit = min(settings["maximum_speed_rad_s"], self._slow_speed)
                    target = self._targets[device]
                result[device] = _slew(previous, target, limit * dt)
            self._last_commands = result
            self._last_step_ns = now_ns
            self._advance_staged(phase, result, measured, now_ns, feedback)
            return result
        except SafetyFault as error:
            self.fault = str(error)
            raise

    def _enter_phase(self, phase, now_ns):
        self._staged_phase = phase
        self._phase_start_ns = now_ns
        self._settle_since_ns = None
        if phase in (ALIGNING, HOMING):
            self._rest = {device: FeedbackRest(self._rest_speed, self._settle_ns, self.feedback_timeout_ns)
                          for device in self.devices}

    def _device_settled(self, device, command, measured):
        target = self._targets[device]
        if max(abs(value - goal) for value, goal in zip(command, target)) > _COMMAND_EPSILON:
            return False
        return max(abs(value - goal)
                   for value, goal in zip(measured, target)) <= self.configuration[device]["alignment_rad"]

    def _timeout_detail(self, label, measured, devices):
        worst = max((abs(value - goal), device)
                    for device in devices
                    for value, goal in zip(measured[device], self._targets[device]))
        return (f"{label} did not settle within {self._timeout_ns / 1e9:g} s "
                f"({worst[1]} is still {worst[0]:.4f} rad from its target)")

    def _advance_staged(self, phase, commands, measured, now_ns, feedback):
        if phase not in _SLOW_PHASES:
            self._settle_since_ns = None
            return
        # HOME is an arm destination. Hands keep their commanded grip and all
        # health/tracking guards, but contact need not satisfy a new pose match.
        settling_devices = ("arms",) if phase in (HOMING, HOME_REACHED) else self.devices
        results = [self._rest[device].observe(
            measured[device], feedback[device].received_monotonic_ns,
            self._device_settled(device, commands[device], measured[device]))
            for device in settling_devices]
        settled = all(results)
        if phase in (ALIGNING, HOMING):
            if settled:
                self._enter_phase(READY if phase == ALIGNING else HOME_REACHED, now_ns)
                return
            else:
                self._settle_since_ns = None
            if now_ns - self._phase_start_ns > self._timeout_ns:
                raise SafetyFault(self._timeout_detail(
                    "alignment" if phase == ALIGNING else "homing", measured, settling_devices))
        elif phase == READY and not all(self._rest[device].resting for device in settling_devices):
            # Physical drift after settling revokes READY rather than authorizing
            # a teleop handoff from an unverified pose.
            self._enter_phase(ALIGNING, now_ns)
