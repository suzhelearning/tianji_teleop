"""Staged enable/teleop/home state machine on top of the MotionGate guards.

The base gate enables a device and immediately follows the live input. This
staged gate instead authorizes an explicit operator sequence:

    WAITING --arm()--> ALIGNING --settled--> READY --start_teleop()--> TELEOP
                                                              |
                                              start_homing()  v
                                    HOMING --settled--> HOME_REACHED

    WAITING --arm_home()--> HOMING --> HOME_REACHED --start_collection_alignment()--> ALIGNING

Data episodes stay armed at Home, opening hands only after measured arm rest.
TELEOP can commit to a bounded brake and hold while recording services complete.

Every inherited source/feedback/bounds/epoch guard stays active in every armed
phase; this module only changes where setpoints come from and when the operator
is told the robot is ready. It never fabricates a command frame to satisfy a
guard, never substitutes zero for Home, and never homes after a fault.
"""
from __future__ import annotations

import math

from .safety import MotionGate, SafetyFault, _finite_vector, _slew
from .settling import FeedbackRest
from .approach_trajectory import ApproachTrajectory, BrakingTrajectory

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
        acceleration = settings.get("maximum_acceleration_rad_s2", .2)
        if type(acceleration) not in (int, float) or not math.isfinite(acceleration) or acceleration <= 0:
            raise ValueError("staged_motion maximum_acceleration_rad_s2 must be positive and finite")
        timeout = settings.get("timeout_s")
        if type(timeout) not in (int, float) or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("staged_motion timeout_s must be positive and finite")
        settle = settings.get("settle_time_s")
        if type(settle) not in (int, float) or not math.isfinite(settle) or settle < 0:
            raise ValueError("staged_motion settle_time_s must be finite and nonnegative")
        self._home = home
        self._slow_speed = float(speed)
        self._slow_acceleration = float(acceleration)
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
        self._execution_started = False
        self._targets = {}
        self.paused = False
        self._trajectories = {}
        self._awaiting_rest = False
        self._pause_active = False
        self._command_moving = False
        self._hold_rest = {}
        self._teleop_held = False
        self._teleop_velocities = {}
        self._teleop_brakes = {}
        self._episode_settings = settings.get("episode", {})
        self._collection = False
        self._opening_hands = False
        self._configure_episode()

    def _configure_episode(self):
        settings = self._episode_settings
        if not isinstance(settings, dict):
            raise ValueError("staged_motion episode must be a mapping")
        value = settings.get("hand_ready_tolerance_rad", .15)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError("staged_motion episode hand_ready_tolerance_rad must be positive and finite")
        self._hand_ready_tolerance_rad = float(value)
        try:
            self._hand_open = _finite_vector(
                settings.get("hand_open_rad", [0.0] * 20), 20, "episode hand_open_rad")
        except (SafetyFault, TypeError) as error:
            raise ValueError(str(error)) from error
        for device in self.devices:
            if device == "arms":
                continue
            if any(not lo <= value <= hi for value, lo, hi in zip(
                    self._hand_open, self.configuration[device]["lower_rad"],
                    self.configuration[device]["upper_rad"])):
                raise ValueError(f"episode hand_open_rad outside {device} bounds")

    @property
    def opening_hands(self):
        return self._collection and self.phase == HOMING and self._opening_hands

    @property
    def phase(self) -> str:
        """Operator-visible phase: WAITING/ALIGNING/READY/TELEOP/HOMING/HOME_REACHED."""
        return self._staged_phase

    @property
    def staged_stopped(self) -> bool:
        """Command velocity is zero, including completion of any pause brake."""
        return (self.phase in _SLOW_PHASES and bool(self._trajectories)
                and all(path.stopped for path in self._trajectories.values()))

    @property
    def teleop_stopped(self) -> bool:
        """A requested TELEOP hold has stopped commands and measured motion."""
        return (self.armed and not self.fault and self.phase == TELEOP
                and self._teleop_held and bool(self._teleop_brakes)
                and all(path.stopped for path in self._teleop_brakes.values())
                and all(self._rest[device].resting for device in self.devices))

    def hold_teleop(self):
        """Brake the arms from emitted velocity; retain the last hand targets.

        Hand targets use the established hold behavior, not arm acceleration
        limits: stopping an episode must not add finger closure or opening.
        An infeasible stop faults rather than clipping a joint at its limit.
        """
        if self.fault:
            raise SafetyFault(self.fault)
        if not self.armed or self.phase != TELEOP:
            raise SafetyFault("teleop hold requires an active TELEOP session")
        if self._teleop_held:
            return
        brakes = {
            device: BrakingTrajectory(
                self._last_commands[device],
                self._teleop_velocities[device] if device == "arms"
                else (0.0,) * len(self._last_commands[device]),
                self._slow_acceleration)
            for device in self.devices}
        for device, path in brakes.items():
            limits = self.configuration[device]
            for index, (start, stop, lower, upper) in enumerate(zip(
                    path.start, path.target, limits["lower_rad"], limits["upper_rad"])):
                if not lower <= start <= upper or not lower <= stop <= upper:
                    self.fault = (f"{device}: braking stop joint {index} at {stop:.6f} rad "
                                  f"outside [{lower}, {upper}]")
                    raise SafetyFault(self.fault)
        self._teleop_brakes = brakes
        self._teleop_held = True
        self._targets = {device: path.target for device, path in brakes.items()}
        self._rest = {device: FeedbackRest(self._rest_speed, self._settle_ns, self.feedback_timeout_ns)
                      for device in self.devices}

    def release_teleop_hold(self):
        """Resume following only after every commanded and measured group rests."""
        if self.fault:
            raise SafetyFault(self.fault)
        if not self.teleop_stopped:
            raise SafetyFault("release requires a held stationary TELEOP session")
        self._teleop_held = False
        self._teleop_brakes = {}

    def _reset_hold_rest(self):
        self._hold_rest = {
            device: FeedbackRest(self._rest_speed, self._settle_ns, self.feedback_timeout_ns)
            for device in self.devices}

    def reset_staged_motion(self):
        """Replan from held commands, never from measured positions.

        Call only after a stationary seed/target change. Fresh measured rest is
        checked before motion as well; resetting is not permission to stop an
        already moving trajectory instantaneously.
        """
        if any(not path.stopped for path in self._trajectories.values()):
            self.fault = "cannot reset staged motion while command velocity is nonzero"
            raise SafetyFault(self.fault)
        self._trajectories = {
            device: ApproachTrajectory(
                self._last_commands[device], self._targets[device],
                min(self.configuration[device]["maximum_speed_rad_s"], self._slow_speed),
                self._slow_acceleration)
            for device in self.devices}
        self._check_approach_duration(self._timeout_ns, 2)
        self._awaiting_rest = True
        self._pause_active = False
        self._command_moving = False
        self._reset_hold_rest()

    def _check_approach_duration(self, available_ns, rest_periods):
        duration = max(path.duration for path in self._trajectories.values())
        # Include both measured-rest intervals at entry, only the final one
        # after a verified hold, plus sampling margins. Never knowingly start a
        # trajectory that the configured deadline cannot accommodate.
        needed = duration + rest_periods * self._settle_ns / 1e9 + 4 / self.rate_hz
        if needed * 1e9 > available_ns:
            self.fault = (f"staged approach requires at least {needed:.3f} s including measured rest; "
                          f"only {max(0, available_ns) / 1e9:.3f} s remain in the configured timeout")
            raise SafetyFault(self.fault)

    def _slow_commands(self, measured, feedback, now_ns, dt):
        # Paused wall time, including a committed brake after early resume, does
        # not consume approach timeout. Source/feedback watchdogs still do.
        if self.paused or any(path.braking for path in self._trajectories.values()):
            self._phase_start_ns += now_ns - self._last_step_ns
        if self.paused and not self._pause_active:
            self._pause_active = True
            self._awaiting_rest = True
            self._reset_hold_rest()
            for path in self._trajectories.values():
                path.brake()
        if self._pause_active:
            result = {device: path.advance(dt) for device, path in self._trajectories.items()}
        elif self._awaiting_rest:
            result = dict(self._last_commands)
        else:
            return {device: path.advance(dt) for device, path in self._trajectories.items()}
        stationary = []
        settling_devices = (("arms",) if self.phase in (HOMING, HOME_REACHED)
                            and not (self._collection and self._opening_hands) else self.devices)
        for device in settling_devices:
            near_hold = max(abs(a - b) for a, b in zip(measured[device], result[device])) <= (
                self.configuration[device]["alignment_rad"])
            stationary.append(self._hold_rest[device].observe(
                measured[device], feedback[device].received_monotonic_ns,
                self.staged_stopped and (near_hold or (
                    self._collection and self.phase == HOMING and device != "arms"))))
        if not self.paused and all(stationary):
            # A new rest-to-rest path starts on the NEXT tick from this command,
            # not from lagging feedback. All groups finish braking first.
            self._last_commands = result
            self.reset_staged_motion()
            self._check_approach_duration(
                self._timeout_ns - (now_ns - self._phase_start_ns), 1)
            self._awaiting_rest = False
        return result

    @property
    def display_targets(self) -> dict:
        """Snapshot of the active target per device.

        Alignment/READY report the frozen goal, TELEOP the latest input target
        (or committed stopping pose while held), HOMING/HOME_REACHED the arm Home
        hold and held/opening hand goals, and WAITING an empty mapping.
        The returned dict and its
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
        """Authorize initial alignment; the existing real-mode entry point."""
        self._arm(frame, feedback, now_ns, ALIGNING)

    def check_home_enable_ready(self, frame, feedback, now_ns):
        """Collection startup requires measured arm Home before any enable."""
        self.check_enable_ready(frame, feedback, now_ns)
        positions = feedback["arms"].position_rad
        error = max(abs(actual - home) for actual, home in zip(positions, self._home))
        if error > self.configuration["arms"]["alignment_rad"]:
            raise SafetyFault(
                f"collection startup requires measured Home before enable "
                f"(maximum joint error {error:.4f} rad); return Home separately first")

    def arm_home(self, frame, feedback, now_ns):
        """Authorize hand opening at verified Home, without moving the arms."""
        self.check_home_enable_ready(frame, feedback, now_ns)
        self._collection = True
        self._opening_hands = False
        self._arm(frame, feedback, now_ns, HOMING)

    def _arm(self, frame, feedback, now_ns, phase):
        if self.armed or self.fault:
            raise SafetyFault("session already armed/faulted; restart is required")
        self.check_enable_ready(frame, feedback, now_ns)
        self._epoch = frame.tracking_epoch
        self._last_step_ns = now_ns
        self._targets = {device: tuple(float(value) for value in frame.positions(device))
                         for device in self.devices}
        for device in self.devices:
            # The initial hold is measured; fresh stationary feedback is required
            # before the fixed-target trajectory starts. Hands skip zero/ramp.
            self._last_commands[device] = tuple(float(value) for value in feedback[device].position_rad)
        if phase == HOMING:
            self._targets = dict(self._last_commands)
            if not self._collection:
                self._targets["arms"] = self._home
        self.armed = True
        self._enter_phase(phase, now_ns)

    def complete_enable(self, frame, feedback, now_ns):
        """Start motion timing once, after SDK enable and fresh measured feedback."""
        if self.fault:
            raise SafetyFault(self.fault)
        if not self.armed or self._execution_started or self.phase not in (ALIGNING, HOMING):
            raise SafetyFault("enable completion requires an armed session before its first motion step")
        try:
            self.observe_source(frame, now_ns)
            measured = {
                device: self.check_feedback(device, feedback[device], now_ns, require_enabled=True)
                for device in self.devices}
            # No motion command has been sent. Seed from the post-enable measured
            # pose, keeping the operator-authorized alignment target frozen.
            if self._collection:
                error = max(abs(q - home) for q, home in zip(measured["arms"], self._home))
                if error > self.configuration["arms"]["alignment_rad"]:
                    raise SafetyFault("collection Home was lost during device enable")
                self._targets = dict(measured)
            self._last_commands = measured
            self._last_step_ns = now_ns
            self._enter_phase(self.phase, now_ns)
            self._execution_started = True
        except SafetyFault as error:
            self.fault = str(error)
            raise

    def _episode_feedback(self, frame, feedback, now_ns):
        """Run armed safety guards even when a readiness request cannot succeed."""
        try:
            self.observe_source(frame, now_ns)
            measured = {
                device: self.check_feedback(device, feedback[device], now_ns, require_enabled=True)
                for device in self.devices}
            for device in self.devices:
                self._check_tracking(device, self._last_commands[device], measured[device], now_ns,
                                     feedback[device].received_monotonic_ns)
            return measured
        except SafetyFault as error:
            self.fault = str(error)
            raise

    def _require_episode_home(self, measured, feedback):
        for device in self.devices:
            goal = self._home if device == "arms" else self._hand_open
            tolerance = (self.configuration["arms"]["alignment_rad"] if device == "arms"
                         else min(self.configuration[device]["alignment_rad"],
                                  self._hand_ready_tolerance_rad))
            near_home = max(abs(q - target) for q, target in zip(measured[device], goal)) <= tolerance
            near_command = max(abs(q - target) for q, target in zip(
                measured[device], self._last_commands[device])) <= self.configuration[device]["alignment_rad"]
            self._rest[device].observe(
                measured[device], feedback[device].received_monotonic_ns, near_home and near_command)
            if not near_home or not self._rest[device].resting:
                raise ValueError(f"{device}: episode requires stationary measured Home and open hands")

    def start_collection_alignment(self, frame, feedback, now_ns):
        """Freeze all live targets from stationary actual Home with open hands.

        A healthy but non-ready robot pose is retryable; source, epoch, device
        health, bounds and tracking violations latch the normal hardware fault.
        The operator's arm and hand poses are targets, not readiness conditions.
        """
        if self.fault:
            raise SafetyFault(self.fault)
        if not self.armed or not self._collection:
            raise ValueError("collection alignment requires an armed collection session")
        measured = self._episode_feedback(frame, feedback, now_ns)
        if self.phase != HOME_REACHED or self.paused or self._awaiting_rest or not self.staged_stopped:
            raise ValueError("collection alignment requires stationary HOME_REACHED")
        self._require_episode_home(measured, feedback)
        self._targets = {
            device: tuple(float(value) for value in frame.positions(device))
            for device in self.devices}
        self._opening_hands = False
        self._enter_phase(ALIGNING, now_ns)

    def start_teleop(self, frame, feedback, now_ns):
        """Switch a settled READY alignment to continuous teleop without a jump."""
        if self.fault:
            raise SafetyFault(self.fault)
        if not self.armed or self._staged_phase != READY:
            raise SafetyFault("continuous teleop requires a settled READY alignment; "
                              "press ENTER only after READY")
        if self.paused or self._awaiting_rest or not self.staged_stopped:
            raise SafetyFault("continuous teleop requires an unpaused stationary alignment")
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
        self._teleop_held = False
        self._teleop_brakes = {}
        # READY verified command and measured rest; this is the only zero-velocity
        # initialization. A hold during moving TELEOP uses emitted velocities.
        self._teleop_velocities = {
            device: (0.0,) * len(self._last_commands[device]) for device in self.devices}

    def start_homing(self, frame, feedback, now_ns):
        """Return arms Home with hands held; collection then opens at measured rest."""
        if self.fault:
            raise SafetyFault(self.fault)
        if not self.armed or self._staged_phase != TELEOP:
            raise SafetyFault("homing requires an active TELEOP session")
        try:
            self.observe_source(frame, now_ns)
            measured = {
                device: self.check_feedback(device, feedback[device], now_ns, require_enabled=True)
                for device in self.devices}
        except SafetyFault as error:
            self.fault = str(error)
            raise
        # Never invent zero velocity at the handoff from live following:
        # The emitted commands and actual arms must first come to rest.
        for device in ("arms",):
            near_hold = max(abs(a - b) for a, b in zip(
                measured[device], self._last_commands[device])) <= self.configuration[device]["alignment_rad"]
            self._rest[device].observe(measured[device], feedback[device].received_monotonic_ns,
                                       near_hold and not self._command_moving)
        if self._command_moving or not self._rest["arms"].resting:
            raise SafetyFault("homing requires stationary live commands and measured feedback")
        if self._teleop_held and not self.teleop_stopped:
            raise SafetyFault("homing requires all held devices to be stationary")
        self._targets["arms"] = self._home
        for device in self.devices:
            if device != "arms":
                # Hold the exact last commanded hand pose; no open/zero shortcut.
                self._targets[device] = self._last_commands[device]
        self._opening_hands = False
        self._enter_phase(HOMING, now_ns)

    def step(self, frame, feedback, now_ns):
        if not self.armed or self.fault:
            raise SafetyFault(self.fault or "hardware output requires explicit arming")
        self._execution_started = True
        try:
            self.observe_source(frame, now_ns)
            phase = self._staged_phase
            elapsed_ns = now_ns - self._last_step_ns
            if elapsed_ns <= 0:
                raise SafetyFault(
                    f"execution monotonic clock did not advance: phase={phase} "
                    f"elapsed_ms={elapsed_ns / 1e6:.3f}")
            if ((phase in _SLOW_PHASES or self._teleop_held or self._collection)
                    and elapsed_ns > self.command_timeout_ns):
                raise SafetyFault(
                    f"execution cycle exceeded watchdog: phase={phase} "
                    f"elapsed_ms={elapsed_ns / 1e6:.3f} "
                    f"limit_ms={self.command_timeout_ns / 1e6:.3f}")
            # Keep legacy capped live slew. Slow trajectories use real elapsed
            # time: abruptly capping it would itself jump commanded velocity.
            dt = elapsed_ns / 1e9
            if phase == TELEOP and not self._teleop_held:
                dt = min(dt, 1.0 / self.rate_hz)
            measured = {device: self.check_feedback(device, feedback[device], now_ns, require_enabled=True)
                        for device in self.devices}
            for device in self.devices:
                self._check_tracking(device, self._last_commands[device], measured[device], now_ns,
                                     feedback[device].received_monotonic_ns)
            if phase == TELEOP:
                if self._teleop_held:
                    result = {device: path.advance(dt)
                              for device, path in self._teleop_brakes.items()}
                    self._teleop_velocities = {
                        device: path.velocity for device, path in self._teleop_brakes.items()}
                    self._command_moving = not self._teleop_brakes["arms"].stopped
                else:
                    result = {}
                    for device in self.devices:
                        previous = self._last_commands[device]
                        target = frame.positions(device)
                        self._targets[device] = tuple(float(value) for value in target)
                        result[device] = _slew(
                            previous, target,
                            self.configuration[device]["maximum_speed_rad_s"] * dt)
                        self._teleop_velocities[device] = tuple(
                            (value - old) / (elapsed_ns / 1e9)
                            for value, old in zip(result[device], previous))
                    self._command_moving = result["arms"] != self._last_commands["arms"]
                for device in self.devices:
                    near_hold = max(abs(a - b) for a, b in zip(
                        measured[device], result[device])) <= self.configuration[device]["alignment_rad"]
                    if self._teleop_held:
                        # Contact may hold fingers short of the command. Require
                        # measured rest, not a new grip/opening pose match.
                        stationary = (self._teleop_brakes[device].stopped
                                      and (device != "arms" or near_hold))
                    else:
                        stationary = near_hold and result[device] == self._last_commands[device]
                    self._rest[device].observe(measured[device], feedback[device].received_monotonic_ns,
                                               stationary)
            else:
                result = self._slow_commands(measured, feedback, now_ns, dt)
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
            self.reset_staged_motion()

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
        # Legacy Home holds fingers, allowing contact. Collection first settles
        # arms under the same rule, then explicitly opens and settles all groups.
        settling_devices = (("arms",) if phase in (HOMING, HOME_REACHED)
                            and not (self._collection and self._opening_hands) else self.devices)
        results = [self._rest[device].observe(
            measured[device], feedback[device].received_monotonic_ns,
            self._device_settled(device, commands[device], measured[device]))
            for device in settling_devices]
        settled = all(results)
        if self.paused or self._awaiting_rest or not all(
                path.finished for path in self._trajectories.values()):
            settled = False
        if phase in (ALIGNING, HOMING):
            if settled:
                if phase == HOMING and self._collection and not self._opening_hands:
                    self._opening_hands = True
                    for device in self.devices:
                        if device != "arms":
                            self._targets[device] = self._hand_open
                    self._rest = {
                        device: FeedbackRest(self._rest_speed, self._settle_ns, self.feedback_timeout_ns)
                        for device in self.devices}
                    self.reset_staged_motion()
                    self._check_approach_duration(
                        self._timeout_ns - (now_ns - self._phase_start_ns), 2)
                    return
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
