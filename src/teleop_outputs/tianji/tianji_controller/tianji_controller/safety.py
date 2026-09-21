"""Hardware-boundary checks and bounded setpoint motion; no IK or SDK imports."""
import math
from .protocol import DEVICE_READY_FLAGS


# Model limits are compared against encoder feedback, so a joint parked on a
# limit reads a few encoder counts past it: the vendor SDK reports 1e-4 deg
# steps and this arm's zero pose scatters ~3e-3 deg around the four limits that
# are exactly 0 rad (Joint3_L/Joint4_L upper, Joint3_R/Joint4_R lower).
# A shared 0.01 rad margin applies to target and feedback bounds for all joints.
# Values beyond this margin still trigger the position-limit guard.
_BOUND_SLACK_RAD = 1e-2


class SafetyFault(RuntimeError):
    def __init__(self, message, *, details=None):
        super().__init__(message)
        self.details = details



def _finite_vector(values, count, label):
    if len(values) != count or not all(math.isfinite(value) for value in values):
        raise SafetyFault(f"{label}: expected {count} finite joint values")
    return tuple(float(value) for value in values)


def _slew(previous, target, maximum_step):
    return tuple(value + max(-maximum_step, min(maximum_step, goal - value))
                 for value, goal in zip(previous, target))


class MotionGate:
    def __init__(self, configuration: dict, devices: tuple[str, ...]):
        if not devices or len(set(devices)) != len(devices) or any(
                device not in DEVICE_READY_FLAGS for device in devices):
            raise ValueError("select arms, left_hand and/or right_hand without duplicates")
        self.devices = devices
        self.configuration = configuration
        self.rate_hz = float(configuration["rate_hz"])
        self.command_timeout_ns = float(configuration["command_timeout_s"]) * 1e9
        self.feedback_timeout_ns = float(configuration["feedback_timeout_s"]) * 1e9
        if any(not math.isfinite(value) or value <= 0 for value in
               (self.rate_hz, self.command_timeout_ns, self.feedback_timeout_ns)):
            raise ValueError("rate and watchdog timeouts must be positive and finite")
        self._zero_ramp = {}
        for device in devices:
            settings = configuration[device]
            count = 14 if device == "arms" else 20
            lower = _finite_vector(settings["lower_rad"], count, f"{device} lower limits")
            upper = _finite_vector(settings["upper_rad"], count, f"{device} upper limits")
            if any(lo >= hi for lo, hi in zip(lower, upper)):
                raise ValueError(f"{device}: invalid joint limits")
            for key in ("alignment_rad", "maximum_speed_rad_s", "tracking_error_rad"):
                if not math.isfinite(settings[key]) or settings[key] <= 0:
                    raise ValueError(f"{device}: {key} must be positive and finite")
            # Enabled devices may declare the zero-then-ramp enable sequence
            # instead of the pose-matching one; arms keep the pose match.
            ramp = settings.get("enable_zero_ramp_seconds")
            if ramp is not None:
                if device == "arms" or any(not lo <= 0 <= hi for lo, hi in zip(lower, upper)):
                    raise ValueError(f"{device}: zero-ramp enable requires hand joints whose limits contain zero")
                if type(ramp) not in (int, float) or not math.isfinite(ramp) or ramp <= 0:
                    raise ValueError(f"{device}: enable_zero_ramp_seconds must be positive and finite")
                for key in ("enable_zero_tolerance_rad", "enable_zero_timeout_s"):
                    value = settings.get(key)
                    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                        raise ValueError(f"{device}: {key} must be positive and finite for a zero-ramp enable")
                ramp = float(ramp)
            self._zero_ramp[device] = ramp
        self.armed = False
        self.fault = None
        self._epoch = 0
        self._last_commands = {}
        self._last_step_ns = 0
        self._phase = {device: "live" for device in devices}
        self._phase_start_ns = 0
        self._ramp_start_ns = {}
        self._ramp_pose = {}

    def validate_targets(self, frame, now_ns):
        age = now_ns - frame.timestamp_ns
        if age < -5_000_000 or age > self.command_timeout_ns:
            raise SafetyFault("controller command is stale or its monotonic clock is invalid")
        for device in self.devices:
            flag = DEVICE_READY_FLAGS[device]
            if not frame.flags & flag and not self.allow_unready_hold(device,frame,now_ns):
                raise SafetyFault(f"{device}: source input is not fresh/ready")
            if device == "arms" and frame.tracking_epoch <= 0:
                raise SafetyFault("arms: missing PICO tracking epoch")
            self._check_bounds(device, frame.positions(device), "target")

    def allow_unready_hold(self, device, frame, now_ns):
        return False

    def observe_source(self, frame, now_ns):
        """Latch every unsafe source event, before a receiver coalesces positions."""
        if self.fault:
            raise SafetyFault(self.fault)
        try:
            self.validate_targets(frame, now_ns)
            if self.armed and "arms" in self.devices and frame.tracking_epoch != self._epoch:
                raise SafetyFault("PICO tracking epoch changed during real teleoperation")
        except SafetyFault as error:
            if self.armed:
                self.fault = str(error)
            raise

    def _check_bounds(self, device, values, label):
        settings = self.configuration[device]
        positions = _finite_vector(values, 14 if device == "arms" else 20, f"{device} {label}")
        for index, (value, lower, upper) in enumerate(zip(
                positions, settings["lower_rad"], settings["upper_rad"])):
            if not lower - _BOUND_SLACK_RAD <= value <= upper + _BOUND_SLACK_RAD:
                raise SafetyFault(f"{device} {label} joint {index}: {value:.6f} rad outside [{lower}, {upper}]")
        return positions

    def check_feedback(self, device, measured, now_ns, require_enabled=False):
        age = now_ns - measured.received_monotonic_ns
        if measured.received_monotonic_ns <= 0 or age < -5_000_000 or age > self.feedback_timeout_ns:
            raise SafetyFault(f"{device}: feedback stopped advancing or is stale")
        if not measured.healthy:
            raise SafetyFault(f"{device}: unhealthy device feedback: {measured.detail}")
        if require_enabled and not measured.enabled:
            raise SafetyFault(f"{device}: servo disabled during active teleoperation")
        return self._check_bounds(device, measured.position_rad, "feedback")

    def _check_tracking(self, device, previous, measured, now_ns, feedback_stamp_ns):
        # Hand contact may prevent position tracking during a grasp. Keep the
        # runtime following-error stop for arms only; other guards still apply.
        if device != "arms":
            return
        limit = self.configuration[device]["tracking_error_rad"]
        if max(abs(command - actual) for command, actual in zip(previous, measured)) <= limit:
            return
        # Build diagnostics only on failure; the real-time path keeps the same
        # threshold and never performs file I/O or per-joint formatting.
        index = max(range(len(previous)), key=lambda i: abs(previous[i] - measured[i]))
        joint = (f"Joint{index % 7 + 1}_{'L' if index < 7 else 'R'}"
                 if device == "arms" else f"{device}[{index}]")
        error = previous[index] - measured[index]
        details = {
            "kind": "tracking_error", "device": device, "joint": joint,
            "joint_index": index, "command_rad": previous[index],
            "actual_rad": measured[index], "error_rad": error,
            "absolute_error_rad": abs(error), "limit_rad": limit,
            "feedback_age_ms": (now_ns - feedback_stamp_ns) / 1e6,
            "step_dt_ms": (now_ns - self._last_step_ns) / 1e6,
        }
        raise SafetyFault(
            f"{device}: physical feedback is not following the last command; "
            f"joint={joint} joint_index={index} command_rad={previous[index]:.6f} "
            f"actual_rad={measured[index]:.6f} error_rad={error:.6f} "
            f"absolute_error_deg={math.degrees(abs(error)):.3f} limit_rad={limit:.6f} "
            f"feedback_age_ms={details['feedback_age_ms']:.3f} "
            f"step_dt_ms={details['step_dt_ms']:.3f}", details=details)

    def check_enable_ready(self, frame, feedback, now_ns):
        """Everything the enable step needs; a zero-ramp device owes no pose match."""
        self.validate_targets(frame, now_ns)
        for device in self.devices:
            measured = self.check_feedback(device, feedback[device], now_ns)
            if self._zero_ramp[device] is not None:
                continue
            error = max(abs(target - actual) for target, actual in zip(frame.positions(device), measured))
            if error > self.configuration[device]["alignment_rad"]:
                raise SafetyFault(f"{device}: start pose mismatch {error:.4f} rad; align operator/robot before enabling")

    def arm(self, frame, feedback, now_ns):
        if self.armed or self.fault:
            raise SafetyFault("session already armed/faulted; restart is required")
        self.check_enable_ready(frame, feedback, now_ns)
        self._epoch = frame.tracking_epoch
        self._phase_start_ns = now_ns
        self._ramp_start_ns = {}
        self._ramp_pose = {}
        for device in self.devices:
            # Never command a jump: the setpoint starts at the measured pose and
            # only the commanded trajectory moves it.
            self._last_commands[device] = tuple(feedback[device].position_rad)
            self._phase[device] = "zero" if self._zero_ramp[device] is not None else "live"
        self._last_step_ns = now_ns
        self.armed = True

    def _advance_phase(self, device, frame, measured, now_ns):
        """Drive the zero -> ramp -> live enable sequence for a zero-ramp device."""
        if self._zero_ramp[device] is None or self._phase[device] == "live":
            return "live"
        settings = self.configuration[device]
        if self._phase[device] == "zero":
            if (not any(self._last_commands[device])
                    and max(abs(value) for value in measured) <= settings["enable_zero_tolerance_rad"]):
                # Capture the input pose as it is now; the ramp never chases it.
                self._ramp_pose[device] = frame.positions(device)
                self._ramp_start_ns[device] = now_ns
                self._phase[device] = "ramp"
                return "ramp"
            if (now_ns - self._phase_start_ns) / 1e9 > settings["enable_zero_timeout_s"]:
                raise SafetyFault(f"{device}: did not reach the zero start pose within "
                                  f"{settings['enable_zero_timeout_s']:g} s")
            return "zero"
        progress = (now_ns - self._ramp_start_ns[device]) / (self._zero_ramp[device] * 1e9)
        if progress < 1.0:
            return "ramp"
        arrived = max(abs(target - actual) for target, actual in zip(self._ramp_pose[device], measured))
        if arrived > settings["tracking_error_rad"]:
            raise SafetyFault(f"{device}: did not reach the input pose within the "
                              f"{self._zero_ramp[device]:g} s enable ramp ({arrived:.4f} rad)")
        self._phase[device] = "live"
        return "ramp"

    def step(self, frame, feedback, now_ns):
        if not self.armed or self.fault:
            raise SafetyFault(self.fault or "hardware output requires explicit arming")
        try:
            self.observe_source(frame, now_ns)
            if now_ns <= self._last_step_ns:
                raise SafetyFault("execution monotonic clock did not advance")
            if any(phase == "ramp" for phase in self._phase.values()) and now_ns - self._last_step_ns > self.command_timeout_ns:
                raise SafetyFault("execution stalled during enable ramp")
            dt = min((now_ns - self._last_step_ns) / 1e9, 1.0 / self.rate_hz)
            result = {}
            for device in self.devices:
                measured = self.check_feedback(device, feedback[device], now_ns, require_enabled=True)
                previous = self._last_commands[device]
                settings = self.configuration[device]
                self._check_tracking(device, previous, measured, now_ns,
                                     feedback[device].received_monotonic_ns)
                phase = self._advance_phase(device, frame, measured, now_ns)
                if phase == "zero":
                    # Convergence is bounded by enable_zero_timeout_s; the
                    # setpoint only walks toward zero at the configured speed.
                    result[device] = _slew(previous, (0.0,) * len(previous),
                                           settings["maximum_speed_rad_s"] * dt)
                    continue
                if phase == "ramp":
                    progress = min(1.0, (now_ns - self._ramp_start_ns[device]) / (self._zero_ramp[device] * 1e9))
                    result[device] = tuple(progress * value for value in self._ramp_pose[device])
                    continue
                result[device] = _slew(previous, frame.positions(device),
                                       settings["maximum_speed_rad_s"] * dt)
            self._last_commands = result
            self._last_step_ns = now_ns
            return result
        except SafetyFault as error:
            self.fault = str(error)
            raise
