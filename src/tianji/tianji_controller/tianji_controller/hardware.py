"""Read-before-enable I/O for the Marvin arm pair.

Imports and constructors never load the vendor SDK. A stopped or closed session
cannot be re-enabled: create a new session and repeat the measured-state startup
gate. Hand I/O lives in ``wuji_controller.hardware``.
"""
from __future__ import annotations

import importlib.util
import ipaddress
import logging
import math
from pathlib import Path
import threading
import time

from tianji_runtime import device as _runtime
from tianji_runtime.device import Feedback

# Existing call sites use the historical private spellings; keep them as aliases
# of the single shared implementation instead of duplicating the validation.
_TIMEOUT_NS = _runtime.TIMEOUT_NS
_advances = _runtime.advances
_fresh = _runtime.fresh
_positions = _runtime.positions
fresh = _runtime.fresh
positions = _runtime.positions

_LOG = logging.getLogger(__name__)



def _load_marvin(directory):
    spec = importlib.util.spec_from_file_location('_teleop_marvin_sdk', directory / 'fx_robot.py')
    if spec is None or spec.loader is None:
        raise RuntimeError('cannot load Marvin SDK module')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.Marvin_Robot(), module.DCSS()


def _check_guard(guard):
    if guard is not None and guard() is False:
        raise RuntimeError('source enable guard rejected authorization')


# The vendor library prints on every servo-error query, so poll that descriptive
# detail far slower than the payload-derived feedback, which stays at full rate.
_SERVO_DETAIL_INTERVAL_NS = 1_000_000_000


def _cleanup_on_error(device, original):
    try:
        device.close()
    except BaseException as cleanup:
        original.add_note(f'Hardware cleanup also failed: {cleanup}')


class MarvinDevice:
    def __init__(self, sdk_directory: Path, ip: str, velocity_ratio=10, acceleration_ratio=10):
        self.sdk_directory = Path(sdk_directory)
        self.ip = str(ipaddress.IPv4Address(ip))
        for name, value in (('velocity_ratio', velocity_ratio), ('acceleration_ratio', acceleration_ratio)):
            if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 100:
                raise ValueError(f'{name} must be an integer in 1..100')
        self.velocity_ratio = velocity_ratio
        self.acceleration_ratio = acceleration_ratio
        self._robot = None
        self._dcss = None
        self._connected = False
        self._closed = False
        self._stopped = False
        self._stop_completed = False
        self._owns_enable = False
        self._enabled = False
        self._position_transition_deadline_ns = 0
        self._required_states = (1, 1)
        self._mode_transition = False
        self._right_impedance = None
        self._inputs = ()
        self._serials = [None, None]
        self._stamps = [0, 0]
        self._states = ()
        self._commands = ()
        self._ratios = ((), ())
        self._servo_reports = ('None', 'None')
        self._servo_checked_ns = -1
        self._feedback = Feedback((), 0, False, False, 'not connected')

    @staticmethod
    def _success(result, operation):
        # Verified Marvin setters return 1, NOT the Wuji convention of zero.
        if result != 1:
            raise RuntimeError(f'Marvin {operation} failed: {result!r}')

    def _begin_batch(self):
        # OnClearSet can transiently reject an unconsumed 1kHz command buffer.
        # Only retry this pre-write boundary. Never retry a setter or send.
        for attempt in range(3):
            result = self._robot.clear_set()
            if result == 1:
                return
            if attempt < 2:
                time.sleep(.002)
        raise RuntimeError(f'Marvin clear_set failed after 3 attempts: {result!r}')

    def connect(self):
        if self._closed or self._connected:
            raise RuntimeError('Marvin session is closed or already connected')
        try:
            self._robot, self._dcss = _load_marvin(self.sdk_directory)
            self._success(self._robot.connect(self.ip), 'connect')
            # This local SDK switch silences its printf diagnostics; it does not
            # change controller logging or skip any feedback/error-code checks.
            self._robot.local_log_switch('0')
            self._connected = True
            self._wait_feedback(lambda feedback: feedback.healthy)
        except BaseException as exc:
            _cleanup_on_error(self, exc)
            raise

    def read_feedback(self):
        if not self._connected:
            return Feedback((), 0, False, False, 'not connected')
        try:
            payload = self._robot.subscribe(self._dcss)
            outputs, states, inputs = payload['outputs'], payload['states'], payload['inputs']
            if len(outputs) != 2 or len(states) != 2 or len(inputs) != 2:
                raise ValueError('expected two arms')
            self._inputs = inputs
            q = tuple(math.radians(value) for arm in outputs for value in _positions(arm['fb_joint_pos'], 7))
            self._states = tuple(int(arm['cur_state']) for arm in states)
            self._commands = tuple(int(arm['cmd_state']) for arm in states)
            errors = tuple(int(arm['err_code']) for arm in states)
            self._ratios = (tuple(int(arm['joint_vel_ratio']) for arm in inputs),
                            tuple(int(arm['joint_acc_ratio']) for arm in inputs))
            # Only the descriptive servo query is rate-limited: a non-zero payload
            # error forces it immediately so a fault is never masked or delayed.
            query_now = time.monotonic_ns()
            due = self._servo_checked_ns < 0 or query_now - self._servo_checked_ns >= _SERVO_DETAIL_INTERVAL_NS
            if errors != (0, 0) or due:
                reports = tuple(self._robot.get_servo_error_code(arm, lang='EN') for arm in ('A', 'B'))
                self._servo_reports = reports
                self._servo_checked_ns = query_now
            else:
                reports = self._servo_reports
            now = time.monotonic_ns()
            serials_valid = True
            for index, output in enumerate(outputs):
                serial = output['frame_serial']
                if isinstance(serial, bool) or not isinstance(serial, int) or not 0 <= serial < 1_000_000:
                    raise ValueError('invalid arm frame serial')
                previous = self._serials[index]
                if previous is None:
                    self._serials[index] = serial
                elif _advances(serial, previous, 1_000_000):
                    self._serials[index] = serial
                    self._stamps[index] = now
                elif serial != previous:
                    serials_valid = False
            stamp = min(self._stamps)
            measured_enabled = self._states == self._required_states
            # Vendor FxRtCSDef.h: 101 is TRANS_TO_POSITION, not a servo fault.
            # Permit it only during our bounded enable; it never means enabled.
            transitioning = now < self._position_transition_deadline_ns
            healthy = (serials_valid and _fresh(stamp, now) and errors == (0, 0)
                       and reports == ('None', 'None')
                       and all(state in (0, expected) or
                               (transitioning and state in ((1, 3, 101, 103) if self._mode_transition else (101,)))
                               for state, expected in zip(self._states, self._required_states))
                       and all(command in (-1, state) or
                               (transitioning and command in self._required_states)
                               for command, state in zip(self._commands, self._states)))
            if transitioning and self._mode_transition:
                healthy = healthy and self._states[0] == 1 and self._states[1] in (1, 3, 101, 103)
            if self._enabled:
                healthy = healthy and (measured_enabled or (transitioning and self._mode_transition)) and self._ratios_match()
                if not transitioning and self._required_states[1] == 3:
                    healthy = healthy and self._impedance_matches()
            if transitioning and self._mode_transition and healthy:
                # The owned servos stay enabled throughout a guarded mode switch.
                measured_enabled = True
            detail = '' if healthy else (f'Marvin stale/fault/state: states={self._states}, commands={self._commands}, '
                                         f'errors={errors}, servo={reports}, ratios={self._ratios}')
            self._feedback = Feedback(q, stamp, healthy, measured_enabled, detail)
        except Exception as exc:
            self._feedback = Feedback(self._feedback.position_rad, self._feedback.received_monotonic_ns,
                                      False, False, f'invalid Marvin feedback: {exc}')
        return self._feedback

    def _ratios_match(self):
        return all(expected - 1 <= measured <= expected
                   for values, expected in zip(self._ratios, (self.velocity_ratio, self.acceleration_ratio))
                   for measured in values)

    def _impedance_matches(self):
        stiffness, damping, tool_kinematics, tool_dynamics = self._right_impedance
        values = self._inputs[1]
        return (values["imp_type"] == 1 and all(
            len(values[key]) == len(expected) and all(
                math.isfinite(actual) and abs(actual - wanted) <= 1e-3
                for actual, wanted in zip(values[key], expected))
            for key, expected in (("joint_k", stiffness), ("joint_d", damping),
                                  ("tool_kine", tool_kinematics), ("tool_dyn", tool_dynamics))))

    def set_right_impedance(self, enabled, *, stiffness=None, damping=None,
                            tool_kinematics=None, tool_dynamics=None, guard=None):
        """Switch only the right arm, verifying state, K/D and payload echoes.

        SDK K is N m/deg, D is the vendor damping coefficient; tool translation
        and COM are mm, mass kg, inertia kg mm². Never convert these to SI again.
        The left arm remains in its existing position-hold mode.
        """
        if not self._enabled or self._closed or self._stopped:
            raise RuntimeError("impedance transition requires owned enabled arms")
        wanted = (1, 3 if enabled else 1)
        if wanted == self._required_states:
            return
        if enabled:
            stiffness = _positions(stiffness, 7)
            damping = _positions(damping, 7)
            tool_kinematics = _positions(tool_kinematics, 6)
            tool_dynamics = _positions(tool_dynamics, 10)
            if any(not 0 <= k <= 22 for k in stiffness) or any(not 0 <= d <= 1 for d in damping) or tool_dynamics[0] < 0:
                raise ValueError("right impedance parameters exceed SDK ranges")
            self._right_impedance = (stiffness, damping, tool_kinematics, tool_dynamics)
        try:
            _check_guard(guard)
            measured = self.read_feedback()
            if not measured.healthy or not measured.enabled:
                raise RuntimeError("impedance transition requires fresh healthy feedback")
            serials = tuple(self._serials)
            self._begin_batch()
            self._write_positions(measured.position_rad)
            if enabled:
                self._success(self._robot.set_tool('B', tool_kinematics, tool_dynamics), 'right tool payload')
                self._success(self._robot.set_joint_kd_params('B', stiffness, damping), 'right joint K/D')
                self._success(self._robot.set_impedance_type('B', 1), 'right joint impedance type')
            self._required_states = wanted
            self._mode_transition = True
            self._position_transition_deadline_ns = time.monotonic_ns() + _TIMEOUT_NS
            self._success(self._robot.set_state('B', wanted[1]), 'right control mode')
            _check_guard(guard)
            self._success(self._robot.send_cmd(), 'right control mode send')
            if enabled:
                self._success(self._robot.set_PD_vel_est_step('B', 5), 'right velocity estimate 5 ms')
            self._wait_feedback(
                lambda value: value.healthy and self._states == wanted and
                (not enabled or self._impedance_matches()), after=serials, guard=guard)
        except BaseException:
            self.stop()
            raise
        finally:
            self._position_transition_deadline_ns = 0
            self._mode_transition = False

    def _wait_feedback(self, predicate, after=None, guard=None):
        deadline = time.monotonic_ns() + _TIMEOUT_NS
        while True:
            _check_guard(guard)
            feedback = self.read_feedback()
            _check_guard(guard)
            advanced = after is None or all(_advances(value, old, 1_000_000)
                                            for value, old in zip(self._serials, after))
            if advanced and predicate(feedback):
                return feedback
            if time.monotonic_ns() >= deadline:
                raise RuntimeError('Marvin feedback did not reach fresh requested state: '
                                   f'states={self._states}, commands={self._commands}, '
                                   f'enabled={feedback.enabled}, advanced={advanced}; {feedback.detail}')
            time.sleep(.01)

    def enable(self, guard=None, *, allow_enabled=False):
        """Enable idle arms, or explicitly adopt healthy enabled arms for HOME."""
        if not self._connected or self._closed or self._stopped or self._enabled:
            raise RuntimeError('Marvin cannot enable this session')
        try:
            _check_guard(guard)
            before = self.read_feedback()
            adopting = self._states == (1, 1) and allow_enabled
            if not before.healthy or (self._states != (0, 0) and not adopting):
                raise RuntimeError('Marvin enable requires healthy idle arms, not an existing control session')
            if adopting:
                # Explicit HOME authorization owns stop/disable even if setup fails.
                self._owns_enable = True
            serials = tuple(self._serials)
            self._begin_batch()
            _check_guard(guard)
            if adopting:
                # Replace any old target with measured pose before the first send.
                self._write_positions(before.position_rad)
            for arm in ('A', 'B'):
                _check_guard(guard)
                self._success(self._robot.set_vel_acc(arm, self.velocity_ratio, self.acceleration_ratio), f'set_vel_acc({arm})')
            _check_guard(guard)
            self._success(self._robot.send_cmd(), 'send_cmd(ratios)')
            expected_states = (1, 1) if adopting else (0, 0)
            measured = self._wait_feedback(lambda feedback: feedback.healthy and self._states == expected_states
                                           and self._ratios_match(), after=serials, guard=guard)
            if adopting:
                self._enabled = True
                return
            serials = tuple(self._serials)
            self._begin_batch()
            _check_guard(guard)
            self._write_positions(measured.position_rad)
            self._position_transition_deadline_ns = time.monotonic_ns() + _TIMEOUT_NS
            for arm in ('A', 'B'):
                _check_guard(guard)
                # From the first state write cleanup must assume partial enable.
                self._owns_enable = True
                self._success(self._robot.set_state(arm, 1), f'set_state({arm},1)')
            _check_guard(guard)
            self._success(self._robot.send_cmd(), 'send_cmd(enable)')
            self._wait_feedback(lambda feedback: feedback.healthy and feedback.enabled and self._ratios_match(),
                                after=serials, guard=guard)
            self._enabled = True
        except BaseException as exc:
            _cleanup_on_error(self, exc)
            raise
        finally:
            self._position_transition_deadline_ns = 0

    def _write_positions(self, position_rad):
        for arm, joints in (('A', position_rad[:7]), ('B', position_rad[7:])):
            self._success(self._robot.set_joint_cmd_pose(arm, [math.degrees(value) for value in joints]), f'set_joint_cmd_pose({arm})')

    def send(self, position_rad):
        q = _positions(position_rad, 14)
        if not self._enabled or self._stopped or self._closed:
            raise RuntimeError('Marvin send requires explicit enable')
        feedback = self.read_feedback()
        if not feedback.healthy or not feedback.enabled:
            raise RuntimeError(feedback.detail or 'Marvin is not enabled')
        self._begin_batch()
        self._write_positions(q)
        self._success(self._robot.send_cmd(), 'send_cmd')

    def stop(self):
        self._stopped = True
        self._enabled = False
        if self._connected and self._owns_enable and not self._stop_completed:
            self._robot.soft_stop('AB')  # Verified void safety command; no success value.
            self._stop_completed = True

    def close(self):
        if self._closed:
            return
        errors = []
        if self._connected:
            if self._owns_enable:
                try:
                    self.stop()
                except BaseException as exc:
                    errors.append(exc)
                try:
                    serials = tuple(self._serials)
                    self._begin_batch()
                    for arm in ('A', 'B'):
                        self._success(self._robot.set_state(arm, 0), f'set_state({arm},0)')
                    self._success(self._robot.send_cmd(), 'send_cmd(disable)')
                    self._wait_feedback(lambda feedback: feedback.healthy and self._states == (0, 0), after=serials)
                except BaseException as exc:
                    errors.append(exc)
            try:
                self._success(self._robot.release_robot(), 'release_robot')
            except BaseException as exc:
                errors.append(exc)
        self._connected = self._enabled = self._owns_enable = False
        self._closed = True
        if errors:
            raise RuntimeError('Marvin cleanup failed: ' + '; '.join(map(str, errors)))


