"""Explicit, read-before-enable I/O for both Marvin arms and either Hand2 side.

Imports and constructors do not load SDKs. A stopped/closed session cannot be
re-enabled; create a new session and repeat the measured-state startup gate.
"""
from __future__ import annotations

import ctypes as ct
from dataclasses import dataclass
import importlib.util
import ipaddress
import logging
import math
from pathlib import Path
import threading
import time

from . import _wuji_abi as abi

_LOG = logging.getLogger(__name__)


@dataclass(frozen=True)
class Feedback:
    position_rad: tuple[float, ...]
    received_monotonic_ns: int
    healthy: bool
    enabled: bool
    detail: str = ''


_FRESH_NS = 300_000_000
_TIMEOUT_NS = 2_000_000_000
# The vendor library prints on every servo-error query, so poll that descriptive
# detail far slower than the payload-derived feedback, which stays at full rate.
_SERVO_DETAIL_INTERVAL_NS = 1_000_000_000
# Firmware motor NIDs are 1-based, four motors then one tactile slot per finger.
_DENSE_BY_NID = {finger * 5 + motor + 1: finger * 4 + motor
                 for finger in range(5) for motor in range(4)}


def _positions(values, count):
    values = tuple(float(value) for value in values)
    if len(values) != count or not all(math.isfinite(value) for value in values):
        raise ValueError(f'expected {count} finite joint positions in radians')
    return values


def _fresh(stamp, now):
    return stamp > 0 and 0 <= now - stamp <= _FRESH_NS


def _advances(current, previous, modulus):
    return 0 < (current - previous) % modulus < modulus // 2


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
            measured_enabled = all(state == 1 for state in self._states)
            # Vendor FxRtCSDef.h: 101 is TRANS_TO_POSITION, not a servo fault.
            # Permit it only during our bounded enable; it never means enabled.
            transitioning = now < self._position_transition_deadline_ns
            healthy = (serials_valid and _fresh(stamp, now) and errors == (0, 0)
                       and reports == ('None', 'None')
                       and all(state in (0, 1) or (transitioning and state == 101)
                               for state in self._states)
                       and all(command in (-1, state) or
                               (transitioning and state == 101 and command == 1)
                               for command, state in zip(self._commands, self._states)))
            if self._enabled:
                healthy = healthy and measured_enabled and self._ratios_match()
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


class _Hand2Runtime:
    """One initialized SDK per canonical library path, held by owned sessions."""

    _lock = threading.Lock()
    _libraries = {}

    def __init__(self, path):
        self.path = path
        self.sdk = abi.load_sdk(path)
        abi.check(self.sdk, self.sdk.wuji_init(None), 'wuji_init')
        self.serials = set()

    @classmethod
    def acquire(cls, path, serial):
        path = path.resolve()
        with cls._lock:
            runtime = cls._libraries.get(path)
            if runtime is None:
                runtime = cls(path)
                cls._libraries[path] = runtime
            if serial in runtime.serials:
                raise RuntimeError(f'Hand2 serial {serial!r} already has an owned session')
            # Reserve before connecting: a failed second claim must never obtain
            # a handle that could disconnect an already-active device.
            runtime.serials.add(serial)
            return runtime

    def release(self, serial):
        with self._lock:
            self.serials.remove(serial)
            if not self.serials:
                del self._libraries[self.path]
                # Keep acquire blocked through shutdown; a new init must not
                # race with the previous runtime's teardown.
                self.sdk.wuji_shutdown()


class Hand2Device:
    """Hand2 motor authority with optional MIT gains.

    ``kp``/``kd`` of ``None`` means "use the device's own values": they are read
    back after connect and written unchanged at enable. These are device current
    values, not proof of factory defaults; motion is shaped by the trajectory.
    """

    def __init__(self, sdk_library: Path, serial: str, side: str, kp=None, kd=None, effort_limit_amps=1.5):
        if side not in ('left', 'right'):
            raise ValueError('Hand2 side must be explicitly left or right')
        if (not isinstance(serial, str) or not serial or not serial.isascii()
                or any(character.isspace() or not character.isprintable() for character in serial)):
            raise ValueError('an explicit nonblank Hand2 serial without whitespace or control characters is required')
        if (kp is None) != (kd is None):
            raise ValueError('configure hand kp and kd together, or leave both unset for the device values')
        for name, value in (('kp', kp), ('kd', kd), ('effort_limit_amps', effort_limit_amps)):
            if value is None:
                continue
            if not math.isfinite(value) or value < 0 or not math.isfinite(ct.c_float(value).value):
                raise ValueError(f'{name} must be finite and nonnegative')
        if effort_limit_amps <= 0:
            raise ValueError('effort_limit_amps must be positive')
        self.sdk_library, self.serial, self.side = Path(sdk_library), serial, side
        self._alias = f'tianji_{side}_hand_{id(self):x}'.encode('ascii')
        self.kp, self.kd, self.effort_limit_amps = kp, kd, effort_limit_amps
        self.device_kp = self.device_kd = None
        self._sdk = None
        self._dev = ct.c_void_p()
        self._states_sub = ct.c_void_p()
        self._diagnostics_sub = ct.c_void_p()
        self._publisher = ct.c_void_p()
        self._runtime = None
        self._closed = False
        self._stopped = False
        self._stop_completed = False
        self._enabled = False
        self._owns_enable = False
        self._side_verified = False
        self._lock = threading.Lock()
        self._state_callback = None
        self._diagnostics_callback = None
        self._q = ()
        self._state_key = None
        self._diag_key = None
        self._state_ns = self._diag_ns = 0
        self._state_valid = self._diag_valid = False
        self._measured_enabled = False
        self._all_ready = False
        self._state_detail = self._diag_detail = 'no complete advancing frames'
        self._fault_detail = ''
        self._seen_enabled = False
        self._error_descriptions = {}
        self._warnings = {}
        self._warning_detail = ''

    def _call(self, name, *arguments):
        abi.check(self._sdk, getattr(self._sdk, name)(*arguments), name)

    @staticmethod
    def _mapped(frame):
        if not frame.joints or frame.joints_len != 20 or frame.num_joints != 20:
            raise ValueError('expected exactly 20 joint entries')
        mapped = []
        seen = set()
        for index in range(20):
            entry = frame.joints[index]
            dense = _DENSE_BY_NID.get(entry.nid)
            if dense is None or dense in seen:
                raise ValueError('invalid/duplicate firmware motor NID')
            seen.add(dense)
            mapped.append((dense, entry))
        return mapped

    @staticmethod
    def _frame_advances(key, previous):
        # Both fields must advance; repeated cached frames never acquire new age.
        return (previous is not None and _advances(key[0], previous[0], 1 << 32)
                and key[1] > previous[1])

    def _latch_fault(self, detail):
        # Called under _lock; pre-enable discovery is intentionally recoverable.
        if self._owns_enable and not self._stopped and not self._fault_detail:
            self._fault_detail = detail

    def _on_states(self, kind, pointer, unused):
        with self._lock:
            try:
                if kind != 0 or not pointer:
                    raise ValueError(f'state stream kind={kind}')
                frame = pointer.contents
                q = [0.0] * 20
                for dense, entry in self._mapped(frame):
                    if not all(math.isfinite(value) for value in (entry.position, entry.velocity, entry.effort)):
                        raise ValueError('nonfinite hand state')
                    q[dense] = float(entry.position)
                key = (frame.header.seq, frame.header.timestamp_us)
                if self._state_key is None:
                    self._state_key = key
                    return
                if not self._frame_advances(key, self._state_key):
                    raise ValueError('hand state frame did not advance')
                self._state_key = key
                self._q = tuple(q)
                self._state_ns = time.monotonic_ns()
                self._state_valid = True
                self._state_detail = ''
            except Exception as exc:
                self._state_valid = False
                self._state_detail = str(exc)
                self._latch_fault(self._state_detail)

    def _describe_error(self, code):
        description = self._error_descriptions.get(code)
        if description is None:
            info = abi.ErrorInfo()
            # The SDK catalog lookup is static: no device RPC in the callback.
            self._call('wuji_hand_2_describe_error', code, ct.byref(info))
            if info.code != code or info.is_test:
                raise ValueError(f'unrecognized/test-only diagnostic code={code}')
            severity = info.severity.decode('utf-8')
            text = (f'code={code} name={info.name.decode("utf-8")} severity={severity} '
                    f'clear={info.clear_policy.decode("utf-8")}; {info.desc.decode("utf-8")}; '
                    f'cause={info.cause.decode("utf-8")}; resolution={info.resolution.decode("utf-8")}')
            description = (severity, text)
            self._error_descriptions[code] = description
        return description

    def _on_diagnostics(self, kind, pointer, unused):
        with self._lock:
            try:
                if kind != 0 or not pointer:
                    raise ValueError(f'diagnostics stream kind={kind}')
                frame = pointer.contents
                entries = self._mapped(frame)
                states = []
                warnings = {}
                for _, entry in entries:
                    if not all(math.isfinite(value) for value in
                               (entry.current, entry.vbus_v_fb, entry.mcu_temp_c_fb)):
                        raise ValueError(f'hand joint nid={entry.nid}: nonfinite diagnostic values')
                    if entry.error_code_current:
                        identity = f'Hand2 {self.side} SN={self.serial} nid={entry.nid}'
                        try:
                            severity, description = self._describe_error(entry.error_code_current)
                        except Exception as error:
                            raise ValueError(f'{identity} code={entry.error_code_current}: '
                                             f'unknown/unreadable diagnostic: {error}') from error
                        if severity != 'Warning':
                            raise ValueError(f'{identity} {description}')
                        warnings[entry.nid] = f'{identity} {description}'
                    state = entry.status_word & 3
                    if state not in (1, 2):
                        raise ValueError(f'hand joint nid={entry.nid} status=0x{entry.status_word:x}')
                    states.append(state)
                key = (frame.header.seq, frame.header.timestamp_us)
                if self._diag_key is None:
                    self._diag_key = key
                    return
                if not self._frame_advances(key, self._diag_key):
                    raise ValueError('hand diagnostics frame did not advance')
                self._diag_key = key
                self._diag_ns = time.monotonic_ns()
                self._diag_valid = True
                self._measured_enabled = all(state == 2 for state in states)
                if self._owns_enable and self._measured_enabled:
                    self._seen_enabled = True
                elif self._seen_enabled and not self._stopped:
                    raise ValueError('hand joint left Enabled state during owned session')
                self._all_ready = all(state == 1 for state in states)
                if warnings != self._warnings:
                    for nid, message in warnings.items():
                        if self._warnings.get(nid) != message:
                            _LOG.warning('%s', message)
                    for nid in self._warnings.keys() - warnings.keys():
                        _LOG.info('Hand2 %s SN=%s nid=%s warning cleared', self.side, self.serial, nid)
                    self._warnings = warnings
                    self._warning_detail = '; '.join(warnings.values())
                self._diag_detail = ''
            except Exception as exc:
                self._diag_valid = False
                self._measured_enabled = self._all_ready = False
                self._diag_detail = str(exc)
                self._latch_fault(self._diag_detail)

    def read_feedback(self):
        with self._lock:
            now = time.monotonic_ns()
            stamp = min(self._state_ns, self._diag_ns)
            healthy = (bool(self._dev.value) and self._side_verified and self._state_valid and self._diag_valid
                       and _fresh(stamp, now) and not self._fault_detail)
            if self._enabled:
                healthy = healthy and self._measured_enabled
            return Feedback(self._q, stamp, healthy, self._measured_enabled,
                            self._warning_detail if healthy else
                            f'Hand2 {self.side} SN={self.serial} stale/invalid feedback: '
                            f'{self._fault_detail}; {self._state_detail}; {self._diag_detail}')

    def _wait_feedback(self, enabled=False, after=0, guard=None):
        deadline = time.monotonic_ns() + _TIMEOUT_NS
        while True:
            _check_guard(guard)
            feedback = self.read_feedback()
            _check_guard(guard)
            with self._lock:
                fault = self._fault_detail
            if fault:
                raise RuntimeError(f'Hand2 owned-session fault: {fault}')
            if (feedback.healthy and feedback.received_monotonic_ns > after
                    and (not enabled or feedback.enabled)):
                return feedback
            if time.monotonic_ns() >= deadline:
                raise RuntimeError('Hand2 requires fresh measured state and diagnostics before commands: ' + feedback.detail)
            time.sleep(.01)

    def connect(self):
        if self._closed or self._dev.value:
            raise RuntimeError('Hand2 session is closed or already connected')
        try:
            self._runtime = _Hand2Runtime.acquire(self.sdk_library, self.serial)
            self._sdk = self._runtime.sdk
            target = abi.ConnectTarget(0, self.serial.encode('utf-8'))
            options = self._sdk.wuji_connect_options_default()
            options.timeout_ms, options.retry_count = 1000, 0
            # Motor authority must not share the device with another SDK process.
            options.enable_bridge = False
            options.auto_time_sync_interval_enabled = False
            self._call('wuji_connect', ct.byref(target), self._alias, ct.byref(options), ct.byref(self._dev))
            if not self._dev.value:
                raise RuntimeError('Hand2 SDK returned null device')
            self._check_side()
            self._state_callback = abi.StateCallback(self._on_states)
            self._latch_gains()
            self._diagnostics_callback = abi.DiagnosticsCallback(self._on_diagnostics)
            # No userdata pointer: callback closures hold this device until join.
            self._call('wuji_hand_2_subscribe_joint_states', self._dev, self._state_callback, None, ct.byref(self._states_sub))
            if not self._states_sub.value:
                raise RuntimeError('Hand2 SDK returned null state subscription')
            self._call('wuji_hand_2_subscribe_joint_diagnostics', self._dev, self._diagnostics_callback, None, ct.byref(self._diagnostics_sub))
            if not self._diagnostics_sub.value:
                raise RuntimeError('Hand2 SDK returned null diagnostics subscription')
            # Some firmware may not stream before enable. Never enable to discover q0.
            self._wait_feedback()
        except BaseException as exc:
            _cleanup_on_error(self, exc)
            raise

    def _check_side(self):
        self._side_verified = False
        side = ct.c_int(-1)
        self._call('wuji_hand_2_get_handedness', self._dev, ct.byref(side))
        self._side_verified = side.value == (0 if self.side == 'left' else 1)
        if not self._side_verified:
            raise RuntimeError(f'refusing Hand2 handedness={side.value}; {self.side.upper()} required')

    def _latch_gains(self):
        """Read the gains the device already runs with; never assume the vendor example values."""
        kp, kd = (ct.c_float * 20)(), (ct.c_float * 20)()
        online = ct.c_uint32()
        self._call('wuji_hand_2_get_all_mit_params', self._dev, kp, kd, ct.byref(online))
        values = [(float(kp[index]), float(kd[index])) for index in range(20)]
        if online.value & ((1 << 20) - 1) != (1 << 20) - 1 or any(
                not math.isfinite(gain) or gain < 0 for pair in values for gain in pair):
            raise RuntimeError('Hand2 MIT gains require 20 online joints with finite nonnegative values')
        self.device_kp = tuple(pair[0] for pair in values)
        self.device_kd = tuple(pair[1] for pair in values)
        print(f'Hand2 {self.side} {self.serial} device current MIT gains '
              f'kp={self.device_kp} kd={self.device_kd} (20/20 online)', flush=True)

    def enable(self, guard=None):
        if not self._dev.value or self._closed or self._stopped or self._enabled:
            raise RuntimeError('Hand2 cannot enable this session')
        try:
            _check_guard(guard)
            self._check_side()
            feedback = self.read_feedback()
            with self._lock:
                all_ready = self._all_ready
            if not feedback.healthy or not all_ready:
                raise RuntimeError('Hand2 enable requires 20 healthy Ready joints, not an existing session')
            online = ct.c_uint8()
            self._call('wuji_hand_2_online_joints_count', self._dev, ct.byref(online))
            if online.value != 20:
                raise RuntimeError('Hand2 requires 20/20 online motors')
            _check_guard(guard)
            self._call('wuji_hand_2_set_all_effort_limit', self._dev, self.effort_limit_amps)
            gains = (self.kp, self.kd) if self.kp is not None else (self.device_kp, self.device_kd)
            if any(gain is None for gain in gains):
                raise RuntimeError('Hand2 gains were never read from the device; connect before enable')
            kp_values = (gains[0],) * 20 if self.kp is not None else gains[0]
            kd_values = (gains[1],) * 20 if self.kd is not None else gains[1]
            kp, kd = (ct.c_float * 20)(*kp_values), (ct.c_float * 20)(*kd_values)
            _check_guard(guard)
            # Device current gains are written back unchanged; soft enable motion comes
            # from the commanded trajectory, not from lowering the gains.
            self._call('wuji_hand_2_set_all_mit_params', self._dev, kp, kd)
            print(f'Hand2 {self.side} enable: effort<={self.effort_limit_amps:g} A, '
                  f'kp={kp_values} kd={kd_values} '
                  f'({"configured" if self.kp is not None else "device current values"})', flush=True)
            self._check_side()
            # Recheck freshness after blocking RPCs; configurations are not enable.
            feedback = self.read_feedback()
            with self._lock:
                all_ready = self._all_ready
            if not feedback.healthy or not all_ready:
                raise RuntimeError('Hand2 feedback changed during configuration')
            started = time.monotonic_ns()
            _check_guard(guard)
            self._owns_enable = True
            self._call('wuji_hand_2_enable', self._dev, None)
            self._wait_feedback(enabled=True, after=started, guard=guard)
            _check_guard(guard)
            self._call('wuji_hand_2_joint_command_publish', self._dev, ct.byref(self._publisher))
            if not self._publisher.value:
                raise RuntimeError('Hand2 SDK returned null publisher')
            _check_guard(guard)
            self._enabled = True
        except BaseException as exc:
            _cleanup_on_error(self, exc)
            raise

    def send(self, position_rad):
        q = _positions(position_rad, 20)
        if not self._enabled or self._stopped or self._closed or not self._publisher.value:
            raise RuntimeError('Hand2 send requires explicit enable')
        feedback = self.read_feedback()
        if not feedback.healthy or not feedback.enabled:
            raise RuntimeError(feedback.detail or 'Hand2 not enabled')
        commands = (abi.JointCommand * 20)()
        for command, value in zip(commands, q):
            command.position = value
            if not math.isfinite(command.position):
                raise ValueError('Hand2 position exceeds float32 range')
            # ctypes zero initializes velocity and effort: position-only MIT input.
        self._call('wuji_joint_command_publisher_send', self._publisher, commands)

    def stop(self):
        self._stopped = True
        self._enabled = False
        if self._dev.value and self._owns_enable and not self._stop_completed:
            self._call('wuji_hand_2_disable', self._dev, None)
            self._stop_completed = True

    def close(self):
        if self._closed:
            return
        errors = []
        try:
            self.stop()
        except BaseException as exc:
            errors.append(exc)
        # wuji_sub_close joins; callbacks and their closures must outlive it.
        for handle in (self._states_sub, self._diagnostics_sub):
            if handle.value:
                self._sdk.wuji_sub_close(handle)
                handle.value = None
        self._state_callback = self._diagnostics_callback = None
        if self._publisher.value:
            self._sdk.wuji_joint_command_publisher_close(self._publisher)
            self._publisher.value = None
        if self._dev.value:
            try:
                self._call('wuji_dev_disconnect', self._dev)
            except BaseException as exc:
                errors.append(exc)
            finally:
                self._sdk.wuji_dev_release(self._dev)
                self._dev.value = None
        if self._runtime is not None:
            try:
                self._runtime.release(self.serial)
            except BaseException as exc:
                errors.append(exc)
            self._runtime = None
        self._enabled = self._owns_enable = False
        self._closed = True
        if errors:
            raise RuntimeError('Hand2 cleanup failed: ' + '; '.join(map(str, errors)))
