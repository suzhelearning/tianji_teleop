"""Device-boundary regressions: only in-process fake SDKs, never hardware."""
import ctypes as ct
from contextlib import redirect_stdout
import io
import math
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from tianji_controller import hardware as hw
from wuji_controller import hardware as hand_hw
from wuji_controller import _wuji_abi as abi


class Clock:
    def __init__(self):
        self.ns = 1_000_000_000
        self.on_sleep = lambda: None

    def monotonic_ns(self):
        return self.ns

    def sleep(self, seconds):
        self.ns += max(1, int(seconds * 1e9))
        self.on_sleep()


class FakeMarvin:
    def __init__(self):
        self.calls = []
        self.serials = [1, 1]
        self.advance = True
        self.state = [0, 0]
        self.error = 0
        self.servo_report = 'None'
        self.servo_queries = []
        self.ratios = [10, 10]
        self.clear_results = []
        self.fail_state_b = False
        self.local_logging = True
        self.transition_polls = 0
        self.pending_states = {}

    def local_log_switch(self, flag):
        self.local_logging = flag != '0'

    def connect(self, ip):
        self.calls.append(('connect', ip))
        return 1

    def subscribe(self, unused):
        for index, (target, remaining) in tuple(self.pending_states.items()):
            if remaining <= 0:
                self.state[index] = target
                del self.pending_states[index]
            else:
                self.pending_states[index] = (target, remaining - 1)
        if self.advance:
            self.serials = [x + 1 for x in self.serials]
        return dict(states=[dict(cur_state=s, cmd_state=self.pending_states.get(i, (-1,))[0],
                                 err_code=self.error) for i, s in enumerate(self.state)],
                    outputs=[dict(frame_serial=s, fb_joint_pos=[90.0] * 7) for s in self.serials],
                    inputs=[dict(joint_vel_ratio=self.ratios[0], joint_acc_ratio=self.ratios[1])] * 2)

    def get_servo_error_code(self, arm, lang):
        self.servo_queries.append(arm)
        return self.servo_report

    def clear_set(self):
        self.calls.append(('clear',))
        return self.clear_results.pop(0) if self.clear_results else 1

    def set_vel_acc(self, arm, vel, acc):
        self.calls.append(('ratio', arm, vel, acc))
        self.ratios = [vel, acc]
        return 1

    def set_joint_cmd_pose(self, arm, q):
        self.calls.append(('pose', arm, tuple(q)))
        return 1

    def set_state(self, arm, state):
        self.calls.append(('state', arm, state))
        if self.fail_state_b and arm == 'B' and state == 1:
            return 0
        self.state[0 if arm == 'A' else 1] = state
        if self.transition_polls:
            index = 0 if arm == 'A' else 1
            self.state[index] = 101 if state == 1 else 109
            self.pending_states[index] = (state, self.transition_polls)
        return 1

    def send_cmd(self):
        self.calls.append(('send',))
        return 1

    def soft_stop(self, arm):
        self.calls.append(('stop', arm))

    def release_robot(self):
        self.calls.append(('release',))
        return 1


NIDS = [finger * 5 + motor for finger in range(5) for motor in range(1, 5)]


class FakeWuji:
    def __init__(self, side=1, handle=10):
        self.calls = []
        self.side = side
        self.handle = handle
        self.enabled = False
        self.fail_enable = False
        self.callbacks = {}
        self.seq = 0
        self.emit_states = True
        self.entries = None
        self.diagnostic_status = None
        self.diagnostic_fault = 0
        self.other_sdk_connected = False
        self.mit_kp, self.mit_kd = 12.0, 0.3
        self.mit_writes = []

    def wuji_init(self, opts):
        self.calls.append('init')
        return 0

    def wuji_shutdown(self):
        self.calls.append('shutdown')

    def wuji_last_error(self):
        return b'fake error'

    def wuji_connect_options_default(self):
        options = abi.ConnectOptions()
        options.enable_bridge = True
        return options

    def wuji_connect(self, target, name, options, out):
        self.calls.append('connect')
        if self.other_sdk_connected and not options._obj.enable_bridge:
            return -4
        out._obj.value = self.handle
        return 0

    def wuji_hand_2_get_handedness(self, dev, out):
        out._obj.value = self.side
        return 0

    @staticmethod
    def wuji_hand_2_describe_error(code, out):
        catalog = {3: (b'Enc1BitRate', b'Warning'),
                   4: (b'TestImmediateStop', b'ImmediateStop'),
                   5: (b'TestDeferredStop', b'DeferredStop'),
                   6: (b'TestFatal', b'Fatal'),
                   7: (b'TestUnknownSeverity', b'Unexpected')}
        if code not in catalog:
            return -4
        out._obj.code = code
        out._obj.name, out._obj.severity = catalog[code]
        out._obj.clear_policy = b'AutoClear'
        out._obj.desc = b'Encoder data quality warning'
        out._obj.cause = b'Encoder reports suspicious frames'
        out._obj.resolution = b'Inspect encoder installation'
        return 0

    def wuji_hand_2_online_joints_count(self, dev, out):
        out._obj.value = 20
        return 0

    def wuji_hand_2_subscribe_joint_states(self, dev, cb, user, out):
        self.callbacks[self.handle + 1] = cb
        out._obj.value = self.handle + 1
        return 0

    def wuji_hand_2_subscribe_joint_diagnostics(self, dev, cb, user, out):
        self.callbacks[self.handle + 2] = cb
        out._obj.value = self.handle + 2
        return 0

    def emit(self):
        self.seq += 1
        header = abi.FrameHeader(self.seq, self.seq * 1000, b'left' if self.side == 0 else b'right')
        if self.handle + 1 in self.callbacks and self.emit_states:
            entries = self.entries or (abi.JointStateEntry * 20)(*[abi.JointStateEntry(n, i / 10, 0, 0) for i, n in enumerate(NIDS)])
            frame = abi.JointStateFrame(header, len(entries), entries, len(entries))
            self.callbacks[self.handle + 1](0, ct.pointer(frame), None)
        if self.handle + 2 in self.callbacks:
            entries = (abi.JointDiagnosticsEntry * 20)()
            for e, n in zip(entries, NIDS):
                e.nid, e.status_word = n, 0x102 if self.enabled else 0x101
                if self.diagnostic_status is not None:
                    e.status_word = self.diagnostic_status
                e.error_code_current = self.diagnostic_fault
            frame = abi.JointDiagnosticsFrame(header, 20, entries, 20, abi.Hand2CommSummary())
            self.callbacks[self.handle + 2](0, ct.pointer(frame), None)

    def wuji_hand_2_set_all_effort_limit(self, dev, limit):
        self.calls.append('effort')
        return 0

    def wuji_hand_2_set_all_mit_params(self, dev, kp, kd):
        self.calls.append('mit')
        self.mit_writes.append((kp[0], kd[0]))
        return 0

    def wuji_hand_2_get_all_mit_params(self, dev, kp, kd, online):
        self.calls.append('read_mit')
        for index in range(20):
            kp[index], kd[index] = self.mit_kp, self.mit_kd
        online._obj.value = (1 << 20) - 1
        return 0

    def wuji_hand_2_enable(self, dev, mask):
        self.calls.append('enable')
        self.enabled = True
        return -4 if self.fail_enable else 0

    def wuji_hand_2_disable(self, dev, mask):
        self.calls.append('disable')
        self.enabled = False
        return 0

    def wuji_hand_2_joint_command_publish(self, dev, out):
        self.calls.append('publisher')
        out._obj.value = self.handle + 3
        return 0

    def wuji_joint_command_publisher_send(self, pub, commands):
        self.calls.append(('command', [(x.position, x.velocity, x.effort) for x in commands]))
        return 0

    def wuji_joint_command_publisher_close(self, pub):
        self.calls.append('publisher_close')

    def wuji_sub_close(self, sub):
        self.callbacks.pop(sub.value)(2, None, None)
        self.calls.append('sub_close')

    def wuji_dev_disconnect(self, dev):
        self.calls.append('disconnect')
        return 0

    def wuji_dev_release(self, dev):
        self.calls.append('release')


class FakeDualWuji:
    """One SDK runtime routes distinct native handles to two fake devices."""

    def __init__(self):
        self.devices = {'LEFT-SERIAL': FakeWuji(0, 10), 'RIGHT-SERIAL': FakeWuji(1, 20)}
        self.calls = []
        self.aliases = set()
        self.running = False

    def wuji_init(self, opts):
        self.calls.append('init')
        self.running = True
        return 0

    def wuji_shutdown(self):
        self.calls.append('shutdown')
        self.running = False
        for device in self.devices.values():
            device.enabled = False
            device.callbacks.clear()

    def wuji_last_error(self):
        return b'fake dual-device error'

    def wuji_connect_options_default(self):
        return abi.ConnectOptions(enable_bridge=True)

    def wuji_connect(self, target, alias, options, out):
        if not self.running or alias in self.aliases:
            return -4
        device = self.devices[target._obj.value.decode()]
        result = device.wuji_connect(target, alias, options, out)
        if result == 0:
            self.aliases.add(alias)
        return result

    def wuji_hand_2_describe_error(self, code, out):
        return FakeWuji.wuji_hand_2_describe_error(code, out)

    def __getattr__(self, name):
        if not name.startswith('wuji_'):
            raise AttributeError(name)

        def dispatch(handle, *args):
            if not self.running:
                raise RuntimeError('SDK shut down with a live hand')
            device = next(device for device in self.devices.values()
                          if device.handle <= handle.value <= device.handle + 3)
            return getattr(device, name)(handle, *args)

        return dispatch

    def emit(self):
        if self.running:
            for device in self.devices.values():
                device.emit()


class HardwareTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.patches = [patch.object(hw.time, 'monotonic_ns', self.clock.monotonic_ns),
                        patch.object(hw.time, 'sleep', self.clock.sleep)]
        for p in self.patches:
            p.start()
            self.addCleanup(p.stop)

    def arm(self):
        fake = FakeMarvin()
        p = patch.object(hw, '_load_marvin', return_value=(fake, object()))
        p.start()
        self.addCleanup(p.stop)
        return hw.MarvinDevice(Path('/unused'), '127.0.0.1'), fake

    def hand(self, side='right'):
        fake = FakeWuji(side=0 if side == 'left' else 1)
        self.clock.on_sleep = fake.emit
        p = patch.object(abi, 'load_sdk', return_value=fake)
        p.start()
        self.addCleanup(p.stop)
        device = hand_hw.Hand2Device(Path('/unused'), f'{side.upper()}-SERIAL', side)
        self.addCleanup(device.close)
        return device, fake

    def hands(self):
        fake = FakeDualWuji()
        self.clock.on_sleep = fake.emit
        p = patch.object(abi, 'load_sdk', return_value=fake)
        p.start()
        self.addCleanup(p.stop)
        left = hand_hw.Hand2Device(Path('/unused'), 'LEFT-SERIAL', 'left')
        right = hand_hw.Hand2Device(Path('/unused/../unused'), 'RIGHT-SERIAL', 'right')
        self.addCleanup(left.close)
        self.addCleanup(right.close)
        return left, right, fake

    def test_arm_local_sdk_output_is_silent_without_hiding_faults(self):
        device, fake = self.arm()
        original_query = fake.get_servo_error_code

        def noisy_query(arm, lang):
            if fake.local_logging:
                print("[Marvin SDK]: Get int parameter: SERVO0ERR0, value=0")
                print(f"[Marvin SDK]: {arm} arm Servo error code=[0,0,0,0,0,0,0],")
            return original_query(arm, lang)

        output = io.StringIO()
        with patch.object(fake, 'get_servo_error_code', side_effect=noisy_query), redirect_stdout(output):
            device.connect()
            self.clock.sleep(1.1)
            self.assertTrue(device.read_feedback().healthy)
            fake.error = 2
            fake.servo_report = 'E7'
            fault = device.read_feedback()
            self.assertFalse(fault.healthy)
            self.assertIn('E7', fault.detail)
            device.close()
        self.assertEqual(output.getvalue(), '')

    def test_construction_never_loads_sdk(self):
        with patch.object(hw, '_load_marvin', side_effect=AssertionError), patch.object(abi, 'load_sdk', side_effect=AssertionError):
            hw.MarvinDevice(Path('/unused'), '127.0.0.1').close()
            for side in ('left', 'right'):
                hand_hw.Hand2Device(Path('/unused'), f'{side.upper()}-SERIAL', side).close()

    def test_hand_rejects_invalid_identity_without_loading_sdk(self):
        with patch.object(abi, 'load_sdk', side_effect=AssertionError):
            for side in ('', 'LEFT', 'both', None, 0):
                with self.subTest(side=side), self.assertRaises(ValueError):
                    hand_hw.Hand2Device(Path('/unused'), 'LEFT-SERIAL', side)
            for serial in ('', ' ', ' LEFT-SERIAL', 'LEFT-SERIAL ', 'LEFT\nSERIAL', 'LEFT\0SERIAL', None):
                with self.subTest(serial=serial), self.assertRaises(ValueError):
                    hand_hw.Hand2Device(Path('/unused'), serial, 'left')

    def test_both_hands_keep_firmware_order_and_direction_through_commands(self):
        left, right, sdk = self.hands()
        for serial, offset in (('LEFT-SERIAL', -4), ('RIGHT-SERIAL', 8)):
            sdk.devices[serial].entries = (abi.JointStateEntry * 20)(
                *[abi.JointStateEntry(n, offset + NIDS.index(n) / 4, 0, 0) for n in reversed(NIDS)])
        left.connect()
        right.connect()
        left.enable()
        right.enable()
        for device, serial, offset in ((left, 'LEFT-SERIAL', -4), (right, 'RIGHT-SERIAL', 8)):
            expected = tuple(offset + i / 4 for i in range(20))
            feedback = device.read_feedback()
            self.assertTrue(feedback.healthy)
            self.assertTrue(feedback.enabled)
            self.assertEqual(feedback.position_rad, expected)
            device.send(expected)
            self.assertEqual(sdk.devices[serial].calls[-1], ('command', [(q, 0, 0) for q in expected]))
        self.assertEqual(sdk.calls, ['init'])
        left.stop()
        left.close()
        self.assertTrue(right.read_feedback().healthy)
        right.send((.5,) * 20)
        self.assertTrue(sdk.devices['RIGHT-SERIAL'].enabled)
        self.assertEqual(sdk.calls, ['init'])
        right.close()
        right.close()
        self.assertEqual(sdk.calls, ['init', 'shutdown'])

    def test_swapped_hand_roles_are_rejected_for_both_sides(self):
        for side in ('left', 'right'):
            with self.subTest(side=side):
                device, sdk = self.hand(side)
                sdk.side = 1 if side == 'left' else 0
                with self.assertRaises(RuntimeError):
                    device.connect()
                self.assertNotIn('enable', sdk.calls)
                self.assertNotIn('disable', sdk.calls)
                self.assertIn('release', sdk.calls)
                self.assertEqual(sdk.calls.count('shutdown'), 1)

    def test_hand_rechecks_firmware_role_before_enable(self):
        for side in ('left', 'right'):
            for boundary in ('before_configuration', 'during_configuration'):
                with self.subTest(side=side, boundary=boundary):
                    device, sdk = self.hand(side)
                    device.connect()

                    def swap_side(*args):
                        sdk.side = 1 if side == 'left' else 0
                        return 0

                    if boundary == 'before_configuration':
                        swap_side()
                    else:
                        sdk.wuji_hand_2_set_all_mit_params = swap_side
                    with self.assertRaises(RuntimeError):
                        device.enable()
                    self.assertNotIn('enable', sdk.calls)
                    self.assertNotIn('disable', sdk.calls)
                    self.assertIn('release', sdk.calls)

    def test_failed_peer_never_shuts_down_or_disables_active_hand(self):
        for active_side in ('left', 'right'):
            for failure in ('wrong_side', 'connect', 'partial_enable'):
                with self.subTest(active_side=active_side, failure=failure):
                    left, right, sdk = self.hands()
                    active, peer = (left, right) if active_side == 'left' else (right, left)
                    active_sdk = sdk.devices[active.serial]
                    peer_sdk = sdk.devices[peer.serial]
                    active.connect()
                    active.enable()
                    if failure == 'wrong_side':
                        peer_sdk.side = active_sdk.side
                    elif failure == 'connect':
                        peer_sdk.other_sdk_connected = True
                    else:
                        peer_sdk.fail_enable = True
                    with self.assertRaises(RuntimeError):
                        peer.connect()
                        peer.enable()
                    sdk.emit()
                    active.send((.25,) * 20)
                    self.assertTrue(active.read_feedback().healthy)
                    self.assertTrue(active_sdk.enabled)
                    self.assertNotIn('disable', active_sdk.calls)
                    self.assertNotIn('disconnect', active_sdk.calls)
                    self.assertEqual(sdk.calls, ['init'])
                    active.close()
                    self.assertEqual(sdk.calls, ['init', 'shutdown'])

    def test_hand_fault_and_freshness_are_side_local(self):
        left, right, sdk = self.hands()
        left.connect()
        right.connect()
        left.enable()
        right.enable()
        left_sdk, right_sdk = sdk.devices['LEFT-SERIAL'], sdk.devices['RIGHT-SERIAL']
        left_sdk.diagnostic_fault = 4
        sdk.emit()
        left_sdk.diagnostic_fault = 0
        sdk.emit()
        self.assertFalse(left.read_feedback().healthy)
        with self.assertRaises(RuntimeError):
            left.send((0,) * 20)
        right.send((.25,) * 20)
        right_stamp = right.read_feedback().received_monotonic_ns
        self.clock.on_sleep = left_sdk.emit
        self.clock.sleep(.2)
        self.assertEqual(right.read_feedback().received_monotonic_ns, right_stamp)
        self.assertTrue(right.read_feedback().healthy)
        self.clock.sleep(.11)
        self.assertFalse(right.read_feedback().healthy)
        right_sdk.emit()
        right.send((.5,) * 20)
        self.assertFalse(left.read_feedback().healthy)

    def test_duplicate_serial_cannot_claim_or_disconnect_owned_device(self):
        left, right, sdk = self.hands()
        left.connect()
        left.enable()
        duplicate = hand_hw.Hand2Device(Path('/unused'), 'LEFT-SERIAL', 'right')
        self.addCleanup(duplicate.close)
        with self.assertRaises(RuntimeError):
            duplicate.connect()
        left.send((.25,) * 20)
        self.assertEqual(sdk.devices['LEFT-SERIAL'].calls.count('connect'), 1)
        self.assertNotIn('disable', sdk.devices['LEFT-SERIAL'].calls)
        self.assertNotIn('disconnect', sdk.devices['LEFT-SERIAL'].calls)
        self.assertEqual(sdk.calls, ['init'])

    def test_sdk_lifetimes_are_scoped_to_loaded_library_path(self):
        first, second = FakeWuji(0), FakeWuji(1)
        left = hand_hw.Hand2Device(Path('/first-sdk'), 'LEFT-SERIAL', 'left')
        right = hand_hw.Hand2Device(Path('/second-sdk'), 'RIGHT-SERIAL', 'right')
        self.addCleanup(left.close)
        self.addCleanup(right.close)
        self.clock.on_sleep = lambda: (first.emit(), second.emit())
        with patch.object(abi, 'load_sdk', side_effect=[first, second]):
            left.connect()
            right.connect()
            left.close()
            self.assertEqual(first.calls.count('shutdown'), 1)
            self.assertNotIn('shutdown', second.calls)
            right.enable()
            right.send((.25,) * 20)
            right.close()
            self.assertEqual(second.calls.count('shutdown'), 1)

    def test_arm_read_only_connection_and_unit_conversion(self):
        device, fake = self.arm()
        device.connect()
        self.assertEqual(device.read_feedback().position_rad, (math.pi / 2,) * 14)
        with self.assertRaises(RuntimeError):
            device.send((0,) * 14)
        device.close()
        self.assertEqual(fake.calls, [('connect', '127.0.0.1'), ('release',)])

    def test_arm_enable_at_measured_pose_then_radian_commands(self):
        device, fake = self.arm()
        device.connect()
        device.enable()
        poses = [c for c in fake.calls if c[0] == 'pose']
        self.assertEqual(poses, [('pose', 'A', (90.0,) * 7), ('pose', 'B', (90.0,) * 7)])
        device.send((math.pi,) * 14)
        self.assertEqual([c for c in fake.calls if c[0] == 'pose'][-2:], [('pose', 'A', (180.0,) * 7), ('pose', 'B', (180.0,) * 7)])
        device.close()

    def test_arm_guard_allows_requested_transition_but_waits_for_enabled(self):
        device, fake = self.arm()
        device.connect()
        fake.transition_polls = 20
        observed_transition = []

        def guard():
            feedback = device.read_feedback()
            if 101 in fake.state:
                observed_transition.append(True)
                self.assertFalse(feedback.enabled)
                with self.assertRaises(RuntimeError):
                    device.send((0,) * 14)
            if not feedback.healthy:
                raise RuntimeError(feedback.detail)

        device.enable(guard=guard)
        self.assertTrue(observed_transition)
        self.assertTrue(device.read_feedback().enabled)
        device.send((math.pi / 2,) * 14)
        device.close()
        self.assertEqual(fake.state, [0, 0])

    def test_arm_stuck_enable_transition_times_out_and_attempts_disarm(self):
        device, fake = self.arm()
        device.connect()
        fake.transition_polls = 1_000_000
        started = self.clock.ns

        def guard():
            feedback = device.read_feedback()
            if not feedback.healthy:
                raise RuntimeError(feedback.detail)

        with self.assertRaises(RuntimeError):
            device.enable(guard=guard)
        self.assertLessEqual(self.clock.ns - started, 3 * hw._TIMEOUT_NS)
        self.assertIn(('stop', 'AB'), fake.calls)
        self.assertIn(('state', 'A', 0), fake.calls)
        self.assertIn(('state', 'B', 0), fake.calls)
        with self.assertRaises(RuntimeError):
            device.send((0,) * 14)

    def test_arm_enable_transition_does_not_hide_feedback_faults(self):
        for failure in ('stale', 'error', 'servo'):
            with self.subTest(failure=failure):
                device, fake = self.arm()
                device.connect()
                fake.transition_polls = 1_000_000
                started = self.clock.ns
                rejected_at = []

                def inject_fault():
                    if failure == 'stale':
                        fake.advance = False
                    elif failure == 'error':
                        fake.error = 2
                    else:
                        fake.servo_report = 'servo fault'

                self.clock.on_sleep = inject_fault

                def guard():
                    feedback = device.read_feedback()
                    if not feedback.healthy:
                        rejected_at.append(self.clock.ns)
                        raise RuntimeError(feedback.detail)

                with self.assertRaises(RuntimeError):
                    device.enable(guard=guard)
                self.assertIn(('stop', 'AB'), fake.calls)
                self.assertLess(rejected_at[0] - started, hw._TIMEOUT_NS)
                self.assertEqual(fake.calls[-1], ('release',))
                self.clock.on_sleep = lambda: None

    def test_arm_transition_states_are_not_healthy_outside_own_enable(self):
        device, fake = self.arm()
        device.connect()
        for state in (101, 109):
            fake.state = [state, state]
            feedback = device.read_feedback()
            self.assertFalse(feedback.healthy)
            self.assertFalse(feedback.enabled)
        device.close()

    def test_arm_enabled_takeover_requires_explicit_authorization(self):
        device, fake = self.arm()
        fake.state = [1, 1]
        device.connect()
        with self.assertRaises(RuntimeError):
            device.enable()
        self.assertEqual(fake.state, [1, 1])
        self.assertEqual(fake.calls, [('connect', '127.0.0.1'), ('release',)])

    def test_arm_home_takeover_holds_measured_pose_and_disables_on_close(self):
        device, fake = self.arm()
        fake.state = [1, 1]
        fake.ratios = [50, 50]
        device.connect()
        device.enable(allow_enabled=True)
        first_send = fake.calls.index(('send',))
        held = [call for call in fake.calls[:first_send] if call[0] == 'pose']
        self.assertEqual(held, [('pose', 'A', (90.0,) * 7), ('pose', 'B', (90.0,) * 7)])
        self.assertEqual(fake.ratios, [10, 10])
        self.assertEqual(fake.state, [1, 1])
        self.assertFalse(any(call[0] == 'state' for call in fake.calls))
        device.send((math.pi / 3,) * 14)
        device.close()
        self.assertEqual(fake.state, [0, 0])
        self.assertIn(('stop', 'AB'), fake.calls)
        self.assertEqual(fake.calls[-1], ('release',))

    def test_arm_failed_authorized_takeover_still_stops_and_disables(self):
        device, fake = self.arm()
        fake.state = [1, 1]
        device.connect()
        fake.clear_results = [0, 0, 0]
        with self.assertRaises(RuntimeError):
            device.enable(allow_enabled=True)
        self.assertIn(('stop', 'AB'), fake.calls)
        self.assertEqual(fake.state, [0, 0])
        self.assertEqual(fake.calls[-1], ('release',))

    def test_arm_home_takeover_rejects_mixed_enable_states_without_writes(self):
        device, fake = self.arm()
        fake.state = [1, 0]
        device.connect()
        with self.assertRaises(RuntimeError):
            device.enable(allow_enabled=True)
        self.assertEqual(fake.state, [1, 0])
        self.assertEqual(fake.calls, [('connect', '127.0.0.1'), ('release',)])

    def test_arm_duplicate_or_backward_serial_does_not_refresh(self):
        device, fake = self.arm()
        device.connect()
        original = device.read_feedback().received_monotonic_ns
        fake.advance = False
        self.clock.sleep(.2)
        self.assertEqual(device.read_feedback().received_monotonic_ns, original)
        self.assertTrue(device.read_feedback().healthy)
        self.clock.sleep(.11)
        self.assertFalse(device.read_feedback().healthy)
        fake.serials[0] -= 1
        self.assertFalse(device.read_feedback().healthy)
        device.close()

    def test_arm_servo_detail_queries_are_rate_limited(self):
        device, fake = self.arm()
        device.connect()
        self.assertEqual(fake.servo_queries, ['A', 'B'])
        for _ in range(50):
            self.assertTrue(device.read_feedback().healthy)
        self.assertEqual(fake.servo_queries, ['A', 'B'])
        self.clock.sleep(hw._SERVO_DETAIL_INTERVAL_NS / 1e9)
        self.assertTrue(device.read_feedback().healthy)
        self.assertEqual(fake.servo_queries, ['A', 'B', 'A', 'B'])
        device.close()

    def test_arm_payload_fault_forces_immediate_servo_query(self):
        device, fake = self.arm()
        device.connect()
        self.assertTrue(device.read_feedback().healthy)
        self.assertEqual(fake.servo_queries, ['A', 'B'])
        fake.error = 2
        self.assertFalse(device.read_feedback().healthy)
        self.assertEqual(fake.servo_queries, ['A', 'B', 'A', 'B'])
        device.close()

    def test_arm_servo_report_fault_is_never_hidden_by_cache(self):
        device, fake = self.arm()
        device.connect()
        self.assertTrue(device.read_feedback().healthy)
        fake.servo_report = 'E7'
        # Within the interval the cached good report is reused, so a fault that
        # appears later is only observed when the interval elapses.
        self.assertTrue(device.read_feedback().healthy)
        self.clock.sleep(hw._SERVO_DETAIL_INTERVAL_NS / 1e9)
        self.assertFalse(device.read_feedback().healthy)
        self.assertIn("'E7'", device.read_feedback().detail)
        self.assertEqual(fake.servo_queries, ['A', 'B', 'A', 'B'])
        device.close()

    def test_arm_clear_retries_before_writes_only(self):
        device, fake = self.arm()
        device.connect()
        device.enable()
        fake.calls.clear()
        fake.clear_results = [0, 0, 1]
        device.send((0,) * 14)
        self.assertEqual(fake.calls[:3], [('clear',)] * 3)
        self.assertEqual(fake.calls[3][0], 'pose')
        fake.clear_results = [0, 0, 0]
        fake.calls.clear()
        with self.assertRaises(RuntimeError):
            device.send((0,) * 14)
        self.assertNotIn('pose', [c[0] for c in fake.calls])
        device.close()

    def test_arm_fault_connect_does_not_clear_or_disable(self):
        device, fake = self.arm()
        fake.error = 2
        with self.assertRaises(RuntimeError):
            device.connect()
        self.assertEqual(fake.calls, [('connect', '127.0.0.1'), ('release',)])

    def test_arm_partial_enable_cleans_both_arms(self):
        device, fake = self.arm()
        device.connect()
        fake.fail_state_b = True
        with self.assertRaises(RuntimeError):
            device.enable()
        self.assertIn(('stop', 'AB'), fake.calls)
        self.assertIn(('state', 'A', 0), fake.calls)
        self.assertIn(('state', 'B', 0), fake.calls)
        self.assertIn(('release',), fake.calls)

    def test_wrong_hand_is_released_without_enabling_or_disabling(self):
        device, fake = self.hand()
        fake.side = 0
        with self.assertRaises(RuntimeError):
            device.connect()
        self.assertNotIn('enable', fake.calls)
        self.assertNotIn('disable', fake.calls)
        self.assertIn('release', fake.calls)

    def test_hand_no_pre_enable_state_fails_closed(self):
        device, fake = self.hand()
        fake.emit_states = False
        with self.assertRaises(RuntimeError):
            device.connect()
        self.assertNotIn('enable', fake.calls)
        self.assertNotIn('disable', fake.calls)
        self.assertFalse(fake.callbacks)

    def test_hand_refuses_shared_sdk_motor_authority_without_disabling_owner(self):
        device, fake = self.hand()
        fake.other_sdk_connected = True
        fake.enabled = True
        with self.assertRaises(RuntimeError):
            device.connect()
        self.assertTrue(fake.enabled)
        self.assertNotIn('enable', fake.calls)
        self.assertNotIn('disable', fake.calls)
        self.assertNotIn('disconnect', fake.calls)

    def test_hand_read_only_close_preserves_external_session(self):
        device, fake = self.hand()
        device.connect()
        self.assertFalse(device.read_feedback().enabled)
        device.close()
        device.close()
        self.assertNotIn('enable', fake.calls)
        self.assertNotIn('disable', fake.calls)
        self.assertFalse(fake.callbacks)

    def test_hand_reports_and_reuses_device_mit_gains(self):
        device, fake = self.hand()
        self.assertIsNone(device.kp)
        device.connect()
        self.assertTrue(all(abs(value - fake.mit_kp) < 1e-6 for value in device.device_kp))
        self.assertTrue(all(abs(value - fake.mit_kd) < 1e-6 for value in device.device_kd))
        device.enable()
        # The device values are written back unchanged, never replaced by the
        # vendor example gains.
        self.assertEqual(fake.mit_writes, [(device.device_kp[0], device.device_kd[0])])
        device.close()

    def test_hand_configured_gains_override_device_gains(self):
        fake = FakeWuji(side=0)
        self.clock.on_sleep = fake.emit
        p = patch.object(abi, 'load_sdk', return_value=fake)
        p.start()
        self.addCleanup(p.stop)
        device = hand_hw.Hand2Device(Path('/unused'), 'LEFT-SERIAL', 'left', kp=4.5, kd=.25)
        self.addCleanup(device.close)
        device.connect()
        device.enable()
        self.assertTrue(all(abs(value - fake.mit_kp) < 1e-6 for value in device.device_kp))
        self.assertTrue(all(abs(value - fake.mit_kd) < 1e-6 for value in device.device_kd))
        self.assertEqual(fake.mit_writes, [(4.5, .25)])
        device.close()

    def test_hand_enable_waits_for_diagnostics_before_commands(self):
        device, fake = self.hand()
        device.connect()
        with self.assertRaises(RuntimeError):
            device.send((0,) * 20)
        device.enable()
        self.assertEqual([x for x in fake.calls if isinstance(x, tuple)], [])
        self.assertTrue(device.read_feedback().enabled)
        device.send((.25,) * 20)
        self.assertEqual(fake.calls[-1], ('command', [(.25, 0, 0)] * 20))
        device.close()
        self.assertIn('disable', fake.calls)

    def test_hand_partial_enable_failure_disables_and_releases(self):
        device, fake = self.hand()
        device.connect()
        fake.fail_enable = True
        with self.assertRaises(RuntimeError):
            device.enable()
        self.assertIn('disable', fake.calls)
        self.assertIn('release', fake.calls)
        self.assertFalse(fake.callbacks)

    def test_hand_partial_duplicate_nonfinite_states_fail_closed(self):
        for invalid in ('partial', 'duplicate', 'nonfinite'):
            with self.subTest(invalid=invalid):
                device, fake = self.hand()
                device.connect()
                before = device.read_feedback().received_monotonic_ns
                entries = (abi.JointStateEntry * (19 if invalid == 'partial' else 20))()
                for i, e in enumerate(entries):
                    e.nid, e.position = NIDS[i], i / 10
                if invalid == 'duplicate':
                    entries[-1].nid = entries[0].nid
                if invalid == 'nonfinite':
                    entries[0].effort = math.nan
                fake.entries = entries
                self.clock.sleep(.01)
                feedback = device.read_feedback()
                self.assertFalse(feedback.healthy)
                self.assertEqual(feedback.received_monotonic_ns, before)
                device.close()

    def test_hand_state_order_and_stale_cache(self):
        device, fake = self.hand()
        fake.entries = (abi.JointStateEntry * 20)(*[abi.JointStateEntry(n, NIDS.index(n), 0, 0) for n in reversed(NIDS)])
        device.connect()
        before = device.read_feedback()
        self.assertEqual(before.position_rad, tuple(range(20)))
        self.clock.on_sleep = lambda: None
        self.clock.sleep(.2)
        self.assertTrue(device.read_feedback().healthy)
        self.clock.sleep(.11)
        after = device.read_feedback()
        self.assertEqual(after.received_monotonic_ns, before.received_monotonic_ns)
        self.assertFalse(after.healthy)
        device.close()

    def test_warning_is_visible_without_stopping_and_clears(self):
        device, fake = self.hand()
        device.connect()
        device.enable()
        fake.diagnostic_fault = 3
        # The hand adapter owns hand diagnostics, so its logger is the right one.
        with self.assertLogs('wuji_controller.hardware', level='WARNING') as logs:
            fake.emit()
            fake.emit()
        status = device.read_feedback()
        self.assertTrue(status.healthy)
        self.assertTrue(status.enabled)
        self.assertIn('RIGHT-SERIAL', status.detail)
        self.assertIn('right', status.detail)
        self.assertIn('Enc1BitRate', status.detail)
        self.assertEqual(len(logs.records), 20)
        device.send((.25,) * 20)
        self.assertEqual(fake.calls[-1], ('command', [(.25, 0, 0)] * 20))
        fake.diagnostic_fault = 0
        fake.emit()
        self.assertEqual(device.read_feedback().detail, '')
        self.assertTrue(device.read_feedback().healthy)
        device.close()

    def test_stop_and_unknown_severities_remain_latched(self):
        for code in (4, 5, 6, 7, 65535):
            with self.subTest(code=code):
                device, fake = self.hand()
                try:
                    device.connect()
                    device.enable()
                    fake.diagnostic_fault = code
                    fake.emit()
                    self.assertFalse(device.read_feedback().healthy)
                    self.assertIn('RIGHT-SERIAL', device.read_feedback().detail)
                    fake.diagnostic_fault = 0
                    fake.emit()
                    with self.assertRaises(RuntimeError):
                        device.send((0,) * 20)
                finally:
                    device.close()

    def test_warning_does_not_allow_disabled_motor_state(self):
        device, fake = self.hand()
        device.connect()
        device.enable()
        fake.diagnostic_fault = 3
        fake.diagnostic_status = 0x101
        fake.emit()
        self.assertFalse(device.read_feedback().healthy)
        with self.assertRaises(RuntimeError):
            device.send((0,) * 20)
        device.close()

    def test_owned_hand_fault_cannot_be_hidden_by_next_healthy_frame(self):
        for event in ('partial', 'error', 'stopped', 'disabled', 'stream_end'):
            with self.subTest(event=event):
                device, fake = self.hand()
                device.connect()
                device.enable()
                if event == 'partial':
                    fake.entries = (abi.JointStateEntry * 19)()
                elif event == 'error':
                    fake.diagnostic_fault = 4
                elif event == 'stopped':
                    fake.diagnostic_status = 0x103
                elif event == 'disabled':
                    fake.diagnostic_status = 0x101
                else:
                    fake.callbacks[11](2, None, None)
                fake.emit()
                fake.entries = None
                fake.diagnostic_fault = 0
                fake.diagnostic_status = None
                fake.emit()
                self.assertFalse(device.read_feedback().healthy)
                with self.assertRaises(RuntimeError):
                    device.send((0,) * 20)
                device.close()

    def test_pre_enable_invalid_hand_frame_can_recover(self):
        device, fake = self.hand()
        device.connect()
        fake.callbacks[11](1, None, None)
        fake.emit()
        self.assertTrue(device.read_feedback().healthy)
        device.enable()
        self.assertTrue(device.read_feedback().enabled)
        device.close()

    def test_arm_guard_covers_each_enabling_state_write_and_send(self):
        for invalidate_after, expected_enabled_writes in (('pose', 0), ('state_A', 1), ('state_B', 2)):
            with self.subTest(boundary=invalidate_after):
                device, fake = self.arm()
                device.connect()
                authorized = True
                original_pose, original_state = fake.set_joint_cmd_pose, fake.set_state

                def pose(arm, q):
                    nonlocal authorized
                    result = original_pose(arm, q)
                    if invalidate_after == 'pose' and arm == 'B':
                        authorized = False
                    return result

                def state(arm, value):
                    nonlocal authorized
                    result = original_state(arm, value)
                    if value == 1 and invalidate_after == 'state_' + arm:
                        authorized = False
                    return result

                fake.set_joint_cmd_pose, fake.set_state = pose, state
                with self.assertRaises(RuntimeError):
                    device.enable(guard=lambda: authorized)
                self.assertEqual(len([call for call in fake.calls if call[0] == 'state' and call[2] == 1]),
                                 expected_enabled_writes)
                # Ratios are sent; an attempted enable is only followed by cleanup.
                self.assertEqual(fake.calls.count(('send',)), 1 + bool(expected_enabled_writes))
                self.assertIn(('release',), fake.calls)

    def test_hand_guard_rejects_source_expiring_during_blocking_configuration(self):
        device, fake = self.hand()
        device.connect()
        authorized = True
        original = fake.wuji_hand_2_set_all_mit_params

        def configure(*args):
            nonlocal authorized
            result = original(*args)
            authorized = False
            return result

        fake.wuji_hand_2_set_all_mit_params = configure
        with self.assertRaises(RuntimeError):
            device.enable(guard=lambda: authorized)
        self.assertNotIn('enable', fake.calls)
        self.assertNotIn('disable', fake.calls)
        self.assertIn('release', fake.calls)

    def test_hand_guard_cancels_enable_wait_and_cleans_partial_enable(self):
        device, fake = self.hand()
        device.connect()
        authorized = True

        def source_expires():
            nonlocal authorized
            fake.emit()
            authorized = False

        self.clock.on_sleep = source_expires
        with self.assertRaises(RuntimeError):
            device.enable(guard=lambda: authorized)
        self.assertIn('enable', fake.calls)
        self.assertIn('disable', fake.calls)
        self.assertNotIn('publisher', fake.calls)
        self.assertIn('release', fake.calls)

    def test_arm_guard_cancels_feedback_wait_before_enabling(self):
        device, fake = self.arm()
        device.connect()
        fake.advance = False
        authorized = True

        def source_expires():
            nonlocal authorized
            authorized = False

        self.clock.on_sleep = source_expires
        with self.assertRaises(RuntimeError):
            device.enable(guard=lambda: authorized)
        self.assertFalse(any(call[0] == 'state' for call in fake.calls))
        self.assertIn(('release',), fake.calls)

    def test_stop_before_enable_latches_without_touching_motors(self):
        arm, arm_sdk = self.arm()
        hand, hand_sdk = self.hand()
        arm.connect()
        hand.connect()
        arm.stop()
        hand.stop()
        with self.assertRaises(RuntimeError):
            arm.enable()
        with self.assertRaises(RuntimeError):
            hand.enable()
        arm.close()
        hand.close()
        self.assertEqual(arm_sdk.calls, [('connect', '127.0.0.1'), ('release',)])
        self.assertNotIn('enable', hand_sdk.calls)
        self.assertNotIn('disable', hand_sdk.calls)


class HardwareAbiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temporary.cleanup)
        # The pinned Wuji SDK lives at the workspace vendor root.
        from tianji_runtime import vendor_path
        library = Path(cls.temporary.name) / 'hardware_abi.so'
        subprocess.run(['cc', '-shared', '-fPIC', '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-I', str(vendor_path('wuji-sdk', 'include')),
                        str(Path(__file__).parent / 'hardware_abi_fixture.c'), '-o', str(library)],
                       check=True, capture_output=True, text=True)
        cls.fixture = ct.CDLL(str(library))
        cls.fixture.hardware_abi_size.argtypes = [ct.c_uint]
        cls.fixture.hardware_abi_size.restype = ct.c_size_t
        cls.fixture.hardware_abi_emit.argtypes = [abi.StateCallback, abi.DiagnosticsCallback,
                                                 ct.c_uint32, ct.c_uint32, ct.c_uint16]
        cls.fixture.hardware_abi_emit.restype = None
        cls.fixture.hardware_abi_commands.argtypes = [ct.POINTER(abi.JointCommand)]
        cls.fixture.hardware_abi_commands.restype = ct.c_int

    def test_ctypes_sizes_match_pinned_c_header(self):
        types = (abi.FrameHeader, abi.JointStateEntry, abi.JointStateFrame,
                 abi.JointDiagnosticsEntry, abi.Hand2CommSummary, abi.JointDiagnosticsFrame,
                 abi.JointCommand, abi.ConnectTarget, abi.ConnectOptions, abi.ErrorInfo)
        for index, structure in enumerate(types):
            with self.subTest(structure=structure.__name__):
                self.assertEqual(ct.sizeof(structure), self.fixture.hardware_abi_size(index))

    def test_c_callback_decoding_and_fault_status(self):
        for side in ('left', 'right'):
            with self.subTest(side=side):
                device = hand_hw.Hand2Device(Path('/unused'), f'{side.upper()}-SERIAL', side)
                # Fake connected handles never reach an SDK: only C stack frames.
                device._dev.value, device._side_verified = 10, True
                states = abi.StateCallback(device._on_states)
                diagnostics = abi.DiagnosticsCallback(device._on_diagnostics)
                self.fixture.hardware_abi_emit(states, diagnostics, 1, 0x101, 0)
                self.fixture.hardware_abi_emit(states, diagnostics, 2, 0x102, 0)
                feedback = device.read_feedback()
                self.assertTrue(feedback.healthy)
                self.assertTrue(feedback.enabled)
                self.assertEqual(feedback.position_rad, tuple(i / 4 for i in range(20)))
                # Replayed SDK frames must not gain a new receipt timestamp.
                self.fixture.hardware_abi_emit(states, diagnostics, 2, 0x102, 0)
                self.assertFalse(device.read_feedback().healthy)
                self.assertEqual(device.read_feedback().received_monotonic_ns, feedback.received_monotonic_ns)
                self.fixture.hardware_abi_emit(states, diagnostics, 3, 0x102, 4)
                self.assertFalse(device.read_feedback().healthy)
                self.fixture.hardware_abi_emit(states, diagnostics, 4, 0x103, 0)
                self.assertFalse(device.read_feedback().healthy)

    def test_twenty_commands_are_float_aos_without_velocity_or_effort(self):
        commands = (abi.JointCommand * 20)()
        for i, command in enumerate(commands):
            command.position = i / 4
        self.assertEqual(self.fixture.hardware_abi_commands(commands), 1)
        commands[19].effort = .25
        self.assertEqual(self.fixture.hardware_abi_commands(commands), 0)


if __name__ == '__main__':
    unittest.main()
