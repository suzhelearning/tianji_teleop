"""Actual native dropout policy over synthetic loopback input; no SDK/devices."""
import math
import signal
import socket
import struct
import subprocess
import time
import unittest
import zlib
from test_port import BUNDLE, BUILD, packet
from real_robot.run_teleop import CommandReceiver
from real_robot.mapped_events import EventReceiver
from real_robot.protocol import ARMS_READY


class NativeDropoutTests(unittest.TestCase):
    def setUp(self):
        self.receiver = EventReceiver(CommandReceiver(0), bounded=True, dropout=True)
        self.source = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.source.bind(('127.0.0.1', 0))
        self.port = self.source.getsockname()[1]
        self.source.close()
        self.source = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sequence = 0
        self.sent_stamps = set()
        self.frames = []
        args = [str(BUILD/'mapped_palm_tjrc_controller'),
                '--config', str(BUNDLE/'config/deployment.yaml'),
                '--model', str(BUNDLE/'assets/mapped_palm/marvin_m6_wuji2.xml'),
                '--pico-port', str(self.port), '--joint-command-host', '127.0.0.1',
                '--joint-command-port', str(self.receiver.port), *self.receiver.arguments()]
        self.child = subprocess.Popen(args, pass_fds=(self.receiver.writer.fileno(),),
                                      stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.receiver.writer.close()
        self.addCleanup(self.close)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            self.pump(.01)
            if self.frames and self.frames[-1].event_state == 1:
                break
        self.assertEqual(self.frames[-1].event_state if self.frames else None, 1)
        self.pump(.05)
        self.assertTrue(all(f.event_state == 1 for f in self.frames[-5:]))

    def close(self):
        self.child.send_signal(signal.SIGCONT)
        self.child.terminate()
        _, errors = self.child.communicate(timeout=5)
        self.receiver.close()
        self.source.close()
        self.assertEqual(self.child.returncode, 0, errors.decode())

    def send(self, *, epoch=9, angle=0., button=False, bridge=None):
        self.sequence += 1
        stamp = time.monotonic_ns()
        raw = bytearray(packet(self.sequence, stamp, epoch))
        if bridge is not None:
            struct.pack_into('<q', raw, 32, bridge)
        self.sent_stamps.add(stamp if bridge is None else bridge)
        for point in range(8):
            struct.pack_into('<4d', raw, 396+point*32,
                             0, 0, math.sin(angle/2), math.cos(angle/2))
        if button:
            flags, = struct.unpack_from('<I', raw, 40)
            struct.pack_into('<I', raw, 40, flags | 256)
        struct.pack_into('<I', raw, 652, zlib.crc32(raw[:652]))
        self.source.sendto(raw, ('127.0.0.1', self.port))

    def pump(self, duration, *, send=True, **kwargs):
        first = len(self.frames)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if send:
                self.send(**kwargs)
            time.sleep(.003)
            self.receiver.drain(self.frames.append)
        return self.frames[first:]

    def hold(self):
        frames = self.pump(.11, send=False)
        held = [f for f in frames if f.event_state == 9]
        self.assertTrue(held, [(f.event_state, (f.timestamp_ns-f.input_valid_ns)/1e6) for f in frames])
        return held[-1]

    def test_absence_freezes_commands_and_recovers_without_timestamp_sliding(self):
        first = len(self.frames)
        held = self.hold()
        ready = next(f for f in reversed(self.frames) if f.event_state == 1)
        self.assertIn(held.input_valid_ns, self.sent_stamps)
        self.assertEqual(held.input_valid_ns, ready.input_valid_ns)
        for frame in (f for f in self.frames[first:] if f.event_state == 9):
            self.assertFalse(frame.flags & ARMS_READY)
            self.assertEqual(frame.left_arm+frame.right_arm, ready.left_arm+ready.right_arm)
            self.assertEqual((frame.tracking_epoch, frame.generation),
                             (ready.tracking_epoch, ready.generation))
            self.assertEqual(frame.input_valid_ns, held.input_valid_ns)
            self.assertGreaterEqual(frame.timestamp_ns-frame.input_valid_ns, 50_000_000)
            self.assertLessEqual(frame.timestamp_ns-frame.input_valid_ns, 300_000_000)
        resumed = self.pump(.06)
        self.assertTrue(any(f.event_state == 1 for f in resumed))
        self.assertTrue(all(f.event_state in (1, 9) for f in resumed))
        self.assertGreater(resumed[-1].input_valid_ns, held.input_valid_ns)
        self.assertEqual(resumed[-1].generation, held.generation)
        self.assertEqual(resumed[-1].calibration_state, 0)
        self.assertEqual(resumed[-1].calibration_offsets, (0., 0., 0., 0.))

    def test_expiry_latches_and_late_input_cannot_recover(self):
        held = self.hold()
        expired = self.pump(.25, send=False)
        self.assertEqual(expired[-1].event_state, 4)
        self.assertEqual(expired[-1].input_valid_ns, held.input_valid_ns)
        self.assertTrue(all(f.timestamp_ns-held.input_valid_ns <= 300_000_000
                            for f in expired if f.event_state == 9))
        late = self.pump(.05)
        self.assertTrue(all(f.event_state == 4 and not f.flags & ARMS_READY for f in late))

    def test_late_frame_cannot_renew_deadline_after_controller_scheduling_gap(self):
        held = self.hold()
        self.child.send_signal(signal.SIGSTOP)
        try:
            self.receiver.drain(self.frames.append)
            time.sleep(.32)
            self.send()
        finally:
            self.child.send_signal(signal.SIGCONT)
        after = self.pump(.07)
        after_deadline = [f for f in after if f.timestamp_ns-held.input_valid_ns > 300_000_000]
        self.assertTrue(after_deadline)
        self.assertTrue(all(f.event_state == 4 for f in after_deadline))
        self.assertTrue(all(f.input_valid_ns == held.input_valid_ns for f in after_deadline))

    def test_fresh_receive_of_old_bridge_input_does_not_extend_hold(self):
        held = self.hold()
        delayed = self.pump(.24, bridge=held.input_valid_ns+10_000_000)
        self.assertTrue(any(f.event_state == 9 for f in delayed))
        self.assertTrue(all(f.input_valid_ns == held.input_valid_ns for f in delayed))
        self.assertEqual(delayed[-1].event_state, 4)

    def test_invalid_packet_during_hold_is_not_dropout(self):
        self.hold()
        self.source.sendto(b'invalid', ('127.0.0.1', self.port))
        invalid = self.pump(.025, send=False)
        self.assertIn(invalid[-1].event_state, (4, 6))
        recovered = self.pump(.035)
        self.assertTrue(all(f.event_state == invalid[-1].event_state for f in recovered))

    def test_epoch_change_during_hold_still_latches(self):
        self.hold()
        changed = self.pump(.035, epoch=10)
        self.assertEqual(changed[-1].event_state, 6)
        self.assertTrue(all(f.event_state == 6 for f in self.pump(.025, epoch=10)))

    def test_jump_during_hold_cannot_enter_bounded_resync(self):
        self.hold()
        changed = self.pump(.035, angle=1.2)
        self.assertIn(changed[-1].event_state, (4, 6))
        self.assertFalse(any(f.event_state in (1, 2, 3) for f in changed))

    def test_operator_button_during_hold_still_latches(self):
        self.hold()
        changed = self.pump(.035, button=True)
        self.assertEqual(changed[-1].event_state, 7)
        self.assertTrue(all(f.event_state == 7 for f in self.pump(.025, button=True)))
