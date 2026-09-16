import dataclasses
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from unittest.mock import MagicMock
import time
from real_robot.mapped_calibration import CalibrationGuard
from real_robot.mapped_events import EventFrame
from real_robot.safety import SafetyFault
from test_staged_motion import gate,frame,feedback,NOW,TICK


def calibrated(state=2,revision=1,epoch=7,stamp=NOW,sequence=1):
    base=frame(stamp=stamp,epoch=epoch,sequence=sequence)
    return EventFrame(**base.__dict__,event_state=1,generation=0,
                      calibration_revision=revision,calibration_epoch=7,calibration_state=state,
                      input_valid_ns=stamp)


class CalibrationTests(unittest.TestCase):
    def test_real_entry_c_then_enter_lock_precedes_mock_enable(self):
        from real_robot import run_teleop
        events=[]
        device=MagicMock()
        device.enabled=False
        def read_feedback():
            return SimpleNamespace(position_rad=(0.,)*14,received_monotonic_ns=time.monotonic_ns(),
                                   healthy=True,enabled=device.enabled,detail='mock')
        device.read_feedback.side_effect=read_feedback
        def enable(**kwargs):
            self.assertEqual(events,['C','LOCK'])
            events.append('ENABLE'); device.enabled=True
        device.enable.side_effect=enable
        class Receiver:
            def __init__(inner,*args,**kwargs):
                self.assertFalse(kwargs['bounded'])
                self.assertTrue(kwargs['calibration'])
                inner.writer=MagicMock(); inner.writer.fileno.return_value=123
                inner.port=17000; inner.latest=None; inner.count=0; inner.state=0; inner.revision=0
            def arguments(inner): return []
            def drain(inner,validate=None):
                inner.count+=1
                inner.latest=calibrated(inner.state,revision=inner.revision,stamp=time.monotonic_ns(),sequence=inner.count)
                if validate: validate(inner.latest)
                return inner.latest
            def calibration_command(inner,action,f):
                self.assertFalse(device.enabled)
                if action==1:
                    events.append('C'); inner.revision+=1; inner.state=2
                else:
                    events.append('LOCK'); inner.state=3
            def close(inner): pass
        class Keyboard:
            def __init__(inner,on_key,on_calibrate=None): inner.calibrate=on_calibrate; inner.calls=0
            def poll_enter(inner):
                inner.calls+=1
                if device.enabled: raise KeyboardInterrupt
                if inner.calls==1:
                    inner.calibrate(); self.assertEqual(events,['C']); return False
                return inner.calls==2
            def close(inner): pass
        child=MagicMock(); child.poll.return_value=None
        with patch('real_robot.run_teleop.sys.stdin.isatty',return_value=True), \
             patch('real_robot.run_teleop.make_hardware',return_value={'arms':device}), \
             patch('real_robot.run_teleop.RealRobotViewer'), \
             patch('real_robot.run_teleop.CommandReceiver'), \
             patch('real_robot.mapped_events.EventReceiver',Receiver), \
             patch('data_collection.keyboard.CollectionKeyboard',Keyboard), \
             patch('real_robot.run_teleop.subprocess.Popen',return_value=child), \
             patch('real_robot.run_teleop.stop_controller'), \
             patch('real_robot.run_teleop.SessionLog.from_environment',return_value=None):
            result=run_teleop.main(['--confirm-real','--devices','arms','--ik-backend','mapped-palm','--mapped-palm-xz-calibration','--duration','2'])
        self.assertEqual(events,['C','LOCK','ENABLE'])
        self.assertEqual(result,0) # Existing teleop entry treats operator interrupt as clean stop.
        device.send.assert_not_called()
        device.stop.assert_called_once()

    def setUp(self):
        self.sent=[]
        self.receiver=SimpleNamespace(latest=calibrated(0,0),
            calibration_command=lambda action,f:self.sent.append((action,f)))
        self.guard=CalibrationGuard(self.receiver)
        self.gate=gate()
        self.gate.calibration_guard=self.guard

    def test_c_never_arms_and_enter_requires_lock(self):
        with self.assertRaises(SafetyFault):
            self.gate.arm(calibrated(),feedback(),NOW)
        with patch('real_robot.mapped_calibration.time.monotonic_ns',return_value=NOW):
            self.guard.calibrate(self.gate)
        self.assertFalse(self.gate.armed)
        self.assertEqual([a for a,f in self.sent],[1])
        self.gate.check_enable_ready(calibrated(),feedback(),NOW)
        with self.assertRaises(SafetyFault): self.gate.arm(calibrated(),feedback(),NOW)
        self.guard.request_lock(calibrated(),NOW)
        self.assertFalse(self.guard.lock_acknowledged(calibrated(),NOW))
        self.assertFalse(self.guard.lock_acknowledged(calibrated(3),NOW))
        self.assertTrue(self.guard.lock_acknowledged(calibrated(3,stamp=NOW+TICK,sequence=2),NOW+TICK))
        self.gate.arm(calibrated(3),feedback(),NOW)
        self.assertEqual(self.gate.phase,'ALIGNING')
        for seq in range(2,80):
            now=NOW+seq*TICK
            self.gate.step(calibrated(3,stamp=now,sequence=seq),feedback(enabled=True,stamp=now),now)
            if self.gate.phase=='READY': break
        self.assertEqual(self.gate.phase,'READY')
        self.gate.start_teleop(calibrated(3,stamp=now,sequence=seq),feedback(enabled=True,stamp=now),now)
        self.assertEqual(self.gate.phase,'TELEOP')
        before=len(self.sent)
        self.guard.calibrate(self.gate)
        self.assertEqual(len(self.sent),before)
        with self.assertRaises(SafetyFault):
            self.gate.step(calibrated(0,epoch=8,stamp=now+TICK,sequence=seq+1),feedback(enabled=True,stamp=now+TICK),now+TICK)

    def test_failed_old_epoch_or_old_revision_cannot_enable(self):
        self.guard.expected_revision=1
        for f in (calibrated(0),calibrated(1),calibrated(4),calibrated(revision=0),calibrated(epoch=8),calibrated(stamp=NOW-200_000_000)):
            with self.subTest(frame=f):
                with self.assertRaises(SafetyFault): self.gate.check_enable_ready(f,feedback(),NOW)

    def test_lock_timeout_and_c_during_lock(self):
        self.guard.expected_revision=1
        self.guard.request_lock(calibrated(),NOW)
        self.guard.calibrate(self.gate)
        self.assertEqual([a for a,f in self.sent],[2])
        with self.assertRaises(SafetyFault): self.guard.lock_acknowledged(calibrated(3,stamp=NOW+3_000_000_000),NOW+3_000_000_000)

    def test_optional_keyboard_callback_keeps_enter(self):
        from data_collection.keyboard import CollectionKeyboard
        calls=[]
        with patch('data_collection.keyboard.sys.stdin',SimpleNamespace(fileno=lambda:123)), \
             patch('data_collection.keyboard.termios.tcgetattr',return_value=[]), \
             patch('data_collection.keyboard.termios.tcsetattr'), \
             patch('data_collection.keyboard.tty.setcbreak'), \
             patch('data_collection.keyboard.select.select',return_value=([123],[],[])), \
             patch('data_collection.keyboard.os.read',side_effect=[b'C',b'\n']):
            keyboard=CollectionKeyboard(calls.append,on_calibrate=lambda:calls.append('calibrate'))
            try: self.assertTrue(keyboard.poll_enter())
            finally: keyboard.close()
        self.assertEqual(calls,['calibrate'])
