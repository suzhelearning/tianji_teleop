import dataclasses
import struct
import unittest
import zlib
from types import SimpleNamespace
from real_robot.mapped_events import HEADER,CALIBRATION,INPUT_VALID,decode_event,EventFrame,EventReceiver
from real_robot.protocol import BODY,PACKET_SIZE
from real_robot.safety import SafetyFault
from test_staged_motion import gate,frame,feedback,settle,NOW,TICK,held


def event(sequence,stamp,state=1,value=0.,epoch=7,generation=0,input_valid_ns=None):
    base=frame(value=value,flags=7 if state in (1,3) else 6,epoch=epoch,stamp=stamp,sequence=sequence)
    return EventFrame(**base.__dict__,event_state=state,generation=generation,
                      input_valid_ns=stamp if input_valid_ns is None else input_valid_ns)


def wire_event(frame,token,version=3):
    body=BODY.pack(b'TJRC',2,frame.flags,PACKET_SIZE,frame.sequence,frame.timestamp_ns,
                   frame.tracking_epoch,*(frame.positions('arms')+frame.left_hand+frame.right_hand))
    extra=b''
    if version>=2:
        extra=CALIBRATION.pack(frame.calibration_revision,frame.calibration_epoch,
                               frame.calibration_state,*frame.calibration_offsets)
    if version==3: extra+=INPUT_VALID.pack(frame.input_valid_ns)
    header=HEADER.pack(b'MRE1',version,frame.event_state,HEADER.size+len(extra)+PACKET_SIZE,
                       token.encode(),frame.sequence,frame.sequence,frame.timestamp_ns,
                       frame.tracking_epoch,frame.generation)
    return header+extra+body+struct.pack('<I',zlib.crc32(body))


class EventTests(unittest.TestCase):
    def test_closed_or_gapped_channel_fails(self):
        for closed in (False,True):
            with self.subTest(closed=closed):
                receiver=EventReceiver(SimpleNamespace(port=1,close=lambda:None))
                try:
                    if closed: receiver.writer.close()
                    else:
                        body=BODY.pack(b'TJRC',2,7,PACKET_SIZE,2,NOW,7,*([0.]*54))
                        wire=body+struct.pack('<I',zlib.crc32(body))
                        header=HEADER.pack(b'MRE1',1,1,80+PACKET_SIZE,receiver.token.encode(),2,2,NOW,7,0)
                        receiver.writer.send(header+wire)
                    with self.assertRaises(SafetyFault): receiver.drain()
                finally: receiver.close()

    def make_gate(self,*,dropout=False,bounded=True,phase='TELEOP'):
        g=gate(); g.bounded_resync=bounded; g.dropout_hold=dropout
        g.arm(event(1,NOW),feedback(),NOW)
        if phase=='ALIGNING': return g,NOW,1
        now=NOW
        for seq in range(2,100):
            now+=TICK
            g.step(event(seq,now),feedback(enabled=True,stamp=now),now)
            if g.phase=='READY': break
        self.assertEqual(g.phase,'READY')
        if phase=='READY': return g,now,seq
        g.start_teleop(event(seq,now),feedback(enabled=True,stamp=now),now)
        return g,now,seq

    def test_bounded_hold_exact_command_then_slew(self):
        g,now,seq=self.make_gate()
        previous=dict(g._last_commands)
        now+=TICK
        result=g.step(event(seq+1,now,2,value=.1),feedback(enabled=True,stamp=now),now)
        self.assertEqual(result,previous)
        now+=TICK
        result=g.step(event(seq+2,now,3,value=.1,generation=1),feedback(enabled=True,stamp=now),now)
        self.assertGreater(result['arms'][0],0)
        self.assertLessEqual(result['arms'][0],.5*TICK/1e9)

    def test_fault_epoch_timeout_and_default_fail_closed(self):
        for kind in ('fault','epoch','timeout','feedback','default'):
            with self.subTest(kind=kind):
                g,now,seq=self.make_gate()
                if kind=='default': g.bounded_resync=False
                if kind=='timeout':
                    now+=TICK
                    g.step(event(seq+1,now,2),feedback(enabled=True,stamp=now),now)
                    seq+=1; now+=110_000_000
                else: now+=TICK
                f=event(seq+1,now,4 if kind=='fault' else 2,epoch=8 if kind=='epoch' else 7)
                fb=feedback(enabled=True,stamp=now)
                if kind=='feedback': fb['arms'].healthy=False
                with self.assertRaises(SafetyFault): g.step(f,fb,now)
                self.assertIsNotNone(g.fault)

    def test_protocol_identity_cycle_and_ready_consistency(self):
        token='a'*32
        body=BODY.pack(b'TJRC',2,7,PACKET_SIZE,1,NOW,7,*([0.]*54))
        wire=body+struct.pack('<I',zlib.crc32(body))
        header=HEADER.pack(b'MRE1',1,1,80+PACKET_SIZE,token.encode(),1,1,NOW,7,0)
        self.assertEqual(decode_event(header+wire,token).sequence,1)
        with self.assertRaises(SafetyFault): decode_event(header+wire,'b'*32)
        broken=bytearray(header+wire); struct.pack_into('<Q',broken,48,2)
        with self.assertRaises(SafetyFault): decode_event(broken,token)
        broken=bytearray(header+wire); broken[5]=2
        with self.assertRaises(SafetyFault): decode_event(broken,token)

    def test_missing_event_never_authorizes_hold(self):
        g,now,seq=self.make_gate()
        with self.assertRaises(SafetyFault):
            g.step(frame(stamp=now+TICK),feedback(enabled=True,stamp=now+TICK),now+TICK)

    def test_dropout_holds_sent_setpoints_for_all_devices_and_resumes_with_slew(self):
        g,now,seq=self.make_gate(dropout=True,bounded=False)
        now+=TICK; seq+=1
        sent=g.step(event(seq,now,value=.8),feedback(enabled=True,stamp=now),now)
        source=now
        for age in (50_000_001,150_000_000,300_000_000):
            now=source+age; seq+=1
            result=g.step(event(seq,now,9,value=.9,input_valid_ns=source),
                          feedback(enabled=True,stamp=now,values=sent),now)
            self.assertEqual(result,sent)
            self.assertEqual(g.display_targets,sent)
        # A separate hold recovers exactly at the inclusive source-time deadline.
        g,now,seq=self.make_gate(dropout=True,bounded=False)
        source=now
        now+=50_000_001
        sent=g.step(event(seq+1,now,9,input_valid_ns=source),feedback(enabled=True,stamp=now),now)
        now=source+300_000_000
        result=g.step(event(seq+2,now,value=.8),feedback(enabled=True,stamp=now,values=sent),now)
        self.assertEqual(result['arms'],(.0025,)*14)
        self.assertEqual(result['left_hand'],(.005,)*20)
        self.assertEqual(result['right_hand'],(.005,)*20)

    def test_source_deadline_cannot_slide_or_be_bypassed_by_late_recovery(self):
        for recovered,observe_hold in ((False,True),(True,True),(True,False)):
            with self.subTest(recovered=recovered,observe_hold=observe_hold):
                g,source,seq=self.make_gate(dropout=True)
                if observe_hold:
                    now=source+50_000_001; seq+=1
                    g.step(event(seq,now,9,input_valid_ns=source),feedback(enabled=True,stamp=now),now)
                    now=source+299_000_000; seq+=1
                    g.step(event(seq,now,9,input_valid_ns=source),feedback(enabled=True,stamp=now),now)
                now=source+300_000_001
                late=event(seq+1,now,1 if recovered else 9,
                           input_valid_ns=now if recovered else source)
                with self.assertRaises(SafetyFault):
                    g.step(late,feedback(enabled=True,stamp=now),now)
                self.assertIsNotNone(g.fault)
                with self.assertRaises(SafetyFault):
                    g.step(event(seq+2,now+TICK),feedback(enabled=True,stamp=now+TICK),now+TICK)

    def test_dropout_cannot_refresh_timestamp_or_change_identity(self):
        for change in ({'input_valid_ns':NOW},{'generation':1},{'tracking_epoch':8},
                       {'input_valid_ns':0},{'flags':7}):
            with self.subTest(change=change):
                g,source,seq=self.make_gate(dropout=True)
                now=source+60_000_000
                g.step(event(seq+1,now,9,input_valid_ns=source),feedback(enabled=True,stamp=now),now)
                now+=TICK
                held_frame=event(seq+2,now,9,input_valid_ns=source)
                if change=={'input_valid_ns':NOW}: change={'input_valid_ns':now}
                with self.assertRaises(SafetyFault):
                    g.step(dataclasses.replace(held_frame,**change),feedback(enabled=True,stamp=now),now)

    def test_dropout_faults_do_not_hide_behind_hold_or_recovery(self):
        for state in (2,3,4,5,6,7,8):
            with self.subTest(state=state):
                g,source,seq=self.make_gate(dropout=True)
                now=source+60_000_000
                g.step(event(seq+1,now,9,input_valid_ns=source),feedback(enabled=True,stamp=now),now)
                now+=TICK
                with self.assertRaises(SafetyFault):
                    g.step(event(seq+2,now,state),feedback(enabled=True,stamp=now),now)
                self.assertIsNotNone(g.fault)
        for identity in ({'generation':1},{'tracking_epoch':8}):
            g,source,seq=self.make_gate(dropout=True)
            now=source+60_000_000
            g.step(event(seq+1,now,9,input_valid_ns=source),feedback(enabled=True,stamp=now),now)
            now+=TICK
            with self.assertRaises(SafetyFault):
                g.step(dataclasses.replace(event(seq+2,now),**identity),
                       feedback(enabled=True,stamp=now),now)

    def test_dropout_keeps_feedback_hand_transport_and_bounds_guards(self):
        for kind in ('health','disabled','feedback_age','hand_health','hand_age','hand_ready',
                     'tracking','feedback_bounds','target_bounds','transport_age','calibration'):
            with self.subTest(kind=kind):
                g,source,seq=self.make_gate(dropout=True)
                now=source+60_000_000
                f=event(seq+1,now,9,input_valid_ns=source)
                measured=feedback(enabled=True,stamp=now)
                if kind=='health': measured['arms'].healthy=False
                if kind=='disabled': measured['arms'].enabled=False
                if kind=='feedback_age': measured['arms'].received_monotonic_ns=now-150_000_001
                if kind=='hand_health': measured['left_hand'].healthy=False
                if kind=='hand_age': measured['right_hand'].received_monotonic_ns=now-150_000_001
                if kind=='hand_ready': f=dataclasses.replace(f,flags=2)
                if kind=='tracking': measured['arms'].position_rad=(.2,)*14
                if kind=='feedback_bounds': measured['right_hand'].position_rad=(3.,)*20
                if kind=='target_bounds': f=dataclasses.replace(f,left_arm=(3.,)*7)
                if kind=='transport_age':
                    now=source+220_000_001
                    measured=feedback(enabled=True,stamp=now)
                if kind=='calibration':
                    from real_robot.mapped_calibration import CalibrationGuard
                    g.calibration_guard=CalibrationGuard(None)
                    g.calibration_guard.expected_revision=1
                    f=dataclasses.replace(f,calibration_revision=1,calibration_epoch=7,calibration_state=3)
                    g.step(f,measured,now)
                    now+=TICK
                    f=dataclasses.replace(f,sequence=seq+2,timestamp_ns=now,calibration_state=2)
                    measured=feedback(enabled=True,stamp=now)
                with self.assertRaises(SafetyFault): g.step(f,measured,now)
                self.assertIsNotNone(g.fault)

    def test_dropout_requires_teleop_and_validated_ready_predecessor(self):
        for phase in ('WAITING','ALIGNING','READY','HOMING','HOME_REACHED'):
            with self.subTest(phase=phase):
                if phase=='WAITING':
                    g=gate(); g.dropout_hold=True
                    with self.assertRaises(SafetyFault):
                        g.arm(event(1,NOW,9,input_valid_ns=NOW-TICK),feedback(),NOW)
                    continue
                g,now,seq=self.make_gate(dropout=True,phase=phase if phase in ('ALIGNING','READY') else 'TELEOP')
                if phase in ('HOMING','HOME_REACHED'):
                    g.start_homing(event(seq,now),feedback(enabled=True,stamp=now),now)
                    if phase=='HOME_REACHED':
                        for _ in range(5000):
                            now+=TICK; seq+=1
                            positions=g.step(event(seq,now),
                                             feedback(enabled=True,stamp=now,values=g._last_commands),now)
                            if g.phase=='HOME_REACHED': break
                        self.assertEqual(g.phase,'HOME_REACHED')
                source=now; now+=60_000_000
                with self.assertRaises(SafetyFault):
                    g.step(event(seq+1,now,9,input_valid_ns=source),feedback(enabled=True,stamp=now),now)
        g,now,seq=self.make_gate(dropout=True)
        now+=TICK
        g.step(event(seq+1,now,2),feedback(enabled=True,stamp=now),now)
        now+=TICK
        with self.assertRaises(SafetyFault):
            g.step(event(seq+2,now,9,input_valid_ns=now-TICK),feedback(enabled=True,stamp=now),now)

    def test_operator_home_during_dropout_stops_instead_of_moving(self):
        g,source,seq=self.make_gate(dropout=True)
        now=source+60_000_000
        f=event(seq+1,now,9,input_valid_ns=source)
        sent=g.step(f,feedback(enabled=True,stamp=now),now)
        with self.assertRaises(SafetyFault):
            g.start_homing(f,feedback(enabled=True,stamp=now,values=sent),now)
        self.assertIsNotNone(g.fault)
        self.assertEqual(g._last_commands,sent)

    def test_v3_metadata_and_calibration_compatibility(self):
        token='a'*32
        f=dataclasses.replace(event(1,NOW,input_valid_ns=NOW-TICK),
                              calibration_revision=2,calibration_epoch=7,calibration_state=3,
                              calibration_offsets=(.1,.2,.3,.4))
        for version in (1,2,3):
            decoded=decode_event(wire_event(f,token,version),token)
            self.assertEqual(decoded.input_valid_ns,NOW-TICK if version==3 else 0)
            self.assertEqual(decoded.calibration_offsets,f.calibration_offsets if version>=2 else (0.,)*4)
        receiver=EventReceiver(SimpleNamespace(port=1,close=lambda:None),dropout=True,calibration=True)
        try:
            receiver.writer.send(wire_event(f,receiver.token))
            self.assertEqual(receiver.drain(),f)
            self.assertIn('--real-input-hold-300ms',receiver.arguments())
        finally: receiver.close()

    def test_v3_rejects_invalid_source_metadata_ready_flags_and_downgrades(self):
        token='a'*32
        for source in (-1,0,NOW+1):
            with self.subTest(source=source),self.assertRaises(SafetyFault):
                decode_event(wire_event(event(1,NOW,9,input_valid_ns=source),token),token)
        f=dataclasses.replace(event(1,NOW,9,input_valid_ns=NOW-TICK),flags=7)
        with self.assertRaises(SafetyFault): decode_event(wire_event(f,token),token)
        for version in (1,2):
            receiver=EventReceiver(SimpleNamespace(port=1,close=lambda:None),dropout=True)
            try:
                receiver.writer.send(wire_event(event(1,NOW),receiver.token,version))
                with self.assertRaises(SafetyFault): receiver.drain()
            finally: receiver.close()

    def test_receiver_validates_every_event_before_coalescing_recovery(self):
        g,source,seq=self.make_gate(dropout=True)
        receiver=EventReceiver(SimpleNamespace(port=1,close=lambda:None),dropout=True)
        try:
            receiver.count=seq
            now=source+60_000_000
            receiver.writer.send(wire_event(event(seq+1,now,5,input_valid_ns=source),receiver.token))
            receiver.writer.send(wire_event(event(seq+2,now+TICK),receiver.token))
            with self.assertRaises(SafetyFault):
                receiver.drain(lambda f:g.observe_source(f,now+TICK))
            self.assertIsNotNone(g.fault)
        finally: receiver.close()

    def test_stop_policy_and_bounded_resync_remain_independent(self):
        g,source,seq=self.make_gate(dropout=False)
        now=source+60_000_000
        with self.assertRaises(SafetyFault):
            g.step(event(seq+1,now,9,input_valid_ns=source),feedback(enabled=True,stamp=now),now)
        g,now,seq=self.make_gate(dropout=True,bounded=False)
        now+=TICK
        with self.assertRaises(SafetyFault):
            g.step(event(seq+1,now,2),feedback(enabled=True,stamp=now),now)
        g,now,seq=self.make_gate(dropout=True,bounded=True)
        now+=TICK
        sent=g.step(event(seq+1,now,2,value=.5),feedback(enabled=True,stamp=now),now)
        now+=TICK
        resumed=g.step(event(seq+2,now,3,value=.5,generation=1),
                       feedback(enabled=True,stamp=now,values=sent),now)
        self.assertEqual(resumed['arms'],(.0025,)*14)
        source=now; now+=60_000_000
        held=g.step(event(seq+3,now,9,value=.5,generation=1,input_valid_ns=source),
                    feedback(enabled=True,stamp=now,values=resumed),now)
        self.assertEqual(held,resumed)

    def test_repeated_ready_events_do_not_refresh_source_deadline(self):
        g,source,seq=self.make_gate(dropout=True)
        now=source+40_000_000
        g.step(event(seq+1,now,input_valid_ns=source),feedback(enabled=True,stamp=now),now)
        now=source+50_000_001
        g.step(event(seq+2,now,9,input_valid_ns=source),feedback(enabled=True,stamp=now),now)
        now=source+300_000_001
        with self.assertRaises(SafetyFault):
            g.step(event(seq+3,now),feedback(enabled=True,stamp=now),now)

    def test_receiver_rejects_zero_or_rollback_after_valid_input(self):
        for source in (0,NOW-TICK):
            receiver=EventReceiver(SimpleNamespace(port=1,close=lambda:None),dropout=True)
            try:
                receiver.writer.send(wire_event(event(1,NOW),receiver.token))
                receiver.drain()
                receiver.writer.send(wire_event(event(2,NOW+TICK,4,input_valid_ns=source),receiver.token))
                with self.assertRaises(SafetyFault): receiver.drain()
                self.assertEqual(receiver.latest.sequence,1)
            finally: receiver.close()
