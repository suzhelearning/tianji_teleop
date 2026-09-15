import dataclasses
import struct
import unittest
import zlib
from types import SimpleNamespace
from real_robot.mapped_events import HEADER,decode_event,EventFrame,EventReceiver
from real_robot.protocol import BODY,PACKET_SIZE
from real_robot.safety import SafetyFault
from test_staged_motion import gate,frame,feedback,settle,NOW,TICK,held


def event(sequence,stamp,state=1,value=0.,epoch=7,generation=0):
    base=frame(value=value,flags=6 if state in (2,4) else 7,epoch=epoch,stamp=stamp,sequence=sequence)
    return EventFrame(**base.__dict__,event_state=state,generation=generation)


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

    def make_gate(self):
        g=gate(); g.bounded_resync=True
        g.arm(event(1,NOW),feedback(),NOW)
        now=NOW
        for seq in range(2,100):
            now+=TICK
            g.step(event(seq,now),feedback(enabled=True,stamp=now),now)
            if g.phase=='READY': break
        self.assertEqual(g.phase,'READY')
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
