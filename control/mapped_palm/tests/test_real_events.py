"""Actual native adapter, synthetic PICO only; no hardware/SDK."""
import math
import socket
import struct
import subprocess
import time
import unittest
import zlib
from test_port import ROOT,BUNDLE,BUILD,packet
from real_robot.run_teleop import CommandReceiver
from real_robot.mapped_events import EventReceiver


class NativeEventTests(unittest.TestCase):
    def test_private_channel_through_launcher_and_same_epoch_resync(self):
        self.exercise(False)

    def test_unconfirmed_jump_times_out_and_latches(self):
        self.exercise(True)

    def exercise(self, timeout):
        receiver=EventReceiver(CommandReceiver(0))
        source=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        source.bind(('127.0.0.1',0)); port=source.getsockname()[1]; source.close()
        source=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        args=['pixi','run','--manifest-path',str(ROOT/'pixi.toml'),str(BUILD/'mapped_palm_tjrc_controller'),
              '--config',str(BUNDLE/'config/deployment.yaml'),'--model',str(BUNDLE/'assets/mapped_palm/marvin_m6_wuji2.xml'),
              '--pico-port',str(port),'--joint-command-host','127.0.0.1','--joint-command-port',str(receiver.port),
              *receiver.arguments()]
        child=subprocess.Popen(args,pass_fds=(receiver.writer.fileno(),),stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        receiver.writer.close()
        sequence=0
        def pump(angle,epoch=9,count=90):
            nonlocal sequence
            frames=[]
            for _ in range(count):
                sequence+=1; raw=bytearray(packet(sequence,time.monotonic_ns(),epoch))
                for point in range(8): struct.pack_into('<4d',raw,396+point*32,0,0,math.sin(angle/2),math.cos(angle/2))
                struct.pack_into('<I',raw,652,zlib.crc32(raw[:652]))
                source.sendto(raw,('127.0.0.1',port))
                time.sleep(.005)
                receiver.drain(frames.append)
            return frames
        try:
            initial=pump(0.,count=180)
            self.assertTrue(any(f.event_state==1 for f in initial))
            moved=pump(1.2)
            self.assertTrue(any(f.event_state in (2,3) for f in moved))
            self.assertTrue(any(f.event_state==1 and f.generation==1 for f in moved[-20:]))
            if timeout:
                changed=[]
                for i in range(45): changed.extend(pump(0. if i%2 else 2.4,count=1))
                self.assertTrue(any(f.event_state==2 for f in changed))
                self.assertTrue(all(f.event_state==8 for f in changed[-10:]))
                self.assertTrue(all(f.event_state==8 for f in pump(1.2)[-10:]))
            else:
                changed=pump(1.2,epoch=10)
                self.assertTrue(all(f.event_state==6 for f in changed[-20:]))
        finally:
            child.terminate(); stdout,stderr=child.communicate(timeout=5)
            receiver.close(); source.close()
        self.assertEqual(child.returncode,0,stderr.decode())
