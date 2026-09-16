"""Native C/lock lifecycle with synthetic input, no SDK or devices."""
import socket
import struct
import subprocess
import time
import unittest
import zlib
from test_port import ROOT,BUNDLE,BUILD,packet
from real_robot.run_teleop import CommandReceiver
from real_robot.mapped_events import EventReceiver


class NativeCalibrationTests(unittest.TestCase):
    def test_c_success_lock_and_epoch_invalidation_without_bounded(self):
        self.exercise(False)

    def test_c_failed_retry_and_lock_with_bounded(self):
        self.exercise(True)

    def test_c_success_lock_and_epoch_invalidation_with_dropout(self):
        self.exercise(True, dropout=True)

    def exercise(self,bounded,dropout=False):
        receiver=EventReceiver(CommandReceiver(0),bounded=bounded,calibration=True,dropout=dropout)
        source=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        source.bind(('127.0.0.1',0)); port=source.getsockname()[1]; source.close()
        source=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        args=['pixi','run','--manifest-path',str(ROOT/'pixi.toml'),str(BUILD/'mapped_palm_tjrc_controller'),
              '--config',str(BUNDLE/'config/deployment.yaml'),'--model',str(BUNDLE/'assets/mapped_palm/marvin_m6_wuji2.xml'),
              '--pico-port',str(port),'--joint-command-host','127.0.0.1','--joint-command-port',str(receiver.port),
              *receiver.arguments()]
        self.assertEqual('--real-resync-bounded' in args,bounded)
        child=subprocess.Popen(args,pass_fds=(receiver.writer.fileno(),),stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        receiver.writer.close(); sequence=0
        def pump(count,epoch=9,moving=False):
            nonlocal sequence
            frames=[]
            for _ in range(count):
                sequence+=1; raw=bytearray(packet(sequence,time.monotonic_ns(),epoch))
                for i in range(8): struct.pack_into('<3d',raw,204+i*24,.1+.2*(i%4),.2 if i<4 else -.2,1.1+(.08 if moving and sequence%2 else 0))
                struct.pack_into('<I',raw,652,zlib.crc32(raw[:652]))
                source.sendto(raw,('127.0.0.1',port)); time.sleep(.005)
                receiver.drain(frames.append)
            return frames
        try:
            first=pump(400)
            self.assertTrue(first)
            self.assertTrue(all(not f.flags&1 and f.calibration_state==0 for f in first))
            if bounded:
                receiver.calibration_command(1,receiver.latest)
                failed=pump(520,moving=True)
                self.assertEqual(failed[-1].calibration_state,4)
                self.assertTrue(all(not f.flags&1 for f in failed))
            receiver.calibration_command(1,receiver.latest)
            sampled=pump(520)
            self.assertTrue(any(f.calibration_state==1 for f in sampled))
            self.assertTrue(any(f.calibration_state==2 and f.flags&1 for f in sampled))
            self.assertEqual(receiver.latest.calibration_revision,2 if bounded else 1)
            self.assertEqual(receiver.latest.calibration_epoch,9)
            receiver.calibration_command(2,receiver.latest)
            locked=pump(30)
            self.assertEqual(locked[-1].calibration_state,3)
            invalid=pump(30,epoch=10)
            self.assertEqual(invalid[-1].calibration_state,0)
            self.assertFalse(invalid[-1].flags&1)
        finally:
            child.terminate(); out,err=child.communicate(timeout=5)
            receiver.close(); source.close()
            if child.returncode: print('native calibration failure:',err.decode(),out.decode())
        self.assertEqual(child.returncode,0,err.decode())
