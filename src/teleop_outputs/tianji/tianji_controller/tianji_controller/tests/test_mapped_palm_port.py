"""Offline native/protocol tests. Only synthetic loopback sockets; no SDK."""
import json
import math
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import time
import unittest
import zlib

from tianji_runtime import workspace

# Build/bundle locations follow the workspace layout: the native tree lives
# inside the controller package and both CMake projects install under install/control.
# Deployment config and mapped-palm assets live with the description
# resources; only the native build tree stays under tianji_controller.
DESCRIPTION = workspace() / 'src/teleop_outputs/tianji/tianji_description'
BUILD = workspace() / 'build' / 'control' / 'mapped-palm'

from tianji_controller.protocol import decode_packet, ARMS_READY
from simulation.mapped_overlay import decode as decode_target


def packet(sequence, stamp, epoch=9):
    p = bytearray(656)
    p[:4] = b'TJVR'
    struct.pack_into('<HHQQqqI', p, 4, 4, 656, sequence, epoch, stamp, stamp, 255)
    for offset, index in ((44,10), (100,20)):
        struct.pack_into('<7d', p, offset, index, index+1, index+2, 0,0,0,1)
    for offset in (156,180):
        struct.pack_into('<3d', p, offset, 1,0,0)
    for i in range(8):
        struct.pack_into('<3d', p, 204+24*i, i,i+1,i+2)
        struct.pack_into('<4d', p, 396+32*i, 0,0,0,1)
    struct.pack_into('<I', p, 652, zlib.crc32(p[:652]))
    return bytes(p)


def worker(root, binary, requests):
    args = [str(binary), str(root/'config/bandwidth.yaml'),
            str(root/'assets/mapped_palm/marvin_m6_wuji2.xml'),
            str(root/'assets/mapped_palm/marvin_m6_s_ccs_696_v4_local.urdf'), '--deterministic-test']
    p = subprocess.run(args, input=requests, text=True, capture_output=True, timeout=30)
    if p.returncode:
        raise RuntimeError(p.stderr)
    return [json.loads(line) for line in p.stdout.splitlines()]


class PortTests(unittest.TestCase):
    def test_recovery_requires_fresh_rest_then_explicit_start(self):
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as output, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as reserve:
            output.bind(('127.0.0.1',0)); output.settimeout(2.)
            reserve.bind(('127.0.0.1',0)); port=reserve.getsockname()[1]; reserve.close()
            command=[str(BUILD/'mapped_palm_tjrc_controller'),'--config',str(DESCRIPTION/'config/deployment.yaml'),
                     '--model',str(DESCRIPTION/'mapped_palm/assets/mapped_palm/marvin_m6_wuji2.xml'),
                     '--joint-command-host','127.0.0.1','--joint-command-port',str(output.getsockname()[1]),
                     '--pico-port',str(port),'--simulation-recovery']
            child=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            sequence=0
            shift=0.
            angle=0.
            def pump(epoch,count=25):
                nonlocal sequence
                results=[]
                for _ in range(count):
                    sequence+=1
                    raw=bytearray(packet(sequence,time.monotonic_ns(),epoch))
                    for point in range(8): struct.pack_into('<d',raw,204+24*point,point+shift)
                    for point in range(8): struct.pack_into('<4d',raw,396+32*point,0,0,math.sin(angle/2),math.cos(angle/2))
                    struct.pack_into('<I',raw,652,zlib.crc32(raw[:652]))
                    output.sendto(raw,('127.0.0.1',port))
                    results.append(decode_packet(output.recv(1024))); time.sleep(.005)
                return results
            def key(value): child.stdin.write(value); child.stdin.flush()
            def feedback(stamp, speed=0):
                key('R '+str(stamp)+' '+' '.join(map(str,home+([speed]*14)))+'\n')
            try:
                initial=decode_packet(output.recv(1024)); home=list(initial.left_arm+initial.right_arm)
                self.assertTrue(any(f.flags&ARMS_READY for f in pump(9)))
                self.assertFalse(any(f.flags&ARMS_READY for f in pump(10)[-10:]))
                key('s'); self.assertFalse(any(f.flags&ARMS_READY for f in pump(10)))
                feedback(time.monotonic_ns()-1_000_000_000)
                key('s'); self.assertFalse(any(f.flags&ARMS_READY for f in pump(10)))
                feedback(time.monotonic_ns(),.5)
                key('s'); self.assertFalse(any(f.flags&ARMS_READY for f in pump(10)))
                feedback(time.monotonic_ns())
                self.assertFalse(any(f.flags&ARMS_READY for f in pump(10,80)))
                key('s'); self.assertTrue(any(f.flags&ARMS_READY for f in pump(10)))
                shift=.3
                self.assertTrue(any(f.flags&ARMS_READY for f in pump(10)[-10:]))
                angle=1.2
                self.assertTrue(any(f.flags&ARMS_READY for f in pump(10)[-10:]))
                key('p'); self.assertFalse(any(f.flags&ARMS_READY for f in pump(10)[-10:]))
                feedback(time.monotonic_ns()); pump(10,80)
                pump(11); key('s')
                self.assertFalse(any(f.flags&ARMS_READY for f in pump(11)))
            finally:
                child.terminate(); logs,errors=child.communicate(timeout=5)
            self.assertEqual(child.returncode,0,errors)
            self.assertIn('R accepted at measured rest',logs)
            diagnostic=next(line for line in logs.splitlines() if line.startswith('MAPPED_RESET_DIAG:'))
            fields=dict(token.split('=',1) for token in diagnostic.split()[1:])
            self.assertEqual(fields['epoch_before'],'9')
            self.assertEqual(fields['epoch_now'],'10')
            self.assertEqual(fields['generation_now'],'0')
            self.assertEqual(fields['anchor_epoch'],'9')
            self.assertEqual(float(fields['left_jump_m']),0.)
            self.assertEqual(float(fields['right_jump_rad']),0.)
            self.assertGreater(int(fields['event_sequence']),int(fields['anchor_sequence']))
            discontinuity=[dict(token.split('=',1) for token in line.split()[1:])
                for line in logs.splitlines() if line.startswith('MAPPED_RESET_DIAG:')]
            jumped=next(item for item in discontinuity if item['generation_now']=='1')
            self.assertEqual(jumped['epoch_before'],jumped['epoch_now'])
            self.assertAlmostEqual(float(jumped['left_jump_m']),.3,places=5)
            self.assertAlmostEqual(float(jumped['right_jump_m']),.3,places=5)
            self.assertEqual(jumped['hold_required'],'0')
            rotated=next(item for item in discontinuity if item['generation_now']=='2')
            self.assertAlmostEqual(float(rotated['left_jump_rad']),1.2,places=5)
            self.assertEqual(rotated['hold_required'],'0')
            self.assertEqual(fields['hold_required'],'1')

    def test_xz_calibration_requires_c_then_s_and_stable_input(self):
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as output, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as reserve:
            output.bind(('127.0.0.1',0)); output.settimeout(1.)
            reserve.bind(('127.0.0.1',0)); input_port=reserve.getsockname()[1]; reserve.close()
            args=[str(BUILD/'mapped_palm_tjrc_controller'), '--config', str(DESCRIPTION/'config/deployment.yaml'),
                  '--model', str(DESCRIPTION/'mapped_palm/assets/mapped_palm/marvin_m6_wuji2.xml'),
                  '--joint-command-host','127.0.0.1','--joint-command-port',str(output.getsockname()[1]),
                  '--pico-port',str(input_port),'--mapped-palm-xz-calibration']
            process=subprocess.Popen(args,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            def pump(sequence):
                raw=bytearray(packet(sequence,time.monotonic_ns()))
                for i in range(8):
                    struct.pack_into('<3d',raw,204+i*24,.1+.2*(i%4),.2 if i<4 else -.2,1.1)
                struct.pack_into('<I',raw,652,zlib.crc32(raw[:652]))
                output.sendto(raw,('127.0.0.1',input_port))
                result=decode_packet(output.recv(1024)); time.sleep(.005)
                return result
            try:
                self.assertEqual(decode_packet(output.recv(1024)).flags,0)
                process.stdin.write('s'); process.stdin.flush()
                for i in range(1,10): self.assertFalse(pump(i).flags & ARMS_READY)
                process.stdin.write('c'); process.stdin.flush()
                for i in range(10,450): self.assertFalse(pump(i).flags & ARMS_READY)
                process.stdin.write('s'); process.stdin.flush()
                live=False
                for i in range(450,490): live |= bool(pump(i).flags & ARMS_READY)
                self.assertTrue(live)
            finally:
                process.terminate(); logs,errors=process.communicate(timeout=5)
            self.assertEqual(process.returncode,0,errors)
            self.assertIn('calibration SUCCESS',logs)
            self.assertIn('simulation input started',logs)

    def test_help_has_no_sdk_or_input_startup(self):
        p = subprocess.run([str(BUILD/'mapped_palm_tjrc_controller'), '--help'],
                           capture_output=True, text=True, timeout=5)
        self.assertEqual(p.returncode, 0, p.stderr)

    def test_native_target_observer_is_separate_and_invalid_without_input(self):
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as output, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as observer, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as reserve:
            for sock in (output, observer, reserve):
                sock.bind(('127.0.0.1',0)); sock.settimeout(2.)
            input_port=reserve.getsockname()[1]; reserve.close()
            args=[str(BUILD/'mapped_palm_tjrc_controller'), '--config', str(DESCRIPTION/'config/deployment.yaml'),
                  '--model', str(DESCRIPTION/'mapped_palm/assets/mapped_palm/marvin_m6_wuji2.xml'),
                  '--joint-command-host','127.0.0.1','--joint-command-port',str(output.getsockname()[1]),
                  '--pico-port',str(input_port),'--target-overlay-port',str(observer.getsockname()[1])]
            process=subprocess.Popen(args,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,text=True)
            try:
                wire=observer.recv(1024)
                self.assertFalse(decode_target(wire,time.monotonic_ns())[2])
                self.assertEqual(decode_packet(output.recv(1024)).flags,0)
            finally:
                process.terminate(); _,errors=process.communicate(timeout=5)
            self.assertEqual(process.returncode,0,errors)

    def test_tjrc_live_stale_and_reset_use_destination_protocol(self):
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as output, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as reserve:
            output.bind(('127.0.0.1',0)); output.settimeout(.5)
            reserve.bind(('127.0.0.1',0)); input_port=reserve.getsockname()[1]; reserve.close()
            args=[str(BUILD/'mapped_palm_tjrc_controller'), '--config', str(DESCRIPTION/'config/deployment.yaml'),
                  '--model', str(DESCRIPTION/'mapped_palm/assets/mapped_palm/marvin_m6_wuji2.xml'),
                  '--joint-command-host','127.0.0.1','--joint-command-port',str(output.getsockname()[1]),
                  '--pico-port',str(input_port)]
            process=subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
            try:
                initial=decode_packet(output.recv(1024))
                self.assertEqual(initial.flags,0)
                live=False
                for seq in range(1,101):
                    output.sendto(packet(seq,time.monotonic_ns()),('127.0.0.1',input_port))
                    frame=decode_packet(output.recv(1024))
                    live |= bool(frame.flags & ARMS_READY)
                    time.sleep(.005)
                self.assertTrue(live,'no accepted live bilateral output')
                stale_after=time.monotonic_ns()+100_000_000
                while True:
                    frame=decode_packet(output.recv(1024))
                    if frame.timestamp_ns>stale_after:
                        self.assertFalse(frame.flags & ARMS_READY); break
                for seq in range(101,141):
                    output.sendto(packet(seq,time.monotonic_ns(),10),('127.0.0.1',input_port))
                    frame=decode_packet(output.recv(1024))
                    self.assertFalse(frame.flags & ARMS_READY)
                    time.sleep(.005)
            finally:
                process.terminate()
                _, errors=process.communicate(timeout=5)
            self.assertEqual(process.returncode,0,errors)

    @unittest.skipUnless(os.environ.get('MAPPED_PALM_REFERENCE_ROOT'), 'explicit offline source checkout')
    def test_same_input_matches_validated_source_worker(self):
        ref=Path(os.environ['MAPPED_PALM_REFERENCE_ROOT']).resolve()
        requests=''
        for tick in range(1,101):
            now=1_000_000_000+5_000_000*tick
            requests+=f'TJSC1 {tick} {now} {now} 0 0 {packet(tick,now).hex()}\n'
        actual=worker(DESCRIPTION,BUILD/'mapped_palm_native_worker',requests)
        # Same files/parameters; source model assets occupy a different directory.
        source_args=[str(ref/'build/mapped-palm-native/mapped_palm_native_worker'),
                     str(ref/'src/tianji_teleop/src/ik/mapped_palm/config/bandwidth.yaml'),
                     str(ref/'src/tianji_teleop/assets/mapped_palm/marvin_m6_wuji2.xml'),
                     str(ref/'src/tianji_teleop/assets/mapped_palm/marvin_m6_s_ccs_696_v4_local.urdf'),
                     '--deterministic-test']
        result=subprocess.run(source_args,input=requests,text=True,capture_output=True,timeout=30)
        self.assertEqual(result.returncode,0,result.stderr)
        expected=[json.loads(line) for line in result.stdout.splitlines()]
        self.assertEqual(len(actual),100); self.assertEqual(len(expected),100)
        for a,b in zip(actual,expected):
            for side in ('left','right'):
                self.assertEqual(a[side]['accepted'],b[side]['accepted'])
                for field in ('q','qdot','qddot','target_position','target_quaternion_xyzw'):
                    for x,y in zip(a[side][field],b[side][field]):
                        self.assertAlmostEqual(x,y,places=9,msg=f"tick={a['tick_id']} {side} {field}")


if __name__=='__main__': unittest.main()
