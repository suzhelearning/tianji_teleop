"""Offline real-trace comparison. No SDK, no actuator publisher.

Deterministic worker comparison uses recorded receive order. Optional paced UDP
exercise targets only a child adapter and an owned ephemeral TJRC receiver.
UDP exercise is not phase-identical and is NOT a numerical equivalence verdict.
"""
import argparse
from contextlib import ExitStack
import hashlib
import json
from pathlib import Path
import socket
import struct
import subprocess
import sys
import threading
import time
import zlib

from tianji_runtime.resources import native_executable, package_share


def shifted(packet, offset, bridge_offset=None):
    data = bytearray(packet)
    source, bridge = struct.unpack_from('<qq', data, 24)
    struct.pack_into('<qq', data, 24, source + offset, bridge + (offset if bridge_offset is None else bridge_offset))
    struct.pack_into('<I', data, len(data)-4, zlib.crc32(data[:-4]))
    return bytes(data)


def udp_exercise(records, bundle, recovery_test=False):
    from .protocol import decode_packet, ARMS_READY
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as output, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserved:
        output.bind(('127.0.0.1', 0)); output.settimeout(.1)
        reserved.bind(('127.0.0.1', 0)); port = reserved.getsockname()[1]; reserved.close()
        args = [str(native_executable("mapped_palm_tjrc_controller")),
                '--config', str(bundle/'config/deployment.yaml'), '--model', str(bundle/'mapped_palm/assets/mapped_palm/marvin_m6_wuji2.xml'),
                '--joint-command-host', '127.0.0.1', '--joint-command-port', str(output.getsockname()[1]), '--pico-port', str(port)]
        if recovery_test: args.append('--simulation-recovery')
        child = subprocess.Popen(args, stdin=subprocess.PIPE if recovery_test else subprocess.DEVNULL,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        frames = []; failures = []; stop = threading.Event()
        def collect():
            while not stop.is_set():
                try: frames.append(decode_packet(output.recv(1024)))
                except socket.timeout: continue
                except Exception as error: failures.append(str(error)); return
        reader = threading.Thread(target=collect)
        reader.start()
        try:
            deadline = time.monotonic()+10
            while not frames:
                if child.poll() is not None or failures or time.monotonic()>deadline:
                    raise RuntimeError('adapter failed to produce initial TJRC')
                time.sleep(.01)
            origin = time.monotonic_ns()+50_000_000
            first_source = struct.unpack_from('<q', records[0].packet, 24)[0]
            offset = origin+records[0].relative_receive_ns-first_source
            first_bridge = struct.unpack_from('<q', records[0].packet, 32)[0]
            bridge_offset = origin+records[0].relative_receive_ns-first_bridge
            max_lag = 0
            last_ready=None; recovery_sent=None; start_sent=False; recovered=False
            for index, record in enumerate(records):
                target = origin+record.relative_receive_ns
                delay = (target-time.monotonic_ns())/1e9
                if delay>0: time.sleep(delay)
                max_lag=max(max_lag, time.monotonic_ns()-target)
                if child.poll() is not None or failures: raise RuntimeError('adapter exited or TJRC decode failed')
                output.sendto(shifted(record.packet, offset, bridge_offset), ('127.0.0.1',port))
                if recovery_test:
                    current=frames[-1]
                    if current.flags & ARMS_READY:
                        last_ready=current
                        if start_sent: recovered=True
                    elif last_ready is not None and current.tracking_epoch!=last_ready.tracking_epoch and recovery_sent is None:
                        # Test fixture only: synthetic settled state is NOT actual
                        # measured feedback and must never authorize hardware.
                        q=last_ready.left_arm+last_ready.right_arm
                        message='R '+str(time.monotonic_ns())+' '+' '.join(map(str,(*q,*([0.]*14))))+'\n'
                        child.stdin.write(message); child.stdin.flush(); recovery_sent=time.monotonic()
                    if recovery_sent is not None and not start_sent and time.monotonic()-recovery_sent>.8:
                        child.stdin.write('s'); child.stdin.flush(); start_sent=True
                if index%2000==0: print(f'UDP trace {index}/{len(records)}',flush=True)
            time.sleep(.3)
        finally:
            child.terminate()
            try: _, errors = child.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill(); child.communicate(); raise
            finally: stop.set(); reader.join(timeout=2)
        if child.returncode or failures:
            raise RuntimeError(f'adapter exit={child.returncode} errors={errors} decoder={failures}')
        ready = [f for f in frames if f.flags & ARMS_READY]
        transitions = []
        previous = None
        for frame in frames:
            state = (bool(frame.flags & ARMS_READY),frame.tracking_epoch)
            if state != previous:
                transitions.append(dict(sequence=frame.sequence,ready=state[0],epoch=state[1]))
                previous=state
        return dict(scope='paced_real_trace_to_target_TJRC_adapter_only',received=len(frames),ready=len(ready),
                    transitions=transitions,maximum_replay_lag_ms=max_lag/1e6,exit_code=child.returncode,
                    simulation_recovery_test=recovery_test,synthetic_rest_feedback=recovery_test,recovered=recovered,
                    numerical_equivalence_checked=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root',required=True,type=Path)
    parser.add_argument('--trace',required=True,type=Path)
    parser.add_argument('--report',required=True,type=Path)
    parser.add_argument('--udp',action='store_true')
    args=parser.parse_args()
    sys.path.insert(0,str(args.source_root/'src/tianji_teleop'))
    from tianji_teleop.hand_tracking.tjvr_trace import iter_tjvr_records, TjvrRecord
    from tianji_teleop.hand_tracking.spark_replay import iter_reference_ticks
    from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReferenceTjvrReceiver
    from tianji_teleop.hand_tracking.mapped_palm_worker_client import MappedPalmWorkerClient
    with args.trace.open('rb') as stream: records=list(iter_tjvr_records(stream))
    if not records: raise ValueError('empty trace')
    source=args.source_root/'src/tianji_teleop'
    bundle=package_share("tianji_description")
    result=dict(complete=False,trace_sha256=hashlib.sha256(args.trace.read_bytes()).hexdigest(),
                input_frames=len(records),duration_s=records[-1].relative_receive_ns/1e9,
                scope='real_trace_deterministic_workers_plus_optional_target_adapter',
                limitations=['not full scheduler/state-machine equivalence','no MuJoCo dynamics or real hardware',
                             'no C calibration or Manus input','same deployment Home for both workers',
                             'XR source and PC bridge clocks rebased independently; original absolute network latency unavailable; raw geometry unchanged'])
    result['asset_sha256'] = {
        name: hashlib.sha256(path.read_bytes()).hexdigest()
        for name,path in {
            'shared_deployment_config': bundle/'config/deployment.yaml',
            'source_worker': args.source_root/'build/mapped-palm-native/mapped_palm_native_worker',
            'target_worker': native_executable("mapped_palm_native_worker"),
            'target_adapter': native_executable("mapped_palm_tjrc_controller"),
        }.items()
    }
    with args.report.open('x') as report:
        try:
            first_source=struct.unpack_from('<q',records[0].packet,24)[0]
            offset=1_000_000_000+records[0].relative_receive_ns-first_source
            first_bridge=struct.unpack_from('<q',records[0].packet,32)[0]
            bridge_offset=1_000_000_000+records[0].relative_receive_ns-first_bridge
            rebased=[TjvrRecord(r.relative_receive_ns,shifted(r.packet,offset,bridge_offset)) for r in records]
            receiver=ReferenceTjvrReceiver('trace-compare',.15,.6,target_source='mapped_corrected_palm')
            common=dict(config=bundle/'config/deployment.yaml',deterministic_test=True,startup_handshake=True)
            maxima={name:0. for name in ('q','qdot','qddot','target_position','target_quaternion_xyzw')}
            first_difference=None; cycles=0; live=0; accepted={'left':0,'right':0}; resets=0
            with ExitStack() as stack:
                old=stack.enter_context(MappedPalmWorkerClient(worker=args.source_root/'build/mapped-palm-native/mapped_palm_native_worker',
                    model=source/'assets/mapped_palm/marvin_m6_wuji2.xml',urdf=source/'assets/mapped_palm/marvin_m6_s_ccs_696_v4_local.urdf',**common))
                new=stack.enter_context(MappedPalmWorkerClient(worker=native_executable("mapped_palm_native_worker"),
                    model=bundle/'mapped_palm/assets/mapped_palm/marvin_m6_wuji2.xml',urdf=bundle/'mapped_palm/assets/mapped_palm/marvin_m6_s_ccs_696_v4_local.urdf',**common))
                for tick in iter_reference_ticks(rebased,receiver=receiver):
                    a,b=old.step(tick),new.step(tick); cycles+=1; live+=bool(a['input_live']); resets+=bool(a['epoch_reset'])
                    for field in ('input_live','applied_epoch','applied_sequence','epoch_reset'):
                        if a[field]!=b[field] and first_difference is None: first_difference=dict(tick=tick.tick_id,field=field)
                    for side in ('left','right'):
                        accepted[side]+=bool(a[side]['accepted'])
                        if a[side]['accepted']!=b[side]['accepted'] and first_difference is None:
                            first_difference=dict(tick=tick.tick_id,side=side,field='accepted')
                        for field in maxima:
                            error=max(abs(x-y) for x,y in zip(a[side][field],b[side][field]))
                            maxima[field]=max(maxima[field],error)
                            if error>1e-9 and first_difference is None: first_difference=dict(tick=tick.tick_id,side=side,field=field,error=error)
                    if cycles%4000==0: print(f'Worker trace {cycles} ticks',flush=True)
            result['workers']=dict(passed=first_difference is None,cycles=cycles,input_live=live,accepted=accepted,epoch_resets=resets,
                                   receiver=receiver.stats(),maximum_absolute_error=maxima,first_difference=first_difference)
            if args.udp: result['target_adapter']=udp_exercise(records,bundle)
            result['complete']=True
        except Exception as error:
            result['error']=repr(error)
            raise
        finally:
            json.dump(result,report,indent=2); report.write('\n'); report.flush()
    print(json.dumps(result),flush=True)
    return 0 if result['workers']['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
