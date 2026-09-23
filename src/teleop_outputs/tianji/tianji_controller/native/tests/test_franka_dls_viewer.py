"""Opt-in startup and no-input safety, loopback only, no device traffic."""
import csv
from pathlib import Path
import select
import signal
import socket
import subprocess
import struct
import sys
import tempfile
import time
import zlib

import yaml
from test_joint_command_viewer import decode, hand_packet
from pico_viewer_fixture import encode_packet, reserve_udp_port

viewer = Path(sys.argv[1]).resolve()
control = Path(__file__).resolve().parents[1]
profile = control / 'config/qp_ik_pico_shared_root_dls.yaml'


def run(config, *extra):
    return subprocess.run([str(viewer), '--config', str(config), '--headless',
                           '--duration', '3', '--pico-teleop', *extra],
                          cwd=control, capture_output=True, text=True, timeout=30)

def executor_command(config, pico_port, hand_port, output_port, *extra):
    return [str(viewer), '--config', str(config), '--headless', '--continuous',
            '--franka-dls-executor', '--pico-teleop', '--hand-teleop',
            '--pico-bind', '127.0.0.1', '--pico-port', str(pico_port),
            '--hand-bind', '127.0.0.1', '--hand-port', str(hand_port),
            '--joint-command-port', str(output_port), *extra]


def exercise_executor(trial, folder):
    config = yaml.safe_load(trial.read_text())
    # Measured startup deliberately differs from nominal/Home. The first
    # exported reference must retain it, not silently substitute profile Home.
    config['controller']['initial_left_q_rad'][0] += .03
    config['controller']['initial_right_q_rad'][0] -= .03
    measured = folder / 'measured.yaml'
    measured.write_text(yaml.safe_dump(config, sort_keys=False))
    initial = tuple(config['controller']['initial_left_q_rad'] +
                    config['controller']['initial_right_q_rad'])
    pico_port, hand_port = reserve_udp_port(), reserve_udp_port()
    while hand_port == pico_port:
        hand_port = reserve_udp_port()
    joints = folder / 'executor-joints.csv'
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver, \
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        receiver.bind(('127.0.0.1', 0))
        command = executor_command(measured, pico_port, hand_port,
                                   receiver.getsockname()[1],
                                   '--joint-telemetry', str(joints))
        process = subprocess.Popen(command, cwd=control, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, text=True)
        records = []
        sequence = 0
        epoch = 9

        def pump(seconds, mode='live', hands=(True, True)):
            nonlocal sequence
            start = time.monotonic()
            next_send = start
            frames = []
            while time.monotonic() - start < seconds:
                now = time.monotonic()
                if now >= next_send:
                    sequence += 1
                    if mode != 'stale':
                        packet = bytearray(encode_packet(
                            sequence, time.monotonic_ns(), 0., 0.))
                        struct.pack_into('<Q', packet, 16, epoch)
                        if mode == 'invalid':
                            # Wire-valid but impossible bilateral morphology:
                            # zero upper-arm lengths must invalidate mapping.
                            for base in (204, 300):
                                packet[base+24:base+48] = packet[base:base+24]
                        if mode == 'old_bridge':
                            struct.pack_into('<q', packet, 32,
                                             time.monotonic_ns() - 500_000_000)
                        struct.pack_into('<I', packet, 652, zlib.crc32(packet[:-4]))
                        sender.sendto(packet, ('127.0.0.1', pico_port))
                    hand, _, _ = hand_packet(sequence, *hands)
                    sender.sendto(hand, ('127.0.0.1', hand_port))
                    next_send = now + .01
                if select.select([receiver], [], [], .003)[0]:
                    frame = decode(receiver.recv(512))
                    assert frame[1] > records[-1][1] and frame[2] > records[-1][2]
                    records.append(frame)
                    frames.append((time.monotonic() - start, frame))
            assert process.poll() is None, 'continuous DLS controller stopped'
            assert frames, 'controller stopped publishing TJRC'
            return frames

        try:
            receiver.settimeout(15)
            first = decode(receiver.recv(512))
            assert first[0] == 0
            assert all(abs(a-b) < 1e-10 for a, b in zip(first[4][:14], initial))
            records.append(first)
            live = pump(1.5)
            assert any(frame[0] == 7 for _, frame in live), 'DLS arms never became ready'
            _, left, right = hand_packet(sequence+1, True, True)
            assert any(frame[0] == 7 and frame[4][14:34] == left and
                       frame[4][34:54] == right for _, frame in live)
            stale = pump(.4, 'stale', (True, False))
            assert all(not frame[0] & 1 for elapsed, frame in stale if elapsed > .15)
            assert any(frame[0] & 4 for elapsed, frame in stale if elapsed > .2)
            assert all(not frame[0] & 2 for elapsed, frame in stale if elapsed > .2)
            resumed = pump(1., hands=(False, True))
            assert any(frame[0] & 1 for _, frame in resumed), 'fresh mapping did not recover'
            assert any(frame[0] & 2 for elapsed, frame in resumed if elapsed > .2)
            assert all(not frame[0] & 4 for elapsed, frame in resumed if elapsed > .2)
            invalid = pump(.25, 'invalid')
            assert all(not frame[0] & 1 for elapsed, frame in invalid if elapsed > .06)
            assert any(frame[0] & 1 for _, frame in pump(1.)), 'valid mapping did not recover'
            delayed = pump(.25, 'old_bridge')
            assert all(not frame[0] & 1 for elapsed, frame in delayed if elapsed > .06)
            assert any(frame[0] & 1 for _, frame in pump(.6)), 'fresh bridge did not recover'
            epoch = 10
            reset = pump(.8)
            assert any(frame[3] == epoch for _, frame in reset)
            assert all(not frame[0] & 1 for elapsed, frame in reset if elapsed > .06), \
                'epoch reset restored hardware arms without controller restart'
            process.send_signal(signal.SIGTERM)
            stdout, stderr = process.communicate(timeout=10)
            assert process.returncode == 0 and not stderr, (stdout, stderr)
            while select.select([receiver], [], [], .02)[0]:
                records.append(decode(receiver.recv(512)))
            assert records[-1][0] == 0, 'orderly shutdown did not revoke readiness'
            with joints.open() as stream:
                rows = {int(row['sequence']): row for row in csv.DictReader(stream)}
            for frame in records:
                if not frame[0] & 1:
                    continue
                row = rows[frame[1] - first[1]]
                expected = tuple(float(row[f'{side}_j{j}_reference_q'])
                                 for side in ('left', 'right') for j in range(1, 8))
                assert all(abs(a-b) < 1e-10 for a, b in zip(frame[4][:14], expected)), \
                    'TJRC differs from committed bilateral Ruckig references'
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate()



# A shipped disabled profile must never silently run the legacy mapper.
result = run(profile)
assert result.returncode != 0, result
with tempfile.TemporaryDirectory(prefix='tianji_franka_dls_startup_') as directory:
    folder = Path(directory)
    config = yaml.safe_load(profile.read_text())
    shared = config['shared_root']
    shared['enabled'] = True
    for key in ['input_contract_artifact', 'robot_geometry_artifact']:
        shared[key] = str((profile.parent/shared[key]).resolve())
    config['controller']['pico_ee_dls_kinematics_urdf_path'] = str(
        (profile.parent/config['controller']['pico_ee_dls_kinematics_urdf_path']).resolve())
    trial = folder/'enabled.yaml'
    trial.write_text(yaml.safe_dump(config, sort_keys=False))
    result = run(trial, '--sim-allow-pico-jumps')
    assert result.returncode != 0, result
    result = run(trial, '--joint-command-port', '26999')
    assert result.returncode != 0, result
    # Every failure must occur before opening a joint-command stream.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(('127.0.0.1', 0))
        guarded = executor_command(trial, 26001, 26002, receiver.getsockname()[1])
        rejected = [
            guarded + ['--no-pico-teleop'],
            guarded + ['--no-hand-teleop'],
            guarded + ['--joint-command-port', '0'],
            guarded + ['--pico-bind', '0.0.0.0'],
            guarded + ['--hand-bind', '0.0.0.0'],
            guarded + ['--joint-command-host', '192.0.2.1'],
            guarded + ['--simulation-recovery'],
            guarded + ['--sim-allow-pico-jumps'],
            guarded + ['--actual-feedback-control'],
            [arg for arg in guarded if arg != '--continuous'],
            [arg for arg in guarded if arg != '--headless'],
            executor_command(profile, 26001, 26002, receiver.getsockname()[1]),
        ]
        non_ruckig = yaml.safe_load(trial.read_text())
        non_ruckig['pico_ee_franka_dls']['post_smoothing']['mode'] = 'none'
        unsupported = folder/'non-ruckig.yaml'
        unsupported.write_text(yaml.safe_dump(non_ruckig, sort_keys=False))
        rejected.append(executor_command(unsupported, 26001, 26002, receiver.getsockname()[1]))
        for command in rejected:
            result = subprocess.run(command, cwd=control, capture_output=True,
                                    text=True, timeout=15)
            assert result.returncode != 0, (command, result)
            assert not select.select([receiver], [], [], 0)[0], command
    exercise_executor(trial, folder)
    config['controller']['model_state_only'] = False
    feedback = folder/'feedback.yaml'
    feedback.write_text(yaml.safe_dump(config, sort_keys=False))
    result = run(feedback)
    assert result.returncode != 0, result
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    telemetry = folder/'telemetry.csv'
    result = run(trial, '--pico-bind', '127.0.0.1', '--pico-port', str(port),
                 '--telemetry', str(telemetry))
    # Waiting is a successful bounded hold, not an IK/Ruckig failure.
    assert result.returncode == 0 and not result.stderr, result
    with telemetry.open() as stream:
        rows = list(csv.DictReader(stream))
    assert rows and {r['algorithm'] for r in rows} == {'pico_ee_franka_dls'}
    assert all(r['pico_live'] == '0' for r in rows)
    assert all(r['left_ee_pinocchio_kinematics'] == r['right_ee_pinocchio_kinematics'] == '1' for r in rows)
    assert all(float(r['left_ee_ik_wall_time_us']) == float(r['right_ee_ik_wall_time_us']) == 0 for r in rows)
    assert all(r['left_accepted'] == r['right_accepted'] == '0' for r in rows)
    joints = folder/'recovery_joints.csv'
    recovery = folder/'recovery.csv'
    result = run(trial, '--simulation-recovery', '--pico-bind', '127.0.0.1',
                 '--pico-port', str(port), '--joint-telemetry', str(joints),
                 '--telemetry', str(recovery))
    assert result.returncode in (0, 2) and not result.stderr, result
    with recovery.open() as stream:
        states = list(csv.DictReader(stream))
    assert states and all(r['hold_reason'] == r['left_hold_reason'] == r['right_hold_reason'] == 'none' for r in states)
    assert all(r['left_accepted'] == r['right_accepted'] == '0' for r in states)
    assert all(r['simulation_phase'] == '0' for r in states)
    with joints.open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) > 3
    assert all(r['left_ruckig_output'] == r['right_ruckig_output'] == '1' for r in rows)
    assert all(r['left_reference_jerk_valid'] == r['right_reference_jerk_valid'] == '1' for r in rows[2:])
    result = run(trial, '--simulation-recovery', '--sim-allow-pico-jumps',
                 '--pico-bind', '127.0.0.1', '--pico-port', str(port))
    assert result.returncode in (0, 2), result
print('franka_dls_viewer: restricted executor, measured seed, TJRC Ruckig references, '
      'mapping/bridge freshness, independent hands, epoch latch and shutdown verified')
