"""Opt-in startup and no-input safety, loopback only, no device traffic."""
import csv
from pathlib import Path
import socket
import subprocess
import sys
import tempfile

import yaml

from test_joint_command_viewer import decode
viewer = Path(sys.argv[1]).resolve()
control = Path(__file__).resolve().parents[1]
profile = control / 'config/qp_ik_pico_shared_root_dls.yaml'


def run(config, *extra, headless=True):
    return subprocess.run([str(viewer), '--config', str(config),
                           *(['--headless'] if headless else []),
                           '--duration', '3', '--pico-teleop', *extra],
                          cwd=control, capture_output=True, text=True, timeout=30)


# A shipped disabled profile must never silently run the legacy mapper.
result = run(profile)
assert result.returncode == 1, result
with tempfile.TemporaryDirectory(prefix='tianji_franka_dls_startup_') as directory:
    folder = Path(directory)
    config = yaml.safe_load(profile.read_text())
    shared = config['spark_shared_root']
    shared['enabled'] = True
    for key in ['input_contract_artifact', 'robot_geometry_artifact']:
        shared[key] = str((profile.parent/shared[key]).resolve())
    config['controller']['pico_ee_dls_kinematics_urdf_path'] = str(
        (profile.parent/config['controller']['pico_ee_dls_kinematics_urdf_path']).resolve())
    trial = folder/'enabled.yaml'
    trial.write_text(yaml.safe_dump(config, sort_keys=False))
    result = run(trial, '--sim-allow-pico-jumps')
    assert result.returncode == 1, result
    config['controller']['model_state_only'] = False
    feedback = folder/'feedback.yaml'
    feedback.write_text(yaml.safe_dump(config, sort_keys=False))
    result = run(feedback)
    assert result.returncode == 1, result
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as commands:
        commands.bind(('127.0.0.1', 0))
        export = ['--guarded-dls-export', '--joint-command-port',
                  str(commands.getsockname()[1])]
        unsafe = [
            (trial, ['--joint-command-port', export[-1]], True),
            (trial, ['--guarded-dls-export'], True),
            (trial, export + ['--no-pico-teleop'], True),
            (trial, export + ['--actual-feedback-control'], True),
            (trial, export + ['--control-level', 'acceleration'], True),
            (trial, export + ['--algorithm', 'pico_ee_franka_ceres_lm'], True),
            (trial, export + ['--simulation-recovery'], True),
            (trial, export + ['--sim-allow-pico-jumps'], True),
            (trial, export + ['--joint-command-host', '192.0.2.1'], True),
            (trial, export, False),
            (profile, export, True),
            (control/'config/qp_ik_pico_teleop.yaml',
             export + ['--algorithm', 'pico_ee_franka_dls'], True),
        ]
        for selected, arguments, headless in unsafe:
            result = run(selected, *arguments, headless=headless)
            assert result.returncode == 1, (arguments, result)
        commands.setblocking(False)
        try:
            unexpected = commands.recv(512)
        except BlockingIOError:
            pass
        else:
            raise AssertionError(('unsafe invocation exported a command', unexpected))

        # A measured seed differs from the profile Home; absent source input
        # must neither authorize either arm nor replace that seed with Home.
        measured = yaml.safe_load(trial.read_text())
        measured['controller']['initial_left_q_rad'][0] += .015
        measured['controller']['initial_right_q_rad'][0] -= .012
        seeded = folder/'measured.yaml'
        seeded.write_text(yaml.safe_dump(measured, sort_keys=False))
        expected = (measured['controller']['initial_left_q_rad'] +
                    measured['controller']['initial_right_q_rad'])
        # Startup loads/fingerprints both kinematic models before the first
        # tick. Wait for actual UDP output, not a duration shorter than startup.
        process = subprocess.Popen(
            [str(viewer), '--config', str(seeded), '--headless', '--continuous',
             '--pico-teleop', '--pico-port', str(port), *export],
            cwd=control, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            commands.settimeout(20)
            packets = [decode(commands.recv(512))]
            commands.settimeout(1)
            packets.extend(decode(commands.recv(512)) for _ in range(5))
            assert all(frame[0] == 0 and frame[3] == 0 for frame in packets)
            assert all(abs(a - b) < 1e-8 for frame in packets
                       for a, b in zip(frame[4][:14], expected)), packets
            process.terminate()
            stdout, stderr = process.communicate(timeout=10)
            assert process.returncode in (0, 2), (stdout, stderr)
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate()
    telemetry = folder/'telemetry.csv'
    result = run(trial, '--pico-bind', '127.0.0.1', '--pico-port', str(port),
                 '--telemetry', str(telemetry))
    # Headless uses exit 2 for a session with no accepted control cycles.
    assert result.returncode in (0, 2) and not result.stderr, result
    with telemetry.open() as stream:
        rows = list(csv.DictReader(stream))
    assert rows and {r['algorithm'] for r in rows} == {'pico_ee_franka_dls'}
    assert all(r['pico_live'] == '0' for r in rows)
    assert all(r['left_ee_pinocchio_kinematics'] == r['right_ee_pinocchio_kinematics'] == '1' for r in rows)
    assert all(float(r['left_ee_ik_wall_time_us']) == float(r['right_ee_ik_wall_time_us']) == 0 for r in rows)
    assert all(r['left_accepted'] == r['right_accepted'] == '0' for r in rows)
    assert all(float(r['left_qdot_max_ratio']) == float(r['right_qdot_max_ratio']) == 0 for r in rows)
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
                 '--pico-bind', '127.0.0.1', '--pico-port', str(port),
                 '--telemetry', str(recovery))
    assert result.returncode in (0, 2), result
    with recovery.open() as stream:
        states = list(csv.DictReader(stream))
    assert states and all(r['simulation_phase'] == '0' for r in states)
    assert all(r['left_accepted'] == r['right_accepted'] == '0' for r in states)
print('franka_dls_viewer_startup: guarded no-input seed/disabled flags, unsafe export refusals, simulation recovery verified')
