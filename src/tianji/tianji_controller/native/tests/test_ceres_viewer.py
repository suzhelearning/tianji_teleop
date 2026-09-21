"""Opt-in startup and no-input safety, loopback only, no device traffic."""
import csv
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time

import yaml
from test_pico_viewer_integration import encode_hand_packet, parse_summary, reserve_udp_port

viewer = Path(sys.argv[1]).resolve()
control = Path(__file__).resolve().parents[1]
profile = control / 'config/qp_ik_pico_shared_root_ceres.yaml'


def run(config, *extra):
    return subprocess.run([str(viewer), '--config', str(config), '--headless',
                           '--duration', '3', '--pico-teleop', *extra],
                          cwd=control, capture_output=True, text=True, timeout=30)


# A shipped disabled profile must never silently run the legacy mapper.
result = run(profile)
assert result.returncode != 0 and 'requires enabled shared-root' in result.stderr, result
with tempfile.TemporaryDirectory(prefix='tianji_ceres_startup_') as directory:
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
    result = run(trial, '--joint-command-port', '26999')
    assert result.returncode != 0 and 'forbids joint command export' in result.stderr, result
    config['controller']['model_state_only'] = False
    feedback = folder/'feedback.yaml'
    feedback.write_text(yaml.safe_dump(config, sort_keys=False))
    result = run(feedback)
    assert result.returncode != 0 and 'model-only' in result.stderr, result
    if '--unavailable' in sys.argv[2:]:
        result = run(trial)
        assert result.returncode != 0 and 'Ceres not built' in result.stderr, result
        print('Ceres-disabled build rejects opt-in profile without opening a session')
        sys.exit(0)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    telemetry = folder/'telemetry.csv'
    result = run(trial, '--pico-bind', '127.0.0.1', '--pico-port', str(port),
                 '--telemetry', str(telemetry))
    # Headless uses exit 2 for a session with no accepted control cycles.
    assert result.returncode in (0, 2) and not result.stderr, result
    with telemetry.open() as stream:
        rows = list(csv.DictReader(stream))
    assert rows and {r['algorithm'] for r in rows} == {'pico_ee_franka_ceres_lm'}
    assert all(r['pico_live'] == '0' for r in rows)
    assert all(r['left_ee_pinocchio_kinematics'] == r['right_ee_pinocchio_kinematics'] == '1' for r in rows)
    assert all(float(r['left_ee_ik_wall_time_us']) == float(r['right_ee_ik_wall_time_us']) == 0 for r in rows)
    assert all(r['left_accepted'] == r['right_accepted'] == '0' for r in rows)
    assert all(float(r['left_qdot_max_ratio']) == float(r['right_qdot_max_ratio']) == 0 for r in rows)
    result = run(trial, '--simulation-recovery', '--hand-teleop', '--hand-bind', '0.0.0.0')
    assert result.returncode != 0 and 'hand input must bind' in result.stderr, result
    result = run(trial, '--simulation-recovery', '--hand-teleop', '--pico-port', '26001', '--hand-port', '26001')
    assert result.returncode != 0 and 'ports must differ' in result.stderr, result
    # A real loopback TJH2 receiver must not animate fingers before explicit S.
    for backend in ('ceres', 'dls'):
        hand_profile = control / f'config/qp_ik_pico_shared_root_{backend}.yaml'
        hand_config = yaml.safe_load(hand_profile.read_text())
        hand_config['spark_shared_root']['enabled'] = True
        for key in ('input_contract_artifact', 'robot_geometry_artifact'):
            hand_config['spark_shared_root'][key] = str((hand_profile.parent/hand_config['spark_shared_root'][key]).resolve())
        key = 'pico_ee_dls_kinematics_urdf_path'
        hand_config['controller'][key] = str((hand_profile.parent/hand_config['controller'][key]).resolve())
        hand_trial = folder / f'{backend}_hands.yaml'
        hand_trial.write_text(yaml.safe_dump(hand_config))
        hand_port = reserve_udp_port()
        pico_port = reserve_udp_port()
        while pico_port == hand_port:
            pico_port = reserve_udp_port()
        process = subprocess.Popen([str(viewer), '--config', str(hand_trial), '--headless',
            '--duration', '1.5', '--pico-teleop', '--pico-port', str(pico_port),
            '--simulation-recovery', '--hand-teleop', '--hand-port', str(hand_port)],
            cwd=control, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                for seq in range(1, 121):
                    sender.sendto(encode_hand_packet(seq, .7, .8), ('127.0.0.1', hand_port))
                    time.sleep(.01)
            stdout, stderr = process.communicate(timeout=15)
            assert process.returncode in (0, 2) and not stderr, (stdout, stderr)
            summary = parse_summary(stdout)
            assert int(summary['hand_accepted']) > 0, summary
            assert summary['hand_configured'] == '1' and summary['hand_live'] == '0', summary
            assert float(summary['hand_left_q0']) == float(summary['hand_right_q0']) == 0, summary
            assert ': WAITING' in stdout and ': TELEOP' not in stdout, stdout
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.communicate(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.communicate()
    recovery_telemetry = folder/'recovery.csv'
    recovery_joints = folder/'recovery_joints.csv'
    result = run(trial, '--simulation-recovery', '--pico-bind', '127.0.0.1',
                 '--pico-port', str(port), '--telemetry', str(recovery_telemetry),
                 '--joint-telemetry', str(recovery_joints))
    assert result.returncode in (0, 2) and not result.stderr, result
    assert 'CERES_SIM: WAITING' in result.stdout and 'CERES_SIM: TELEOP' not in result.stdout
    with recovery_telemetry.open() as stream:
        rows = list(csv.DictReader(stream))
    assert rows and all(r['left_accepted'] == r['right_accepted'] == '0' for r in rows)
    assert all(r['hold_reason'] == r['left_hold_reason'] == r['right_hold_reason'] == 'none' for r in rows)
    assert all(r['simulation_phase'] == '0' for r in rows)
    with recovery_joints.open() as stream:
        joints = list(csv.DictReader(stream))
    assert len(joints) > 3
    for row in joints[2:]:
        for side in ('left', 'right'):
            assert row[side+'_ruckig_output'] == '1'
            assert row[side+'_reference_acceleration_valid'] == '1'
            assert row[side+'_reference_jerk_valid'] == '1'
            for j in range(1, 8):
                for metric in ('qdot', 'qddot', 'jerk'):
                    assert abs(float(row[f'{side}_j{j}_reference_{metric}'])) < 1e-10
print('ceres_viewer_startup: disabled/export/feedback rejected; DLS/Ceres hands receive but hold before S')
