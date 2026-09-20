"""Opt-in startup and no-input safety, loopback only, no device traffic."""
import csv
from pathlib import Path
import socket
import subprocess
import sys
import tempfile

import yaml

viewer = Path(sys.argv[1]).resolve()
control = Path(__file__).resolve().parents[1]
profile = control / 'config/qp_ik_pico_shared_root_dls.yaml'


def run(config, *extra):
    return subprocess.run([str(viewer), '--config', str(config), '--headless',
                           '--duration', '3', '--pico-teleop', *extra],
                          cwd=control, capture_output=True, text=True, timeout=30)


# A shipped disabled profile must never silently run the legacy mapper.
result = run(profile)
assert result.returncode != 0 and 'requires enabled shared-root' in result.stderr, result
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
    assert result.returncode != 0 and 'requires --simulation-recovery' in result.stderr, result
    result = run(trial, '--joint-command-port', '26999')
    assert result.returncode != 0 and 'forbids joint command export' in result.stderr, result
    config['controller']['model_state_only'] = False
    feedback = folder/'feedback.yaml'
    feedback.write_text(yaml.safe_dump(config, sort_keys=False))
    result = run(feedback)
    assert result.returncode != 0 and 'model-only' in result.stderr, result
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
    assert 'WAITING' in result.stdout and 'TELEOP' not in result.stdout
    assert 'DLS_SIM: WAITING' in result.stdout
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
    assert 'pose jump rejection DISABLED' in result.stderr, result
    assert 'DLS_SIM: WAITING' in result.stdout and 'DLS_SIM: TELEOP' not in result.stdout
print('franka_dls_viewer_startup: disabled/export/feedback rejected; interactive recovery waits with Ruckig plots')
