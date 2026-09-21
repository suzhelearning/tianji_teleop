"""Exercise installed PICO entrypoints without opening any input devices."""
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
import pytest
import rclpy
import yaml


@pytest.mark.parametrize('entrypoint', ['launch', 'executable'])
@pytest.mark.parametrize('config_name', [None, 'collect_config_pico.yaml'])
def test_installed_entrypoint_loads_configured_streams_without_tty(tmp_path, entrypoint, config_name):
    share = Path(get_package_share_directory('pico_recorder'))
    config_path = share / 'config' / (config_name or 'collect_config.yaml')
    config = yaml.safe_load(config_path.read_text())
    if entrypoint == 'launch':
        command = ['ros2', 'launch', 'pico_recorder', 'pico_recorder.launch.py']
        if config_name:
            command.append(f'config_path:={config_path}')
        node_name = 'pico_recorder_node'
    else:
        command = ['ros2', 'run', 'pico_recorder', 'pico_recorder_node']
        if config_name:
            command.extend(['--config', str(config_path)])
        node_name = 'pico_recorder'
    expected_topics = {dataset['topic'] for dataset in config['datasets'].values()}
    expected_topics.add('/pico/record_flag')
    # Use the runner's dedicated test ROS domain; these nodes only subscribe.
    rclpy.init(args=[])
    probe = rclpy.create_node(f'pico_recorder_test_{os.getpid()}')
    process = None
    log_path = tmp_path / 'recorder.log'
    try:
        with log_path.open('w') as log:
            process = subprocess.Popen(
                command, cwd=tmp_path, stdin=subprocess.DEVNULL, stdout=log,
                stderr=subprocess.STDOUT, start_new_session=True,
            )
            deadline = time.monotonic() + 20.0
            observed = set()
            while time.monotonic() < deadline:
                assert process.poll() is None, log_path.read_text()
                if (node_name, '/') in probe.get_node_names_and_namespaces():
                    observed = {
                        topic for topic, _ in probe.get_subscriber_names_and_types_by_node(node_name, '/')
                    }
                    if expected_topics <= observed:
                        break
                rclpy.spin_once(probe, timeout_sec=0.05)
            assert expected_topics <= observed, log_path.read_text()
            if config_name:
                assert '/raw/odom/odin' not in observed
            assert process.poll() is None, log_path.read_text()
    finally:
        if process is not None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        probe.destroy_node()
        rclpy.try_shutdown()
