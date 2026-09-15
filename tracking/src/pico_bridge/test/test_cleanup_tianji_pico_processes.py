import importlib.util
from pathlib import Path


REPO_ROOT = Path(__file__).parents[3]
CLEANUP_SCRIPT = REPO_ROOT / "scripts" / "cleanup_tianji_pico_processes.py"


def _load_cleanup_module():
    spec = importlib.util.spec_from_file_location("cleanup_tianji_pico_processes", CLEANUP_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _write_cmdline(proc_root: Path, pid: int, *arguments: str) -> None:
    process_dir = proc_root / str(pid)
    process_dir.mkdir(parents=True)
    process_dir.joinpath("cmdline").write_bytes(
        b"\0".join(argument.encode() for argument in arguments) + b"\0"
    )


def test_finds_exact_driver_m0_and_bridge_targets_across_paths_and_domains(tmp_path):
    module = _load_cleanup_module()
    proc_root = tmp_path / "proc"

    targets = {
        101: ("python", "/old/env/ros2", "launch", "pico_bridge", "start_pico_bridge.launch.py"),
        102: ("/old/install/pico_bridge/lib/pico_bridge/pico_bridge_node", "--ros-args"),
        103: ("/old/install/pico_bridge/lib/pico_bridge/pico_smpl_ground", "--ros-args"),
        201: ("python", "/v2/env/ros2", "launch", "pico_bridge", "start_pico_palm_skeleton_filter.launch.py"),
        202: ("python3", "/v2/install/pico_bridge/lib/pico_bridge/pico_palm_tcp_publisher", "--side", "left"),
        203: ("python3", "/v2/install/pico_bridge/lib/pico_bridge/pico_palm_skeleton_filter", "--ros-args"),
        204: ("python3", "/v2/install/pico_bridge/lib/pico_bridge/smpl_mujoco_visualizer", "--topic", "/pico/smpl_raw"),
        301: ("python", "/domain99/env/ros2", "launch", "pico_bridge", "start_tianji_mujoco_teleop.launch.py"),
        302: ("/domain99/install/pico_bridge/lib/pico_bridge/tianji_mujoco_teleop_bridge", "--ros-args"),
    }
    for pid, arguments in targets.items():
        _write_cmdline(proc_root, pid, *arguments)

    found = module.find_target_processes(proc_root, excluded_pids=set())

    assert {process.pid for process in found} == set(targets)


def test_preserves_unrelated_ros_python_apk_and_similarly_named_processes(tmp_path):
    module = _load_cleanup_module()
    proc_root = tmp_path / "proc"
    unrelated = {
        401: ("python", "/opt/ros/bin/ros2", "launch", "another_pkg", "start_pico_bridge.launch.py"),
        402: ("python", "/opt/ros/bin/ros2", "launch", "pico_bridge", "other.launch.py"),
        403: ("/tmp/pico_bridge_node_debug",),
        404: ("python3", "/tmp/smpl_mujoco_visualizer_backup.py"),
        405: ("com.PICO.wholebody_stream.unity",),
        406: ("adb", "forward", "tcp:9999", "tcp:9999"),
        407: ("python", "worker.py"),
        408: ("cat", "/tmp/pico_bridge_node"),
    }
    for pid, arguments in unrelated.items():
        _write_cmdline(proc_root, pid, *arguments)

    assert module.find_target_processes(proc_root, excluded_pids=set()) == []


def test_excludes_cleanup_process_and_ancestors_even_if_they_match(tmp_path):
    module = _load_cleanup_module()
    proc_root = tmp_path / "proc"
    _write_cmdline(proc_root, 501, "/path/pico_bridge_node")
    _write_cmdline(proc_root, 502, "/path/tianji_mujoco_teleop_bridge")

    found = module.find_target_processes(proc_root, excluded_pids={501})

    assert {process.pid for process in found} == {502}
