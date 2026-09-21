"""Offline launch safety checks: no tmux servers, ROS nodes or hardware."""
import json
import os
from pathlib import Path
import runpy
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
READY = runpy.run_path(str(SCRIPTS / "check_pico_session_ready.py"))


def payload(sent=10, stamp=100, epoch=1, errors=0):
    return json.dumps(dict(packets_sent=sent, source_stamp_ns=stamp,
                           tracking_epoch=epoch, send_errors=errors))


def test_readiness_requires_two_advancing_valid_samples():
    tracker = READY["BridgeProgress"]()
    tracker.observe(payload())
    assert not tracker.ready
    tracker.observe(payload(11, 101))
    assert tracker.ready


def test_viewer_readiness_requires_matching_fresh_render_heartbeat():
    tracker = READY["ViewerProgress"]("%12")
    tracker.observe(json.dumps(dict(pane="%12", stamp_ns=100)))
    assert not tracker.ready(100)
    tracker.observe(json.dumps(dict(pane="%12", stamp_ns=200)))
    assert tracker.ready(200)
    assert not tracker.ready(199)
    assert not tracker.ready(1_000_000_200)
    tracker.observe(json.dumps(dict(pane="%13", stamp_ns=300)))
    assert not tracker.ready(300)
    tracker.observe("null")
    assert not tracker.ready(300)


@pytest.mark.parametrize("sync_fails", [False, True])
def test_skeleton_heartbeat_is_only_sent_after_viewer_sync(monkeypatch, sync_fails):
    import sys
    import types
    from contextlib import nullcontext
    module = runpy.run_path(str(SCRIPTS / "smpl_mujoco_visualizer.py"))
    events = []
    loops = iter([True, False])
    def sync():
        events.append("sync")
        if sync_fails:
            raise RuntimeError("render failed")
    viewer = types.SimpleNamespace(is_running=lambda: next(loops), lock=nullcontext, sync=sync)
    mujoco = types.ModuleType("mujoco")
    mujoco.viewer = types.ModuleType("mujoco.viewer")
    mujoco.viewer.launch_passive = lambda *args: nullcontext(viewer)
    monkeypatch.setitem(sys.modules, "mujoco", mujoco)
    monkeypatch.setitem(sys.modules, "mujoco.viewer", mujoco.viewer)
    msg = types.ModuleType("std_msgs.msg")
    msg.String = types.SimpleNamespace
    monkeypatch.setitem(sys.modules, "std_msgs.msg", msg)
    monkeypatch.setenv("TMUX_PANE", "%test")
    obj = module["SmplMujocoVisualizer"].__new__(module["SmplMujocoVisualizer"])
    obj.model = obj.data = None
    obj.rate = 1000
    obj._update_viewer_scene = lambda v: False
    obj._update_hand_overlay = lambda v: None
    def publish(message):
        assert events[-1] == "sync"
        assert json.loads(message.data)["pane"] == "%test"
        events.append("heartbeat")
    obj.node = types.SimpleNamespace(create_publisher=lambda *args: types.SimpleNamespace(publish=publish))
    obj.rclpy = types.SimpleNamespace(spin=lambda n: None, ok=lambda: True, shutdown=lambda: None)
    obj.external_shutdown_exception = obj.rcl_error = RuntimeError
    if sync_fails:
        with pytest.raises(RuntimeError, match="render failed"):
            obj.run()
        assert events == ["sync"]
    else:
        obj.run()
        assert events == ["sync", "heartbeat"]


@pytest.mark.parametrize("sample", [payload(), payload(11, 100), payload(9, 101),
    payload(11, 101, epoch=2), payload(11, 101, errors=1), "{}", "null", "broken"])
def test_readiness_rejects_stale_reset_error_and_bad_json(sample):
    tracker = READY["BridgeProgress"]()
    tracker.observe(payload())
    tracker.observe(sample)
    assert not tracker.ready


@pytest.mark.parametrize("owner,panes,valid", [
    ("/checkout", "driver:0\nm0:0\nbridge:0", True),
    ("/other", "driver:0\nm0:0\nbridge:0", False),
    ("/checkout", "driver:0\nm0:1\nbridge:0", False),
    ("/checkout", "driver:0\nm0:0", False),
])
def test_window_health(owner, panes, valid):
    def run(cmd, **kwargs):
        return subprocess.CompletedProcess(cmd, 0, owner if cmd[1] == "show-options" else panes, "")
    if valid:
        READY["check_panes"]("session", "/checkout", run)
    else:
        with pytest.raises(RuntimeError):
            READY["check_panes"]("session", "/checkout", run)


def test_conflict_inspection_never_signals(tmp_path, monkeypatch):
    cleaner = runpy.run_path(str(SCRIPTS / "cleanup_tianji_pico_processes.py"))
    proc = tmp_path / "987654"
    proc.mkdir()
    (proc / "cmdline").write_bytes(b"/other/checkout/pico_bridge_node\0")
    monkeypatch.setattr(os, "kill", lambda *args: pytest.fail("must not signal any process"))
    assert cleaner["cleanup_processes"](tmp_path) == 2
    assert cleaner["cleanup_processes"](tmp_path, dry_run=True) == 0


@pytest.mark.parametrize("ready_rc", [0, 2])
def test_launcher_propagates_readiness_failure_without_cleanup(tmp_path, ready_rc):
    import shutil
    scripts = tmp_path / "bash"
    scripts.mkdir()
    (tmp_path / "install/default").mkdir(parents=True)
    (tmp_path / "install/default/local_setup.bash").touch()
    shutil.copy(ROOT / "bash/start_tianji_pico_teleop.sh", scripts)
    binaries = tmp_path / "bin"
    binaries.mkdir()
    def executable(name, body):
        path = binaries / name
        path.write_text("#!/bin/bash\n" + body)
        path.chmod(0o755)
        return path
    executable("tmux", '''echo "$*" >> "$TEST_LOG"
case "$1" in
  has-session) exit 1 ;;
  kill-session) exit 99 ;;
  capture-pane) echo 'readiness failed: missing messages: /pico/smpl_raw' ;;
esac
''')
    executable("adb", 'echo device\n')
    python = executable("fake-python", '''case "$1" in
  */cleanup_tianji_pico_processes.py) exit 0 ;;
  */check_pico_session_ready.py) exit "$READY_RC" ;;
  *) exit 98 ;;
esac
''')
    (scripts / "environment.sh").write_text('ros_setup=/unused/setup.bash\n')
    log = tmp_path / "calls"
    result = subprocess.run(["bash", str(scripts / "start_tianji_pico_teleop.sh"), "--detach"],
        env={**os.environ, "PATH": str(binaries) + ":" + os.environ["PATH"],
             "TIANJI_PYTHON": str(python), "READY_RC": str(ready_rc), "TEST_LOG": str(log),
             "TMPDIR": str(tmp_path), "TIANJI_ENVIRONMENT": "default"},
        capture_output=True, text=True, timeout=5)
    assert result.returncode == ready_rc, result.stderr
    assert ("Started Tianji PICO" in result.stdout) == (ready_rc == 0)
    calls = log.read_text().splitlines()
    assert "kill-session" not in log.read_text()
    for window in ("driver", "m0", "bridge"):
        retain = f"set-option -w -t pico_tianji_teleop:{window} remain-on-exit on"
        assert retain in calls
        send = next(i for i, call in enumerate(calls)
                    if call.startswith(f"send-keys -t pico_tianji_teleop:{window} "))
        assert calls.index(retain) < send
        capture = f"capture-pane -p -t pico_tianji_teleop:{window} -S -200"
        assert (capture in calls) == (ready_rc != 0)
    assert ("PICO startup diagnostic logs:" in result.stderr) == (ready_rc != 0)
    if ready_rc:
        assert "missing messages: /pico/smpl_raw" in result.stderr
        diagnostic_dirs = list(tmp_path.glob("pico-startup.*"))
        assert len(diagnostic_dirs) == 1
        assert "missing messages: /pico/smpl_raw" in (diagnostic_dirs[0] / "m0.log").read_text()


@pytest.mark.parametrize("owned", [True, False])
def test_stop_requires_matching_checkout(tmp_path, owned):
    import shutil
    scripts = tmp_path / "bash"
    scripts.mkdir()
    shutil.copy(ROOT / "bash/start_tianji_pico_teleop.sh", scripts)
    tmux = tmp_path / "tmux"
    tmux.write_text('''#!/bin/bash
case "$1" in
  has-session) exit 0 ;;
  show-options) echo "$TEST_OWNER" ;;
  kill-session) echo stopped > "$TEST_STOP" ;;
esac
''')
    tmux.chmod(0o755)
    marker = tmp_path / "stop-marker"
    result = subprocess.run(["bash", str(scripts / "start_tianji_pico_teleop.sh"), "--stop"],
        env={**os.environ, "PATH": str(tmp_path) + ":" + os.environ["PATH"],
             "TEST_OWNER": str(tmp_path) if owned else "/other", "TEST_STOP": str(marker)},
        capture_output=True, text=True, timeout=5)
    assert result.returncode == (0 if owned else 2), result.stderr
    assert marker.exists() == owned
