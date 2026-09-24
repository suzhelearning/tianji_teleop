"""Offline launch safety checks: no tmux servers, ROS nodes or hardware."""
import json
import os
from pathlib import Path
import runpy
import subprocess
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
READY = runpy.run_path(str(SCRIPTS / "check_pico_session_ready.py"))


BOOT_ID = "01234567-89ab-4cde-8123-456789abcdef"
SESSION_ID = "abcdef01-2345-4678-9abc-def012345678"
VIEWER_OWNER = "0123456789ab4cde8123456789abcdef"


def frame(sequence=10, stamp=100, epoch=1, published=1000, **overrides):
    values = dict(sequence=sequence, source_timestamp_ns=stamp, tracking_epoch=epoch,
                  published_monotonic_ns=published, boot_id=BOOT_ID,
                  session_id=SESSION_ID, revocation_generation=0, valid=True)
    values.update(overrides)
    return SimpleNamespace(**values)


def test_readiness_requires_two_advancing_live_frames():
    tracker = READY["ArmInputProgress"](BOOT_ID)
    tracker.observe(frame(), 1000)
    assert not tracker.ready(1000)
    tracker.observe(frame(11, 101, published=1001), 1001)
    assert tracker.ready(1001)
    assert not tracker.ready(100_001_001)


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

def test_foreground_viewer_rejects_foreign_stale_and_replayed_heartbeats():
    tracker = READY["ViewerProgress"](owner=VIEWER_OWNER)
    def observe(stamp, owner=VIEWER_OWNER):
        tracker.observe(json.dumps(dict(owner=owner, pane="", stamp_ns=stamp)))
    observe(100)
    assert not tracker.ready(100)
    observe(200)
    assert tracker.ready(200)
    assert not tracker.ready(199)
    assert not tracker.ready(1_000_000_200)
    observe(300, "abcdef01234546789abcdef012345678")
    assert not tracker.ready(300)
    observe(100)
    assert not tracker.ready(300)
    observe(200)
    assert not tracker.ready(300)
    observe(400)
    assert tracker.ready(400)
    tracker.observe(json.dumps(dict(pane="%12", stamp_ns=500)))
    assert not tracker.ready(500)


@pytest.mark.parametrize("arguments", [
    ["--viewer-owner", ""],
    ["--viewer-owner", "not-a-uuid"],
    ["--viewer-owner", BOOT_ID],
    ["--viewer-owner", VIEWER_OWNER, "--session", "pico", "--checkout", "/checkout"],
    ["--session", "pico"],
    *[["--viewer-owner", VIEWER_OWNER, "--timeout-s", value] for value in ("0", "-1", "nan", "inf")],
])
def test_readiness_rejects_invalid_ownership_and_deadlines(monkeypatch, arguments):
    monkeypatch.setattr(sys, "argv", ["check_pico_session_ready.py", *arguments])
    with pytest.raises(SystemExit) as exit_info:
        READY["main"]()
    assert exit_info.value.code == 2


@pytest.mark.parametrize("arm_publishers,viewer_publishers", [(1, 1), (2, 1), (1, 2)])
def test_foreground_wait_requires_unique_advancing_streams_without_tmux(
        monkeypatch, tmp_path, arm_publishers, viewer_publishers):
    subscriptions = {}
    state = dict(step=0, destroyed=False, shutdown=False)
    node = SimpleNamespace(
        create_subscription=lambda msg_type, topic, callback, qos: subscriptions.update({topic: callback}),
        count_publishers=lambda topic: arm_publishers if topic == "/arm" else viewer_publishers,
        destroy_node=lambda: state.update(destroyed=True),
    )
    now_ns = lambda: 1_000_000_000 + state["step"] * 10_000_000
    boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    def spin_once(node, timeout_sec):
        state["step"] += 1
        assert state["step"] <= 2
        subscriptions["/arm"](frame(sequence=state["step"], stamp=state["step"],
                                   published=now_ns(), boot_id=boot_id))
        subscriptions["/pico/skeleton_viewer/status"](SimpleNamespace(
            data=json.dumps(dict(owner=VIEWER_OWNER, stamp_ns=now_ns()))))
    monkeypatch.setitem(sys.modules, "rclpy", SimpleNamespace(
        init=lambda: None, create_node=lambda name: node, spin_once=spin_once,
        shutdown=lambda: state.update(shutdown=True)))
    monkeypatch.setitem(sys.modules, "rclpy.qos", SimpleNamespace(
        QoSProfile=lambda **kwargs: None, ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=1),
        DurabilityPolicy=SimpleNamespace(VOLATILE=1)))
    monkeypatch.setitem(sys.modules, "std_msgs.msg", SimpleNamespace(String=SimpleNamespace))
    monkeypatch.setitem(sys.modules, "tianji_interfaces.msg", SimpleNamespace(PicoArmInput=SimpleNamespace))
    config = tmp_path / "robot.json"
    config.write_text(json.dumps(dict(pico_input_topic="/arm")))
    monkeypatch.setitem(sys.modules, "tianji_runtime.resources", SimpleNamespace(config_path=lambda name: config))
    monkeypatch.setitem(READY["wait_ready"].__globals__, "time",
                        SimpleNamespace(monotonic=lambda: now_ns() / 1e9, monotonic_ns=now_ns))
    monkeypatch.setattr(subprocess, "run", lambda *args, **kwargs: pytest.fail("foreground must not query tmux"))
    monkeypatch.setattr(sys, "argv", ["check_pico_session_ready.py", "--viewer-owner", VIEWER_OWNER])
    if arm_publishers == viewer_publishers == 1:
        READY["main"]()
    else:
        with pytest.raises(SystemExit) as exit_info:
            READY["main"]()
        assert exit_info.value.code == 2
    assert state == dict(step=2, destroyed=True, shutdown=True)



@pytest.mark.parametrize("sync_fails", [False, True])
@pytest.mark.parametrize("foreground", [False, True])
def test_skeleton_heartbeat_is_only_sent_after_viewer_sync(monkeypatch, sync_fails, foreground):
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
    monkeypatch.setenv("TIANJI_PICO_SESSION_OWNER", VIEWER_OWNER)
    monkeypatch.setattr(module["time"], "monotonic_ns", lambda: 200)
    tracker = (READY["ViewerProgress"](owner=VIEWER_OWNER) if foreground
               else READY["ViewerProgress"]("%test"))
    tracker.observe(json.dumps(dict(owner=VIEWER_OWNER, pane="%test", stamp_ns=100)))
    obj = module["SmplMujocoVisualizer"].__new__(module["SmplMujocoVisualizer"])
    obj.model = obj.data = None
    obj.rate = 1000
    obj._update_viewer_scene = lambda v: False
    obj._update_hand_overlay = lambda v: None
    def publish(message):
        assert events[-1] == "sync"
        tracker.observe(message.data)
        assert tracker.ready(200)
        events.append("heartbeat")
    obj.node = types.SimpleNamespace(create_publisher=lambda *args: types.SimpleNamespace(publish=publish))
    obj.rclpy = types.SimpleNamespace(spin=lambda n: None, ok=lambda: True, shutdown=lambda: None)
    obj.external_shutdown_exception = obj.rcl_error = RuntimeError
    if sync_fails:
        with pytest.raises(RuntimeError, match="render failed"):
            obj.run()
        assert events == ["sync"]
        assert not tracker.ready(200)
    else:
        obj.run()
        assert events == ["sync", "heartbeat"]


@pytest.mark.parametrize("sample", [
    frame(), frame(11, 100, published=1001), frame(9, 101, published=1001),
    frame(11, 101, epoch=2, published=1001), frame(11, 101, valid=False),
    frame(11, 101, boot_id="another-boot"), frame(11, 101, session_id="bad"),
    frame(11, 101, session_id=BOOT_ID, published=1001),
    frame(11, 101, published=1002), SimpleNamespace(),
    frame(11, 101, published=1001, revocation_generation=1),
])
def test_readiness_rejects_stale_reset_invalid_and_foreign_frames(sample):
    tracker = READY["ArmInputProgress"](BOOT_ID)
    tracker.observe(frame(), 1000)
    tracker.observe(sample, 1001)
    assert not tracker.ready(1001)


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
    run_log_dir = tmp_path / "tmp/logs/pico/current-run"
    run_log_dir.mkdir(parents=True)
    result = subprocess.run(["bash", str(scripts / "start_tianji_pico_teleop.sh"), "--detach"],
        env={**os.environ, "PATH": str(binaries) + ":" + os.environ["PATH"],
             "TIANJI_PYTHON": str(python), "READY_RC": str(ready_rc), "TEST_LOG": str(log),
             "TMPDIR": str(tmp_path), "TIANJI_ENVIRONMENT": "default",
             "TIANJI_RUN_LOG_DIR": str(run_log_dir)},
        capture_output=True, text=True, timeout=5)
    assert result.returncode == ready_rc, result.stderr
    calls = log.read_text().splitlines()
    assert "kill-session" not in log.read_text()
    for window in ("driver", "m0", "bridge"):
        capture = f"capture-pane -p -t pico_tianji_teleop:{window} -S -200"
        assert (capture in calls) == (ready_rc != 0)
    if ready_rc:
        assert "missing messages: /pico/smpl_raw" in result.stderr
        diagnostic_dirs = list(run_log_dir.glob("startup-failure.*"))
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
