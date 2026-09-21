"""Standalone wizard tests; no personal profiles, ROS nodes, SDK or devices."""
from pathlib import Path
import subprocess
import sys
import signal

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts import setup_pico as wizard


def scenario(tmp_path, *, published=True, collect_result=0, startup_result=0, fail_ready=False):
    events = []
    def popen(command, **kwargs):
        assert kwargs["start_new_session"] is True
        assert kwargs["stdin"] == subprocess.DEVNULL
        assert command[-1].endswith("start_pico_driver.sh")
        events.append("driver")
        return object()
    def ready(driver, timeout):
        events.append("ready")
        if fail_ready:
            raise RuntimeError("no input")
    def collect(user, height, **kwargs):
        events.append("calibrate")
        if published and not collect_result:
            kwargs["on_publish"](tmp_path / "profiles/person/pico-simple/cal-exact")
        return collect_result
    def run(command, **kwargs):
        events.append("skeleton")
        assert "--detach" in command
        assert command[command.index("--calibration-dir") + 1].endswith("cal-exact")
        assert not any("teleop.sh --sim" in arg for arg in command)
        return subprocess.CompletedProcess(command, startup_result)
    options = dict(root=tmp_path, check=lambda _: events.append("preflight"), collect=collect,
        ready=ready, stop=lambda _: events.append("stop"), popen=popen, run=run)
    return events, options


def test_publish_stops_driver_before_exact_revision_skeleton(tmp_path):
    events, options = scenario(tmp_path)
    assert wizard.setup("person", 1.7, **options) == 0
    assert events == ["preflight", "driver", "ready", "calibrate", "stop", "skeleton"]


@pytest.mark.parametrize("published,code", [(False, 0), (True, 7)])
def test_cancel_or_failure_never_opens_old_profile(tmp_path, published, code):
    events, options = scenario(tmp_path, published=published, collect_result=code)
    assert wizard.setup("person", 1.7, **options) == code
    assert events[-1] == "stop"
    assert "skeleton" not in events


def test_timeout_stops_owned_driver_without_calibration(tmp_path):
    events, options = scenario(tmp_path, fail_ready=True)
    with pytest.raises(RuntimeError, match="no input"):
        wizard.setup("person", 1.7, **options)
    assert events == ["preflight", "driver", "ready", "stop"]


def test_interrupt_during_calibration_stops_driver(tmp_path):
    events, options = scenario(tmp_path)
    def interrupt(*args, **kwargs):
        raise KeyboardInterrupt
    options["collect"] = interrupt
    with pytest.raises(KeyboardInterrupt):
        wizard.setup("person", 1.7, **options)
    assert events[-1] == "stop"
    assert "skeleton" not in events


def test_cleanup_failure_blocks_second_input(tmp_path):
    events, options = scenario(tmp_path)
    def fail(_):
        raise RuntimeError("owned group remains")
    options["stop"] = fail
    with pytest.raises(RuntimeError, match="owned group"):
        wizard.setup("person", 1.7, **options)
    assert "skeleton" not in events


def test_skeleton_failure_is_not_reported_as_success(tmp_path):
    events, options = scenario(tmp_path, startup_result=2)
    assert wizard.setup("person", 1.7, **options) == 2
    assert events[-2:] == ["stop", "skeleton"]


@pytest.mark.parametrize("user,height,timeout", [("../bad",1.7,30), ("a",170,30),
    ("a",float("nan"),30), ("a",1.7,0), ("a",1.7,float("inf"))])
def test_invalid_input_cannot_start_driver(tmp_path, user, height, timeout):
    events, options = scenario(tmp_path)
    with pytest.raises(ValueError):
        wizard.setup(user, height, timeout, **options)
    assert events == []


def test_preflight_conflict_does_not_start_driver(tmp_path):
    events, options = scenario(tmp_path)
    def conflict(_):
        raise RuntimeError("existing input")
    options["check"] = conflict
    with pytest.raises(RuntimeError, match="existing input"):
        wizard.setup("person", 1.7, **options)
    assert events == []


def test_owned_stop_only_signals_saved_process_group(monkeypatch):
    calls = []
    class Driver:
        pid = 987654
        def poll(self): return 0
        def wait(self, **kwargs): return 0
    def killpg(pid, sig):
        calls.append((pid, sig))
        if sig == 0:
            raise ProcessLookupError
    monkeypatch.setattr(wizard.os, "killpg", killpg)
    wizard.stop_owned_driver(Driver())
    assert calls == [(987654, signal.SIGINT), (987654, 0)]


def test_help_without_devices():
    result = subprocess.run([sys.executable, str(ROOT / "scripts/setup_pico.py"), "--help"],
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0
    assert "--height-m" in result.stdout


@pytest.mark.parametrize("answers,published", [([""], True), (["q"], False),
    ([" Q "], False), (["wrong", ""], True), (["wrong", "q"], False)])
def test_wizard_confirmation_controls_publication_and_skeleton(tmp_path, answers, published):
    events, options = scenario(tmp_path)
    prompts = []
    inputs = iter(answers)
    def read(prompt):
        prompts.append(prompt)
        return next(inputs)
    def collect(user, height, **kwargs):
        events.append("results displayed")
        if kwargs["confirm"]("legacy publish prompt") == "publish":
            events.append("published")
            kwargs["on_publish"](tmp_path / "profiles/person/pico-simple/cal-exact")
        return 0
    options.update(collect=collect, confirm=read)
    assert wizard.setup("person", 1.7, **options) == 0
    assert prompts == ["回车确认使用并显示骨架，输入 q 取消："] * len(answers)
    assert ("published" in events) == published
    assert ("skeleton" in events) == published
    if published:
        assert events.index("results displayed") < events.index("published") < events.index("stop") < events.index("skeleton")


@pytest.mark.parametrize("error", [EOFError, KeyboardInterrupt])
def test_confirmation_interruption_does_not_publish(tmp_path, error):
    events, options = scenario(tmp_path)
    def read(_):
        raise error
    def collect(user, height, **kwargs):
        kwargs["confirm"]("legacy prompt")
        pytest.fail("interruption must not reach publication")
    options.update(collect=collect, confirm=read)
    with pytest.raises(error):
        wizard.setup("person", 1.7, **options)
    assert events[-1] == "stop"
    assert "skeleton" not in events


@pytest.mark.parametrize("answer,code,published", [("publish",0,True), ("cancel",0,False), ("publish",7,False)])
def test_actual_lifecycle_notifies_only_after_publish(tmp_path, monkeypatch, answer, code, published):
    from scripts import calibrate_pico_simple as lifecycle
    import json
    def bundle(path, *args, **kwargs):
        for filename in (lifecycle.MANIFEST, "pico_right_palm_tcp.yaml",
                         "pico_right_wrist_pivot.yaml", "pico_left_arm_geometry.yaml"):
            (path / filename).write_text("{}")
    # Exercise publication orchestration without fabricating a measured calibration.
    monkeypatch.setattr(lifecycle, "build_bundle", bundle)
    monkeypatch.setattr(lifecycle, "_validate_pico_revision", lambda _: None)
    calls = []
    assert lifecycle.calibrate("person", 1.7, tmp_path, runner=lambda *_: code,
        confirm=lambda _: answer, on_publish=calls.append) == code
    pointer = tmp_path / "profiles/person/pico-simple/active.json"
    assert pointer.exists() == published
    assert bool(calls) == published
    if published:
        assert calls == [(pointer.parent / json.loads(pointer.read_text())["active_set"]).resolve()]
