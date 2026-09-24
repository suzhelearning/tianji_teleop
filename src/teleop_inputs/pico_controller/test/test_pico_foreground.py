"""Real local subprocess lifecycle checks; no ROS, tmux, network or hardware."""
from contextlib import contextmanager
import json
import os
from pathlib import Path
import runpy
import selectors
import shlex
import signal
import subprocess
import sys
import time

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "pico_foreground.py"


def python_command(code):
    return shlex.join([sys.executable, "-u", "-c", code])


def wait_until(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.01)
    raise AssertionError("Timed out waiting for local child process")


def process_running(pid):
    try:
        return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0] != "Z"
    except FileNotFoundError:
        return False


@contextmanager
def launch(tmp_path, *, ready_code="print('checker accepted', flush=True)",
           overrides=None, lock_path=None, interrupt_grace=0.5):
    tmp_path.mkdir(exist_ok=True)
    commands = {}
    for name in ("driver", "m0", "bridge"):
        commands[name] = python_command(
            "import os,time; from pathlib import Path; "
            f"Path({str(tmp_path / (name + '.pid'))!r}).write_text(str(os.getpid())); "
            f"print({name!r} + ' tracking alive', flush=True); time.sleep(60)"
        )
    commands.update(overrides or {})
    config = {
        "checkout": str(tmp_path), "log_dir": str(tmp_path), "commands": commands,
        "readiness_command": [sys.executable, "-u", "-c", ready_code],
        "lock_path": str(lock_path or tmp_path / "launch.lock"),
        "interrupt_grace_s": interrupt_grace, "terminate_grace_s": 0.2,
    }
    entry = tmp_path / "launch.py"
    entry.write_text(
        "import runpy\n"
        f"module = runpy.run_path({str(SCRIPT)!r})\n"
        f"raise SystemExit(module['run_foreground'](**{config!r}))\n"
    )
    process = subprocess.Popen(
        [sys.executable, str(entry)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    try:
        yield process
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGTERM)
        try:
            if process.stdout.closed:
                process.wait(timeout=8)
            else:
                process.communicate(timeout=8)
        except subprocess.TimeoutExpired:
            process.kill()
            if process.stdout.closed:
                process.wait(timeout=2)
            else:
                process.communicate(timeout=2)
            # Failure isolation: only groups whose PIDs this test recorded.
            for pid_file in tmp_path.glob("*.pid"):
                try:
                    os.killpg(int(pid_file.read_text()), signal.SIGKILL)
                except ProcessLookupError:
                    pass


def read_until(process, needle, timeout=5):
    data = getattr(process, "_live_output", bytearray())
    process._live_output = data
    if needle in data:
        return bytes(data)
    with selectors.DefaultSelector() as selector:
        selector.register(process.stdout, selectors.EVENT_READ)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for _key, _event in selector.select(0.05):
                chunk = os.read(process.stdout.fileno(), 4096)
                if not chunk:
                    raise AssertionError(f"Supervisor exited before {needle!r}: {bytes(data)!r}")
                data.extend(chunk)
                if needle in data:
                    return bytes(data)
    raise AssertionError(f"Missing live output {needle!r}: {bytes(data)!r}")


def test_partial_output_and_logs_are_live_before_children_exit(tmp_path):
    marker = "Right controller A: Set Ground locked"
    command = python_command(
        "import os,sys,time,json; "
        "print(json.dumps({'owner':os.environ['TIANJI_PICO_SESSION_OWNER'],"
        "'tmux':os.environ.get('TMUX'),'pane':os.environ.get('TMUX_PANE')}),flush=True); "
        f"sys.stdout.write({marker!r}); sys.stdout.flush(); time.sleep(60)"
    )
    with launch(tmp_path, overrides={"driver": command},
                ready_code="import time; print('checking',flush=True); time.sleep(0.2)") as process:
        shown = read_until(process, marker.encode())
        assert b"[driver]" in shown
        assert process.poll() is None
        raw = (tmp_path / "driver.log").read_text()
        assert raw.endswith(marker)
        environment = json.loads(raw.splitlines()[0])
        assert len(environment["owner"]) == 32
        assert environment["tmux"] is None and environment["pane"] is None
        read_until(process, b"PICO operational")
        assert "checking" in (tmp_path / "readiness.log").read_text()


@pytest.mark.parametrize("ready_code", [
    "import sys; print('frames not advancing',flush=True); sys.exit(7)",
    "raise RuntimeError('readiness crashed')",
])
def test_readiness_failure_never_declares_operational(tmp_path, ready_code):
    with launch(tmp_path, ready_code=ready_code) as process:
        output, _ = process.communicate(timeout=5)
        assert process.returncode != 0
        assert b"PICO operational" not in output
        assert b"Readiness checker exited" in output
        assert (tmp_path / "readiness.log").read_bytes()
        for pid_file in tmp_path.glob("*.pid"):
            assert not process_running(int(pid_file.read_text()))


def test_successful_checker_cannot_mask_pipeline_exit(tmp_path):
    with launch(tmp_path, ready_code="import time; time.sleep(0.3)",
                overrides={"driver": "exit 0"}) as process:
        output, _ = process.communicate(timeout=5)
        assert process.returncode != 0
        assert b"driver exited with status 0" in output
        assert b"PICO operational" not in output


@pytest.mark.parametrize("stop_signal", [signal.SIGINT, signal.SIGTERM, signal.SIGHUP])
def test_signals_clean_owned_descendants_and_leave_sentinel(tmp_path, stop_signal):
    descendant = tmp_path / "descendant.pid"
    nested = tmp_path / "nested.pid"
    cleaned = tmp_path / "m0-cleaned"
    stubborn_code = (
        "import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); "
        "signal.signal(signal.SIGTERM,signal.SIG_IGN); print('descendant ready',flush=True); time.sleep(60)"
    )
    driver = python_command(
        "import os,signal,subprocess,time; from pathlib import Path; "
        f"p=subprocess.Popen({[sys.executable, '-u', '-c', stubborn_code]!r}); "
        f"Path({str(descendant)!r}).write_text(str(p.pid)); "
        "signal.signal(signal.SIGINT,lambda *_:exit(0)); time.sleep(60)"
    )
    m0_code = (
        "import os,signal,subprocess,time\nfrom pathlib import Path\n"
        f"p=subprocess.Popen({[sys.executable, '-u', '-c', stubborn_code]!r},start_new_session=True)\n"
        f"Path({str(nested)!r}).write_text(str(p.pid))\n"
        "def cleanup(*_):\n"
        "    time.sleep(0.2)\n"
        "    os.killpg(p.pid,signal.SIGKILL)\n"
        "    p.wait()\n"
        f"    Path({str(cleaned)!r}).touch()\n"
        "    print('m0 cleanup persisted',flush=True)\n"
        "    raise SystemExit(0)\n"
        "signal.signal(signal.SIGINT,cleanup)\n"
        "signal.signal(signal.SIGTERM,cleanup)\n"
        "print('wrapper ready',flush=True)\n"
        "time.sleep(60)\n"
    )
    sentinel = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"],
                                start_new_session=True)
    try:
        with launch(tmp_path, overrides={"driver": driver, "m0": python_command(m0_code)}) as process:
            read_until(process, b"wrapper ready")
            wait_until(lambda: descendant.exists() and nested.exists())
            # Both descendants announce readiness before ignoring soft signals.
            wait_until(lambda: b"descendant ready" in (tmp_path / "driver.log").read_bytes())
            wait_until(lambda: b"descendant ready" in (tmp_path / "skeleton-calibration-viewer.log").read_bytes())
            process.send_signal(stop_signal)
            process.communicate(timeout=5)
            assert process.returncode == 128 + stop_signal
            assert cleaned.exists()
            assert not process_running(int(descendant.read_text()))
            assert not process_running(int(nested.read_text()))
            assert sentinel.poll() is None
            assert "m0 cleanup persisted" in (tmp_path / "skeleton-calibration-viewer.log").read_text()
    finally:
        sentinel.terminate()
        sentinel.wait(timeout=2)
        for pid_file in (descendant, nested):
            if pid_file.exists():
                try:
                    os.kill(int(pid_file.read_text()), signal.SIGKILL)
                except ProcessLookupError:
                    pass


def test_broken_terminal_still_persists_shutdown_logs(tmp_path):
    driver = python_command(
        "import signal,time\n"
        "def cleanup(*_):\n"
        "    print('shutdown after terminal loss',flush=True)\n"
        "    raise SystemExit(0)\n"
        "signal.signal(signal.SIGINT,cleanup)\n"
        "print('writer ready',flush=True)\n"
        "while True:\n"
        "    print('tracking frame',flush=True)\n"
        "    time.sleep(0.02)\n"
    )
    with launch(tmp_path, overrides={"driver": driver}) as process:
        read_until(process, b"writer ready")
        process.stdout.close()
        process.wait(timeout=5)
        assert process.returncode != 0
        assert "shutdown after terminal loss" in (tmp_path / "driver.log").read_text()
        assert "Terminal output closed" in (tmp_path / "foreground.log").read_text()


def test_per_user_lock_rejects_second_launch_without_touching_first(tmp_path):
    lock = tmp_path / "shared.lock"
    with launch(tmp_path / "first", lock_path=lock) as first:
        read_until(first, b"PICO operational")
        with launch(tmp_path / "second", lock_path=lock) as second:
            output, _ = second.communicate(timeout=5)
            assert second.returncode != 0
            assert b"already running" in output
            assert not list((tmp_path / "second").glob("*.pid"))
            assert first.poll() is None


def test_discovery_returns_only_locked_launch_metadata(tmp_path):
    discover = runpy.run_path(str(SCRIPT))["discover_foreground"]
    lock = tmp_path / "launch.lock"
    assert discover(lock) is None
    with launch(tmp_path, lock_path=lock) as process:
        read_until(process, b"PICO operational")
        metadata = discover(lock)
        assert metadata["pid"] == process.pid
        assert metadata["checkout"] == str(tmp_path.resolve())
        assert metadata["calibration_dir"] is None
        assert len(metadata["owner"]) == 32
    assert lock.exists()
    assert discover(lock) is None


def test_split_utf8_is_displayed_live_without_corrupting_characters(tmp_path):
    resume = tmp_path / "resume-writer"
    payload = "地面锁定\n".encode("utf-8")
    driver = python_command(
        "import os,time\nfrom pathlib import Path\n"
        f"payload={payload!r}\n"
        "os.write(1,payload[:1])\n"
        f"while not Path({str(resume)!r}).exists():\n"
        "    time.sleep(0.01)\n"
        "os.write(1,payload[1:])\n"
        "time.sleep(60)\n"
    )
    with launch(tmp_path, overrides={"driver": driver}) as process:
        logfile = tmp_path / "driver.log"
        # The log proves the first byte reached the supervisor separately.
        wait_until(lambda: logfile.exists() and logfile.read_bytes() == payload[:1])
        resume.touch()
        shown = read_until(process, "[driver] 地面锁定".encode("utf-8"))
        assert "地面锁定" in shown.decode("utf-8")
        assert b"\xef\xbf\xbd" not in shown
        assert logfile.read_bytes() == payload
        assert process.poll() is None
