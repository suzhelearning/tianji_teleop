"""Run-logging contract for ``teleop.sh --real``: console capture, status, TTY, signals.

Every test drives ``real_robot/run_logging.py`` with a fake executor; no vendor SDK is
imported, no device is contacted and no real controller is started.
"""
from __future__ import annotations

import io
import json
import os
from pathlib import Path
import pty
import signal
import subprocess
import sys
import threading
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from real_robot.run_logging import ConsoleLog, create_run_directory

LAUNCHER = ROOT / "real_robot" / "run_logging.py"
CONSOLE_LOG = "console.log"
EXIT_RECORD = "launcher_exit.json"
TIMEOUT_S = 30.0

# Static fixture: every knob arrives through the environment, so the script itself is
# never reformatted by the test and stays readable in failure output.
FAKE_EXECUTOR = '''\
"""Fake real executor: same stdio/TTY/signal contract, no SDK import, no hardware."""
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

EXIT_CODE = int(os.environ.get("FAKE_EXIT_CODE", "0"))
CLEANUP_CODE = int(os.environ.get("FAKE_CLEANUP_CODE", "9"))
CLEANUP_DELAY_S = float(os.environ.get("FAKE_CLEANUP_DELAY_S", "0"))
WATCH_ENTER = os.environ.get("FAKE_WATCH_ENTER") == "1"
ARGV_FAILURE = os.environ.get("FAKE_ARGV_FAILURE") == "1"
BULK_LINES = int(os.environ.get("FAKE_BULK_LINES", "0"))
SELF_SIGNAL = os.environ.get("FAKE_SELF_SIGNAL", "")

WAIT_SIGNAL = os.environ.get("FAKE_WAIT_SIGNAL") == "1"

def note(text):
    print(text, flush=True)


def cleanup(signum, frame):
    # The real executor ignores repeated terminal interrupts while releasing hardware.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    note("FAKE_CLEANUP signal=" + str(signum))
    time.sleep(CLEANUP_DELAY_S)
    note("FAKE_CLEANUP_DONE")
    sys.exit(CLEANUP_CODE)


signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

if os.environ.get("FAKE_CBREAK") == "1":
    import tty
    tty.setcbreak(sys.stdin.fileno())
marker = os.environ.get("FAKE_MARKER")
if marker:
    Path(marker).write_text("executor started\\n")

if ARGV_FAILURE:
    print("usage: run_teleop.py [--devices DEVICES] [--confirm-real]")
    print("run_teleop.py: error: unrecognized arguments: --nonsense", file=sys.stderr, flush=True)
    sys.exit(2)

note("FAKE_START tag=" + os.environ.get("FAKE_TAG", "") +
     " tty=" + str(sys.stdin.isatty()) +
     " log=" + str(os.environ.get("TIANJI_RUN_LOG_DIR")))
print("FAKE_STDERR", file=sys.stderr, flush=True)
if SELF_SIGNAL:
    note("FAKE_SELF_SIGNAL " + SELF_SIGNAL)
    os.kill(os.getpid(), getattr(signal, "SIG" + SELF_SIGNAL))
subprocess.run([sys.executable, "-c", "print('FAKE_CHILD_OUTPUT', flush=True)"], check=True)
for line in range(BULK_LINES):
    print("FAKE_BULK " + str(line).ljust(52, "o"), flush=True)
    print("FAKE_BULK_ERR " + str(line).ljust(48, "e"), file=sys.stderr, flush=True)
if WATCH_ENTER:
    note("FAKE_WAITING_ENTER")
    deadline = time.monotonic() + 30.0
    while not select.select([sys.stdin.fileno()], [], [], 0.1)[0]:
        if time.monotonic() > deadline:
            note("FAKE_ENTER_TIMEOUT")
            sys.exit(8)
    os.read(sys.stdin.fileno(), 64)
    note("FAKE_ENTER_SEEN")
    note("FAKE_IDLE")
    time.sleep(30.0)  # stays armed until the test interrupts it
if WAIT_SIGNAL:
    note("FAKE_WAITING_SIGNAL")
    signal.pause()
note("FAKE_EXIT " + str(EXIT_CODE))
sys.exit(EXIT_CODE)
'''


@pytest.fixture
def fake_executor(tmp_path):
    path = tmp_path / "fake_executor.py"
    path.write_text(FAKE_EXECUTOR)
    return path


def launcher_command(log_root, executor, *arguments):
    return [sys.executable, str(LAUNCHER), "--log-root", str(log_root),
            "--", sys.executable, str(executor), *arguments]


def run_launcher(log_root, executor, *, environment=None, **arguments):
    return subprocess.run(launcher_command(log_root, executor), capture_output=True,
                          text=True, timeout=TIMEOUT_S, env={**os.environ, **(environment or {})},
                          **arguments)


def wait_for_run_directory(log_root):
    """Return the single run directory below ``log_root`` once the launcher created it."""
    deadline = time.monotonic() + TIMEOUT_S
    while time.monotonic() < deadline:
        root = Path(log_root)
        if root.is_dir():
            created = sorted(path for path in root.iterdir() if path.is_dir())
            if len(created) == 1:
                return created[0]
            if len(created) > 1:
                raise AssertionError(f"expected one run directory, found {created}")
        time.sleep(0.02)
    raise AssertionError(f"no run directory appeared under {log_root}")


def wait_for_text(path, marker, timeout=TIMEOUT_S):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if Path(path).exists() and marker in Path(path).read_text(errors="replace"):
            return
        time.sleep(0.02)
    raise AssertionError(f"{marker!r} never appeared in {path}")


def wait_for_exit(pid, timeout=TIMEOUT_S):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        finished, status = os.waitpid(pid, os.WNOHANG)
        if finished == pid:
            return status
        time.sleep(0.05)
    os.kill(pid, signal.SIGKILL)
    os.waitpid(pid, 0)
    raise AssertionError("launcher did not exit")


def reap_quietly(pid):
    try:
        os.kill(pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    try:
        os.waitpid(pid, 0)
    except ChildProcessError:
        pass


def stop_quietly(process):
    if process.poll() is None:
        process.kill()
    process.wait(timeout=TIMEOUT_S)


def drain_into(stream, chunks):
    """Keep a pipe moving, the way the operator's terminal does."""
    while True:
        chunk = stream.read(65536)
        if not chunk:
            return
        chunks.append(chunk)


def test_console_log_keeps_stdout_stderr_child_output_and_failure_status(tmp_path, fake_executor):
    log_root = tmp_path / "logs"
    result = run_launcher(log_root, fake_executor, environment={"FAKE_EXIT_CODE": "7"})

    assert result.returncode == 7, result.stderr
    run_directory = wait_for_run_directory(log_root)
    log = (run_directory / CONSOLE_LOG).read_text()
    for marker in ("FAKE_START", "FAKE_STDERR", "FAKE_CHILD_OUTPUT", "FAKE_EXIT 7"):
        assert marker in log
    # The directory was handed to the executor and mirrored to the operator terminal.
    assert f"log={run_directory.resolve()}" in log
    assert str(run_directory) in result.stdout and "FAKE_START" in result.stdout
    assert "FAKE_STDERR" in result.stderr

    record = json.loads((run_directory / EXIT_RECORD).read_text())
    assert record["status"] == "exited"
    assert record["exit_code"] == 7
    assert record["signal"] is None
    assert record["console_log_complete"] is True
    assert record["log_errors"] == [] and record["mirror_errors"] == []
    assert record["console_log_bytes"] == (run_directory / CONSOLE_LOG).stat().st_size
    assert record["executor_pid"] > 0
    assert record["environment"]["TIANJI_RUN_LOG_DIR"] == str(run_directory.resolve())


def test_zero_status_run_keeps_its_own_directory_without_overwriting(tmp_path, fake_executor):
    log_root = tmp_path / "logs"
    first = run_launcher(log_root, fake_executor,
                         environment={"FAKE_EXIT_CODE": "0", "FAKE_TAG": "first"})
    assert first.returncode == 0, first.stderr
    second = run_launcher(log_root, fake_executor,
                          environment={"FAKE_EXIT_CODE": "7", "FAKE_TAG": "second"})
    assert second.returncode == 7, second.stderr

    directories = sorted(path for path in log_root.iterdir() if path.is_dir())
    assert len(directories) == 2
    assert directories[0] != directories[1]
    logs = [(directory / CONSOLE_LOG).read_text() for directory in directories]
    first_log = next(log for log in logs if "FAKE_START tag=first" in log)
    second_log = next(log for log in logs if "FAKE_START tag=second" in log)
    assert "FAKE_EXIT 0" in first_log
    assert "FAKE_EXIT 7" in second_log
    codes = sorted(json.loads((directory / EXIT_RECORD).read_text())["exit_code"]
                   for directory in directories)
    assert codes == [0, 7]


def test_stop_signal_is_forwarded_even_after_executor_closes_output(tmp_path):
    executor = tmp_path / "closed_output_executor.py"
    marker = tmp_path / "cleanup.txt"
    executor.write_text(
        "import os, signal, time\n"
        "from pathlib import Path\n"
        f"marker = Path({str(marker)!r})\n"
        "def stop(signum, frame):\n"
        "    marker.write_text('cleanup completed')\n"
        "    os._exit(9)\n"
        "signal.signal(signal.SIGTERM, stop)\n"
        "os.close(1)\n"
        "os.close(2)\n"
        "# Allow the launcher to observe both EOFs before requesting a stop.\n"
        "time.sleep(0.5)\n"
        "os.kill(os.getppid(), signal.SIGTERM)\n"
        "time.sleep(3)\n"
        "os._exit(8)\n")
    result = run_launcher(tmp_path / "logs", executor)
    assert result.returncode == 9
    assert marker.read_text() == "cleanup completed"
    record = json.loads((wait_for_run_directory(tmp_path / "logs") / EXIT_RECORD).read_text())
    assert record["exit_code"] == 9
    assert record["signals_received"] == ["SIGTERM"]


def test_help_and_argument_failure_output_is_kept(tmp_path, fake_executor):
    log_root = tmp_path / "logs"
    result = run_launcher(log_root, fake_executor, environment={"FAKE_ARGV_FAILURE": "1"})

    assert result.returncode == 2, result.stderr
    run_directory = wait_for_run_directory(log_root)
    log = (run_directory / CONSOLE_LOG).read_text()
    assert "usage: run_teleop.py" in log
    assert "error: unrecognized arguments: --nonsense" in log
    assert json.loads((run_directory / EXIT_RECORD).read_text())["exit_code"] == 2


def test_bulk_output_does_not_deadlock_or_lose_the_exit_status(tmp_path, fake_executor):
    log_root = tmp_path / "logs"
    result = run_launcher(log_root, fake_executor,
                          environment={"FAKE_EXIT_CODE": "5", "FAKE_BULK_LINES": "4000"})

    assert result.returncode == 5, result.stderr
    run_directory = wait_for_run_directory(log_root)
    log = (run_directory / CONSOLE_LOG).read_text()
    assert log.count("FAKE_BULK ") == 4000
    assert log.count("FAKE_BULK_ERR ") == 4000
    assert json.loads((run_directory / EXIT_RECORD).read_text())["console_log_complete"] is True


def test_launcher_refuses_to_start_before_contacting_the_executor(tmp_path, fake_executor):
    occupied = tmp_path / "not-a-directory"
    occupied.write_text("occupied")
    marker = tmp_path / "executor-started"

    result = run_launcher(occupied, fake_executor, environment={"FAKE_MARKER": str(marker)})

    assert result.returncode == 3, result.stderr
    assert "REFUSING TO START" in result.stderr
    assert not marker.exists()
    assert list(tmp_path.glob("*/" + CONSOLE_LOG)) == []


def test_closed_terminal_stdout_still_records_the_run(tmp_path, fake_executor):
    log_root = tmp_path / "logs"
    process = subprocess.Popen(launcher_command(log_root, fake_executor),
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               env={**os.environ, "FAKE_EXIT_CODE": "7", "FAKE_BULK_LINES": "4000"})
    stderr_chunks = []
    reader = threading.Thread(target=drain_into, args=(process.stderr, stderr_chunks), daemon=True)
    reader.start()
    try:
        process.stdout.close()  # the operator's terminal went away (for example `| head`)
        assert process.wait(timeout=TIMEOUT_S) == 7
    finally:
        stop_quietly(process)
    reader.join(timeout=TIMEOUT_S)
    stderr = b"".join(stderr_chunks).decode()

    run_directory = wait_for_run_directory(log_root)
    log = (run_directory / CONSOLE_LOG).read_text()
    assert log.count("FAKE_BULK ") == 4000
    assert "FAKE_EXIT 7" in log
    assert "Traceback" not in stderr
    assert "no longer writable" in stderr
    record = json.loads((run_directory / EXIT_RECORD).read_text())
    assert record["exit_code"] == 7 and record["console_log_complete"] is True
    assert record["mirror_errors"] and not record["log_errors"]


def test_sigterm_is_forwarded_and_executor_cleanup_finishes_first(tmp_path, fake_executor):
    log_root = tmp_path / "logs"
    process = subprocess.Popen(
        launcher_command(log_root, fake_executor), start_new_session=True,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        env={**os.environ, "FAKE_CLEANUP_DELAY_S": "0.5", "FAKE_CLEANUP_CODE": "9", "FAKE_WAIT_SIGNAL": "1"})
    try:
        run_directory = wait_for_run_directory(log_root)
        wait_for_text(run_directory / CONSOLE_LOG, "FAKE_WAITING_SIGNAL")

        process.send_signal(signal.SIGTERM)  # the launcher alone, not the process group
        assert process.wait(timeout=TIMEOUT_S) == 9
    finally:
        stop_quietly(process)

    log = (run_directory / CONSOLE_LOG).read_text()
    assert "FAKE_CLEANUP signal=15" in log
    assert "FAKE_CLEANUP_DONE" in log
    record = json.loads((run_directory / EXIT_RECORD).read_text())
    assert record["signals_received"] == ["SIGTERM"]
    assert record["status"] == "exited" and record["exit_code"] == 9


def test_launcher_restores_terminal_after_executor_is_killed(tmp_path, fake_executor):
    import termios
    master, slave = pty.openpty()
    original = termios.tcgetattr(slave)
    try:
        result = run_launcher(tmp_path / 'logs', fake_executor, stdin=slave,
                              environment={'FAKE_CBREAK': '1', 'FAKE_SELF_SIGNAL': 'KILL'})
        assert result.returncode == 137
        assert termios.tcgetattr(slave) == original
    finally:
        termios.tcsetattr(slave, termios.TCSANOW, original)
        os.close(master)
        os.close(slave)


def test_pty_stdin_keeps_enter_and_ctrl_c_working(tmp_path, fake_executor):
    log_root = tmp_path / "logs"
    environment = {**os.environ, "FAKE_WATCH_ENTER": "1", "FAKE_EXIT_CODE": "0",
                   "FAKE_CLEANUP_CODE": "9"}
    pid, master = pty.fork()
    if pid == 0:
        try:
            os.execve(sys.executable, launcher_command(log_root, fake_executor), environment)
        except BaseException:
            os._exit(127)
    transcript = bytearray()
    try:
        run_directory = wait_for_run_directory(log_root)
        console = run_directory / CONSOLE_LOG
        wait_for_text(console, "FAKE_WAITING_ENTER")

        os.write(master, b"\n")  # Enter through the pty line discipline
        wait_for_text(console, "FAKE_IDLE")

        os.write(master, b"\x03")  # Ctrl+C: SIGINT to the whole foreground group
        wait_for_text(console, "FAKE_CLEANUP_DONE")
        status = wait_for_exit(pid)
        while True:
            try:
                chunk = os.read(master, 65536)
            except OSError:
                break
            if not chunk:
                break
            transcript.extend(chunk)
    finally:
        os.close(master)
        reap_quietly(pid)

    assert os.waitstatus_to_exitcode(status) == 9
    log = (run_directory / CONSOLE_LOG).read_text()
    assert "tty=True" in log  # the executor kept the operator terminal on stdin
    assert "FAKE_ENTER_SEEN" in log and "FAKE_CLEANUP_DONE" in log
    # The operator still sees a real terminal transcript, including the printed path.
    assert b"FAKE_START" in transcript
    assert str(log_root).encode() in transcript


def test_executor_killed_by_signal_keeps_the_shell_status(tmp_path, fake_executor):
    log_root = tmp_path / "logs"
    result = run_launcher(log_root, fake_executor, environment={"FAKE_SELF_SIGNAL": "KILL"})

    assert result.returncode == 137, result.stderr
    run_directory = wait_for_run_directory(log_root)
    assert "FAKE_SELF_SIGNAL KILL" in (run_directory / CONSOLE_LOG).read_text()
    record = json.loads((run_directory / EXIT_RECORD).read_text())
    assert record["status"] == "signaled"
    assert record["signal"] == signal.SIGKILL
    assert record["exit_code"] == 137


class BrokenSink:
    """Log file that fails every write, like a full or unplugged device."""

    def write(self, data):
        raise OSError(28, "No space left on device")

    def flush(self):
        raise OSError(28, "No space left on device")

    def close(self):
        raise OSError(28, "No space left on device")


def test_log_sink_failure_is_reported_and_not_claimed_complete(tmp_path):
    terminal_out, terminal_err = io.StringIO(), io.StringIO()
    console = ConsoleLog(tmp_path / CONSOLE_LOG, sink=BrokenSink(),
                         stdout=terminal_out, stderr=terminal_err)

    console.write("stdout", b"REAL_TRACE_LINE\n")

    assert console.complete is False
    assert console.errors
    assert "CONSOLE LOG INCOMPLETE" in terminal_err.getvalue()
    # The run keeps its terminal copy instead of failing into silence.
    assert "REAL_TRACE_LINE" in terminal_out.getvalue()
    assert console.close() is False


def test_run_directories_are_never_reused(tmp_path):
    created = {create_run_directory(tmp_path) for _ in range(5)}
    assert len(created) == 5
    assert all(path.is_dir() for path in created)
