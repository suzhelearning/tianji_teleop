#!/usr/bin/env python3
"""Per-run console logging for operator terminal entrypoints.

The launcher creates a unique ``<log-root>/<timestamp>-<unique>/`` directory before
the command is started, mirrors its stdout/stderr (Python, vendor SDK and child
controller output) to the operator terminal and to ``console.log``, exports the
directory as ``TIANJI_RUN_LOG_DIR`` and records the raw exit status in
``launcher_exit.json``. The four terminal entrypoints select their own roots under
``tmp/logs``; direct commands without this wrapper keep their previous behaviour.

Design constraints kept here:

* stdin is inherited, so the executor still owns the operator TTY for Enter/Ctrl+C.
* stdout/stderr are mirrored, not replaced: the operator sees the same text that
  lands in ``console.log``.  ``PYTHONUNBUFFERED=1`` is exported because a pipe
  would otherwise delay operator prompts such as "Press ENTER to enable".
* the executor keeps its own signal handling.  The launcher forwards SIGINT/SIGTERM
  to the executor and waits for its cleanup; it never kills the executor or the
  hardware controller itself.
* a run is refused before the executor starts when the log directory cannot be
  created, and a log that stops accepting writes mid-run is reported loudly and
  marked incomplete rather than silently presented as a full record.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import secrets
import select
import shlex
import signal
import subprocess
import sys
import time
import termios

CONSOLE_LOG_NAME = "console.log"
EXIT_RECORD_NAME = "launcher_exit.json"
LOG_DIRECTORY_VARIABLE = "TIANJI_RUN_LOG_DIR"
DEFAULT_LOG_ROOT = Path("logs") / "real"
# Refused before any hardware work; distinct from the executor's own 1 and argparse 2.
LOG_SETUP_EXIT_CODE = 3
# Executor could not be spawned at all.
SPAWN_FAILURE_EXIT_CODE = 127
DRAIN_GRACE_S = 5.0
POLL_INTERVAL_S = 0.2
READ_SIZE = 65536


def _write_bytes(stream, data):
    buffer = getattr(stream, "buffer", None)
    if buffer is None:
        stream.write(data.decode("utf-8", "backslashreplace"))
    else:
        buffer.write(data)
    stream.flush()


def _detach(stream):
    """Point a dead terminal stream at /dev/null so late writes cannot raise again."""
    try:
        descriptor = stream.fileno()
        null = os.open(os.devnull, os.O_WRONLY)
        try:
            os.dup2(null, descriptor)
        finally:
            os.close(null)
    except (OSError, ValueError, AttributeError):
        pass


class ConsoleLog:
    """Mirror of one run: true record in ``path``, best-effort terminal copy."""

    def __init__(self, path, *, sink=None, stdout=None, stderr=None):
        self.path = Path(path)
        self.stdout = sys.stdout if stdout is None else stdout
        self.stderr = sys.stderr if stderr is None else stderr
        self.sink = self.path.open("wb") if sink is None else sink
        self.bytes_written = 0
        self.errors = []
        self.mirror_errors = []
        self._reported_loss = False

    @property
    def complete(self):
        return not self.errors

    def write(self, stream, data):
        """Record ``data`` in the log, then show it on the matching terminal stream."""
        self._append(data)
        self._mirror(stream, data)

    def note(self, text):
        """Launcher message: visible on the terminal and inside the log."""
        self.write("stdout", (text + "\n").encode("utf-8", "backslashreplace"))

    def warn(self, message, *, incomplete=False):
        """Explicit failure report; ``incomplete`` marks the log as not a full record."""
        if incomplete:
            self.errors.append(message)
        else:
            self.mirror_errors.append(message)
        line = f"WARNING: {message}\n"
        self._append(line.encode("utf-8", "backslashreplace"))
        self._mirror("stderr", line.encode("utf-8", "backslashreplace"))

    def close(self):
        if self.sink is None:
            return self.complete
        try:
            self.sink.flush()
            self.sink.close()
        except (OSError, ValueError) as error:
            self.errors.append(f"{self.path}: {error}")
        finally:
            self.sink = None
        return self.complete

    def _append(self, data):
        if self.sink is None:
            return
        try:
            self.sink.write(data)
            self.sink.flush()
        except (OSError, ValueError) as error:
            self.sink = None
            self.errors.append(f"console log write failed: {error}")
            self._report(
                f"CONSOLE LOG INCOMPLETE: {self.path} stopped accepting writes ({error}); "
                "this run continues but the log is not a complete record")
            return
        self.bytes_written += len(data)

    def _mirror(self, stream, data):
        target = self.stderr if stream == "stderr" else self.stdout
        if target is None:
            return
        try:
            _write_bytes(target, data)
        except (OSError, ValueError) as error:
            if stream == "stderr":
                self.stderr = None
            else:
                self.stdout = None
            self.mirror_errors.append(f"{stream}: {error}")
            _detach(target)
            self._report(f"{stream} is no longer writable ({error}); the console log continues")

    def _report(self, message):
        """Best-effort notice on the terminal stderr; never raises and never repeats."""
        if self._reported_loss:
            return
        self._reported_loss = True
        line = f"WARNING: {message}\n".encode("utf-8", "backslashreplace")
        self._append(line)
        try:
            _write_bytes(self.stderr if self.stderr is not None else sys.stderr, line)
        except (OSError, ValueError):
            pass


class SignalForwarder:
    """Hand signals to the executor so its own cleanup runs; never kill it first."""

    SIGNALS = (signal.SIGINT, signal.SIGTERM)

    def __init__(self, process, console):
        self.process = process
        self.console = console
        self.received = []
        self._previous = {}

    def install(self):
        self._previous = {sig: signal.signal(sig, self._handle) for sig in self.SIGNALS}

    def restore(self):
        for sig, handler in self._previous.items():
            signal.signal(sig, handler)
        self._previous = {}

    def _handle(self, signum, frame):
        name = signal.Signals(signum).name
        self.received.append(name)
        # A Python signal may interrupt an in-progress buffered write. Do not
        # log here: reentrant logging can prevent the stop signal being sent.
        if self.process is not None and self.process.poll() is None:
            try:
                os.kill(self.process.pid, signum)
            except ProcessLookupError:
                pass


def create_run_directory(log_root):
    """Create a fresh ``<timestamp>-<unique>`` directory; never reuse an existing one."""
    root = Path(log_root)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    for _ in range(8):
        directory = root / f"{stamp}-{os.getpid()}-{secrets.token_hex(3)}"
        try:
            directory.mkdir(parents=True, exist_ok=False)
        except FileExistsError:
            continue
        return directory
    raise FileExistsError(f"no unique run directory name available under {root}")


def write_exit_record(log_dir, record, *, console=None):
    """Atomically replace ``launcher_exit.json``; failures are reported, not swallowed."""
    path = Path(log_dir) / EXIT_RECORD_NAME
    temporary = path.with_name(path.name + ".tmp")
    try:
        temporary.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n")
        os.replace(temporary, path)
    except OSError as error:
        message = f"cannot record exit status in {path}: {error}"
        if console is None:
            print(f"WARNING: {message}", file=sys.stderr, flush=True)
        else:
            console.warn(message, incomplete=True)
        return False
    return True


def _relay(process, console):
    """Copy executor stdout/stderr to the terminal and log until both pipes close."""
    descriptors = {name: stream.fileno()
                   for name, stream in (("stdout", process.stdout), ("stderr", process.stderr))}
    deadline = None
    while descriptors:
        readable, _, _ = select.select(list(descriptors.values()), [], [], POLL_INTERVAL_S)
        for name, descriptor in list(descriptors.items()):
            if descriptor not in readable:
                continue
            try:
                data = os.read(descriptor, READ_SIZE)
            except OSError as error:
                console.warn(f"{name} pipe could not be read ({error}); trailing output may be lost",
                             incomplete=True)
                data = b""
            if data:
                console.write(name, data)
            else:
                del descriptors[name]
        if process.poll() is None:
            deadline = None
        elif deadline is None:
            deadline = time.monotonic() + DRAIN_GRACE_S
        elif time.monotonic() >= deadline:
            console.warn("executor exited but " + ", ".join(sorted(descriptors)) +
                         f" stayed open for {DRAIN_GRACE_S:g} s; the log may miss trailing output",
                         incomplete=True)
            break


def _exit_status(returncode):
    """Map a subprocess status to (status, signal number or None, exit code)."""
    if returncode is None:
        raise ValueError("executor status is not known yet")
    if returncode < 0:
        number = -returncode
        return "signaled", number, 128 + number
    return "exited", None, returncode


def run_executor(command, log_dir, console, *, cwd=None):
    """Start the executor under ``log_dir`` and return the final launcher exit record."""
    log_dir = Path(log_dir).resolve()
    started_at = datetime.now().astimezone()
    started = time.monotonic()
    environment = dict(os.environ)
    environment[LOG_DIRECTORY_VARIABLE] = str(log_dir)
    # A pipe would otherwise buffer operator prompts; the log must mirror the terminal.
    environment["PYTHONUNBUFFERED"] = "1"
    record = {
        "status": "running",
        "log_dir": str(log_dir),
        "console_log": CONSOLE_LOG_NAME,
        "command": list(command),
        "cwd": str(Path(cwd).resolve() if cwd else Path.cwd()),
        "launcher_pid": os.getpid(),
        "started_at": started_at.isoformat(timespec="seconds"),
        "environment": {LOG_DIRECTORY_VARIABLE: str(log_dir)},
    }
    console.note(f"run log: {log_dir}")
    console.note(f"command: {shlex.join(command)}")
    # Verify both records before starting anything that could connect hardware.
    if not console.complete or not write_exit_record(log_dir, record, console=console):
        console.warn("REFUSING TO START: initial run log could not be recorded", incomplete=True)
        return LOG_SETUP_EXIT_CODE
    forwarder = SignalForwarder(None, console)
    forwarder.install()
    process = None
    try:
        process = subprocess.Popen(command, cwd=cwd, env=environment, stdin=None,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except OSError as error:
        forwarder.restore()
        console.warn(f"executor was not started: {error}", incomplete=True)
        record.update(status="spawn_failed", exit_code=SPAWN_FAILURE_EXIT_CODE, signal=None,
                      finished_at=datetime.now().astimezone().isoformat(timespec="seconds"),
                      duration_s=round(time.monotonic() - started, 3),
                      console_log_bytes=console.bytes_written, console_log_complete=False,
                      log_errors=list(console.errors), mirror_errors=list(console.mirror_errors))
        write_exit_record(log_dir, record, console=console)
        return SPAWN_FAILURE_EXIT_CODE
    except BaseException:
        forwarder.restore()
        raise
    record["executor_pid"] = process.pid
    forwarder.process = process
    try:
        # A signal arriving inside Popen must not disappear before attachment.
        for name in tuple(forwarder.received):
            if process.poll() is None:
                process.send_signal(getattr(signal, name))
        write_exit_record(log_dir, record, console=console)
        _relay(process, console)
        # EOF on stdout/stderr does not imply the executor has exited. Keep
        # forwarding stop signals throughout its remaining cleanup.
        process.wait()
    except BaseException:
        if process.poll() is None:
            process.terminate()
        try:
            _relay(process, console)
        finally:
            process.wait()
        raise
    finally:
        forwarder.restore()
        for stream in (process.stdout, process.stderr):
            try:
                stream.close()
            except OSError:
                pass
    status, number, exit_code = _exit_status(process.returncode)
    console.note(f"exit status {exit_code} ({status}); console log: {log_dir / CONSOLE_LOG_NAME}")
    complete = console.close()
    if not complete:
        console.warn(f"CONSOLE LOG INCOMPLETE: {log_dir / CONSOLE_LOG_NAME}; this run is recorded in "
                     f"{log_dir / EXIT_RECORD_NAME} as incomplete")
    record.update(status=status, signal=number, exit_code=exit_code,
                  finished_at=datetime.now().astimezone().isoformat(timespec="seconds"),
                  duration_s=round(time.monotonic() - started, 3),
                  signals_received=forwarder.received,
                  console_log_bytes=console.bytes_written,
                  console_log_complete=complete,
                  log_errors=list(console.errors), mirror_errors=list(console.mirror_errors))
    write_exit_record(log_dir, record)
    return exit_code


def main(argv=None):
    arguments = list(sys.argv[1:] if argv is None else argv)
    if "--" in arguments:
        separator = arguments.index("--")
        options, command = arguments[:separator], arguments[separator + 1:]
    else:
        options, command = arguments, []
    parser = argparse.ArgumentParser(
        prog="run_logging.py",
        description="Run a command with a per-run console log.",
        epilog="example: run_logging.py --log-root tmp/logs/teleop -- bash bash/run_teleop.sh --help")
    parser.add_argument("--log-root", type=Path, default=DEFAULT_LOG_ROOT,
                        help=f"directory receiving one <timestamp>-<unique> run directory per run "
                             f"(default: {DEFAULT_LOG_ROOT})")
    args = parser.parse_args(options)
    if not command:
        parser.error("no executor command given; expected: --log-root DIR -- COMMAND [ARG...]")
    log_root = args.log_root if args.log_root.is_absolute() else Path.cwd() / args.log_root
    try:
        log_dir = create_run_directory(log_root)
    except OSError as error:
        print(f"REFUSING TO START: cannot create a run log directory under {log_root}: {error}",
              file=sys.stderr, flush=True)
        print("No executor was started and no hardware was contacted.", file=sys.stderr, flush=True)
        return LOG_SETUP_EXIT_CODE
    try:
        console = ConsoleLog(log_dir / CONSOLE_LOG_NAME)
    except OSError as error:
        print(f"REFUSING TO START: cannot write {log_dir / CONSOLE_LOG_NAME}: {error}",
              file=sys.stderr, flush=True)
        print("No executor was started and no hardware was contacted.", file=sys.stderr, flush=True)
        write_exit_record(log_dir, {
            "status": "log_setup_failed", "log_dir": str(log_dir), "command": list(command),
            "error": str(error), "console_log_complete": False,
            "finished_at": datetime.now().astimezone().isoformat(timespec="seconds")})
        return LOG_SETUP_EXIT_CODE
    terminal = None
    if sys.stdin.isatty():
        terminal = (sys.stdin.fileno(), termios.tcgetattr(sys.stdin.fileno()))
    try:
        return run_executor(command, log_dir, console)
    finally:
        console.close()
        if terminal is not None:
            try:
                termios.tcsetattr(terminal[0], termios.TCSANOW, terminal[1])
            except OSError as error:
                print(f"WARNING: could not restore operator terminal: {error}", file=sys.stderr, flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
