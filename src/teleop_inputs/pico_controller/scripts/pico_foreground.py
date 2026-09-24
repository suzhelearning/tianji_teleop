#!/usr/bin/env python3
"""Run the owned PICO input pipeline in the operator's current terminal."""
import argparse
import codecs
from collections import deque
import fcntl
import json
import os
from pathlib import Path
import selectors
import signal
import stat
import subprocess
import sys
import time
import uuid


LOG_NAMES = {
    "driver": "driver.log",
    "m0": "skeleton-calibration-viewer.log",
    "bridge": "arm-input-publisher.log",
    "ready": "readiness.log",
    "foreground": "foreground.log",
}
READ_SIZE = 4096
OUTPUT_LIMIT = 256 * 1024


class ForegroundError(RuntimeError):
    pass
def _lock_path(path):
    return Path(path) if path is not None else Path(
        f"/tmp/tianji-pico-foreground-{os.getuid()}.lock")


def _check_lock_file(fd):
    metadata = os.fstat(fd)
    if metadata.st_uid != os.getuid() or not stat.S_ISREG(metadata.st_mode):
        raise ForegroundError("Unsafe foreground launch lock")


def discover_foreground(lock_path=None):
    """Return metadata only while a foreground supervisor holds its launch lock."""
    try:
        fd = os.open(_lock_path(lock_path), os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC)
    except FileNotFoundError:
        return None
    try:
        _check_lock_file(fd)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            payload = os.read(fd, 16 * 1024 + 1)
            try:
                if len(payload) > 16 * 1024:
                    raise ValueError("oversized metadata")
                metadata = json.loads(payload)
                if (not isinstance(metadata, dict)
                        or type(metadata["pid"]) is not int or metadata["pid"] <= 1
                        or not isinstance(metadata["owner"], str)
                        or uuid.UUID(metadata["owner"]).hex != metadata["owner"]
                        or not isinstance(metadata["checkout"], str)
                        or not Path(metadata["checkout"]).is_absolute()):
                    raise ValueError("invalid foreground identity")
                directory = metadata["calibration_dir"]
                fingerprint = metadata["calibration_sha256"]
                if (directory is None) != (fingerprint is None):
                    raise ValueError("incomplete calibration identity")
                if directory is not None and (
                        not isinstance(directory, str) or not Path(directory).is_absolute()
                        or not isinstance(fingerprint, str) or len(fingerprint) != 64
                        or any(char not in "0123456789abcdef" for char in fingerprint)):
                    raise ValueError("invalid calibration identity")
                return metadata
            except (ValueError, KeyError, TypeError, AttributeError) as error:
                raise ForegroundError("Active foreground launch metadata is incomplete or invalid") from error
        return None
    finally:
        os.close(fd)




def _running_groups(groups):
    """A leader can exit before its descendants; zombies need no further signals."""
    running = set()
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / "stat").read_text().rsplit(")", 1)[1].split()
            group = int(fields[2])
            if group in groups and fields[0] != "Z":
                running.add(group)
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    return running


class _Supervisor:
    def __init__(self, checkout, logs, output_fd, env):
        self.checkout = checkout
        self.logs = logs
        self.output_fd = output_fd
        self.env = env
        self.selector = selectors.DefaultSelector()
        self.children = {}
        self.groups = set()
        self.output = deque()
        self.output_size = 0
        self.output_failed = False
        self.io_error = None
        self.received_signal = None
        self.decoders = {
            name: codecs.getincrementaldecoder("utf-8")(errors="replace")
            for name in logs
        }

    def signal_received(self, signum, _frame):
        if self.received_signal is None:
            self.received_signal = signum

    def emit(self, name, data, *, final=False):
        # Log before touching the terminal, including throughout shutdown.
        try:
            if self.logs[name].write(data) != len(data):
                raise OSError("short logfile write")
        except OSError as error:
            self.io_error = f"Cannot persist {name} output: {error}"
        if self.output_failed:
            return
        text = self.decoders[name].decode(data, final=final)
        if not text:
            return
        framed = "".join(
            f"[{name}] " + part + ("" if part.endswith("\n") else "\n")
            for part in text.splitlines(keepends=True)
        ).encode("utf-8")
        if self.output_size + len(framed) > OUTPUT_LIMIT:
            self.output_failed = True
            self.io_error = "Terminal output is not draining; stopping owned PICO processes"
            self.output.clear()
            self.output_size = 0
            return
        self.output.append(framed)
        self.output_size += len(framed)
        self.flush_output()

    def flush_output(self):
        while self.output and not self.output_failed:
            chunk = self.output[0]
            if not chunk:
                self.output.popleft()
                continue
            try:
                count = os.write(self.output_fd, chunk)
            except BlockingIOError:
                return
            except OSError as error:
                self.output_failed = True
                self.io_error = f"Terminal output closed: {error}"
                self.output.clear()
                self.output_size = 0
                return
            self.output_size -= count
            if count == len(chunk):
                self.output.popleft()
            else:
                self.output[0] = chunk[count:]

    def message(self, text):
        self.emit("foreground", (text + "\n").encode())

    def launch(self, name, command):
        process = subprocess.Popen(
            command, cwd=self.checkout, env=self.env, stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=0,
            start_new_session=True,
        )
        # Record ownership before any operation which can fail.
        self.children[name] = process
        self.groups.add(process.pid)
        os.set_blocking(process.stdout.fileno(), False)
        self.selector.register(process.stdout, selectors.EVENT_READ, name)

    def pump(self, timeout=0.05):
        self.flush_output()
        for key, _events in self.selector.select(timeout):
            try:
                data = os.read(key.fd, READ_SIZE)
            except BlockingIOError:
                continue
            except OSError as error:
                self.io_error = f"Cannot read {key.data} output: {error}"
                data = b""
            if data:
                self.emit(key.data, data)
            else:
                self.emit(key.data, b"", final=True)
                self.selector.unregister(key.fileobj)
                key.fileobj.close()
        exited = {process.pid for process in self.children.values()
                  if process.pid in self.groups and process.poll() is not None}
        if exited:
            # Never retain an extinct readiness group ID for a later interrupt:
            # a long-running session could otherwise signal a reused PID.
            self.groups.difference_update(exited - _running_groups(exited))

    def stop(self, interrupt_grace_s, terminate_grace_s):
        groups = set(self.groups)
        # M0 owns further setsid groups and must complete its own reverse-order
        # cleanup, including the recorder's 30-second save window, before TERM.
        for signum, grace in ((signal.SIGINT, interrupt_grace_s),
                              (signal.SIGTERM, terminate_grace_s),
                              (signal.SIGKILL, 1.0)):
            for group in groups:
                try:
                    os.killpg(group, signum)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + grace
            while groups and time.monotonic() < deadline:
                self.pump()
                for process in self.children.values():
                    process.poll()
                groups = _running_groups(groups)
            if not groups:
                break
        # Drain buffered shutdown output, but never wait indefinitely for an
        # inherited pipe held by a process outside the owned groups.
        deadline = time.monotonic() + 0.5
        while self.selector.get_map() and time.monotonic() < deadline:
            self.pump(0.01)
        for process in self.children.values():
            try:
                process.wait(timeout=0.1)
            except subprocess.TimeoutExpired:
                self.message(f"Process {process.pid} did not exit after SIGKILL")
            if process.stdout is not None:
                process.stdout.close()
        self.flush_output()


def run_foreground(checkout, log_dir, commands, *, readiness_command=None,
                   lock_path=None, output_fd=None, interrupt_grace_s=40.0,
                   terminate_grace_s=3.0, calibration_dir=None,
                   calibration_sha256=None):
    """Supervise until interruption/failure; return a shell-compatible exit code.

    Tests may inject a harmless readiness argv and a private lock path. The CLI
    always invokes the real readiness checker; successful startup is not exit.
    """
    if set(commands) != {"driver", "m0", "bridge"}:
        raise ValueError("Exactly driver, m0 and bridge commands are required")
    if (calibration_dir is None) != (calibration_sha256 is None):
        raise ValueError("calibration directory and fingerprint must be supplied together")
    checkout = Path(checkout).resolve()
    log_dir = Path(log_dir)
    output_fd = sys.stdout.fileno() if output_fd is None else output_fd
    lock_path = _lock_path(lock_path)
    owner = uuid.uuid4().hex
    env = os.environ.copy()
    env.pop("TMUX", None)
    env.pop("TMUX_PANE", None)
    env.update(TIANJI_PICO_SESSION_OWNER=owner, PYTHONUNBUFFERED="1")
    if readiness_command is None:
        readiness_command = [
            sys.executable, str(Path(__file__).with_name("check_pico_session_ready.py")),
            "--viewer-owner", owner, "--timeout-s", "30",
        ]

    def check_calibration():
        if calibration_dir is not None:
            from ensure_pico_user import fingerprint
            if fingerprint(calibration_dir) != calibration_sha256:
                raise ForegroundError("PICO calibration content changed during startup")

    logs = {}
    lock_fd = None
    old_flags = None
    old_handlers = {}
    supervisor = None
    result = 2
    try:
        lock_fd = os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
        _check_lock_file(lock_fd)
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise ForegroundError("A foreground PICO launch is already running for this user") from error
        metadata = {
            "checkout": str(checkout),
            "calibration_dir": str(Path(calibration_dir).resolve()) if calibration_dir is not None else None,
            "calibration_sha256": calibration_sha256,
            "owner": owner,
            "pid": os.getpid(),
        }
        os.ftruncate(lock_fd, 0)
        payload = json.dumps(metadata).encode()
        if os.write(lock_fd, payload) != len(payload):
            raise ForegroundError("Cannot persist foreground launch metadata")
        for name, filename in LOG_NAMES.items():
            logs[name] = (log_dir / filename).open("ab", buffering=0)
        old_flags = fcntl.fcntl(output_fd, fcntl.F_GETFL)
        fcntl.fcntl(output_fd, fcntl.F_SETFL, old_flags | os.O_NONBLOCK)
        supervisor = _Supervisor(checkout, logs, output_fd, env)
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            old_handlers[signum] = signal.signal(signum, supervisor.signal_received)
        check_calibration()
        for name in ("driver", "m0", "bridge"):
            if supervisor.received_signal is not None:
                return 128 + supervisor.received_signal
            supervisor.launch(name, ["bash", "--noprofile", "--norc", "-c", commands[name]])
        supervisor.launch("ready", readiness_command)
        ready = False
        deadline = time.monotonic() + 35.0
        supervisor.message(f"Starting PICO in this terminal; logs: {log_dir}; Ctrl+C stops owned processes")
        while True:
            supervisor.pump()
            if supervisor.received_signal is not None:
                result = 128 + supervisor.received_signal
                break
            if supervisor.io_error is not None:
                raise ForegroundError(supervisor.io_error)
            for name in ("driver", "m0", "bridge"):
                status = supervisor.children[name].poll()
                if status is not None:
                    raise ForegroundError(f"{name} exited with status {status}; PICO pipeline stopped")
            if not ready:
                status = supervisor.children["ready"].poll()
                if status is not None:
                    if status != 0:
                        raise ForegroundError(f"Readiness checker exited with status {status}; PICO startup failed")
                    check_calibration()
                    # Fingerprinting is I/O: recheck children after it completes.
                    if any(supervisor.children[name].poll() is not None for name in commands):
                        raise ForegroundError("PICO pipeline exited during readiness verification")
                    ready = True
                    supervisor.message("PICO operational: live tracking and viewer verified; no executor authorization")
                elif time.monotonic() >= deadline:
                    raise ForegroundError("Readiness checker exceeded its startup deadline")
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        if supervisor is not None:
            supervisor.message(str(error))
        else:
            try:
                os.write(2, (f"PICO foreground: {error}\n").encode())
            except OSError:
                pass
    finally:
        try:
            if supervisor is not None:
                supervisor.stop(interrupt_grace_s, terminate_grace_s)
                supervisor.selector.close()
        finally:
            for signum, handler in old_handlers.items():
                signal.signal(signum, handler)
            if old_flags is not None:
                try:
                    fcntl.fcntl(output_fd, fcntl.F_SETFL, old_flags)
                except OSError:
                    pass
            for logfile in logs.values():
                logfile.close()
            if lock_fd is not None:
                os.close(lock_fd)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkout", required=True, type=Path)
    parser.add_argument("--log-dir", required=True, type=Path)
    for name in ("driver", "m0", "bridge"):
        parser.add_argument("--" + name, required=True)
    parser.add_argument("--calibration-dir", type=Path)
    parser.add_argument("--calibration-sha256")
    args = parser.parse_args(argv)
    if (args.calibration_dir is None) != (args.calibration_sha256 is None):
        parser.error("--calibration-dir and --calibration-sha256 must be supplied together")
    return run_foreground(
        args.checkout, args.log_dir,
        {name: getattr(args, name) for name in ("driver", "m0", "bridge")},
        calibration_dir=args.calibration_dir, calibration_sha256=args.calibration_sha256,
    )


if __name__ == "__main__":
    raise SystemExit(main())
