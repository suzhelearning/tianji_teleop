"""Private, display-only pipe to the installed native simulation viewer.

The parent owns all state transitions and exit handling.
Frames are atomic nonblocking writes: a busy renderer never delays control.
"""
import math
import os
import select
import subprocess
import time

from tianji_runtime.resources import native_executable


class NativeViewer:
    def __init__(self, model, profile, events, *, startup_timeout_s=15.0,
                 continuous_follow=False):
        self._events = events
        self._control_keys = (32, 82, 83, 80, 72, 81) if continuous_follow else (32, 67, 83, 80, 72, 81)
        self._output = bytearray()
        self._ready = False
        self._disconnected = False
        self._exit_sent = False
        self._closed = False
        command = [str(native_executable("tianji_qp_ik_viewer")), "--external-display",
                   "--model", str(model), "--config", str(profile)]
        if continuous_follow:
            command.append("--continuous-follow")
        self._process = subprocess.Popen(
            command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        try:
            os.set_blocking(self._process.stdin.fileno(), False)
            os.set_blocking(self._process.stdout.fileno(), False)
            self._pipe_buf = os.fpathconf(self._process.stdin.fileno(), "PC_PIPE_BUF")
            deadline = time.monotonic() + startup_timeout_s
            while not self._ready:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or not select.select([self._process.stdout], [], [], remaining)[0]:
                    raise TimeoutError("native display did not report DISPLAY_READY")
                self.poll_events()
                if self._disconnected:
                    raise RuntimeError("native display exited before window setup")
        except BaseException:
            self.close()
            raise

    def _request_exit(self):
        if not self._exit_sent:
            self._exit_sent = True
            self._events.put("q")

    def poll_events(self):
        """Drain a bounded amount of child output without waiting for a key."""
        if self._closed or self._disconnected:
            return
        try:
            block = os.read(self._process.stdout.fileno(), 65536)
        except BlockingIOError:
            block = None
        except OSError:
            block = b""
        if block == b"":
            self._disconnected = True
        elif block:
            self._output.extend(block)
            lines = self._output.split(b"\n")
            self._output = bytearray(lines.pop())
            for line in lines:
                if line == b"DISPLAY_READY":
                    self._ready = True
                elif line.startswith(b"DISPLAY_KEY "):
                    try:
                        code = int(line[len(b"DISPLAY_KEY "):])
                    except ValueError:
                        continue
                    if code in self._control_keys:
                        key = chr(code).lower()
                        if key == "q":
                            self._request_exit()
                        else:
                            self._events.put(key)
            if len(self._output) > 65536:
                self._disconnected = True
        if self._process.poll() is not None:
            self._disconnected = True
        if self._disconnected:
            self._request_exit()

    def submit(self, timestamp_ns, joints, status):
        """Offer the latest model reference, dropping it when the pipe is busy."""
        if self._closed or self._disconnected:
            return False
        if len(joints) != 54 or not all(math.isfinite(value) for value in joints):
            raise ValueError("native display requires 54 finite joint radians")
        prefix = (str(timestamp_ns) + " " + " ".join(format(value, ".17g") for value in joints)).encode("ascii")
        # Keep the complete record <= PIPE_BUF so EAGAIN drops the whole frame,
        # never leaving a partial line in the child's parser.
        remaining = self._pipe_buf - len(prefix) - 2
        if remaining < 0:
            raise ValueError("native display frame exceeds atomic pipe capacity")
        text = " ".join(str(status).split()).encode("ascii", errors="replace")[:remaining]
        message = prefix + b" " + text + b"\n"
        try:
            os.write(self._process.stdin.fileno(), message)
        except BlockingIOError:
            return False
        except OSError:
            self._disconnected = True
            self._request_exit()
            return False
        return True

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self._process.stdin.close()
            try:
                self._process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._process.terminate()
                try:
                    self._process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self._process.kill()
                    self._process.wait(timeout=2)
        finally:
            self._process.stdout.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
