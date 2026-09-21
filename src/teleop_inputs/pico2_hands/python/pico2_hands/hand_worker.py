"""Bounded single-side native retarget client, without publication authority.

Each side owns a worker. A failed worker must be closed, not silently restarted:
the session owner must invalidate input and explicitly reset its control epoch.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import select
import struct
import subprocess
import tempfile
import time

import numpy as np

from .resources import hand_retargeting_root, hand_runtime_launcher, hand_runtime_python

ROOT = Path(__file__).resolve().parent
REQUEST = struct.Struct("<4sBBHQQII126d")
RESPONSE = struct.Struct("<4sBBHQQ40d")


class NativeHandWorker:
    def __init__(self, side, *, timeout_s=1.0, startup_timeout_s=15.0):
        if side not in ("left", "right"):
            raise ValueError("explicit hand side required")
        if any(not math.isfinite(x) or x <= 0 for x in (timeout_s, startup_timeout_s)):
            raise ValueError("timeouts must be finite and positive")
        self.side, self.flag = side, 1 if side == "left" else 2
        self.timeout_s = timeout_s
        self.sequence = self.timestamp = 0
        self.failed = self.closed = False
        python = hand_runtime_python()
        worker = ROOT / "native/build/pico2-hand/tianji_hand_native_worker"
        for path in (python, worker):
            if not path.is_file():
                raise FileNotFoundError(f"build/install the isolated PICO2 Hand2 runtime: {path}")
        self.errors = tempfile.TemporaryFile()
        self.process = None
        try:
            self.process = subprocess.Popen([
                str(python), str(hand_runtime_launcher()),
                "--official-root", str(hand_retargeting_root()),
                "--native-worker", str(worker), "--single-hand-side", side,
                "--startup-handshake"], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=self.errors, bufsize=0)
            os.set_blocking(self.process.stdin.fileno(), False)
            os.set_blocking(self.process.stdout.fileno(), False)
            deadline = time.monotonic() + startup_timeout_s
            line = bytearray()
            while len(line) < 4096:
                line.extend(self._read(1, deadline))
                if line[-1:] == b"\n":
                    break
            else:
                raise RuntimeError("oversized native worker handshake")
            ready = json.loads(line)
            if (ready.get("schema_version") != 1 or ready.get("kind") != "wuji_worker_ready"
                    or ready.get("algorithm") != "official_wuji_hand2"):
                raise RuntimeError("native worker handshake mismatch")
        except BaseException:
            self.failed = True
            self.close()
            raise

    def _read(self, count, deadline):
        result = bytearray()
        while len(result) < count:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self.process.stdout], [], [], remaining)[0]:
                raise TimeoutError("native hand worker response timeout")
            block = os.read(self.process.stdout.fileno(), count - len(result))
            if not block:
                raise RuntimeError("native hand worker response truncated/closed")
            result.extend(block)
        return bytes(result)

    def retarget(self, points, sequence, timestamp_ns):
        if self.closed or self.failed:
            raise RuntimeError("native hand worker closed/failed; explicit session reset required")
        for value in (sequence, timestamp_ns):
            if type(value) is not int or not 0 < value <= (1 << 63) - 1:
                raise ValueError("positive int64 callback sequence and timestamp required")
        if sequence <= self.sequence or timestamp_ns < self.timestamp:
            raise ValueError("callback sequence/timestamp rollback")
        points = np.asarray(points, dtype=np.float64)
        if points.shape != (21, 3) or not np.isfinite(points).all():
            raise ValueError("finite 21x3 official retarget input required")
        request = REQUEST.pack(b"TJWI", 1, self.flag, REQUEST.size, sequence,
                               timestamp_ns, 63, 0, *(points.ravel().tolist() + [0.] * 63))
        deadline = time.monotonic() + self.timeout_s
        try:
            pending = memoryview(request)
            while pending:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or not select.select([], [self.process.stdin], [], remaining)[1]:
                    raise TimeoutError("native hand worker request timeout")
                written = os.write(self.process.stdin.fileno(), pending)
                if written <= 0:
                    raise RuntimeError("native hand worker request closed")
                pending = pending[written:]
            row = RESPONSE.unpack(self._read(RESPONSE.size, deadline))
            if row[:6] != (b"TJHR", 1, self.flag, RESPONSE.size, sequence, timestamp_ns):
                raise RuntimeError("native hand response identity mismatch")
            joints = np.asarray(row[6:], dtype=np.float64)
            if not np.isfinite(joints).all():
                raise RuntimeError("nonfinite native hand result")
            self.sequence, self.timestamp = sequence, timestamp_ns
            return joints[:20].copy() if self.side == "left" else joints[20:].copy()
        except BaseException:
            self.failed = True
            raise

    def close(self):
        if self.closed:
            return
        self.closed = True
        already_failed = self.failed
        forced = False
        process = self.process
        try:
            if process is not None:
                if process.stdin:
                    process.stdin.close()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    forced = True
                    process.terminate()
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=2)
                if process.stdout:
                    process.stdout.close()
        finally:
            self.errors.close()
        if process is not None and (forced or process.returncode != 0):
            self.failed = True
            if not already_failed:
                raise RuntimeError(f"Hand2 worker cleanup failed: exit={process.returncode}; forced={forced}")

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
