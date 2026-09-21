"""Bounded local-pipe client for simulation-only V131, not an executor.

Targets are Base_L/R -> TCP_Link_L/R, metres and xyzw quaternions. Seeds and
results are Joint1..7_L followed by Joint1..7_R, radians. Single owner only.
Reset is a numerical epoch barrier, NOT proof of Home/standstill authorization.
"""
import json
import math
import os
from pathlib import Path
import select
import struct
import subprocess
import tempfile
import threading
import time

import numpy as np

ROOT = Path(__file__).resolve().parent
REQUEST = struct.Struct("<4sBBHQQ31d")
HEADER = struct.Struct("<4sBBHQQ")
SIDE = struct.Struct("<I17d64s")
RESPONSE_SIZE = HEADER.size + 2 * SIDE.size


class NativeIkWorker:
    def _command(self):
        return [str(ROOT / "native/build/pico2-v131/pico2_v131_worker"),
                str(ROOT / "native/models/marvin_m6_s_ccs_696_v4.urdf"),
                str(ROOT / "native/models/marvin_m6_qp_pico_fast_kinematics.xml")]

    ready_kind = "pico2_ik_ready"

    def __init__(self, *, timeout_s=1.0):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("finite positive timeout required")
        self.timeout_s = timeout_s
        self.epoch = self.sequence = 0
        self.failed = self.closed = False
        self.lock = threading.Lock()
        self.errors = tempfile.TemporaryFile()
        self.process = None
        try:
            self.process = subprocess.Popen(self._command(),
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.errors, bufsize=0)
            os.set_blocking(self.process.stdin.fileno(), False)
            os.set_blocking(self.process.stdout.fileno(), False)
            deadline = time.monotonic() + 15
            line = bytearray()
            while len(line) < 4096:
                line.extend(self._read(1, deadline))
                if line[-1:] == b"\n":
                    break
            else:
                raise RuntimeError("oversized IK handshake")
            ready = json.loads(line)
            if ready != dict(schema_version=1, kind=self.ready_kind, simulation_only=True):
                raise RuntimeError("IK handshake mismatch")
        except BaseException:
            self.failed = True
            self.close()
            raise

    def _read(self, count, deadline):
        result = bytearray()
        while len(result) < count:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self.process.stdout], [], [], remaining)[0]:
                raise TimeoutError("IK response timeout")
            block = os.read(self.process.stdout.fileno(), count - len(result))
            if not block:
                raise RuntimeError("IK worker closed/truncated response")
            result.extend(block)
        return result

    def _exchange(self, operation, seeds, targets, source, received, now):
        if not self.lock.acquire(blocking=False):
            raise RuntimeError("IK worker requires a single owner")
        try:
            if self.closed or self.failed:
                raise RuntimeError("IK worker unavailable; explicit replacement required")
            seeds, targets = np.asarray(seeds, dtype=float), np.asarray(targets, dtype=float)
            if seeds.shape != (2, 7) or targets.shape != (2, 7):
                raise ValueError("bilateral 2x7 seed/pose required")
            if not np.isfinite(seeds).all() or not np.isfinite(targets).all():
                raise ValueError("finite seed/pose required")
            if not all(math.isfinite(v) and v >= 0 for v in (source, received, now)) or received > now:
                raise ValueError("invalid IK timestamps")
            if operation == 2:
                if self.epoch == 0:
                    raise ValueError("explicit numerical reset required before solve")
                if np.any(np.linalg.norm(targets[:, 3:], axis=1) < 1e-12):
                    raise ValueError("nonzero target quaternion required")
            sequence = self.sequence + 1
            epoch = self.epoch + (operation == 1)
            message = REQUEST.pack(b"P2IQ", 1, operation, REQUEST.size, sequence, epoch,
                                   source, received, now, *seeds.ravel(), *targets.ravel())
            try:
                deadline = time.monotonic() + self.timeout_s
                pending = memoryview(message)
                while pending:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0 or not select.select([], [self.process.stdin], [], remaining)[1]:
                        raise TimeoutError("IK request timeout")
                    written = os.write(self.process.stdin.fileno(), pending)
                    if written <= 0:
                        raise RuntimeError("IK request pipe closed")
                    pending = pending[written:]
                response = self._read(RESPONSE_SIZE, deadline)
                if HEADER.unpack_from(response) != (b"P2IR", 1, operation, RESPONSE_SIZE, sequence, epoch):
                    raise RuntimeError("IK response association mismatch")
                result = {}
                for i, side in enumerate(("left", "right")):
                    row = SIDE.unpack_from(response, HEADER.size + SIDE.size * i)
                    if row[0] & ~7 or not np.isfinite(row[1:18]).all():
                        raise RuntimeError("invalid IK result flags/numerics")
                    result[side] = dict(accepted=bool(row[0] & 1), converged=bool(row[0] & 2),
                        model_state_only=bool(row[0] & 4), joints=np.array(row[1:8]),
                        achieved_pose=np.array(row[8:15]), position_error_m=row[15],
                        orientation_error_rad=row[16], solve_time_ms=row[17],
                        status=row[18].split(b"\0", 1)[0].decode("ascii"))
                self.sequence, self.epoch = sequence, epoch
                return result
            except BaseException:
                self.failed = True
                raise
        finally:
            self.lock.release()

    def reset(self, seeds):
        return self._exchange(1, seeds, np.zeros((2, 7)), 0., 0., 0.)

    def forward(self, seeds):
        return self._exchange(3, seeds, np.zeros((2, 7)), 0., 0., 0.)

    def solve(self, seeds, targets, *, source_time, received_time, now):
        return self._exchange(2, seeds, targets, source_time, received_time, now)

    def close(self):
        if self.closed:
            return
        self.closed = True
        already_failed = self.failed
        forced = False
        try:
            if self.process is not None:
                self.process.stdin.close()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    forced = True
                    self.process.terminate()
                    try:
                        self.process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        self.process.kill()
                        self.process.wait(timeout=2)
                self.process.stdout.close()
        finally:
            self.errors.close()
        if self.process is not None and (forced or self.process.returncode != 0):
            self.failed = True
            if not already_failed:
                raise RuntimeError(f"IK worker cleanup failed: exit={self.process.returncode}; forced={forced}")

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
