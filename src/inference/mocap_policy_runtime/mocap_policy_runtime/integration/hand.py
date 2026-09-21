"""Bounded synchronous Hand2 solves in the isolated Manus Python environment."""
from __future__ import annotations

import json
import os
from pathlib import Path
import select
import subprocess
import time

import numpy as np

from tianji_runtime.resources import workspace


class HandRetargeter:
    """Own one stateful official solver; errors close it without restart/fallback."""

    def __init__(self, side: str, *, timeout: float = 0.15, startup_timeout: float = 30.0):
        self._process = None
        self._sequence = 0
        self.timeout = float(timeout)
        if side not in ("left", "right"):
            raise ValueError("hand side must be left/right")
        self.side = side
        if not np.isfinite([self.timeout, startup_timeout]).all() or min(self.timeout, startup_timeout) <= 0:
            raise ValueError("hand worker timeouts must be finite and positive")
        root = workspace()
        prefix = root / ".pixi" / "envs" / "manus"
        python = prefix / "bin" / "python"
        worker = Path(__file__).resolve().with_name("hand_worker.py")
        for path in (python, worker):
            if not path.is_file():
                raise FileNotFoundError(f"{path}; install Manus with pixi install and bash bash/build_manus.sh")
        # An allowlist prevents caller overlays, Python hooks, native preload/search
        # paths and ROS settings from crossing the ABI boundary. -I also disables
        # user site-packages and cwd/script-directory import injection.
        environment = {key: value for key, value in os.environ.items()
                       if key in ("HOME", "LANG", "TZ", "TMPDIR", "TMP", "TEMP") or key.startswith("LC_")}
        environment.update(PATH=f"{prefix / 'bin'}:/usr/bin:/bin", CONDA_PREFIX=str(prefix),
                           TIANJI_WORKSPACE=str(root), TIANJI_ENVIRONMENT="manus",
                           PIXI_ENVIRONMENT_NAME="manus")
        try:
            self._process = subprocess.Popen(
                [str(python), "-I", str(worker), side], cwd=root, env=environment,
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None, bufsize=0,
            )
            os.set_blocking(self._process.stdin.fileno(), False)
            os.set_blocking(self._process.stdout.fileno(), False)
            self._exchange({"kind": "init", "sequence": 0, "side": side}, "ready", startup_timeout)
        except BaseException:
            self.close()
            raise

    def _failure(self, reason: str) -> RuntimeError:
        status = self._process.poll() if self._process is not None else None
        return RuntimeError(f"{self.side} hand worker {reason}; sequence {self._sequence}, "
                            f"exit status {status}; see worker stderr")

    def _exchange(self, request: dict, kind: str, timeout: float) -> dict:
        process = self._process
        if process is None or process.poll() is not None:
            raise self._failure("is closed or exited")
        data = (json.dumps(request, allow_nan=False, separators=(",", ":")) + "\n").encode("ascii")
        if len(data) > 8192:
            raise ValueError("hand worker request exceeds 8192-byte bound")
        deadline = time.monotonic() + timeout
        offset = 0
        while offset < len(data):
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([], [process.stdin], [], remaining)[1]:
                raise TimeoutError(f"{self.side} hand worker request timed out")
            try:
                offset += os.write(process.stdin.fileno(), data[offset:])
            except BlockingIOError:
                continue
            except BrokenPipeError as error:
                raise self._failure("closed its request pipe") from error
        response_bytes = bytearray()
        while b"\n" not in response_bytes:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([process.stdout], [], [], remaining)[0]:
                raise TimeoutError(f"{self.side} hand worker response timed out")
            try:
                chunk = os.read(process.stdout.fileno(), 4096)
            except BlockingIOError:
                continue
            if not chunk:
                raise self._failure("closed its response pipe")
            response_bytes.extend(chunk)
            if len(response_bytes) > 16384:
                raise self._failure("response exceeds 16384-byte bound")
        raw, _, remainder = response_bytes.partition(b"\n")
        if remainder:
            raise self._failure("sent unsolicited response data")
        response = json.loads(raw)
        if not isinstance(response, dict) or response.get("sequence") != self._sequence:
            raise self._failure("response sequence mismatch")
        if response.get("kind") == "error":
            raise self._failure(f"failed: {response.get('error', 'missing diagnostic')}")
        if response.get("kind") != kind or response.get("side") != self.side:
            raise self._failure("response kind/side mismatch")
        return response

    def retarget(self, points) -> np.ndarray:
        try:
            points = np.asarray(points, dtype=np.float64)
            if points.shape != (21, 3) or not np.isfinite(points).all():
                raise ValueError("hand keypoints must be finite 21x3 wrist-local metres")
            if np.linalg.norm(points[0]) > 1e-8:
                raise ValueError("hand keypoint zero must be the wrist-local origin")
            self._sequence += 1
            response = self._exchange({"kind": "retarget", "sequence": self._sequence,
                                       "points": points.tolist()}, "joints", self.timeout)
            joints = np.asarray(response.get("joints"), dtype=np.float64)
            if joints.shape != (20,) or not np.isfinite(joints).all():
                raise self._failure("returned invalid joint angles")
            return joints
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        process, self._process = self._process, None
        if process is None:
            return
        if process.stdin is not None:
            process.stdin.close()
        try:
            process.wait(timeout=0.2)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        finally:
            if process.stdout is not None:
                process.stdout.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
