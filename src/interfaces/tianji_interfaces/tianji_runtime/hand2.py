"""Client for the shared Hand2 retargeting service (21 landmarks -> 20 joints).

Every input link uses this client instead of vendoring the solver: the client
spawns the workspace `manus` interpreter running
:mod:`wuji_retargeting.worker` and speaks the bounded line-delimited JSON
protocol documented there. One instance owns one side's stateful solver; a
failure closes it and is never masked by a previous pose.

The module needs only the standard library and numpy, so a ROS-free environment
(the data-glove checkout) can import it as long as the workspace
`src/interfaces/tianji_interfaces` directory is on PYTHONPATH.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import select
import subprocess
import time

import numpy as np

from .resources import workspace


#: Module executed in the service interpreter.
SERVICE_MODULE = "wuji_retargeting.worker"
#: Environment that owns the reviewed solver ABI.
SERVICE_ENVIRONMENT = "manus"
#: Only these variables cross into the service; everything else (ROS setup,
#: Python hooks, native preload/search paths) would change the solver's ABI.
_PASSTHROUGH = ("HOME", "LANG", "TZ", "TMPDIR", "TMP", "TEMP")
MAX_REQUEST_BYTES = 8192
MAX_RESPONSE_BYTES = 16384


class Hand2ServiceError(RuntimeError):
    """The service refused, died or answered out of contract."""


class Hand2Retargeter:
    """One side's stateful Hand2 solver, hosted by the shared service process.

    Args:
        side: ``"left"`` or ``"right"``.
        timeout: per-request deadline in seconds (solve + transport).
        startup_timeout: deadline for the service's ``ready`` reply.
        config: retargeting config override; the service's bundled per-side
            config is used when omitted.
        python: service interpreter override (tests inject a fake service).
    """

    def __init__(self, side: str, *, timeout: float = 0.15, startup_timeout: float = 30.0,
                 config: str | Path | None = None, python: str | Path | None = None):
        self._process = None
        self._sequence = 0
        self.timeout = float(timeout)
        if side not in ("left", "right"):
            raise ValueError("hand side must be left/right")
        self.side = side
        if not np.isfinite([self.timeout, startup_timeout]).all() or min(self.timeout, startup_timeout) <= 0:
            raise ValueError("hand service timeouts must be finite and positive")
        root = workspace()
        prefix = root / ".pixi" / "envs" / SERVICE_ENVIRONMENT
        interpreter = Path(python) if python is not None else prefix / "bin" / "python"
        if not interpreter.is_file():
            raise FileNotFoundError(
                f"{interpreter}; install the retargeting service with "
                "'bash bash/install.sh' (or run build_runtime.py in the "
                f"{SERVICE_ENVIRONMENT} environment)")
        command = [str(interpreter), "-I", "-m", SERVICE_MODULE, side]
        if config is not None:
            command += ["--config", str(config)]
        environment = {key: value for key, value in os.environ.items()
                       if key in _PASSTHROUGH or key.startswith("LC_")}
        environment.update(PATH=f"{prefix / 'bin'}:/usr/bin:/bin", CONDA_PREFIX=str(prefix),
                           TIANJI_WORKSPACE=str(root), TIANJI_ENVIRONMENT=SERVICE_ENVIRONMENT,
                           PIXI_ENVIRONMENT_NAME=SERVICE_ENVIRONMENT)
        try:
            self._process = subprocess.Popen(
                command, cwd=root, env=environment,
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None, bufsize=0,
            )
            os.set_blocking(self._process.stdin.fileno(), False)
            os.set_blocking(self._process.stdout.fileno(), False)
            self._exchange({"kind": "init", "sequence": 0, "side": side}, "ready",
                           startup_timeout)
        except BaseException:
            self.close()
            raise

    def _failure(self, reason: str) -> Hand2ServiceError:
        status = self._process.poll() if self._process is not None else None
        return Hand2ServiceError(f"{self.side} hand service {reason}; sequence {self._sequence}, "
                                 f"exit status {status}; see service stderr")

    def _exchange(self, request: dict, kind: str, timeout: float) -> dict:
        process = self._process
        if process is None or process.poll() is not None:
            raise self._failure("is closed or exited")
        data = (json.dumps(request, allow_nan=False, separators=(",", ":")) + "\n").encode("ascii")
        if len(data) > MAX_REQUEST_BYTES:
            raise ValueError("hand service request exceeds the size bound")
        deadline = time.monotonic() + timeout
        offset = 0
        while offset < len(data):
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([], [process.stdin], [], remaining)[1]:
                raise TimeoutError(f"{self.side} hand service request timed out")
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
                raise TimeoutError(f"{self.side} hand service response timed out")
            try:
                chunk = os.read(process.stdout.fileno(), 4096)
            except BlockingIOError:
                continue
            if not chunk:
                raise self._failure("closed its response pipe")
            response_bytes.extend(chunk)
            if len(response_bytes) > MAX_RESPONSE_BYTES:
                raise self._failure("response exceeds the size bound")
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
        """Solve one landmark frame into 20 Hand2 joints in firmware order."""
        try:
            points = np.asarray(points, dtype=np.float64)
            if points.shape != (21, 3) or not np.isfinite(points).all():
                raise ValueError("hand keypoints must be finite 21x3")
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
