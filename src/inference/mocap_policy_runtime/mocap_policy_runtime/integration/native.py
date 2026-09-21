"""Bounded, synchronous IPC to the repository's native 200 Hz QP controller."""
from __future__ import annotations

import json
import os
from pathlib import Path
import select
import signal
import subprocess
import time

import numpy as np

from tianji_description.home_config import load_controller_posture
from tianji_runtime.resources import (
    controller_profile,
    config_path as config_file,
    native_executable,
    package_share,
)

from ..data.geometry import compose_pose, invert_pose

SIDES = ("left", "right")


def robot_config() -> dict:
    """Site configuration: device identity, ports and safety bounds."""
    return json.loads(config_file("robot.json").read_text())


def model_asset(name: str) -> Path:
    """MuJoCo model from the active installed description package."""
    return package_share("tianji_description", "models", name)


def finite_vector(value, count: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (count,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {count} finite values")
    return result


def pose_xyzw(value, name: str = "pose") -> np.ndarray:
    result = finite_vector(value, 7, name).copy()
    norm = np.linalg.norm(result[3:])
    if abs(norm - 1.0) > 1e-3:
        raise ValueError(f"{name} must contain a unit xyzw quaternion")
    result[3:] /= norm
    return result


def safety_limits() -> tuple[np.ndarray, np.ndarray]:
    config = robot_config()
    groups = (config["safety"][name] for name in ("arms", "left_hand", "right_hand"))
    pairs = [(group["lower_rad"], group["upper_rad"]) for group in groups]
    lower = finite_vector(np.concatenate([pair[0] for pair in pairs]), 54, "lower limits")
    upper = finite_vector(np.concatenate([pair[1] for pair in pairs]), 54, "upper limits")
    if np.any(lower >= upper):
        raise ValueError("invalid target safety joint limits")
    return lower, upper


def configured_home() -> np.ndarray:
    site = robot_config()
    controller_path = Path(site["controller_config"])
    if not controller_path.is_absolute():
        controller_path = controller_profile(str(controller_path))
    posture = load_controller_posture(controller_path)
    if posture is None:
        raise ValueError(f"{controller_path} must specify its initial home posture")
    return finite_vector(np.concatenate(posture), 14, "home")


class NativeIK:
    """One ``step`` is exactly 5 ms of controller time, not a converged IK solve.

    The owner must call at 200 Hz, repeating a 50 Hz policy target four times.
    Any IPC failure or requested-arm solver rejection closes the worker; there
    is no automatic restart or fallback joint output. No hardware is contacted.
    """

    def __init__(self, model=None, config=None, initial_positions=None, *, binary=None,
                 timeout: float = 0.15, startup_timeout: float = 30.0):
        self._process = None
        self._buffer = bytearray()
        self._sequence = 0
        self.timeout = float(timeout)
        if not np.isfinite([self.timeout, startup_timeout]).all() or min(self.timeout, startup_timeout) <= 0:
            raise ValueError("worker timeouts must be finite and positive")
        self.model_path = (Path(model) if model is not None else model_asset("marvin_m6_wuji2.xml")).resolve()
        self.config_path = (Path(config) if config is not None else controller_profile("qp_ik_cartesian_otg_velocity.yaml")).resolve()
        executable = Path(binary) if binary is not None else native_executable("mocap_tcp_worker")
        for path in (self.model_path, self.config_path, executable):
            if not path.is_file():
                raise FileNotFoundError(f"{path}; build the workspace with: pixi run build")
        self.lower, self.upper = safety_limits()
        q = configured_home() if initial_positions is None else finite_vector(initial_positions, 14, "initial arm positions")
        self._validate_q(q)
        try:
            self._process = subprocess.Popen(
                [str(executable.resolve()), str(self.config_path), str(self.model_path)],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None, bufsize=0,
            )
            os.set_blocking(self._process.stdin.fileno(), False)
            os.set_blocking(self._process.stdout.fileno(), False)
            response = self._exchange("INIT " + self._numbers(q), startup_timeout)
            self._check_header(response, "ready")
            self.lower = np.maximum(self.lower, finite_vector(response["lower"], 54, "native lower limits"))
            self.upper = np.minimum(self.upper, finite_vector(response["upper"], 54, "native upper limits"))
            if np.any(self.lower >= self.upper):
                raise ValueError("model and safety joint limits do not overlap")
            self._update_state(response)
            if not np.allclose(self.current_positions, q, rtol=0, atol=1e-12):
                raise RuntimeError("native initial joint state changed")
            self.capture_home()
            self.base_poses = {side: pose_xyzw(response["base"][side], f"{side} base") for side in SIDES}
            self.tcp_to_wrist = {side: compose_pose(invert_pose(self.home_tcp[side]), self.home_wrist[side]) for side in SIDES}
            self.flange_to_tcp = {
                side: compose_pose(invert_pose(pose_xyzw(response["flange"][side], f"{side} flange")), self.home_tcp[side])
                for side in SIDES
            }
        except BaseException:
            self.close()
            raise

    @staticmethod
    def _numbers(values) -> str:
        return " ".join(format(float(value), ".17g") for value in values)

    def _validate_q(self, q):
        if np.any(q < self.lower[:14]) or np.any(q > self.upper[:14]):
            raise ValueError("arm positions exceed model or configured safety limits")

    def _check_header(self, response, kind):
        if response.get("sequence") != self._sequence or response.get("kind") != kind:
            raise RuntimeError("native response kind/sequence mismatch")

    def _update_state(self, response):
        q = finite_vector(response["q"], 14, "native arm result")
        self._validate_q(q)
        tcp = {side: pose_xyzw(response["tcp"][side], f"{side} TCP") for side in SIDES}
        wrist = {side: pose_xyzw(response["wrist"][side], f"{side} wrist") for side in SIDES}
        self.current_positions, self.current_tcp, self.current_wrist = q.copy(), tcp, wrist

    def _worker_error(self, reason: str, request: str) -> RuntimeError:
        process = self._process
        status = process.poll() if process is not None else None
        detail = ""
        if status is not None:
            detail = f"; exit status {status}"
            if status < 0:
                detail += f" ({signal.Signals(-status).name})"
        return RuntimeError(
            f"native IK worker {reason} during {request.split()[0]} "
            f"sequence {self._sequence}{detail}; see native stderr"
        )

    def _exchange(self, request: str, timeout: float) -> dict:
        process = self._process
        if process is None or process.poll() is not None:
            raise self._worker_error("is closed or exited", request)
        data = (request + "\n").encode("ascii")
        if len(data) > 2049:
            raise ValueError("native request exceeds 2048-byte bound")
        deadline = time.monotonic() + timeout
        offset = 0
        while offset < len(data):
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([], [process.stdin], [], remaining)[1]:
                raise TimeoutError("native IK request timed out")
            try:
                offset += os.write(process.stdin.fileno(), data[offset:])
            except BlockingIOError:
                continue
            except BrokenPipeError as error:
                raise self._worker_error("closed its request pipe", request) from error
        while b"\n" not in self._buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([process.stdout], [], [], remaining)[0]:
                raise TimeoutError("native IK response timed out")
            try:
                chunk = os.read(process.stdout.fileno(), 4096)
            except BlockingIOError:
                continue
            if not chunk:
                raise self._worker_error("closed its response pipe", request)
            self._buffer.extend(chunk)
            if len(self._buffer) > 16384:
                raise RuntimeError("native response exceeds 16384-byte bound")
        raw, _, remainder = self._buffer.partition(b"\n")
        self._buffer = bytearray(remainder)
        response = json.loads(raw)
        if not isinstance(response, dict):
            raise RuntimeError("native response is not an object")
        if response.get("kind") == "error":
            message = response.get("error")
            if not isinstance(message, str) or not message:
                raise RuntimeError("native error response has no diagnostic")
            raise RuntimeError(
                f"native IK {request.split()[0]} sequence {self._sequence} failed: {message}"
            )
        return response

    def step(self, targets: dict[str, np.ndarray]) -> np.ndarray:
        """Advance requested arms; omitted arms hold with cleared motion history."""
        if set(targets) - set(SIDES):
            raise ValueError("TCP target sides must be left/right")
        poses = {side: pose_xyzw(value, f"{side} target") for side, value in targets.items()}
        mask = sum(1 << i for i, side in enumerate(SIDES) if side in poses)
        self._sequence += 1
        request = f"STEP {self._sequence} {mask} " + " ".join(self._numbers(poses[side]) for side in SIDES if side in poses)
        try:
            response = self._exchange(request, self.timeout)
            self._check_header(response, "step")
            accepted = response.get("accepted")
            if not isinstance(accepted, list) or len(accepted) != 2 or any(type(value) is not bool for value in accepted):
                raise RuntimeError("native response has invalid acceptance flags")
            if any(not accepted[i] for i, side in enumerate(SIDES) if side in poses):
                raise RuntimeError(f"native IK rejected target: hold={response.get('hold_reason')}, solver={response.get('solver_status')}")
            self._update_state(response)
            return self.current_positions.copy()
        except BaseException:
            self.close()
            raise

    def synchronize(self, positions) -> None:
        """Reseed command-model state when changing from direct joints to IK."""
        q = finite_vector(positions, 14, "synchronized positions")
        self._validate_q(q)
        self._sequence += 1
        try:
            response = self._exchange(f"SYNC {self._sequence} " + self._numbers(q), self.timeout)
            self._check_header(response, "sync")
            self._update_state(response)
        except BaseException:
            self.close()
            raise

    def capture_home(self) -> None:
        """Explicitly capture current FK after an owner's supervised Home move."""
        self.home_positions = self.current_positions.copy()
        self.home_tcp = {side: self.current_tcp[side].copy() for side in SIDES}
        self.home_wrist = {side: self.current_wrist[side].copy() for side in SIDES}

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
