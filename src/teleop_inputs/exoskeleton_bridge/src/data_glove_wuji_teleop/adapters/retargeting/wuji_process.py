"""官方 Wuji retarget worker 的常驻 JSON-line 进程传输。"""

from __future__ import annotations

import argparse
import os
import json
import math
import subprocess
import threading
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FutureTimeout
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import IO, Protocol

from ...project import PROJECT_ROOT
from ...profiles.wuji_v2.retargeting import validate_retargeting_resources
from .wuji_official import OfficialWujiRetargetAdapter


class TextProcess(Protocol):
    stdin: IO[str] | None
    stdout: IO[str] | None

    def poll(self) -> int | None: ...

    def wait(self, timeout: float | None = None) -> int: ...

    def terminate(self) -> None: ...

    def kill(self) -> None: ...


class JsonLineProcessTransport:
    """通过一问一答 JSON 行协议复用一个外部 Python 求解进程。"""

    def __init__(
        self,
        process: TextProcess,
        *,
        response_timeout: float = 1.0,
        shutdown_timeout: float = 2.0,
    ) -> None:
        if process.stdin is None or process.stdout is None:
            raise ValueError("retarget worker 必须提供 stdin/stdout 管道")
        self._process = process
        self._stdin = process.stdin
        self._stdout = process.stdout
        self._lock = threading.Lock()
        self._closed = False
        self._response_timeout = float(response_timeout)
        self._shutdown_timeout = float(shutdown_timeout)
        if self._response_timeout <= 0.0 or self._shutdown_timeout <= 0.0:
            raise ValueError("worker 超时必须是正数")
        self._reader = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="wuji-retarget-reader",
        )

    @classmethod
    def launch(
        cls,
        command: Sequence[str],
        *,
        env: Mapping[str, str],
        cwd: str | Path,
        response_timeout: float = 1.0,
        shutdown_timeout: float = 2.0,
        start_new_session: bool = False,
    ) -> "JsonLineProcessTransport":
        process = subprocess.Popen(
            tuple(str(value) for value in command),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=None,
            text=True,
            bufsize=1,
            env=dict(env),
            cwd=str(cwd),
            start_new_session=bool(start_new_session),
        )
        return cls(
            process,
            response_timeout=response_timeout,
            shutdown_timeout=shutdown_timeout,
        )

    def wait_ready(self, *, timeout: float = 30.0) -> None:
        """等待初始化完成；不占用或改变逐帧响应时限。"""
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("worker 初始化超时必须是有限正数")
        with self._lock:
            if self._closed:
                raise RuntimeError("retarget worker 已关闭")
            try:
                if self._read_response_locked(timeout) != {"ready": True}:
                    raise RuntimeError("retarget worker 未返回初始化就绪消息")
            except BaseException:
                self._stop_process_locked()
                raise

    def request(self, payload: dict[str, object]) -> dict[str, object]:
        with self._lock:
            if self._closed:
                raise RuntimeError("retarget worker 已关闭")
            return_code = self._process.poll()
            if return_code is not None:
                raise RuntimeError(
                    f"retarget worker 已退出，状态码 {return_code}"
                )
            self._stdin.write(
                json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
                + "\n"
            )
            self._stdin.flush()
            return self._read_response_locked(self._response_timeout)

    def _read_response_locked(self, timeout: float) -> dict[str, object]:
        future = self._reader.submit(self._stdout.readline)
        try:
            response_line = future.result(timeout=timeout)
        except FutureTimeout as exc:
            self._stop_process_locked()
            raise TimeoutError(
                f"retarget worker 响应超时：{timeout:.3f}s"
            ) from exc
        if not response_line:
            self._stop_process_locked()
            raise EOFError("retarget worker 未返回结果")
        try:
            response = json.loads(response_line)
        except json.JSONDecodeError as exc:
            self._stop_process_locked()
            raise RuntimeError("retarget worker 返回了无效 JSON") from exc
        if not isinstance(response, dict):
            self._stop_process_locked()
            raise RuntimeError("retarget worker 响应根节点必须是对象")
        return response

    def _stop_process_locked(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._stdin.close()
        except OSError:
            pass
        try:
            self._process.wait(timeout=self._shutdown_timeout)
        except subprocess.TimeoutExpired:
            self._process.terminate()
            try:
                self._process.wait(timeout=self._shutdown_timeout)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=self._shutdown_timeout)
        self._reader.shutdown(wait=False, cancel_futures=True)

    def close(self) -> None:
        with self._lock:
            self._stop_process_locked()


def launch_official_adapter(
    args: argparse.Namespace,
    *,
    response_timeout: float = 1.0,
    start_new_session: bool = False,
) -> OfficialWujiRetargetAdapter:
    """供在线仿真、回放和外部UDP输出共用的官方求解进程启动入口。"""
    python_path = Path(args.worker_python).resolve()
    checkout = Path(args.wuji_checkout).resolve()
    config = validate_retargeting_resources(
        args.hand, mano_morphology=args.mano_morphology, wuji_config=args.wuji_config,
    )
    for label, path in (
        ("worker Python", python_path),
        ("官方 checkout", checkout),
        ("官方配置", config),
    ):
        if not path.exists():
            raise FileNotFoundError(f"{label} 不存在：{path}")
    command = [
        str(python_path),
        "-m",
        "data_glove_wuji_teleop.adapters.retargeting.wuji_worker",
        "--checkout",
        str(checkout),
        "--config",
        str(config),
        "--hand",
        args.hand,
        "--pinch-alpha-max",
        str(args.pinch_alpha_max),
    ]
    if args.pinch_d1_cm is not None:
        command.extend(("--pinch-d1-cm", str(args.pinch_d1_cm)))
    if args.wuji_urdf:
        command.extend(("--robot-urdf", str(Path(args.wuji_urdf).resolve())))
    environment = os.environ.copy()
    source_root = str(PROJECT_ROOT / "src")
    current_pythonpath = environment.get("PYTHONPATH", "")
    environment["PYTHONPATH"] = (
        source_root
        if not current_pythonpath
        else f"{source_root}{os.pathsep}{current_pythonpath}"
    )
    transport = JsonLineProcessTransport.launch(
        command,
        env=environment,
        cwd=PROJECT_ROOT,
        response_timeout=response_timeout,
        start_new_session=start_new_session,
    )
    transport.wait_ready()
    return OfficialWujiRetargetAdapter(transport)
