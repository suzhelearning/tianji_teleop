"""MuJoCo 查看器共享的语义 20-DOF ZMQ 订阅器。"""

from __future__ import annotations

import json
import threading

from ...domain.hand_target import DOF_ORDER


class ZmqDofSubscriber:
    """后台订阅最新的单手语义 20-DOF 目标。"""

    def __init__(self, host: str, port: int, side: str):
        try:
            import zmq
        except ImportError as exc:
            raise SystemExit("缺少 pyzmq，请在 Pixi 环境中运行") from exc

        self._zmq = zmq
        self._side = side
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self._socket.setsockopt(zmq.RCVTIMEO, 100)
        self._socket.setsockopt(zmq.CONFLATE, 1)
        self._socket.connect(f"tcp://{host}:{port}")
        self._dof: dict[str, float] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _decode(self, payload: str) -> dict[str, float] | None:
        try:
            message = json.loads(payload)
        except json.JSONDecodeError:
            return None
        if not isinstance(message, dict):
            return None

        if message.get("type") == "hand_dof":
            if message.get("hand", self._side) != self._side:
                return None
            return self._values_to_dof(message.get("values", []))
        if message.get("type") == "hand_dof_both":
            return self._values_to_dof(message.get(self._side, []))
        if any(name in message for name in DOF_ORDER):
            try:
                return {
                    name: float(message.get(name, 0.0))
                    for name in DOF_ORDER
                }
            except (TypeError, ValueError):
                return None
        return None

    @staticmethod
    def _values_to_dof(values) -> dict[str, float] | None:
        try:
            if len(values) < len(DOF_ORDER):
                return None
            return {
                name: float(values[index])
                for index, name in enumerate(DOF_ORDER)
            }
        except (TypeError, ValueError):
            return None

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                payload = self._socket.recv_string()
            except self._zmq.Again:
                continue
            except self._zmq.ZMQError:
                if self._stop.is_set():
                    break
                continue

            dof = self._decode(payload)
            if dof is not None:
                with self._lock:
                    self._dof = dof

    def get(self) -> dict[str, float]:
        with self._lock:
            return dict(self._dof)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._socket.close(0)
        self._context.term()
