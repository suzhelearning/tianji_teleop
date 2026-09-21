"""低延迟本机 ZMQ JSON 发布原语。"""

from __future__ import annotations

import json
import time
from collections.abc import Mapping


class ZmqJsonPublisher:
    """只保留最新消息的本机 PUB socket。"""

    def __init__(self, bind_host: str, port: int) -> None:
        try:
            import zmq
        except ImportError as exc:
            raise RuntimeError("缺少 pyzmq，请在 pixi 环境运行") from exc

        self._zmq = zmq
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.PUB)
        self._socket.setsockopt(zmq.SNDHWM, 1)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.bind(f"tcp://{bind_host}:{port}")
        time.sleep(0.2)

    def publish_json(self, message: Mapping[str, object]) -> None:
        try:
            self._socket.send_string(
                json.dumps(message, separators=(",", ":")),
                flags=self._zmq.NOBLOCK,
            )
        except self._zmq.Again:
            pass

    def close(self) -> None:
        self._socket.close(linger=0)
        self._context.term()
