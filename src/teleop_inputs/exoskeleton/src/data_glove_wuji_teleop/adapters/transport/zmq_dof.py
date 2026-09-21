"""向 MuJoCo 与真机扇出 hand_dof 的本机 ZMQ 适配器。"""

from __future__ import annotations

import json
import math

from ...domain.hand_target import DOF_ORDER, HandTarget
from .zmq_json import ZmqJsonPublisher


DEFAULT_ZMQ_PORT = {
    "left": 15559,
    "right": 15558,
}


def _decode_hand_target(payload: str, *, expected_hand: str) -> HandTarget:
    """严格解码供真机复用的 canonical hand_dof 消息。"""

    try:
        message = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise ValueError("ZMQ 目标不是合法 JSON") from exc
    if not isinstance(message, dict) or message.get("type") != "hand_dof":
        raise ValueError("ZMQ 真机输入必须是 hand_dof 消息")
    hand = str(message.get("hand", ""))
    if hand != expected_hand:
        raise ValueError(
            f"ZMQ 目标左右手不匹配：请求 {expected_hand}，实际 {hand}"
        )
    try:
        values = tuple(float(value) for value in message["values"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("ZMQ hand_dof values 无效") from exc
    if len(values) != len(DOF_ORDER):
        raise ValueError(
            f"ZMQ hand_dof 必须包含 {len(DOF_ORDER)} 个关节，"
            f"实际 {len(values)}"
        )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("ZMQ hand_dof 必须全部是有限数值")
    try:
        sequence = int(message["sequence"])
        timestamp_ns = int(message["timestamp_ns"])
        dropped = int(message["dropped"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("ZMQ hand_dof 帧元数据无效") from exc
    return HandTarget(
        hand=hand,
        values=values,
        source=str(message.get("source", "dataglove_zmq")),
        sequence=sequence,
        timestamp_ns=timestamp_ns,
        dropped=dropped,
    )


class ZmqDofPublisher(ZmqJsonPublisher):
    """在指定地址向多个订阅者发布最新 hand_dof JSON。"""

    def publish(self, target: HandTarget) -> None:
        message = {
            "type": "hand_dof",
            "hand": target.hand,
            "values": target.values,
            "source": target.source,
            "sequence": target.sequence,
            "timestamp_ns": target.timestamp_ns,
            "dropped": target.dropped,
        }
        self.publish_json(message)


class ZmqTargetSubscriber:
    """同步接收最新 hand_dof，供真机与 MuJoCo 共享同一手套桥。"""

    def __init__(
        self,
        host: str,
        port: int,
        hand: str,
        *,
        timeout_seconds: float = 0.05,
    ) -> None:
        if hand not in DEFAULT_ZMQ_PORT:
            raise ValueError(f"不支持的目标侧别：{hand}")
        if not 1 <= int(port) <= 65535:
            raise ValueError("ZMQ 目标端口必须在 1..65535")
        if timeout_seconds <= 0.0:
            raise ValueError("ZMQ 接收超时必须为正数")
        try:
            import zmq
        except ImportError as exc:
            raise RuntimeError("缺少 pyzmq，请在 pixi 环境运行") from exc

        self._zmq = zmq
        self._hand = hand
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self._socket.setsockopt(zmq.CONFLATE, 1)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.setsockopt(
            zmq.RCVTIMEO,
            max(1, int(timeout_seconds * 1000.0)),
        )
        self._socket.connect(f"tcp://{host}:{port}")
        self._closed = False

    def recv(self) -> HandTarget | None:
        if self._closed:
            raise RuntimeError("ZMQ 目标订阅器已经关闭")
        try:
            payload = self._socket.recv_string()
        except self._zmq.Again:
            return None
        return _decode_hand_target(payload, expected_hand=self._hand)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._socket.close(linger=0)
        self._context.term()
