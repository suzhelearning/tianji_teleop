"""按机器人模型坐标向本机控制器直接发送 TJH2 v2，不连接机器人。"""

from __future__ import annotations

import ipaddress
import math
import socket
import time
from collections.abc import Sequence
from dataclasses import dataclass
from numbers import Real

import mujoco
from tianji import hand_protocol

from ...profiles.wuji_v2.model import full_joint_name


def _robot_angles(values: Sequence[float], label: str) -> tuple[float, ...]:
    try:
        if len(values) != 20:
            raise ValueError(f"{label} 必须恰好包含 20 个机器人弧度值")
        angles = []
        for value in values:
            if isinstance(value, bool) or not isinstance(value, Real):
                raise ValueError(f"{label} 角度必须是非布尔实数")
            angle = float(value)
            if not math.isfinite(angle):
                raise ValueError(f"{label} 角度必须全部有限")
            angles.append(angle)
    except (TypeError, OverflowError) as exc:
        raise ValueError(f"{label} 必须是 20 个有限机器人弧度值") from exc
    return tuple(angles)


@dataclass(frozen=True)
class TianjiJointBinding:
    """绑定官方结果的名称顺序，只重排 qpos；不使用语义映射 scale。"""

    source_joint_names: tuple[str, ...]
    _source_indices: tuple[int, ...]

    @classmethod
    def from_model(
        cls,
        hand: str,
        joint_names: Sequence[str],
        model: mujoco.MjModel,
    ) -> TianjiJointBinding:
        expected = tuple(full_joint_name(hand, name) for name in hand_protocol.JOINT_STEMS)
        names = tuple(joint_names)
        if (
            len(names) != 20
            or not all(isinstance(name, str) for name in names)
            or len(set(names)) != 20
            or set(names) != set(expected)
        ):
            raise ValueError(f"{hand} 官方结果必须包含协议对应的 20 个唯一模型关节名")

        for name in expected:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise ValueError(f"模型缺少关节：{name}")
            if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
                raise ValueError(f"机器人弧度槽位必须对应单自由度转动关节：{name}")
            actuator_ids = [
                index
                for index in range(model.nu)
                if model.actuator_trntype[index] == mujoco.mjtTrn.mjTRN_JOINT
                and int(model.actuator_trnid[index, 0]) == joint_id
            ]
            if len(actuator_ids) != 1:
                raise ValueError(f"关节必须恰好绑定一个直接位置执行器：{name}")
            actuator_id = actuator_ids[0]
            gain = float(model.actuator_gainprm[actuator_id, 0])
            bias = model.actuator_biasprm[actuator_id]
            gear = model.actuator_gear[actuator_id]
            if (
                model.actuator_gaintype[actuator_id] != mujoco.mjtGain.mjGAIN_FIXED
                or model.actuator_biastype[actuator_id] != mujoco.mjtBias.mjBIAS_AFFINE
                or not math.isfinite(gain)
                or gain <= 0
                or bias[0] != 0
                or bias[1] != -gain
                or not math.isfinite(float(bias[2]))
                or bias[2] > 0
                or gear[0] != 1
                or any(component != 0 for component in gear[1:])
            ):
                raise ValueError(f"执行器必须以原始关节弧度为位置目标：{name}")
            if not model.jnt_limited[joint_id] or not model.actuator_ctrllimited[actuator_id]:
                raise ValueError(f"关节与位置执行器必须都具备有效限位：{name}")
            joint_lower, joint_upper = map(float, model.jnt_range[joint_id])
            ctrl_lower, ctrl_upper = map(float, model.actuator_ctrlrange[actuator_id])
            if (
                not all(math.isfinite(v) for v in (joint_lower, joint_upper, ctrl_lower, ctrl_upper))
                or joint_lower >= joint_upper
                or ctrl_lower >= ctrl_upper
            ):
                raise ValueError(f"关节或位置执行器限位无效：{name}")
            lower, upper = max(joint_lower, ctrl_lower), min(joint_upper, ctrl_upper)
            if lower >= upper:
                raise ValueError(f"关节与执行器限位没有有效交集：{name}")
        return cls(names, tuple(names.index(name) for name in expected))

    def convert(self, qpos: Sequence[float]) -> tuple[float, ...]:
        """校验角度格式并按协议重排，不检查或裁剪角度范围。"""

        source = _robot_angles(qpos, "官方 qpos")
        return tuple(source[index] for index in self._source_indices)


class TianjiHandSender:
    """单线程串行发送左右手新结果；返回序号只表示本机 sendto 完成，无 ACK。"""

    def __init__(self, host: str = "127.0.0.1", port: int = 16000) -> None:
        if not isinstance(host, str):
            raise ValueError("host 必须是本机回环 IPv4 地址")
        try:
            address = ipaddress.IPv4Address(host)
        except ipaddress.AddressValueError as exc:
            raise ValueError("host 必须是本机回环 IPv4 地址，不接受主机名或 IPv6") from exc
        if not address.is_loopback:
            raise ValueError("TJH2 源时间戳要求同机单调时钟；host 必须是回环 IPv4 地址")
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise ValueError("UDP 端口必须是 1..65535 的整数")
        self._sequence = time.monotonic_ns()
        self._check_sequence(self._sequence)
        self._address = (str(address), port)
        self._targets = {"left": (None, 0), "right": (None, 0)}
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._closed = False

    @staticmethod
    def _check_sequence(sequence: int) -> None:
        if not 1 <= sequence < 2**64:
            raise ValueError("Tianji sequence 必须满足 1 <= sequence < 2^64")

    def invalidate(self, hand: str) -> None:
        """清除失败侧，不发包；后续健康侧包将该侧标为无效。"""
        if hand not in self._targets:
            raise ValueError("hand 必须是 left 或 right")
        self._targets[hand] = (None, 0)

    def send(
        self,
        *,
        left: Sequence[float] | None = None,
        right: Sequence[float] | None = None,
        left_timestamp_ns: int = 0,
        right_timestamp_ns: int = 0,
    ) -> int | None:
        """仅新结果触发发包；未更新侧保留原始测量时间，绝不刷新缓存年龄。"""

        if left is None and right is None:
            return None
        if self._closed:
            raise RuntimeError("Tianji UDP 发送器已关闭")
        targets = self._targets.copy()
        for side, values, sampled_ns in (
            ("left", left, left_timestamp_ns), ("right", right, right_timestamp_ns),
        ):
            if isinstance(sampled_ns, bool) or not isinstance(sampled_ns, int):
                raise ValueError(f"{side} 测量时间必须是本机单调纳秒整数")
            if values is not None:
                targets[side] = (_robot_angles(values, side), sampled_ns)
            elif sampled_ns != 0:
                raise ValueError(f"{side} 没有新结果时不能提供测量时间")
        timestamp_ns = time.monotonic_ns()
        sequence = max(self._sequence + 1, timestamp_ns)
        self._check_sequence(sequence)
        left_values, left_ns = targets["left"]
        right_values, right_ns = targets["right"]
        payload = hand_protocol.encode_packet(
            sequence, timestamp_ns, left=left_values, right=right_values,
            left_timestamp_ns=left_ns, right_timestamp_ns=right_ns,
        )
        # 全部验证和编码成功后才原子提交两侧；无效双侧输入不污染旧缓存。
        self._targets = targets
        # 发送失败也不重用该序号，更不补发旧姿态。
        self._sequence = sequence
        sent = self._socket.sendto(payload, self._address)
        if sent != len(payload):
            raise OSError("Tianji UDP 数据报未完整交给本机套接字")
        return sequence

    def close(self) -> None:
        """只关闭套接字，不发送零位或任何退出姿态。"""

        self._closed = True
        self._socket.close()
