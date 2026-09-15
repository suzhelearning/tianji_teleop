"""任务只绑定物理设备；零位、机构和通信参数仍由各设备档案独立拥有。"""

from __future__ import annotations

import argparse
import ipaddress
import json
import os
import re
import tempfile
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Iterable

from ..project import PROJECT_ROOT
from .dataglove.device import GloveDeviceProfile, validate_profile_name

HANDS = ("left", "right")


def task_directory() -> Path:
    return PROJECT_ROOT / "config/teleoperation/tasks"


def validate_glove_set(profiles: Iterable[GloveDeviceProfile], *, check_interfaces: bool = False) -> None:
    """校验档案隔离；历史网卡名可复用，实际并用网卡仅在在线解析后校验。"""
    selected = tuple(profiles)
    if not selected or len(selected) > 2:
        raise ValueError("任务必须绑定一只或两只手套")
    if any(not profile.device_id for profile in selected):
        raise ValueError("任务手套必须绑定真实 device_id；请先 register-glove --discover")
    for label, values in (
        ("手侧", [profile.hand for profile in selected]),
        ("设备 ID", [profile.device_id for profile in selected]),
        ("设备档案", [profile.path for profile in selected]),
        ("源地址", [str(ipaddress.IPv4Interface(profile.network["host_address"]).ip) for profile in selected]),
        ("路由表", [profile.network["route_table"] for profile in selected]),
    ):
        if len(set(values)) != len(values):
            raise ValueError(f"任务中的左右手不能共用{label}")
    if check_interfaces and len(selected) == 2:
        first, second = selected
        if first.network["mac"] == second.network["mac"] and (
            not first.network.get("interface") or not second.network.get("interface")
            or first.network["interface"] == second.network["interface"]
        ):
            raise ValueError("同 MAC 的两只手套必须分别绑定不同的明确网卡")


@dataclass(frozen=True)
class TeleopTask:
    path: Path
    name: str
    gloves: dict[str, Path]
    robot_serials: dict[str, str]
    robot_network: dict[str, str]

    @classmethod
    def load(cls, path_or_name: str | Path) -> "TeleopTask":
        text = str(path_or_name)
        if isinstance(path_or_name, str) and "/" not in text and not Path(text).suffix:
            path = task_directory() / f"{validate_profile_name(text)}.json"
        else:
            path = Path(path_or_name).expanduser()
        if path.suffix != ".json":
            raise ValueError("任务档案必须为 .json 文件")
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            raise ValueError(f"任务 JSON 格式无效：{path}：{exc}") from exc
        return cls.from_dict(data, path=path)

    @classmethod
    def from_dict(cls, data: object, *, path: str | Path) -> "TeleopTask":
        required = {"version", "name", "gloves"}
        allowed = required | {"robot_serials", "robot_network"}
        if not isinstance(data, dict) or not required <= data.keys() or data.keys() - allowed:
            raise ValueError("任务字段必须为 version、name、gloves，以及可选 robot_serials、robot_network")
        if type(data["version"]) is not int or data["version"] != 1:
            raise ValueError("任务 version 必须为 1")
        name = validate_profile_name(data["name"])
        destination = Path(path).expanduser().resolve()
        bindings = data["gloves"]
        if not isinstance(bindings, dict) or not bindings or bindings.keys() - set(HANDS):
            raise ValueError("gloves 必须包含 left 和/或 right 的设备档案路径")
        gloves = {}
        for hand, value in bindings.items():
            if not isinstance(value, str) or not value.strip() or "\x00" in value:
                raise ValueError(f"gloves.{hand} 必须为非空设备档案路径")
            gloves[hand] = (destination.parent / Path(value).expanduser()).resolve()
        serials = data.get("robot_serials", {})
        if not isinstance(serials, dict) or serials.keys() - gloves.keys():
            raise ValueError("robot_serials 只能指定已绑定手侧；未指定则自动发现")
        if any(not isinstance(value, str) or not value.strip() or "\x00" in value for value in serials.values()):
            raise ValueError("机器人 SN 必须为非空字符串；自动发现请省略该侧 SN")
        if len(set(serials.values())) != len(serials):
            raise ValueError("左右机器人不能使用同一个 SN")
        network = data.get("robot_network", {})
        if not isinstance(network, dict) or network.keys() - {"connection", "interface"}:
            raise ValueError("robot_network 仅支持 connection 和 interface")
        if any(not isinstance(value, str) or not value.strip() or any(c in value for c in "\x00\r\n") for value in network.values()):
            raise ValueError("机器人网络连接名和网卡必须为非空单行字符串")
        interface = network.get("interface")
        if interface is not None and not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_.:-]{0,14}", interface):
            raise ValueError("robot_network.interface 不是有效网卡名")
        return cls(destination, name, gloves, dict(serials), dict(network))

    def profiles(self, hand: str | None = None) -> tuple[GloveDeviceProfile, ...]:
        if hand not in (None, "left", "right", "both"):
            raise ValueError("手侧必须为 left、right 或 both")
        selected_hands = HANDS if hand in (None, "both") else (hand,)
        if hand is not None and any(side not in self.gloves for side in selected_hands):
            raise ValueError(f"任务 {self.name} 未绑定请求的 {hand} 手套")
        profiles = []
        for side in selected_hands:
            if side not in self.gloves:
                continue
            profile = GloveDeviceProfile.load(self.gloves[side])
            if profile.hand != side:
                raise ValueError(f"任务 {self.name} 的 {side} 绑定到了 {profile.hand} 手套 {profile.name}")
            profiles.append(profile)
        validate_glove_set(profiles)
        return tuple(profiles)

    def to_dict(self, *, directory: Path | None = None) -> dict[str, object]:
        base = self.path.parent if directory is None else directory
        return {
            "version": 1, "name": self.name,
            "gloves": {hand: os.path.relpath(path, base) for hand, path in self.gloves.items()},
            "robot_serials": dict(self.robot_serials),
            "robot_network": dict(self.robot_network),
        }

    def save(self, *, overwrite: bool = False) -> Path:
        self.profiles()
        if self.path.suffix != ".json":
            raise ValueError("任务档案必须为 .json 文件")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=self.path.parent,
                                             prefix=f".{self.path.name}.", suffix=".tmp", delete=False) as stream:
                temporary = Path(stream.name)
                json.dump(self.to_dict(), stream, ensure_ascii=False, indent=2, allow_nan=False)
                stream.write("\n")
            if overwrite:
                os.replace(temporary, self.path)
            else:
                os.link(temporary, self.path)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
        return self.path


def bind_default_glove(profile: GloveDeviceProfile) -> TeleopTask:
    """仅将已经完整标定的设备放入对应手侧，保留另一侧和机器人配置。"""
    profile.validate_resources()
    path = task_directory() / "default.json"
    existing = path.exists()
    task = (
        TeleopTask.load(path)
        if existing
        else TeleopTask(path.resolve(), "default", {}, {}, {})
    )
    if task.gloves.get(profile.hand) == profile.path:
        return task
    updated = replace(task, gloves={**task.gloves, profile.hand: profile.path})
    updated.save(overwrite=existing)
    return updated


def apply_task_glove(args: argparse.Namespace) -> TeleopTask | None:
    """单手命令解析任务一次，之后走既有具名设备消费链路。"""
    selected = getattr(args, "task", None)
    if selected is None:
        return None
    if getattr(args, "glove_profile", None) is not None:
        raise ValueError("--task 与 --glove-profile 不能同时指定")
    task = TeleopTask.load(selected)
    profiles = task.profiles(getattr(args, "hand", None))
    if len(profiles) != 1:
        raise ValueError("该命令一次处理一只手套；双手任务请指定 --hand left 或 --hand right")
    profile = profiles[0]
    args.hand = profile.hand
    args.glove_profile = str(profile.path)
    return task
