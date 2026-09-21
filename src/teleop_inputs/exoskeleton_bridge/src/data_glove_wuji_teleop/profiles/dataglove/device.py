"""设备侧档案：通信身份、编码器标定与目标手套 URDF，不携带下游重映射参数。"""

from __future__ import annotations

import argparse
import ipaddress
import json
import os
import re
import tempfile
from copy import deepcopy
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Mapping

from ...project import PROJECT_ROOT
from .urdf_mapping import DataGloveUrdfMapping
from .urdf_zero import UrdfZeroProfile

_NETWORK_FIELDS = {"mac", "interface", "host", "port", "http_port", "mtu", "host_address", "route_table", "protocol"}


def validate_profile_name(name: str) -> str:
    """名称只用于一个文件名，不能包含目录或隐藏文件前缀。"""
    if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,127}", name):
        raise ValueError("名称必须为 1..128 位字母、数字、下划线或连字符，并以字母或数字开头")
    return name


def canonical_device_name(hand: str, device_id: str) -> str:
    """手侧绑定完整 64 位十六进制身份；保留所有前导零，不使用型号别名。"""
    if hand not in ("left", "right"):
        raise ValueError("设备命名要求明确绑定 left 或 right")
    if not isinstance(device_id, str) or not re.fullmatch(r"(?:0[xX])?[0-9a-fA-F]{16}", device_id):
        raise ValueError("device_id 必须为完整 16 位十六进制 ID，可带 0x 前缀")
    normalized = device_id.lower().removeprefix("0x")
    return f"{hand}-{normalized}"


def device_directory() -> Path:
    return PROJECT_ROOT / "config/dataglove/devices"


def _read_json(path: Path) -> dict:
    if path.suffix != ".json":
        raise ValueError(f"设备档案必须为 .json 文件：{path}")
    try:
        with path.open(encoding="utf-8") as stream:
            data = json.load(stream)
    except json.JSONDecodeError as exc:
        raise ValueError(f"JSON 格式无效：{path}：{exc}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"JSON 根节点必须是对象：{path}")
    return data


@dataclass(frozen=True)
class GloveDeviceProfile:
    path: Path
    name: str
    hand: str
    device_id: str
    network: dict[str, object]
    glove_urdf: Path
    mapping_data: dict[str, object]
    zeros: dict[str, dict[str, object]]
    active_zero: str | None
    version: int = 4

    @classmethod
    def load(cls, path_or_name: str | Path) -> "GloveDeviceProfile":
        text = str(path_or_name)
        path = Path(path_or_name).expanduser()
        if isinstance(path_or_name, str) and "/" not in text and not path.is_file():
            identity = text.lower().removeprefix("0x")
            matches = []
            for path in sorted(device_directory().glob("*.json")):
                profile = cls.from_dict(_read_json(path), path=path)
                if (
                    text in (profile.name, path.stem, path.name)
                    or identity == profile.device_id.removeprefix("0x")
                ):
                    matches.append(profile)
            if not matches:
                raise FileNotFoundError(f"未找到设备档案：{text}")
            if len(matches) != 1:
                raise ValueError(f"设备索引 {text!r} 不唯一；请指定完整JSON路径或唯一device_id")
            return matches[0]
        return cls.from_dict(_read_json(path), path=path)

    @classmethod
    def from_dict(cls, data: Mapping[str, object], *, path: str | Path) -> "GloveDeviceProfile":
        required = {"version", "name", "hand", "device_id", "network", "glove_urdf", "mapping", "zeros", "active_zero"}
        if not isinstance(data, Mapping) or set(data) != required:
            raise ValueError("设备档案必须为version4单文件结构：身份、network、glove_urdf、mapping、zeros、active_zero")
        version = data["version"]
        if type(version) is not int or version != 4:
            raise ValueError("设备档案 version 必须为 4；旧拆分档案需先合并")
        name = validate_profile_name(data["name"])
        hand = data["hand"]
        if hand not in ("left", "right"):
            raise ValueError("设备档案 hand 必须为 left 或 right")
        device_id = data["device_id"]
        canonical_device_name(hand, device_id)
        device_id = "0x" + device_id.lower().removeprefix("0x")
        network_raw = data["network"]
        if not isinstance(network_raw, dict):
            raise ValueError("network 必须是对象")
        missing = (_NETWORK_FIELDS - {"interface", "protocol"}) - set(network_raw)
        unknown = set(network_raw) - _NETWORK_FIELDS
        if missing or unknown:
            raise ValueError(f"network 字段错误：缺少 {sorted(missing)}，未知 {sorted(unknown, key=str)}")
        network = dict(network_raw)
        network.setdefault("protocol", "dgst-v3")
        if network["protocol"] not in ("dgst-v3", "encoder-v1"):
            raise ValueError("network.protocol 必须为 dgst-v3 或 encoder-v1")
        mac = network["mac"]
        if not isinstance(mac, str) or not re.fullmatch(r"(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}", mac):
            raise ValueError("network.mac 必须为六段冒号分隔的 MAC 地址")
        network["mac"] = mac.lower()
        interface = network.get("interface")
        if interface is not None and (
            not isinstance(interface, str)
            or not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_.:-]{0,14}", interface)
        ):
            raise ValueError("network.interface 必须为有效网卡名（最多 15 字节）")
        for key, lower, upper in (
            ("port", 1, 65535), ("http_port", 1, 65535),
            ("mtu", 68, 9000), ("route_table", 10000, 30000),
        ):
            value = network[key]
            if type(value) is not int or not lower <= value <= upper:
                raise ValueError(f"network.{key} 必须为 {lower}..{upper} 的整数")
        try:
            if not isinstance(network["host"], str):
                raise ValueError("host 必须是 IPv4 字符串")
            host = ipaddress.IPv4Address(network["host"])
            address_text = network["host_address"]
            if not isinstance(address_text, str) or "/" not in address_text:
                raise ValueError("缺少 CIDR 前缀")
            source = ipaddress.IPv4Interface(address_text)
        except (ValueError, TypeError) as exc:
            raise ValueError("network.host 必须为 IPv4，host_address 必须为 IPv4/CIDR") from exc
        if host.is_unspecified or host.is_multicast or source.ip.is_unspecified or source.ip.is_multicast:
            raise ValueError("网络地址必须为单播 IPv4")
        if host == source.ip or host not in source.network:
            raise ValueError("手套 host 必须与 host_address 同网段且不同于主机地址")
        if source.network.prefixlen < 31 and (
            source.ip in (source.network.network_address, source.network.broadcast_address)
            or host in (source.network.network_address, source.network.broadcast_address)
        ):
            raise ValueError("网络地址不能是网段地址或广播地址")
        network["host"] = str(host)
        network["host_address"] = str(source)
        resolved_path = Path(path).expanduser().resolve()
        value = data["glove_urdf"]
        if not isinstance(value, str) or not value.strip() or "\x00" in value:
            raise ValueError("glove_urdf 必须为非空文件路径")
        urdf = (resolved_path.parent / Path(value).expanduser()).resolve()
        if not isinstance(data["mapping"], dict) or not isinstance(data["zeros"], dict):
            raise ValueError("mapping 和 zeros 必须为内嵌JSON对象")
        profile = cls(
            path=resolved_path, name=name, hand=hand, device_id=device_id, network=network,
            glove_urdf=urdf, mapping_data=deepcopy(data["mapping"]),
            zeros=deepcopy(data["zeros"]), active_zero=data["active_zero"],
        )
        profile._validate_embedded()
        return profile

    def load_mapping(self) -> DataGloveUrdfMapping:
        return DataGloveUrdfMapping.from_dict(self.mapping_data)

    def load_zero(self, name: str | None = None) -> UrdfZeroProfile | None:
        selected = self.active_zero if name is None else validate_profile_name(name)
        if selected is None or selected not in self.zeros:
            return None
        return UrdfZeroProfile.from_dict(self.zeros[selected])

    def _validate_embedded(self) -> DataGloveUrdfMapping:
        mapping = self.load_mapping()
        if mapping.hand != self.hand:
            raise ValueError("设备 hand 与内嵌 mapping 手侧不一致")
        for name, data in self.zeros.items():
            validate_profile_name(name)
            if not isinstance(data, dict):
                raise ValueError(f"zeros.{name} 必须为零位JSON对象")
            zero = UrdfZeroProfile.from_dict(data)
            if zero.hand != self.hand:
                raise ValueError(f"zeros.{name} 与设备手侧不一致")
            if zero.expected_cs_by_joint != mapping.expected_cs_by_joint:
                raise ValueError(f"mapping 与 zeros.{name} 的编码器接线顺序不一致")
        if self.active_zero is not None:
            validate_profile_name(self.active_zero)
            active = self.load_zero()
            if active is None or not active.complete:
                raise ValueError("active_zero 必须指向已有完整零位组")
        return mapping

    def validate_resources(self, *, require_zero: bool = True) -> None:
        """通信身份和标定同文件；登记/标定允许尚无已激活零位。"""
        mapping = self._validate_embedded()
        if not self.glove_urdf.is_file() or self.glove_urdf == self.path:
            raise ValueError(f"设备 {self.name} 的 glove_urdf 文件不存在或无效：{self.glove_urdf}")
        mapping.validate_urdf(self.glove_urdf)
        if require_zero and self.load_zero() is None:
            raise ValueError(f"设备 {self.name} 尚无完整的已激活零位")

    def to_dict(self, *, directory: Path | None = None) -> dict[str, object]:
        base = self.path.parent if directory is None else directory
        return {
            "version": self.version, "name": self.name, "hand": self.hand, "device_id": self.device_id,
            "network": dict(self.network), "glove_urdf": os.path.relpath(self.glove_urdf, base),
            "mapping": deepcopy(self.mapping_data), "zeros": deepcopy(self.zeros),
            "active_zero": self.active_zero,
        }

    def save(self, path: str | Path | None = None, *, overwrite: bool = False) -> Path:
        destination = self.path if path is None else Path(path).expanduser().resolve()
        if destination.suffix != ".json":
            raise ValueError(f"设备档案必须为 .json 文件：{destination}")
        self.from_dict(self.to_dict(directory=destination.parent), path=destination)
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", dir=destination.parent,
                prefix=f".{destination.name}.", suffix=".tmp", delete=False,
            ) as stream:
                temporary_path = Path(stream.name)
                json.dump(self.to_dict(directory=destination.parent), stream, ensure_ascii=False, indent=2, allow_nan=False)
                stream.write("\n")
            if overwrite:
                os.replace(temporary_path, destination)
            else:
                os.link(temporary_path, destination)
        finally:
            if temporary_path is not None:
                temporary_path.unlink(missing_ok=True)
        return destination

    def store_zero(
        self, name: str, zero: UrdfZeroProfile, *, activate: bool = False,
    ) -> "GloveDeviceProfile":
        """原子更新单组进度，保留最新网络/机构及其他组；不以部分标定替换活动组。"""
        validate_profile_name(name)
        current = self.load(self.path)
        if current.device_id != self.device_id or current.hand != self.hand:
            raise ValueError("采集期间设备档案身份发生变化，拒绝写入")
        if (activate or name == current.active_zero) and not zero.complete:
            raise ValueError("不完整零位不能激活或替换活动组")
        updated = replace(
            current, zeros={**current.zeros, name: zero.to_dict()},
            active_zero=name if activate else current.active_zero,
        )
        updated._validate_embedded()
        updated.save(overwrite=True)
        return updated

    def activate_zero(self, name: str) -> "GloveDeviceProfile":
        current = self.load(self.path)
        zero = current.load_zero(name)
        if zero is None:
            raise ValueError(f"不存在零位组：{name}")
        return self.store_zero(name, zero, activate=True)


def apply_glove_profile(
    args: argparse.Namespace, *, require_zero: bool = True, require_resources: bool = True,
) -> GloveDeviceProfile | None:
    selected = getattr(args, "glove_profile", None)
    if selected is None:
        return None
    profile = GloveDeviceProfile.load(selected)
    hand = getattr(args, "hand", None)
    if hand is not None and hand != profile.hand:
        raise ValueError(f"--hand={hand} 与设备 {profile.name} 的 {profile.hand} 不一致")
    if require_resources:
        profile.validate_resources(require_zero=require_zero)
    args.hand = profile.hand
    args.glove_device_id = profile.device_id
    args.glove_urdf = profile.glove_urdf
    for key in ("host", "port"):
        setattr(args, key, profile.network[key])
    for key in ("interface", "mac", "host_address", "route_table", "http_port", "mtu", "protocol"):
        setattr(args, f"glove_{key}", profile.network.get(key))
    return profile
