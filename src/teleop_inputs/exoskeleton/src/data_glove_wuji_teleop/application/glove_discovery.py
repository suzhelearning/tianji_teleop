"""按实物手侧发现标定对象；网络准备复用专用源路由实现，不触碰机器人。"""

from __future__ import annotations

import argparse
import ipaddress
import json
import math
import subprocess
from dataclasses import dataclass, replace
from pathlib import Path

from ..adapters.glove.dgst_protocol import StreamMetadata, fetch_metadata
from ..adapters.glove.network import prepare_glove_network
from ..profiles.dataglove.device import GloveDeviceProfile, canonical_device_name, device_directory
from ..profiles.teleop_task import validate_glove_set
from .glove_profiles import STANDARD_GLOVE_MAC, _registered_profiles, command_register_glove

_HOST = "192.168.7.2"
_SUBNET = ipaddress.IPv4Network("192.168.7.0/24")
_SYS_CLASS_NET = Path("/sys/class/net")


def _ip_json(*arguments: str) -> list[dict]:
    result = subprocess.run(["ip", "-j", "-4", *arguments], capture_output=True, text=True, check=True)
    data = json.loads(result.stdout)
    if not isinstance(data, list) or any(not isinstance(row, dict) for row in data):
        raise ValueError("无法读取主机 IPv4 网络占用信息")
    return data


def _table_aliases() -> dict[str, int]:
    aliases = {"unspec": 0, "default": 253, "main": 254, "local": 255}
    for directory in (Path("/usr/lib/iproute2"), Path("/etc/iproute2")):
        for path in (directory / "rt_tables", *sorted((directory / "rt_tables.d").glob("*.conf"))):
            if not path.is_file():
                continue
            for line in path.read_text(encoding="utf-8").splitlines():
                fields = line.partition("#")[0].split()
                if len(fields) == 2 and fields[0].isdigit():
                    aliases[fields[1]] = int(fields[0])
    return aliases


@dataclass
class _Inventory:
    addresses: dict[str, set[str]]
    routes: list[dict]
    rules: list[dict]
    aliases: dict[str, int]
    sources: set[str]
    tables: set[int]

    def table(self, value) -> int:
        text = str(value)
        if text.isdigit():
            return int(text)
        if text not in self.aliases:
            raise ValueError(f"无法解析已占用策略路由表 {text!r}，拒绝自动分配")
        return self.aliases[text]

    @classmethod
    def read(cls, profiles: list[GloveDeviceProfile]) -> "_Inventory":
        addresses: dict[str, set[str]] = {}
        for interface in _ip_json("address", "show"):
            for address in interface.get("addr_info", []):
                if address.get("family") == "inet":
                    addresses.setdefault(address["local"], set()).add(interface["ifname"])
        routes = _ip_json("route", "show", "table", "all")
        rules = _ip_json("rule", "show")
        aliases = _table_aliases()
        inventory = cls(addresses, routes, rules, aliases, set(addresses), set(aliases.values()))
        for profile in profiles:
            inventory.sources.add(str(ipaddress.IPv4Interface(profile.network["host_address"]).ip))
            inventory.tables.add(profile.network["route_table"])
        for rule in rules:
            if "table" in rule:
                inventory.tables.add(inventory.table(rule["table"]))
            if "priority" in rule:
                inventory.tables.add(int(rule["priority"]))
            source = rule.get("src", "all")
            if source != "all":
                selector = ipaddress.IPv4Network(source, strict=False)
                if selector.prefixlen:
                    inventory.sources.update(str(ip) for ip in _SUBNET.hosts() if ip in selector)
        for route in routes:
            inventory.tables.add(inventory.table(route.get("table", "main")))
            if "prefsrc" in route:
                inventory.sources.add(route["prefsrc"])
        return inventory

    def allocate(self, interface: str, mac: str) -> dict:
        source = next((str(ip) for ip in _SUBNET.hosts() if str(ip) != _HOST and str(ip) not in self.sources), None)
        table = next((value for value in range(17000, 30001) if value not in self.tables), None)
        if source is None or table is None:
            raise ValueError("没有可安全分配的手套独立源地址或策略路由表；请显式配置网络")
        self.sources.add(source)
        self.tables.add(table)
        return {"interface": interface, "mac": mac, "host": _HOST, "host_address": f"{source}/24",
                "route_table": table, "http_port": 5570, "port": 9100, "protocol": "encoder-v1", "mtu": 8000}

    def configured(self, interface: str, mac: str) -> dict | None:
        for route in sorted(self.routes, key=lambda row: str(row.get("table", "main"))):
            table = self.table(route.get("table", "main"))
            source = route.get("prefsrc")
            if (not 10000 <= table <= 30000 or route.get("dev") != interface or route.get("gateway")
                    or route.get("dst") not in (_HOST, f"{_HOST}/32")
                    or self.addresses.get(source) != {interface}):
                continue
            address = ipaddress.IPv4Address(source)
            if address not in _SUBNET or str(address) in (_HOST, str(_SUBNET.network_address), str(_SUBNET.broadcast_address)):
                continue
            matches = [rule for rule in self.rules if "table" in rule
                       and self.table(rule["table"]) == table and int(rule.get("priority", -1)) == table
                       and rule.get("src") in (source, f"{source}/32")]
            if len(matches) != 1:
                continue
            return {"interface": interface, "mac": mac, "host": _HOST, "host_address": f"{source}/24",
                    "route_table": table, "http_port": 5570, "port": 9100, "protocol": "encoder-v1", "mtu": 8000}
        return None

    def can_reuse(self, network: dict, interface: str) -> bool:
        source = str(ipaddress.IPv4Interface(network["host_address"]).ip)
        if self.addresses.get(source, set()) - {interface}:
            return False
        table = network["route_table"]
        for route in self.routes:
            if self.table(route.get("table", "main")) == table and (
                route.get("dev") != interface or route.get("gateway")
                or route.get("dst") not in (network["host"], f"{network['host']}/32")
                or route.get("prefsrc") != source
            ):
                return False
        for rule in self.rules:
            rule_table = self.table(rule["table"]) if "table" in rule else None
            if rule_table == table or rule.get("priority") == table or rule.get("src") in (source, f"{source}/32"):
                if (rule_table != table or rule.get("priority") != table
                        or rule.get("src") not in (source, f"{source}/32")):
                    return False
        return True


def _prepare(network: dict, *, timeout: float, enabled: bool) -> None:
    args = argparse.Namespace(
        glove_profile="calibration-discovery", host=network["host"], port=network["port"], timeout=timeout,
        **{f"glove_{key}": network.get(key) for key in ("interface", "mac", "host_address", "route_table", "mtu")},
    )
    prepare_glove_network(args, check_only=not enabled)


def _probe(network: dict, *, timeout: float) -> StreamMetadata:
    return fetch_metadata(network["host"], port=network["http_port"], timeout=timeout,
                          source_address=str(ipaddress.IPv4Interface(network["host_address"]).ip))


def _observe_connected(profiles, *, timeout: float, prepare_network: bool):
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout 必须为正有限秒数")
    known_macs = {STANDARD_GLOVE_MAC, *(profile.network["mac"] for profile in profiles)}
    candidates = []
    for address in sorted(_SYS_CLASS_NET.glob("*/address")):
        mac = address.read_text(encoding="ascii").strip().lower()
        if mac in known_macs:
            candidates.append((address.parent.name, mac))
    if not candidates:
        raise ValueError("未发现手套候选网卡；请连接手套 USB")
    inventory = _Inventory.read(profiles)
    observed: list[tuple[dict, StreamMetadata]] = []
    errors = []
    for interface, mac in candidates:
        try:
            network = inventory.configured(interface, mac)
            if network is None:
                if not prepare_network:
                    raise ValueError("手套独立源路由尚未配置（当前为只读检查）")
                network = inventory.allocate(interface, mac)
            _prepare(network, timeout=timeout, enabled=prepare_network)
            metadata = _probe(network, timeout=timeout)
            canonical_device_name(metadata.device["hand"], metadata.device["device_id"])
            observed.append((network, metadata))
        except (OSError, ValueError, RuntimeError) as exc:
            errors.append(f"{interface}: {exc}")
    if errors:
        raise ValueError("无法完整识别所有手套候选，不能保证手侧唯一：" + "; ".join(errors))
    return inventory, observed


def _rebind_observed(profile, network, metadata, inventory, *, timeout, prepare_network):
    if profile.device_id != metadata.device["device_id"] or profile.hand != metadata.device["hand"]:
        raise ValueError("实物 device_id/hand 与已登记设备身份不一致")
    mapping = profile.load_mapping()
    mapping.validate_stream(channels=metadata.sensors["encoder"]["channels"],
                            cs_by_joint=metadata.sensors["encoder"]["cs_by_joint"])
    profile.validate_resources(require_zero=False)
    if inventory.can_reuse(profile.network, network["interface"]):
        final_network = {**profile.network, "interface": network["interface"], "mac": network["mac"]}
    else:
        final_network = {**inventory.allocate(network["interface"], network["mac"]),
                         "protocol": profile.network["protocol"], "port": profile.network["port"],
                         "http_port": profile.network["http_port"]}
    _prepare(final_network, timeout=timeout, enabled=prepare_network)
    confirmed = _probe(final_network, timeout=timeout)
    if (confirmed.device["device_id"] != profile.device_id or confirmed.device["hand"] != profile.hand
            or confirmed.sha256 != metadata.sha256):
        raise ValueError("网络准备期间实物身份/元数据变化，未更新设备档案")
    return replace(profile, network=final_network)


def resolve_glove_profiles(
    profiles, *, timeout: float = 5.0, prepare_network: bool = True,
) -> tuple[GloveDeviceProfile, ...]:
    """启动前按指定ID解析当前网卡；不注册、不换设备、不修改机构或零位。"""
    selected = tuple(profiles)
    validate_glove_set(selected)
    for profile in selected:
        profile.validate_resources()
    registered = {profile.path: profile for profile in _registered_profiles(device_directory())}
    registered.update({profile.path: profile for profile in selected})
    inventory, observed = _observe_connected(
        list(registered.values()), timeout=timeout, prepare_network=prepare_network,
    )
    seen = ", ".join(f"{meta.device['hand']}={meta.device['device_id']}@{net['interface']}"
                     for net, meta in observed)
    matched = []
    for profile in selected:
        candidates = [(net, meta) for net, meta in observed if meta.device["device_id"] == profile.device_id]
        if len(candidates) != 1:
            raise ValueError(
                f"请求 {profile.hand} ID={profile.device_id}，发现 {len(candidates)} 个匹配端点；"
                f"已识别：{seen}。不会换用其他设备的标定。"
            )
        network, metadata = candidates[0]
        if metadata.device["hand"] != profile.hand:
            raise ValueError(f"ID={profile.device_id} 的实物手侧与档案不一致")
        matched.append((profile, network, metadata))
    resolved = tuple(
        _rebind_observed(profile, network, metadata, inventory,
                         timeout=timeout, prepare_network=prepare_network)
        for profile, network, metadata in matched
    )
    validate_glove_set(resolved, check_interfaces=True)
    for before, after in zip(selected, resolved):
        if before.network != after.network:
            after.save(overwrite=True)
    return resolved


def discover_glove_for_calibration(
    hand: str, *, timeout: float = 5.0, prepare_network: bool = True,
) -> GloveDeviceProfile:
    """全部候选均可识别且指定手侧唯一时返回；仅选中设备才可登记/更新网络字段。"""
    if hand not in ("left", "right"):
        raise ValueError("标定手侧必须为 left 或 right")
    profiles = list(_registered_profiles(device_directory()))
    inventory, observed = _observe_connected(profiles, timeout=timeout, prepare_network=prepare_network)
    selected = [(network, metadata) for network, metadata in observed if metadata.device["hand"] == hand]
    seen = ", ".join(f"{metadata.device['hand']}={metadata.device['device_id']}@{network['interface']}"
                     for network, metadata in observed)
    if len(selected) != 1:
        reason = "未发现" if not selected else "发现多个"
        raise ValueError(f"{reason} {hand} 手套，拒绝标定；已识别：{seen}")
    network, metadata = selected[0]
    device_id = metadata.device["device_id"]
    name = canonical_device_name(hand, device_id)
    normalized_id = device_id.lower().removeprefix("0x")
    matches = [profile for profile in profiles if profile.device_id is not None
               and profile.device_id.lower().removeprefix("0x") == normalized_id]
    if len(matches) > 1:
        raise ValueError(f"设备 {name} 存在多个历史档案，拒绝猜选标定产物")
    if matches:
        profile = matches[0]
        updated = _rebind_observed(profile, network, metadata, inventory,
                                   timeout=timeout, prepare_network=prepare_network)
        if updated.network != profile.network:
            updated.save(overwrite=True)
        return updated
    # 发现源路由可能属于同一网卡上的历史硬件；新硬件不能继承它的source/table。
    source = str(ipaddress.IPv4Interface(network["host_address"]).ip)
    if any(profile.network["route_table"] == network["route_table"]
           or str(ipaddress.IPv4Interface(profile.network["host_address"]).ip) == source for profile in profiles):
        if not prepare_network:
            raise ValueError("新设备需要独立源地址/路由表；只读检查不能借用历史设备网络身份")
        network = inventory.allocate(network["interface"], network["mac"])
    args = argparse.Namespace(
        name=name, hand=hand, device_id=device_id, discover=True, protocol="auto", port=None,
        host=network["host"], http_port=network["http_port"], mtu=network["mtu"],
        mac=network["mac"], interface=network["interface"], host_address=network["host_address"],
        route_table=network["route_table"], timeout=timeout, prepare_network=prepare_network,
        glove_urdf=None, glove_config=None, glove_config_template=None,
        output=device_directory() / f"{name}.json", overwrite=False,
    )
    command_register_glove(args)
    return GloveDeviceProfile.load(args.output)
