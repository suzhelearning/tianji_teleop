"""Hand2 专用直连网络准备；只修改本模块创建的 NM 档案，不修改硬件 IP。

自动分配仅限 RFC1918 /24，主机优先 .254（手占用时用 .253）。不探测或
猜测设备掩码；其他掩码、非私有地址必须预先配置。NM IPv4 DAD 为 3000ms。
专用连接保留原有 IPv4 地址，原档案不变；承载默认路由、全局 IPv6 或复杂
路由的网卡不能自动切换。成功后专用连接保持活动但不自启；失败恢复原连接。
"""

from __future__ import annotations

from collections.abc import Sequence
from hashlib import sha256
from ipaddress import IPv4Address, IPv4Interface, IPv4Network
import json
import os
import subprocess
from urllib.parse import urlsplit

_OWNER = "data-glove-wuji-teleop.hand2-network-v1"
_PRIVATE = tuple(IPv4Network(value) for value in ("10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16"))


def _run(*args: str, unreachable_ok: bool = False) -> str:
    try:
        result = subprocess.run(
            args, stdin=subprocess.DEVNULL, capture_output=True, text=True,
            timeout=20, env={**os.environ, "LC_ALL": "C"}, check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError(f"无法运行 {args[0]}（需要 iproute2/NetworkManager）：{exc}") from exc
    if result.returncode:
        detail = result.stderr.strip() or result.stdout.strip()
        if unreachable_ok and "Network is unreachable" in detail:
            return "[]"
        raise RuntimeError(
            f"{' '.join(args)} 失败：{detail}。若权限不足，请由管理员预先授权 "
            "NetworkManager 网络控制或手工准备直连连接，再使用 prepare_network=False；"
            "本程序不会询问或尝试密码。"
        )
    return result.stdout.strip()


def _nm(*args: str) -> str:
    return _run("nmcli", "--wait", "15", *args)


def _get(uuid: str, field: str) -> str:
    return _nm("--escape", "no", "-g", field, "connection", "show", "uuid", uuid)


def _ipv4(address: str) -> IPv4Address:
    try:
        parsed = urlsplit(address if "://" in address else f"udp://{address}")
        if parsed.scheme != "udp" or parsed.username or parsed.password or parsed.path or parsed.query or parsed.fragment:
            raise ValueError("不是 UDP IPv4 地址")
        if parsed.port is not None and not 1 <= parsed.port <= 65535:
            raise ValueError("端口超出范围")
        target = IPv4Address(parsed.hostname or "")
    except ValueError as exc:
        raise ValueError(f"无效 Hand2 UDP 地址：{address}") from exc
    if (target.is_multicast or target.is_loopback or target.is_link_local
            or target.is_unspecified or target.is_reserved
            or (not target.is_global and not any(target in private for private in _PRIVATE))):
        raise ValueError(f"拒绝不安全的 Hand2 IPv4 地址：{target}")
    return target


def _route(target: IPv4Address) -> dict:
    rows = json.loads(_run("ip", "-j", "-4", "route", "get", str(target), unreachable_ok=True))
    if len(rows) > 1:
        raise RuntimeError(f"{target} 返回多条路由，无法安全选择")
    return rows[0] if rows else {}


def _addresses() -> list[dict]:
    return json.loads(_run("ip", "-j", "address", "show"))


def _direct(target: IPv4Address, route: dict, interfaces: list[dict], excluded: set[str], fixed: str | None) -> bool:
    device = route.get("dev")
    if device in excluded:
        raise RuntimeError(f"{target} 的路由使用被排除的手套网卡 {device}")
    if not device or (fixed and device != fixed) or route.get("gateway") or route.get("type", "unicast") != "unicast":
        return False
    source = route.get("prefsrc", route.get("src"))
    for interface in interfaces:
        if interface["ifname"] != device:
            continue
        for address in interface.get("addr_info", []):
            if address.get("family") != "inet" or address.get("local") != source:
                continue
            local = IPv4Interface(f"{source}/{address['prefixlen']}")
            if target == local.ip:
                raise RuntimeError(f"手地址 {target} 与主机地址重复")
            if target in local.network and target not in (local.network.network_address, local.network.broadcast_address):
                return True
    return False


def _devices() -> dict[str, str]:
    rows = _nm("--escape", "no", "-t", "-f", "DEVICE,TYPE", "device", "status")
    return dict(row.split(":", 1) for row in rows.splitlines() if row)


def _connections() -> dict[str, str]:
    rows = _nm("--escape", "no", "-t", "-f", "UUID,NAME", "connection", "show")
    return dict(row.split(":", 1) for row in rows.splitlines() if row)


def _named(name: str, connections: dict[str, str]) -> str | None:
    matches = [uuid for uuid, candidate in connections.items() if candidate == name]
    if len(matches) > 1:
        raise RuntimeError(f"NetworkManager 连接名不唯一：{name}")
    return matches[0] if matches else None


def _active(interface: str) -> str | None:
    value = _nm("--escape", "no", "-g", "GENERAL.CON-UUID", "device", "show", interface)
    return None if value in ("", "--") else value


def prepare_robot_network(
    addresses: Sequence[str], *, network_connection: str | None = None,
    network_interface: str | None = None, excluded_interfaces: Sequence[str] = (),
    prepare_network: bool = True,
) -> None:
    """验证所有候选的真实直连路径；必要时事务式启用专用有线 NM 连接。"""
    targets = [_ipv4(address) for address in addresses]
    if not targets:
        return
    if len(set(targets)) != len(targets):
        raise ValueError("多个 Hand2 候选使用同一 IPv4 地址")
    excluded = set(excluded_interfaces)
    if any(not name or name != name.strip() for name in excluded):
        raise ValueError("排除网卡名不可为空或含首尾空白")
    if network_interface is not None and (not network_interface or network_interface != network_interface.strip()):
        raise ValueError("network_interface 无效")
    if network_connection is not None and (not network_connection or network_connection != network_connection.strip()):
        raise ValueError("network_connection 无效")
    if network_interface in excluded:
        raise ValueError("指定网卡是被排除的手套网卡")
    interfaces = _addresses()
    known = {item["ifname"] for item in interfaces}
    if excluded - known:
        raise ValueError(f"排除网卡不存在：{sorted(excluded - known)}")
    if network_interface and network_interface not in known:
        raise ValueError(f"指定网卡不存在：{network_interface}")
    locals_ = {item["local"] for interface in interfaces for item in interface.get("addr_info", []) if item.get("family") == "inet"}
    if any(str(target) in locals_ for target in targets):
        raise ValueError("发现的手地址与主机现有地址重复")
    routes = [_route(target) for target in targets]
    direct = [_direct(target, route, interfaces, excluded, network_interface) for target, route in zip(targets, routes)]
    types = _devices()
    if all(direct) and all(types.get(route.get("dev")) == "ethernet" for route in routes):
        return
    if not prepare_network:
        raise RuntimeError("Hand2 缺少正确的有线直连路由；prepare_network=False 禁止修改网络")

    connections = _connections()
    explicit_uuid = None
    if network_connection:
        explicit_uuid = _named(network_connection, connections)
        if explicit_uuid is None:
            raise ValueError(f"指定的既有网络连接不存在：{network_connection}")
        bound = _get(explicit_uuid, "connection.interface-name")
        if bound not in ("", "--"):
            if network_interface and bound != network_interface:
                raise ValueError("指定连接绑定的网卡与 network_interface 不一致")
            network_interface = bound
    available = [
        item["ifname"] for item in interfaces
        if types.get(item["ifname"]) == "ethernet"
        and item["ifname"] not in excluded
        and "LOWER_UP" in item.get("flags", [])
        and not item.get("master")
    ]
    if network_interface:
        if network_interface not in available:
            raise RuntimeError("指定网卡不是可用的、未排除的独立有线网卡")
        interface = network_interface
    elif len(available) == 1:
        interface = available[0]
    else:
        raise RuntimeError(f"可用有线网卡为 {available}；请指定 network_interface/绑定网卡的 network_connection")
    original_defaults = {}
    for family in ("-4", "-6"):
        defaults = json.loads(_run("ip", "-j", family, "route", "show", "default"))
        original_defaults[family] = defaults
        if any(route.get("dev") == interface for route in defaults):
            raise RuntimeError(f"网卡 {interface} 承载默认路由，拒绝切换；请预先配置直连地址")
    original = _active(interface)
    interface_info = next(item for item in interfaces if item["ifname"] == interface)
    if original is None and any(item.get("scope") == "global" for item in interface_info.get("addr_info", [])):
        raise RuntimeError("所选网卡含未受 NM 活动连接管理的地址，拒绝切换")

    created: str | None = None
    attempted = False
    selected = explicit_uuid
    try:
        if selected is None:
            existing = [
                IPv4Interface(f"{item['local']}/{item['prefixlen']}")
                for item in interface_info.get("addr_info", [])
                if item.get("family") == "inet"
            ]
            if original and existing and _get(original, "ipv4.method") != "manual":
                raise RuntimeError("现有 IPv4 地址不是 NM 静态配置，拒绝把 DHCP 租约冻结为静态地址")
            if any(item.get("family") == "inet6" and item.get("scope") == "global"
                   for item in interface_info.get("addr_info", [])):
                raise RuntimeError("所选网卡有全局 IPv6 地址，请预先配置连接，避免切换损失地址")
            old_routes = json.loads(_run("ip", "-j", "-4", "route", "show", "table", "all", "dev", interface))
            for route in old_routes:
                if route.get("table") == "local" or route.get("type") in ("local", "broadcast"):
                    continue
                if (route.get("gateway") or route.get("protocol") != "kernel"
                        or route.get("dst") not in {str(local.network) for local in existing}
                        or route.get("table", "main") not in ("main", 254)):
                    raise RuntimeError("所选网卡有非地址派生路由，请手工准备连接以保留原路由")
            additions = []
            for network in sorted({IPv4Network(f"{target}/24", strict=False) for target in targets}):
                if not any(network.subnet_of(private) for private in _PRIVATE):
                    raise ValueError(f"自动配址仅支持 RFC1918 /24，不能推断 {network} 的硬件掩码")
                if any(target in (network.network_address, network.broadcast_address) for target in targets):
                    raise ValueError(f"{network} 中的手地址是网络/广播地址")
                # 现有地址已覆盖网段时保留它，不创建第二个主机地址。
                if any(local.network.overlaps(network) and local.network != network for local in existing):
                    raise ValueError(f"{network} 与所选网卡的非 /24 网段重叠，不能推断硬件掩码")
                if any(local.network == network for local in existing):
                    continue
                host = network.network_address + (253 if network.network_address + 254 in targets else 254)
                if host in targets or str(host) in locals_:
                    raise ValueError(f"保守主机地址 {host} 已被使用，拒绝猜测其他地址")
                for other in interfaces:
                    if other["ifname"] == interface:
                        continue
                    for item in other.get("addr_info", []):
                        if item.get("family") == "inet" and IPv4Interface(f"{item['local']}/{item['prefixlen']}").network.overlaps(network):
                            raise ValueError(f"{network} 与其他网卡 {other['ifname']} 重叠")
                additions.append(f"{host}/24")
            if not additions:
                raise RuntimeError("所选网卡已有目标网段地址但路由仍错误，请手工检查策略路由")
            configured_addresses = [str(local) for local in existing] + additions
            plan = f"{interface}|{','.join(configured_addresses)}"
            name = f"wuji-hand2-{interface}-{sha256(plan.encode()).hexdigest()[:12]}"
            selected = _named(name, connections)
            if selected:
                # 只接受精确的本模块配置，不覆写同名外来或被用户改动的连接。
                expected = {
                    "connection.interface-name": interface,
                    "connection.autoconnect": "no",
                    "ipv4.method": "manual",
                    "ipv4.gateway": "",
                    "ipv4.routes": "",
                    "ipv4.route-table": "0",
                    "ipv4.routing-rules": "",
                    "ipv6.method": "disabled",
                    "ipv4.never-default": "yes",
                    "ipv4.dad-timeout": "3000",
                }
                if any(_get(selected, field) != value for field, value in expected.items()):
                    raise RuntimeError(f"同名连接 {name} 不属于本模块或参数已改变，拒绝覆写")
                metadata = dict(
                    part.strip().split("=", 1)
                    for part in _get(selected, "user.data").replace(" ", "").split(",")
                    if "=" in part
                )
                if metadata != {
                    "org.wuji.teleop.owner": _OWNER,
                    "org.wuji.teleop.plan": sha256(plan.encode()).hexdigest(),
                }:
                    raise RuntimeError(f"同名连接 {name} 不属于本模块，拒绝覆写")
                configured = _get(selected, "ipv4.addresses").replace(" ", "").split(",")
                if sorted(configured) != sorted(configured_addresses):
                    raise RuntimeError(f"同名连接 {name} 的地址参数不匹配")
            else:
                _nm("connection", "add", "type", "ethernet", "ifname", interface,
                    "con-name", name, "connection.autoconnect", "no",
                    "ipv4.method", "manual", "ipv4.addresses", ",".join(configured_addresses),
                    "ipv4.never-default", "yes", "ipv6.method", "disabled",
                    "ipv4.dad-timeout", "3000",
                    "user.data", f"org.wuji.teleop.owner={_OWNER},org.wuji.teleop.plan={sha256(plan.encode()).hexdigest()}")
                selected = _named(name, _connections())
                if selected is None:
                    raise RuntimeError("NM 创建后未找到唯一专用连接")
                created = selected
        else:
            if (_get(selected, "connection.type") != "802-3-ethernet"
                    or _get(selected, "ipv4.never-default") != "yes"
                    or (_get(selected, "ipv6.method") != "disabled"
                        and _get(selected, "ipv6.never-default") != "yes")):
                raise RuntimeError("指定连接必须是有线且禁止 IPv4/IPv6 默认路由；请由管理员预先配置")
            if int(_get(selected, "ipv4.dad-timeout")) <= 0:
                raise RuntimeError("指定连接必须显式启用 IPv4 地址冲突检测（建议 dad-timeout=3000）")
        attempted = True
        _nm("connection", "up", "uuid", selected, "ifname", interface)
        for family, previous_defaults in original_defaults.items():
            current_defaults = json.loads(_run("ip", "-j", family, "route", "show", "default"))
            if current_defaults != previous_defaults:
                raise RuntimeError("启用连接改变了默认路由，正在恢复原活动连接")
        refreshed = _addresses()
        for target in targets:
            if not _direct(target, _route(target), refreshed, excluded, interface):
                raise RuntimeError(f"NM 启用后 {target} 仍无所选有线网卡的真实直连路由")
        # 保留切换前的全局地址；不能只信任 NM 命令成功。
        previous = {(item.get("family"), item.get("local"), item.get("prefixlen")) for item in interface_info.get("addr_info", []) if item.get("scope") == "global"}
        current = {(item.get("family"), item.get("local"), item.get("prefixlen")) for info in refreshed if info["ifname"] == interface for item in info.get("addr_info", []) if item.get("scope") == "global"}
        if not previous <= current:
            raise RuntimeError("连接切换未保留原有地址，正在恢复原连接")
    except BaseException as failure:
        cleanup = []
        if attempted:
            try:
                if original:
                    _nm("connection", "up", "uuid", original, "ifname", interface)
                    if _active(interface) != original:
                        raise RuntimeError("NM 未恢复原活动连接")
                else:
                    _nm("device", "disconnect", interface)
                    if _active(interface) is not None:
                        raise RuntimeError("NM 未恢复原断开状态")
            except BaseException as exc:
                cleanup.append(exc)
        if created:
            try:
                _nm("connection", "delete", "uuid", created)
            except BaseException as exc:
                cleanup.append(exc)
        if cleanup:
            raise BaseExceptionGroup("网络准备失败，且恢复/清理失败", [failure, *cleanup]) from None
        raise
