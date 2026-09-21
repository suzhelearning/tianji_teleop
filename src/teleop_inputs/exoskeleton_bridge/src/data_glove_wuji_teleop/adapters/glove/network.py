"""手套专用网络预检与绑定源地址的编码器连接。"""

from __future__ import annotations

import argparse
import ipaddress
import math
import os
import subprocess

from ...project import PROJECT_ROOT
from .encoder_stream import EncoderConnection


_NETWORK_VARIABLES = (
    ("glove_interface", "DATAGLOVE_INTERFACE"),
    ("glove_mac", "DATAGLOVE_MAC"),
    ("glove_host_address", "DATAGLOVE_HOST_ADDRESS"),
    ("glove_route_table", "DATAGLOVE_ROUTE_TABLE"),
    ("glove_mtu", "DATAGLOVE_MTU"),
)


def _network_value(args: argparse.Namespace, argument: str, variable: str):
    value = getattr(args, argument, None)
    if value is None and not getattr(args, "glove_profile", None):
        value = os.environ.get(variable)
    return value


def _source_address(args: argparse.Namespace) -> str | None:
    table = _network_value(args, "glove_route_table", "DATAGLOVE_ROUTE_TABLE")
    if table is None or table == "":
        return None
    if isinstance(table, bool) or not str(table).isascii() or not str(table).isdigit() or not 10000 <= int(table) <= 30000:
        raise ValueError("手套路由表必须是 10000..30000 的整数")
    address = _network_value(args, "glove_host_address", "DATAGLOVE_HOST_ADDRESS")
    if address is None:
        address = "192.168.7.1/24"
    try:
        return str(ipaddress.IPv4Interface(address).ip)
    except (ValueError, TypeError) as exc:
        raise ValueError("手套源地址必须是 IPv4 主机地址/CIDR") from exc


def prepare_glove_network(args: argparse.Namespace, *, check_only: bool = False) -> None:
    """预检失败即拒绝启动；具名设备不继承其他手套的网络环境变量。"""
    timeout = getattr(args, "timeout", 5.0)
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError("--timeout 必须为正有限秒数")
    if not 1 <= args.port <= 65535:
        raise ValueError("--port 必须在 1..65535")
    _source_address(args)
    if getattr(args, "skip_network_setup", False) and not check_only:
        return
    environment = os.environ.copy()
    environment["DATAGLOVE_HOST"] = args.host
    environment["DATAGLOVE_PORT"] = str(args.port)
    environment["DATAGLOVE_NETWORK_CHECK_ONLY"] = "1" if check_only else "0"
    for argument, variable in _NETWORK_VARIABLES:
        value = _network_value(args, argument, variable)
        if value is None:
            environment.pop(variable, None)
        else:
            environment[variable] = str(value)
    usb_wait_timeout = getattr(args, "usb_wait_timeout", None)
    if usb_wait_timeout is not None:
        environment["DATAGLOVE_USB_WAIT_TIMEOUT_SECONDS"] = str(usb_wait_timeout)
    result = subprocess.run(
        ["bash", str(PROJECT_ROOT / "scripts/lib/dataglove_network.sh")],
        env=environment,
        check=False,
    )
    if result.returncode:
        raise RuntimeError("手套网络预检未通过；未连接手套或启动目标发布器")


def connect_glove(args: argparse.Namespace) -> EncoderConnection:
    """策略路由模式必须绑定对应主机 IPv4；绑定失败不重试其他接口。"""
    source_address = _source_address(args)
    options = {"port": args.port, "timeout": getattr(args, "timeout", 5.0)}
    device_id = getattr(args, "glove_device_id", None)
    if getattr(args, "glove_profile", None) and not device_id:
        raise ValueError("设备档案未绑定 device_id；请先读取信息包并登记实际设备身份")
    options["http_port"] = getattr(args, "glove_http_port", None) or 5570
    options["expected_device_id"] = device_id
    options["expected_hand"] = getattr(args, "hand", None)
    options["protocol"] = getattr(args, "glove_protocol", None) or "dgst-v3"
    if source_address is not None:
        options["source_address"] = source_address
    try:
        return EncoderConnection.connect(args.host, **options)
    except OSError as exc:
        raise ConnectionError(
            f"无法连接数据手套 {args.host}:{args.port}：{exc}；"
            "请检查手套供电、USB 连接及编码器服务"
        ) from exc
