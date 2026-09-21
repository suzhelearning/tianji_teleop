"""手套离线登记与绑定指定网卡、真实元数据和首帧的现场登记。"""

from __future__ import annotations

import argparse
import ipaddress
import json
import re
import socket
from dataclasses import replace
from pathlib import Path

from ..adapters.glove.dgst_protocol import fetch_metadata
from ..adapters.glove.encoder_stream import EncoderConnection
from ..adapters.glove.network import prepare_glove_network
from ..profiles.dataglove.device import (
    GloveDeviceProfile,
    canonical_device_name,
    device_directory,
    validate_profile_name,
)

from ..profiles.dataglove.urdf_mapping import DataGloveUrdfMapping
from ..project import PROJECT_ROOT

STANDARD_GLOVE_MAC = "02:33:80:00:00:01"


def add_profile_commands(subparsers) -> None:
    register = subparsers.add_parser(
        "register-glove", help="离线登记，或 --discover 核验真实 HTTP 身份与所选协议首帧",
        description="离线登记必须提供 --device-id 和 --mac（或可读取 MAC 的 --interface）。"
        "现场 --discover 自动读取真实 ID 并验证 --hand；若提供 ID 则严格核对。"
        "现场只准备手套专用网络、读取 HTTP 元数据和 START/STOP 编码器流，不驱动机器人。"
        "每个设备必须使用独立主机源地址和路由表；历史设备可热插拔复用网卡，不能同任务共用。机构模板内嵌至设备 JSON，"
        "新设备零位组为空，必须随后真实采集；不生成假零位或额外标定文件。",
    )
    register.add_argument("--name", help="自定义设备标签；省略时新设备自动使用 <left|right>-<16位完整ID>")
    register.add_argument("--hand", choices=("left", "right"), help="现场可省略并从实物元数据识别；提供时严格断言。离线须指定，或由显式机构文件读取")
    register.add_argument("--discover", action="store_true", help="通过指定网卡核验真实 HTTP 元数据、21 通道 CS 和首帧，再保存")
    register.add_argument("--device-id", help="离线必填；现场可自动读取，提供时必须与实物完全一致")
    register.add_argument("--mac", help="指定网卡 MAC；省略时从 --interface 对应的本机网卡读取")
    register.add_argument("--interface", help="手套专用网卡；仅一张符合已知手套 MAC 的候选时可自动选择，否则必须显式指定")
    register.add_argument("--host", default="192.168.7.2")
    register.add_argument("--protocol", choices=("auto", "dgst-v3", "encoder-v1"), default="auto",
                          help="现场 auto 优先探测 encoder-v1；只有端点连接拒绝才选择 DGST，身份/数据/STOP错误不回退；离线必须明确协议")
    register.add_argument("--port", type=int, default=None, help="明确协议时可覆盖端口；auto 使用 encoder-v1 9100 / DGST 5580")
    register.add_argument("--http-port", type=int, default=5570)
    register.add_argument("--mtu", type=int, default=8000)
    register.add_argument("--host-address", required=True, help="本设备独占的主机 IPv4/CIDR，不能与其他设备共用")
    register.add_argument("--route-table", type=int, required=True, help="本设备独占的策略路由表 10000..30000")
    register.add_argument("--timeout", type=float, default=5.0, help="现场 HTTP/首帧超时秒数")
    register.add_argument("--glove-urdf", type=Path, help="默认使用对应手侧 URDF；不会修改")
    mapping = register.add_mutually_exclusive_group()
    mapping.add_argument("--glove-config", type=Path, help="显式使用已有机构文件，只读核验，绝不自动改写")
    mapping.add_argument("--glove-config-template", type=Path, help="内嵌至设备 JSON 的机构模板；默认使用对应手侧模板，方向与 ready 重置为未验收")
    register.add_argument("--output", type=Path, help="设备 JSON 路径，可独立命名；默认 devices/<name>.json")
    register.add_argument("--overwrite", action="store_true", help="允许原子覆盖同一身份的设备 JSON；保留已有机构配置、全部零位组及当前激活组")
    subparsers.add_parser("list-gloves", help="列出已登记的具名设备")
    show = subparsers.add_parser("show-glove", help="查看唯一设备 JSON 与内部标定组状态")
    show.add_argument("glove_profile", help="设备名称、文件名、device_id 或 JSON 路径")


def _registration_mac(interface: str | None, mac: str | None, *, discover: bool) -> str:
    if discover and not interface:
        raise ValueError("--discover 必须明确提供 --interface")
    if interface is not None:
        if not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_.:-]{0,14}", interface):
            raise ValueError("--interface 必须为有效网卡名")
        address = Path("/sys/class/net") / interface / "address"
        if address.is_file():
            actual = address.read_text(encoding="ascii").strip().lower()
            if mac is not None and mac.lower() != actual:
                raise ValueError(f"--mac 与网卡 {interface} 的实际 MAC 不一致")
            mac = actual
        elif discover:
            raise ValueError(f"指定网卡不存在：{interface}")
    if mac is None:
        raise ValueError("必须提供 --mac，或通过存在的 --interface 读取 MAC")
    return mac


def _registered_profiles(directory: Path):
    paths = {path.resolve() for root in {device_directory(), directory}
             for path in root.glob("*.json")}
    for path in sorted(paths):
        with path.open(encoding="utf-8") as stream:
            data = json.load(stream)
        if isinstance(data, dict) and "network" in data and "device_id" in data:
            yield GloveDeviceProfile.from_dict(data, path=path)


def _discover_interface(mac: str | None) -> str:
    # 仅信任明确/标准手套 MAC 或既有设备登记过的 MAC，不按 USB 网卡名称猜硬件。
    known_macs = {mac.lower()} if mac is not None else {
        STANDARD_GLOVE_MAC, *(profile.network["mac"] for profile in _registered_profiles(device_directory())),
    }
    candidates = []
    for address in Path("/sys/class/net").glob("*/address"):
        if address.read_text(encoding="ascii").strip().lower() in known_macs:
            candidates.append(address.parent.name)
    if len(candidates) != 1:
        raise ValueError(f"无法唯一识别手套网卡（候选：{sorted(candidates)}）；请明确 --interface")
    return candidates[0]


def _check_network_ownership(network: dict, other: GloveDeviceProfile) -> None:
    same_source = (ipaddress.IPv4Interface(other.network["host_address"]).ip
                   == ipaddress.IPv4Interface(network["host_address"]).ip)
    same_table = other.network["route_table"] == network["route_table"]
    if same_source or same_table:
        raise ValueError(f"设备 {other.name} 已占用所选源地址或路由表；请为每台设备单独指定")


def _validate_registration_ownership(profile: GloveDeviceProfile, previous: GloveDeviceProfile | None) -> None:
    if previous is not None and (
        previous.hand != profile.hand or previous.device_id != profile.device_id
    ):
        raise ValueError("不能覆盖另一设备身份的档案或复用其标定；请使用独立档案")
    for other in _registered_profiles(profile.path.parent):
        if other.path == profile.path:
            continue
        _check_network_ownership(profile.network, other)


def _discover_protocol(profile: GloveDeviceProfile, *, timeout: float) -> tuple[str, int]:
    source = str(ipaddress.IPv4Interface(profile.network["host_address"]).ip)
    for protocol, port in (("encoder-v1", 9100), ("dgst-v3", 5580)):
        try:
            probe = socket.create_connection(
                (profile.network["host"], port), timeout=timeout, source_address=(source, 0),
            )
        except ConnectionRefusedError:
            if protocol == "dgst-v3":
                raise
        else:
            probe.close()
            return protocol, port
    raise ConnectionError("手套未提供 encoder-v1 或 DGST 端点")


def command_register_glove(args: argparse.Namespace) -> int:
    discover = getattr(args, "discover", False)
    requested_name = getattr(args, "name", None)
    if requested_name is not None:
        validate_profile_name(requested_name)
    device_id = getattr(args, "device_id", None)
    hand = getattr(args, "hand", None)
    if device_id is not None and (
        not isinstance(device_id, str) or not re.fullmatch(r"(?:0[xX])?[0-9a-fA-F]{16}", device_id)
    ):
        raise ValueError("--device-id 必须为完整 16 位十六进制 ID，可带 0x 前缀")
    if device_id is not None:
        device_id = f"0x{int(device_id, 16):016x}"
    if not discover and device_id is None:
        raise ValueError("离线登记必须提供 --device-id；现场可用 --discover 自动读取")
    explicit_mapping = getattr(args, "glove_config", None)
    template = getattr(args, "glove_config_template", None)
    if explicit_mapping is not None and template is not None:
        raise ValueError("--glove-config 与 --glove-config-template 不能同时指定")
    if not discover and hand is None:
        if explicit_mapping is None and template is None:
            raise ValueError("离线登记需要 --hand，或由显式机构文件读取 hand")
        hand = DataGloveUrdfMapping.load(explicit_mapping or template).hand
    protocol = getattr(args, "protocol", "auto")
    if protocol == "auto" and not discover:
        raise ValueError("离线登记必须用 --protocol 指定 encoder-v1 或 dgst-v3；auto 需要 --discover")
    if protocol == "auto" and args.port is not None:
        raise ValueError("--protocol auto 使用各协议默认端口；自定义 --port 时请明确 --protocol")
    output_arg = Path(args.output).expanduser().resolve() if args.output is not None else None
    if output_arg is not None and output_arg.suffix != ".json":
        raise ValueError("设备档案必须为 .json 文件")
    directory = output_arg.parent if output_arg is not None else device_directory()
    interface = getattr(args, "interface", None)
    if discover and interface is None:
        interface = _discover_interface(args.mac)
    network = {key: getattr(args, key) for key in
               ("host", "port", "http_port", "mtu", "host_address", "route_table")}
    network["mac"] = _registration_mac(interface, args.mac, discover=discover)
    network["protocol"] = "encoder-v1" if protocol == "auto" else protocol
    if network["port"] is None:
        network["port"] = 9100 if network["protocol"] == "encoder-v1" else 5580
    if interface is not None:
        network["interface"] = interface
    initial_metadata = None
    if discover:
        target_name = requested_name or (
            canonical_device_name(hand, device_id) if hand is not None and device_id is not None else None
        )
        target_path = output_arg or ((directory / f"{target_name}.json").resolve() if target_name else None)
        for other in _registered_profiles(directory):
            if args.overwrite and other.path == target_path:
                continue
            _check_network_ownership(network, other)
        network_args = argparse.Namespace(
            glove_profile="registration", host=network["host"], port=network["port"],
            timeout=getattr(args, "timeout", 5.0),
            **{f"glove_{key}": network.get(key) for key in
               ("interface", "mac", "host_address", "route_table", "mtu")},
        )
        prepare_glove_network(network_args, check_only=not getattr(args, "prepare_network", True))
        initial_metadata = fetch_metadata(
            network["host"], port=network["http_port"], timeout=network_args.timeout,
            source_address=str(ipaddress.IPv4Interface(network["host_address"]).ip),
            expected_device_id=device_id, expected_hand=hand,
        )
        actual_hand = initial_metadata.device["hand"]
        actual_id = initial_metadata.device["device_id"]
        if (hand is not None and actual_hand != hand) or (device_id is not None and actual_id != device_id):
            raise ValueError("实物 hand/device_id 与登记参数不一致")
        hand, device_id = actual_hand, actual_id
    name = requested_name or canonical_device_name(hand, device_id)
    output = output_arg or (directory / f"{name}.json").resolve()
    previous = GloveDeviceProfile.load(output) if output.exists() else None
    if previous is not None and not args.overwrite:
        raise FileExistsError(f"设备档案已存在：{output}")
    if previous is not None and requested_name is None:
        name = previous.name
    side = "L" if hand == "left" else "R"
    urdf = getattr(args, "glove_urdf", None) or (
        previous.glove_urdf if previous is not None else
        PROJECT_ROOT / f"assets/data_glove_urdf/data_glove_urdf_{side}/urdf/data_glove_urdf_{side}.urdf"
    )
    if previous is not None:
        mapping_data = previous.mapping_data
        if explicit_mapping is not None or template is not None:
            with Path(explicit_mapping or template).expanduser().open(encoding="utf-8") as stream:
                requested_mapping = json.load(stream)
            if requested_mapping != mapping_data:
                raise ValueError("--overwrite 必须保留已有内嵌机构配置，不能使用不同机构文件或模板")
    else:
        source_mapping = Path(
            explicit_mapping or template or PROJECT_ROOT / f"config/dataglove/urdf/{hand}.json"
        ).expanduser()
        with source_mapping.open(encoding="utf-8") as stream:
            mapping_data = json.load(stream)
        DataGloveUrdfMapping.from_dict(mapping_data)
        if explicit_mapping is None:
            mapping_data["ready"] = False
            mapping_data["directions_verified"] = False
    profile = GloveDeviceProfile.from_dict({
        "version": 4, "name": name, "hand": hand, "device_id": device_id, "network": network,
        "glove_urdf": str(Path(urdf).expanduser().resolve()), "mapping": mapping_data,
        "zeros": previous.zeros if previous is not None else {},
        "active_zero": previous.active_zero if previous is not None else None,
    }, path=output)
    if profile.path == profile.glove_urdf:
        raise ValueError("设备档案与 URDF 必须使用不同路径")
    _validate_registration_ownership(profile, previous)
    mapping = profile.load_mapping()
    if mapping.hand != hand:
        raise ValueError("mapping 与设备 hand 不一致")
    mapping.validate_urdf(profile.glove_urdf)
    cs = None
    if initial_metadata is not None:
        cs = list(initial_metadata.sensors["encoder"]["cs_by_joint"])
        if (initial_metadata.sensors["encoder"]["channels"] != 21 or len(cs) != 21
                or any(type(value) is not int for value in cs) or sorted(cs) != list(range(21))):
            raise ValueError("现场登记要求 21 通道及 0..20 的完整 CS 排列")
        if explicit_mapping is not None or previous is not None:
            mapping.validate_stream(channels=21, cs_by_joint=cs)
        else:
            mapping_data["expected_cs_by_joint"] = cs
            profile = replace(profile, mapping_data=mapping_data)
        if protocol == "auto":
            selected_protocol, selected_port = _discover_protocol(profile, timeout=network_args.timeout)
            profile = replace(profile, network={**profile.network, "protocol": selected_protocol, "port": selected_port})
        with EncoderConnection.connect(
            profile.network["host"], port=profile.network["port"],
            http_port=profile.network["http_port"], timeout=network_args.timeout,
            source_address=str(ipaddress.IPv4Interface(profile.network["host_address"]).ip),
            expected_device_id=device_id, expected_hand=hand, protocol=profile.network["protocol"],
        ) as connection:
            metadata = connection.metadata
            if metadata is None or metadata.device["device_id"] != device_id or metadata.device["hand"] != hand:
                raise ValueError("实物 hand/device_id 在能力探测期间发生变化")
            if metadata.sha256 != initial_metadata.sha256:
                raise ValueError("能力探测期间 HTTP 元数据发生变化，拒绝登记")
            if connection.channels != 21 or list(connection.cs_by_joint) != cs:
                raise ValueError("所选协议与 HTTP 元数据的编码器通道不一致")
            connection.read_frame()
        # 正常 STOP 完成后才允许提交。此处无任何错误回退路径。
    profile.validate_resources(require_zero=False)
    profile.save(overwrite=args.overwrite)
    print(f"已登记设备 {profile.name}（{profile.hand}，ID={profile.device_id}）：{profile.path}")
    if discover:
        print(f"已核验 {profile.network['protocol']} 首帧及 21 通道接线，并完成 STOP")
    print(f"内嵌标定：{_calibration_status(profile)}")
    return 0


def _calibration_status(profile: GloveDeviceProfile) -> str:
    states = []
    for name in sorted(profile.zeros):
        zero = profile.load_zero(name)
        state = "完整" if zero.complete else f"未完成（待采集：{','.join(zero.pending_groups)}）"
        active = "，当前激活" if name == profile.active_zero else ""
        states.append(f"{name}：{state}{active}")
    if not states:
        return "零位组为空，尚未采集；方向须逐项实物验收"
    return "；".join(states) + ("；尚未激活零位" if profile.active_zero is None else "")


def command_list_gloves(args: argparse.Namespace) -> int:
    for path in sorted(device_directory().glob("*.json")):
        profile = GloveDeviceProfile.load(path)
        print(f"{profile.name}\t{profile.hand}\t{profile.network['host']}:{profile.network['port']}\t{path}\t{_calibration_status(profile)}")
    return 0


def command_show_glove(args: argparse.Namespace) -> int:
    profile = GloveDeviceProfile.load(args.glove_profile)
    data = profile.to_dict()
    data["glove_urdf"] = str(profile.glove_urdf)
    print(f"唯一设备档案：{profile.path}")
    print(f"内嵌标定：{_calibration_status(profile)}")
    print(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False))
    return 0
