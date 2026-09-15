"""Hand2 身份预检：只扫描、连接、GET 和断开，绝不使能或写设备资源。"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from uuid import uuid4

from .network import prepare_robot_network
from .wuji_sdk_driver import _load_sdk, _normalized_hand_side


def resolve_robot_serials(
    requested: Mapping[str, str | None],
    *,
    network_connection: str | None = None,
    network_interface: str | None = None,
    excluded_interfaces: Sequence[str] = (),
    prepare_network: bool = True,
) -> dict[str, str]:
    """原子解析请求的左右手 SN；任何身份、网络、关节或清理错误都不返回部分结果。

    空 SN 按设备 GET 的 handedness 选择，不使用序列号命名规律。
    自动网络只支持 RFC1918 IPv4 /24；其他网络须由用户先配置直连路由。
    调用方必须先完成仿真首帧检查，并在启动控制前重新检查仿真新鲜度。
    """
    if not requested or any(side not in ("left", "right") for side in requested):
        raise ValueError("请求必须包含 left 和/或 right")
    for serial in requested.values():
        if serial is not None and (not isinstance(serial, str) or serial != serial.strip()):
            raise ValueError("SN 必须是无首尾空白的字符串或 None")
    explicit = [serial for serial in requested.values() if serial]
    if len(set(explicit)) != len(explicit):
        raise ValueError("同一个 SN 不能绑定双手")

    sdk = _load_sdk()
    manager = sdk.SdkManager.instance()
    scanned = list(manager.scan())
    for serial in explicit:
        matches = [info for info in scanned if info.sn == serial]
        if len(matches) != 1 or matches[0].device_type != sdk.DeviceType.WujiHand2:
            raise RuntimeError(f"未唯一发现指定的 WujiHand2 SN={serial}；不会替换 SN")
    automatic = any(not serial for serial in requested.values())
    candidates = [
        info for info in scanned
        if info.device_type == sdk.DeviceType.WujiHand2
        and (automatic or info.sn in explicit)
    ]
    if not candidates:
        raise RuntimeError("未发现 WujiHand2")
    serials = [info.sn for info in candidates]
    if any(not serial for serial in serials) or len(set(serials)) != len(serials):
        raise RuntimeError("扫描结果包含空 SN 或重复 SN，无法唯一绑定")
    addresses = []
    for info in candidates:
        if info.transport_type == sdk.TransportType.Udp:
            addresses.append(info.address)
        elif info.transport_type != sdk.TransportType.Usb:
            raise RuntimeError(f"SN={info.sn} 的传输类型不受支持")
    if addresses:
        prepare_robot_network(
            addresses,
            network_connection=network_connection,
            network_interface=network_interface,
            excluded_interfaces=excluded_interfaces,
            prepare_network=prepare_network,
        )

    identities: dict[str, tuple[str, int]] = {}
    for info in candidates:
        name = f"hand2_identity_{uuid4().hex}"
        failure: BaseException | None = None
        try:
            # 即使 connect 抛错也尝试清理该唯一名字，不触碰其他 SDK 连接。
            device = manager.connect(
                sn=info.sn,
                device_name=name,
                options=sdk.ConnectOptions(
                    enable_bridge=False, retry_count=0,
                    auto_time_sync_interval_ms=None,
                ),
            )
            if not isinstance(device, sdk.WujiHand2):
                raise TypeError(f"SN={info.sn} 返回的设备不是 WujiHand2")
            if device.serial_number != info.sn:
                raise RuntimeError(f"扫描 SN={info.sn} 与连接后的真实 SN 不一致")
            side = _normalized_hand_side(device.handedness().get())
            if side not in ("left", "right"):
                raise RuntimeError(f"SN={info.sn} 的真实手侧无法识别：{side}")
            identities[info.sn] = (side, device.online_joints_count().get())
        except BaseException as exc:
            failure = exc
            raise
        finally:
            try:
                manager.disconnect(name)
            except BaseException as cleanup_error:
                if failure is None:
                    raise
                raise BaseExceptionGroup(
                    f"SN={info.sn} 预检失败且断开失败", [failure, cleanup_error]
                ) from None

    resolved = {}
    for side, serial in requested.items():
        matches = ([serial] if serial else [
            sn for sn, (actual, _) in identities.items() if actual == side
        ])
        if len(matches) != 1:
            raise RuntimeError(f"{side} 手候选数量为 {len(matches)}，请指定唯一真实 SN")
        selected = matches[0]
        actual, online = identities[selected]
        if actual != side:
            raise ValueError(f"SN={selected} 手侧不匹配：请求 {side}，实际 {actual}")
        if online != 20:
            raise RuntimeError(f"SN={selected} 仅 {online}/20 关节在线，拒绝启动")
        resolved[side] = selected
    if len(set(resolved.values())) != len(resolved):
        raise ValueError("同一个 SN 不能绑定双手")
    return resolved
