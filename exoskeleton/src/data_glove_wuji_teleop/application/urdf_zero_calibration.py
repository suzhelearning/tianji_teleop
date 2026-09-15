"""实物数据手套自身 URDF 的六步交互软件零位标定。"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from ..adapters.glove.network import connect_glove, prepare_glove_network
from ..profiles.dataglove.device import apply_glove_profile, validate_profile_name
from ..profiles.teleop_task import apply_task_glove, bind_default_glove
from ..profiles.dataglove.urdf_zero import (
    URDF_ZERO_GROUPS,
    UrdfZeroCaptureSummary,
    UrdfZeroProfile,
)
from .calibration import DEFAULT_PORT
from .encoder_capture import read_encoder_window

from .glove_discovery import discover_glove_for_calibration


def _print_capture(summary: UrdfZeroCaptureSummary) -> None:
    print(f"{summary.group.label} 零位采样通过：")
    for channel, offset, motion in zip(
        summary.group.channels,
        summary.offsets_deg,
        summary.motion_deg,
    ):
        print(
            f"  {channel}: zero={offset:8.3f}° "
            f"motion80={motion:.3f}°"
        )


def command_calibrate_urdf_zero(args: argparse.Namespace) -> int:
    """依次采五根手指，再单独采小拇指 CMC，并逐步保存。"""

    sample_seconds = float(args.sample_seconds)
    max_motion_deg = float(args.max_motion_deg)
    timeout = float(args.timeout)
    if not math.isfinite(sample_seconds) or sample_seconds <= 0.0:
        raise ValueError("--sample-seconds 必须是正有限数值")
    if not math.isfinite(max_motion_deg) or max_motion_deg < 0.0:
        raise ValueError("--max-motion-deg 必须是非负有限数值")
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError("--timeout 必须是正有限数值")
    name = getattr(args, "name", None)
    explicit_output = getattr(args, "output", "")
    if name is not None:
        validate_profile_name(name)
        if explicit_output:
            raise ValueError("--name 与 --output 不能同时使用")

    automatic = (
        args.hand in ("left", "right")
        and getattr(args, "task", None) is None
        and getattr(args, "glove_profile", None) is None
        and getattr(args, "host", None) is None
        and getattr(args, "port", None) is None
        and not explicit_output
    )
    if automatic:
        discovered = discover_glove_for_calibration(
            args.hand, timeout=timeout,
            prepare_network=not getattr(args, "skip_network_setup", False),
        )
        args.glove_profile = str(discovered.path)

    if (
        getattr(args, "task", None) is None and getattr(args, "glove_profile", None) is None
        and args.hand is None and getattr(args, "host", None) is None
        and getattr(args, "port", None) is None and not getattr(args, "output", "")
    ):
        args.task = "default"
    apply_task_glove(args)
    selected = getattr(args, "glove_profile", None)
    if selected is not None:
        if explicit_output:
            raise ValueError("--glove-profile 不能与 --output 同时使用；请用 --name 创建零位")
        if getattr(args, "host", None) is not None or getattr(args, "port", None) is not None:
            raise ValueError("--glove-profile 的网络绑定不能由 --host/--port 覆盖")
    device = apply_glove_profile(args, require_zero=False)
    mapping = device.load_mapping() if device is not None else None
    if args.hand not in ("left", "right"):
        raise ValueError("未选择设备档案时必须提供 --hand")
    if device is None:
        if not explicit_output:
            raise ValueError("独立零位模式必须提供 --output；设备内具名零位请指定 --glove-profile")
        args.host = getattr(args, "host", None) or "192.168.7.2"
        args.port = DEFAULT_PORT if getattr(args, "port", None) is None else args.port
        output = Path(explicit_output)
        profile = UrdfZeroProfile.load(output) if output.exists() else None
        destination = str(output)
    else:
        name = name or device.active_zero or "initial"
        profile = device.load_zero(name)
        destination = f"{device.path}（零位组：{name}）"
    if profile is not None and profile.hand != args.hand:
        raise ValueError(
            f"{destination} 是 {profile.hand} 零位，不能用于 {args.hand}"
        )
    if profile is not None and mapping is not None:
        mapping.validate_stream(channels=21, cs_by_joint=profile.expected_cs_by_joint)
    if profile is not None and profile.complete:
        if device is not None and name != device.active_zero:
            device = device.activate_zero(name)
        if automatic:
            bind_default_glove(device)
            print(f"已更新 default 的 {device.hand} 绑定：{device.name}")
        print(f"URDF 软件零位已经完整：{destination}")
        return 0

    print("数据手套自身 URDF 软件零位标定")
    print(
        "顺序：大拇指 → 食指 → 中指 → 无名指 → "
        "小拇指 → 小拇指 CMC"
    )
    print(
        "每一步只更新所列通道；"
        "其他手指不需要同时保持 URDF 零位。"
    )
    print("采样值会在四连杆换算前作为原始编码器零位扣除。\n")
    if device is not None:
        prepare_glove_network(args)

    captured = set(profile.captured_groups) if profile is not None else set()
    for step, group in enumerate(URDF_ZERO_GROUPS, start=1):
        if group.name in captured:
            print(f"步骤 {step}/6 已完成，跳过：{group.label}")
            continue
        channels = "、".join(group.channels)
        while True:
            input(
                f"\n步骤 {step}/6：{group.label}\n"
                f"将 {group.label} 摆到 URDF 零位并保持静止；"
                f"本步采集 {channels}。按 Enter 开始采样..."
            )
            capture_options = {"timeout": timeout}
            if device is not None:
                capture_options["connection_factory"] = lambda: connect_glove(args)
            window = read_encoder_window(
                args.host,
                args.port,
                sample_seconds,
                **capture_options,
            )
            if window.channels != 21:
                raise ValueError(
                    f"手套返回 {window.channels} 路，URDF 零位要求 21 路"
                )
            if mapping is not None:
                mapping.validate_stream(channels=window.channels, cs_by_joint=window.mapping)
            if profile is None:
                profile = UrdfZeroProfile.empty(
                    args.hand,
                    window.mapping,
                    stream_zeroed=window.zeroed,
                    range_min=window.range_min,
                    range_max=window.range_max,
                )
            else:
                profile.validate_stream(
                    window.channels,
                    window.mapping,
                    zeroed=window.zeroed,
                    range_min=window.range_min,
                    range_max=window.range_max,
                )
            print(
                f"已连接 {args.host}:{args.port}，"
                f"zeroed={str(window.zeroed).lower()}（仅记录）"
            )
            try:
                updated, summary = profile.capture_group(
                    group.name,
                    (frame.angles_deg for frame in window.frames),
                    max_motion_deg=max_motion_deg,
                )
            except ValueError as exc:
                print(
                    f"本步骤未通过：{exc}\n"
                    "保持当前手指静止后重新采样，"
                    "或按 Ctrl+C 取消。",
                    file=sys.stderr,
                )
                continue
            profile = updated
            if device is not None:
                device = device.store_zero(name, profile, activate=profile.complete)
            else:
                profile.save(output)
            _print_capture(summary)
            print(f"已保存进度：{destination}")
            break
    if automatic:
        bind_default_glove(device)
        print(f"已更新 default 的 {device.hand} 绑定：{device.name}")

    print(f"\n六步 URDF 软件零位标定完成：{destination}")
    return 0
