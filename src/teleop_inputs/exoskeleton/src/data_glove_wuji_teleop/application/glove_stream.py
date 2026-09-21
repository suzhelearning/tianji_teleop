"""仅接收原始编码器数据的命令，不加载标定或机器人资源。"""

from __future__ import annotations

import argparse
import json
import math
import signal
import time

from ..adapters.glove.network import connect_glove, prepare_glove_network
from ..profiles.dataglove.device import apply_glove_profile
from ..profiles.teleop_task import apply_task_glove


def add_stream_commands(subparsers) -> None:
    stream = subparsers.add_parser("stream-glove", help="按绑定设备协议接收原始角度；默认使用 default 绑定，不应用标定")
    stream.add_argument("--glove-profile", default=None, help="显式硬件身份档案名或路径；与--task互斥")
    stream.add_argument("--task", help="具名任务；双手任务同时指定 --hand")
    stream.add_argument("--hand", choices=("left", "right"))
    stream.add_argument("--seconds", type=float, default=10.0, help="接收秒数；0 持续接收，Ctrl+C 发送 STOP 并排空")
    stream.add_argument("--timeout", type=float, default=15.0, help="收流与 STOP 排空超时，不是机器人看门狗")
    stream.add_argument("--print-interval", type=float, default=1.0, help="终端角度输出间隔；后台始终持续收流")
    stream.add_argument("--skip-network-setup", action="store_true", help="仅检查现有网络，不自动配置")
    prepare = subparsers.add_parser("prepare-glove-network", help="显式准备设备 JSON 指定的网卡；可能需要 sudo")
    prepare.add_argument("--glove-profile", default=None)
    prepare.add_argument("--task", help="具名任务；双手任务同时指定 --hand")
    prepare.add_argument("--hand", choices=("left", "right"))
    prepare.add_argument("--timeout", type=float, default=15.0)


def command_prepare_glove_network(args: argparse.Namespace) -> int:
    if args.glove_profile is None and getattr(args, "task", None) is None:
        args.task = "default"
    apply_task_glove(args)
    apply_glove_profile(args, require_resources=False)
    prepare_glove_network(args)
    return 0


def command_stream_glove(args: argparse.Namespace) -> int:
    if not math.isfinite(args.seconds) or args.seconds < 0:
        raise ValueError("--seconds 必须为非负有限秒数")
    for value, name in ((args.timeout, "timeout"), (args.print_interval, "print-interval")):
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"--{name} 必须为正有限秒数")
    if args.glove_profile is None and getattr(args, "task", None) is None:
        args.task = "default"
    apply_task_glove(args)
    profile = apply_glove_profile(args, require_resources=False)
    prepare_glove_network(args, check_only=getattr(args, "skip_network_setup", False))
    delivered = 0
    interrupted = False
    connection = connect_glove(args)
    started = time.monotonic()
    deadline = started + args.seconds if args.seconds else math.inf
    next_print = started

    def stop_on_signal(_signum, _frame):
        raise KeyboardInterrupt

    previous = signal.signal(signal.SIGTERM, stop_on_signal)
    try:
        with connection:
            print(json.dumps({
                "event": "connected", "profile": profile.name,
                "protocol": connection.protocol,
                "device": connection.metadata.device,
                "channels": connection.channels, "cs_by_joint": connection.cs_by_joint,
                "angles": "raw_absolute_deg", "calibration_applied": False,
            }, ensure_ascii=False), flush=True)
            try:
                while time.monotonic() < deadline:
                    try:
                        frame = connection.read_latest_frame(deadline_monotonic=min(deadline, time.monotonic() + args.timeout))
                    except TimeoutError:
                        if time.monotonic() >= deadline:
                            break
                        raise
                    delivered += 1
                    now = time.monotonic()
                    if now >= next_print:
                        print(json.dumps({
                            "sequence": frame.sequence, "timestamp_ns": frame.timestamp_ns,
                            "angles_deg": frame.angles_deg,
                        }, ensure_ascii=False), flush=True)
                        next_print = now + args.print_interval
            except KeyboardInterrupt:
                interrupted = True
    finally:
        signal.signal(signal.SIGTERM, previous)
        print(json.dumps({
            "event": "closed", "complete": connection.complete,
            "stop_confirmation": connection.stop_confirmation,
            "delivered_encoder_frames": delivered,
            "messages_by_kind": connection.counts,
            "elapsed_seconds": round(time.monotonic() - started, 3),
        }, ensure_ascii=False), flush=True)
    if not delivered:
        raise RuntimeError("本次连接未收到编码器帧")
    return 130 if interrupted else 0
