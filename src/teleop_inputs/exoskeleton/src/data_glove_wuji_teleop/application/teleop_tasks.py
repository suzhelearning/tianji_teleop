"""用具名任务绑定手套与可选机器人身份，不启动网络或运动。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from ..profiles.dataglove.device import GloveDeviceProfile, validate_profile_name
from ..profiles.teleop_task import HANDS, TeleopTask, task_directory


def add_task_commands(subparsers) -> None:
    bind = subparsers.add_parser("bind-task", help="一条命令绑定任务的左右手套；标定/仿真/真机共用，不启动硬件")
    bind.add_argument("--name", default="default", help="可选实验组合名称；默认保存为默认绑定 default")
    bind.add_argument("--glove-profile", dest="glove_profiles", action="append", default=[],
                      help="设备档案名或JSON路径，可重复；按档案中的hand自动分配左右手")
    for hand in HANDS:
        bind.add_argument(f"--{hand}-profile", help=f"{hand} 手套的已登记设备档案名或JSON路径")
        bind.add_argument(f"--{hand}-hand-sn", default="", help="可选固定机器人SN；不指定则启动时按手侧自动发现")
    bind.add_argument("--robot-network", help="可选机器人NetworkManager连接名，不是手套连接名")
    bind.add_argument("--robot-interface", help="可选机器人专用有线网卡；省略时仅允许无歧义自动选择")
    bind.add_argument("--output", type=Path, help="默认 config/teleoperation/tasks/<name>.json")
    bind.add_argument("--overwrite", action="store_true", help="明确替换已有任务绑定，不修改任何设备或标定文件")
    subparsers.add_parser("list-tasks", help="列出任务及各侧绑定设备")
    show = subparsers.add_parser("show-task", help="查看任务、实际设备ID、协议及标定路径")
    show.add_argument("task", help="任务短名或JSON路径")


def command_bind_task(args: argparse.Namespace) -> int:
    name = validate_profile_name(args.name)
    path = (args.output or task_directory() / f"{name}.json").expanduser().resolve()
    gloves = {}
    serials = {}
    for selected in args.glove_profiles:
        profile = GloveDeviceProfile.load(selected)
        profile.validate_resources(require_zero=False)
        if profile.hand in gloves:
            raise ValueError(f"同一绑定不能包含两只 {profile.hand} 手套")
        gloves[profile.hand] = str(profile.path)
    for hand in HANDS:
        selected = getattr(args, f"{hand}_profile")
        serial = getattr(args, f"{hand}_hand_sn")
        if selected:
            if hand in gloves:
                raise ValueError(f"{hand} 手套被重复指定")
            profile = GloveDeviceProfile.load(selected)
            profile.validate_resources(require_zero=False)
            if profile.hand != hand:
                raise ValueError(f"{selected} 是 {profile.hand}，不能绑定到 {hand}")
            gloves[hand] = str(profile.path)
        if hand not in gloves:
            if serial:
                raise ValueError(f"提供了 {hand} 机器人SN，但未绑定 {hand} 手套")
            continue
        if serial:
            serials[hand] = serial
    network = {}
    if args.robot_network:
        network["connection"] = args.robot_network
    if args.robot_interface:
        network["interface"] = args.robot_interface
    task = TeleopTask.from_dict({"version": 1, "name": name, "gloves": gloves,
                                "robot_serials": serials, "robot_network": network}, path=path)
    task.save(overwrite=args.overwrite)
    print(f"已绑定任务 {name}：{task.path}")
    for profile in task.profiles():
        print(f"  {profile.hand}: {profile.name}，设备ID={profile.device_id}，内嵌零位组={profile.active_zero}")
    print(f"仿真：pixi run official-teleop-v2 --task {name} --commission-directions")
    print("真机需另加 --confirm-real；机器人SN未绑定时会严格按手侧唯一识别。")
    return 0


def command_list_tasks(args: argparse.Namespace) -> int:
    for path in sorted(task_directory().glob("*.json")):
        task = TeleopTask.load(path)
        bindings = ", ".join(f"{profile.hand}={profile.name}" for profile in task.profiles())
        print(f"{task.name}\t{bindings}\t{path}")
    return 0


def command_show_task(args: argparse.Namespace) -> int:
    task = TeleopTask.load(args.task)
    data = task.to_dict()
    data["path"] = str(task.path)
    data["devices"] = {
        profile.hand: {
            "name": profile.name, "device_id": profile.device_id,
            "profile": str(profile.path), "protocol": profile.network["protocol"],
            "active_zero": profile.active_zero, "zero_groups": list(profile.zeros),
        }
        for profile in task.profiles()
    }
    print(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False))
    return 0
