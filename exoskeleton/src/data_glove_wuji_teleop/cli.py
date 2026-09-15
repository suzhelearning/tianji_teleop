"""数据手套遥操作工具的稳定命令行入口。"""

from __future__ import annotations

import argparse
import shutil
import sys
import threading
from pathlib import Path

from .adapters.glove.encoder_stream import ProtocolError
from .adapters.transport.zmq_dof import DEFAULT_ZMQ_PORT, ZmqDofPublisher
from .application.calibration import (
    DEFAULT_DISABLED_CHANNELS,
    DEFAULT_INVERTED_CHANNELS,
    DEFAULT_LATERAL_TARGET_DEG,
    DEFAULT_PORT,
    MAX_LATERAL_TARGET_DEG,
    command_calibrate,
    command_inspect,
)
from .application.parallel_hands import run_parallel_hands
from .application.teleoperation import TeleoperationOptions, run_teleoperation
from .application.urdf_zero_calibration import command_calibrate_urdf_zero
from .application.glove_stream import (
    add_stream_commands,
    command_prepare_glove_network,
    command_stream_glove,
)
from .application.glove_profiles import (
    add_profile_commands,
    command_list_gloves,
    command_register_glove,
    command_show_glove,
)
from .application.teleop_tasks import (
    add_task_commands,
    command_bind_task,
    command_list_tasks,
    command_show_task,
)
from .profiles.dataglove.mapping import (
    CalibrationProfile,
    apply_faulty_encoder_reuse,
)


def _validate_profile_hand(calibration_file: str, hand: str) -> None:
    if not calibration_file:
        raise ValueError(f"{hand} 模式必须提供标定文件")
    profile = CalibrationProfile.load(calibration_file)
    if profile.hand != hand:
        raise ValueError(
            f"{calibration_file} 是 {profile.hand} 标定，不能用于 {hand}"
        )


def _run_one_hand(
    args: argparse.Namespace,
    *,
    hand: str,
    calibration_file: str,
    glove_host: str,
    glove_port: int,
    zmq_port: int,
    stop_event: threading.Event | None = None,
    install_signal_handlers: bool = True,
) -> int:
    """装配并运行指定一侧的手套输入和目标输出。"""

    _validate_profile_hand(calibration_file, hand)
    publisher = None
    if not args.dry_run:
        selected_port = zmq_port or DEFAULT_ZMQ_PORT[hand]
        publisher = ZmqDofPublisher(args.zmq_bind, selected_port)
        print(
            f"[ZMQ:{hand}] PUB tcp://{args.zmq_bind}:{selected_port}"
        )
    options = TeleoperationOptions(
        calibration_file=calibration_file,
        glove_host=glove_host,
        glove_port=glove_port,
        rate_hz=args.rate,
        reconnect_seconds=args.reconnect_seconds,
        duration_seconds=args.duration,
        print_interval_seconds=args.print_interval,
        dry_run=args.dry_run,
    )
    try:
        return run_teleoperation(
            options,
            publisher,
            stop_event=stop_event,
            install_signal_handlers=install_signal_handlers,
        )
    finally:
        if publisher is not None:
            publisher.close()


def _run_both_hands(args: argparse.Namespace) -> int:
    for hand in ("left", "right"):
        if not getattr(args, f"{hand}_host"):
            raise ValueError(f"both 模式必须显式提供 --{hand}-host")
        _validate_profile_hand(
            getattr(args, f"{hand}_cal_file"),
            hand,
        )

    def worker(hand: str, stop: threading.Event) -> int:
        return _run_one_hand(
            args,
            hand=hand,
            calibration_file=getattr(args, f"{hand}_cal_file"),
            glove_host=getattr(args, f"{hand}_host"),
            glove_port=getattr(args, f"{hand}_port"),
            zmq_port=getattr(args, f"{hand}_zmq_port"),
            stop_event=stop,
            install_signal_handlers=False,
        )

    return run_parallel_hands(worker, thread_prefix="dataglove")


def command_run(args: argparse.Namespace) -> int:
    """按 --hand 装配单手或双手遥操作链路。"""

    if args.hand == "both":
        return _run_both_hands(args)
    return _run_one_hand(
        args,
        hand=args.hand,
        calibration_file=args.cal_file,
        glove_host=args.host,
        glove_port=args.port,
        zmq_port=args.zmq_port,
    )


def command_reuse_faulty_encoders(args: argparse.Namespace) -> int:
    """把临时编码器复用策略写入现有标定文件，并保留原始备份。"""

    calibration_path = Path(args.cal_file)
    profile = CalibrationProfile.load(calibration_path)
    if profile.hand != args.hand:
        raise ValueError(
            f"{calibration_path} 是 {profile.hand} 标定，"
            f"不能应用到 {args.hand}"
        )
    reused = apply_faulty_encoder_reuse(profile)
    if args.backup:
        backup_path = Path(args.backup)
    else:
        backup_path = calibration_path.with_name(
            f"{calibration_path.stem}.pre-reuse.backup"
            f"{calibration_path.suffix}"
        )
        sequence = 2
        while backup_path.exists():
            backup_path = calibration_path.with_name(
                f"{calibration_path.stem}.pre-reuse.{sequence}.backup"
                f"{calibration_path.suffix}"
            )
            sequence += 1
    if backup_path.resolve() == calibration_path.resolve():
        raise ValueError("备份文件不能与标定文件相同")
    if backup_path.exists():
        raise ValueError(f"备份文件已存在，拒绝覆盖: {backup_path}")
    else:
        backup_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(calibration_path, backup_path)
        print(f"已备份原标定：{backup_path}")

    reused.save(calibration_path)
    print(f"故障编码器复用已应用：{calibration_path}")
    print("  小拇指屈伸: J10<-J9, J15<-J14, J20<-J19")
    print("  小拇指侧摆: J5=0")
    print("  大拇指掌骨屈伸: J6<-J16")
    print("  大拇指侧摆: J1 保持独立输入")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="数据手套到 Wuji 的尺度标定、检查与实时目标桥"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    add_profile_commands(subparsers)
    add_stream_commands(subparsers)
    add_task_commands(subparsers)

    inspect_parser = subparsers.add_parser("inspect", help="只读检查实时 21 路数据")
    inspect_parser.add_argument("--host", default="192.168.7.2")
    inspect_parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    inspect_parser.add_argument("--seconds", type=float, default=3.0)

    calibrate_parser = subparsers.add_parser(
        "calibrate",
        help="交互式五步零位与尺度标定（大拇指独立）",
    )
    calibrate_parser.add_argument(
        "--hand",
        choices=("left", "right"),
        required=True,
    )
    calibrate_parser.add_argument("--host", default="192.168.7.2")
    calibrate_parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    calibrate_parser.add_argument("--output", default="")
    calibrate_parser.add_argument(
        "--disabled",
        default=",".join(DEFAULT_DISABLED_CHANNELS),
        help="逗号分隔的屏蔽通道",
    )
    calibrate_parser.add_argument(
        "--invert",
        default=None,
        help="输出方向取反的通道；左手默认 J1..J5，右手默认不反转",
    )
    calibrate_parser.add_argument("--sample-seconds", type=float, default=1.5)
    calibrate_parser.add_argument("--sweep-seconds", type=float, default=8.0)
    calibrate_parser.add_argument(
        "--deadband-deg",
        type=float,
        default=0.0,
        help="零位死区（度，默认 0；仅仿真发现零位抖动时再增加）",
    )
    calibrate_parser.add_argument(
        "--finger-flex-target-deg",
        default="80,80,60",
        help="四指三段最大输出角（度）",
    )
    calibrate_parser.add_argument(
        "--thumb-flex-target-deg",
        default="",
        help="拇指三段最大输出角（度）；默认按左右手选择",
    )
    calibrate_parser.add_argument(
        "--lateral-target-deg",
        type=float,
        default=DEFAULT_LATERAL_TARGET_DEG,
        help=f"侧摆最大输出角（度，范围 (0,{MAX_LATERAL_TARGET_DEG:g}]）",
    )
    calibrate_parser.add_argument("--max-stable-motion-deg", type=float, default=3.0)
    calibrate_parser.add_argument(
        "--max-stable-motion-ratio",
        type=float,
        default=0.1,
        help="稳定姿态允许的相对运动比例（默认 0.1）",
    )
    calibrate_parser.add_argument("--min-flex-travel-deg", type=float, default=5.0)
    calibrate_parser.add_argument(
        "--min-lateral-travel-deg",
        "--min-lateral-each-side-deg",
        dest="min_lateral_travel_deg",
        type=float,
        default=3.0,
        help="侧摆至少一个方向的最小有效量程（度）",
    )

    urdf_zero_parser = subparsers.add_parser(
        "calibrate-urdf-zero",
        help=(
            "按五根手指和小拇指 CMC 六步标定"
            "实物手套 URDF 软件零位"
        ),
    )
    urdf_zero_parser.add_argument(
        "--hand",
        choices=("left", "right"),
        default=None,
        help="仅指定手侧时自动识别实物 ID，按设备保存零位，完整后更新 default 绑定",
    )
    urdf_zero_parser.add_argument("--glove-profile", default=None)
    urdf_zero_parser.add_argument("--task", help="具名任务；双手任务同时指定 --hand")
    urdf_zero_parser.add_argument(
        "--name",
        help="设备 JSON 内创建或续采具名零位组；默认活动组或 initial，六步完整后激活",
    )
    urdf_zero_parser.add_argument("--host", default=None)
    urdf_zero_parser.add_argument("--port", type=int, default=None)
    urdf_zero_parser.add_argument("--skip-network-setup", action="store_true")
    urdf_zero_parser.add_argument("--usb-wait-timeout", type=float, default=None)
    urdf_zero_parser.add_argument("--timeout", type=float, default=5.0)
    urdf_zero_parser.add_argument(
        "--output", default="", help="独立零位文件输出；不能与设备档案或 --name 同用",
    )
    urdf_zero_parser.add_argument("--sample-seconds", type=float, default=1.5)
    urdf_zero_parser.add_argument("--max-motion-deg", type=float, default=1.5)

    reuse_parser = subparsers.add_parser(
        "reuse-faulty-encoders",
        help="将临时坏编码器复用策略应用到现有标定文件",
    )
    reuse_parser.add_argument(
        "--hand",
        choices=("left", "right"),
        required=True,
    )
    reuse_parser.add_argument("--cal-file", required=True)
    reuse_parser.add_argument(
        "--backup",
        default="",
        help="备份路径；默认生成 *.pre-reuse.backup.json",
    )

    run_parser = subparsers.add_parser(
        "run",
        help="按 hand=left/right/both 发布一代手目标",
    )
    run_parser.add_argument(
        "--hand",
        choices=("left", "right", "both"),
        required=True,
    )
    run_parser.add_argument("--cal-file", default="")
    run_parser.add_argument("--host", default="192.168.7.2")
    run_parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    run_parser.add_argument("--left-cal-file", default="")
    run_parser.add_argument("--right-cal-file", default="")
    run_parser.add_argument("--left-host", default="")
    run_parser.add_argument("--right-host", default="")
    run_parser.add_argument("--left-port", type=int, default=DEFAULT_PORT)
    run_parser.add_argument("--right-port", type=int, default=DEFAULT_PORT)
    run_parser.add_argument("--zmq-bind", default="127.0.0.1")
    run_parser.add_argument("--zmq-port", type=int, default=0)
    run_parser.add_argument("--left-zmq-port", type=int, default=0)
    run_parser.add_argument("--right-zmq-port", type=int, default=0)
    run_parser.add_argument("--rate", type=float, default=100.0)
    run_parser.add_argument("--reconnect-seconds", type=float, default=1.0)
    run_parser.add_argument("--dry-run", action="store_true")
    run_parser.add_argument("--duration", type=float, default=0.0)
    run_parser.add_argument("--print-interval", type=float, default=1.0)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        if args.command == "bind-task":
            return command_bind_task(args)
        if args.command == "list-tasks":
            return command_list_tasks(args)
        if args.command == "show-task":
            return command_show_task(args)
        if args.command == "stream-glove":
            return command_stream_glove(args)
        if args.command == "prepare-glove-network":
            return command_prepare_glove_network(args)
        if args.command == "register-glove":
            return command_register_glove(args)
        if args.command == "list-gloves":
            return command_list_gloves(args)
        if args.command == "show-glove":
            return command_show_glove(args)
        if args.command == "inspect":
            return command_inspect(args)
        if args.command == "calibrate":
            return command_calibrate(args)
        if args.command == "calibrate-urdf-zero":
            return command_calibrate_urdf_zero(args)
        if args.command == "reuse-faulty-encoders":
            return command_reuse_faulty_encoders(args)
        if args.command == "run":
            return command_run(args)
    except KeyboardInterrupt:
        if args.command == "calibrate-urdf-zero":
            print(
                "\n已取消；已通过步骤的进度已保留。",
                file=sys.stderr,
            )
        else:
            print("\n已取消，未写入标定文件。", file=sys.stderr)
        return 130
    except (EOFError, OSError, ProtocolError, RuntimeError, ValueError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 2
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
