"""Wuji 一代/二代左右及双手真机遥操作入口。"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from contextlib import ExitStack, contextmanager
from dataclasses import dataclass

from .adapters.hardware.command_safety import FirmwareCommandMapper
from .adapters.hardware.safe_publisher import (
    HardwareSafetyOptions,
    SafeHardwarePublisher,
)
from .adapters.hardware.wuji_sdk_driver import (
    DEFAULT_V2_KD,
    DEFAULT_V2_KP,
    open_wuji_sdk_driver,
    validate_v2_mit_gains,
)
from .adapters.hardware.wuji_v2_limits import load_v2_firmware_limits
from .adapters.runtime.simulation_session import require_active_simulation
from .adapters.transport.zmq_dof import (
    DEFAULT_ZMQ_PORT,
    ZmqTargetSubscriber,
)
from .application.calibration import DEFAULT_PORT
from .application.parallel_hands import run_parallel_hands
from .project import get_hand_resources


DEFAULT_REAL_RATE_HZ = {
    "v1": 20.0,
    "v2": 50.0,
}
DEFAULT_INPUT_FILTER_HZ = {
    "v1": 5.0,
    "v2": 5.0,
}
# 官方 IK 目标约 20 Hz（实测最大帧间隔约 56 ms）；允许 50 Hz 真机循环
# 在下一帧到达前继续重采样最近目标，真正断流仍由 command_timeout 处理。
REALTIME_TARGET_MAX_AGE_SECONDS = 0.075
DEFAULT_HOME_TOLERANCE_RAD = {
    "v1": 0.01,
    "v2": 0.05,
}


@dataclass(frozen=True)
class RealSideSettings:
    """一侧真机及仿真目标流的显式装配参数。"""

    hand: str
    serial: str
    target_host: str
    target_port: int


@contextmanager
def _interrupt_on_shutdown_signals():
    """初始化阶段将可捕获关闭信号转成可清理的 Python 中断。"""

    previous_handlers = {}

    def interrupt(_signum=None, _frame=None) -> None:
        raise KeyboardInterrupt

    try:
        for name in ("SIGINT", "SIGTERM", "SIGHUP"):
            signum = getattr(signal, name, None)
            if signum is not None:
                previous_handlers[signum] = signal.getsignal(signum)
                signal.signal(signum, interrupt)
        yield
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


def _simulation_check(generation: str, hand: str):
    return require_active_simulation(generation=generation, hand=hand)


def open_real_publisher(
    *,
    generation: str,
    hand: str,
    serial: str,
    safety: HardwareSafetyOptions,
    arm_command_timeout: bool = False,
    kp: float = DEFAULT_V2_KP,
    kd: float = DEFAULT_V2_KD,
) -> SafeHardwarePublisher:
    """在验证同代同侧仿真后，连接并安全初始化一侧真机。"""

    _simulation_check(generation, hand)
    resources = get_hand_resources(hand, generation=generation)
    lower = None
    upper = None
    mapper = None
    if generation == "v2":
        if resources.mjcf is None:
            raise ValueError("Wuji v2 官方 MJCF 路径未配置")
        lower, upper = load_v2_firmware_limits(
            model_path=resources.mjcf,
            config_path=resources.mapping,
            hand=hand,
        )
        mapper = FirmwareCommandMapper.from_config(
            resources.mapping,
            generation=generation,
            hand=hand,
            lower=lower,
            upper=upper,
        )

    driver = open_wuji_sdk_driver(
        generation=generation,
        hand=hand,
        serial=serial,
        kp=kp,
        kd=kd,
        lower=lower,
        upper=upper,
    )
    if mapper is None:
        try:
            mapper = FirmwareCommandMapper.from_config(
                resources.mapping,
                generation=generation,
                hand=hand,
                lower=driver.lower,
                upper=driver.upper,
            )
        except BaseException:
            driver.close()
            raise

    return SafeHardwarePublisher.open(
        driver=driver,
        mapper=mapper,
        simulation_check=lambda: _simulation_check(generation, hand),
        options=safety,
        arm_command_timeout=arm_command_timeout,
    )


def _side_settings(
    args: argparse.Namespace,
) -> tuple[RealSideSettings, ...]:
    if args.hand in ("left", "right"):
        hand = args.hand
        if not args.hand_sn:
            raise ValueError("单手真机控制必须提供 --hand-sn")
        return (
            RealSideSettings(
                hand=hand,
                serial=args.hand_sn,
                target_host=args.target_host,
                target_port=(
                    args.target_port or DEFAULT_ZMQ_PORT[hand]
                ),
            ),
        )

    if not args.left_hand_sn or not args.right_hand_sn:
        raise ValueError(
            "both 真机控制必须提供 --left-hand-sn 和 --right-hand-sn"
        )
    if args.left_hand_sn == args.right_hand_sn:
        raise ValueError("左右手真机 SN 不能相同")
    return tuple(
        RealSideSettings(
            hand=hand,
            serial=getattr(args, f"{hand}_hand_sn"),
            target_host=getattr(args, f"{hand}_target_host"),
            target_port=(
                getattr(args, f"{hand}_target_port")
                or DEFAULT_ZMQ_PORT[hand]
            ),
        )
        for hand in ("left", "right")
    )


def _open_target_receivers(
    settings: tuple[RealSideSettings, ...],
) -> dict[str, ZmqTargetSubscriber]:
    """在连接真机前确认仿真手套桥已经发布同侧目标。"""

    receivers: dict[str, ZmqTargetSubscriber] = {}
    try:
        for side in settings:
            receiver = ZmqTargetSubscriber(
                side.target_host,
                side.target_port,
                side.hand,
            )
            receivers[side.hand] = receiver
            deadline = time.monotonic() + 3.0
            target = None
            while target is None and time.monotonic() < deadline:
                target = receiver.recv()
            if target is None:
                raise RuntimeError(
                    f"未收到 {side.hand} 仿真目标："
                    f"tcp://{side.target_host}:{side.target_port}"
                )
            print(
                f"[目标:{side.hand}] 已检测到仿真手套目标 "
                f"tcp://{side.target_host}:{side.target_port}"
            )
        return receivers
    except BaseException:
        for receiver in receivers.values():
            receiver.close()
        raise


def _run_target_side(
    args: argparse.Namespace,
    settings: RealSideSettings,
    receiver: ZmqTargetSubscriber,
    publisher: SafeHardwarePublisher,
    *,
    stop_event: threading.Event | None = None,
) -> int:
    """以代际频率把仿真手套桥的最新目标转发给一侧真机。"""

    if getattr(args, "tracking_mode", "legacy") == "realtime":
        return _run_realtime_target_side(
            args,
            settings,
            receiver,
            publisher,
            stop_event=stop_event,
        )

    stop = stop_event if stop_event is not None else threading.Event()
    rate_hz = args.rate or DEFAULT_REAL_RATE_HZ[args.generation]
    period = 1.0 / rate_hz
    started_at = time.monotonic()
    last_received_at = started_at
    next_publish_at = started_at
    received = 0
    sent = 0
    while not stop.is_set():
        now = time.monotonic()
        if args.duration > 0.0 and now - started_at >= args.duration:
            break
        target = receiver.recv()
        now = time.monotonic()
        if target is None:
            if now - last_received_at > args.command_timeout:
                raise RuntimeError(
                    f"{settings.hand} 仿真手套目标超时"
                )
            continue
        last_received_at = now
        received += 1
        if now < next_publish_at:
            continue
        publisher.publish(target)
        sent += 1
        next_publish_at = now + period
    print(
        f"[结束:{settings.hand}] ZMQ received={received} sent={sent}"
    )
    return 0


def _run_realtime_target_side(
    args: argparse.Namespace,
    settings: RealSideSettings,
    receiver: ZmqTargetSubscriber,
    publisher: SafeHardwarePublisher,
    *,
    stop_event: threading.Event | None = None,
) -> int:
    """固定 50 Hz 发布接收线程覆盖保存的最新有效目标。"""

    stop = stop_event if stop_event is not None else threading.Event()
    receiver_stop = threading.Event()
    state_lock = threading.Lock()
    latest_target = None
    latest_received_at: float | None = None
    receive_error: BaseException | None = None
    received = 0

    def receive_latest() -> None:
        nonlocal latest_target, latest_received_at, receive_error, received
        try:
            while not stop.is_set() and not receiver_stop.is_set():
                target = receiver.recv()
                if target is None:
                    continue
                with state_lock:
                    latest_target = target
                    latest_received_at = time.monotonic()
                    received += 1
        except BaseException as exc:
            with state_lock:
                receive_error = exc

    receive_thread = threading.Thread(
        target=receive_latest,
        name=f"realtime-target-{settings.hand}",
        daemon=True,
    )
    receive_thread.start()

    rate_hz = args.rate or DEFAULT_REAL_RATE_HZ[args.generation]
    period = 1.0 / rate_hz
    started_at = time.monotonic()
    next_publish_at = started_at
    sent = 0
    try:
        while not stop.is_set():
            now = time.monotonic()
            if args.duration > 0.0 and now - started_at >= args.duration:
                break
            with state_lock:
                target = latest_target
                target_received_at = latest_received_at
                error = receive_error
            if error is not None:
                raise error
            if target is None or target_received_at is None:
                if now - started_at > args.command_timeout:
                    raise RuntimeError(
                        f"{settings.hand} 仿真手套目标超时"
                    )
            else:
                target_age = now - target_received_at
                if target_age > args.command_timeout:
                    raise RuntimeError(
                        f"{settings.hand} 仿真手套目标超时"
                    )
                if target_age <= REALTIME_TARGET_MAX_AGE_SECONDS:
                    publisher.publish(target)
                    sent += 1

            next_publish_at += period
            after_publish = time.monotonic()
            if next_publish_at <= after_publish:
                missed = int(
                    (after_publish - next_publish_at) / period
                ) + 1
                next_publish_at += missed * period
            stop.wait(max(0.0, next_publish_at - after_publish))
    finally:
        receiver_stop.set()
        receive_thread.join(timeout=0.1)
    print(
        f"[结束:{settings.hand}:realtime] "
        f"ZMQ received={received} sent={sent}"
    )
    return 0


def _run_both(
    args: argparse.Namespace,
    settings: tuple[RealSideSettings, ...],
    receivers: dict[str, ZmqTargetSubscriber],
    publishers: dict[str, SafeHardwarePublisher],
) -> int:
    by_hand = {side.hand: side for side in settings}

    def worker(hand: str, stop: threading.Event) -> int:
        side = by_hand[hand]
        return _run_target_side(
            args,
            side,
            receivers[hand],
            publishers[hand],
            stop_event=stop,
        )

    return run_parallel_hands(
        worker,
        thread_prefix=f"real-{args.generation}",
    )


def _run_publishers(
    args: argparse.Namespace,
    settings: tuple[RealSideSettings, ...],
    safety: HardwareSafetyOptions,
) -> int:
    publishers: dict[str, SafeHardwarePublisher] = {}
    with ExitStack() as cleanup:
        receivers = _open_target_receivers(settings)
        for receiver in receivers.values():
            cleanup.callback(receiver.close)

        for side in settings:
            publisher = open_real_publisher(
                generation=args.generation,
                hand=side.hand,
                serial=side.serial,
                safety=safety,
                arm_command_timeout=False,
                kp=getattr(args, "kp", DEFAULT_V2_KP),
                kd=getattr(args, "kd", DEFAULT_V2_KD),
            )
            publishers[side.hand] = publisher
            cleanup.callback(publisher.close)
            print(
                f"[真机:{args.generation}:{side.hand}] "
                f"已清错、使能并回零，SN={side.serial}"
            )

        if len(settings) == 1:
            side = settings[0]
            return _run_target_side(
                args,
                side,
                receivers[side.hand],
                publishers[side.hand],
            )
        return _run_both(args, settings, receivers, publishers)


def command_real(args: argparse.Namespace) -> int:
    """独立启动真机控制；不创建或关闭 MuJoCo 进程。"""

    if not args.confirm_real:
        raise ValueError(
            "真机控制必须显式添加 --confirm-real，"
            "并确认急停可用且手周围没有障碍物"
        )
    if args.rate < 0.0:
        raise ValueError("--rate 必须为 0 或正数")
    if args.input_filter_hz < 0.0:
        raise ValueError("--input-filter-hz 必须为 0 或正数")
    if args.duration < 0.0:
        raise ValueError("--duration 不能为负数")
    if args.home_tolerance < 0.0:
        raise ValueError("--home-tolerance 必须为 0 或正数")
    if args.generation == "v2":
        validate_v2_mit_gains(args.kp, args.kd)
    selected_rate = args.rate or DEFAULT_REAL_RATE_HZ[args.generation]
    selected_filter = (
        args.input_filter_hz
        or DEFAULT_INPUT_FILTER_HZ[args.generation]
    )
    selected_home_tolerance = (
        args.home_tolerance
        or DEFAULT_HOME_TOLERANCE_RAD[args.generation]
    )
    if selected_filter >= selected_rate * 0.5:
        raise ValueError("输入滤波截止频率必须低于真机发布频率的一半")
    if args.tracking_mode == "realtime":
        if args.generation != "v2":
            raise ValueError("realtime 模式当前仅支持 Wuji v2")
        if selected_rate != 50.0:
            raise ValueError("realtime 模式必须使用 50 Hz 真机发布频率")
    settings = _side_settings(args)
    for side in settings:
        if not 1 <= side.target_port <= 65535:
            raise ValueError(f"{side.hand} ZMQ 目标端口必须在 1..65535")
    safety = HardwareSafetyOptions(
        max_velocity=args.max_velocity,
        max_acceleration=args.max_acceleration,
        input_filter_hz=selected_filter,
        home_rate_hz=args.home_rate,
        home_tolerance=selected_home_tolerance,
        home_timeout=args.home_timeout,
        command_timeout=args.command_timeout,
        watchdog_period=args.watchdog_period,
        tracking_mode=args.tracking_mode,
        realtime_max_velocity=args.realtime_max_velocity,
    )

    # 双手必须先一次性通过两个仿真门禁，之后才允许连接第一只真机。
    for side in settings:
        _simulation_check(args.generation, side.hand)

    with _interrupt_on_shutdown_signals():
        return _run_publishers(args, settings, safety)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "仿真手套目标到 Wuji 真机的独立控制入口；"
            "要求先在另一终端启动同代同侧 MuJoCo"
        )
    )
    parser.add_argument(
        "--generation",
        choices=("v1", "v2"),
        required=True,
    )
    parser.add_argument(
        "--hand",
        choices=("left", "right", "both"),
        required=True,
    )
    parser.add_argument("--hand-sn", default="")
    parser.add_argument(
        "--glove-host",
        default="",
        help="兼容旧命令；手套连接现由 MuJoCo 终端唯一持有",
    )
    parser.add_argument(
        "--glove-port",
        type=int,
        default=DEFAULT_PORT,
        help="兼容旧命令；真机默认复用仿真 ZMQ 目标",
    )
    parser.add_argument("--target-host", default="127.0.0.1")
    parser.add_argument("--target-port", type=int, default=0)
    parser.add_argument("--cal-file", default="")
    parser.add_argument("--left-hand-sn", default="")
    parser.add_argument("--right-hand-sn", default="")
    parser.add_argument(
        "--left-glove-host",
        default="",
        help="兼容旧命令；左手套连接由 MuJoCo 终端持有",
    )
    parser.add_argument(
        "--right-glove-host",
        default="",
        help="兼容旧命令；右手套连接由 MuJoCo 终端持有",
    )
    parser.add_argument("--left-glove-port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--right-glove-port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--left-target-host", default="127.0.0.1")
    parser.add_argument("--right-target-host", default="127.0.0.1")
    parser.add_argument("--left-target-port", type=int, default=0)
    parser.add_argument("--right-target-port", type=int, default=0)
    parser.add_argument("--left-cal-file", default="")
    parser.add_argument("--right-cal-file", default="")
    parser.add_argument(
        "--rate",
        type=float,
        default=0.0,
        help="真机发布频率（0=按代际默认：v1 20 Hz、v2 50 Hz）",
    )
    parser.add_argument(
        "--tracking-mode",
        choices=("legacy", "realtime"),
        default="legacy",
        help="目标跟踪模式（默认 legacy；realtime 为固定 50 Hz 低延迟模式）",
    )
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--max-velocity", type=float, default=1.5)
    parser.add_argument(
        "--realtime-max-velocity",
        type=float,
        default=4.0,
        help="realtime 遥操作速度上限 rad/s；不影响回零速度",
    )
    parser.add_argument("--max-acceleration", type=float, default=8.0)
    parser.add_argument(
        "--input-filter-hz",
        type=float,
        default=0.0,
        help="输入关节角低通截止频率（0=按代际默认，当前均为 5 Hz）",
    )
    parser.add_argument("--home-rate", type=float, default=100.0)
    parser.add_argument(
        "--home-tolerance",
        type=float,
        default=0.0,
        help=(
            "回零允许误差（rad；0=按代际默认："
            "v1 0.01、v2 0.05）"
        ),
    )
    parser.add_argument("--home-timeout", type=float, default=8.0)
    parser.add_argument("--command-timeout", type=float, default=0.5)
    parser.add_argument("--watchdog-period", type=float, default=0.05)
    parser.add_argument("--kp", type=float, default=DEFAULT_V2_KP)
    parser.add_argument("--kd", type=float, default=DEFAULT_V2_KD)
    parser.add_argument(
        "--confirm-real",
        action="store_true",
        help="确认急停可用、手周围无障碍，并允许连接真机",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        return command_real(args)
    except KeyboardInterrupt:
        print("\n收到中断，真机已进入回零关闭流程。", file=sys.stderr)
        return 130
    except (
        EOFError,
        OSError,
        RuntimeError,
        ValueError,
    ) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
