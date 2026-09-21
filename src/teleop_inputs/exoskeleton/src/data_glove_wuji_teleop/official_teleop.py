"""按具名手套档案启动一侧或双侧官方仿真，并显式选择真机跟随。"""

from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import time
from contextlib import ExitStack, contextmanager
from dataclasses import dataclass, replace

from .application.glove_discovery import resolve_glove_profiles
from .adapters.hardware.discovery import resolve_robot_serials
from .adapters.runtime.simulation_session import require_active_simulation
from .adapters.transport.zmq_dof import DEFAULT_ZMQ_PORT, ZmqTargetSubscriber
from .domain.hand_target import DOF_ORDER
from .profiles.dataglove.device import GloveDeviceProfile
from .profiles.wuji_v2.retargeting import official_config_path, validate_retargeting_resources
from .profiles.teleop_task import TeleopTask, validate_glove_set
from .project import PROJECT_ROOT


@dataclass(frozen=True)
class SelectedGlove:
    profile: GloveDeviceProfile
    serial: str


def select_gloves(args: argparse.Namespace) -> tuple[SelectedGlove, ...]:
    """离线解析任务或显式设备选择；缺省机器人SN留待按真实手侧发现。"""
    task_name = getattr(args, "task", None)
    names = getattr(args, "glove_profiles", ())
    explicit = {hand: name for hand in ("left", "right") if (name := getattr(args, f"{hand}_profile"))}
    if names and explicit:
        raise ValueError("--glove-profile 与 --left-profile/--right-profile 不能混用")
    if task_name and (names or explicit):
        raise ValueError("--task 与显式手套档案不能混用")
    for name in names:
        profile = GloveDeviceProfile.load(name)
        if profile.hand in explicit:
            raise ValueError(f"不能同时选择两只 {profile.hand} 手套")
        explicit[profile.hand] = profile
    if not task_name and not explicit:
        task_name = "default"
    task = TeleopTask.load(task_name) if task_name else None
    requested_hand = getattr(args, "hand", None)
    profiles = {profile.hand: profile for profile in task.profiles(requested_hand)} if task else explicit
    if task is None and requested_hand is not None:
        requested = ("left", "right") if requested_hand == "both" else (requested_hand,)
        if any(hand not in profiles for hand in requested):
            raise ValueError(f"未绑定请求的 {requested_hand} 手套")
        profiles = {hand: profiles[hand] for hand in requested}
    if task is not None:
        for argument, key in (("robot_network", "connection"), ("robot_interface", "interface")):
            if getattr(args, argument, None) is None:
                setattr(args, argument, task.robot_network.get(key))
    selected = []
    for hand in ("left", "right"):
        name = profiles.get(hand)
        serial = getattr(args, f"{hand}_hand_sn")
        if not name:
            if serial:
                raise ValueError(f"提供了 {hand} SN，但未选择该侧手套")
            continue
        profile = name if isinstance(name, GloveDeviceProfile) else GloveDeviceProfile.load(name)
        if task is not None and not serial:
            serial = task.robot_serials.get(hand, "")
        profile.validate_resources()
        if profile.hand != hand:
            raise ValueError(f"{profile.name} 是 {profile.hand} 手套，不能作为 {hand}")
        validate_retargeting_resources(hand)
        mapping = profile.load_mapping()
        mapping.require_all_joints_enabled()
        if not args.commission_directions:
            mapping.require_verified_directions()
        selected.append(SelectedGlove(profile, serial))
    validate_glove_set(side.profile for side in selected)
    serials = [side.serial for side in selected if side.serial]
    if len(serials) != len(set(serials)):
        raise ValueError("左右机器人必须使用不同的 SN")
    if not math.isfinite(args.ready_timeout) or args.ready_timeout <= 0:
        raise ValueError("--ready-timeout 必须是正有限秒数")
    return tuple(selected)


def simulation_command(side: SelectedGlove, args: argparse.Namespace) -> list[str]:
    command = [
        sys.executable, "-u", "-m", "data_glove_wuji_teleop.adapters.simulation.mujoco_wuji_official",
        "--hand", side.profile.hand, "--glove-profile", str(side.profile.path),
        "--skip-network-setup", "--apply-filter",
    ]
    if args.commission_directions:
        command.append("--commission-directions")
    if args.confirm_real:
        command.append("--publish-targets")
    return command


def hardware_command(selected: tuple[SelectedGlove, ...]) -> list[str]:
    command = [
        sys.executable, "-u", "-m", "data_glove_wuji_teleop.hardware_cli",
        "--generation", "v2", "--tracking-mode", "realtime", "--rate", "50",
        "--kp", "8.0", "--kd", "0.2", "--realtime-max-velocity", "3.0",
        "--home-tolerance", "0.10", "--home-timeout", "15", "--confirm-real",
    ]
    if len(selected) == 1:
        command.extend(("--hand", selected[0].profile.hand, "--hand-sn", selected[0].serial))
    else:
        command.extend(("--hand", "both"))
        for side in selected:
            command.extend((f"--{side.profile.hand}-hand-sn", side.serial))
    return command




def _stop_process(process: subprocess.Popen, timeout: float) -> None:
    """有界等待主进程回零，并清除已退出主进程留下的同组 worker。"""
    def send(sig):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass

    for sig, wait_seconds in ((signal.SIGINT, timeout), (signal.SIGTERM, 3.0), (signal.SIGKILL, 3.0)):
        send(sig)
        try:
            process.wait(timeout=wait_seconds)
        except subprocess.TimeoutExpired:
            continue
        # wait 只回收组长；组长已退出也不能遗漏忽略 SIGTERM 的 worker。
        send(signal.SIGTERM)
        time.sleep(0.1)
        send(signal.SIGKILL)
        return
    raise RuntimeError(f"进程组 {process.pid} 未能退出，请使用物理急停并检查设备")


@contextmanager
def _shutdown_signals():
    previous = {}
    def interrupt(_signum, _frame):
        raise KeyboardInterrupt
    try:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            previous[sig] = signal.signal(sig, interrupt)
        yield
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def _wait_ready(selected, simulations, receivers, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    latest = {}
    while time.monotonic() < deadline:
        for side in selected:
            hand = side.profile.hand
            process = simulations[hand]
            if process.poll() is not None:
                raise RuntimeError(f"{hand} 官方仿真提前退出；不会启动真机控制")
            try:
                session = require_active_simulation(generation="v2", hand=hand)
            except RuntimeError:
                latest.pop(hand, None)
                continue
            if os.getpgid(session.pid) != process.pid:
                raise RuntimeError(f"{hand} 仿真租约不属于本次进程组")
            if hand in latest and latest[hand][0] != session.token:
                latest.pop(hand)
            target = receivers[hand].recv()
            if target is None:
                continue
            if target.source != "official_wuji_retarget" or target.hand != hand:
                raise RuntimeError(f"{hand} 未收到正常控制首帧：{target.source}；不会使能真机")
            if len(target.values) != len(DOF_ORDER) or not all(math.isfinite(v) for v in target.values):
                raise RuntimeError(f"{hand} 控制目标必须包含20个有限弧度值")
            if target.timestamp_ns >= session.started_ns:
                latest[hand] = (session.token, target.timestamp_ns)
        now = time.time_ns()
        if all(
            simulations[side.profile.hand].poll() is None
            and 0 <= now - latest.get(side.profile.hand, ("", 0))[1] <= 500_000_000
            for side in selected
        ):
            return
        time.sleep(0.01)
    raise TimeoutError("未能在限时内同时取得所选各侧的仿真租约和新鲜控制首帧；不会使能真机")


def run(args: argparse.Namespace) -> int:
    selected = select_gloves(args)
    for side in selected:
        profile = side.profile
        print(f"[{profile.hand}] 手套={profile.name}；档案={profile.path}\n"
              f"  内嵌零位组={profile.active_zero}\n  公共官方 YAML={official_config_path(profile.hand)}\n"
              f"  档案网卡={profile.network.get('interface') or '未指定'}；"
              f"主机={profile.network['host_address']}；路由表={profile.network['route_table']}", flush=True)
    if args.check_config:
        print("任务、设备身份绑定与资源检查通过；未联网，机器人自动识别和网络将在启动时验证。")
        return 0
    print("[模式] 真机控制：将连接、使能并回零。" if args.confirm_real else
          "[模式] 仅仿真：不扫描机器人、不发布目标、不创建真机租约。", flush=True)
    print("[传输] 按实物ID定位当前手套网卡；真机按SN/手侧自动识别并校验直连网络。", flush=True)
    for side in selected:
        try:
            require_active_simulation(generation="v2", hand=side.profile.hand)
        except RuntimeError:
            pass
        else:
            raise RuntimeError(f"已有 v2/{side.profile.hand} 仿真租约，请先关闭旧进程")
    simulations = {}
    real = None
    with _shutdown_signals(), ExitStack() as stack:
        try:
            receivers = {}
            resolved = resolve_glove_profiles(
                tuple(side.profile for side in selected), timeout=args.ready_timeout,
                prepare_network=not getattr(args, "skip_network_setup", False),
            )
            selected = tuple(replace(side, profile=profile) for side, profile in zip(selected, resolved))
            for side in selected:
                print(f"[{side.profile.hand}] 实物ID={side.profile.device_id}；"
                      f"当前网卡={side.profile.network['interface']}；"
                      f"源地址={side.profile.network['host_address']}", flush=True)
            if args.confirm_real:
                for side in selected:
                    hand = side.profile.hand
                    receiver = ZmqTargetSubscriber("127.0.0.1", DEFAULT_ZMQ_PORT[hand], hand, timeout_seconds=0.05)
                    receivers[hand] = receiver
                    stack.callback(receiver.close)
            for side in selected:
                simulations[side.profile.hand] = subprocess.Popen(
                    simulation_command(side, args), cwd=PROJECT_ROOT, start_new_session=True,
                )
            if args.confirm_real:
                _wait_ready(selected, simulations, receivers, args.ready_timeout)
                serials = resolve_robot_serials(
                    {side.profile.hand: side.serial or None for side in selected},
                    network_connection=getattr(args, "robot_network", None),
                    network_interface=getattr(args, "robot_interface", None),
                    excluded_interfaces=tuple(
                        side.profile.network["interface"] for side in selected
                        if side.profile.network.get("interface")
                    ),
                    prepare_network=not getattr(args, "skip_network_setup", False),
                )
                selected = tuple(replace(side, serial=serials[side.profile.hand]) for side in selected)
                for side in selected:
                    print(f"[{side.profile.hand}] 真机身份确认：SN={side.serial}", flush=True)
                # SDK扫描耗时期间任一仿真可能退出；再次同时检查两侧的新鲜首帧。
                _wait_ready(selected, simulations, receivers, args.ready_timeout)
                print("所有设备与目标均已通过预检，启动真机控制。", flush=True)
                real = subprocess.Popen(hardware_command(selected), cwd=PROJECT_ROOT, start_new_session=True)
            while True:
                if real is not None and real.poll() is not None:
                    return real.returncode
                for hand, process in simulations.items():
                    if process.poll() is not None:
                        if real is not None:
                            raise RuntimeError(f"{hand} 仿真退出，停止双手控制并进入回零清理")
                        return process.returncode
                time.sleep(0.05)
        finally:
            # 必须保持仿真租约/目标源，直至真机的回零清理完成。
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                signal.signal(sig, signal.SIG_IGN)
            cleanup_errors = []
            processes = ([(real, 20.0)] if real is not None else []) + [
                (process, 5.0) for process in simulations.values()
            ]
            for process, timeout in processes:
                try:
                    _stop_process(process, timeout=timeout)
                except Exception as exc:
                    cleanup_errors.append(f"进程组 {process.pid}: {exc}")
            if cleanup_errors:
                raise RuntimeError("清理失败；已尝试停止所有进程：" + "；".join(cleanup_errors))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="具名手套的单/双手 Hand2 遥操作；默认仅仿真")
    parser.add_argument("--task", help="可选实验组合；未指定设备时使用 default 绑定")
    parser.add_argument("--glove-profile", dest="glove_profiles", action="append", default=[],
                        help="直接选择设备档案，可重复；依据档案hand自动分配左右手")
    parser.add_argument("--hand", choices=("left", "right", "both"), help="可选手侧筛选；默认运行所有已绑定侧")
    parser.add_argument("--robot-network", help="机器人NetworkManager连接名；默认自动检查/准备有线直连")
    parser.add_argument("--robot-interface", help="固定机器人有线网卡；不允许选用手套网卡")
    parser.add_argument("--skip-network-setup", action="store_true", help="仅验证既有网络，不自动修改网络")
    for hand in ("left", "right"):
        parser.add_argument(f"--{hand}-profile", help="显式设备档案名或JSON路径；与--task互斥")
        parser.add_argument(f"--{hand}-hand-sn", default="", help="固定机器人SN；省略时按真实手侧唯一发现")
    parser.add_argument("--commission-directions", action="store_true", help="显式允许未验收方向配置，仅用于逐指调试")
    parser.add_argument("--confirm-real", action="store_true", help="确认现场安全，允许真机连接、使能、回零及跟随")
    parser.add_argument("--check-config", action="store_true", help="仅离线检查所选档案，不连接任何设备")
    parser.add_argument("--ready-timeout", type=float, default=30.0)
    return parser


def main() -> int:
    try:
        return run(build_parser().parse_args())
    except KeyboardInterrupt:
        print("已请求停止；正常退出包含回零运动，紧急情况请使用物理急停。", file=sys.stderr)
        return 130
    except (EOFError, OSError, RuntimeError, ValueError, subprocess.SubprocessError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
