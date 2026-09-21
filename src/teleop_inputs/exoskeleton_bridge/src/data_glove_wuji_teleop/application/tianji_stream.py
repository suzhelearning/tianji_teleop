"""具名外骨骼手套到本机 TJH2 v2 控制器；只发送官方模型 qpos，不控制机器人。"""

from __future__ import annotations

import argparse
import ipaddress
import json
import math
import signal
import sys
import threading
import time
from contextlib import ExitStack, contextmanager
from dataclasses import dataclass
from pathlib import Path

import mujoco

from ..adapters.glove.network import connect_glove
from ..adapters.retargeting.glove_pipeline import GloveRetargetPipeline
from ..adapters.retargeting.wuji_process import launch_official_adapter
from ..adapters.transport.tianji_udp import TianjiHandSender, TianjiJointBinding
from ..official_teleop import select_gloves
from ..profiles.dataglove.device import GloveDeviceProfile, apply_glove_profile
from ..profiles.wuji_v2.retargeting import DEFAULT_MANO_MORPHOLOGY, validate_retargeting_resources
from ..project import PROJECT_ROOT, get_hand_resources
from .glove_discovery import resolve_glove_profiles


@dataclass(frozen=True)
class _Measurement:
    values: tuple[float, ...]
    sampled_at_ns: int
    source_sequence: int
    source_timestamp_ns: int


@dataclass
class _HandState:
    """容量一的最新结果邮箱；所有读写均由共享 Condition 保护。"""

    hand: str
    pending: _Measurement | None = None
    failure: str | None = None
    solved: int = 0
    sent: int = 0
    stale: int = 0
    duplicate: int = 0

    def fail(self, error: Exception) -> None:
        self.failure = str(error) or type(error).__name__
        self.pending = None

    def take(self, now_ns: int, max_age: float) -> _Measurement | None:
        sample, self.pending = self.pending, None
        if self.failure is not None or sample is None:
            return None
        if not 0 <= now_ns - sample.sampled_at_ns <= max_age * 1_000_000_000:
            self.stale += 1
            return None
        return sample


@dataclass
class _HandConfig:
    profile: GloveDeviceProfile
    args: argparse.Namespace
    pipeline: GloveRetargetPipeline
    model: mujoco.MjModel


def _prepare_hand(profile: GloveDeviceProfile, args: argparse.Namespace) -> _HandConfig:
    """仅加载本地资源；完整零位、方向与模型检查发生在联网之前。"""
    options = argparse.Namespace(**vars(args))
    options.hand = profile.hand
    options.glove_profile = str(profile.path)
    apply_glove_profile(options)
    options.wuji_config = validate_retargeting_resources(
        profile.hand, mano_morphology=args.mano_morphology, wuji_config=args.wuji_config,
    )
    for label, path in (("worker Python", args.worker_python), ("官方 checkout", args.wuji_checkout)):
        if not Path(path).exists():
            raise ValueError(f"{label} 不存在：{path}")
    if args.wuji_urdf is not None and not args.wuji_urdf.is_file():
        raise ValueError(f"官方机器人 URDF 不存在：{args.wuji_urdf}")
    pipeline = GloveRetargetPipeline.from_config(
        glove_urdf=profile.glove_urdf,
        mapping_data=profile.mapping_data,
        zero_profile=profile.load_zero(),
        morphology_file=args.mano_morphology,
        commissioning=args.commission_directions,
    )
    if pipeline.zero_profile is None or not pipeline.zero_profile.complete:
        raise ValueError(f"{profile.hand} 必须使用完整内嵌软件零位")
    resources = get_hand_resources(profile.hand, generation="v2")
    if resources.mjcf is None:
        raise ValueError(f"Wuji Hand2 {profile.hand} 缺少 MJCF")
    model = mujoco.MjModel.from_xml_path(str(resources.mjcf))
    names = tuple(model.joint(index).name for index in range(model.njnt))
    TianjiJointBinding.from_model(profile.hand, names, model)
    return _HandConfig(profile, options, pipeline, model)


def _produce(
    config: _HandConfig,
    state: _HandState,
    condition: threading.Condition,
    stop: threading.Event,
) -> None:
    """每侧独立采集/求解，绝不把缓存重新包装成新测量。"""
    args = config.args
    binding = None
    last_sequence = last_timestamp = None
    frame = None
    stage = "connect"
    # 失败先清邮箱，再关闭资源；关闭握手可能耗时，不能延后失效通知。
    with ExitStack() as resources:
        try:
            retargeter = launch_official_adapter(args, response_timeout=args.timeout)
            resources.callback(retargeter.close)
            if stop.is_set():
                return
            connection = connect_glove(args)
            resources.callback(connection.close)
            config.pipeline.validate_stream(connection)
            while not stop.is_set():
                frame = None
                stage = "read"
                frame = connection.read_latest_frame()
                # 板端时间只用于去重；控制器年龄使用同机读取完成时刻，包含后续 FK/求解耗时。
                sampled_at_ns = time.monotonic_ns()
                if stop.is_set():
                    break
                if ((last_sequence is not None and frame.sequence <= last_sequence)
                        or (last_timestamp is not None and frame.timestamp_ns <= last_timestamp)):
                    with condition:
                        state.duplicate += 1
                    continue
                last_sequence, last_timestamp = frame.sequence, frame.timestamp_ns
                stage = "glove_kinematics"
                points = config.pipeline.process_frame(frame, zero_profile=config.pipeline.zero_profile)
                stage = "retarget"
                result = retargeter.retarget(points, apply_filter=args.apply_filter)
                stage = "joint_binding"
                if binding is None:
                    binding = TianjiJointBinding.from_model(args.hand, result.joint_names, config.model)
                elif tuple(result.joint_names) != binding.source_joint_names:
                    raise ValueError("官方 worker 的关节顺序在流中发生变化；必须重启并重新绑定")
                values = binding.convert(result.qpos)
                with condition:
                    state.solved += 1
                    if time.monotonic_ns() - sampled_at_ns > args.max_frame_age * 1_000_000_000:
                        state.stale += 1
                    elif not stop.is_set() and state.failure is None:
                        state.pending = _Measurement(values, sampled_at_ns, frame.sequence, frame.timestamp_ns)
                        condition.notify_all()
        except Exception as exc:
            if not stop.is_set():
                with condition:
                    state.fail(exc)
                    condition.notify_all()
                if frame is not None:
                    diagnostic = {
                        "hand": args.hand,
                        "stage": stage,
                        "sequence": frame.sequence,
                        "timestamp_ns": frame.timestamp_ns,
                        "dropped": frame.dropped,
                        "angles_deg": {
                            f"J{index + 1}": angle
                            for index, angle in enumerate(frame.angles_deg)
                        },
                        "error": str(exc),
                    }
                    sys.stderr.write(
                        f"[{args.hand}] 失败帧: {json.dumps(diagnostic, ensure_ascii=False)}\n")
                    sys.stderr.flush()
        finally:
            # ExitStack 会继续尝试全部 close；将清理错误也计入侧失败。
            try:
                resources.close()
            except Exception as exc:
                with condition:
                    state.fail(exc)
                    condition.notify_all()


@contextmanager
def _shutdown_signals(stop: threading.Event):
    previous = {}

    def request_stop(_signum, _frame):
        stop.set()

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, request_stop)
        yield
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)


def _run_stream(configs: list[_HandConfig], args: argparse.Namespace) -> int:
    condition = threading.Condition()
    stop = threading.Event()
    states = [_HandState(config.args.hand) for config in configs]
    threads = []
    sender = None
    started = time.monotonic()
    reported = set()
    try:
        with _shutdown_signals(stop):
            sender = TianjiHandSender(args.udp_host, args.udp_port)
            for config, state in zip(configs, states):
                thread = threading.Thread(
                    target=_produce, args=(config, state, condition, stop),
                    name=f"tianji-{state.hand}", daemon=True,
                )
                thread.start()
                threads.append(thread)
            while not stop.is_set():
                with condition:
                    now = time.monotonic()
                    if args.seconds and now - started >= args.seconds:
                        break
                    for state in states:
                        if state.failure is not None and state.hand not in reported:
                            sender.invalidate(state.hand)
                            print(f"[{state.hand}] 已停止：{state.failure}；该侧必须重启才恢复", file=sys.stderr, flush=True)
                            reported.add(state.hand)
                    if all(state.failure is not None for state in states):
                        break
                    samples = {state.hand: state.take(time.monotonic_ns(), args.max_frame_age) for state in states}
                    if any(sample is not None for sample in samples.values()):
                        updates = {}
                        for hand, sample in samples.items():
                            if sample is not None:
                                updates[hand] = sample.values
                                updates[f"{hand}_timestamp_ns"] = sample.sampled_at_ns
                        # 与失效/清邮箱互斥；失败缓存已清除，不会泄漏到健康侧的新包。
                        sender.send(**updates)
                        for state in states:
                            if samples[state.hand] is not None:
                                state.sent += 1
                    else:
                        condition.wait(timeout=0.01)
    finally:
        stop.set()
        elapsed = max(time.monotonic() - started, 1e-9)
        with condition:
            for state in states:
                state.pending = None
            condition.notify_all()
        # worker 就绪最多30秒；元数据/连接握手、读取和关闭均留出有限超时预算。
        deadline = time.monotonic() + 35.0 + 4 * args.timeout
        for thread in threads:
            thread.join(timeout=max(0.0, deadline - time.monotonic()))
        if sender is not None:
            sender.close()
        for state in states:
            print(f"[{state.hand}] 有效求解={state.solved}，发送新帧={state.sent}，"
                  f"实际求解均频={state.solved / elapsed:.2f} Hz，"
                  f"实际发送均频={state.sent / elapsed:.2f} Hz，"
                  f"过期丢弃={state.stale}，重复/倒退丢弃={state.duplicate}；退出不补零", flush=True)
        if any(thread.is_alive() for thread in threads):
            raise RuntimeError("手套线程未在关闭期限内退出；已停止UDP，请检查仍占用的设备/worker")
    if not any(state.sent for state in states):
        print("本次运行未发送任何新有效结果，返回失败；不会以旧值或零值补包。", file=sys.stderr)
        return 1
    return 1 if not states or all(state.failure is not None for state in states) else 0


def _validate_options(args: argparse.Namespace) -> None:
    for name, maximum in (("timeout", 60.0), ("max_frame_age", 10.0)):
        value = getattr(args, name)
        if not math.isfinite(value) or not 0 < value <= maximum:
            raise ValueError(f"--{name.replace('_', '-')} 必须在 (0, {maximum:g}] 秒")
    if not ipaddress.IPv4Address(args.udp_host).is_loopback:
        raise ValueError("--udp-host 必须是本机回环 IPv4 地址，源时间戳要求同机单调时钟")
    if not math.isfinite(args.seconds) or args.seconds < 0:
        raise ValueError("--seconds 必须是非负有限秒数，0表示持续运行")
    if not 1 <= args.udp_port <= 65535:
        raise ValueError("--udp-port 必须在 1..65535")
    if not math.isfinite(args.pinch_alpha_max) or not 0 <= args.pinch_alpha_max <= 1:
        raise ValueError("--pinch-alpha-max 必须在 [0, 1]")
    if args.pinch_d1_cm is not None and (not math.isfinite(args.pinch_d1_cm) or args.pinch_d1_cm <= 0):
        raise ValueError("--pinch-d1-cm 必须是正有限数值")


def run(args: argparse.Namespace) -> int:
    if not args.check_config and not args.confirm_send:
        print("未发送：必须显式指定 --confirm-send；当前未打开设备或网络。")
        return 2
    _validate_options(args)
    # select_gloves 只做离线选择；不暴露也不调用它的机器人发现/控制入口。
    args.left_hand_sn = args.right_hand_sn = ""
    args.robot_network = args.robot_interface = None
    args.ready_timeout = args.timeout
    selected = select_gloves(args)
    configs = [_prepare_hand(side.profile, args) for side in selected]
    if args.check_config:
        for config in configs:
            direction = "方向调试（未验收）" if args.commission_directions else "方向验收门禁"
            print(f"[{config.args.hand}] 身份档案、完整软件零位、{direction}、FK/MANO及Wuji模型检查通过")
        print("离线检查结束：未发现设备、未配置网络、未启动worker、未创建UDP。")
        return 0
    print("仅发送官方 Wuji 模型 qpos；不接机器人SDK，不使能、不发ZMQ、不创建仿真租约。", flush=True)
    if args.commission_directions:
        print("方向调试：已显式允许尚未验收的方向；不能据此宣称硬件或真机动作已验证。", flush=True)
    # 解析会检查严格实物ID且可能重配网卡；串行执行，某侧失败不替换设备。
    ready = []
    for config in configs:
        try:
            resolved, = resolve_glove_profiles(
                (config.profile,), timeout=args.timeout, prepare_network=not args.skip_network_setup,
            )
            config.profile = resolved
            apply_glove_profile(config.args)
            ready.append(config)
        except Exception as exc:
            print(f"[{config.args.hand}] 启动失败：{exc}；该侧必须重启才恢复", file=sys.stderr, flush=True)
    if not ready:
        return 1
    return _run_stream(ready, args)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="外骨骼手套 → 官方求解器模型qpos → 本机 TJH2 v2 UDP（无真机控制）")
    parser.add_argument("--task", help="具名任务；未指定档案时使用default绑定")
    parser.add_argument("--glove-profile", dest="glove_profiles", action="append", default=[])
    parser.add_argument("--left-profile")
    parser.add_argument("--right-profile")
    parser.add_argument("--hand", choices=("left", "right", "both"))
    parser.add_argument("--udp-host", default="127.0.0.1", help="本机回环 IPv4 地址；源时间戳要求与控制器同机")
    parser.add_argument("--udp-port", type=int, default=16000)
    parser.add_argument("--confirm-send", action="store_true", help="显式授权采集及UDP发送，不授权机器人动作")
    parser.add_argument("--check-config", action="store_true", help="全程离线检查，优先于发送确认")
    parser.add_argument("--seconds", type=float, default=0.0, help="有限运行秒数；0持续运行")
    parser.add_argument("--timeout", type=float, default=2.0, help="设备与逐帧worker响应超时，(0,60]秒")
    parser.add_argument("--max-frame-age", type=float, default=0.2, help="从本机读取原始帧完成起的最大年龄，(0,10]秒")
    parser.add_argument("--skip-network-setup", action="store_true")
    parser.add_argument("--commission-directions", action="store_true")
    parser.add_argument("--worker-python", type=Path, default=Path(sys.executable))
    parser.add_argument("--wuji-checkout", type=Path, default=PROJECT_ROOT / "vendor/wuji-retargeting")
    parser.add_argument("--wuji-config", type=Path)
    parser.add_argument("--wuji-urdf", type=Path)
    parser.add_argument("--mano-morphology", type=Path, default=DEFAULT_MANO_MORPHOLOGY)
    parser.add_argument("--apply-filter", action="store_true")
    parser.add_argument("--pinch-d1-cm", type=float)
    parser.add_argument("--pinch-alpha-max", type=float, default=1.0)
    return parser


def main() -> int:
    try:
        return run(build_parser().parse_args())
    except KeyboardInterrupt:
        print("已停止；不发送退出零值。", file=sys.stderr)
        return 130
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
