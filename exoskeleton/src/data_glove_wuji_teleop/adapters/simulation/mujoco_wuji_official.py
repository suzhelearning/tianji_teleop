"""官方命名关节目标驱动 Wuji Hand 2 MuJoCo 的验证入口。"""

from __future__ import annotations

import argparse
import os
import sys
import time
from contextlib import contextmanager, nullcontext
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Sequence

import mujoco
import mujoco.viewer
import numpy as np

from ...project import PROJECT_ROOT, get_hand_resources
from ...domain.hand_target import DOF_ORDER, HandTarget
from ...profiles.joint_mapping import load_joint_mapping
from ...profiles.dataglove.device import apply_glove_profile
from ...profiles.teleop_task import apply_task_glove
from ...profiles.wuji_v2.model import full_joint_name
from ...profiles.wuji_v2.retargeting import (
    DEFAULT_MANO_MORPHOLOGY,
    official_config_path,
)
from ..glove.network import connect_glove, prepare_glove_network
from ..retargeting.glove_pipeline import GloveRetargetPipeline
from ..retargeting.wuji_official import (
    compute_tip_fit_metrics,
)
from ..retargeting.wuji_process import launch_official_adapter
from ..runtime.simulation_session import SimulationLease
from ..transport.zmq_dof import DEFAULT_ZMQ_PORT, ZmqDofPublisher
from .mujoco_dataglove import DEFAULT_URDF
from .mujoco_overlay import append_bone, append_sphere
from .mujoco_retarget_composite import load_retarget_composite_model


_TARGET_COLOR = (1.0, 0.80, 0.05, 1.0)
_MEDIAPIPE_CHAINS = tuple(
    (0, 1 + 4 * finger, 2 + 4 * finger, 3 + 4 * finger, 4 + 4 * finger)
    for finger in range(5)
)
_DEFAULT_GLOVE_CONFIG = PROJECT_ROOT / "config/dataglove/urdf/right.json"
_DEFAULT_ZERO_FILE = PROJECT_ROOT / "config/dataglove/urdf/zero/right.json"
_DEFAULT_WORKER_PYTHON = Path(sys.executable)
_DEFAULT_WUJI_CHECKOUT = PROJECT_ROOT / "vendor/wuji-retargeting"
_FINGER_NAMES = ("thumb", "index", "middle", "ring", "pinky")
_SHUTDOWN_ZERO_BURST = 3
_SHUTDOWN_ZERO_INTERVAL_SECONDS = 0.02
class _ProfileBoundArgument(argparse.Action):
    """记录显式资源选择，区分命令行覆盖与现有单手默认值。"""

    def __call__(self, parser, namespace, values, option_string=None):
        setattr(namespace, self.dest, values)
        selected = set(getattr(namespace, "_profile_overrides", ()))
        selected.add(option_string)
        namespace._profile_overrides = selected




@dataclass(frozen=True)
class NamedQposBinding:
    """官方输出顺序到 MuJoCo qpos 地址的双射。"""

    joint_names: tuple[str, ...]
    qpos_addresses: tuple[int, ...]


@dataclass(frozen=True)
class OfficialTargetBinding:
    """官方 qpos 顺序到 canonical 语义 DOF 的逆映射。"""

    source_joint_names: tuple[str, ...]
    hand: str
    qpos_indices: tuple[int, ...]
    scales: tuple[float, ...]
    offsets: tuple[float, ...]


def build_official_target_binding(
    joint_names: Sequence[str],
    *,
    mapping_path: str | Path,
    hand: str = "right",
) -> OfficialTargetBinding:
    """按指定侧的二代映射绑定官方关节，并准备模型映射的逆变换。"""

    names = tuple(str(name) for name in joint_names)
    if len(names) != len(DOF_ORDER) or len(set(names)) != len(names):
        raise ValueError("官方目标必须包含 20 个唯一关节名")
    index_by_name = {name: index for index, name in enumerate(names)}
    rules = load_joint_mapping(
        mapping_path,
        expected_hand=hand,
        expected_generation="v2",
    )
    by_dof = {rule.dof_name: rule for rule in rules}
    indices: list[int] = []
    scales: list[float] = []
    offsets: list[float] = []
    for dof_name in DOF_ORDER:
        rule = by_dof[dof_name]
        official_name = full_joint_name(hand, rule.joint_name)
        if official_name not in index_by_name:
            raise ValueError(f"官方 qpos 缺少关节：{official_name}")
        if rule.scale == 0.0:
            raise ValueError(f"{dof_name} 的模型映射 scale 不能为 0")
        indices.append(index_by_name[official_name])
        scales.append(rule.scale)
        offsets.append(rule.offset_rad)
    return OfficialTargetBinding(
        source_joint_names=names,
        hand=hand,
        qpos_indices=tuple(indices),
        scales=tuple(scales),
        offsets=tuple(offsets),
    )


def make_official_hand_target(
    binding: OfficialTargetBinding,
    qpos: np.ndarray,
    *,
    sequence: int,
    timestamp_ns: int,
    dropped: int = 0,
) -> HandTarget:
    """把官方模型关节角逆映射为供仿真和真机共享的 hand_dof。"""

    values = np.asarray(qpos, dtype=np.float64)
    if (
        values.shape != (len(binding.source_joint_names),)
        or not np.isfinite(values).all()
    ):
        raise ValueError("官方 qpos 数量或数值无效")
    semantic_values = tuple(
        float((values[index] - offset) / scale)
        for index, scale, offset in zip(
            binding.qpos_indices,
            binding.scales,
            binding.offsets,
        )
    )
    return HandTarget(
        hand=binding.hand,
        values=semantic_values,
        source="official_wuji_retarget",
        sequence=int(sequence),
        timestamp_ns=int(timestamp_ns),
        dropped=int(dropped),
    )


@contextmanager
def open_official_target_publisher(
    bind_host: str,
    port: int,
    *,
    hand: str = "right",
) -> Iterator[ZmqDofPublisher]:
    """发布官方目标，并在退出时先发送零目标再关闭端口。"""

    if hand not in ("left", "right"):
        raise ValueError("目标发布侧别必须为 left/right")
    selected_port = int(port)
    if not str(bind_host):
        raise ValueError("ZMQ bind host 不能为空")
    if not 1 <= selected_port <= 65535:
        raise ValueError("ZMQ port 必须在 1..65535")
    publisher = ZmqDofPublisher(str(bind_host), selected_port)
    try:
        yield publisher
    finally:
        try:
            for sequence in range(_SHUTDOWN_ZERO_BURST):
                publisher.publish(
                    HandTarget(
                        hand=hand,
                        values=(0.0,) * len(DOF_ORDER),
                        source="official_wuji_retarget_shutdown",
                        sequence=sequence,
                        timestamp_ns=time.time_ns(),
                        dropped=0,
                    )
                )
                time.sleep(_SHUTDOWN_ZERO_INTERVAL_SECONDS)
        finally:
            publisher.close()


def build_named_qpos_binding(
    model: mujoco.MjModel,
    joint_names: Sequence[str],
    *,
    model_prefix: str = "",
    require_complete_model: bool = True,
) -> NamedQposBinding:
    """验证官方关节名完整覆盖模型，并解析 qpos 地址。"""

    names = tuple(str(name) for name in joint_names)
    if not names or len(set(names)) != len(names):
        raise ValueError("源关节名必须非空且唯一")
    resolved_names = tuple(f"{model_prefix}{name}" for name in names)
    model_names = tuple(
        mujoco.mj_id2name(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            joint_id,
        )
        for joint_id in range(model.njnt)
    )
    if require_complete_model and set(resolved_names) != set(model_names):
        missing = sorted(set(model_names) - set(resolved_names))
        unexpected = sorted(set(resolved_names) - set(model_names))
        raise ValueError(
            f"官方关节名与 MuJoCo 不一致：missing={missing}，"
            f"unexpected={unexpected}"
        )
    addresses = []
    for name in resolved_names:
        joint_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            name,
        )
        if joint_id < 0:
            raise ValueError(f"MuJoCo 组合模型缺少关节：{name}")
        addresses.append(int(model.jnt_qposadr[joint_id]))
    return NamedQposBinding(names, tuple(addresses))


def apply_named_qpos(
    data: mujoco.MjData,
    binding: NamedQposBinding,
    values: np.ndarray,
) -> None:
    """按绑定写入一帧有限关节值，并清空旧速度。"""

    qpos = np.asarray(values, dtype=np.float64)
    if qpos.shape != (len(binding.qpos_addresses),) or not np.isfinite(qpos).all():
        raise ValueError("官方关节目标数量或数值无效")
    data.qpos[list(binding.qpos_addresses)] = qpos
    data.qvel[:] = 0.0


def update_mediapipe_target_scene(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    scene: mujoco.MjvScene,
    transformed_keypoints: np.ndarray,
    *,
    world_offset: np.ndarray | None = None,
    wrist_body_name: str = "r_wrist",
    clear_scene: bool = True,
) -> None:
    """在已竖直对齐的官方优化器坐标中绘制 MediaPipe 骨架。"""

    points = np.asarray(transformed_keypoints, dtype=np.float64)
    if points.shape != (21, 3) or not np.isfinite(points).all():
        raise ValueError("目标 MediaPipe 骨架必须是有限的 (21, 3)")
    offset = (
        np.zeros(3, dtype=np.float64)
        if world_offset is None
        else np.asarray(world_offset, dtype=np.float64)
    )
    if offset.shape != (3,) or not np.isfinite(offset).all():
        raise ValueError("MediaPipe 显示偏移必须是有限的 3 维向量")
    required_geoms = 41
    existing_geoms = 0 if clear_scene else int(scene.ngeom)
    if scene.maxgeom < existing_geoms + required_geoms:
        raise ValueError(
            f"目标骨架需要 {required_geoms} 个 geom，"
            f"已有 {existing_geoms} 个，场景仅允许 {scene.maxgeom} 个"
        )
    wrist_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        wrist_body_name,
    )
    if wrist_id < 0:
        raise ValueError(f"Wuji Hand2 模型缺少 {wrist_body_name}")
    local_points = points - points[0]
    world_points = (
        data.xpos[wrist_id]
        + local_points
        + offset
    )

    if clear_scene:
        scene.ngeom = 0
    tip_indices = {4, 8, 12, 16, 20}
    for point_index, position in enumerate(world_points):
        radius = 0.006 if point_index in tip_indices else 0.004
        append_sphere(
            scene,
            position,
            _TARGET_COLOR,
            radius,
            emission=1.0,
        )
    for chain in _MEDIAPIPE_CHAINS:
        for parent, child in zip(chain, chain[1:]):
            append_bone(
                scene,
                world_points[parent],
                world_points[child],
                _TARGET_COLOR,
                0.003,
                emission=1.0,
            )





def run(args: argparse.Namespace) -> None:
    apply_task_glove(args)
    overrides = getattr(args, "_profile_overrides", ())
    if args.glove_profile is not None and overrides:
        raise ValueError(
            "--glove-profile 不能与档案资源/网络选项同时指定："
            + ", ".join(sorted(overrides))
            + "；请修改或另存对应手套 JSON"
        )
    profile = apply_glove_profile(
        args, require_zero=not args.check_connection,
        require_resources=not args.check_connection,
    )
    args.hand = args.hand or "right"
    if args.hand == "left" and profile is None:
        raise ValueError("左手必须通过 --glove-profile 选择独立手套标定和模型，不能沿用右手默认文件")
    if args.zmq_port is None:
        args.zmq_port = DEFAULT_ZMQ_PORT[args.hand]
    if args.check_connection:
        prepare_glove_network(args)
        with connect_glove(args) as connection:
            frame = connection.read_frame()
            print(
                f"手套连接通过：channels={connection.channels} "
                f"sequence={frame.sequence}；已读取一帧，不启动仿真或发布控制目标",
                flush=True,
            )
        return
    if args.wuji_config is None:
        args.wuji_config = official_config_path(args.hand)
    print(
        "[模式] 已显式开启 hand_dof 目标发布"
        if args.publish_targets
        else "[模式] 仅仿真显示：不发布控制目标，不创建真机仿真租约",
        flush=True,
    )
    print("[启动 1/5] 加载数据手套观测与独立 MANO 拓扑手型...", flush=True)
    if profile is not None:
        glove = GloveRetargetPipeline.from_config(
            glove_urdf=profile.glove_urdf,
            mapping_data=profile.mapping_data,
            zero_profile=profile.load_zero(),
            morphology_file=args.mano_morphology,
            commissioning=args.commission_directions,
        )
    else:
        glove = GloveRetargetPipeline.load(
            glove_urdf=args.glove_urdf,
            glove_config=args.glove_config,
            zero_file=args.zero_file,
            morphology_file=args.mano_morphology,
            commissioning=args.commission_directions,
        )
    if glove.mapping.hand != args.hand:
        raise ValueError(f"手套配置为 {glove.mapping.hand}，与所选 {args.hand} 不符")
    resources = get_hand_resources(args.hand, generation="v2")
    if resources.mjcf is None:
        raise ValueError(f"Wuji Hand2 {args.hand} MJCF 未配置")
    print("[启动 2/5] 加载手套/MANO/Wuji 三列组合模型...", flush=True)
    scene_model, scene_data = load_retarget_composite_model(
        glove_urdf=args.glove_urdf,
        wuji_mjcf=resources.mjcf,
        column_spacing=args.column_spacing,
        hand=args.hand,
    )
    mujoco.mj_forward(scene_model, scene_data)
    glove_wrist_body_id = mujoco.mj_name2id(
        scene_model,
        mujoco.mjtObj.mjOBJ_BODY,
        "glove_wrist_link",
    )
    if glove_wrist_body_id < 0:
        raise ValueError("三列组合模型缺少 glove_wrist_link")
    glove_display_rotation = scene_data.xmat[
        glove_wrist_body_id
    ].reshape(3, 3).copy()
    glove_joint_names = tuple(glove.qpos_by_joint)
    glove_binding = build_named_qpos_binding(
        scene_model,
        glove_joint_names,
        model_prefix="glove_",
        require_complete_model=False,
    )
    binding: NamedQposBinding | None = None
    target_binding: OfficialTargetBinding | None = None
    target_sequence = 0
    prepare_glove_network(args)

    print("[启动 3/5] 启动官方 Wuji retarget worker...", flush=True)
    with (
        (
            SimulationLease(generation="v2", hand=args.hand)
            if args.publish_targets else nullcontext()
        ),
        (
            open_official_target_publisher(args.zmq_bind, args.zmq_port, hand=args.hand)
            if args.publish_targets else nullcontext()
        ) as target_publisher,
        launch_official_adapter(args) as retargeter,
    ):
        print(
            f"[启动 4/5] 连接数据手套 {args.host}:{args.port}...",
            flush=True,
        )
        with connect_glove(args) as connection:
            glove.validate_stream(connection)
            print(
                f"[启动 5/5] 手套握手通过，创建验证窗口："
                f"{args.host}:{args.port}",
                flush=True,
            )
            print(f"  worker: {args.worker_python}", flush=True)
            print(f"  config: {args.wuji_config}", flush=True)
            print(f"  morphology: {args.mano_morphology}", flush=True)
            if args.pinch_d1_cm is None:
                print("  pinch d1/d2: 使用官方 YAML 各指配置", flush=True)
            else:
                print(
                    f"  pinch d1: {args.pinch_d1_cm:g} cm（命令行覆盖）",
                    flush=True,
                )
            print(f"  pinch alpha max: {args.pinch_alpha_max:g}", flush=True)
            if target_publisher is not None:
                print(
                    f"  ZMQ: tcp://{args.zmq_bind}:{args.zmq_port} "
                    f"({args.hand} hand_dof)",
                    flush=True,
                )
            print(
                "  三列: 手套 URDF / 黄色独立手型 / Wuji Hand2，间距 "
                f"{args.column_spacing * 1000:.0f} mm",
                flush=True,
            )

            with mujoco.viewer.launch_passive(
                scene_model,
                scene_data,
                show_left_ui=False,
                show_right_ui=False,
            ) as viewer:
                viewer.cam.lookat[:] = scene_data.xpos[1:].mean(axis=0)
                viewer.cam.distance = 0.85
                viewer.cam.azimuth = 90.0
                viewer.cam.elevation = -20.0
                last_report = 0.0
                first_frame = True
                while viewer.is_running():
                    started = time.monotonic()
                    if first_frame:
                        print("[首帧 1/4] 读取最新手套帧...", flush=True)
                    keypoints = glove.read_mediapipe(
                        connection,
                        trust_hardware_zero=args.trust_hardware_zero,
                    )
                    if first_frame:
                        print("[首帧 2/4] 调用官方 Wuji 优化器...", flush=True)
                    result = retargeter.retarget(
                        keypoints,
                        apply_filter=args.apply_filter,
                    )
                    if first_frame:
                        print("[首帧 3/4] 写入 Wuji Hand2 qpos...", flush=True)
                    if binding is None:
                        binding = build_named_qpos_binding(
                            scene_model,
                            result.joint_names,
                            model_prefix="wuji_",
                            require_complete_model=False,
                        )
                        if target_publisher is not None:
                            target_binding = build_official_target_binding(
                                result.joint_names,
                                mapping_path=resources.mapping,
                                hand=args.hand,
                            )
                    elif result.joint_names != binding.joint_names:
                        raise RuntimeError(
                            "官方 worker 运行中改变了关节输出顺序"
                        )
                    glove_values = np.array(
                        [
                            glove.data.qpos[glove.qpos_by_joint[name]]
                            for name in glove_joint_names
                        ],
                        dtype=np.float64,
                    )
                    apply_named_qpos(scene_data, glove_binding, glove_values)
                    apply_named_qpos(scene_data, binding, result.qpos)
                    mujoco.mj_forward(scene_model, scene_data)
                    if target_publisher is not None:
                        if target_binding is None:
                            raise RuntimeError("官方真机目标绑定尚未初始化")
                        target_publisher.publish(
                            make_official_hand_target(
                                target_binding,
                                result.qpos,
                                sequence=target_sequence,
                                timestamp_ns=time.time_ns(),
                            )
                        )
                        target_sequence += 1
                    glove.landmarks.skeleton.update_scene(
                        glove.data,
                        viewer.user_scn,
                        world_offset=np.array(
                            (-args.column_spacing, 0.0, 0.0),
                            dtype=np.float64,
                        ),
                        world_rotation=glove_display_rotation,
                        clear_scene=True,
                    )
                    update_mediapipe_target_scene(
                        scene_model,
                        scene_data,
                        viewer.user_scn,
                        result.transformed_keypoints,
                        world_offset=np.array(
                            (-args.column_spacing, 0.0, 0.0),
                            dtype=np.float64,
                        ),
                        wrist_body_name=f"wuji_{args.hand[0]}_wrist",
                        clear_scene=False,
                    )
                    if first_frame:
                        print(
                            "[首帧 4/4] 已更新手套/Wuji 仿真姿态和两层骨架"
                            + (f"，已发布 {args.hand} hand_dof" if target_publisher is not None else ""),
                            flush=True,
                        )
                        first_frame = False
                    now = time.monotonic()
                    if now - last_report >= args.print_interval:
                        metrics = compute_tip_fit_metrics(result)
                        positions = ", ".join(
                            f"{name}={error * 1000:.1f}mm"
                            for name, error in zip(
                                _FINGER_NAMES,
                                metrics.position_error_m,
                            )
                        )
                        directions = ", ".join(
                            f"{name}={error:.1f}°"
                            for name, error in zip(
                                _FINGER_NAMES,
                                metrics.direction_error_deg,
                            )
                        )
                        elapsed_ms = (now - started) * 1000.0
                        print(
                            f"[tip-pos] {positions}\n"
                            f"[tip-dir] {directions}\n"
                            f"[solve] cost={result.cost:.4f} "
                            f"frame={elapsed_ms:.2f}ms",
                            flush=True,
                        )
                        last_report = now
                    viewer.sync()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="手套骨架 → 官方 Wuji 映射 → Hand2 MuJoCo 验证"
    )
    parser.add_argument("--hand", choices=("left", "right"), default=None)
    parser.add_argument("--glove-profile", help="具名手套档案名称或 JSON 路径")
    parser.add_argument("--task", help="具名任务；可配合--hand选择其中一侧")
    parser.add_argument("--host", action=_ProfileBoundArgument, default=os.environ.get("DATAGLOVE_HOST", "192.168.7.2"))
    parser.add_argument("--port", type=int, action=_ProfileBoundArgument, default=os.environ.get("DATAGLOVE_PORT", "5580"))
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--glove-interface", action=_ProfileBoundArgument, help="手套 USB 网卡名；默认按 MAC 自动识别")
    parser.add_argument("--glove-mac", action=_ProfileBoundArgument, help="手套 USB MAC；默认使用 DATAGLOVE_MAC 或内置值")
    parser.add_argument("--glove-host-address", action=_ProfileBoundArgument, help="手套网卡主机 IPv4/CIDR；默认 192.168.7.1/24")
    parser.add_argument("--glove-route-table", type=int, action=_ProfileBoundArgument, help="独立源地址策略路由表，范围 10000..30000")
    parser.add_argument("--usb-wait-timeout", type=int, help="等待手套 USB 网卡的秒数；默认 30")
    parser.add_argument(
        "--skip-network-setup", action="store_true",
        help="跳过网卡配置；用于已配置网络、远程手套或离线协议验证",
    )
    parser.add_argument(
        "--check-connection", action="store_true",
        help="仅做手套网络预检、协议握手并读取一帧，随后退出；绝不发布目标",
    )
    parser.add_argument(
        "--publish-targets", action="store_true",
        help="显式开启本机 hand_dof 发布和真机仿真租约；默认关闭",
    )
    parser.add_argument("--glove-urdf", type=Path, action=_ProfileBoundArgument, default=DEFAULT_URDF)
    parser.add_argument(
        "--glove-config",
        action=_ProfileBoundArgument,
        type=Path,
        default=_DEFAULT_GLOVE_CONFIG,
    )
    parser.add_argument(
        "--zero-file",
        action=_ProfileBoundArgument,
        type=Path,
        default=_DEFAULT_ZERO_FILE,
    )
    parser.add_argument(
        "--mano-morphology",
        type=Path,
        default=DEFAULT_MANO_MORPHOLOGY,
        help="独立 MANO 拓扑手型 v2 配置：固定掌部和骨长，以末端/方向优化姿态",
    )
    parser.add_argument("--commission-directions", action="store_true")
    parser.add_argument("--trust-hardware-zero", action="store_true")
    parser.add_argument(
        "--worker-python",
        type=Path,
        default=_DEFAULT_WORKER_PYTHON,
    )
    parser.add_argument(
        "--wuji-checkout",
        type=Path,
        default=_DEFAULT_WUJI_CHECKOUT,
    )
    parser.add_argument(
        "--wuji-config",
        type=Path,
        default=None,
        help="公共官方配置；默认仅按手侧选择 official/left.yaml 或 right.yaml",
    )
    parser.add_argument("--wuji-urdf", type=Path, default=None)
    parser.add_argument("--apply-filter", action="store_true")
    parser.add_argument(
        "--column-spacing",
        "--target-offset-y",
        dest="column_spacing",
        type=float,
        default=0.22,
    )
    parser.add_argument("--print-interval", type=float, default=1.0)
    parser.add_argument(
        "--pinch-d1-cm",
        type=float,
        default=None,
        help="统一覆盖四指对指 d1（cm）；默认保留官方 YAML 各指阈值",
    )
    parser.add_argument("--pinch-alpha-max", type=float, default=1.0)
    parser.add_argument("--zmq-bind", default="127.0.0.1")
    parser.add_argument(
        "--zmq-port",
        type=int,
        default=None,
    )
    return parser


def main() -> None:
    try:
        run(build_parser().parse_args())
    except KeyboardInterrupt:
        print("\n已停止，正在关闭手套连接。", file=sys.stderr)
        raise SystemExit(130) from None
    except (EOFError, OSError, RuntimeError, ValueError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        raise SystemExit(1) from None


if __name__ == "__main__":
    main()
