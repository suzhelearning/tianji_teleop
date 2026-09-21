"""用实物数据手套的 21 路关节帧直接驱动其自身 URDF。"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import mujoco
import mujoco.viewer

from ...profiles.dataglove.urdf_mapping import (
    DataGloveUrdfMapping,
    UrdfJointFrame,
)
from ...profiles.dataglove.urdf_zero import UrdfZeroProfile
from ...project import PROJECT_ROOT
from ..glove.encoder_kinematics import EncoderKinematics, GloveJointFrame
from ..glove.encoder_stream import EncoderConnection, EncoderFrame
from .dataglove_skeleton import HandSkeleton


DEFAULT_URDF = (
    PROJECT_ROOT
    / "assets/data_glove_urdf/data_glove_urdf_R/urdf/data_glove_urdf_R.urdf"
)
DEFAULT_CONFIG = PROJECT_ROOT / "config/dataglove/urdf/right.json"
_PACKAGE_MESH_PATTERN = re.compile(
    r'filename="package://data_glove_urdf_[LR]/meshes/([^"/]+)"'
)


def load_dataglove_spec(urdf_path: str | Path) -> mujoco.MjSpec:
    """加载数据手套 URDF 和 package 网格为可组合的 MuJoCo spec。"""

    path = Path(urdf_path).resolve()
    package_root = path.parents[1]
    xml = path.read_text(encoding="utf-8")
    assets: dict[str, bytes] = {}

    def collect_mesh(match: re.Match[str]) -> str:
        filename = match.group(1)
        mesh_path = package_root / "meshes" / filename
        if not mesh_path.is_file():
            raise FileNotFoundError(f"数据手套 URDF 缺少网格：{mesh_path}")
        assets.setdefault(filename, mesh_path.read_bytes())
        return f'filename="{filename}"'

    xml = _PACKAGE_MESH_PATTERN.sub(collect_mesh, xml)
    return mujoco.MjSpec.from_string(xml, assets=assets)


def load_dataglove_urdf(
    urdf_path: str | Path,
) -> tuple[mujoco.MjModel, mujoco.MjData, dict[str, int]]:
    """加载 ROS package URI 网格，并返回全部可动关节的 qpos 地址。"""

    model = load_dataglove_spec(urdf_path).compile()
    data = mujoco.MjData(model)
    qpos_by_joint: dict[str, int] = {}
    for joint_id in range(model.njnt):
        name = mujoco.mj_id2name(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            joint_id,
        )
        if name:
            qpos_by_joint[name] = int(model.jnt_qposadr[joint_id])
    return model, data, qpos_by_joint


def apply_urdf_joint_frame(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    qpos_by_joint: dict[str, int],
    frame: UrdfJointFrame,
) -> None:
    """按名称写入一帧 21 路 URDF 关节角并刷新模型位姿。"""

    if len(frame.joint_names) != len(frame.positions_rad):
        raise ValueError("URDF 关节名数量与角度数量不一致")
    missing = [
        name for name in frame.joint_names if name not in qpos_by_joint
    ]
    if missing:
        raise ValueError(f"数据手套 URDF 缺少映射关节：{missing}")
    for name, position in zip(frame.joint_names, frame.positions_rad):
        data.qpos[qpos_by_joint[name]] = position
    mujoco.mj_forward(model, data)


def convert_encoder_frame_for_urdf(
    frame: EncoderFrame,
    kinematics: EncoderKinematics,
    *,
    zero_profile: UrdfZeroProfile | None = None,
) -> GloveJointFrame:
    """先应用可选软件零位，再执行非线性编码器机构换算。"""

    if zero_profile is None:
        zeroed_frame = frame
    else:
        zeroed_frame = EncoderFrame(
            sequence=frame.sequence,
            timestamp_ns=frame.timestamp_ns,
            dropped=frame.dropped,
            angles_deg=zero_profile.apply_angles(frame.angles_deg),
        )
    return kinematics.convert_zeroed_frame(zeroed_frame)


def run(args: argparse.Namespace) -> int:
    urdf_path = Path(args.urdf)
    mapping = DataGloveUrdfMapping.load(args.config)
    mapping.validate_urdf(urdf_path)
    if args.commission_directions:
        if not mapping.directions_verified:
            print(
                "[方向调试] 当前 direction 尚未完成实物确认；"
                "请逐关节运动并核对 URDF 屈伸/侧摆方向"
            )
    else:
        mapping.require_verified_directions()
    kinematics = EncoderKinematics.load(
        args.config,
        commissioning=args.commission_directions,
    )
    if kinematics.hand != mapping.hand:
        raise ValueError(
            "编码器运动学与 URDF 映射的 hand 不一致："
            f"{kinematics.hand} != {mapping.hand}"
        )
    zero_profile = (
        UrdfZeroProfile.load(args.zero_file) if args.zero_file else None
    )
    if zero_profile is not None:
        if zero_profile.hand != mapping.hand:
            raise ValueError(
                f"{args.zero_file} 是 {zero_profile.hand} 零位，"
                f"不能用于 {mapping.hand} URDF"
            )
        if not zero_profile.complete:
            labels = ", ".join(zero_profile.pending_groups)
            raise ValueError(f"URDF 软件零位标定未完成：{labels}")
    if kinematics.unset_input_direction_channels:
        channels = ", ".join(
            kinematics.unset_input_direction_channels
        )
        print(
            "[方向调试] 以下 input_directions 尚未填写，"
            f"本次临时按 +1 计算：{channels}"
        )

    model, data, qpos_by_joint = load_dataglove_urdf(urdf_path)
    skeleton = HandSkeleton.from_model(model) if args.show_skeleton else None
    if skeleton is not None:
        skeleton.make_mesh_translucent(model)
        print(
            "[骨架] 已启用五指父子关节链与掌骨框架；"
            "网格已设为半透明"
        )
    mujoco.mj_forward(model, data)
    missing = [
        name for name in mapping.joint_names if name not in qpos_by_joint
    ]
    if missing:
        raise ValueError(f"数据手套 URDF 缺少映射关节：{missing}")

    with EncoderConnection.connect(
        args.host,
        port=args.port,
        timeout=args.timeout,
    ) as connection:
        mapping.validate_stream(
            channels=connection.channels,
            cs_by_joint=connection.cs_by_joint,
        )
        if zero_profile is not None:
            zero_profile.validate_stream(
                connection.channels,
                connection.cs_by_joint,
                zeroed=connection.zeroed,
                range_min=connection.range_min,
                range_max=connection.range_max,
            )
        print(
            f"数据手套 URDF 已连接 {args.host}:{args.port}，"
            f"zeroed={str(connection.zeroed).lower()}，"
            f"映射关节={len(mapping.joint_names)}"
        )
        if zero_profile is not None:
            print(f"[软件零位] 四连杆换算前应用：{args.zero_file}")
        if (
            zero_profile is None
            and not connection.zeroed
            and args.trust_hardware_zero
        ):
            print(
                "[硬件零位] 板端报告 zeroed=false；"
                "按用户确认信任硬件已校零，不应用软件零位"
            )
        with mujoco.viewer.launch_passive(
            model,
            data,
            show_left_ui=False,
            show_right_ui=False,
        ) as viewer:
            if model.nbody > 1:
                viewer.cam.lookat[:] = data.xpos[1:].mean(axis=0)
            viewer.cam.distance = args.camera_distance
            viewer.cam.azimuth = args.camera_azimuth
            viewer.cam.elevation = args.camera_elevation
            if skeleton is not None:
                skeleton.update_scene(data, viewer.user_scn)
                viewer.sync()
            while viewer.is_running():
                if zero_profile is None:
                    glove_frame = connection.read_joint_frame(
                        kinematics,
                        trust_hardware_zero=args.trust_hardware_zero,
                    )
                else:
                    glove_frame = convert_encoder_frame_for_urdf(
                        connection.read_frame(),
                        kinematics,
                        zero_profile=zero_profile,
                    )
                urdf_frame = mapping.map_frame(glove_frame)
                apply_urdf_joint_frame(
                    model,
                    data,
                    qpos_by_joint,
                    urdf_frame,
                )
                if skeleton is not None:
                    skeleton.update_scene(data, viewer.user_scn)
                viewer.sync()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="实物数据手套 21 路编码器直接驱动自身 URDF"
    )
    parser.add_argument("--host", required=True, help="数据手套板端地址")
    parser.add_argument("--port", type=int, default=5580)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--zero-file",
        type=Path,
        default=None,
        help=(
            "逐指手工标定生成的软件零位 JSON；"
            "在四连杆换算前应用"
        ),
    )
    parser.add_argument(
        "--commission-directions",
        action="store_true",
        help=(
            "允许未确认的 direction 和 input_directions "
            "启动纯 URDF 查看器"
        ),
    )
    parser.add_argument(
        "--trust-hardware-zero",
        action="store_true",
        help=(
            "板端报告 zeroed=false 时，仍按已完成硬件校零处理；"
            "不会应用软件零位"
        ),
    )
    parser.add_argument(
        "--show-skeleton",
        action="store_true",
        help="叠加五指关节球、父子骨段和掌骨框架，并淡化原网格",
    )
    parser.add_argument("--camera-distance", type=float, default=0.35)
    parser.add_argument("--camera-azimuth", type=float, default=90.0)
    parser.add_argument("--camera-elevation", type=float, default=-25.0)
    return parser


def main() -> None:
    raise SystemExit(run(build_parser().parse_args()))


if __name__ == "__main__":
    main()
