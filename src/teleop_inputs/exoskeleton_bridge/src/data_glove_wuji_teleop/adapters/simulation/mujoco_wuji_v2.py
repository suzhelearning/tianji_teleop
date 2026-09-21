"""数据手套语义 20-DOF 到 Wuji Hand 2 MuJoCo 的仿真适配器。"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import mujoco
import mujoco.viewer
import numpy as np

from ...profiles.joint_mapping import load_joint_mapping
from ...profiles.wuji_v2.model import full_joint_name
from ...project import SUPPORTED_HANDS, get_hand_resources
from ..runtime.simulation_session import SimulationLease
from ..transport.zmq_dof import DEFAULT_ZMQ_PORT
from .target_stream import ZmqDofSubscriber


@dataclass(frozen=True)
class ControlBinding:
    """一个语义 DOF 对应的官方 MJCF 位置执行器。"""

    dof_name: str
    joint_name: str
    actuator_id: int
    scale: float
    offset_rad: float
    lower: float
    upper: float


def _orient_fixed_wrist_upright(model: mujoco.MjModel) -> None:
    """将官方 -Z 展开的固定腕坐标转换为查看器的 +Z 竖直约定。"""

    wrist_ids: list[int] = []
    for name in ("l_wrist", "r_wrist"):
        body_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            name,
        )
        if body_id >= 0:
            wrist_ids.append(body_id)
    if len(wrist_ids) != 1:
        raise ValueError(
            "Wuji v2 模型必须恰好包含一个 l_wrist 或 r_wrist 固定根体"
        )
    wrist_id = wrist_ids[0]
    if int(model.body_parentid[wrist_id]) != 0:
        raise ValueError("Wuji v2 wrist 必须直接连接 world")
    if int(model.body_jntnum[wrist_id]) != 0:
        raise ValueError("Wuji v2 wrist 必须是固定根体")

    upright_rotation = np.array((0.0, 1.0, 0.0, 0.0))
    oriented_quaternion = np.empty(4)
    mujoco.mju_mulQuat(
        oriented_quaternion,
        upright_rotation,
        model.body_quat[wrist_id],
    )
    model.body_quat[wrist_id] = oriented_quaternion


def load_model(model_path: str | Path) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """加载官方二代 MJCF，校验执行器并应用仿真世界姿态。"""

    path = Path(model_path)
    model = mujoco.MjModel.from_xml_path(str(path.resolve()))
    if model.nq != 20 or model.nv != 20 or model.nu != 20:
        raise ValueError(
            "Wuji v2 模型必须是 20 qpos / 20 dof / 20 actuator，"
            f"实际为 {model.nq}/{model.nv}/{model.nu}"
        )
    _orient_fixed_wrist_upright(model)
    return model, mujoco.MjData(model)


def build_control_bindings(
    model: mujoco.MjModel,
    *,
    hand: str,
    config_path: str | Path,
) -> tuple[ControlBinding, ...]:
    """把完整语义映射绑定到模型中的唯一位置执行器。"""

    rules = load_joint_mapping(
        config_path,
        expected_hand=hand,
        expected_generation="v2",
    )
    bindings: list[ControlBinding] = []
    used_actuators: set[int] = set()
    for rule in rules:
        joint_name = full_joint_name(hand, rule.joint_name)
        joint_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            joint_name,
        )
        if joint_id < 0:
            raise ValueError(f"Wuji v2 模型缺少关节：{joint_name}")
        actuator_ids = [
            actuator_id
            for actuator_id in range(model.nu)
            if int(model.actuator_trnid[actuator_id, 0]) == joint_id
        ]
        if len(actuator_ids) != 1:
            raise ValueError(
                f"关节 {joint_name} 必须恰好绑定一个执行器，"
                f"实际 {len(actuator_ids)}"
            )
        actuator_id = actuator_ids[0]
        if actuator_id in used_actuators:
            raise ValueError(f"执行器被重复映射：{actuator_id}")
        used_actuators.add(actuator_id)
        if model.actuator_ctrllimited[actuator_id]:
            lower, upper = model.actuator_ctrlrange[actuator_id]
        elif model.jnt_limited[joint_id]:
            lower, upper = model.jnt_range[joint_id]
        else:
            raise ValueError(f"关节与执行器均无限位：{joint_name}")
        bindings.append(
            ControlBinding(
                dof_name=rule.dof_name,
                joint_name=joint_name,
                actuator_id=actuator_id,
                scale=rule.scale,
                offset_rad=rule.offset_rad,
                lower=float(lower),
                upper=float(upper),
            )
        )
    return tuple(bindings)


def apply_dof_target(
    data: mujoco.MjData,
    bindings: tuple[ControlBinding, ...],
    dof: Mapping[str, float],
) -> None:
    """将一帧语义 DOF 写入官方位置执行器，并执行模型限位。"""

    for binding in bindings:
        value = (
            float(dof.get(binding.dof_name, 0.0)) * binding.scale
            + binding.offset_rad
        )
        data.ctrl[binding.actuator_id] = float(
            np.clip(value, binding.lower, binding.upper)
        )


def run(args: argparse.Namespace) -> None:
    hand = args.hand
    resources = get_hand_resources(hand, generation="v2")
    if resources.mjcf is None:
        raise ValueError("Wuji v2 MJCF 路径未配置")
    config_path = Path(args.config) if args.config else resources.mapping
    model, data = load_model(resources.mjcf)
    bindings = build_control_bindings(
        model,
        hand=hand,
        config_path=config_path,
    )
    port = args.port or DEFAULT_ZMQ_PORT[hand]

    print(f"加载 Wuji Hand 2 ({hand})：{resources.mjcf}")
    print(f"  配置文件：{config_path}")
    print(f"  已绑定 {len(bindings)}/{model.nu} 个位置执行器")
    subscriber = ZmqDofSubscriber(
        host=args.zmq_host,
        port=port,
        side=hand,
    )
    print(f"  监听 ZMQ tcp://{args.zmq_host}:{port}")

    mujoco.mj_forward(model, data)
    frame_seconds = 0.02
    steps_per_frame = max(
        1,
        int(round(frame_seconds / float(model.opt.timestep))),
    )
    try:
        with SimulationLease(generation="v2", hand=hand):
            with mujoco.viewer.launch_passive(
                model,
                data,
                show_left_ui=False,
                show_right_ui=False,
            ) as viewer:
                viewer.cam.lookat[:] = data.xpos[1:].mean(axis=0)
                viewer.cam.distance = 0.5
                viewer.cam.azimuth = float(args.camera.split(",")[0])
                viewer.cam.elevation = float(args.camera.split(",")[1])

                frame = 0
                while viewer.is_running():
                    started = time.monotonic()
                    dof = subscriber.get()
                    if dof:
                        apply_dof_target(data, bindings, dof)
                    for _ in range(steps_per_frame):
                        mujoco.mj_step(model, data)
                    if frame % 100 == 0:
                        state = "已接收目标" if dof else "等待目标"
                        print(f"[wuji-v2:{hand}] {state} frame={frame}")
                    frame += 1
                    viewer.sync()
                    remaining = frame_seconds - (time.monotonic() - started)
                    if remaining > 0.0:
                        time.sleep(remaining)
    finally:
        subscriber.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="数据手套遥操作 Wuji Hand 2 MuJoCo 查看器"
    )
    parser.add_argument(
        "--hand",
        choices=SUPPORTED_HANDS,
        required=True,
        help="选择 left 或 right 模型",
    )
    parser.add_argument("--zmq-host", default="localhost")
    parser.add_argument(
        "--port",
        type=int,
        default=0,
        help="ZMQ 端口（0=left 15559/right 15558）",
    )
    parser.add_argument(
        "--config",
        default=None,
        help="v2 映射配置路径（默认按 --hand 选择）",
    )
    parser.add_argument("--camera", default="90,-20")
    return parser


def main() -> None:
    run(build_parser().parse_args())


if __name__ == "__main__":
    main()
