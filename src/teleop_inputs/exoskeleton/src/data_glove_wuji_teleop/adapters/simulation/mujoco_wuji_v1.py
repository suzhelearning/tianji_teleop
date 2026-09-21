"""
无极一代灵巧手 MuJoCo 查看器。

接收本机 ZMQ 消息，并将数据手套映射后的 20 路 DOF 映射到指定侧模型。
用法:
  python -m data_glove_wuji_teleop.adapters.simulation.mujoco_wuji_v1 \
    --hand left --port 15559
"""
from __future__ import annotations

import argparse
import math
import re
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from ...profiles.joint_mapping import load_joint_mapping
from ...project import SUPPORTED_HANDS, get_hand_resources
from ..runtime.simulation_session import SimulationLease
from ..transport.zmq_dof import DEFAULT_ZMQ_PORT
from .target_stream import ZmqDofSubscriber


def _build_mapping(side: str, config_path: Path) -> list[tuple[str, str, float, float]]:
    """加载配置并给关节名加前缀，生成完整映射表。"""
    rules = load_joint_mapping(
        config_path,
        expected_hand=side,
        expected_generation="v1",
    )
    return [
        (
            rule.dof_name,
            f"{side}_{rule.joint_name}",
            rule.scale,
            rule.offset_rad,
        )
        for rule in rules
    ]


def _load_model(urdf: Path) -> tuple[mujoco.MjModel, mujoco.MjData, dict[str, int]]:
    text = urdf.read_text(encoding="utf-8")
    assets: dict[str, bytes] = {}

    def collect(m: re.Match) -> str:
        rel  = m.group(1)
        full = (urdf.parent / rel).resolve()
        fname = full.name
        if full.exists() and fname not in assets:
            assets[fname] = full.read_bytes()
        return fname

    # 自动检测 palm link 名（官方 URDF 带 left_/right_ 前缀）
    import xml.etree.ElementTree as _ET
    _root = _ET.fromstring(text)
    palm_name = "palm_link"
    for _l in _root.findall("link"):
        if "palm" in _l.get("name", ""):
            palm_name = _l.get("name")
            break

    xml = re.sub(r'filename="(\.\.[^"]+)"', lambda m: f'filename="{collect(m)}"', text)
    xml = xml.replace('</robot>',
        f'\n  <link name="world_ref"/>'
        f'\n  <joint name="__world_fix__" type="fixed">'
        f'\n    <origin xyz="0 0 0" rpy="0 0 0"/>'
        f'\n    <parent link="world_ref"/>'
        f'\n    <child link="{palm_name}"/>'
        f'\n  </joint>\n</robot>')
    model = mujoco.MjModel.from_xml_string(xml, assets=assets)
    data  = mujoco.MjData(model)

    jidx: dict[str, int] = {}
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if name:
            jidx[name] = model.jnt_qposadr[i]
    return model, data, jidx


def run(args: argparse.Namespace) -> None:
    side = args.hand
    resources = get_hand_resources(side)
    urdf = resources.urdf
    port = args.port or DEFAULT_ZMQ_PORT[side]

    # 加载配置文件
    config_path = Path(args.config) if args.config else resources.mapping
    if not config_path.exists():
        print(f"错误: 配置文件不存在: {config_path}")
        return

    print(f"加载灵巧手模型 ({side})...")
    print(f"  配置文件: {config_path}")
    model, data, jidx = _load_model(urdf)

    semantic_to_wuji = _build_mapping(side, config_path)

    # 预取关节限位
    jlimits: dict[str, tuple[float, float]] = {}
    for jname, qi in jidx.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if model.jnt_limited[jid]:
            lo, hi = model.jnt_range[jid]
        else:
            lo, hi = -math.pi, math.pi
        jlimits[jname] = (float(lo), float(hi))

    missing = [
        joint
        for _, joint, _, _ in semantic_to_wuji
        if joint not in jidx
    ]
    if missing:
        print(f"  [警告] 找不到关节: {missing}")
    print(
        f"  已映射 {len(semantic_to_wuji) - len(missing)}/"
        f"{len(semantic_to_wuji)} 个关节"
    )

    sub = ZmqDofSubscriber(host=args.zmq_host, port=port, side=side)
    print(f"  监听 ZMQ tcp://{args.zmq_host}:{port}，等待手套 DOF 数据...")

    # 关节初始角
    thumb_j2 = f"{side}_finger1_joint2"
    initial_qpos: dict[str, float] = {thumb_j2: math.radians(0)}
    for jname, val in initial_qpos.items():
        qi = jidx.get(jname)
        if qi is not None:
            lo, hi = jlimits[jname]
            data.qpos[qi] = float(np.clip(val, lo, hi))

    mujoco.mj_forward(model, data)
    try:
        with SimulationLease(generation="v1", hand=side):
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
                f5j1 = f"{side}_finger5_joint1"
                while viewer.is_running():
                    dof = sub.get()
                    if dof:
                        for (
                            semantic_dof,
                            wuji_joint,
                            scale,
                            offset,
                        ) in semantic_to_wuji:
                            qi = jidx.get(wuji_joint)
                            if qi is None:
                                continue
                            semantic_value = dof.get(semantic_dof, 0.0)
                            val = semantic_value * scale + offset
                            lo, hi = jlimits[wuji_joint]
                            clipped = float(np.clip(val, lo, hi))
                            data.qpos[qi] = clipped
                        mujoco.mj_forward(model, data)
                        if frame % 100 == 0:
                            qi5 = jidx.get(f5j1)
                            finger5 = (
                                math.degrees(data.qpos[qi5])
                                if qi5 is not None
                                else "N/A"
                            )
                            print(
                                f"[wuji-{side}] "
                                f"index1_flex="
                                f"{math.degrees(dof.get('index1_flex', 0)):.1f}° "
                                f"pinky1_flex="
                                f"{math.degrees(dof.get('pinky1_flex', 0)):.1f}° "
                                f"finger5_j1={finger5}°"
                            )
                    else:
                        if frame % 100 == 0:
                            print(f"[wuji-{side}] 未收到数据 (frame={frame})")
                    frame += 1
                    viewer.sync()
                    time.sleep(0.02)
    finally:
        sub.close()


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="数据手套遥操作 Wuji MuJoCo 查看器")
    p.add_argument(
        "--hand",
        choices=SUPPORTED_HANDS,
        required=True,
        help="选择 left 或 right 模型",
    )
    p.add_argument("--zmq-host", default="localhost",
                   help="ZMQ 发布端主机（默认: localhost）")
    p.add_argument("--port", type=int, default=0,
                   help="ZMQ 发布端口（0=left 15559/right 15558）")
    p.add_argument("--config", type=str, default=None,
                   help="映射配置文件路径（默认按 --hand 选择）")
    p.add_argument("--camera", default="90,-20",
                   help="相机角度 azimuth,elevation")
    return p


def main() -> None:
    run(build_parser().parse_args())


if __name__ == "__main__":
    main()
