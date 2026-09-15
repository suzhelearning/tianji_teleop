"""无需硬件的 L/R 数据手套同场景运动学预览。"""

from __future__ import annotations

import argparse
import math
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer

from ...project import PROJECT_ROOT
from .mujoco_dataglove import DEFAULT_URDF, load_dataglove_spec


DEFAULT_LEFT_URDF = (
    PROJECT_ROOT / "assets/data_glove_urdf/data_glove_urdf_L/urdf/data_glove_urdf_L.urdf"
)


def load_pair_model(
    *,
    left_urdf: str | Path = DEFAULT_LEFT_URDF,
    right_urdf: str | Path = DEFAULT_URDF,
    spacing: float = 0.20,
) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """沿 Y 轴并排放置两只手；子模型名称全部加 L_/R_ 前缀。"""
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError("左右手间距必须为正有限数，单位为米")
    scene = mujoco.MjSpec()
    scene.modelname = "data_glove_L_R"
    for side, path, offset in (
        ("L", left_urdf, spacing / 2),
        ("R", right_urdf, -spacing / 2),
    ):
        frame = scene.worldbody.add_frame(
            name=f"{side}_frame", pos=(0.0, offset, 0.0)
        )
        scene.attach(load_dataglove_spec(path), prefix=f"{side}_", frame=frame)
    model = scene.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def configure_camera(camera: mujoco.MjvCamera, spacing: float) -> None:
    """从掌面正视，屏幕左侧为 L，右侧为 R。"""
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (0.0, 0.0, 0.065)
    camera.azimuth = 0.0
    camera.elevation = 0.0
    camera.distance = max(0.55, spacing * 2.5)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="L/R 数据手套并排预览，无需连接硬件")
    parser.add_argument("--left-urdf", type=Path, default=DEFAULT_LEFT_URDF)
    parser.add_argument("--right-urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--spacing", type=float, default=0.20, help="腕部间距，单位米")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    model, data = load_pair_model(
        left_urdf=args.left_urdf, right_urdf=args.right_urdf, spacing=args.spacing
    )
    reset_requested = threading.Event()

    def on_key(key: int) -> None:
        if key == ord("0"):
            reset_requested.set()

    print("左侧 L / 右侧 R；展开右侧 Joint 面板调整关节，按 0 将双手恢复零位。", flush=True)
    print("此入口仅做运动学预览，不连接硬件、不应用重力或编码器标定。", flush=True)
    with mujoco.viewer.launch_passive(
        model, data, key_callback=on_key, show_left_ui=False, show_right_ui=True
    ) as viewer:
        configure_camera(viewer.cam, args.spacing)
        viewer.set_texts(
            (None, None, "L (left)  |  R (right)", "Joint sliders; 0: reset pose")
        )
        while viewer.is_running():
            with viewer.lock():
                if reset_requested.is_set():
                    mujoco.mj_resetData(model, data)
                    reset_requested.clear()
                mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(1.0 / 60.0)


if __name__ == "__main__":
    main()
