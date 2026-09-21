"""把手套 URDF 与 Wuji Hand2 合并为三列对比用单一 MuJoCo model。"""

from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np

from .mujoco_dataglove import load_dataglove_spec


# 由零位的“腕→中指”和“食指根→小指根”两组掌心基做刚体配准得到。
_GLOVE_TO_WUJI_DISPLAY_QUATERNION = np.array(
    (0.625444, -0.034932, -0.039589, -0.778481),
    dtype=np.float64,
)


def _orient_prefixed_glove_like_wuji(model: mujoco.MjModel, hand: str) -> None:
    """将手套零位掌心基旋转到 Wuji 查看器掌心基。"""

    wrist_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        "glove_wrist_link",
    )
    if wrist_id < 0:
        raise ValueError("组合模型缺少 glove_wrist_link")
    if int(model.body_parentid[wrist_id]) != 0:
        raise ValueError("组合模型 glove wrist 必须直接连接 world")
    rotation = _GLOVE_TO_WUJI_DISPLAY_QUATERNION
    if hand == "left":
        # 左手的真实 URDF 可能有不同根朝向；用两侧零位掌部基配准，
        # 不把右手预先标定的显示四元数套到左手，也不改变遥操坐标。
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        def palm_basis(wrist: str, joints: tuple[str, str, str]) -> np.ndarray:
            wrist_position = data.body(wrist).xpos
            middle, index, pinky = (
                data.xanchor[model.joint(name).id] for name in joints
            )
            y = middle - wrist_position
            y_norm = np.linalg.norm(y)
            if y_norm <= 1e-9:
                raise ValueError("左手显示配准的腕部到中指方向退化")
            y /= y_norm
            x = index - pinky
            x -= y * np.dot(x, y)
            x_norm = np.linalg.norm(x)
            if x_norm <= 1e-9:
                raise ValueError("左手显示配准的掌部横向退化")
            x /= x_norm
            return np.column_stack((x, y, np.cross(x, y)))

        glove_basis = palm_basis(
            "glove_wrist_link",
            ("glove_middle_mcp_flex_joint", "glove_index_mcp_flex_joint", "glove_pinky_mcp_abd_joint"),
        )
        wuji_basis = palm_basis(
            "wuji_l_wrist",
            ("wuji_l_middle_finger_mcp_flex", "wuji_l_index_finger_mcp_flex", "wuji_l_pinky_mcp_abd"),
        )
        rotation = np.empty(4)
        mujoco.mju_mat2Quat(rotation, (wuji_basis @ glove_basis.T).ravel())
    oriented_quaternion = np.empty(4)
    mujoco.mju_mulQuat(
        oriented_quaternion,
        rotation,
        model.body_quat[wrist_id],
    )
    model.body_quat[wrist_id] = oriented_quaternion


def _orient_prefixed_wrist_upright(model: mujoco.MjModel, hand: str) -> None:
    wrist_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        f"wuji_{hand[0]}_wrist",
    )
    if wrist_id < 0:
        raise ValueError(f"组合模型缺少 wuji_{hand[0]}_wrist")
    upright_rotation = np.array((0.0, 1.0, 0.0, 0.0))
    oriented_quaternion = np.empty(4)
    mujoco.mju_mulQuat(
        oriented_quaternion,
        upright_rotation,
        model.body_quat[wrist_id],
    )
    model.body_quat[wrist_id] = oriented_quaternion


def load_retarget_composite_model(
    *,
    glove_urdf: str | Path,
    wuji_mjcf: str | Path,
    column_spacing: float,
    hand: str = "right",
) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """编译手套和同侧 Wuji 两个前缀子模型，中列留给 MANO overlay。"""

    if hand not in ("left", "right"):
        raise ValueError("组合模型侧别必须是 left/right")
    spacing = float(column_spacing)
    if not math.isfinite(spacing) or spacing <= 0.0:
        raise ValueError("column_spacing 必须是正有限数值")
    glove_spec = load_dataglove_spec(glove_urdf)
    wuji_spec = mujoco.MjSpec.from_file(str(Path(wuji_mjcf).resolve()))
    composite = mujoco.MjSpec()
    composite.modelname = "glove-mano-wuji-comparison"
    composite.option.tolerance = wuji_spec.option.tolerance
    composite.option.iterations = wuji_spec.option.iterations
    composite.option.integrator = wuji_spec.option.integrator
    composite.option.jacobian = wuji_spec.option.jacobian
    glove_frame = composite.worldbody.add_frame(
        name="glove_column",
        pos=(-spacing, 0.0, 0.0),
    )
    wuji_frame = composite.worldbody.add_frame(
        name="wuji_column",
        pos=(spacing, 0.0, 0.0),
    )
    composite.attach(wuji_spec, prefix="wuji_", frame=wuji_frame)
    composite.attach(glove_spec, prefix="glove_", frame=glove_frame)
    model = composite.compile()
    if (model.nq, model.nv, model.nu) != (41, 41, 20):
        raise ValueError(
            "组合模型必须是 41 qpos / 41 dof / 20 actuator，"
            f"实际为 {model.nq}/{model.nv}/{model.nu}"
        )
    _orient_prefixed_wrist_upright(model, hand)
    _orient_prefixed_glove_like_wuji(model, hand)
    for geom_id in range(model.ngeom):
        body_name = mujoco.mj_id2name(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            int(model.geom_bodyid[geom_id]),
        )
        if body_name and body_name.startswith("glove_"):
            model.geom_rgba[geom_id, 3] = min(
                float(model.geom_rgba[geom_id, 3]),
                0.2,
            )
    return model, mujoco.MjData(model)
