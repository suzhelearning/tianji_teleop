"""MuJoCo 用户场景的关节球和骨段绘制原语。"""

from __future__ import annotations

from collections.abc import Sequence

import mujoco
import numpy as np


_IDENTITY_MATRIX = np.eye(3, dtype=np.float64).reshape(9)
_ZERO = np.zeros(3, dtype=np.float64)


def append_sphere(
    scene: mujoco.MjvScene,
    position: np.ndarray,
    rgba: Sequence[float],
    radius: float,
    *,
    emission: float = 0.0,
) -> None:
    """向用户场景追加一个彩色球。"""

    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.array((radius, 0.0, 0.0), dtype=np.float64),
        np.asarray(position, dtype=np.float64),
        _IDENTITY_MATRIX,
        np.asarray(rgba, dtype=np.float32),
    )
    geom.emission = float(emission)
    scene.ngeom += 1


def append_bone(
    scene: mujoco.MjvScene,
    start: np.ndarray,
    end: np.ndarray,
    rgba: Sequence[float],
    radius: float,
    *,
    emission: float = 0.0,
) -> None:
    """向用户场景追加一个连接两点的彩色胶囊。"""

    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        _ZERO,
        _ZERO,
        _IDENTITY_MATRIX,
        np.asarray(rgba, dtype=np.float32),
    )
    geom.emission = float(emission)
    mujoco.mjv_connector(
        geom,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        radius,
        np.asarray(start, dtype=np.float64),
        np.asarray(end, dtype=np.float64),
    )
    scene.ngeom += 1
