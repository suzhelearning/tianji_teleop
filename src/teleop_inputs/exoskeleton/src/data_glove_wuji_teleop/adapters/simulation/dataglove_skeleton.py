"""实物数据手套 URDF 的运行时骨架叠层。"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from .mujoco_overlay import append_bone, append_sphere


_CHAIN_NAMES = (
    (
        "thumb",
        (
            "thumb_cmc_flex_joint",
            "thumb_cmc_abd_joint",
            "thumb_mcp_joint",
            "thumb_ip_joint",
        ),
    ),
    (
        "index",
        (
            "index_mcp_abd_joint",
            "index_mcp_flex_joint",
            "index_pip_joint",
            "index_dip_joint",
        ),
    ),
    (
        "middle",
        (
            "middle_mcp_abd_joint",
            "middle_mcp_flex_joint",
            "middle_pip_joint",
            "middle_dip_joint",
        ),
    ),
    (
        "ring",
        (
            "ring_mcp_abd_joint",
            "ring_mcp_flex_joint",
            "ring_pip_joint",
            "ring_dip_joint",
        ),
    ),
    (
        "pinky",
        (
            "pinky_cmc_joint",
            "pinky_mcp_abd_joint",
            "pink_mcp_flex_joint",
            "pink_pip_joint",
            "pink_dip_joint",
        ),
    ),
)
_CHAIN_COLORS = {
    "thumb": (1.00, 0.55, 0.10, 1.00),
    "index": (0.20, 0.90, 0.35, 1.00),
    "middle": (0.20, 0.65, 1.00, 1.00),
    "ring": (0.75, 0.35, 1.00, 1.00),
    "pinky": (1.00, 0.25, 0.35, 1.00),
}
_PALM_COLOR = (0.90, 0.90, 0.90, 1.00)
_FOUR_FINGER_FINGERTIP_LENGTH_M = 0.025
_THUMB_FINGERTIP_LENGTH_M = 0.030
_MIN_DIRECTION_LENGTH_M = 1e-9
_FINGERTIP_LOCAL_DIRECTION = np.array(
    (0.0, 0.0, 1.0),
    dtype=np.float64,
)


@dataclass(frozen=True)
class SkeletonChain:
    """一根手指按 URDF 父子顺序排列的关节。"""

    name: str
    joint_names: tuple[str, ...]
    joint_ids: tuple[int, ...]
    terminal_body_id: int
    fingertip_local_direction: tuple[float, float, float]
    fingertip_length_m: float
    rgba: tuple[float, float, float, float]


@dataclass(frozen=True)
class HandSkeleton:
    """经模型拓扑校验的五指关节链。"""

    chains: tuple[SkeletonChain, ...]

    @classmethod
    def from_model(cls, model: mujoco.MjModel) -> "HandSkeleton":
        reference_data = mujoco.MjData(model)
        mujoco.mj_forward(model, reference_data)
        chains: list[SkeletonChain] = []
        for name, joint_names in _CHAIN_NAMES:
            joint_ids = tuple(
                int(
                    mujoco.mj_name2id(
                        model,
                        mujoco.mjtObj.mjOBJ_JOINT,
                        joint_name,
                    )
                )
                for joint_name in joint_names
            )
            missing = tuple(
                joint_name
                for joint_name, joint_id in zip(joint_names, joint_ids)
                if joint_id < 0
            )
            if missing:
                raise ValueError(f"骨架缺少关节：{missing}")
            for parent_joint_id, child_joint_id in zip(
                joint_ids,
                joint_ids[1:],
            ):
                parent_body_id = int(model.jnt_bodyid[parent_joint_id])
                child_body_id = int(model.jnt_bodyid[child_joint_id])
                if int(model.body_parentid[child_body_id]) != parent_body_id:
                    raise ValueError(
                        f"{joint_names} 不符合模型父子 body 顺序"
                    )
            terminal_body_id = int(model.jnt_bodyid[joint_ids[-1]])
            fingertip_local_direction = _FINGERTIP_LOCAL_DIRECTION.copy()
            is_thumb = name == "thumb"
            if is_thumb:
                terminal_position = reference_data.xanchor[joint_ids[-1]]
                parent_position = reference_data.xanchor[joint_ids[-2]]
                reference_direction = terminal_position - parent_position
                reference_length = float(np.linalg.norm(reference_direction))
                if (
                    not np.isfinite(reference_length)
                    or reference_length <= _MIN_DIRECTION_LENGTH_M
                ):
                    raise ValueError(
                        "thumb 末端关节与父节点重合，无法确定指尖方向"
                    )
                terminal_rotation = reference_data.xmat[
                    terminal_body_id
                ].reshape(3, 3)
                fingertip_local_direction = (
                    terminal_rotation.T
                    @ (reference_direction / reference_length)
                )
            chains.append(
                SkeletonChain(
                    name=name,
                    joint_names=joint_names,
                    joint_ids=joint_ids,
                    terminal_body_id=terminal_body_id,
                    fingertip_local_direction=tuple(
                        float(value) for value in fingertip_local_direction
                    ),
                    fingertip_length_m=(
                        _THUMB_FINGERTIP_LENGTH_M
                        if is_thumb
                        else _FOUR_FINGER_FINGERTIP_LENGTH_M
                    ),
                    rgba=_CHAIN_COLORS[name],
                )
            )
        return cls(chains=tuple(chains))

    @property
    def required_geoms(self) -> int:
        joint_nodes = sum(len(chain.joint_ids) for chain in self.chains)
        finger_bones = sum(len(chain.joint_ids) - 1 for chain in self.chains)
        fingertip_nodes = len(self.chains)
        fingertip_bones = len(self.chains)
        palm_node = 1
        palm_spokes = len(self.chains)
        return (
            joint_nodes
            + finger_bones
            + fingertip_nodes
            + fingertip_bones
            + palm_node
            + palm_spokes
        )

    @staticmethod
    def make_mesh_translucent(
        model: mujoco.MjModel,
        *,
        max_alpha: float = 0.2,
    ) -> None:
        """降低已有模型几何体的不透明度，使内部骨架保持可见。"""

        alpha = float(max_alpha)
        if not np.isfinite(alpha) or not 0.0 <= alpha <= 1.0:
            raise ValueError("骨架网格透明度必须在 [0, 1] 内")
        model.geom_rgba[:, 3] = np.minimum(model.geom_rgba[:, 3], alpha)

    @staticmethod
    def fingertip_position(
        data: mujoco.MjData,
        chain: SkeletonChain,
    ) -> np.ndarray:
        """返回由末端刚体姿态驱动的固定长度指尖位置。"""

        terminal_position = np.asarray(
            data.xanchor[chain.joint_ids[-1]],
            dtype=np.float64,
        )
        terminal_rotation = data.xmat[chain.terminal_body_id].reshape(3, 3)
        world_direction = terminal_rotation @ np.asarray(
            chain.fingertip_local_direction,
            dtype=np.float64,
        )
        direction_length = float(np.linalg.norm(world_direction))
        if (
            not np.isfinite(direction_length)
            or direction_length <= _MIN_DIRECTION_LENGTH_M
        ):
            raise ValueError(f"{chain.name} 指尖方向无效")
        return (
            terminal_position
            + world_direction
            / direction_length
            * chain.fingertip_length_m
        )

    def update_scene(
        self,
        data: mujoco.MjData,
        scene: mujoco.MjvScene,
        *,
        joint_radius: float = 0.003,
        bone_radius: float = 0.002,
        world_offset: np.ndarray | None = None,
        world_rotation: np.ndarray | None = None,
        clear_scene: bool = True,
    ) -> None:
        """按当前关节锚点刷新关节球、骨段和掌骨中心。"""

        offset = (
            np.zeros(3, dtype=np.float64)
            if world_offset is None
            else np.asarray(world_offset, dtype=np.float64)
        )
        if offset.shape != (3,) or not np.isfinite(offset).all():
            raise ValueError("手套骨架显示偏移必须是有限的 3 维向量")
        rotation = (
            np.eye(3, dtype=np.float64)
            if world_rotation is None
            else np.asarray(world_rotation, dtype=np.float64)
        )
        if (
            rotation.shape != (3, 3)
            or not np.isfinite(rotation).all()
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-7)
        ):
            raise ValueError("手套骨架显示旋转必须是正交的 3x3 旋转矩阵")
        existing_geoms = 0 if clear_scene else int(scene.ngeom)
        if scene.maxgeom < existing_geoms + self.required_geoms:
            raise ValueError(
                f"骨架需要 {self.required_geoms} 个用户 geom，"
                f"已有 {existing_geoms} 个，场景仅允许 {scene.maxgeom} 个"
            )
        if clear_scene:
            scene.ngeom = 0
        root_positions: list[np.ndarray] = []
        for chain in self.chains:
            positions = tuple(
                rotation
                @ np.asarray(data.xanchor[joint_id], dtype=np.float64)
                + offset
                for joint_id in chain.joint_ids
            )
            root_positions.append(positions[0])
            for position in positions:
                append_sphere(
                    scene,
                    position,
                    chain.rgba,
                    joint_radius,
                )
            for start, end in zip(positions, positions[1:]):
                append_bone(
                    scene,
                    start,
                    end,
                    chain.rgba,
                    bone_radius,
                )
            fingertip_position = (
                rotation @ self.fingertip_position(data, chain) + offset
            )
            append_bone(
                scene,
                positions[-1],
                fingertip_position,
                chain.rgba,
                bone_radius,
            )
            append_sphere(
                scene,
                fingertip_position,
                chain.rgba,
                joint_radius,
            )

        palm_center = np.mean(root_positions, axis=0)
        append_sphere(
            scene,
            palm_center,
            _PALM_COLOR,
            joint_radius * 1.35,
        )
        for root_position in root_positions:
            append_bone(
                scene,
                palm_center,
                root_position,
                _PALM_COLOR,
                bone_radius,
            )
