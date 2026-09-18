"""CPU/CUDA runtime contract for the 123-observation/26-action Regrind actor."""
from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import torch
from torch import nn

from ...data.reference import RegrindReference


DIMS = (123, 1024, 512, 256, 128, 26)
DEFAULT_JOINT_POS = np.asarray([0.28, *([0.0] * 19)], dtype=np.float64)




def load_actor(checkpoint: Path, device: str = "auto") -> tuple[nn.Sequential, torch.Tensor, torch.Tensor, int]:
    if device not in {"auto", "cpu", "cuda"}:
        raise ValueError("device must be auto, cpu, or cuda")
    if device == "auto":
        device = "cuda" if torch.cuda.is_available() else "cpu"
    selected = torch.device(device)
    if selected.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but unavailable; install CUDA-enabled PyTorch and an NVIDIA driver")
    payload = torch.load(checkpoint, map_location="cpu", weights_only=True)
    state = payload.get("actor_state_dict")
    if not isinstance(state, dict):
        raise ValueError("checkpoint has no actor_state_dict")
    layers: list[nn.Module] = []
    for index, (input_dim, output_dim) in enumerate(zip(DIMS[:-1], DIMS[1:])):
        layers.append(nn.Linear(input_dim, output_dim))
        if index < len(DIMS) - 2:
            layers.append(nn.ELU())
    actor = nn.Sequential(*layers)
    actor.load_state_dict(
        {key.removeprefix("mlp."): value for key, value in state.items() if key.startswith("mlp.")},
        strict=True,
    )
    mean = state.get("obs_normalizer._mean")
    variance = state.get("obs_normalizer._var")
    if not isinstance(mean, torch.Tensor) or not isinstance(variance, torch.Tensor):
        raise ValueError("checkpoint has no observation normalization statistics")
    mean, variance = mean.reshape(-1), variance.reshape(-1)
    if mean.shape != (DIMS[0],) or variance.shape != (DIMS[0],):
        raise ValueError(f"normalizer must have shape ({DIMS[0]},)")
    actor.to(selected)
    mean, variance = mean.to(selected), variance.to(selected)
    actor.eval()
    # Initialize CUDA kernels before entering a live control loop.
    infer(actor, mean, variance, np.zeros(DIMS[0], dtype=np.float32))
    return actor, mean, variance, int(payload.get("iter", -1))


def quat_wxyz_to_rot6d(quaternion: np.ndarray) -> np.ndarray:
    value = np.asarray(quaternion, dtype=np.float64)
    if value.shape != (4,) or not np.isfinite(value).all() or np.linalg.norm(value) < 1e-8:
        raise ValueError("quaternion must be four finite nonzero wxyz values")
    matrix = Rotation.from_quat(np.roll(value / np.linalg.norm(value), -1)).as_matrix()
    return matrix[:, :2].reshape(-1)


def build_observation(
    *,
    object_pos: np.ndarray,
    object_quat_wxyz: np.ndarray,
    previous_wrist_pos: np.ndarray,
    wrist_pos: np.ndarray,
    previous_wrist_quat_wxyz: np.ndarray,
    wrist_quat_wxyz: np.ndarray,
    previous_joints: np.ndarray,
    joints: np.ndarray,
    last_action: np.ndarray,
    phase: float,
    base_wrist_pos: np.ndarray,
    base_wrist_quat_wxyz: np.ndarray,
    base_joints: np.ndarray,
) -> np.ndarray:
    observation = np.concatenate(
        (
            object_pos,
            quat_wxyz_to_rot6d(object_quat_wxyz),
            previous_wrist_pos,
            wrist_pos,
            quat_wxyz_to_rot6d(previous_wrist_quat_wxyz),
            quat_wxyz_to_rot6d(wrist_quat_wxyz),
            previous_joints - DEFAULT_JOINT_POS,
            joints - DEFAULT_JOINT_POS,
            last_action,
            [phase],
            base_wrist_pos,
            quat_wxyz_to_rot6d(base_wrist_quat_wxyz),
            base_joints,
        )
    ).astype(np.float32)
    if observation.shape != (DIMS[0],) or not np.isfinite(observation).all():
        raise ValueError(f"observation must be {DIMS[0]} finite values")
    return observation


def infer(actor: nn.Sequential, mean: torch.Tensor, variance: torch.Tensor, observation: np.ndarray) -> np.ndarray:
    if observation.shape != (DIMS[0],) or not np.isfinite(observation).all():
        raise ValueError(f"observation must be {DIMS[0]} finite values")
    obs = torch.from_numpy(observation.astype(np.float32, copy=False)).reshape(1, -1).to(mean.device)
    with torch.inference_mode():
        action = actor((obs - mean) / (torch.sqrt(variance.clamp_min(1e-16)) + 1e-8))
    # The blocking copy also makes caller wall-clock timings include GPU work.
    result = action.squeeze(0).cpu().numpy().astype(np.float64)
    if result.shape != (DIMS[-1],) or not np.isfinite(result).all():
        raise ValueError(f"actor output must be {DIMS[-1]} finite values")
    return result


def action_to_targets(
    action: np.ndarray,
    base_wrist_pos: np.ndarray,
    base_wrist_quat_wxyz: np.ndarray,
    base_joints: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    clipped = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
    if clipped.shape != (DIMS[-1],) or not np.isfinite(clipped).all():
        raise ValueError(f"action must be {DIMS[-1]} finite values")
    wrist_pos = np.asarray(base_wrist_pos, dtype=np.float64) + 0.02 * clipped[:3]
    base_xyzw = np.roll(np.asarray(base_wrist_quat_wxyz, dtype=np.float64), -1)
    wrist_xyzw = (Rotation.from_rotvec(0.064 * clipped[3:6]) * Rotation.from_quat(base_xyzw)).as_quat()
    joints = np.asarray(base_joints, dtype=np.float64) + 0.064 * clipped[6:]
    return wrist_pos, np.roll(wrist_xyzw, 1), joints


def frame_zero_observation(reference: RegrindReference) -> np.ndarray:
    return build_observation(
        object_pos=reference.object_pos[0], object_quat_wxyz=reference.object_quat_wxyz[0],
        previous_wrist_pos=reference.wrist_pos[0], wrist_pos=reference.wrist_pos[0],
        previous_wrist_quat_wxyz=reference.wrist_quat_wxyz[0], wrist_quat_wxyz=reference.wrist_quat_wxyz[0],
        previous_joints=reference.joints[0], joints=reference.joints[0], last_action=np.zeros(26), phase=0.0,
        base_wrist_pos=reference.wrist_pos[0], base_wrist_quat_wxyz=reference.wrist_quat_wxyz[0],
        base_joints=reference.joints[0],
    )
