"""在带 Pinocchio/NLopt 的外部 Python 中运行官方 Wuji Retargeter。"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path
from types import MethodType

import numpy as np


_PINCH_FINGERS = ("index", "middle", "ring", "pinky")
_M_TO_CM = 100.0


def apply_pinch_d1_override(
    config: dict[str, object],
    pinch_d1_cm: float,
) -> None:
    """统一覆盖四指对指 d1，并要求每项仍严格小于 d2。"""

    value = float(pinch_d1_cm)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("pinch d1 必须是正有限厘米数")
    retarget = config.get("retarget", {})
    if not isinstance(retarget, dict):
        raise ValueError("retarget 配置必须是对象")
    thresholds = retarget.get("pinch_thresholds", {})
    if not isinstance(thresholds, dict):
        raise ValueError("pinch_thresholds 必须是对象")
    entries: dict[str, dict[str, object]] = {}
    for finger in _PINCH_FINGERS:
        current_entry = thresholds.get(finger, {})
        if not isinstance(current_entry, dict):
            raise ValueError(f"pinch_thresholds.{finger} 必须是对象")
        entry = dict(current_entry)
        try:
            d2 = float(entry.get("d2", 4.0))
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(
                f"pinch_thresholds.{finger}.d2 必须是有限数值"
            ) from exc
        if not math.isfinite(d2) or value >= d2:
            raise ValueError(
                f"pinch d1={value:g} 必须小于 {finger}.d2={d2:g}"
            )
        entry["d1"] = value
        entries[finger] = entry
    if "retarget" not in config:
        config["retarget"] = retarget
    if "pinch_thresholds" not in retarget:
        retarget["pinch_thresholds"] = thresholds
    thresholds.update(entries)


def apply_pinch_alpha_max_override(retargeter, alpha_max: float) -> None:
    """覆盖官方 Adaptive optimizer 硬编码的 pinch alpha 上限。"""

    value = float(alpha_max)
    if not math.isfinite(value) or not 0.0 < value <= 1.0:
        raise ValueError("pinch alpha max 必须在 (0, 1] 内")
    optimizer = retargeter.optimizer
    original = getattr(optimizer, "_compute_pinch_alpha", None)
    if not callable(original):
        raise ValueError("官方 optimizer 缺少 _compute_pinch_alpha API")
    d1 = np.asarray(optimizer.d1, dtype=np.float64)
    d2 = np.asarray(optimizer.d2, dtype=np.float64)
    tip_indices = tuple(int(index) for index in optimizer.MP_TIP_INDICES)
    if (
        d1.shape != (4,)
        or d2.shape != (4,)
        or not np.isfinite(d1).all()
        or not np.isfinite(d2).all()
        or np.any(d1 >= d2)
    ):
        raise ValueError("官方 optimizer pinch d1/d2 必须是四组递增有限阈值")
    if (
        len(tip_indices) != 5
        or len(set(tip_indices)) != 5
        or any(index < 0 or index >= 21 for index in tip_indices)
    ):
        raise ValueError("官方 optimizer pinch 阈值或指尖索引无效")

    # 官方 2026.7 实现将 0.7 硬编码在私有方法中；仅对当前实例安装兼容层，
    # 并在上面完整验证它依赖的 API，避免污染其他 Retargeter 实例。
    def compute_pinch_alpha(self, mediapipe_keypoints: np.ndarray) -> np.ndarray:
        keypoints = np.asarray(mediapipe_keypoints, dtype=np.float64)
        thumb_tip = keypoints[tip_indices[0]]
        finger_tips = keypoints[list(tip_indices[1:])]
        distances = np.linalg.norm(finger_tips - thumb_tip, axis=1) * _M_TO_CM
        # 安装时已验证 d1 < d2；不加 epsilon，保证阈值处能达到完整权重。
        alphas_4 = np.clip(
            (self.d2 - distances) / (self.d2 - self.d1),
            0.0,
            value,
        )
        return np.concatenate(([float(np.max(alphas_4))], alphas_4))

    optimizer._compute_pinch_alpha = MethodType(  # noqa: SLF001
        compute_pinch_alpha,
        optimizer,
    )


def _robot_endpoint_positions(retargeter, qpos: np.ndarray) -> dict[str, object]:
    optimizer = retargeter.optimizer
    robot = optimizer.robot
    robot.compute_forward_kinematics(qpos)

    def position(link_name: str) -> list[float]:
        link_index = robot.get_link_index(link_name)
        pose = robot.get_link_pose(link_index)
        return np.asarray(pose[:3, 3], dtype=np.float64).tolist()

    return {
        "robot_wrist": position(optimizer.origin_link_name),
        "robot_dip_positions": [
            position(name) for name in optimizer.link4_names
        ],
        "robot_tip_positions": [
            position(name) for name in optimizer.task_link_names
        ],
    }


def process_retarget_request(
    retargeter,
    joint_names: Sequence[str],
    payload: Mapping[str, object],
) -> dict[str, object]:
    """处理一帧协议请求，返回可 JSON 序列化的官方求解结果。"""

    keypoints = np.asarray(payload.get("keypoints"), dtype=np.float64)
    if keypoints.shape != (21, 3) or not np.isfinite(keypoints).all():
        raise ValueError("keypoints 必须是有限的 (21, 3) 数组")
    qpos, verbose = retargeter.retarget_verbose(
        keypoints,
        apply_filter=bool(payload.get("apply_filter", False)),
    )
    transformed = np.asarray(verbose["mediapipe_kp"], dtype=np.float64)
    response = {
        "qpos": np.asarray(qpos, dtype=np.float64).tolist(),
        "joint_names": [str(name) for name in joint_names],
        "transformed_keypoints": transformed.tolist(),
        "cost": float(verbose["cost"]),
    }
    response.update(_robot_endpoint_positions(retargeter, np.asarray(qpos)))
    return response


def _load_retargeter(args: argparse.Namespace):
    checkout = Path(args.checkout).resolve()
    if not (checkout / "wuji_retargeting").is_dir():
        raise FileNotFoundError(f"官方 checkout 无效：{checkout}")
    sys.path.insert(0, str(checkout))
    import yaml

    from wuji_retargeting import Retargeter

    config_path = Path(args.config).resolve()
    with config_path.open(encoding="utf-8") as file:
        config = yaml.safe_load(file)
    config["__yaml_dir"] = str(config_path.parent)
    if args.pinch_d1_cm is not None:
        apply_pinch_d1_override(config, args.pinch_d1_cm)
    if args.robot_urdf:
        config.setdefault("optimizer", {})["urdf_path"] = str(
            Path(args.robot_urdf).resolve()
        )
    retargeter = Retargeter.from_config(config, hand_side=args.hand)
    if args.pinch_alpha_max is not None:
        apply_pinch_alpha_max_override(
            retargeter,
            args.pinch_alpha_max,
        )
    joint_names = tuple(retargeter.optimizer.robot.dof_joint_names)
    return retargeter, joint_names


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="官方 Wuji Retargeter 常驻 JSON-line worker"
    )
    parser.add_argument("--checkout", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--robot-urdf", default="")
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument("--pinch-d1-cm", type=float, default=None)
    parser.add_argument("--pinch-alpha-max", type=float, default=None)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    with contextlib.redirect_stdout(sys.stderr):
        retargeter, joint_names = _load_retargeter(args)
    print(json.dumps({"ready": True}), flush=True)

    for request_line in sys.stdin:
        try:
            payload = json.loads(request_line)
            if not isinstance(payload, Mapping):
                raise ValueError("请求根节点必须是对象")
            with contextlib.redirect_stdout(sys.stderr):
                response = process_retarget_request(
                    retargeter,
                    joint_names,
                    payload,
                )
        except Exception as exc:  # noqa: BLE001 - error crosses process protocol
            response = {"error": f"{type(exc).__name__}: {exc}"}
        print(
            json.dumps(response, ensure_ascii=False, separators=(",", ":")),
            flush=True,
        )


if __name__ == "__main__":
    main()
