#!/usr/bin/env python3
"""Cold-path compiler/exec wrapper for the native Wuji Hand 2 worker.

The pinned Python environment is used once to resolve the official YAML files,
Pinocchio frame/joint order and all numeric retarget settings.  The generated
manifest is consumed by the C++ process and the launcher then ``exec``s it;
there is no Python callback or Python retarget call after startup.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import struct
import tempfile

import numpy as np
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[1]
OFFICIAL = ROOT / "third_party/wuji_hand_retargeting"
HAND2_JOINT_NAMES = {
    "left": (
        "l_thumb_cmc_flex", "l_thumb_cmc_abd", "l_thumb_mcp", "l_thumb_ip",
        "l_index_finger_mcp_flex", "l_index_finger_mcp_abd",
        "l_index_finger_pip", "l_index_finger_dip",
        "l_middle_finger_mcp_flex", "l_middle_finger_mcp_abd",
        "l_middle_finger_pip", "l_middle_finger_dip",
        "l_ring_finger_mcp_flex", "l_ring_finger_mcp_abd",
        "l_ring_finger_pip", "l_ring_finger_dip",
        "l_pinky_mcp_flex", "l_pinky_mcp_abd",
        "l_pinky_pip", "l_pinky_dip",
    ),
    "right": (
        "r_thumb_cmc_flex", "r_thumb_cmc_abd", "r_thumb_mcp", "r_thumb_ip",
        "r_index_finger_mcp_flex", "r_index_finger_mcp_abd",
        "r_index_finger_pip", "r_index_finger_dip",
        "r_middle_finger_mcp_flex", "r_middle_finger_mcp_abd",
        "r_middle_finger_pip", "r_middle_finger_dip",
        "r_ring_finger_mcp_flex", "r_ring_finger_mcp_abd",
        "r_ring_finger_pip", "r_ring_finger_dip",
        "r_pinky_mcp_flex", "r_pinky_mcp_abd",
        "r_pinky_pip", "r_pinky_dip",
    ),
}

_MANIFEST = struct.Struct("<4sHHI")
_SIDE = struct.Struct("<HHIIIIII d")


def _finite_array(value, shape, field):
    array = np.ascontiguousarray(value, dtype=np.float64)
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError(f"{field} must be finite with shape {shape}")
    return array


def _side_record(retargeter, side):
    optimizer = retargeter.optimizer
    if type(optimizer).__name__ != "AdaptiveOptimizerAnalytical" or optimizer.num_joints != 20:
        raise ValueError("native worker only supports the pinned 20-DOF AdaptiveOptimizerAnalytical")

    role_names = [optimizer.origin_link_name] + optimizer.task_link_names
    role_names += optimizer.link3_names + optimizer.link4_names
    frame_names = [optimizer.robot.model.frames[optimizer.robot.get_link_index(name)].name
                   for name in role_names]
    joint_names = list(optimizer.robot.dof_joint_names)
    names = frame_names + joint_names
    if len(names) != 36 or len(set(names)) != 36:
        raise ValueError(f"native worker requires 36 unique names for {side}")

    rotation_xyz = retargeter.rotation_xyz
    rotation = Rotation.from_euler(
        "xyz", [rotation_xyz.get(axis, 0.0) for axis in ("x", "y", "z")], degrees=True
    ).as_matrix()
    geometry = np.concatenate((
        _finite_array(retargeter.input_axis_sign, (3,), f"{side} input_axis_sign"),
        _finite_array(rotation, (3, 3), f"{side} rotation").reshape(-1),
        _finite_array(retargeter.wrist_offset_m, (3,), f"{side} wrist_offset"),
        _finite_array(retargeter.thumb_offset_m, (3,), f"{side} thumb_offset"),
    ))
    params = np.asarray([
        optimizer.huber_delta, optimizer.huber_delta_dir, optimizer.norm_delta,
        optimizer.w_pos, optimizer.w_dir, optimizer.scaling, optimizer.w_full_hand,
        float(optimizer.thumb_skip_pip), optimizer.w_hyper, optimizer.soft_min,
        optimizer.w_couple, optimizer.couple_ratio,
        *optimizer.segment_scaling.reshape(-1).tolist(),
        *optimizer.d1.tolist(), *optimizer.d2.tolist(),
    ], dtype=np.float64)
    if params.shape != (35,) or not np.isfinite(params).all():
        raise ValueError(f"invalid {side} optimizer parameters")

    limits = _finite_array(optimizer.robot.joint_limits, (20, 2), f"{side} joint limits")
    canonical = HAND2_JOINT_NAMES[side]
    source_index = {name: index for index, name in enumerate(joint_names)}
    if set(source_index) != set(canonical):
        raise ValueError(f"{side} native worker joint names do not match Hand2 contract")
    permutation = np.asarray([source_index[name] for name in canonical], dtype=np.int32)
    return dict(
        side=1 if side == "left" else 2,
        urdf=str(Path(optimizer.config["optimizer"]["urdf_path"]).resolve()
                 if Path(optimizer.config["optimizer"]["urdf_path"]).is_absolute()
                 else (Path(optimizer.config["__yaml_dir"]) /
                       optimizer.config["optimizer"]["urdf_path"]).resolve()),
        names=names,
        geometry=geometry,
        params=params,
        limits=limits.reshape(-1),
        permutation=permutation,
        alpha=float(retargeter.lp_filter.alpha),
    )


def _manifest(left_config, right_config, selected_side):
    # Importing the pinned official bridge is intentionally confined to this
    # startup helper.  The resulting child is immediately replaced by C++.
    import sys
    sys.path.insert(0, str(OFFICIAL / "example"))
    from tj_wuji2_hand_bridge import OfficialWujiHand2Bridge

    bridge = OfficialWujiHand2Bridge(
        OFFICIAL,
        single_hand_side=selected_side,
        left_config=left_config,
        right_config=right_config,
    )
    records = [_side_record(bridge._retargeters[side], side) for side in ("left", "right")]
    payload = bytearray(_MANIFEST.pack(b"TJWM", 1, 1 if selected_side == "left" else 2, 2))
    for record in records:
        urdf = record["urdf"].encode("utf-8")
        if not urdf or len(urdf) > 1 << 20:
            raise ValueError("invalid native worker URDF path")
        names = [name.encode("utf-8") for name in record["names"]]
        payload.extend(_SIDE.pack(record["side"], 0, len(urdf), len(names), 18, 35, 40, 20,
                                  record["alpha"]))
        payload.extend(urdf)
        for name in names:
            if not name or len(name) > 4096:
                raise ValueError("invalid native worker model name")
            payload.extend(struct.pack("<I", len(name)))
            payload.extend(name)
        payload.extend(np.asarray(record["geometry"], dtype="<f8").tobytes())
        payload.extend(np.asarray(record["params"], dtype="<f8").tobytes())
        payload.extend(np.asarray(record["limits"], dtype="<f8").tobytes())
        payload.extend(np.asarray(record["permutation"], dtype="<i4").tobytes())
    return bytes(payload)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--native-worker", type=Path, required=True)
    parser.add_argument("--single-hand-side", choices=("left", "right"), required=True)
    parser.add_argument("--startup-handshake", action="store_true")
    parser.add_argument("--left-config", type=Path,
                        default=OFFICIAL / "example/config/adaptive_analytical_manus_wuji_hand_2_left.yaml")
    parser.add_argument("--right-config", type=Path,
                        default=OFFICIAL / "example/config/adaptive_analytical_manus_wuji_hand_2_right.yaml")
    args = parser.parse_args()
    native_worker = args.native_worker.resolve(strict=True)
    left_config = args.left_config.resolve(strict=True)
    right_config = args.right_config.resolve(strict=True)
    if left_config == right_config:
        raise ValueError("left and right Hand2 configurations must be distinct")
    runtime_dir = Path(os.environ.get("XDG_RUNTIME_DIR", tempfile.gettempdir())) / "tianji-teleop-hand"
    runtime_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    fd, manifest_name = tempfile.mkstemp(prefix="wuji-hand-", suffix=".manifest", dir=runtime_dir)
    os.close(fd)
    manifest = Path(manifest_name)
    try:
        manifest.write_bytes(_manifest(left_config, right_config, args.single_hand_side))
        os.chmod(manifest, 0o600)
        command = [str(native_worker), "--manifest", str(manifest),
                   "--single-hand-side", args.single_hand_side]
        if args.startup_handshake:
            command.append("--startup-handshake")
        os.execv(str(native_worker), command)
    except BaseException:
        manifest.unlink(missing_ok=True)
        raise


if __name__ == "__main__":
    main()
