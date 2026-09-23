"""Offline, read-only shared-root artifact validation. No SDK, network or viewer."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml
from tianji_runtime import controller_profile
from tianji_runtime.resources import controller_resource


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate(profile_path: Path) -> dict:
    profile_path = profile_path.resolve(strict=True)
    profile = yaml.safe_load(profile_path.read_text())
    cfg = profile["shared_root"]
    required = {
        "enabled", "input_contract_artifact", "robot_geometry_artifact", "input_mode",
        "control_point_semantic", "shape_proxy_semantic", "morphology",
        "bridge_frame_filter", "continuity", "target_gate",
    }
    unknown = set(cfg) - required - {"reachable_projection"}
    missing = required - set(cfg)
    if unknown or missing:
        raise ValueError(f"invalid shared-root fields: missing={sorted(missing)}, unknown={sorted(unknown)}")
    input_path = controller_resource(profile_path, cfg["input_contract_artifact"])
    geometry_path = controller_resource(profile_path, cfg["robot_geometry_artifact"])
    contract = yaml.safe_load(input_path.read_text())["tjvr_shared_root_input"]
    geometry = yaml.safe_load(geometry_path.read_text())["robot_geometry"]
    if contract["contract_version"] != 1 or geometry["contract_version"] != 2:
        raise ValueError("unsupported contract version")
    if (geometry["transform_convention"] != "parent_T_child" or
            geometry["wrist_center_orientation_semantic"] != "virtual_frame_axes_parallel_to_solver_tcp"):
        raise ValueError("invalid closure frame convention")
    for side, suffix in (("left", "L"), ("right", "R")):
        node = geometry[side]
        for key, prefix in (("shoulder_center_frame", "Link1_"), ("elbow_center_frame", "Link4_"),
                            ("wrist_center_frame", "Link5_"), ("solver_tcp_frame", "tcp_")):
            if node[key] != prefix + suffix:
                raise ValueError("invalid joint center source")
        for key in ("upper_arm_length_m", "forearm_length_m", "wrist_to_shape_proxy_length_m"):
            if not np.isfinite(node[key]) or node[key] <= 0:
                raise ValueError("invalid closure length")
        for key in ("T_solver_tcp_to_wrist_center", "T_link7_to_solver_tcp"):
            t = np.asarray(node[key]["translation_m"], dtype=float)
            q = np.asarray(node[key]["quaternion_wxyz"], dtype=float)
            if t.shape != (3,) or q.shape != (4,) or not np.isfinite(t).all() or not np.isfinite(q).all():
                raise ValueError("invalid closure transform")
            np.testing.assert_allclose(np.linalg.norm(q), 1, atol=1e-10, rtol=0)
        np.testing.assert_allclose(node["T_solver_tcp_to_wrist_center"]["quaternion_wxyz"],
                                   [1, 0, 0, 0], atol=1e-10, rtol=0)
        w, x, y, z = node["T_link7_to_solver_tcp"]["quaternion_wxyz"]
        rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                             [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                             [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        np.testing.assert_allclose(np.asarray(node["T_link7_to_solver_tcp"]["translation_m"]) +
                                   rotation @ np.asarray(node["T_solver_tcp_to_wrist_center"]["translation_m"]),
                                   0, atol=1e-10, rtol=0)
    evidence = geometry["closure_validation"]
    if evidence["sampled_pose_count"] < 100 or evidence["sampling_method"] != "deterministic_joint_range_sine_v1":
        raise ValueError("incomplete closure validation")
    if evidence["sampled_joint_range_source"] != "mujoco_joint_ranges":
        raise ValueError("closure validation must sample the MuJoCo joint ranges")
    # The measured per-axis maxima stay in the referenced evidence report; the
    # artifact binds the bounds they were accepted against. Declared
    # cross-model tolerances may never be looser than those recorded bounds.
    for tolerance, declared in (("tolerance_m", "cross_model_position_tolerance_m"),
                                ("tolerance_rad", "cross_model_orientation_tolerance_rad"),
                                ("invariant_tolerance_m", "fixed_shoulder_position_tolerance_m")):
        bound = evidence[tolerance]
        if not np.isfinite(bound) or bound <= 0:
            raise ValueError("invalid closure tolerance: " + tolerance)
        if not np.isfinite(geometry[declared]) or not 0 < geometry[declared] <= bound:
            raise ValueError("invalid declared closure tolerance: " + declared)
    if geometry["tjvr_input_contract_sha256"] != digest(input_path):
        raise ValueError("input contract fingerprint mismatch")
    for name in ("urdf", "mujoco_xml"):
        path = controller_resource(geometry_path, geometry[name + "_path"])
        if digest(path) != geometry[name + "_sha256"]:
            raise ValueError("model fingerprint mismatch: " + name)
    np.testing.assert_array_equal(contract["o_Ct_contract_m"], [0, 0, 1.121])
    for matrix in (contract["left_basis"], contract["right_basis"], geometry["R_BCt"]):
        r = np.asarray(matrix, dtype=float)
        if r.shape != (3, 3) or not np.isfinite(r).all():
            raise ValueError("invalid rotation shape/finite")
        np.testing.assert_allclose(r.T @ r, np.eye(3), atol=1e-12)
        np.testing.assert_allclose(np.linalg.det(r), 1, atol=1e-12)
    if contract["palm_pose_source"]["runtime_source_fallback_allowed"] is not False:
        raise ValueError("source fallback forbidden")
    if contract["upstream_symmetry"] != "symmetric_max_allowed_and_preserved":
        raise ValueError("effective symmetric geometry contract missing")
    if contract.get("allowed_geometry_sources") != ["measured_symmetric_max", "left_measured_symmetric_local_y_height_template"]:
        raise ValueError("unsupported geometry sources")
    simple = contract.get("simple_geometry_contract", {})
    if (simple.get("derived_schema") != "pico_simple_derived_v1" or
            simple.get("tcp_mirror") != "explicit_local_y_model_assumption_requires_device_axis_check"):
        raise ValueError("missing explicit simple model semantics")
    for key in ("morphology", "bridge_frame_filter", "continuity", "target_gate"):
        if not isinstance(cfg[key], dict) or not cfg[key]:
            raise ValueError("missing shared-root configuration: " + key)
        for value in cfg[key].values():
            values = np.asarray(value, dtype=float)
            if not np.isfinite(values).all() or not (values > 0).all():
                raise ValueError("configuration values must be finite and positive")
    m = cfg["morphology"]
    if not 2 <= m["minimum_unique_samples"] <= m["window_frames"] <= 128:
        raise ValueError("invalid bounded morphology window")
    for key, value in m.items():
        if isinstance(value, list) and (len(value) != 2 or value[0] >= value[1]):
            raise ValueError("invalid range: " + key)
    if profile["ik"]["algorithm"] != "pico_ee_franka_dls":
        raise ValueError("expected Franka DLS profile")
    return {"passed": True, "scope": "offline_artifact_consistency_only",
            "profile_sha256": digest(profile_path),
            "input_contract_sha256": digest(input_path),
            "robot_geometry_sha256": digest(geometry_path),
            "publisher_attestation": False, "motion_authorized": False}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile", type=Path, nargs="?")
    args = parser.parse_args()
    if args.profile is None:
        args.profile = controller_profile("qp_ik_pico_shared_root_dls.yaml")
    print(json.dumps(validate(args.profile), sort_keys=True))
