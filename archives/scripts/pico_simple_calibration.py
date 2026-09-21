"""Offline candidate only: left TCP mirror + shared wrist distance + height model.

Does not publish profiles, emit legacy measured artifacts or access devices.
The caller must explicitly choose a model or supply both LOCAL-frame reflection
matrices. A model convention is not proof of the hardware's mirror convention.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tracking/src/pico_bridge/scripts"))
from pico_calibration_artifact import validate_artifact
from pico_palm_tcp_runtime import quaternion_to_matrix, matrix_to_quaternion

TEMPLATE = "noitom_male170_opensim_spine_v1"
UPPER_RATIO = 0.155882
FOREARM_RATIO = 0.152941


def height_lengths(height_m):
    if isinstance(height_m, bool) or not isinstance(height_m, (float, int)):
        raise ValueError("height_m must be a number in metres")
    if not math.isfinite(height_m) or not 1.0 <= height_m <= 2.4:
        raise ValueError("height_m must be within [1.0, 2.4]; do not supply cm")
    return {"upper_arm_length_m": height_m * UPPER_RATIO,
            "forearm_length_m": height_m * FOREARM_RATIO}


def reflection(value):
    m = np.asarray(value, dtype=float)
    if m.shape != (3, 3) or not np.isfinite(m).all():
        raise ValueError("mirror must be a finite 3x3 plane reflection")
    if not (np.allclose(m.T @ m, np.eye(3), atol=1e-9, rtol=0)
            and np.allclose(m @ m, np.eye(3), atol=1e-9, rtol=0)
            and abs(np.linalg.det(m) + 1) < 1e-9
            and abs(np.trace(m) - 1) < 1e-9):
        raise ValueError("mirror must be an orthogonal plane reflection, not a rotation")
    return m


def mirror_tcp(translation, quaternion, controller_mirror, palm_mirror):
    mc, mp = reflection(controller_mirror), reflection(palm_mirror)
    t, q = np.asarray(translation, dtype=float), np.asarray(quaternion, dtype=float)
    if t.shape != (3,) or not np.isfinite(t).all():
        raise ValueError("translation must be a finite 3-vector")
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 1e-3:
        raise ValueError("quaternion must be unit xyzw")
    return (mc @ t).tolist(), matrix_to_quaternion(mc @ quaternion_to_matrix(q) @ mp).tolist()


def load_mirror_contract(mirror_contract=None, mirror_model=None):
    if (mirror_contract is None) == (mirror_model is None):
        raise ValueError("choose exactly one mirror contract or mirror model")
    if mirror_model is not None:
        if mirror_model != "symmetric_local_y":
            raise ValueError("unknown mirror model")
        return {
            "schema_version": 1,
            "controller_mirror": np.diag([1, -1, 1]).tolist(),
            "palm_mirror": np.diag([1, -1, 1]).tolist(),
            "evidence": "Explicit symmetric_local_y model assumption: corresponding local "
                        "Y axes are lateral; not fitted to existing right TCP calibration "
                        "and not hardware-verified",
        }
    contract = json.loads(Path(mirror_contract).read_text())
    if not isinstance(contract, dict) or set(contract) != {
        "schema_version", "controller_mirror", "palm_mirror", "evidence"
    } or type(contract["schema_version"]) is not int or contract["schema_version"] != 1:
        raise ValueError("invalid mirror contract fields/version")
    if not isinstance(contract["evidence"], str) or not contract["evidence"].strip():
        raise ValueError("mirror contract requires an evidence description")
    return contract


def candidate(left_tcp, left_wrist, height_m, mirror_contract=None, *, mirror_model=None):
    lengths = height_lengths(height_m)
    contract = load_mirror_contract(mirror_contract, mirror_model)
    paths = {"left_tcp": Path(left_tcp), "left_wrist": Path(left_wrist)}
    before = {name: path.read_bytes() for name, path in paths.items()}
    # Reuse the original strict measured gates without weakening them.
    validate_artifact(paths["left_tcp"], "tcp", "left")
    validate_artifact(paths["left_wrist"], "wrist", "left")
    docs = {name: yaml.safe_load(data) for name, data in before.items()}
    tcp, wrist = docs["left_tcp"], docs["left_wrist"]
    t, q = mirror_tcp(tcp["translation_m"], tcp["quaternion_xyzw"],
                      contract["controller_mirror"], contract["palm_mirror"])
    if "wrist_to_palm_distance_m" in wrist:
        distance = float(wrist["wrist_to_palm_distance_m"])
    else:
        v = wrist["wrist_to_palm_m"]
        if isinstance(v, dict):
            v = [v[axis] for axis in "xyz"]
        distance = float(np.linalg.norm(v))
    if any(path.read_bytes() != before[name] for name, path in paths.items()):
        raise ValueError("calibration changed while building candidate")
    return {
        "schema_version": "pico_simple_candidate_v1",
        "candidate_status": "review_required",
        "runtime_eligible": False,
        "motion_authorized": False,
        "limitations": ["local mirror convention requires engineering verification",
                        "bilateral grip symmetry requires device validation",
                        "legacy wrist artifact may lack binding to current TCP",
                        "height template is not measured personal anatomy"],
        "source_sha256": {name: hashlib.sha256(data).hexdigest() for name, data in before.items()},
        "mirror_contract": contract,
        "mirror_model": mirror_model or "explicit_contract",
        "body": {"source": "height_template", "template_id": TEMPLATE,
                 "height_m": height_m, "ratios": {"upper_arm": UPPER_RATIO, "forearm": FOREARM_RATIO},
                 "left": lengths.copy(), "right": lengths.copy()},
        "wrist": {"source": "left_measured_shared_scalar", "axis": "palm_local_positive_x",
                  "left_distance_m": distance, "right_distance_m": distance},
        "tcp": {
            "transform_convention": "T_controller_palm",
            "left": {"source": "measured", "translation_m": tcp["translation_m"],
                     "quaternion_xyzw": tcp["quaternion_xyzw"]},
            "right": {"source": "mirrored_from_left", "translation_m": t, "quaternion_xyzw": q},
        },
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left-tcp", type=Path, required=True)
    parser.add_argument("--left-wrist", type=Path, required=True)
    parser.add_argument("--height-m", type=float, required=True)
    mirror = parser.add_mutually_exclusive_group(required=True)
    mirror.add_argument("--mirror-contract", type=Path)
    mirror.add_argument("--mirror-model", choices=["symmetric_local_y"],
                        help="explicit symmetric model; does not establish hardware axis validity")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        doc = candidate(args.left_tcp, args.left_wrist, args.height_m, args.mirror_contract,
                        mirror_model=args.mirror_model)
        payload = json.dumps(doc, indent=2, allow_nan=False) + "\n"
        # Exclusive creation, including when --output aliases a measured input.
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(payload)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
        parser.exit(2, str(error) + "\n")
    print(f"Candidate only (not published): {args.output}")


if __name__ == "__main__":
    main()
