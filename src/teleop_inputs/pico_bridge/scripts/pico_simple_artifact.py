"""Versioned derived model, separate from measured calibration quality gates."""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml

from pico_palm_tcp_runtime import quaternion_to_matrix, matrix_to_quaternion

SCHEMA = "pico_simple_derived_v1"
MANIFEST = "pico_simple_model.json"
LEFT_TCP = "pico_left_palm_tcp.yaml"
LEFT_WRIST = "pico_left_wrist_pivot.yaml"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def height_lengths(height):
    if type(height) not in (int, float) or not math.isfinite(height) or not 1 <= height <= 2.4:
        raise ValueError("height_m must be within [1,2.4] metres")
    return height * .155882, height * .152941


def payloads(directory, manifest):
    # Lazy import avoids the artifact dispatch import cycle. Sources must remain
    # measured artifacts, never recursively derived copies.
    from pico_calibration_artifact import _validate_tcp, _validate_wrist
    root = Path(directory)
    if (not isinstance(manifest, dict) or type(manifest.get("schema_version")) is not int or manifest.get("schema_version") not in (1, 2) or
            manifest.get("model") != "symmetric_local_y" or
            manifest.get("accepted_model_assumption") is not True):
        raise ValueError("invalid simple model manifest")
    if not isinstance(manifest.get("source_sha256"), dict):
        raise ValueError("simple model source hashes must be a mapping")
    upper, forearm = height_lengths(manifest.get("height_m"))
    height_wrist = manifest["schema_version"] == 2
    sources = (LEFT_TCP,) if height_wrist else (LEFT_TCP, LEFT_WRIST)
    if set(manifest["source_sha256"]) != set(sources):
        raise ValueError("simple model source set mismatch")
    if height_wrist and manifest.get("wrist_model") != "height_times_0.037037_palm_local_positive_x":
        raise ValueError("invalid height wrist model")
    for name in sources:
        path = root / name
        if path.is_symlink() or digest(path) != manifest.get("source_sha256", {}).get(name):
            raise ValueError("simple model source changed: " + name)
    tcp_summary = _validate_tcp(root / LEFT_TCP, "left")
    tcp = yaml.safe_load((root / LEFT_TCP).read_text())
    if height_wrist:
        distance = manifest["height_m"] * .037037
        epoch = tcp.get("orientation_calibration", {}).get("tracking_epoch", tcp_summary.tracking_epoch)
    else:
        wrist_summary = _validate_wrist(root / LEFT_WRIST, "left")
        wrist = yaml.safe_load((root / LEFT_WRIST).read_text())
        distance = wrist.get("wrist_to_palm_distance_m")
        if distance is None:
            v = wrist["wrist_to_palm_m"]
            distance = np.linalg.norm([v[a] for a in "xyz"] if isinstance(v, dict) else v)
        epoch = wrist_summary.tracking_epoch
    m = np.diag([1., -1., 1.])
    t = tcp["translation_m"]
    if isinstance(t, dict):
        t = [t[a] for a in "xyz"]
    q = tcp["quaternion_xyzw"]
    if isinstance(q, dict):
        q = [q[a] for a in "xyzw"]
    if abs(np.linalg.norm(q) - 1) > 1e-3:
        raise ValueError("left TCP quaternion must be unit")
    common = dict(schema_version=SCHEMA, valid=True, calibration_revision=tcp_summary.calibration_revision,
                  tracking_epoch=epoch, source_manifest=MANIFEST)
    output = {}
    output["pico_right_palm_tcp.yaml"] = dict(common, side="right", kind="tcp",
        derivation="mirrored_from_left", pose_semantics="controller_pose",
        transform_convention="T_controller_palm", orientation_calibrated=True,
        orientation_source="mirrored_left_not_independently_measured",
        translation_m=(m @ np.asarray(t)).tolist(),
        quaternion_xyzw=matrix_to_quaternion(m @ quaternion_to_matrix(q) @ m).tolist())
    output["pico_right_wrist_pivot.yaml"] = dict(common, side="right", kind="wrist",
        derivation="shared_left_distance", transform_convention="wrist_to_palm",
        source_frame="pico", source_topic="/pico/palm_right", wrist_to_palm_distance_m=float(distance))
    if height_wrist:
        for side in ("left", "right"):
            output[f"pico_{side}_wrist_pivot.yaml"] = dict(common, side=side, kind="wrist",
                derivation="height_template_not_measured", transform_convention="wrist_to_palm",
                source_frame="pico", source_topic=f"/pico/palm_{side}",
                wrist_to_palm_distance_m=float(distance), height_ratio=.037037,
                axis="palm_local_positive_x")
    for side in ("left", "right"):
        output[f"pico_{side}_arm_geometry.yaml"] = dict(common, side=side, kind="geometry",
            derivation="height_template", template_id="noitom_male170_opensim_spine_v1",
            upper_arm_length_m=upper, forearm_length_m=forearm)
    for name in sources:
        if digest(root / name) != manifest["source_sha256"][name]:
            raise ValueError("simple model source changed while validating")
    return output


def validate_derived(path, kind, side, tcp_path=None, wrist_path=None):
    from pico_calibration_artifact import ArtifactSummary
    from pico_palm_orientation_core import translation_fingerprint
    path = Path(path)
    root = path.parent
    manifest_path = root / MANIFEST
    if path.is_symlink() or manifest_path.is_symlink():
        raise ValueError("simple model artifacts must not be symlinks")
    manifest = json.loads(manifest_path.read_text())
    expected = payloads(root, manifest)
    # Validate the entire derived bundle: no mixture of old and new sides.
    for name, document in expected.items():
        file = root / name
        if file.is_symlink() or yaml.safe_load(file.read_text()) != document:
            raise ValueError("simple model derived payload mismatch: " + name)
    document = expected.get(path.name)
    if document is None or document["kind"] != kind or document["side"] != side:
        raise ValueError("simple model artifact identity mismatch")
    if kind == "geometry":
        for supplied, name in ((tcp_path, f"pico_{side}_palm_tcp.yaml"),
                               (wrist_path, f"pico_{side}_wrist_pivot.yaml")):
            if supplied is None or Path(supplied).absolute() != (root / name).absolute():
                raise ValueError("simple geometry requires exact bundle TCP/wrist paths")
    return ArtifactSummary(True, kind, side, str(path.resolve()), SCHEMA, 1,
        document["calibration_revision"], document["tracking_epoch"], True,
        document["calibration_revision"] if kind == "tcp" else 0,
        translation_fingerprint(document) if kind == "tcp" else "")
