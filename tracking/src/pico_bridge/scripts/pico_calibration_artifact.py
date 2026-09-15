#!/usr/bin/env python3
"""Shared fail-closed validation and activation for PICO calibration artifacts."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import tempfile

import numpy as np
import yaml

from pico_arm_geometry_core import (
    MAX_NEUTRAL_PAIR_POSITION_ERROR_M,
    MAX_STATIC_TRANSITION_DIRECTION_ERROR_RAD,
)
from pico_palm_orientation_core import translation_fingerprint


EXPLICIT_EPOCH_SOURCES = {"tcp_connection", "wire_world_reset"}
EXPECTED_GEOMETRY_COVARIANCE_ORDER = (
    "shoulder_x_m",
    "shoulder_y_m",
    "shoulder_z_m",
    "elbow_x_m",
    "elbow_y_m",
    "elbow_z_m",
    "upper_arm_length_m",
    "forearm_length_m",
)


@dataclass(frozen=True)
class ArtifactSummary:
    valid: bool
    kind: str
    side: str
    path: str
    artifact_type: str
    schema_version: int
    calibration_revision: int
    tracking_epoch: int
    lineage_match: bool
    translation_revision: int = 0
    translation_fingerprint_sha256: str = ""
    orientation_only_ancestor_sha256: tuple[str, ...] = ()


def file_sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).expanduser().open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load(path: str | Path) -> tuple[Path, dict]:
    artifact_path = Path(path).expanduser()
    try:
        document = yaml.safe_load(artifact_path.read_text(encoding="utf-8")) or {}
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f"artifact unreadable: {artifact_path}: {error}") from error
    if not isinstance(document, dict):
        raise ValueError(f"artifact must be a YAML mapping: {artifact_path}")
    return artifact_path, document


def _require_side_and_valid(document: dict, side: str) -> None:
    if side not in {"left", "right"}:
        raise ValueError(f"side must be left or right, got {side!r}")
    if document.get("valid") is not True:
        raise ValueError("valid must be true")
    if document.get("side") != side:
        raise ValueError(f"side mismatch: expected {side}")


def _positive_int(value, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _finite_vector(value, length: int, name: str) -> np.ndarray:
    if isinstance(value, dict):
        axes = "xyzw" if length == 4 else "xyz"
        value = [value.get(axis) for axis in axes]
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must contain numeric values") from error
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {length} finite values")
    return result


def _lineage(document: dict) -> list[str]:
    lineage = document.get("lineage")
    if not isinstance(lineage, list) or not lineage or not all(
        isinstance(value, str) and value for value in lineage
    ):
        raise ValueError("lineage must be a non-empty string list")
    return lineage


def _validate_tcp(path: str | Path, side: str) -> ArtifactSummary:
    artifact_path, document = _load(path)
    _require_side_and_valid(document, side)
    if document.get("schema_version") != 2:
        raise ValueError("TCP schema_version must be 2")
    if document.get("pose_semantics") != "controller_pose":
        raise ValueError("TCP pose_semantics must be controller_pose")
    if document.get("transform_convention") != "T_controller_palm":
        raise ValueError("TCP transform_convention must be T_controller_palm")
    _finite_vector(document.get("translation_m"), 3, "translation_m")
    quaternion = _finite_vector(document.get("quaternion_xyzw"), 4, "quaternion_xyzw")
    if np.linalg.norm(quaternion) <= 1.0e-9:
        raise ValueError("quaternion_xyzw must be non-zero")
    if document.get("orientation_calibrated") is not True:
        raise ValueError("TCP orientation must be calibrated")
    quality = document.get("quality")
    if not isinstance(quality, dict):
        raise ValueError("TCP quality must be a mapping")
    try:
        sample_count = int(quality["sample_count"])
        position_rms = float(quality["position_rms_m"])
        sample_rank = int(quality["sample_matrix_rank"])
        sample_condition = float(quality["sample_matrix_condition"])
    except (KeyError, TypeError, ValueError) as error:
        missing = str(error).strip("'")
        raise ValueError(f"TCP quality {missing} must be numeric") from error
    if sample_count < 4:
        raise ValueError("TCP quality sample_count is below gate")
    if not math.isfinite(position_rms) or position_rms > 0.02:
        raise ValueError("TCP quality position_rms_m exceeds gate")
    if sample_rank != 6:
        raise ValueError("TCP quality sample_matrix_rank must equal 6")
    if not math.isfinite(sample_condition) or sample_condition > 1.0e3:
        raise ValueError("TCP quality sample_matrix_condition exceeds gate")
    if document.get("orientation_reference") == "gravity_leveled_hmd_heading":
        orientation = document.get("orientation_calibration")
        if not isinstance(orientation, dict):
            raise ValueError("TCP orientation_calibration must be a mapping")
        if orientation.get("method") != (
            "gravity_leveled_hmd_heading_bilateral_forward_palms_facing"
        ):
            raise ValueError("TCP orientation_calibration method is invalid")
        _positive_int(
            orientation.get("tracking_epoch"),
            "TCP orientation_calibration tracking_epoch",
        )
        orientation_sample_count = _positive_int(
            orientation.get("sample_count"),
            "TCP orientation_calibration sample_count",
        )
        if orientation_sample_count < 120:
            raise ValueError("TCP orientation_calibration sample_count is below gate")
        try:
            orientation_rms = float(orientation["orientation_rms_rad"])
            correction_angle = float(orientation["correction_angle_rad"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                "TCP orientation_calibration angular quality must be numeric"
            ) from error
        if not math.isfinite(orientation_rms) or orientation_rms > 0.05236:
            raise ValueError(
                "TCP orientation_calibration orientation_rms_rad exceeds gate"
            )
        if not math.isfinite(correction_angle) or correction_angle > 0.7854:
            raise ValueError(
                "TCP orientation_calibration correction_angle_rad exceeds gate"
            )
        covariance_values = _finite_vector(
            document.get("covariance_upper_triangle_6x6"),
            21,
            "covariance_upper_triangle_6x6",
        )
        covariance = np.zeros((6, 6))
        cursor = 0
        for row in range(6):
            for column in range(row, 6):
                covariance[row, column] = covariance_values[cursor]
                covariance[column, row] = covariance_values[cursor]
                cursor += 1
        if float(np.min(np.linalg.eigvalsh(covariance))) < -1.0e-10:
            raise ValueError("TCP covariance_upper_triangle_6x6 must be PSD")
    revision = _positive_int(document.get("calibration_revision"), "calibration_revision")
    translation_revision = _positive_int(
        document.get("translation_revision", revision), "translation_revision"
    )
    current_translation_fingerprint = translation_fingerprint(document)
    stored_translation_fingerprint = document.get("translation_fingerprint_sha256")
    if (
        stored_translation_fingerprint is not None
        and stored_translation_fingerprint != current_translation_fingerprint
    ):
        raise ValueError("TCP translation fingerprint mismatch")
    ancestors = document.get("orientation_only_ancestor_sha256", [])
    if not isinstance(ancestors, list) or not all(
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value.lower())
        for value in ancestors
    ):
        raise ValueError("orientation_only_ancestor_sha256 must contain SHA-256 values")
    _lineage(document)
    return ArtifactSummary(
        True,
        "tcp",
        side,
        str(artifact_path.resolve()),
        str(document.get("artifact_type", "pico_palm_tcp_v2")),
        2,
        revision,
        0,
        True,
        translation_revision,
        current_translation_fingerprint,
        tuple(ancestors),
    )


def _validate_wrist(path: str | Path, side: str) -> ArtifactSummary:
    artifact_path, document = _load(path)
    _require_side_and_valid(document, side)
    if document.get("schema_version") != 1:
        raise ValueError("wrist schema_version must be 1")
    if document.get("transform_convention") != "wrist_to_palm":
        raise ValueError("wrist transform_convention must be wrist_to_palm")
    if document.get("source_frame") != "pico":
        raise ValueError("wrist source_frame must be pico")
    expected_source_topic = f"/pico/palm_{side}"
    source_topic = document.get("source_topic")
    if source_topic is not None and source_topic != expected_source_topic:
        raise ValueError(f"wrist source_topic must be {expected_source_topic}")
    lineage_match = source_topic == expected_source_topic
    if document.get("wrist_to_palm_distance_m") is not None:
        try:
            distance = float(document["wrist_to_palm_distance_m"])
        except (TypeError, ValueError) as error:
            raise ValueError("wrist_to_palm_distance_m must be numeric") from error
        if not math.isfinite(distance) or not 0.01 <= distance <= 0.16:
            raise ValueError(
                "wrist_to_palm_distance_m must be within [0.01, 0.16] m"
            )
    else:
        vector = _finite_vector(document.get("wrist_to_palm_m"), 3, "wrist_to_palm_m")
        distance = float(np.linalg.norm(vector))
        if not 0.01 <= distance <= 0.16:
            raise ValueError("wrist-to-palm distance must be within [0.01, 0.16] m")
        if vector[0] <= 0.0:
            raise ValueError("wrist-to-palm local +X component must exceed 0.0 m")
    epoch = _positive_int(document.get("tracking_epoch"), "tracking_epoch")
    if document.get("tracking_epoch_source") not in EXPLICIT_EPOCH_SOURCES:
        raise ValueError("wrist tracking_epoch_source is invalid")
    quality = document.get("quality")
    if not isinstance(quality, dict):
        raise ValueError("wrist quality must be a mapping")
    gates = {
        "sample_count": (60.0, None),
        "position_rms_m": (None, 0.015),
        "condition_number": (None, 1.0e4),
    }
    for name, (minimum, maximum) in gates.items():
        try:
            value = float(quality[name])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"wrist quality {name} must be numeric") from error
        if not math.isfinite(value):
            raise ValueError(f"wrist quality {name} must be finite")
        if minimum is not None and value < minimum:
            raise ValueError(f"wrist quality {name} is below gate")
        if maximum is not None and value > maximum:
            raise ValueError(f"wrist quality {name} exceeds gate")
    return ArtifactSummary(
        True,
        "wrist",
        side,
        str(artifact_path.resolve()),
        "pico_palm_wrist_pivot_v1",
        1,
        int(document.get("calibration_revision", 0)),
        epoch,
        lineage_match,
    )


def _validate_geometry(
    path: str | Path,
    side: str,
    tcp_path: str | Path | None,
    wrist_path: str | Path | None,
    expected_tcp_revision: int | None,
    expected_tcp_sha256: str | None,
    expected_wrist_pivot_sha256: str | None,
) -> ArtifactSummary:
    tcp_summary: ArtifactSummary | None = None
    if tcp_path is not None and wrist_path is not None:
        tcp_summary = _validate_tcp(tcp_path, side)
        _validate_wrist(wrist_path, side)
        expected_tcp_revision = tcp_summary.calibration_revision
        expected_tcp_sha256 = file_sha256(tcp_path)
        expected_wrist_pivot_sha256 = file_sha256(wrist_path)
    elif (
        expected_tcp_revision is None
        or expected_tcp_sha256 is None
        or expected_wrist_pivot_sha256 is None
    ):
        raise ValueError(
            "geometry validation requires artifact paths or exact expected lineage"
        )
    artifact_path, document = _load(path)
    _require_side_and_valid(document, side)
    expected_type = f"pico_{side}_arm_geometry_quick_v3"
    if document.get("artifact_type") != expected_type:
        raise ValueError(f"geometry artifact_type must be {expected_type}")
    if document.get("schema_version") != 3:
        raise ValueError("geometry schema_version must be 3")
    if document.get("candidate_status") != "accepted":
        raise ValueError("geometry candidate_status must be accepted")
    if document.get("rejection_reasons") != []:
        raise ValueError("geometry rejection_reasons must be empty")
    revision = _positive_int(document.get("calibration_revision"), "calibration_revision")
    epoch = _positive_int(document.get("tracking_epoch"), "tracking_epoch")
    if document.get("tracking_epoch_source") not in EXPLICIT_EPOCH_SOURCES:
        raise ValueError("geometry tracking_epoch_source is invalid")
    _lineage(document)
    try:
        upper = float(document["upper_arm_length_m"])
        forearm = float(document["forearm_length_m"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("geometry lengths must be numeric") from error
    if not math.isfinite(upper) or not 0.15 <= upper <= 0.45:
        raise ValueError("upper_arm_length_m is outside [0.15, 0.45]")
    if not math.isfinite(forearm) or not 0.15 <= forearm <= 0.40:
        raise ValueError("forearm_length_m is outside [0.15, 0.40]")

    quality = document.get("quality")
    if not isinstance(quality, dict):
        raise ValueError("geometry quality must be a mapping")
    if quality.get("right_angle_error_role") != "diagnostic_only":
        raise ValueError("right_angle_error_role must be diagnostic_only")
    raw_available = quality.get("raw_smpl_diagnostic_available")
    if not isinstance(raw_available, bool):
        raise ValueError("raw_smpl_diagnostic_available must be boolean")
    raw_error = quality.get("right_angle_error_rad")
    if raw_available:
        try:
            raw_error_value = float(raw_error)
        except (TypeError, ValueError) as error:
            raise ValueError("right_angle_error_rad must be numeric") from error
        if not math.isfinite(raw_error_value) or raw_error_value < 0.0:
            raise ValueError("right_angle_error_rad must be finite and non-negative")
    elif raw_error is not None:
        raise ValueError("unavailable raw diagnostic must use null right_angle_error_rad")

    selected = quality.get("selected_straight_groups")
    valid_groups = {"straight_1", "straight_2", "validation"}
    if (
        not isinstance(selected, list)
        or len(selected) != 2
        or len(set(selected)) != 2
        or not set(selected) <= valid_groups
    ):
        raise ValueError("selected_straight_groups is invalid")
    discarded = quality.get("discarded_straight_group")
    if (
        discarded not in valid_groups
        or discarded in selected
        or set(selected) | {discarded} != valid_groups
    ):
        raise ValueError("discarded_straight_group is invalid")
    quality_gates = {
        "length_std_m": (None, 0.030),
        "straight_pair_position_error_m": (None, 0.10),
        "upper_repeat_error_m": (None, 0.06),
        "neutral_pair_position_error_m": (
            None,
            MAX_NEUTRAL_PAIR_POSITION_ERROR_M,
        ),
        "forearm_repeat_error_m": (None, 0.04),
        "static_motion_rms_m": (None, 0.015),
        "transition_direction_error_rad": (
            None,
            MAX_STATIC_TRANSITION_DIRECTION_ERROR_RAD,
        ),
        "length_closure_error_m": (None, 0.025),
        "straight_1_sample_count": (50.0, None),
        "straight_2_sample_count": (50.0, None),
        "validation_sample_count": (50.0, None),
        "neutral_start_sample_count": (50.0, None),
        "neutral_return_sample_count": (50.0, None),
        "right_angle_sample_count": (50.0, None),
    }
    for name, (minimum, maximum) in quality_gates.items():
        try:
            value = float(quality[name])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"geometry quality {name} must be numeric") from error
        if not math.isfinite(value):
            raise ValueError(f"geometry quality {name} must be finite")
        if minimum is not None and value < minimum:
            raise ValueError(f"geometry quality {name} is below gate")
        if maximum is not None and value > maximum:
            raise ValueError(f"geometry quality {name} exceeds gate")
    shoulder_motion = quality.get("shoulder_motion_rms_m")
    try:
        shoulder_motion = float(shoulder_motion)
    except (TypeError, ValueError) as error:
        raise ValueError("shoulder_motion_rms_m must be numeric") from error
    if not math.isfinite(shoulder_motion) or shoulder_motion < 0.0:
        raise ValueError("shoulder_motion_rms_m must be finite and non-negative")

    gate_results = document.get("gate_results")
    if not isinstance(gate_results, dict) or not gate_results or not all(
        value is True for value in gate_results.values()
    ):
        raise ValueError("geometry gate_results must all be true")
    if document.get("covariance_tangent_order") != list(
        EXPECTED_GEOMETRY_COVARIANCE_ORDER
    ):
        raise ValueError("covariance_tangent_order is invalid")
    try:
        covariance_values = np.asarray(
            document.get("covariance_upper_triangle_8x8"), dtype=float
        )
    except (TypeError, ValueError) as error:
        raise ValueError("geometry covariance must be numeric") from error
    if covariance_values.shape != (36,) or not np.all(np.isfinite(covariance_values)):
        raise ValueError("geometry covariance must contain 36 finite values")
    covariance = np.zeros((8, 8), dtype=float)
    cursor = 0
    for row in range(8):
        for column in range(row, 8):
            covariance[row, column] = covariance_values[cursor]
            covariance[column, row] = covariance_values[cursor]
            cursor += 1
    if np.linalg.eigvalsh(covariance).min() < -1.0e-10:
        raise ValueError("geometry covariance must be PSD")

    semantic_revision = document.get("tcp_translation_revision")
    semantic_fingerprint = document.get("tcp_translation_fingerprint_sha256")
    uses_semantic_lineage = (
        semantic_revision is not None or semantic_fingerprint is not None
    )
    if uses_semantic_lineage:
        if semantic_revision is None or semantic_fingerprint is None:
            raise ValueError("geometry TCP translation lineage is incomplete")
        if tcp_summary is None:
            raise ValueError("semantic geometry validation requires TCP artifact path")
        if semantic_revision != tcp_summary.translation_revision:
            raise ValueError("geometry TCP translation revision mismatch")
        if semantic_fingerprint != tcp_summary.translation_fingerprint_sha256:
            raise ValueError("geometry TCP translation fingerprint mismatch")
    else:
        legacy_revision_matches = document.get("tcp_calibration_revision") == (
            tcp_summary.translation_revision if tcp_summary else expected_tcp_revision
        )
        if not legacy_revision_matches:
            raise ValueError("geometry TCP revision mismatch")
        legacy_hash = document.get("tcp_artifact_sha256")
        accepted_hashes = {expected_tcp_sha256}
        if tcp_summary is not None:
            accepted_hashes.update(tcp_summary.orientation_only_ancestor_sha256)
        if legacy_hash not in accepted_hashes:
            raise ValueError("geometry TCP artifact hash mismatch")
    if document.get("wrist_pivot_sha256") != expected_wrist_pivot_sha256:
        raise ValueError("geometry wrist pivot hash mismatch")
    return ArtifactSummary(
        True,
        "geometry",
        side,
        str(artifact_path.resolve()),
        expected_type,
        3,
        revision,
        epoch,
        True,
    )


def validate_artifact(
    path: str | Path,
    kind: str,
    side: str,
    *,
    tcp_path: str | Path | None = None,
    wrist_path: str | Path | None = None,
    expected_tcp_revision: int | None = None,
    expected_tcp_sha256: str | None = None,
    expected_wrist_pivot_sha256: str | None = None,
) -> ArtifactSummary:
    if kind == "tcp":
        return _validate_tcp(path, side)
    if kind == "wrist":
        return _validate_wrist(path, side)
    if kind == "geometry":
        return _validate_geometry(
            path,
            side,
            tcp_path,
            wrist_path,
            expected_tcp_revision,
            expected_tcp_sha256,
            expected_wrist_pivot_sha256,
        )
    raise ValueError(f"unknown artifact kind: {kind}")


def activate_artifact(
    candidate: str | Path,
    active: str | Path,
    kind: str,
    side: str,
    *,
    tcp_path: str | Path | None = None,
    wrist_path: str | Path | None = None,
) -> ArtifactSummary:
    summary = validate_artifact(
        candidate, kind, side, tcp_path=tcp_path, wrist_path=wrist_path
    )
    candidate_path = Path(candidate).expanduser()
    active_path = Path(active).expanduser()
    active_path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{active_path.name}.tmp.", dir=active_path.parent
    )
    try:
        with os.fdopen(descriptor, "wb") as output, candidate_path.open("rb") as source:
            shutil.copyfileobj(source, output)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary_name, active_path)
        directory_fd = os.open(active_path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        Path(temporary_name).unlink(missing_ok=True)
    return summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("validate", "activate"))
    parser.add_argument("--path", required=True)
    parser.add_argument("--kind", choices=("tcp", "wrist", "geometry"), required=True)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--tcp-path")
    parser.add_argument("--wrist-path")
    parser.add_argument("--active")
    arguments = parser.parse_args()
    if arguments.command == "activate":
        if not arguments.active:
            raise SystemExit("--active is required for activation")
        summary = activate_artifact(
            arguments.path,
            arguments.active,
            arguments.kind,
            arguments.side,
            tcp_path=arguments.tcp_path,
            wrist_path=arguments.wrist_path,
        )
    else:
        summary = validate_artifact(
            arguments.path,
            arguments.kind,
            arguments.side,
            tcp_path=arguments.tcp_path,
            wrist_path=arguments.wrist_path,
        )
    print(json.dumps(asdict(summary), sort_keys=True))


if __name__ == "__main__":
    main()
