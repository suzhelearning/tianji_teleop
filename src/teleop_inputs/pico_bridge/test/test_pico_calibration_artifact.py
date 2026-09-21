import hashlib
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml


sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))

from pico_calibration_artifact import (  # noqa: E402
    EXPECTED_GEOMETRY_COVARIANCE_ORDER,
    activate_artifact,
    validate_artifact,
)
from pico_palm_orientation_core import (  # noqa: E402
    OrientationSolution,
    build_orientation_only_update,
    rotation_exp,
    translation_fingerprint,
)


def _write(path: Path, document: dict) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return path


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _chain(tmp_path: Path):
    tcp = _write(
        tmp_path / "tcp.yaml",
        {
            "artifact_type": "pico_palm_tcp_v2",
            "schema_version": 2,
            "valid": True,
            "side": "left",
            "pose_semantics": "controller_pose",
            "transform_convention": "T_controller_palm",
            "translation_m": [0.01, 0.0, 0.0],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            "orientation_calibrated": True,
            "calibration_revision": 4,
            "source_topic": "/pico/pose/left_hand",
            "lineage": [
                "/pico/pose/left_hand",
                "/pico/pose/head",
                "hmd_relative_known_palm_pose",
            ],
            "quality": {
                "sample_count": 4,
                "position_rms_m": 0.003,
                "sample_matrix_rank": 6,
                "sample_matrix_condition": 12.0,
            },
        },
    )
    wrist = _write(
        tmp_path / "wrist.yaml",
        {
            "schema_version": 1,
            "valid": True,
            "side": "left",
            "transform_convention": "wrist_to_palm",
            "source_topic": "/pico/palm_left",
            "source_frame": "pico",
            "tracking_epoch": 7,
            "tracking_epoch_source": "tcp_connection",
            "wrist_to_palm_m": {"x": 0.08, "y": 0.0, "z": 0.0},
            "quality": {
                "sample_count": 60,
                "position_rms_m": 0.002,
                "condition_number": 20.0,
            },
        },
    )
    covariance = np.eye(8) * 1.0e-4
    geometry = _write(
        tmp_path / "geometry.yaml",
        {
            "artifact_type": "pico_left_arm_geometry_quick_v3",
            "schema_version": 3,
            "valid": True,
            "candidate_status": "accepted",
            "side": "left",
            "calibration_revision": 9,
            "upper_arm_length_m": 0.30,
            "forearm_length_m": 0.25,
            "covariance_tangent_order": list(EXPECTED_GEOMETRY_COVARIANCE_ORDER),
            "covariance_upper_triangle_8x8": [
                float(covariance[row, column])
                for row in range(8)
                for column in range(row, 8)
            ],
            "quality": {
                "length_std_m": 0.005,
                "selected_straight_groups": ["straight_1", "validation"],
                "discarded_straight_group": "straight_2",
                "straight_pair_position_error_m": 0.01,
                "upper_repeat_error_m": 0.01,
                "neutral_pair_position_error_m": 0.01,
                "forearm_repeat_error_m": 0.01,
                "static_motion_rms_m": 0.002,
                "right_angle_error_rad": None,
                "right_angle_error_role": "diagnostic_only",
                "raw_smpl_diagnostic_available": False,
                "transition_direction_error_rad": 0.05,
                "length_closure_error_m": 0.003,
                "shoulder_motion_rms_m": 0.20,
                "straight_1_sample_count": 80,
                "straight_2_sample_count": 80,
                "validation_sample_count": 80,
                "neutral_start_sample_count": 80,
                "neutral_return_sample_count": 80,
                "right_angle_sample_count": 80,
            },
            "gate_results": {"palm_geometry": True},
            "tcp_calibration_revision": 4,
            "tcp_artifact_sha256": _sha256(tcp),
            "wrist_pivot_sha256": _sha256(wrist),
            "tracking_epoch": 7,
            "tracking_epoch_source": "tcp_connection",
            "rejection_reasons": [],
            "lineage": [
                "/pico/smpl_raw",
                "/pico/palm_left",
                "palm_local_positive_x_wrist",
            ],
        },
    )
    return tcp, wrist, geometry


def test_shared_contract_accepts_diagnostic_shoulder_motion_and_reports_lineage(tmp_path):
    tcp, wrist, geometry = _chain(tmp_path)

    summary = validate_artifact(
        geometry, "geometry", "left", tcp_path=tcp, wrist_path=wrist
    )

    assert summary.valid
    assert summary.schema_version == 3
    assert summary.calibration_revision == 9
    assert summary.lineage_match is True


def test_geometry_contract_accepts_29_degrees_but_rejects_above_30(tmp_path):
    tcp, wrist, geometry = _chain(tmp_path)
    document = yaml.safe_load(geometry.read_text(encoding="utf-8"))
    document["quality"]["transition_direction_error_rad"] = float(np.deg2rad(29.0))
    _write(geometry, document)

    assert validate_artifact(
        geometry, "geometry", "left", tcp_path=tcp, wrist_path=wrist
    ).valid

    document["quality"]["transition_direction_error_rad"] = float(np.deg2rad(30.1))
    _write(geometry, document)
    with pytest.raises(ValueError, match="transition_direction_error_rad"):
        validate_artifact(
            geometry, "geometry", "left", tcp_path=tcp, wrist_path=wrist
        )


def test_geometry_contract_accepts_65_mm_neutral_pair_but_rejects_above_70_mm(
    tmp_path,
):
    tcp, wrist, geometry = _chain(tmp_path)
    document = yaml.safe_load(geometry.read_text(encoding="utf-8"))
    document["quality"]["neutral_pair_position_error_m"] = 0.065
    _write(geometry, document)

    assert validate_artifact(
        geometry, "geometry", "left", tcp_path=tcp, wrist_path=wrist
    ).valid

    document["quality"]["neutral_pair_position_error_m"] = 0.071
    _write(geometry, document)
    with pytest.raises(ValueError, match="neutral_pair_position_error_m"):
        validate_artifact(
            geometry, "geometry", "left", tcp_path=tcp, wrist_path=wrist
        )


@pytest.mark.parametrize("field", ["covariance_tangent_order", "lineage"])
def test_shared_contract_rejects_missing_geometry_semantics(tmp_path, field):
    tcp, wrist, geometry = _chain(tmp_path)
    document = yaml.safe_load(geometry.read_text(encoding="utf-8"))
    document.pop(field)
    _write(geometry, document)

    with pytest.raises(ValueError, match=field):
        validate_artifact(
            geometry, "geometry", "left", tcp_path=tcp, wrist_path=wrist
        )


def test_invalid_candidate_never_replaces_active_artifact(tmp_path):
    tcp, wrist, candidate = _chain(tmp_path)
    active = _write(tmp_path / "active.yaml", {"old": "preserved"})
    document = yaml.safe_load(candidate.read_text(encoding="utf-8"))
    document["covariance_upper_triangle_8x8"][0] = float("nan")
    _write(candidate, document)

    with pytest.raises(ValueError, match="covariance"):
        activate_artifact(
            candidate,
            active,
            "geometry",
            "left",
            tcp_path=tcp,
            wrist_path=wrist,
        )

    assert yaml.safe_load(active.read_text(encoding="utf-8")) == {"old": "preserved"}


def test_wrist_contract_rejects_cross_side_source_topic(tmp_path):
    _, wrist, _ = _chain(tmp_path)
    document = yaml.safe_load(wrist.read_text(encoding="utf-8"))
    document["source_topic"] = "/pico/palm_right"
    _write(wrist, document)

    with pytest.raises(ValueError, match="source_topic"):
        validate_artifact(wrist, "wrist", "left")


def test_wrist_contract_rejects_implausibly_short_distance(tmp_path):
    _, wrist, _ = _chain(tmp_path)
    document = yaml.safe_load(wrist.read_text(encoding="utf-8"))
    document["wrist_to_palm_m"] = {"x": 0.001, "y": 0.0, "z": 0.0}
    _write(wrist, document)

    with pytest.raises(ValueError, match="distance"):
        validate_artifact(wrist, "wrist", "left")


def test_wrist_contract_accepts_one_centimeter_distance(tmp_path):
    _, wrist, _ = _chain(tmp_path)
    document = yaml.safe_load(wrist.read_text(encoding="utf-8"))
    document["wrist_to_palm_m"] = {"x": 0.01, "y": 0.0, "z": 0.0}
    _write(wrist, document)

    assert validate_artifact(wrist, "wrist", "left").valid


def test_wrist_contract_rejects_zero_local_x(tmp_path):
    _, wrist, _ = _chain(tmp_path)
    document = yaml.safe_load(wrist.read_text(encoding="utf-8"))
    document["wrist_to_palm_m"] = {"x": 0.0, "y": 0.02, "z": 0.0}
    _write(wrist, document)

    with pytest.raises(ValueError, match=r"local \+X"):
        validate_artifact(wrist, "wrist", "left")


def test_tcp_contract_requires_excitation_evidence(tmp_path):
    tcp, _, _ = _chain(tmp_path)
    document = yaml.safe_load(tcp.read_text(encoding="utf-8"))
    document["quality"].pop("sample_matrix_rank")
    _write(tcp, document)

    with pytest.raises(ValueError, match="sample_matrix_rank"):
        validate_artifact(tcp, "tcp", "left")


def test_gravity_leveled_tcp_contract_requires_multi_frame_orientation_evidence(tmp_path):
    tcp, _, _ = _chain(tmp_path)
    updated = _orientation_only_update(tcp)
    updated["orientation_calibration"].pop("tracking_epoch")
    _write(tcp, updated)

    with pytest.raises(ValueError, match="tracking_epoch"):
        validate_artifact(tcp, "tcp", "left")


def test_gravity_leveled_tcp_contract_rejects_orientation_rms_above_gate(tmp_path):
    tcp, _, _ = _chain(tmp_path)
    updated = _orientation_only_update(tcp)
    updated["orientation_calibration"]["orientation_rms_rad"] = 0.06
    _write(tcp, updated)

    with pytest.raises(ValueError, match="orientation_rms_rad"):
        validate_artifact(tcp, "tcp", "left")


def _orientation_only_update(tcp: Path) -> dict:
    document = yaml.safe_load(tcp.read_text(encoding="utf-8"))
    solution = OrientationSolution(
        rotation=rotation_exp(np.array([0.0, 0.0, 0.1])),
        covariance=np.eye(3) * 1.0e-5,
        sample_count=120,
        tracking_epoch=7,
        orientation_rms_rad=0.01,
        correction_angle_rad=0.1,
    )
    return build_orientation_only_update(document, solution, _sha256(tcp))


def test_legacy_geometry_survives_controlled_orientation_only_tcp_update(tmp_path):
    tcp, wrist, geometry = _chain(tmp_path)
    _write(tcp, _orientation_only_update(tcp))

    summary = validate_artifact(
        geometry, "geometry", "left", tcp_path=tcp, wrist_path=wrist
    )

    assert summary.valid


def test_semantic_geometry_survives_orientation_only_tcp_update(tmp_path):
    tcp, wrist, geometry = _chain(tmp_path)
    document = yaml.safe_load(geometry.read_text(encoding="utf-8"))
    tcp_document = yaml.safe_load(tcp.read_text(encoding="utf-8"))
    document["tcp_translation_revision"] = 4
    document["tcp_translation_fingerprint_sha256"] = translation_fingerprint(
        tcp_document
    )
    _write(geometry, document)
    _write(tcp, _orientation_only_update(tcp))

    assert validate_artifact(
        geometry, "geometry", "left", tcp_path=tcp, wrist_path=wrist
    ).valid


def test_tcp_contract_rejects_translation_changed_after_orientation_update(tmp_path):
    tcp, _, _ = _chain(tmp_path)
    updated = _orientation_only_update(tcp)
    updated["translation_m"][0] += 0.01
    _write(tcp, updated)

    with pytest.raises(ValueError, match="translation fingerprint"):
        validate_artifact(tcp, "tcp", "left")


def test_legacy_geometry_rejects_unrelated_tcp_hash(tmp_path):
    tcp, wrist, geometry = _chain(tmp_path)
    updated = _orientation_only_update(tcp)
    updated["orientation_only_ancestor_sha256"] = ["0" * 64]
    _write(tcp, updated)

    with pytest.raises(ValueError, match="TCP artifact hash"):
        validate_artifact(
            geometry, "geometry", "left", tcp_path=tcp, wrist_path=wrist
        )
