import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml


sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))

from pico_palm_orientation_core import (  # noqa: E402
    OrientationCaptureBuffer,
    OrientationGates,
    OrientationSample,
    atomic_replace_if_unchanged,
    build_orientation_only_update,
    gravity_leveled_heading_rotation,
    matrix_to_quaternion_xyzw,
    rotation_exp,
    solve_orientation,
    translation_fingerprint,
)


def _rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def test_gravity_leveled_heading_keeps_yaw_and_removes_head_tilt():
    leveled = gravity_leveled_heading_rotation(_rpy(0.12, -0.09, 0.37))

    np.testing.assert_allclose(leveled, _rpy(0.0, 0.0, 0.37), atol=1e-9)
    np.testing.assert_allclose(leveled[:, 2], [0.0, 0.0, 1.0], atol=1e-12)


def test_gravity_leveled_heading_rejects_vertical_head_forward_axis():
    with pytest.raises(ValueError, match="heading"):
        gravity_leveled_heading_rotation(_rpy(0.0, -np.pi / 2.0, 0.0))


def test_orientation_solver_does_not_copy_hmd_pitch_or_roll():
    head = _rpy(0.14, -0.11, 0.31)
    samples = [_sample(head, stamp=index) for index in range(120)]

    solution = solve_orientation(samples, np.eye(3), OrientationGates())

    np.testing.assert_allclose(solution.rotation, _rpy(0.0, 0.0, 0.31), atol=1e-9)


def test_capture_buffer_pairs_monotonic_samples_and_rejects_epoch_change():
    capture = OrientationCaptureBuffer(max_pair_skew_ns=30_000_000)
    capture.begin(5, "tcp_connection")
    capture.add_head(np.eye(3), 100_000_000)
    assert capture.add_controller(np.eye(3), 110_000_000, 5)
    assert not capture.add_controller(np.eye(3), 110_000_000, 5)
    capture.update_epoch(6, "wire_world_reset")

    assert len(capture.samples) == 1
    assert capture.error == "tracking_epoch_changed"


def test_capture_buffer_requires_explicit_epoch_source_and_pair_skew():
    capture = OrientationCaptureBuffer(max_pair_skew_ns=30_000_000)
    with pytest.raises(ValueError, match="explicit"):
        capture.begin(5, "unknown")
    capture.begin(5, "wire_world_reset")
    capture.add_head(np.eye(3), 100_000_000)

    assert not capture.add_controller(np.eye(3), 140_000_001, 5)
    assert capture.samples == ()


def _sample(target, controller=None, epoch=7, stamp=0):
    controller = np.eye(3) if controller is None else controller
    return OrientationSample(
        controller_rotation=controller,
        head_rotation=controller @ target,
        stamp_ns=stamp,
        tracking_epoch=epoch,
    )


def _tcp_document():
    return {
        "schema_version": 2,
        "valid": True,
        "side": "left",
        "pose_semantics": "controller_pose",
        "transform_convention": "T_controller_palm",
        "source_topic": "/pico/pose/left_hand",
        "orientation_reference": "hmd_relative_known_palm_pose",
        "translation_m": [0.05, -0.02, 0.08],
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        "covariance_upper_triangle_6x6": [0.0] * 21,
        "orientation_calibrated": True,
        "calibration_revision": 4,
        "lineage": ["/pico/pose/left_hand", "/pico/pose/head"],
        "quality": {
            "sample_count": 4,
            "position_rms_m": 0.004,
            "sample_matrix_rank": 6,
            "sample_matrix_condition": 12.0,
        },
    }


def test_solver_recovers_gravity_leveled_controller_to_palm_rotation():
    target = rotation_exp(np.array([0.0, 0.0, 0.12]))
    samples = [_sample(target, stamp=index) for index in range(120)]

    solution = solve_orientation(samples, np.eye(3), OrientationGates())

    np.testing.assert_allclose(solution.rotation, target, atol=1e-9)
    assert solution.sample_count == 120
    assert solution.tracking_epoch == 7
    assert solution.orientation_rms_rad < 1e-9


def test_solver_robustly_rejects_a_few_rotation_outliers():
    target = rotation_exp(np.array([0.0, 0.0, -0.06]))
    samples = [_sample(target, stamp=index) for index in range(120)]
    samples.extend(
        _sample(rotation_exp(np.array([0.0, 0.0, 1.2])), stamp=200 + index)
        for index in range(3)
    )

    solution = solve_orientation(samples, np.eye(3), OrientationGates())

    error = solution.rotation.T @ target
    assert np.linalg.norm(error - np.eye(3)) < 0.02


def test_solver_rejects_insufficient_samples_and_epoch_change():
    gates = OrientationGates(min_samples=4)
    target = np.eye(3)
    with pytest.raises(ValueError, match="sample_count"):
        solve_orientation([_sample(target)] * 3, np.eye(3), gates)
    with pytest.raises(ValueError, match="tracking_epoch"):
        solve_orientation(
            [_sample(target, epoch=1, stamp=1), _sample(target, epoch=2, stamp=2),
             _sample(target, epoch=1, stamp=3), _sample(target, epoch=1, stamp=4)],
            np.eye(3), gates,
        )


def test_solver_rejects_excessive_correction():
    target = rotation_exp(np.array([0.0, 0.0, 1.0]))
    samples = [_sample(target, stamp=index) for index in range(4)]
    with pytest.raises(ValueError, match="correction_angle"):
        solve_orientation(
            samples,
            np.eye(3),
            OrientationGates(min_samples=4, max_correction_angle_rad=0.5),
        )


def test_orientation_only_update_preserves_translation_and_advances_only_orientation():
    document = _tcp_document()
    old_translation = list(document["translation_m"])
    solution = solve_orientation(
        [_sample(rotation_exp(np.array([0.1, 0.0, 0.0])), stamp=i) for i in range(4)],
        np.eye(3),
        OrientationGates(min_samples=4),
    )

    updated = build_orientation_only_update(document, solution, "a" * 64)

    assert updated["translation_m"] == old_translation
    assert updated["calibration_revision"] == 5
    assert updated["translation_revision"] == 4
    assert updated["orientation_revision"] == 1
    assert updated["translation_fingerprint_sha256"] == translation_fingerprint(document)
    assert updated["orientation_only_ancestor_sha256"] == ["a" * 64]
    assert updated["orientation_calibration"]["tracking_epoch"] == 7
    assert updated["orientation_reference"] == "gravity_leveled_hmd_heading"
    assert (
        updated["orientation_calibration"]["method"]
        == "gravity_leveled_hmd_heading_bilateral_forward_palms_facing"
    )
    assert updated["quaternion_xyzw"] == pytest.approx(
        matrix_to_quaternion_xyzw(solution.rotation)
    )
    assert len(updated["covariance_upper_triangle_6x6"]) == 21


def test_translation_fingerprint_is_semantic_and_detects_translation_changes():
    first = _tcp_document()
    reordered = json.loads(json.dumps(first))
    reordered["quality"]["position_rms_m"] = 0.010
    reordered["quaternion_xyzw"] = [0.1, 0.0, 0.0, 0.995]
    assert translation_fingerprint(first) == translation_fingerprint(reordered)

    reordered["translation_m"][0] += 0.001
    assert translation_fingerprint(first) != translation_fingerprint(reordered)


def test_atomic_replace_rejects_concurrent_file_change(tmp_path):
    path = tmp_path / "tcp.yaml"
    document = _tcp_document()
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    old_hash = hashlib.sha256(path.read_bytes()).hexdigest()
    path.write_text(yaml.safe_dump({**document, "calibration_revision": 99}), encoding="utf-8")

    with pytest.raises(ValueError, match="changed during calibration"):
        atomic_replace_if_unchanged(path, old_hash, document)

    assert yaml.safe_load(path.read_text())["calibration_revision"] == 99
