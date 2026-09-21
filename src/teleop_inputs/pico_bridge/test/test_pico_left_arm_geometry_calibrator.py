#!/usr/bin/env python3
import json
import sys
from collections import deque
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))

from pico_left_arm_geometry_calibrator import (  # noqa: E402
    CaptureSample,
    LeftArmCaptureBuffer,
    PendingRawSample,
    PicoLeftArmGeometryCalibrator,
    _candidate_from_result,
    atomic_write_bundle,
    default_stage_sequence,
    stable_suffix,
    wrist_from_palm,
)
from pico_palm_skeleton_filter_node import PalmSample  # noqa: E402
from pico_left_arm_geometry_core import LeftArmStaticPoseResult  # noqa: E402


REFERENCE_FIXTURE = (
    Path(__file__).parent
    / "fixtures"
    / "pico_left_arm_geometry_quick_v3_reference.yaml"
)


def test_frozen_left_v3_reference_contract():
    document = yaml.safe_load(REFERENCE_FIXTURE.read_text(encoding="utf-8"))
    assert document["artifact_type"] == "pico_left_arm_geometry_quick_v3"
    assert document["schema_version"] == 3
    assert document["valid"] is True
    assert document["candidate_status"] == "accepted"
    assert document["side"] == "left"
    assert document["upper_arm_length_m"] == pytest.approx(0.28452446135474074)
    assert document["forearm_length_m"] == pytest.approx(0.22400856924665608)
    assert document["quality"]["length_std_m"] == pytest.approx(
        0.007023626378575398
    )
    assert document["quality"]["right_angle_error_role"] == "diagnostic_only"
    assert document["calibration_revision"] > 0


def _sample(stamp_ns: int, epoch: int = 4) -> CaptureSample:
    return CaptureSample(
        stamp_ns=stamp_ns,
        tracking_epoch=epoch,
        shoulder_pico_m=np.array([0.0, 0.25, 1.4]),
        raw_elbow_pico_m=np.array([0.0, 0.25, 1.1]),
        raw_wrist_pico_m=np.array([0.25, 0.25, 1.1]),
        palm_position_pico_m=np.array([0.33, 0.25, 1.1]),
        palm_orientation_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        wrist_pico_m=np.array([0.25, 0.25, 1.1]),
    )


def test_wrist_from_palm_uses_palm_local_positive_x():
    half = np.sqrt(0.5)
    orientation = np.array([0.0, 0.0, half, half])
    wrist = wrist_from_palm(
        np.array([1.0, 2.0, 3.0]), orientation, wrist_to_palm_distance_m=0.08
    )
    np.testing.assert_allclose(wrist, [1.0, 1.92, 3.0], atol=1e-12)


def test_default_sequence_is_left_only_and_fully_automatic():
    stages = default_stage_sequence()
    assert [stage.name for stage in stages] == [
        "neutral_start",
        "straight_reach_1",
        "straight_reach_2",
        "elbow_right_angle",
        "straight_validation",
        "neutral_return",
    ]
    assert all(stage.duration_s > 0.0 for stage in stages)
    assert {stage.group for stage in stages} == {
        "neutral_start",
        "straight_1",
        "straight_2",
        "right_angle",
        "validation",
        "neutral_return",
    }


def test_capture_archive_preserves_raw_palm_orientation_and_raw_wrist():
    capture = LeftArmCaptureBuffer()
    capture.start_stage("elbow_right_angle", "right_angle", tracking_epoch=4)
    capture.append(_sample(10))
    capture.finish_stage("elbow_right_angle")
    archive = capture.archive_arrays()
    np.testing.assert_allclose(
        archive["right_angle_palm_orientation_xyzw"], [[0.0, 0.0, 0.0, 1.0]]
    )
    np.testing.assert_allclose(
        archive["right_angle_palm_position_pico_m"], [[0.33, 0.25, 1.1]]
    )
    np.testing.assert_allclose(
        archive["right_angle_raw_wrist_pico_m"], [[0.25, 0.25, 1.1]]
    )


def test_capture_buffer_rejects_duplicate_stamp_and_epoch_change():
    capture = LeftArmCaptureBuffer()
    capture.start_stage("straight_reach_1", "straight", tracking_epoch=4)
    assert capture.append(_sample(10))
    assert not capture.append(_sample(10))
    with pytest.raises(RuntimeError, match="out_of_order"):
        capture.append(_sample(9))
    with pytest.raises(RuntimeError, match="tracking_epoch_changed"):
        capture.append(_sample(11, epoch=5))
    capture.finish_stage("straight_reach_1")
    assert len(capture.group_samples("straight")) == 1


def test_stage_ranges_count_only_samples_from_each_repeated_group_stage():
    capture = LeftArmCaptureBuffer()
    capture.start_stage("straight_reach_1", "straight", tracking_epoch=4)
    capture.append(_sample(10))
    capture.finish_stage("straight_reach_1")
    capture.start_stage("straight_reach_2", "straight", tracking_epoch=4)
    capture.append(_sample(20))
    capture.finish_stage("straight_reach_2")
    assert [stage["accepted_sample_count"] for stage in capture.stage_ranges] == [1, 1]
    assert capture.stage_ranges[1]["source_start_ns"] == 20


def test_atomic_bundle_preserves_capture_and_rejected_candidate(tmp_path):
    capture = LeftArmCaptureBuffer()
    capture.start_stage("straight_reach_1", "straight", tracking_epoch=4)
    capture.append(_sample(10))
    capture.finish_stage("straight_reach_1")
    candidate = {
        "artifact_type": "pico_left_arm_geometry_quick_v3",
        "valid": False,
        "candidate_status": "rejected",
        "rejection_reasons": ["circle_residual_exceeded"],
    }
    report = {"valid": False, "rejection_reasons": ["circle_residual_exceeded"]}
    atomic_write_bundle(tmp_path, capture, candidate, report)

    assert (tmp_path / "capture.npz").is_file()
    assert (tmp_path / "stage_ranges.yaml").is_file()
    assert (tmp_path / "pico_left_arm_geometry_candidate.yaml").is_file()
    loaded_report = json.loads((tmp_path / "gate_report.json").read_text())
    assert loaded_report["valid"] is False
    archive = np.load(tmp_path / "capture.npz")
    assert archive["straight_stamp_ns"].tolist() == [10]
    with pytest.raises(FileExistsError):
        atomic_write_bundle(tmp_path, capture, candidate, report)


def test_stable_suffix_selects_the_longest_static_tail():
    moving = [
        _sample(index + 1) for index in range(80)
    ]
    stable = []
    for index in range(70):
        sample = _sample(100 + index)
        stable.append(
            CaptureSample(
                **{
                    **sample.__dict__,
                    "wrist_pico_m": np.array([0.4, 0.1, 0.2])
                    + [index * 1.0e-5, 0.0, 0.0],
                }
            )
        )
    for index, sample in enumerate(moving):
        moving[index] = CaptureSample(
            **{
                **sample.__dict__,
                "wrist_pico_m": np.array([index * 0.01, 0.1, 0.2]),
            }
        )
    selected = stable_suffix(tuple(moving + stable))
    assert len(selected) == 70
    wrists = np.asarray([sample.wrist_pico_m for sample in selected])
    center = np.median(wrists, axis=0)
    assert np.sqrt(np.mean(np.sum((wrists - center) ** 2, axis=1))) <= 0.015


def test_pairing_accepts_palm_that_arrives_after_raw():
    node = object.__new__(PicoLeftArmGeometryCalibrator)
    node._max_skew_ns = 30_000_000
    node._wrist_to_palm_distance_m = 0.08
    node._palm_cache = deque(maxlen=240)
    node._pending_raw = deque(maxlen=240)
    node._latest_palm_stamp_ns = 0
    node._capture_error = None
    node.capture = LeftArmCaptureBuffer()
    node.capture.start_stage("flex", "flex", tracking_epoch=4)
    node._pending_raw.append(
        PendingRawSample(
            1_000_000_000,
            4,
            np.array([0.0, 0.25, 1.4]),
            np.array([0.0, 0.25, 1.1]),
            np.array([0.25, 0.25, 1.1]),
        )
    )
    node._drain_pairs()
    assert not node.capture.group_samples("flex")

    palm = PalmSample(
        1_005_000_000,
        np.array([0.28, 0.25, 1.1]),
        np.array([0.0, 0.0, 0.0, 1.0]),
        "pico",
    )
    node._latest_palm_stamp_ns = palm.stamp_ns
    node._palm_cache.append((palm, 4))
    node._drain_pairs()
    assert len(node.capture.group_samples("flex")) == 1
    assert not node._pending_raw


def test_epoch_transition_clears_pending_data_and_is_fatal_during_capture():
    node = object.__new__(PicoLeftArmGeometryCalibrator)
    node._palm_cache = deque([(object(), 4)], maxlen=240)
    node._pending_raw = deque([object()], maxlen=240)
    node._capture_error = None
    node.capture = LeftArmCaptureBuffer()
    node.capture.start_stage("flex", "flex", tracking_epoch=4)
    node._handle_epoch_transition(4, 5)
    assert not node._palm_cache
    assert not node._pending_raw
    with pytest.raises(RuntimeError, match="tracking_epoch_changed"):
        node.raise_capture_error()


def test_candidate_cannot_accept_empty_or_nonfinite_validation(tmp_path):
    result = LeftArmStaticPoseResult(
        valid=True,
        shoulder_anchor_pico_m=(0.0, 0.2, 1.4),
        elbow_center_pico_m=(0.0, 0.2, 1.1),
        upper_arm_length_m=0.30,
        forearm_length_m=0.25,
        length_std_m=0.003,
        selected_straight_groups=("straight_1", "validation"),
        discarded_straight_group="straight_2",
        straight_pair_position_error_m=0.03,
        upper_repeat_error_m=0.01,
        neutral_pair_position_error_m=0.02,
        forearm_repeat_error_m=0.01,
        static_motion_rms_m=0.005,
        right_angle_error_rad=0.02,
        raw_smpl_diagnostic_available=True,
        transition_direction_error_rad=0.03,
        length_closure_error_m=0.005,
        shoulder_motion_rms_m=0.005,
        straight_1_sample_count=100,
        straight_2_sample_count=100,
        validation_sample_count=100,
        neutral_start_sample_count=100,
        neutral_return_sample_count=100,
        right_angle_sample_count=140,
        covariance_8x8=tuple(
            tuple(0.001 if row == column else 0.0 for column in range(8))
            for row in range(8)
        ),
        rejection_reasons=(),
    )
    candidate, report = _candidate_from_result(
        replace(
            result,
            valid=False,
            rejection_reasons=("solver_result_invalid",),
        ),
        tcp_revision=1,
        tcp_artifact_hash="tcp-hash",
        tcp_translation_revision=1,
        tcp_translation_fingerprint="a" * 64,
        wrist_pivot_hash="abc",
        tracking_epoch=4,
        tracking_epoch_source="tcp_connection",
        output_dir=tmp_path,
    )
    assert candidate["candidate_status"] == "rejected"
    assert report["valid"] is False
    assert "solver_result_invalid" in candidate["rejection_reasons"]

    accepted, accepted_report = _candidate_from_result(
        result,
        tcp_revision=1,
        tcp_artifact_hash="tcp-hash",
        tcp_translation_revision=1,
        tcp_translation_fingerprint="a" * 64,
        wrist_pivot_hash="abc",
        tracking_epoch=4,
        tracking_epoch_source="tcp_connection",
        output_dir=tmp_path / "accepted",
    )
    assert accepted["candidate_status"] == "accepted"
    assert "right_angle_pose_error" not in accepted["gate_results"]
    assert accepted["quality"]["right_angle_error_role"] == "diagnostic_only"
    assert all(type(value) is bool for value in accepted["gate_results"].values())

    relaxed, _ = _candidate_from_result(
        replace(result, neutral_pair_position_error_m=0.065),
        tcp_revision=1,
        tcp_artifact_hash="tcp-hash",
        tcp_translation_revision=1,
        tcp_translation_fingerprint="a" * 64,
        wrist_pivot_hash="abc",
        tracking_epoch=4,
        tracking_epoch_source="tcp_connection",
        output_dir=tmp_path / "relaxed",
    )
    assert relaxed["gate_results"]["neutral_pair_position_error"] is True

    excessive, _ = _candidate_from_result(
        replace(result, neutral_pair_position_error_m=0.071),
        tcp_revision=1,
        tcp_artifact_hash="tcp-hash",
        tcp_translation_revision=1,
        tcp_translation_fingerprint="a" * 64,
        wrist_pivot_hash="abc",
        tracking_epoch=4,
        tracking_epoch_source="tcp_connection",
        output_dir=tmp_path / "excessive",
    )
    assert excessive["gate_results"]["neutral_pair_position_error"] is False

    atomic_write_bundle(
        tmp_path / "accepted", LeftArmCaptureBuffer(), accepted, accepted_report
    )
    assert json.loads((tmp_path / "accepted/gate_report.json").read_text())["valid"]
