#!/usr/bin/env python3
import json
import math
import sys
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))

from pico_arm_geometry_calibrator import (  # noqa: E402
    ArmIndices,
    artifact_type_for_side,
    candidate_from_result,
    default_palm_topic,
    default_stage_sequence,
    _parse_arguments,
    human_readable_rejection_reasons,
)
from pico_arm_geometry_core import ArmSide, ArmStaticPoseResult  # noqa: E402


def _valid_result() -> ArmStaticPoseResult:
    covariance = tuple(
        tuple(0.001 if row == column else 0.0 for column in range(8))
        for row in range(8)
    )
    return ArmStaticPoseResult(
        valid=True,
        shoulder_anchor_pico_m=(0.0, -0.2, 1.4),
        elbow_center_pico_m=(0.0, -0.2, 1.1),
        upper_arm_length_m=0.30,
        forearm_length_m=0.25,
        length_std_m=0.003,
        selected_straight_groups=("straight_1", "straight_2"),
        discarded_straight_group="validation",
        straight_pair_position_error_m=0.01,
        upper_repeat_error_m=0.01,
        neutral_pair_position_error_m=0.01,
        forearm_repeat_error_m=0.01,
        static_motion_rms_m=0.003,
        right_angle_error_rad=0.2,
        raw_smpl_diagnostic_available=True,
        transition_direction_error_rad=0.03,
        length_closure_error_m=0.002,
        shoulder_motion_rms_m=0.003,
        straight_1_sample_count=100,
        straight_2_sample_count=100,
        validation_sample_count=100,
        neutral_start_sample_count=100,
        neutral_return_sample_count=100,
        right_angle_sample_count=100,
        covariance_8x8=covariance,
        rejection_reasons=(),
    )


def test_side_mapping_uses_independent_smpl_indices_and_topics():
    assert ArmIndices.for_side(ArmSide.LEFT) == ArmIndices(16, 18, 20)
    assert ArmIndices.for_side(ArmSide.RIGHT) == ArmIndices(17, 19, 21)
    assert default_palm_topic(ArmSide.LEFT) == "/pico/palm_left"
    assert default_palm_topic(ArmSide.RIGHT) == "/pico/palm_right"


def test_stage_instructions_name_only_the_selected_side():
    left_text = " ".join(stage.instruction for stage in default_stage_sequence(ArmSide.LEFT))
    right_text = " ".join(stage.instruction for stage in default_stage_sequence(ArmSide.RIGHT))
    assert "左臂" in left_text
    assert "右臂" not in left_text
    assert "右臂" in right_text
    assert "左臂" not in right_text


def test_geometry_interactive_start_is_space_not_enter(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["pico_arm_geometry_calibrator", "--side", "left",
                                     "--tcp-artifact", "tcp.yaml",
                                     "--wrist-pivot-artifact", "wrist.yaml"])

    arguments, _ = _parse_arguments()

    assert arguments.start_mode == "space"


def test_rejection_reasons_explain_how_operator_can_correct_the_pose():
    messages = human_readable_rejection_reasons([
        "neutral_pair_position_error_exceeded",
        "static_transition_direction_error_exceeded",
        "length_closure_error_exceeded",
    ])

    assert any("回到开始时相同的位置" in message for message in messages)
    assert any("向正前方" in message for message in messages)
    assert any("肘部完全伸直" in message for message in messages)


def test_direction_rejection_reports_measured_and_allowed_angles():
    messages = human_readable_rejection_reasons(
        ["static_transition_direction_error_exceeded"],
        quality={"transition_direction_error_rad": math.radians(31.5)},
    )

    assert "31.5°" in messages[0]
    assert "30.0°" in messages[0]


def test_right_candidate_has_independent_artifact_and_lineage(tmp_path):
    candidate, report = candidate_from_result(
        _valid_result(),
        side=ArmSide.RIGHT,
        tcp_revision=9,
        tcp_artifact_hash="right-tcp-hash",
        tcp_translation_revision=7,
        tcp_translation_fingerprint="a" * 64,
        wrist_pivot_hash="right-wrist-hash",
        tracking_epoch=4,
        tracking_epoch_source="wire_world_reset",
        output_dir=tmp_path,
    )
    assert report["valid"] is True
    assert candidate["artifact_type"] == "pico_right_arm_geometry_quick_v3"
    assert candidate["artifact_type"] == artifact_type_for_side(ArmSide.RIGHT)
    assert candidate["side"] == "right"
    assert candidate["calibration_revision"] > 0
    assert "/pico/palm_right" in candidate["lineage"]
    assert "/pico/palm_left" not in candidate["lineage"]
    assert "left" not in json.dumps(candidate["lineage"])
    assert candidate["tcp_translation_revision"] == 7
    assert candidate["tcp_translation_fingerprint_sha256"] == "a" * 64


def test_unknown_side_is_rejected():
    try:
        ArmIndices.for_side("middle")
    except ValueError as error:
        assert str(error) == "arm_side_invalid:middle"
    else:
        raise AssertionError("unknown side was accepted")


def test_candidate_records_unavailable_raw_smpl_diagnostic_without_rejecting(tmp_path):
    result = replace(
        _valid_result(),
        raw_smpl_diagnostic_available=False,
        right_angle_error_rad=None,
    )

    candidate, report = candidate_from_result(
        result,
        side=ArmSide.LEFT,
        tcp_revision=3,
        tcp_artifact_hash="left-tcp-hash",
        tcp_translation_revision=2,
        tcp_translation_fingerprint="b" * 64,
        wrist_pivot_hash="left-wrist-hash",
        tracking_epoch=2,
        tracking_epoch_source="wire_world_reset",
        output_dir=tmp_path,
    )

    assert report["valid"] is True
    assert candidate["quality"]["raw_smpl_diagnostic_available"] is False
    assert candidate["quality"]["right_angle_error_rad"] is None
