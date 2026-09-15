#!/usr/bin/env python3
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))

from pico_arm_geometry_core import (  # noqa: E402
    ArmSide,
    solve_arm_static_pose_geometry,
)
from pico_left_arm_geometry_core import (  # noqa: E402
    solve_left_arm_static_pose_geometry,
)


def _static_cloud(center: np.ndarray, *, seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    return np.asarray(center, dtype=float) + rng.normal(scale=1.0e-4, size=(80, 3))


def _valid_static_capture() -> dict[str, np.ndarray]:
    shoulder = np.array([0.0, 0.2, 1.0])
    upper_arm_length = 0.30
    forearm_length = 0.25
    neutral = shoulder + np.array([0.0, 0.0, -(upper_arm_length + forearm_length)])
    straight = shoulder + np.array([upper_arm_length + forearm_length, 0.0, 0.0])
    elbow = shoulder + np.array([0.0, 0.0, -upper_arm_length])
    right_angle = elbow + np.array([forearm_length, 0.0, 0.0])
    return {
        "neutral_start_wrist_positions": _static_cloud(neutral, seed=1),
        "neutral_return_wrist_positions": _static_cloud(neutral, seed=2),
        "straight_1_wrist_positions": _static_cloud(straight, seed=3),
        "straight_2_wrist_positions": _static_cloud(straight, seed=4),
        "validation_wrist_positions": _static_cloud(straight, seed=5),
        "right_angle_wrist_positions": _static_cloud(right_angle, seed=6),
        "right_angle_shoulder_positions": _static_cloud(shoulder, seed=7),
        "right_angle_raw_elbow_positions": _static_cloud(elbow, seed=8),
        "right_angle_raw_wrist_positions": _static_cloud(right_angle, seed=9),
    }


def test_generic_solver_is_identical_for_left_and_right():
    capture = _valid_static_capture()
    left = solve_arm_static_pose_geometry(side=ArmSide.LEFT, **capture)
    right = solve_arm_static_pose_geometry(side=ArmSide.RIGHT, **capture)
    assert left.valid
    assert right.valid
    assert left == right


def test_legacy_left_api_matches_generic_solver_exactly():
    capture = _valid_static_capture()
    legacy = solve_left_arm_static_pose_geometry(**capture)
    generic = solve_arm_static_pose_geometry(side=ArmSide.LEFT, **capture)
    assert legacy == generic


def test_generic_solver_rejects_unknown_side_before_solving():
    capture = _valid_static_capture()
    try:
        solve_arm_static_pose_geometry(side="middle", **capture)
    except ValueError as error:
        assert str(error) == "arm_side_invalid:middle"
    else:
        raise AssertionError("unknown arm side was accepted")


def test_degenerate_raw_smpl_diagnostics_do_not_veto_palm_geometry():
    capture = _valid_static_capture()
    degenerate = _static_cloud(np.array([0.0, 0.2, 1.0]), seed=10)
    capture["right_angle_shoulder_positions"] = degenerate
    capture["right_angle_raw_elbow_positions"] = degenerate.copy()
    capture["right_angle_raw_wrist_positions"] = degenerate.copy()

    result = solve_arm_static_pose_geometry(side=ArmSide.LEFT, **capture)

    assert result.valid
    assert result.raw_smpl_diagnostic_available is False
    assert result.right_angle_error_rad is None


def test_raw_smpl_shoulder_motion_is_diagnostic_only():
    capture = _valid_static_capture()
    offsets = np.zeros_like(capture["right_angle_shoulder_positions"])
    offsets[:, 1] = np.linspace(-0.04, 0.04, len(offsets))
    for name in (
        "right_angle_shoulder_positions",
        "right_angle_raw_elbow_positions",
        "right_angle_raw_wrist_positions",
    ):
        capture[name] = capture[name] + offsets

    result = solve_arm_static_pose_geometry(side=ArmSide.LEFT, **capture)

    assert result.shoulder_motion_rms_m > 0.015
    assert result.valid
    assert "right_angle_shoulder_motion_exceeded" not in result.rejection_reasons


def test_nonfinite_raw_smpl_diagnostics_do_not_veto_palm_geometry():
    capture = _valid_static_capture()
    invalid_shoulder = capture["right_angle_shoulder_positions"].copy()
    invalid_shoulder[0, 0] = np.nan
    capture["right_angle_shoulder_positions"] = invalid_shoulder

    result = solve_arm_static_pose_geometry(side=ArmSide.LEFT, **capture)

    assert result.valid
    assert result.raw_smpl_diagnostic_available is False
    assert result.right_angle_error_rad is None
    assert np.all(np.isfinite(result.shoulder_anchor_pico_m))
    assert np.all(np.isfinite(result.elbow_center_pico_m))


def test_static_pose_direction_gate_allows_human_pose_tolerance_below_30_degrees():
    capture = _valid_static_capture()
    right_angle = np.array([0.25, 0.30, 0.70])
    capture["right_angle_wrist_positions"] = _static_cloud(right_angle, seed=20)

    result = solve_arm_static_pose_geometry(side=ArmSide.LEFT, **capture)

    assert np.deg2rad(28.0) < result.transition_direction_error_rad < np.deg2rad(30.0)
    assert result.valid


def test_static_pose_direction_gate_still_rejects_above_30_degrees():
    capture = _valid_static_capture()
    right_angle = np.array([0.25, 0.32, 0.70])
    capture["right_angle_wrist_positions"] = _static_cloud(right_angle, seed=21)

    result = solve_arm_static_pose_geometry(side=ArmSide.LEFT, **capture)

    assert result.transition_direction_error_rad > np.deg2rad(30.0)
    assert "static_transition_direction_error_exceeded" in result.rejection_reasons


def test_neutral_pose_gate_accepts_65_mm_but_rejects_above_70_mm():
    accepted_capture = _valid_static_capture()
    accepted_capture["neutral_return_wrist_positions"] += np.array([0.065, 0.0, 0.0])

    accepted = solve_arm_static_pose_geometry(side=ArmSide.LEFT, **accepted_capture)

    assert accepted.neutral_pair_position_error_m == pytest.approx(0.065, abs=5.0e-5)
    assert "neutral_pair_position_error_exceeded" not in accepted.rejection_reasons

    rejected_capture = _valid_static_capture()
    rejected_capture["neutral_return_wrist_positions"] += np.array([0.071, 0.0, 0.0])

    rejected = solve_arm_static_pose_geometry(side=ArmSide.LEFT, **rejected_capture)

    assert rejected.neutral_pair_position_error_m == pytest.approx(0.071, abs=5.0e-5)
    assert "neutral_pair_position_error_exceeded" in rejected.rejection_reasons
