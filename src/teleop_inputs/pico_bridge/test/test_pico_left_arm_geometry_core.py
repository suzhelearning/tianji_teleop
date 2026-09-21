#!/usr/bin/env python3
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))

from pico_left_arm_geometry_core import (  # noqa: E402
    LeftArmGeometryGate,
    fit_circle_3d,
    solve_left_arm_static_pose_geometry,
    solve_left_arm_two_pose_geometry,
    solve_left_arm_geometry,
)


def _synthetic_capture(
    flex_degrees: float = 120.0,
    circle_noise_m: float = 0.0005,
    seed: int = 7,
):
    rng = np.random.default_rng(seed)
    shoulder = np.array([0.02, 0.24, 1.42])
    elbow = np.array([0.04, 0.24, 1.11])
    upper = float(np.linalg.norm(elbow - shoulder))
    forearm = 0.265

    angles = np.linspace(-0.2, math.radians(flex_degrees) - 0.2, 160)
    flex_wrist = np.column_stack(
        [
            elbow[0] + forearm * np.sin(angles),
            np.full_like(angles, elbow[1]),
            elbow[2] - forearm * np.cos(angles),
        ]
    )
    flex_wrist += rng.normal(scale=circle_noise_m, size=flex_wrist.shape)
    flex_shoulder = shoulder + rng.normal(scale=0.0008, size=(len(angles), 3))
    raw_elbow = elbow + rng.normal(scale=0.004, size=(len(angles), 3))

    reach_directions = np.asarray(
        [[1.0, 0.00, 0.02], [0.99, 0.04, 0.03], [0.99, -0.04, -0.01]]
    )
    reach_directions /= np.linalg.norm(reach_directions, axis=1, keepdims=True)
    straight_wrist = np.repeat(reach_directions, 40, axis=0) * (upper + forearm)
    straight_wrist += shoulder
    straight_wrist += rng.normal(scale=0.001, size=straight_wrist.shape)
    straight_shoulder = shoulder + rng.normal(scale=0.0008, size=straight_wrist.shape)
    return {
        "shoulder": shoulder,
        "elbow": elbow,
        "upper": upper,
        "forearm": forearm,
        "flex_wrist": flex_wrist,
        "flex_shoulder": flex_shoulder,
        "raw_elbow": raw_elbow,
        "straight_wrist": straight_wrist,
        "straight_shoulder": straight_shoulder,
    }


def test_fit_circle_3d_recovers_center_radius_plane_and_coverage():
    data = _synthetic_capture()
    fit = fit_circle_3d(data["flex_wrist"])
    np.testing.assert_allclose(fit.center_m, data["elbow"], atol=0.002)
    assert fit.radius_m == pytest.approx(data["forearm"], abs=0.002)
    assert abs(float(np.dot(fit.normal, [0.0, 1.0, 0.0]))) > 0.99
    assert fit.angular_coverage_rad > math.radians(110.0)
    assert fit.residual_rms_m < 0.002
    assert fit.radius_std_m < 0.003


def test_solve_left_arm_geometry_recovers_lengths_and_covariance():
    data = _synthetic_capture()
    result = solve_left_arm_geometry(
        straight_shoulder_positions=data["straight_shoulder"],
        straight_wrist_positions=data["straight_wrist"],
        flex_shoulder_positions=data["flex_shoulder"],
        flex_wrist_positions=data["flex_wrist"],
        flex_raw_elbow_positions=data["raw_elbow"],
    )
    assert result.valid
    assert result.upper_arm_length_m == pytest.approx(data["upper"], abs=0.005)
    assert result.forearm_length_m == pytest.approx(data["forearm"], abs=0.003)
    np.testing.assert_allclose(
        result.shoulder_anchor_pico_m, data["shoulder"], atol=0.003
    )
    np.testing.assert_allclose(result.elbow_center_pico_m, data["elbow"], atol=0.003)
    assert result.straight_reach_residual_rms_m < 0.005
    covariance = np.asarray(result.covariance_8x8)
    assert covariance.shape == (8, 8)
    np.testing.assert_allclose(covariance, covariance.T, atol=1e-12)
    assert np.linalg.eigvalsh(covariance).min() >= -1e-12


def test_solver_rejects_insufficient_flexion_coverage():
    data = _synthetic_capture(flex_degrees=15.0)
    result = solve_left_arm_geometry(
        data["straight_shoulder"],
        data["straight_wrist"],
        data["flex_shoulder"],
        data["flex_wrist"],
        data["raw_elbow"],
    )
    assert not result.valid
    assert "angular_coverage_below_minimum" in result.rejection_reasons


def test_solver_rejects_excessive_circle_residual():
    data = _synthetic_capture(circle_noise_m=0.035)
    gate = LeftArmGeometryGate(max_circle_rms_m=0.015)
    result = solve_left_arm_geometry(
        data["straight_shoulder"],
        data["straight_wrist"],
        data["flex_shoulder"],
        data["flex_wrist"],
        data["raw_elbow"],
        gate=gate,
    )
    assert not result.valid
    assert "circle_residual_exceeded" in result.rejection_reasons


def test_solver_rejects_nonplanar_wrist_trajectory():
    data = _synthetic_capture(circle_noise_m=0.0)
    nonplanar = data["flex_wrist"].copy()
    nonplanar[:, 1] += 0.04 * np.sin(np.linspace(0.0, 12.0 * math.pi, len(nonplanar)))
    result = solve_left_arm_geometry(
        data["straight_shoulder"],
        data["straight_wrist"],
        data["flex_shoulder"],
        nonplanar,
        data["raw_elbow"],
    )
    assert not result.valid
    assert "circle_residual_exceeded" in result.rejection_reasons


def test_solver_rejects_noisy_near_minimum_arc_that_biases_length_split():
    data = _synthetic_capture(flex_degrees=80.0, circle_noise_m=0.010)
    result = solve_left_arm_geometry(
        data["straight_shoulder"],
        data["straight_wrist"],
        data["flex_shoulder"],
        data["flex_wrist"],
        data["raw_elbow"],
    )
    assert not result.valid
    assert {
        "angular_coverage_below_minimum",
        "circle_residual_exceeded",
        "forearm_uncertainty_exceeded",
    } & set(result.rejection_reasons)


def test_upper_length_uses_flex_shoulder_not_pose_shifted_straight_shoulder():
    data = _synthetic_capture()
    shifted_straight_shoulder = data["straight_shoulder"] + [0.0, 0.025, 0.0]
    shifted_straight_wrist = data["straight_wrist"] + [0.0, 0.025, 0.0]
    result = solve_left_arm_geometry(
        shifted_straight_shoulder,
        shifted_straight_wrist,
        data["flex_shoulder"],
        data["flex_wrist"],
        data["raw_elbow"],
    )
    assert result.valid
    assert result.upper_arm_length_m == pytest.approx(data["upper"], abs=0.005)


def test_solver_is_deterministic_and_rejects_nonfinite_input():
    data = _synthetic_capture()
    args = (
        data["straight_shoulder"],
        data["straight_wrist"],
        data["flex_shoulder"],
        data["flex_wrist"],
        data["raw_elbow"],
    )
    first = solve_left_arm_geometry(*args)
    second = solve_left_arm_geometry(*args)
    assert first == second

    invalid = data["flex_wrist"].copy()
    invalid[0, 0] = np.nan
    with pytest.raises(ValueError, match="finite"):
        solve_left_arm_geometry(
            data["straight_shoulder"],
            data["straight_wrist"],
            data["flex_shoulder"],
            invalid,
            data["raw_elbow"],
        )


def _two_pose_capture(seed: int = 19):
    rng = np.random.default_rng(seed)
    upper = 0.31
    forearm = 0.26
    shoulder = np.array([0.1, 0.2, 1.45])
    elbow = shoulder + np.array([0.0, 0.0, -upper])
    right_wrist = elbow + np.array([forearm, 0.0, 0.0])
    straight_wrist = shoulder + np.array([upper + forearm, 0.0, 0.0])

    def cloud(center, count=180, sigma=0.001):
        return center + rng.normal(scale=sigma, size=(count, 3))

    return {
        "upper": upper,
        "forearm": forearm,
        "straight_1_shoulder": cloud(shoulder),
        "straight_1_wrist": cloud(straight_wrist),
        "straight_2_shoulder": cloud(shoulder),
        "straight_2_wrist": cloud(straight_wrist),
        "right_shoulder": cloud(shoulder),
        "right_wrist": cloud(right_wrist),
        "raw_elbow": cloud(elbow),
        "raw_wrist": cloud(right_wrist),
    }


def test_two_pose_solver_recovers_labeled_upper_and_forearm_lengths():
    data = _two_pose_capture()
    result = solve_left_arm_two_pose_geometry(
        data["straight_1_shoulder"],
        data["straight_1_wrist"],
        data["straight_2_shoulder"],
        data["straight_2_wrist"],
        data["right_shoulder"],
        data["right_wrist"],
        data["raw_elbow"],
        data["raw_wrist"],
    )
    assert result.valid
    assert result.upper_arm_length_m == pytest.approx(data["upper"], abs=0.005)
    assert result.forearm_length_m == pytest.approx(data["forearm"], abs=0.005)
    assert result.straight_repeat_error_m < 0.005
    assert result.right_angle_error_rad < math.radians(3.0)
    assert result.length_std_m < 0.005


def test_two_pose_solver_rejects_impossible_diagonal_geometry():
    data = _two_pose_capture()
    # For positive U/F with S=U+F, the 90-degree diagonal cannot be below
    # S/sqrt(2).  This reproduces the failed hardware capture structurally.
    impossible = data["right_shoulder"] + np.array([0.30, 0.0, 0.0])
    result = solve_left_arm_two_pose_geometry(
        data["straight_1_shoulder"],
        data["straight_1_wrist"],
        data["straight_2_shoulder"],
        data["straight_2_wrist"],
        data["right_shoulder"],
        impossible,
        data["raw_elbow"],
        data["raw_wrist"],
    )
    assert not result.valid
    assert "right_angle_geometry_inconsistent" in result.rejection_reasons


def test_two_pose_solver_rejects_pose_that_is_not_near_right_angle():
    data = _two_pose_capture()
    raw_wrist = data["raw_elbow"] + np.array([0.0, 0.0, -0.25])
    result = solve_left_arm_two_pose_geometry(
        data["straight_1_shoulder"],
        data["straight_1_wrist"],
        data["straight_2_shoulder"],
        data["straight_2_wrist"],
        data["right_shoulder"],
        data["right_wrist"],
        data["raw_elbow"],
        raw_wrist,
    )
    assert not result.valid
    assert "right_angle_pose_error_exceeded" in result.rejection_reasons


def _three_static_pose_capture(seed: int = 31):
    rng = np.random.default_rng(seed)
    upper = 0.29
    forearm = 0.24
    shoulder = np.array([0.12, 0.18, 1.38])
    forward = np.array([1.0, 0.0, 0.0])
    down = np.array([0.0, 0.0, -1.0])
    neutral = shoulder + down * (upper + forearm)
    straight = shoulder + forward * (upper + forearm)
    right_angle = shoulder + down * upper + forward * forearm
    elbow = shoulder + down * upper

    def cloud(center, count=180, sigma=0.001):
        return center + rng.normal(scale=sigma, size=(count, 3))

    return {
        "upper": upper,
        "forearm": forearm,
        "neutral_start": cloud(neutral),
        "neutral_return": cloud(neutral),
        "straight_1": cloud(straight),
        # One full capture may be performed in the wrong direction.  It must
        # not poison the two mutually consistent straight captures.
        "straight_2": cloud(straight + [0.0, -0.22, 0.0]),
        "validation": cloud(straight),
        "right_angle": cloud(right_angle),
        "right_shoulder": cloud(shoulder),
        "raw_elbow": cloud(elbow),
        "raw_wrist": cloud(right_angle),
    }


def test_static_pose_solver_eliminates_unknown_shoulder_and_rejects_one_straight_outlier():
    data = _three_static_pose_capture()
    result = solve_left_arm_static_pose_geometry(
        data["neutral_start"],
        data["neutral_return"],
        data["straight_1"],
        data["straight_2"],
        data["validation"],
        data["right_angle"],
        data["right_shoulder"] + [0.35, -0.20, 0.10],
        data["raw_elbow"] + [0.35, -0.20, 0.10],
        data["raw_wrist"] + [0.35, -0.20, 0.10],
    )
    assert result.valid
    assert result.upper_arm_length_m == pytest.approx(data["upper"], abs=0.005)
    assert result.forearm_length_m == pytest.approx(data["forearm"], abs=0.005)
    assert result.selected_straight_groups == ("straight_1", "validation")
    assert result.discarded_straight_group == "straight_2"
    assert result.transition_direction_error_rad < math.radians(3.0)


def test_static_pose_solver_treats_raw_smpl_elbow_angle_as_diagnostic_only():
    data = _three_static_pose_capture()
    # The raw PICO SMPL elbow can be substantially biased even when the
    # independently tracked palm poses form a valid three-pose calibration.
    biased_raw_wrist = data["raw_elbow"] + np.array([0.0, 0.0, -0.24])
    result = solve_left_arm_static_pose_geometry(
        data["neutral_start"],
        data["neutral_return"],
        data["straight_1"],
        data["straight_2"],
        data["validation"],
        data["right_angle"],
        data["right_shoulder"],
        data["raw_elbow"],
        biased_raw_wrist,
    )
    assert result.valid
    assert result.right_angle_error_rad > math.radians(15.0)
    assert "right_angle_pose_error_exceeded" not in result.rejection_reasons


def test_static_pose_solver_rejects_when_no_two_straight_captures_agree():
    data = _three_static_pose_capture()
    result = solve_left_arm_static_pose_geometry(
        data["neutral_start"],
        data["neutral_return"],
        data["straight_1"],
        data["straight_2"],
        data["validation"] + [0.0, 0.24, 0.0],
        data["right_angle"],
        data["right_shoulder"],
        data["raw_elbow"],
        data["raw_wrist"],
    )
    assert not result.valid
    assert "straight_pair_position_error_exceeded" in result.rejection_reasons
