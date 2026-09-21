#!/usr/bin/env python3
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from pico_palm_skeleton_filter_core import (  # noqa: E402
    adapt_arm_frames_to_ik_frames,
    adapt_shoulders_to_ik_frames,
    correct_side,
    fit_side_calibration,
    quat_apply,
    quat_from_axis_angle,
    quat_identity,
)


LEFT = (16, 18, 20, 22)
RIGHT = (17, 19, 21, 23)


def _ik_frame_positions():
    positions = np.zeros((24, 3), dtype=float)
    positions[LEFT[0]] = [0.0, 0.25, 1.0]
    positions[LEFT[1]] = [0.0, 0.25, 0.7]
    positions[LEFT[2]] = [0.25, 0.25, 0.7]
    positions[RIGHT[0]] = [0.0, -0.25, 1.0]
    positions[RIGHT[1]] = [0.0, -0.25, 0.7]
    positions[RIGHT[2]] = [-0.25, -0.25, 0.7]
    return positions


def test_arm_ik_frame_adapter_aligns_elbow_x_with_forearm_and_y_with_hinge_axis():
    positions = _ik_frame_positions()
    orientations = np.zeros((24, 4), dtype=float)
    orientations[:, 3] = 1.0
    positions_before = positions.copy()
    orientations_before = orientations.copy()

    adapted = adapt_arm_frames_to_ik_frames(
        positions,
        orientations,
        left_shoulder_offset_rad=0.0,
        right_shoulder_offset_rad=0.0,
    )

    for shoulder, elbow, wrist, _ in (LEFT, RIGHT):
        rotation = np.column_stack(
            [
                quat_apply(adapted[elbow], np.eye(3)[axis])
                for axis in range(3)
            ]
        )
        forearm = positions[wrist] - positions[elbow]
        forearm /= np.linalg.norm(forearm)
        upper = positions[elbow] - positions[shoulder]
        hinge = np.cross(upper, forearm)
        hinge /= np.linalg.norm(hinge)
        if np.dot(hinge, np.array([0.0, 1.0, 0.0])) < 0.0:
            hinge = -hinge
        np.testing.assert_allclose(rotation[:, 0], forearm, atol=1.0e-9)
        np.testing.assert_allclose(rotation[:, 1], hinge, atol=1.0e-9)
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-9)
        assert np.linalg.det(rotation) == pytest.approx(1.0, abs=1.0e-9)

    np.testing.assert_array_equal(positions, positions_before)
    np.testing.assert_array_equal(orientations, orientations_before)


def test_arm_ik_frame_adapter_uses_body_left_when_arm_is_straight():
    positions = _ik_frame_positions()
    positions[LEFT[0]] = [-0.3, 0.25, 0.7]
    orientations = np.zeros((24, 4), dtype=float)
    orientations[:, 3] = 1.0

    adapted = adapt_arm_frames_to_ik_frames(
        positions,
        orientations,
        left_shoulder_offset_rad=0.0,
        right_shoulder_offset_rad=0.0,
    )

    np.testing.assert_allclose(
        quat_apply(adapted[LEFT[1]], [1.0, 0.0, 0.0]),
        [1.0, 0.0, 0.0],
        atol=1.0e-9,
    )
    body_left = positions[LEFT[0]] - positions[RIGHT[0]]
    body_left[0] = 0.0  # project away the straight forearm direction
    body_left /= np.linalg.norm(body_left)
    np.testing.assert_allclose(
        quat_apply(adapted[LEFT[1]], [0.0, 1.0, 0.0]),
        body_left,
        atol=1.0e-9,
    )


def test_arm_ik_frame_adapter_uses_body_left_when_arm_is_nearly_straight():
    positions = _ik_frame_positions()
    positions[LEFT[1]] = [0.30, 0.251, 1.0]
    positions[LEFT[2]] = [0.55, 0.251, 1.0]
    orientations = np.zeros((24, 4), dtype=float)
    orientations[:, 3] = 1.0

    adapted = adapt_arm_frames_to_ik_frames(
        positions,
        orientations,
        left_shoulder_offset_rad=0.0,
        right_shoulder_offset_rad=0.0,
    )

    body_left = positions[LEFT[0]] - positions[RIGHT[0]]
    forearm = positions[LEFT[2]] - positions[LEFT[1]]
    forearm /= np.linalg.norm(forearm)
    expected_y = body_left - forearm * float(np.dot(body_left, forearm))
    expected_y /= np.linalg.norm(expected_y)
    np.testing.assert_allclose(
        quat_apply(adapted[LEFT[1]], [0.0, 1.0, 0.0]),
        expected_y,
        atol=1.0e-9,
    )


def test_arm_ik_frame_adapter_elbow_y_always_uses_body_left_sign():
    positions = _ik_frame_positions()
    orientations = np.zeros((24, 4), dtype=float)
    orientations[:, 3] = 1.0
    flipped = orientations.copy()
    flipped[LEFT[1]] = quat_from_axis_angle([1.0, 0.0, 0.0], np.pi)
    flipped[RIGHT[1]] = quat_from_axis_angle([1.0, 0.0, 0.0], np.pi)

    reference = adapt_arm_frames_to_ik_frames(
        positions,
        orientations,
        left_shoulder_offset_rad=0.0,
        right_shoulder_offset_rad=0.0,
    )
    adapted = adapt_arm_frames_to_ik_frames(
        positions,
        flipped,
        left_shoulder_offset_rad=0.0,
        right_shoulder_offset_rad=0.0,
    )
    body_left = positions[LEFT[0]] - positions[RIGHT[0]]
    body_left /= np.linalg.norm(body_left)

    for elbow in (LEFT[1], RIGHT[1]):
        reference_y = quat_apply(reference[elbow], [0.0, 1.0, 0.0])
        adapted_y = quat_apply(adapted[elbow], [0.0, 1.0, 0.0])
        assert np.dot(reference_y, body_left) > 0.999999
        assert np.dot(adapted_y, body_left) > 0.999999
        np.testing.assert_allclose(adapted_y, reference_y, atol=1.0e-9)


def test_arm_ik_frame_adapter_uses_previous_y_when_body_left_cannot_select_sign():
    orientations = np.zeros((24, 4), dtype=float)
    orientations[:, 3] = 1.0
    first_positions = _ik_frame_positions()
    first_positions[LEFT[1]] = [0.30, 0.15, 1.0]
    first_positions[LEFT[2]] = [0.55, 0.15, 1.0]
    previous = adapt_arm_frames_to_ik_frames(
        first_positions,
        orientations,
        left_shoulder_offset_rad=0.0,
        right_shoulder_offset_rad=0.0,
    )
    previous_y = quat_apply(previous[LEFT[1]], [0.0, 1.0, 0.0])

    second_positions = first_positions.copy()
    second_positions[LEFT[1], 1] = 0.35
    second_positions[LEFT[2], 1] = 0.35
    adapted = adapt_arm_frames_to_ik_frames(
        second_positions,
        orientations,
        left_shoulder_offset_rad=0.0,
        right_shoulder_offset_rad=0.0,
        previous_orientations_xyzw=previous,
    )

    np.testing.assert_allclose(
        quat_apply(adapted[LEFT[1]], [0.0, 1.0, 0.0]),
        previous_y,
        atol=1.0e-9,
    )


def test_arm_ik_frame_adapter_ignores_unrelated_zero_quaternion():
    positions = _ik_frame_positions()
    orientations = np.zeros((24, 4), dtype=float)
    orientations[:, 3] = 1.0
    orientations[0] = 0.0

    adapted = adapt_arm_frames_to_ik_frames(
        positions,
        orientations,
        left_shoulder_offset_rad=0.0,
        right_shoulder_offset_rad=0.0,
    )

    np.testing.assert_array_equal(adapted[0], orientations[0])


def test_shoulder_ik_frame_adapter_applies_opposite_local_x_offsets_only():
    orientations = np.zeros((24, 4), dtype=float)
    orientations[:, 3] = 1.0

    adapted = adapt_shoulders_to_ik_frames(
        orientations,
        left_offset_rad=math.pi / 2.0,
        right_offset_rad=-math.pi / 2.0,
    )

    np.testing.assert_allclose(
        adapted[LEFT[0]], quat_from_axis_angle([1.0, 0.0, 0.0], math.pi / 2.0)
    )
    np.testing.assert_allclose(
        adapted[RIGHT[0]], quat_from_axis_angle([1.0, 0.0, 0.0], -math.pi / 2.0)
    )
    untouched = [index for index in range(24) if index not in (LEFT[0], RIGHT[0])]
    np.testing.assert_array_equal(adapted[untouched], orientations[untouched])
    np.testing.assert_array_equal(orientations[:, 3], np.ones(24))


def test_shoulder_ik_frame_adapter_right_multiplies_in_the_local_frame():
    orientations = np.zeros((24, 4), dtype=float)
    orientations[:, 3] = 1.0
    shoulder = quat_from_axis_angle([0.0, 0.0, 1.0], math.pi / 2.0)
    orientations[LEFT[0]] = shoulder

    adapted = adapt_shoulders_to_ik_frames(
        orientations,
        left_offset_rad=math.pi / 2.0,
        right_offset_rad=0.0,
    )

    # Local +Y first follows the shoulder's +Z rotation to world -X, then the
    # local +X offset maps +Y to +Z.  A world-frame left multiplication would
    # produce a different direction.
    np.testing.assert_allclose(
        quat_apply(adapted[LEFT[0]], np.array([0.0, 1.0, 0.0])),
        [0.0, 0.0, 1.0],
        atol=1.0e-9,
    )


@pytest.mark.parametrize(
    "orientations,left_offset,right_offset",
    [
        (np.zeros((23, 4)), 0.0, 0.0),
        (np.full((24, 4), np.nan), 0.0, 0.0),
        (np.tile([0.0, 0.0, 0.0, 1.0], (24, 1)), np.nan, 0.0),
    ],
)
def test_shoulder_ik_frame_adapter_rejects_invalid_inputs(
    orientations, left_offset, right_offset
):
    with pytest.raises(ValueError):
        adapt_shoulders_to_ik_frames(
            orientations,
            left_offset_rad=left_offset,
            right_offset_rad=right_offset,
        )


def _pose_arrays(count: int = 4):
    positions = np.zeros((count, 24, 3), dtype=float)
    orientations = np.zeros((count, 24, 4), dtype=float)
    orientations[..., 3] = 1.0
    return positions, orientations


def _calibration_samples(side: str = "left", count: int = 8):
    positions, orientations = _pose_arrays(count)
    shoulder, elbow, wrist, hand = LEFT if side == "left" else RIGHT
    for index in range(count):
        positions[index, shoulder] = [0.0, 0.25 if side == "left" else -0.25, 1.4]
        positions[index, elbow] = [0.30, positions[index, shoulder, 1], 1.4]
        positions[index, wrist] = [0.30, positions[index, shoulder, 1], 1.05]
        positions[index, hand] = positions[index, wrist]
        angle = 0.03 * index
        orientations[index, wrist] = quat_from_axis_angle([0.0, 0.0, 1.0], angle)
    palm_positions = positions[:, wrist] + np.array([0.0, 0.0, -0.10])
    palm_orientations = orientations[:, wrist].copy()
    return positions, orientations, palm_positions, palm_orientations


def test_fit_side_calibration_recovers_lengths_and_palm_offset():
    positions, orientations, palm_positions, palm_orientations = _calibration_samples()
    calibration = fit_side_calibration(
        positions, orientations, palm_positions, palm_orientations, "left"
    )
    assert calibration.upper_length_m == pytest.approx(0.30)
    assert calibration.forearm_length_m == pytest.approx(0.35)
    np.testing.assert_allclose(
        calibration.palm_to_wrist_offset_m, [0.0, 0.0, 0.10], atol=1e-9
    )


@pytest.mark.parametrize("bad_shape", [(3, 24, 3), (4, 23, 3), (4, 24, 2)])
def test_fit_rejects_bad_shapes(bad_shape):
    positions, orientations, palm_positions, palm_orientations = _calibration_samples()
    with pytest.raises(ValueError):
        fit_side_calibration(
            np.zeros(bad_shape),
            orientations,
            palm_positions,
            palm_orientations,
            "left",
        )


def test_fit_rejects_invalid_side_and_nonfinite_values():
    positions, orientations, palm_positions, palm_orientations = _calibration_samples()
    with pytest.raises(ValueError):
        fit_side_calibration(
            positions, orientations, palm_positions, palm_orientations, "middle"
        )
    positions[0, 16, 0] = np.nan
    with pytest.raises(ValueError):
        fit_side_calibration(
            positions, orientations, palm_positions, palm_orientations, "left"
        )


def test_correct_side_hits_palm_and_preserves_bone_lengths():
    positions, orientations, palm_positions, palm_orientations = _calibration_samples(count=8)
    calibration = fit_side_calibration(
        positions, orientations, palm_positions, palm_orientations, "left"
    )
    frame_positions, frame_orientations = _pose_arrays(1)
    frame_positions[0] = positions[0]
    frame_orientations[0] = orientations[0]
    palm = np.array([0.52, 0.25, 1.05])
    palm_quat = quat_from_axis_angle([0.0, 0.0, 1.0], 0.4)
    result = correct_side(
        frame_positions[0], frame_orientations[0], palm, palm_quat, calibration, "left"
    )
    np.testing.assert_allclose(result.positions[22], palm, atol=1e-9)
    np.testing.assert_allclose(result.orientations_xyzw[20], palm_quat, atol=1e-9)
    np.testing.assert_allclose(result.orientations_xyzw[22], palm_quat, atol=1e-9)
    assert np.linalg.norm(result.positions[18] - result.positions[16]) == pytest.approx(
        calibration.upper_length_m, abs=1e-9
    )
    assert np.linalg.norm(result.positions[20] - result.positions[18]) == pytest.approx(
        calibration.forearm_length_m, abs=1e-9
    )
    np.testing.assert_allclose(result.positions[[0, 1, 2, 3]], frame_positions[0, [0, 1, 2, 3]])


def test_correct_side_uses_wrist_pivot_and_copies_palm_orientation_exactly():
    positions, orientations, palm_positions, palm_orientations = _calibration_samples(count=8)
    calibration = fit_side_calibration(
        positions, orientations, palm_positions, palm_orientations, "left"
    )
    palm = np.array([0.30, 0.35, 1.40])
    palm_quat = quat_from_axis_angle([0.0, 0.0, 1.0], math.pi / 2.0)
    wrist_to_palm_distance = 0.10
    result = correct_side(
        positions[0],
        orientations[0],
        palm,
        palm_quat,
        calibration,
        "left",
        wrist_to_palm_distance_m=wrist_to_palm_distance,
    )
    expected_wrist = palm - quat_apply(
        palm_quat, [wrist_to_palm_distance, 0.0, 0.0]
    )
    np.testing.assert_allclose(result.positions[20], expected_wrist, atol=1.0e-9)
    np.testing.assert_allclose(result.positions[22], palm, atol=1.0e-9)
    np.testing.assert_allclose(result.orientations_xyzw[20], palm_quat, atol=1.0e-9)
    np.testing.assert_allclose(result.orientations_xyzw[22], palm_quat, atol=1.0e-9)
    assert result.wrist_palm_position_residual_m == pytest.approx(0.0, abs=1.0e-9)


def test_wrist_pivot_clamps_infeasible_endpoint_and_preserves_bone_lengths():
    positions, orientations, palm_positions, palm_orientations = _calibration_samples(count=8)
    calibration = fit_side_calibration(
        positions, orientations, palm_positions, palm_orientations, "left"
    )
    palm = np.array([2.0, 0.35, 1.40])
    palm_quat = quat_identity()
    wrist_to_palm_distance = 0.10
    result = correct_side(
        positions[0],
        orientations[0],
        palm,
        palm_quat,
        calibration,
        "left",
        wrist_to_palm_distance_m=wrist_to_palm_distance,
    )
    assert result.reach_clamped
    assert np.linalg.norm(result.positions[18] - result.positions[16]) == pytest.approx(
        calibration.upper_length_m, abs=1.0e-9
    )
    assert np.linalg.norm(result.positions[20] - result.positions[18]) == pytest.approx(
        calibration.forearm_length_m, abs=1.0e-9
    )
    reconstructed_palm = result.positions[20] + quat_apply(
        palm_quat, [wrist_to_palm_distance, 0.0, 0.0]
    )
    np.testing.assert_allclose(result.positions[22], reconstructed_palm, atol=1.0e-9)
    assert result.wrist_palm_position_residual_m > 1.0
    np.testing.assert_allclose(result.orientations_xyzw[20], palm_quat, atol=1.0e-9)
    np.testing.assert_allclose(result.orientations_xyzw[22], palm_quat, atol=1.0e-9)


def test_wrist_target_always_uses_palm_local_positive_x_axis():
    positions, orientations, palm_positions, palm_orientations = _calibration_samples(count=8)
    calibration = fit_side_calibration(
        positions, orientations, palm_positions, palm_orientations, "left"
    )
    palm = np.array([0.30, 0.35, 1.40])
    palm_quat = quat_from_axis_angle([0.0, 0.0, 1.0], math.pi / 2.0)
    result = correct_side(
        positions[0],
        orientations[0],
        palm,
        palm_quat,
        calibration,
        "left",
        wrist_to_palm_distance_m=0.10,
    )
    # A +90 degree yaw rotates palm-local +X to world +Y.  The wrist must
    # therefore be 10 cm along world -Y from the observed palm.
    np.testing.assert_allclose(result.positions[20], palm + [0.0, -0.10, 0.0], atol=1e-9)


def test_correct_side_keeps_original_elbow_branch_and_reports_clamp():
    positions, orientations, palm_positions, palm_orientations = _calibration_samples(count=8)
    calibration = fit_side_calibration(
        positions, orientations, palm_positions, palm_orientations, "left"
    )
    frame_positions = positions[0].copy()
    frame_orientations = orientations[0].copy()
    palm = np.array([3.0, 0.25, 1.4])
    result = correct_side(
        frame_positions,
        frame_orientations,
        palm,
        quat_identity(),
        calibration,
        "left",
    )
    assert result.reach_clamped
    assert result.wrist_palm_position_residual_m > 1.0
    assert result.positions[18, 1] > 0.0


def test_right_side_only_changes_right_arm_indices():
    positions, orientations, palm_positions, palm_orientations = _calibration_samples(
        side="right", count=8
    )
    calibration = fit_side_calibration(
        positions, orientations, palm_positions, palm_orientations, "right"
    )
    result = correct_side(
        positions[0], orientations[0], np.array([0.5, -0.25, 1.1]), quat_identity(), calibration, "right"
    )
    np.testing.assert_allclose(result.positions[list(LEFT)], positions[0, list(LEFT)])
    assert not np.allclose(result.positions[19], positions[0, 19])


def test_quaternion_helpers_handle_rotation_and_antiparallel_vectors():
    q = quat_from_axis_angle([0.0, 0.0, 1.0], math.pi / 2.0)
    np.testing.assert_allclose(quat_apply(q, [1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(quat_apply(q, [0.0, 0.0, 0.2]), [0.0, 0.0, 0.2], atol=1e-9)


def test_deterministic_1000_frame_replay_has_finite_continuous_outputs():
    positions, orientations, palm_positions, palm_orientations = _calibration_samples(
        count=8
    )
    calibration = fit_side_calibration(
        positions, orientations, palm_positions, palm_orientations, "left"
    )
    non_arm_indices = [index for index in range(24) if index not in LEFT]
    previous_elbow = None
    outputs = []
    for frame in range(1000):
        input_positions = positions[frame % len(positions)].copy()
        input_orientations = orientations[frame % len(orientations)].copy()
        palm = np.array(
            [
                0.30 + 0.07 * math.sin(frame * 0.013),
                0.25,
                0.95 + 0.04 * math.cos(frame * 0.017),
            ],
            dtype=float,
        )
        result = correct_side(
            input_positions,
            input_orientations,
            palm,
            quat_identity(),
            calibration,
            "left",
            previous_elbow_position=previous_elbow,
        )
        assert np.all(np.isfinite(result.positions))
        assert np.all(np.isfinite(result.orientations_xyzw))
        np.testing.assert_allclose(result.positions[22], palm, atol=1.0e-6)
        assert np.linalg.norm(result.positions[18] - result.positions[16]) == pytest.approx(
            calibration.upper_length_m, abs=1.0e-9
        )
        assert np.linalg.norm(result.positions[20] - result.positions[18]) == pytest.approx(
            calibration.forearm_length_m, abs=1.0e-9
        )
        np.testing.assert_allclose(
            result.positions[non_arm_indices], input_positions[non_arm_indices], atol=0.0
        )
        previous_elbow = result.positions[18].copy()
        outputs.append(result.positions.copy())

    assert len(outputs) == 1000
    # The branch reference must keep neighboring elbow solutions continuous.
    elbow_steps = np.linalg.norm(np.diff(np.asarray(outputs)[:, 18], axis=0), axis=1)
    assert float(np.max(elbow_steps)) < 0.02
