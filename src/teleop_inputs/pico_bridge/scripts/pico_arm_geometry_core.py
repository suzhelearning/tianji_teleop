#!/usr/bin/env python3
"""ROS-independent, side-neutral geometry for PICO arm calibration."""

from __future__ import annotations

from dataclasses import dataclass
import enum
import math

import numpy as np


MAX_STATIC_TRANSITION_DIRECTION_ERROR_RAD = math.radians(30.0)
MAX_NEUTRAL_PAIR_POSITION_ERROR_M = 0.07


EPS = 1.0e-9


class ArmSide(str, enum.Enum):
    LEFT = "left"
    RIGHT = "right"


def _validated_side(side: ArmSide | str) -> ArmSide:
    try:
        return ArmSide(side)
    except ValueError as error:
        raise ValueError(f"arm_side_invalid:{side}") from error


@dataclass(frozen=True)
class CircleFit3d:
    center_m: tuple[float, float, float]
    normal: tuple[float, float, float]
    radius_m: float
    radius_std_m: float
    residual_rms_m: float
    angular_coverage_rad: float
    sample_count: int


@dataclass(frozen=True)
class ArmGeometryGate:
    min_straight_samples: int = 50
    min_flex_samples: int = 100
    min_angular_coverage_rad: float = math.radians(90.0)
    max_forearm_length_std_m: float = 0.012
    max_circle_rms_m: float = 0.010
    max_straight_reach_rms_m: float = 0.020
    max_shoulder_motion_rms_m: float = 0.015
    max_elbow_hint_error_m: float = 0.100
    min_upper_arm_length_m: float = 0.15
    max_upper_arm_length_m: float = 0.45
    min_forearm_length_m: float = 0.15
    max_forearm_length_m: float = 0.40


@dataclass(frozen=True)
class ArmGeometryResult:
    valid: bool
    shoulder_anchor_pico_m: tuple[float, float, float]
    elbow_center_pico_m: tuple[float, float, float]
    upper_arm_length_m: float
    forearm_length_m: float
    forearm_length_std_m: float
    elbow_plane_normal_pico: tuple[float, float, float]
    circle_residual_rms_m: float
    straight_reach_residual_rms_m: float
    shoulder_motion_rms_m: float
    elbow_hint_error_m: float
    angular_coverage_rad: float
    straight_sample_count: int
    raw_flex_sample_count: int
    flex_sample_count: int
    covariance_8x8: tuple[tuple[float, ...], ...]
    rejection_reasons: tuple[str, ...]


@dataclass(frozen=True)
class ArmTwoPoseGate:
    min_samples_per_pose: int = 50
    max_straight_repeat_error_m: float = 0.020
    max_static_distance_rms_m: float = 0.012
    max_right_angle_error_rad: float = math.radians(15.0)
    max_shoulder_motion_rms_m: float = 0.015
    max_wrist_motion_rms_m: float = 0.015
    max_length_std_m: float = 0.015
    min_upper_arm_length_m: float = 0.15
    max_upper_arm_length_m: float = 0.45
    min_forearm_length_m: float = 0.15
    max_forearm_length_m: float = 0.40


@dataclass(frozen=True)
class ArmTwoPoseResult:
    valid: bool
    shoulder_anchor_pico_m: tuple[float, float, float]
    elbow_center_pico_m: tuple[float, float, float]
    upper_arm_length_m: float
    forearm_length_m: float
    length_std_m: float
    straight_total_length_m: float
    right_angle_distance_m: float
    straight_repeat_error_m: float
    straight_residual_rms_m: float
    right_angle_residual_rms_m: float
    right_angle_error_rad: float
    shoulder_motion_rms_m: float
    wrist_motion_rms_m: float
    straight_1_sample_count: int
    straight_2_sample_count: int
    right_angle_sample_count: int
    covariance_8x8: tuple[tuple[float, ...], ...]
    rejection_reasons: tuple[str, ...]


@dataclass(frozen=True)
class ArmStaticPoseGate:
    """Quality gates for the shoulder-free three-static-pose solution."""

    min_samples_per_pose: int = 50
    max_straight_pair_position_error_m: float = 0.10
    max_upper_repeat_error_m: float = 0.06
    max_neutral_pair_position_error_m: float = MAX_NEUTRAL_PAIR_POSITION_ERROR_M
    max_forearm_repeat_error_m: float = 0.04
    max_static_motion_rms_m: float = 0.015
    max_transition_direction_error_rad: float = MAX_STATIC_TRANSITION_DIRECTION_ERROR_RAD
    max_length_closure_error_m: float = 0.025
    max_shoulder_motion_rms_m: float = 0.015
    max_length_std_m: float = 0.030
    min_upper_arm_length_m: float = 0.15
    max_upper_arm_length_m: float = 0.45
    min_forearm_length_m: float = 0.15
    max_forearm_length_m: float = 0.40


@dataclass(frozen=True)
class ArmStaticPoseResult:
    valid: bool
    shoulder_anchor_pico_m: tuple[float, float, float]
    elbow_center_pico_m: tuple[float, float, float]
    upper_arm_length_m: float
    forearm_length_m: float
    length_std_m: float
    selected_straight_groups: tuple[str, str]
    discarded_straight_group: str
    straight_pair_position_error_m: float
    upper_repeat_error_m: float
    neutral_pair_position_error_m: float
    forearm_repeat_error_m: float
    static_motion_rms_m: float
    right_angle_error_rad: float | None
    raw_smpl_diagnostic_available: bool
    transition_direction_error_rad: float
    length_closure_error_m: float
    shoulder_motion_rms_m: float
    straight_1_sample_count: int
    straight_2_sample_count: int
    validation_sample_count: int
    neutral_start_sample_count: int
    neutral_return_sample_count: int
    right_angle_sample_count: int
    covariance_8x8: tuple[tuple[float, ...], ...]
    rejection_reasons: tuple[str, ...]


def _points(values: np.ndarray, name: str, minimum_count: int = 3) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    if result.ndim != 2 or result.shape[1:] != (3,):
        raise ValueError(f"{name} must have shape (N, 3), got {result.shape}")
    if result.shape[0] < minimum_count:
        raise ValueError(f"{name} requires at least {minimum_count} samples")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return result


def _deterministic_normal(vector: np.ndarray) -> np.ndarray:
    result = vector / max(float(np.linalg.norm(vector)), EPS)
    dominant = int(np.argmax(np.abs(result)))
    if result[dominant] < 0.0:
        result = -result
    return result


def _circle_fit_once(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, float, np.ndarray, np.ndarray]:
    origin = np.mean(points, axis=0)
    _, singular_values, vh = np.linalg.svd(points - origin, full_matrices=False)
    if singular_values.size < 2 or singular_values[1] <= EPS:
        raise ValueError("wrist trajectory is rank deficient")
    basis = vh[:2].T
    normal = _deterministic_normal(vh[2])
    coordinates = (points - origin) @ basis
    design = np.column_stack(
        [coordinates[:, 0], coordinates[:, 1], np.ones(len(coordinates))]
    )
    rhs = -(coordinates[:, 0] ** 2 + coordinates[:, 1] ** 2)
    coefficients, _, rank, _ = np.linalg.lstsq(design, rhs, rcond=None)
    if rank < 3:
        raise ValueError("wrist circle fit is rank deficient")
    center_2d = -0.5 * coefficients[:2]
    radius_sq = float(np.dot(center_2d, center_2d) - coefficients[2])
    if radius_sq <= EPS:
        raise ValueError("wrist circle radius is not positive")
    center = origin + basis @ center_2d
    radius = math.sqrt(radius_sq)
    radial = np.linalg.norm(coordinates - center_2d, axis=1)
    radial_residuals = radial - radius
    plane_offsets = (points - origin) @ normal
    residuals = np.sqrt(radial_residuals**2 + plane_offsets**2)
    return center, normal, radius, residuals, coordinates - center_2d


def _circular_coverage(vectors_2d: np.ndarray) -> float:
    angles = np.mod(np.arctan2(vectors_2d[:, 1], vectors_2d[:, 0]), 2.0 * math.pi)
    ordered = np.sort(angles)
    gaps = np.diff(np.concatenate([ordered, [ordered[0] + 2.0 * math.pi]]))
    return float(2.0 * math.pi - np.max(gaps))


def fit_circle_3d(wrist_positions: np.ndarray) -> CircleFit3d:
    """Fit a robust 3D circle and report the actually excited arc."""

    points = _points(wrist_positions, "wrist_positions", minimum_count=6)
    center, normal, radius, residuals, _ = _circle_fit_once(points)
    median = float(np.median(residuals))
    mad = float(np.median(np.abs(residuals - median)))
    robust_sigma = max(1.4826 * mad, 0.001)
    keep = np.abs(residuals - median) <= 4.0 * robust_sigma
    if int(np.count_nonzero(keep)) >= max(6, int(math.ceil(0.75 * len(points)))):
        points = points[keep]
        center, normal, radius, residuals, vectors_2d = _circle_fit_once(points)
    else:
        _, _, _, _, vectors_2d = _circle_fit_once(points)
    rng = np.random.default_rng(0)
    bootstrap_radii: list[float] = []
    for _ in range(64):
        indices = rng.integers(0, len(points), size=len(points))
        try:
            _, _, bootstrap_radius, _, _ = _circle_fit_once(points[indices])
        except ValueError:
            continue
        if np.isfinite(bootstrap_radius):
            bootstrap_radii.append(float(bootstrap_radius))
    radius_std = (
        float(np.std(bootstrap_radii, ddof=1))
        if len(bootstrap_radii) >= 48
        else math.inf
    )
    return CircleFit3d(
        center_m=tuple(float(value) for value in center),
        normal=tuple(float(value) for value in normal),
        radius_m=float(radius),
        radius_std_m=radius_std,
        residual_rms_m=float(np.sqrt(np.mean(residuals**2))),
        angular_coverage_rad=_circular_coverage(vectors_2d),
        sample_count=int(len(points)),
    )


def _motion_rms(points: np.ndarray) -> float:
    center = np.median(points, axis=0)
    return float(np.sqrt(np.mean(np.sum((points - center) ** 2, axis=1))))


def _distance_samples(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    if len(first) != len(second):
        raise ValueError("paired position sample counts must match")
    return np.linalg.norm(second - first, axis=1)


def _right_angle_samples(
    shoulders: np.ndarray,
    elbows: np.ndarray,
    wrists: np.ndarray,
) -> np.ndarray:
    if len(shoulders) != len(elbows) or len(elbows) != len(wrists):
        raise ValueError("right-angle shoulder/elbow/wrist sample counts must match")
    upper = elbows - shoulders
    forearm = wrists - elbows
    denominator = np.linalg.norm(upper, axis=1) * np.linalg.norm(forearm, axis=1)
    if np.any(denominator <= EPS):
        raise ValueError("right-angle raw arm vectors must be non-zero")
    cosine = np.sum(upper * forearm, axis=1) / denominator
    return np.arccos(np.clip(cosine, -1.0, 1.0))


def _solve_two_pose_lengths(total_length: float, diagonal_length: float) -> tuple[float, float]:
    discriminant = 2.0 * diagonal_length**2 - total_length**2
    if discriminant < 0.0:
        raise ValueError("right_angle_geometry_inconsistent")
    difference = math.sqrt(discriminant)
    # The two scalar equations determine an unordered pair.  Human humerus
    # length is used only to label the larger root as upper arm; neither raw
    # PICO model length enters the numerical estimate.
    return 0.5 * (total_length + difference), 0.5 * (total_length - difference)


def _median_position(points: np.ndarray) -> np.ndarray:
    return np.median(points, axis=0)


def _closest_named_pair(
    positions: dict[str, np.ndarray],
) -> tuple[tuple[str, str], str, float]:
    names = tuple(positions)
    if len(names) != 3:
        raise ValueError("exactly three straight pose groups are required")
    candidates: list[tuple[float, str, str]] = []
    for first_index, first in enumerate(names):
        for second in names[first_index + 1 :]:
            candidates.append(
                (float(np.linalg.norm(positions[first] - positions[second])), first, second)
            )
    distance, first, second = min(candidates, key=lambda value: (value[0], value[1], value[2]))
    discarded = next(name for name in names if name not in {first, second})
    return (first, second), discarded, distance


def _angle_between(first: np.ndarray, second: np.ndarray) -> float:
    denominator = float(np.linalg.norm(first) * np.linalg.norm(second))
    if denominator <= EPS:
        raise ValueError("static pose transition vector must be non-zero")
    cosine = float(np.dot(first, second) / denominator)
    return float(math.acos(np.clip(cosine, -1.0, 1.0)))


def solve_arm_static_pose_geometry(
    neutral_start_wrist_positions: np.ndarray,
    neutral_return_wrist_positions: np.ndarray,
    straight_1_wrist_positions: np.ndarray,
    straight_2_wrist_positions: np.ndarray,
    validation_wrist_positions: np.ndarray,
    right_angle_wrist_positions: np.ndarray,
    right_angle_shoulder_positions: np.ndarray,
    right_angle_raw_elbow_positions: np.ndarray,
    right_angle_raw_wrist_positions: np.ndarray,
    *,
    side: ArmSide | str,
    gate: ArmStaticPoseGate | None = None,
) -> ArmStaticPoseResult:
    """Solve arm lengths without using the PICO shoulder as a length anchor.

    The three prescribed static configurations are

    ``neutral = S + down * (U + F)``,
    ``straight = S + forward * (U + F)``, and
    ``right_angle = S + down * U + forward * F``.

    Consequently ``straight - right_angle`` contains only ``U`` and
    ``right_angle - neutral`` contains only ``F``.  Their ideal direction is
    ``forward - down`` with norm ``sqrt(2)``.  The unknown shoulder position
    therefore cancels exactly.
    """

    _validated_side(side)
    active_gate = gate or ArmStaticPoseGate()
    neutral_start = _points(neutral_start_wrist_positions, "neutral_start_wrists")
    neutral_return = _points(neutral_return_wrist_positions, "neutral_return_wrists")
    straight_groups = {
        "straight_1": _points(straight_1_wrist_positions, "straight_1_wrists"),
        "straight_2": _points(straight_2_wrist_positions, "straight_2_wrists"),
        "validation": _points(validation_wrist_positions, "validation_wrists"),
    }
    right_wrist = _points(right_angle_wrist_positions, "right_angle_wrists")
    raw_smpl_diagnostic_available = True
    try:
        right_shoulder = _points(
            right_angle_shoulder_positions, "right_angle_shoulders"
        )
        raw_elbow = _points(
            right_angle_raw_elbow_positions, "right_angle_raw_elbows"
        )
        raw_wrist = _points(
            right_angle_raw_wrist_positions, "right_angle_raw_wrists"
        )
        if not (len(right_shoulder) == len(raw_elbow) == len(raw_wrist)):
            raise ValueError("right-angle raw arm sample counts must match")
    except (TypeError, ValueError):
        raw_smpl_diagnostic_available = False
        right_shoulder = np.empty((0, 3), dtype=float)
        raw_elbow = np.empty((0, 3), dtype=float)
        raw_wrist = np.empty((0, 3), dtype=float)

    neutral_medians = {
        "neutral_start": _median_position(neutral_start),
        "neutral_return": _median_position(neutral_return),
    }
    straight_medians = {
        name: _median_position(points) for name, points in straight_groups.items()
    }
    selected, discarded, straight_pair_error = _closest_named_pair(straight_medians)
    right_median = _median_position(right_wrist)

    upper_vectors = [straight_medians[name] - right_median for name in selected]
    forearm_vectors = [
        right_median - neutral_medians["neutral_start"],
        right_median - neutral_medians["neutral_return"],
    ]
    upper_estimates = np.asarray(
        [np.linalg.norm(vector) / math.sqrt(2.0) for vector in upper_vectors]
    )
    forearm_estimates = np.asarray(
        [np.linalg.norm(vector) / math.sqrt(2.0) for vector in forearm_vectors]
    )
    upper_length = float(np.mean(upper_estimates))
    forearm_length = float(np.mean(forearm_estimates))
    upper_repeat_error = float(np.ptp(upper_estimates))
    forearm_repeat_error = float(np.ptp(forearm_estimates))
    neutral_pair_error = float(
        np.linalg.norm(
            neutral_medians["neutral_start"] - neutral_medians["neutral_return"]
        )
    )
    mean_upper_vector = np.mean(upper_vectors, axis=0)
    mean_forearm_vector = np.mean(forearm_vectors, axis=0)
    transition_direction_error = _angle_between(mean_upper_vector, mean_forearm_vector)
    mean_straight = np.mean([straight_medians[name] for name in selected], axis=0)
    mean_neutral = np.mean(list(neutral_medians.values()), axis=0)
    direct_total = float(np.linalg.norm(mean_straight - mean_neutral) / math.sqrt(2.0))
    closure_error = abs(direct_total - upper_length - forearm_length)

    selected_clouds = [straight_groups[name] for name in selected]
    static_motion = max(
        _motion_rms(neutral_start),
        _motion_rms(neutral_return),
        *(_motion_rms(points) for points in selected_clouds),
        _motion_rms(right_wrist),
    )
    try:
        if not raw_smpl_diagnostic_available:
            raise ValueError("raw SMPL diagnostics unavailable")
        raw_angles = _right_angle_samples(right_shoulder, raw_elbow, raw_wrist)
        right_angle_error: float | None = float(
            abs(np.median(raw_angles) - 0.5 * math.pi)
        )
    except ValueError:
        raw_smpl_diagnostic_available = False
        right_angle_error = None
    shoulder_motion = (
        _motion_rms(right_shoulder) if raw_smpl_diagnostic_available else 0.0
    )

    reasons: list[str] = []
    counts = {
        "neutral_start": len(neutral_start),
        "neutral_return": len(neutral_return),
        "straight_1": len(straight_groups["straight_1"]),
        "straight_2": len(straight_groups["straight_2"]),
        "validation": len(straight_groups["validation"]),
        "right_angle": len(right_wrist),
    }
    for name, count in counts.items():
        if count < active_gate.min_samples_per_pose:
            reasons.append(f"{name}_sample_count_below_minimum")
    if straight_pair_error > active_gate.max_straight_pair_position_error_m:
        reasons.append("straight_pair_position_error_exceeded")
    if upper_repeat_error > active_gate.max_upper_repeat_error_m:
        reasons.append("upper_length_repeat_error_exceeded")
    if neutral_pair_error > active_gate.max_neutral_pair_position_error_m:
        reasons.append("neutral_pair_position_error_exceeded")
    if forearm_repeat_error > active_gate.max_forearm_repeat_error_m:
        reasons.append("forearm_length_repeat_error_exceeded")
    if static_motion > active_gate.max_static_motion_rms_m:
        reasons.append("static_pose_motion_exceeded")
    if transition_direction_error > active_gate.max_transition_direction_error_rad:
        reasons.append("static_transition_direction_error_exceeded")
    if closure_error > active_gate.max_length_closure_error_m:
        reasons.append("length_closure_error_exceeded")
    rng = np.random.default_rng(0)
    bootstrap_lengths: list[tuple[float, float]] = []
    for _ in range(128):
        sampled_neutral = [
            _median_position(points[rng.integers(0, len(points), len(points))])
            for points in (neutral_start, neutral_return)
        ]
        sampled_straight = {
            name: _median_position(points[rng.integers(0, len(points), len(points))])
            for name, points in straight_groups.items()
        }
        sampled_selected, _, _ = _closest_named_pair(sampled_straight)
        sampled_right = _median_position(
            right_wrist[rng.integers(0, len(right_wrist), len(right_wrist))]
        )
        sampled_upper = np.mean(
            [
                np.linalg.norm(sampled_straight[name] - sampled_right) / math.sqrt(2.0)
                for name in sampled_selected
            ]
        )
        sampled_forearm = np.mean(
            [
                np.linalg.norm(sampled_right - neutral) / math.sqrt(2.0)
                for neutral in sampled_neutral
            ]
        )
        bootstrap_lengths.append((float(sampled_upper), float(sampled_forearm)))
    bootstrap = np.asarray(bootstrap_lengths)
    bootstrap_std = float(np.max(np.std(bootstrap, axis=0, ddof=1)))
    length_std = max(
        bootstrap_std,
        0.5 * upper_repeat_error,
        0.5 * forearm_repeat_error,
    )
    if length_std > active_gate.max_length_std_m:
        reasons.append("length_uncertainty_exceeded")
    if not active_gate.min_upper_arm_length_m <= upper_length <= active_gate.max_upper_arm_length_m:
        reasons.append("upper_arm_length_out_of_range")
    if not active_gate.min_forearm_length_m <= forearm_length <= active_gate.max_forearm_length_m:
        reasons.append("forearm_length_out_of_range")

    # These points are retained as diagnostics only.  They do not participate
    # in either length estimate because raw PICO shoulder translation is not a
    # trusted endpoint reference.
    if raw_smpl_diagnostic_available:
        shoulder_anchor = _median_position(right_shoulder)
        raw_upper_direction = _median_position(raw_elbow - right_shoulder)
        raw_upper_norm = float(np.linalg.norm(raw_upper_direction))
    else:
        shoulder_anchor = right_median.copy()
        raw_upper_direction = np.zeros(3, dtype=float)
        raw_upper_norm = 0.0
    if raw_upper_norm <= EPS:
        raw_smpl_diagnostic_available = False
        right_angle_error = None
        elbow_center = shoulder_anchor.copy()
    else:
        elbow_center = shoulder_anchor + upper_length * raw_upper_direction / raw_upper_norm
    shoulder_variance = max(shoulder_motion**2, 1.0e-8)
    length_variance = max(length_std**2, 1.0e-8)
    covariance = np.diag(
        [
            shoulder_variance,
            shoulder_variance,
            shoulder_variance,
            shoulder_variance + length_variance,
            shoulder_variance + length_variance,
            shoulder_variance + length_variance,
            length_variance,
            length_variance,
        ]
    )
    return ArmStaticPoseResult(
        valid=not reasons,
        shoulder_anchor_pico_m=tuple(float(value) for value in shoulder_anchor),
        elbow_center_pico_m=tuple(float(value) for value in elbow_center),
        upper_arm_length_m=upper_length,
        forearm_length_m=forearm_length,
        length_std_m=length_std,
        selected_straight_groups=selected,
        discarded_straight_group=discarded,
        straight_pair_position_error_m=straight_pair_error,
        upper_repeat_error_m=upper_repeat_error,
        neutral_pair_position_error_m=neutral_pair_error,
        forearm_repeat_error_m=forearm_repeat_error,
        static_motion_rms_m=static_motion,
        right_angle_error_rad=right_angle_error,
        raw_smpl_diagnostic_available=raw_smpl_diagnostic_available,
        transition_direction_error_rad=transition_direction_error,
        length_closure_error_m=closure_error,
        shoulder_motion_rms_m=shoulder_motion,
        straight_1_sample_count=counts["straight_1"],
        straight_2_sample_count=counts["straight_2"],
        validation_sample_count=counts["validation"],
        neutral_start_sample_count=counts["neutral_start"],
        neutral_return_sample_count=counts["neutral_return"],
        right_angle_sample_count=counts["right_angle"],
        covariance_8x8=tuple(
            tuple(float(value) for value in row) for row in covariance
        ),
        rejection_reasons=tuple(reasons),
    )


def solve_arm_two_pose_geometry(
    straight_1_shoulder_positions: np.ndarray,
    straight_1_wrist_positions: np.ndarray,
    straight_2_shoulder_positions: np.ndarray,
    straight_2_wrist_positions: np.ndarray,
    right_angle_shoulder_positions: np.ndarray,
    right_angle_wrist_positions: np.ndarray,
    right_angle_raw_elbow_positions: np.ndarray,
    right_angle_raw_wrist_positions: np.ndarray,
    *,
    side: ArmSide | str,
    gate: ArmTwoPoseGate | None = None,
) -> ArmTwoPoseResult:
    """Estimate arm lengths from straight and nominal 90-degree static poses."""

    _validated_side(side)
    active_gate = gate or ArmTwoPoseGate()
    s1 = _points(straight_1_shoulder_positions, "straight_1_shoulders")
    w1 = _points(straight_1_wrist_positions, "straight_1_wrists")
    s2 = _points(straight_2_shoulder_positions, "straight_2_shoulders")
    w2 = _points(straight_2_wrist_positions, "straight_2_wrists")
    sr = _points(right_angle_shoulder_positions, "right_angle_shoulders")
    wr = _points(right_angle_wrist_positions, "right_angle_wrists")
    er = _points(right_angle_raw_elbow_positions, "right_angle_raw_elbows")
    rr = _points(right_angle_raw_wrist_positions, "right_angle_raw_wrists")

    straight_1 = _distance_samples(s1, w1)
    straight_2 = _distance_samples(s2, w2)
    diagonal = _distance_samples(sr, wr)
    raw_angles = _right_angle_samples(sr, er, rr)
    straight_1_length = float(np.median(straight_1))
    straight_2_length = float(np.median(straight_2))
    total_length = 0.5 * (straight_1_length + straight_2_length)
    diagonal_length = float(np.median(diagonal))
    repeat_error = abs(straight_1_length - straight_2_length)
    straight_residual = float(
        np.sqrt(
            np.mean(
                (np.concatenate([straight_1, straight_2]) - total_length) ** 2
            )
        )
    )
    diagonal_residual = float(
        np.sqrt(np.mean((diagonal - diagonal_length) ** 2))
    )
    right_angle_error = float(abs(np.median(raw_angles) - 0.5 * math.pi))
    shoulder_motion = _motion_rms(sr)
    wrist_motion = _motion_rms(wr)

    reasons: list[str] = []
    try:
        upper_length, forearm_length = _solve_two_pose_lengths(
            total_length, diagonal_length
        )
    except ValueError:
        reasons.append("right_angle_geometry_inconsistent")
        upper_length = 0.5 * total_length
        forearm_length = 0.5 * total_length

    for count, label in (
        (len(s1), "straight_1"),
        (len(s2), "straight_2"),
        (len(sr), "right_angle"),
    ):
        if count < active_gate.min_samples_per_pose:
            reasons.append(f"{label}_sample_count_below_minimum")
    if repeat_error > active_gate.max_straight_repeat_error_m:
        reasons.append("straight_repeat_error_exceeded")
    if straight_residual > active_gate.max_static_distance_rms_m:
        reasons.append("straight_static_residual_exceeded")
    if diagonal_residual > active_gate.max_static_distance_rms_m:
        reasons.append("right_angle_static_residual_exceeded")
    if right_angle_error > active_gate.max_right_angle_error_rad:
        reasons.append("right_angle_pose_error_exceeded")
    if shoulder_motion > active_gate.max_shoulder_motion_rms_m:
        reasons.append("right_angle_shoulder_motion_exceeded")
    if wrist_motion > active_gate.max_wrist_motion_rms_m:
        reasons.append("right_angle_wrist_motion_exceeded")

    rng = np.random.default_rng(0)
    bootstrap_lengths: list[tuple[float, float]] = []
    for _ in range(128):
        sample_s1 = rng.choice(straight_1, size=len(straight_1), replace=True)
        sample_s2 = rng.choice(straight_2, size=len(straight_2), replace=True)
        sample_d = rng.choice(diagonal, size=len(diagonal), replace=True)
        sample_total = 0.5 * (float(np.median(sample_s1)) + float(np.median(sample_s2)))
        try:
            bootstrap_lengths.append(
                _solve_two_pose_lengths(sample_total, float(np.median(sample_d)))
            )
        except ValueError:
            continue
    if len(bootstrap_lengths) >= 96:
        bootstrap = np.asarray(bootstrap_lengths)
        length_std = float(np.max(np.std(bootstrap, axis=0, ddof=1)))
    else:
        length_std = math.inf
    if length_std > active_gate.max_length_std_m:
        reasons.append("length_uncertainty_exceeded")
    if not active_gate.min_upper_arm_length_m <= upper_length <= active_gate.max_upper_arm_length_m:
        reasons.append("upper_arm_length_out_of_range")
    if not active_gate.min_forearm_length_m <= forearm_length <= active_gate.max_forearm_length_m:
        reasons.append("forearm_length_out_of_range")

    shoulder_anchor = np.median(sr, axis=0)
    raw_upper_direction = np.median(er - sr, axis=0)
    raw_upper_norm = float(np.linalg.norm(raw_upper_direction))
    if raw_upper_norm <= EPS:
        raise ValueError("raw upper-arm direction must be non-zero")
    elbow_center = shoulder_anchor + upper_length * raw_upper_direction / raw_upper_norm

    shoulder_variance = max(shoulder_motion**2, 1.0e-8)
    elbow_variance = max(shoulder_variance + diagonal_residual**2, 1.0e-8)
    length_variance = max(length_std**2, 1.0e-8) if np.isfinite(length_std) else 1.0
    covariance = np.diag(
        [
            shoulder_variance,
            shoulder_variance,
            shoulder_variance,
            elbow_variance,
            elbow_variance,
            elbow_variance,
            length_variance,
            length_variance,
        ]
    )
    return ArmTwoPoseResult(
        valid=not reasons,
        shoulder_anchor_pico_m=tuple(float(value) for value in shoulder_anchor),
        elbow_center_pico_m=tuple(float(value) for value in elbow_center),
        upper_arm_length_m=upper_length,
        forearm_length_m=forearm_length,
        length_std_m=length_std,
        straight_total_length_m=total_length,
        right_angle_distance_m=diagonal_length,
        straight_repeat_error_m=repeat_error,
        straight_residual_rms_m=straight_residual,
        right_angle_residual_rms_m=diagonal_residual,
        right_angle_error_rad=right_angle_error,
        shoulder_motion_rms_m=shoulder_motion,
        wrist_motion_rms_m=wrist_motion,
        straight_1_sample_count=len(s1),
        straight_2_sample_count=len(s2),
        right_angle_sample_count=len(sr),
        covariance_8x8=tuple(
            tuple(float(value) for value in row) for row in covariance
        ),
        rejection_reasons=tuple(reasons),
    )


def solve_arm_geometry(
    straight_shoulder_positions: np.ndarray,
    straight_wrist_positions: np.ndarray,
    flex_shoulder_positions: np.ndarray,
    flex_wrist_positions: np.ndarray,
    flex_raw_elbow_positions: np.ndarray,
    *,
    side: ArmSide | str,
    gate: ArmGeometryGate | None = None,
) -> ArmGeometryResult:
    """Estimate left upper/forearm lengths and evaluate the quick-calibration gate."""

    _validated_side(side)
    active_gate = gate or ArmGeometryGate()
    straight_shoulders = _points(
        straight_shoulder_positions, "straight_shoulder_positions"
    )
    straight_wrists = _points(straight_wrist_positions, "straight_wrist_positions")
    flex_shoulders = _points(flex_shoulder_positions, "flex_shoulder_positions")
    flex_wrists = _points(flex_wrist_positions, "flex_wrist_positions", minimum_count=6)
    elbow_hints = _points(flex_raw_elbow_positions, "flex_raw_elbow_positions")
    if len(straight_shoulders) != len(straight_wrists):
        raise ValueError("straight shoulder/wrist sample counts must match")
    if len(flex_shoulders) != len(flex_wrists) or len(flex_wrists) != len(elbow_hints):
        raise ValueError("flex shoulder/wrist/elbow sample counts must match")

    circle = fit_circle_3d(flex_wrists)
    elbow_center = np.asarray(circle.center_m)
    # The upper-arm-at-torso flex stage is the only stage in which the shoulder
    # anchor is expected to remain fixed.  The straight reach is deliberately
    # reserved for a total-length cross-check because normal scapular motion
    # can shift the raw PICO shoulder during forward reach.
    shoulder_samples = flex_shoulders
    shoulder_anchor = np.median(flex_shoulders, axis=0)
    upper_length = float(np.linalg.norm(elbow_center - shoulder_anchor))
    forearm_length = float(circle.radius_m)
    total_length = upper_length + forearm_length

    reach_distances = np.linalg.norm(straight_wrists - straight_shoulders, axis=1)
    straight_residual = float(
        np.sqrt(np.mean((reach_distances - total_length) ** 2))
    )
    shoulder_offsets = shoulder_samples - shoulder_anchor
    shoulder_motion = float(
        np.sqrt(np.mean(np.sum(shoulder_offsets**2, axis=1)))
    )
    elbow_hint_error = float(
        np.linalg.norm(np.median(elbow_hints, axis=0) - elbow_center)
    )

    reasons: list[str] = []
    if len(straight_wrists) < active_gate.min_straight_samples:
        reasons.append("straight_sample_count_below_minimum")
    if circle.sample_count < active_gate.min_flex_samples:
        reasons.append("flex_sample_count_below_minimum")
    if circle.angular_coverage_rad < active_gate.min_angular_coverage_rad:
        reasons.append("angular_coverage_below_minimum")
    if circle.radius_std_m > active_gate.max_forearm_length_std_m:
        reasons.append("forearm_uncertainty_exceeded")
    if circle.residual_rms_m > active_gate.max_circle_rms_m:
        reasons.append("circle_residual_exceeded")
    if straight_residual > active_gate.max_straight_reach_rms_m:
        reasons.append("straight_reach_residual_exceeded")
    if shoulder_motion > active_gate.max_shoulder_motion_rms_m:
        reasons.append("shoulder_motion_exceeded")
    if elbow_hint_error > active_gate.max_elbow_hint_error_m:
        reasons.append("elbow_hint_disagreement_exceeded")
    if not active_gate.min_upper_arm_length_m <= upper_length <= active_gate.max_upper_arm_length_m:
        reasons.append("upper_arm_length_out_of_range")
    if not active_gate.min_forearm_length_m <= forearm_length <= active_gate.max_forearm_length_m:
        reasons.append("forearm_length_out_of_range")

    shoulder_variance = max(shoulder_motion**2, 1.0e-8)
    elbow_variance = max(circle.residual_rms_m**2, 1.0e-8)
    upper_variance = max(shoulder_variance + elbow_variance, 1.0e-8)
    forearm_variance = max(circle.radius_std_m**2, 1.0e-8)
    covariance = np.diag(
        [
            shoulder_variance,
            shoulder_variance,
            shoulder_variance,
            elbow_variance,
            elbow_variance,
            elbow_variance,
            upper_variance,
            forearm_variance,
        ]
    )
    return ArmGeometryResult(
        valid=not reasons,
        shoulder_anchor_pico_m=tuple(float(value) for value in shoulder_anchor),
        elbow_center_pico_m=tuple(float(value) for value in elbow_center),
        upper_arm_length_m=upper_length,
        forearm_length_m=forearm_length,
        forearm_length_std_m=circle.radius_std_m,
        elbow_plane_normal_pico=circle.normal,
        circle_residual_rms_m=circle.residual_rms_m,
        straight_reach_residual_rms_m=straight_residual,
        shoulder_motion_rms_m=shoulder_motion,
        elbow_hint_error_m=elbow_hint_error,
        angular_coverage_rad=circle.angular_coverage_rad,
        straight_sample_count=int(len(straight_wrists)),
        raw_flex_sample_count=int(len(flex_wrists)),
        flex_sample_count=circle.sample_count,
        covariance_8x8=tuple(
            tuple(float(value) for value in row) for row in covariance
        ),
        rejection_reasons=tuple(reasons),
    )
