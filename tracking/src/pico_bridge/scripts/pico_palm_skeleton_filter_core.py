#!/usr/bin/env python3
"""Pure geometry for the fast PICO palm-constrained skeleton experiment."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


EPS = 1.0e-9
ELBOW_FLEXION_PLANE_MIN_SINE = 0.05
ELBOW_BODY_LEFT_SIGN_MIN_ABS_DOT = 0.10
DEFAULT_LEFT_SHOULDER_LOCAL_X_OFFSET_RAD = np.pi / 2.0
DEFAULT_RIGHT_SHOULDER_LOCAL_X_OFFSET_RAD = -np.pi / 2.0
IK_SHOULDER_FRAME_SEMANTICS = "local_x_right_multiply"
IK_ELBOW_FRAME_SEMANTICS = (
    "local_x_forearm_local_y_flexion_axis_body_left_signed_temporal_fallback"
)
JOINT_COUNT = 24
SIDE_INDICES = {
    "left": (16, 18, 20, 22),
    "right": (17, 19, 21, 23),
}


@dataclass(frozen=True)
class SideCalibration:
    upper_length_m: float
    forearm_length_m: float
    palm_to_wrist_offset_m: np.ndarray
    palm_to_wrist_quat_xyzw: np.ndarray


@dataclass(frozen=True)
class SideCorrection:
    positions: np.ndarray
    orientations_xyzw: np.ndarray
    reach_clamped: bool
    wrist_palm_position_residual_m: float


def adapt_shoulders_to_ik_frames(
    orientations_xyzw: np.ndarray,
    *,
    left_offset_rad: float = DEFAULT_LEFT_SHOULDER_LOCAL_X_OFFSET_RAD,
    right_offset_rad: float = DEFAULT_RIGHT_SHOULDER_LOCAL_X_OFFSET_RAD,
) -> np.ndarray:
    """Return shoulder orientations expressed in the YuShu IK frame convention.

    The offsets are fixed changes of basis about each shoulder's *local* X
    axis, so they right-multiply the incoming global joint orientation.  No
    other joint orientation is changed; joint positions are deliberately not
    part of this interface.
    """

    orientations = _finite_array(orientations_xyzw, "orientations_xyzw")
    if orientations.shape != (JOINT_COUNT, 4):
        raise ValueError(
            "orientations_xyzw must have shape (24, 4), "
            f"got {orientations.shape}"
        )
    offsets = {
        "left": float(left_offset_rad),
        "right": float(right_offset_rad),
    }
    if not all(np.isfinite(angle) for angle in offsets.values()):
        raise ValueError("shoulder frame offsets must be finite")

    adapted = orientations.copy()
    local_x = np.array([1.0, 0.0, 0.0], dtype=float)
    for side, angle in offsets.items():
        shoulder = SIDE_INDICES[side][0]
        offset = quat_from_axis_angle(local_x, angle)
        adapted[shoulder] = _same_hemisphere(
            quat_multiply(adapted[shoulder], offset),
            adapted[shoulder],
        )
    return adapted


def adapt_arm_frames_to_ik_frames(
    positions_xyz: np.ndarray,
    orientations_xyzw: np.ndarray,
    *,
    left_shoulder_offset_rad: float = DEFAULT_LEFT_SHOULDER_LOCAL_X_OFFSET_RAD,
    right_shoulder_offset_rad: float = DEFAULT_RIGHT_SHOULDER_LOCAL_X_OFFSET_RAD,
    previous_orientations_xyzw: np.ndarray | None = None,
) -> np.ndarray:
    """Return IK shoulder/elbow frame orientations without changing positions.

    Each elbow frame is reconstructed with local X along ``elbow -> wrist``
    and local Y along the shoulder-elbow-wrist flexion axis.  The body-left
    direction from right shoulder to left shoulder selects the hinge-axis
    sign for both arms.  When the arm is straight and the flexion plane is
    unobservable, projected body-left supplies the stable fallback.  Local Z
    completes a right-handed orthonormal frame.
    """

    positions = _finite_array(positions_xyz, "positions_xyz")
    if positions.shape != (JOINT_COUNT, 3):
        raise ValueError(
            "positions_xyz must have shape (24, 3), "
            f"got {positions.shape}"
        )
    adapted = adapt_shoulders_to_ik_frames(
        orientations_xyzw,
        left_offset_rad=left_shoulder_offset_rad,
        right_offset_rad=right_shoulder_offset_rad,
    )
    input_orientations = np.asarray(orientations_xyzw, dtype=float)
    previous_orientations = None
    if previous_orientations_xyzw is not None:
        previous_orientations = np.asarray(previous_orientations_xyzw, dtype=float)
        if previous_orientations.shape != (JOINT_COUNT, 4):
            raise ValueError(
                "previous_orientations_xyzw must have shape (24, 4), "
                f"got {previous_orientations.shape}"
            )
    body_left = _normalize_vector(
        positions[SIDE_INDICES["left"][0]]
        - positions[SIDE_INDICES["right"][0]],
        "right shoulder to left shoulder",
    )

    for shoulder, elbow, wrist, _ in SIDE_INDICES.values():
        local_x = _normalize_vector(
            positions[wrist] - positions[elbow], "elbow to wrist"
        )
        upper = _normalize_vector(
            positions[elbow] - positions[shoulder], "shoulder to elbow"
        )
        input_elbow_orientation = _normalize_quat(
            input_orientations[elbow], "skeleton elbow orientation"
        )
        input_rotation = quat_to_matrix(input_elbow_orientation)
        input_y = input_rotation[:, 1]

        hinge = np.cross(upper, local_x)
        hinge_norm = float(np.linalg.norm(hinge))
        if hinge_norm < ELBOW_FLEXION_PLANE_MIN_SINE:
            hinge = body_left - local_x * float(np.dot(body_left, local_x))
            hinge_norm = float(np.linalg.norm(hinge))
        if hinge_norm <= EPS:
            hinge = input_y - local_x * float(np.dot(input_y, local_x))
            hinge_norm = float(np.linalg.norm(hinge))
        if hinge_norm <= EPS:
            hinge = _orthogonal_unit(local_x)
        else:
            hinge /= hinge_norm
        sign_score = float(np.dot(hinge, body_left))
        if (
            abs(sign_score) < ELBOW_BODY_LEFT_SIGN_MIN_ABS_DOT
            and previous_orientations is not None
        ):
            previous_y = quat_to_matrix(
                _normalize_quat(
                    previous_orientations[elbow],
                    "previous skeleton elbow orientation",
                )
            )[:, 1]
            previous_score = float(np.dot(hinge, previous_y))
            if abs(previous_score) > EPS:
                sign_score = previous_score
        if abs(sign_score) <= EPS:
            input_score = float(np.dot(hinge, input_y))
            if abs(input_score) > EPS:
                sign_score = input_score
        if sign_score < 0.0:
            hinge = -hinge

        local_z = _normalize_vector(np.cross(local_x, hinge), "elbow local Z")
        local_y = _normalize_vector(np.cross(local_z, local_x), "elbow local Y")
        elbow_rotation = np.column_stack((local_x, local_y, local_z))
        adapted[elbow] = _same_hemisphere(
            rotation_to_quat(elbow_rotation), input_elbow_orientation
        )
    return adapted


def fit_side_calibration(
    skeleton_positions: np.ndarray,
    skeleton_orientations_xyzw: np.ndarray,
    palm_positions: np.ndarray,
    palm_orientations_xyzw: np.ndarray,
    side: str,
) -> SideCalibration:
    """Estimate fixed side geometry from paired PICO skeleton/palm samples."""

    positions = _finite_array(skeleton_positions, "skeleton_positions")
    orientations = _finite_array(
        skeleton_orientations_xyzw, "skeleton_orientations_xyzw"
    )
    palms = _finite_array(palm_positions, "palm_positions")
    palm_rotations = _finite_array(palm_orientations_xyzw, "palm_orientations_xyzw")
    if side not in SIDE_INDICES:
        raise ValueError(f"side must be left or right, got {side!r}")
    if positions.ndim != 3 or positions.shape[1:] != (JOINT_COUNT, 3):
        raise ValueError(
            "skeleton_positions must have shape (N, 24, 3), "
            f"got {positions.shape}"
        )
    if orientations.ndim != 3 or orientations.shape[1:] != (JOINT_COUNT, 4):
        raise ValueError(
            "skeleton_orientations_xyzw must have shape (N, 24, 4), "
            f"got {orientations.shape}"
        )
    if palms.ndim != 2 or palms.shape[1:] != (3,):
        raise ValueError(f"palm_positions must have shape (N, 3), got {palms.shape}")
    if palm_rotations.ndim != 2 or palm_rotations.shape[1:] != (4,):
        raise ValueError(
            "palm_orientations_xyzw must have shape (N, 4), "
            f"got {palm_rotations.shape}"
        )
    sample_count = positions.shape[0]
    if (
        orientations.shape[0] != sample_count
        or palms.shape[0] != sample_count
        or palm_rotations.shape[0] != sample_count
    ):
        raise ValueError("paired calibration arrays must have the same sample count")
    if sample_count == 0:
        raise ValueError("at least one paired calibration sample is required")

    shoulder, elbow, wrist, _ = SIDE_INDICES[side]
    normalized_skeleton_rotations = np.asarray(
        [_normalize_quat(value, "skeleton orientation") for value in orientations.reshape(-1, 4)],
        dtype=float,
    ).reshape(orientations.shape)
    normalized_palm_rotations = np.asarray(
        [_normalize_quat(value, "palm orientation") for value in palm_rotations],
        dtype=float,
    )

    upper_lengths = np.linalg.norm(
        positions[:, elbow] - positions[:, shoulder], axis=1
    )
    forearm_lengths = np.linalg.norm(
        positions[:, wrist] - positions[:, elbow], axis=1
    )
    if np.any(upper_lengths <= EPS) or np.any(forearm_lengths <= EPS):
        raise ValueError("calibration contains a zero-length arm segment")

    offsets = []
    relative_rotations = []
    for index in range(sample_count):
        palm_rotation = quat_to_matrix(normalized_palm_rotations[index])
        wrist_rotation = quat_to_matrix(normalized_skeleton_rotations[index, wrist])
        offsets.append(
            palm_rotation.T @ (positions[index, wrist] - palms[index])
        )
        relative_rotations.append(palm_rotation.T @ wrist_rotation)

    return SideCalibration(
        upper_length_m=float(np.median(upper_lengths)),
        forearm_length_m=float(np.median(forearm_lengths)),
        palm_to_wrist_offset_m=np.median(np.asarray(offsets), axis=0),
        palm_to_wrist_quat_xyzw=rotation_to_quat(
            _mean_rotation(relative_rotations)
        ),
    )


def correct_side(
    skeleton_positions: np.ndarray,
    skeleton_orientations_xyzw: np.ndarray,
    palm_position: np.ndarray,
    palm_orientation_xyzw: np.ndarray,
    calibration: SideCalibration,
    side: str,
    previous_elbow_position: np.ndarray | None = None,
    ratio_max_stretch: float = 0.995,
    epsilon_m: float = 1.0e-6,
    wrist_to_palm_distance_m: float | None = None,
) -> SideCorrection:
    """Reconstruct one arm while leaving all other joints untouched.

    ``wrist_to_palm_distance_m`` is the calibrated scalar distance from wrist
    to palm.  The runtime contract fixes the palm to wrist-local positive X,
    so the TCP palm observation yields the requested wrist as::

        p_wrist_requested = p_palm - R_palm @ [distance, 0, 0]

    The wrist orientation is copied from the TCP palm orientation.  The wrist
    target is projected to the fixed two-link reachable interval before the
    elbow triangle is solved.  A clamped frame therefore preserves both arm
    segment lengths and publishes the best reachable skeleton candidate; the
    TCP endpoint mismatch is reported in ``wrist_palm_position_residual_m``.
    If no pivot artifact is supplied, the legacy paired-sample position offset
    remains available as an explicit degraded fallback.
    """

    positions = _finite_array(skeleton_positions, "skeleton_positions")
    orientations = _finite_array(
        skeleton_orientations_xyzw, "skeleton_orientations_xyzw"
    )
    palm = _finite_vector(palm_position, "palm_position", 3)
    palm_quat = _normalize_quat(palm_orientation_xyzw, "palm_orientation_xyzw")
    if side not in SIDE_INDICES:
        raise ValueError(f"side must be left or right, got {side!r}")
    if positions.shape != (JOINT_COUNT, 3):
        raise ValueError(f"skeleton_positions must have shape (24, 3), got {positions.shape}")
    if orientations.shape != (JOINT_COUNT, 4):
        raise ValueError(
            "skeleton_orientations_xyzw must have shape (24, 4), "
            f"got {orientations.shape}"
        )
    if not np.isfinite(ratio_max_stretch) or ratio_max_stretch <= 0.0:
        raise ValueError("ratio_max_stretch must be finite and positive")
    if calibration.upper_length_m <= EPS or calibration.forearm_length_m <= EPS:
        raise ValueError("calibration arm lengths must be positive")
    wrist_to_palm_distance = None
    if wrist_to_palm_distance_m is not None:
        wrist_to_palm_distance = float(wrist_to_palm_distance_m)
        if (
            not np.isfinite(wrist_to_palm_distance)
            or wrist_to_palm_distance <= EPS
        ):
            raise ValueError("wrist_to_palm_distance_m must be finite and positive")

    shoulder, elbow, wrist, hand = SIDE_INDICES[side]
    output_positions = positions.copy()
    output_orientations = np.asarray(
        [_normalize_quat(value, "skeleton orientation") for value in orientations],
        dtype=float,
    )
    palm_rotation = quat_to_matrix(palm_quat)
    if wrist_to_palm_distance is None:
        wrist_offset_world = palm_rotation @ calibration.palm_to_wrist_offset_m
        wrist_target = palm + wrist_offset_world
    else:
        wrist_to_palm_local = np.array(
            [wrist_to_palm_distance, 0.0, 0.0], dtype=float
        )
        wrist_target = palm - palm_rotation @ wrist_to_palm_local
    # The calibrated TCP palm and reconstructed wrist intentionally share the
    # same orientation.  The old learned relative quaternion belonged to the
    # raw PICO skeleton contract and must not rotate a TCP-derived wrist.
    wrist_rotation = palm_quat.copy()

    shoulder_position = positions[shoulder]
    shoulder_to_wrist = wrist_target - shoulder_position
    distance = float(np.linalg.norm(shoulder_to_wrist))
    if distance <= EPS:
        shoulder_to_wrist = positions[wrist] - shoulder_position
        distance = float(np.linalg.norm(shoulder_to_wrist))
    if distance <= EPS:
        shoulder_to_wrist = np.array([1.0, 0.0, 0.0], dtype=float)
        distance = 1.0
    direction = shoulder_to_wrist / distance
    minimum_distance = abs(calibration.upper_length_m - calibration.forearm_length_m) + epsilon_m
    maximum_distance = (
        calibration.upper_length_m + calibration.forearm_length_m
    ) * ratio_max_stretch
    clamped_distance = min(max(distance, minimum_distance), maximum_distance)
    reach_clamped = not np.isclose(clamped_distance, distance, atol=1.0e-12)
    wrist_target = shoulder_position + direction * clamped_distance

    elbow_reference = positions[elbow]
    if previous_elbow_position is not None:
        previous = _finite_vector(previous_elbow_position, "previous_elbow_position", 3)
        if np.linalg.norm(elbow_reference - shoulder_position) <= EPS:
            elbow_reference = previous
    try:
        elbow_target = solve_triangle(
            shoulder_position,
            wrist_target,
            calibration.upper_length_m,
            calibration.forearm_length_m,
            elbow_reference,
        )
    except ValueError:
        if previous_elbow_position is None:
            raise
        elbow_target = solve_triangle(
            shoulder_position,
            wrist_target,
            calibration.upper_length_m,
            calibration.forearm_length_m,
            previous_elbow_position,
        )

    shoulder_delta = quat_from_two_vectors(
        positions[elbow] - positions[shoulder], elbow_target - shoulder_position
    )
    elbow_delta = quat_from_two_vectors(
        positions[wrist] - positions[elbow], wrist_target - elbow_target
    )
    output_positions[elbow] = elbow_target
    output_positions[wrist] = wrist_target
    if wrist_to_palm_distance is None:
        reconstructed_palm = (
            wrist_target - palm_rotation @ calibration.palm_to_wrist_offset_m
        )
    else:
        reconstructed_palm = wrist_target + palm_rotation @ wrist_to_palm_local
    output_positions[hand] = reconstructed_palm
    output_orientations[shoulder] = _same_hemisphere(
        quat_multiply(shoulder_delta, output_orientations[shoulder]),
        output_orientations[shoulder],
    )
    output_orientations[elbow] = _same_hemisphere(
        quat_multiply(elbow_delta, output_orientations[elbow]),
        output_orientations[elbow],
    )
    output_orientations[wrist] = wrist_rotation
    output_orientations[hand] = palm_quat
    return SideCorrection(
        positions=output_positions,
        orientations_xyzw=output_orientations,
        reach_clamped=reach_clamped,
        wrist_palm_position_residual_m=float(np.linalg.norm(reconstructed_palm - palm)),
    )


def solve_triangle(
    shoulder_position: np.ndarray,
    wrist_position: np.ndarray,
    upper_length_m: float,
    forearm_length_m: float,
    elbow_reference: np.ndarray,
) -> np.ndarray:
    """Solve the elbow on the two-sphere intersection circle."""

    p0 = _finite_vector(shoulder_position, "shoulder_position", 3)
    p2 = _finite_vector(wrist_position, "wrist_position", 3)
    reference = _finite_vector(elbow_reference, "elbow_reference", 3)
    lu = float(upper_length_m)
    lf = float(forearm_length_m)
    if lu <= EPS or lf <= EPS:
        raise ValueError("triangle edge lengths must be positive")
    edge = p2 - p0
    distance = float(np.linalg.norm(edge))
    if distance <= EPS:
        raise ValueError("shoulder and wrist are coincident")
    if distance < abs(lu - lf) - 1.0e-7 or distance > lu + lf + 1.0e-7:
        raise ValueError("triangle edge lengths are incompatible")
    axis = edge / distance
    along = (lu * lu - lf * lf + distance * distance) / (2.0 * distance)
    radius_sq = max(lu * lu - along * along, 0.0)
    center = p0 + axis * along
    reference_offset = reference - center
    tangent = reference_offset - axis * float(np.dot(reference_offset, axis))
    tangent_norm = float(np.linalg.norm(tangent))
    if tangent_norm <= EPS:
        tangent = _orthogonal_unit(axis)
    else:
        tangent /= tangent_norm
    return center + tangent * np.sqrt(radius_sq)


def quat_identity() -> np.ndarray:
    return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)


def quat_from_axis_angle(axis: np.ndarray | list[float], angle_rad: float) -> np.ndarray:
    unit = _normalize_vector(axis, "axis")
    half = float(angle_rad) * 0.5
    return np.array(
        [unit[0] * np.sin(half), unit[1] * np.sin(half), unit[2] * np.sin(half), np.cos(half)],
        dtype=float,
    )


def quat_to_matrix(quat_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = _normalize_quat(quat_xyzw, "quaternion")
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rotation_to_quat(rotation: np.ndarray) -> np.ndarray:
    matrix = _finite_array(rotation, "rotation")
    if matrix.shape != (3, 3):
        raise ValueError(f"rotation must have shape (3, 3), got {matrix.shape}")
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        result = np.array(
            [
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            ],
            dtype=float,
        )
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        next_index = (index + 1) % 3
        last_index = (index + 2) % 3
        scale = 2.0 * np.sqrt(
            max(1.0 + matrix[index, index] - diagonal[next_index] - diagonal[last_index], EPS)
        )
        result = np.zeros(4, dtype=float)
        result[index] = 0.25 * scale
        result[3] = (matrix[last_index, next_index] - matrix[next_index, last_index]) / scale
        result[next_index] = (matrix[next_index, index] + matrix[index, next_index]) / scale
        result[last_index] = (matrix[last_index, index] + matrix[index, last_index]) / scale
    return _normalize_quat(result, "rotation quaternion")


def quat_multiply(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = _normalize_quat(first, "first quaternion")
    x2, y2, z2, w2 = _normalize_quat(second, "second quaternion")
    return _normalize_quat(
        np.array(
            [
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            ],
            dtype=float,
        ),
        "product quaternion",
    )


def quat_inverse(quat_xyzw: np.ndarray) -> np.ndarray:
    quat = _normalize_quat(quat_xyzw, "quaternion")
    return np.array([-quat[0], -quat[1], -quat[2], quat[3]], dtype=float)


def quat_apply(quat_xyzw: np.ndarray, vector_xyz: np.ndarray) -> np.ndarray:
    vector = _finite_vector(vector_xyz, "vector", 3)
    return quat_to_matrix(quat_xyzw) @ vector


def quat_from_two_vectors(vector_from: np.ndarray, vector_to: np.ndarray) -> np.ndarray:
    first = _normalize_vector(vector_from, "vector_from")
    second = _normalize_vector(vector_to, "vector_to")
    dot = float(np.clip(np.dot(first, second), -1.0, 1.0))
    if dot >= 1.0 - 1.0e-8:
        return quat_identity()
    if dot <= -1.0 + 1.0e-8:
        return quat_from_axis_angle(_orthogonal_unit(first), np.pi)
    return _normalize_quat(
        np.array([*(np.cross(first, second)), 1.0 + dot], dtype=float),
        "two-vector quaternion",
    )


def quat_apply_many(quat_xyzw: np.ndarray, vectors_xyz: np.ndarray) -> np.ndarray:
    return np.asarray([quat_apply(quat_xyzw, vector) for vector in vectors_xyz], dtype=float)


def _mean_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    if not rotations:
        raise ValueError("at least one rotation is required")
    reference = rotation_to_quat(rotations[0])
    quaternions = []
    for rotation in rotations:
        quat = rotation_to_quat(rotation)
        if float(np.dot(quat, reference)) < 0.0:
            quat = -quat
        quaternions.append(quat)
    average = np.mean(np.asarray(quaternions), axis=0)
    return quat_to_matrix(_normalize_quat(average, "mean quaternion"))


def _same_hemisphere(quat: np.ndarray, reference: np.ndarray) -> np.ndarray:
    normalized = _normalize_quat(quat, "quaternion")
    ref = _normalize_quat(reference, "reference quaternion")
    return -normalized if float(np.dot(normalized, ref)) < 0.0 else normalized


def _finite_array(value: np.ndarray, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} contains non-finite values")
    return array


def _finite_vector(value: np.ndarray, name: str, size: int) -> np.ndarray:
    array = _finite_array(value, name)
    if array.shape != (size,):
        raise ValueError(f"{name} must have shape ({size},), got {array.shape}")
    return array


def _normalize_quat(value: np.ndarray, name: str) -> np.ndarray:
    quat = _finite_vector(value, name, 4)
    length = float(np.linalg.norm(quat))
    if length <= EPS:
        raise ValueError(f"{name} must be non-zero")
    return quat / length


def _normalize_vector(value: np.ndarray | list[float], name: str) -> np.ndarray:
    vector = _finite_vector(np.asarray(value, dtype=float), name, 3)
    length = float(np.linalg.norm(vector))
    if length <= EPS:
        raise ValueError(f"{name} must be non-zero")
    return vector / length


def _orthogonal_unit(vector: np.ndarray) -> np.ndarray:
    unit = _normalize_vector(vector, "vector")
    helper = np.array([1.0, 0.0, 0.0], dtype=float)
    if abs(float(np.dot(unit, helper))) > 0.9:
        helper = np.array([0.0, 1.0, 0.0], dtype=float)
    return _normalize_vector(np.cross(unit, helper), "orthogonal vector")
