#!/usr/bin/env python3
"""Pure SO(3) and artifact operations for palm orientation recalibration."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Iterable

import numpy as np
import yaml


@dataclass(frozen=True)
class OrientationSample:
    controller_rotation: np.ndarray
    head_rotation: np.ndarray
    stamp_ns: int
    tracking_epoch: int


@dataclass(frozen=True)
class OrientationGates:
    min_samples: int = 120
    max_orientation_rms_rad: float = 0.05236
    max_correction_angle_rad: float = 0.7854
    huber_delta_rad: float = 0.0872665


@dataclass(frozen=True)
class OrientationSolution:
    rotation: np.ndarray
    covariance: np.ndarray
    sample_count: int
    tracking_epoch: int
    orientation_rms_rad: float
    correction_angle_rad: float


class OrientationCaptureBuffer:
    """Pair controller/HMD rotations for one explicit tracking epoch."""

    EXPLICIT_EPOCH_SOURCES = {"tcp_connection", "wire_world_reset"}

    def __init__(self, max_pair_skew_ns: int = 30_000_000) -> None:
        if max_pair_skew_ns <= 0:
            raise ValueError("max_pair_skew_ns must be positive")
        self.max_pair_skew_ns = int(max_pair_skew_ns)
        self._samples: list[OrientationSample] = []
        self._latest_head: tuple[np.ndarray, int] | None = None
        self._epoch = 0
        self._epoch_source = "unknown"
        self._last_stamp_ns = -1
        self.error: str | None = None

    @property
    def samples(self) -> tuple[OrientationSample, ...]:
        return tuple(self._samples)

    def begin(self, tracking_epoch: int, tracking_epoch_source: str) -> None:
        if int(tracking_epoch) <= 0:
            raise ValueError("tracking epoch must be positive")
        if tracking_epoch_source not in self.EXPLICIT_EPOCH_SOURCES:
            raise ValueError("tracking epoch source must be explicit")
        self._samples.clear()
        self._latest_head = None
        self._epoch = int(tracking_epoch)
        self._epoch_source = tracking_epoch_source
        self._last_stamp_ns = -1
        self.error = None

    def update_epoch(self, tracking_epoch: int, tracking_epoch_source: str) -> None:
        if (
            self._epoch > 0
            and (
                int(tracking_epoch) != self._epoch
                or tracking_epoch_source != self._epoch_source
            )
        ):
            self.error = "tracking_epoch_changed"

    def add_head(self, rotation: np.ndarray, stamp_ns: int) -> None:
        if self.error is None:
            self._latest_head = (_proper_rotation(rotation, "head_rotation").copy(), int(stamp_ns))

    def add_controller(
        self, rotation: np.ndarray, stamp_ns: int, tracking_epoch: int
    ) -> bool:
        stamp = int(stamp_ns)
        if self.error is not None or int(tracking_epoch) != self._epoch:
            if int(tracking_epoch) != self._epoch:
                self.error = "tracking_epoch_changed"
            return False
        if self._latest_head is None or stamp <= self._last_stamp_ns:
            return False
        head_rotation, head_stamp = self._latest_head
        if abs(stamp - head_stamp) > self.max_pair_skew_ns:
            return False
        self._samples.append(
            OrientationSample(
                controller_rotation=_proper_rotation(rotation, "controller_rotation").copy(),
                head_rotation=head_rotation.copy(),
                stamp_ns=stamp,
                tracking_epoch=self._epoch,
            )
        )
        self._last_stamp_ns = stamp
        return True


def _proper_rotation(value: np.ndarray, name: str) -> np.ndarray:
    rotation = np.asarray(value, dtype=float)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError(f"{name} must be a finite 3x3 rotation")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-6):
        raise ValueError(f"{name} must be orthonormal")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1.0e-6):
        raise ValueError(f"{name} must be a proper rotation")
    return rotation


def gravity_leveled_heading_rotation(head_rotation: np.ndarray) -> np.ndarray:
    """Keep HMD heading while making its reference frame gravity-level in G."""
    head = _proper_rotation(head_rotation, "head_rotation")
    x_axis = np.array([head[0, 0], head[1, 0], 0.0], dtype=float)
    norm = float(np.linalg.norm(x_axis))
    if norm <= 1.0e-6:
        raise ValueError("head heading is unobservable because its +X axis is vertical")
    x_axis /= norm
    z_axis = np.array([0.0, 0.0, 1.0])
    y_axis = np.cross(z_axis, x_axis)
    return np.column_stack((x_axis, y_axis, z_axis))


def _skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(vector, dtype=float)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def rotation_exp(vector: np.ndarray) -> np.ndarray:
    value = np.asarray(vector, dtype=float)
    if value.shape != (3,) or not np.all(np.isfinite(value)):
        raise ValueError("rotation vector must contain three finite values")
    angle = float(np.linalg.norm(value))
    if angle < 1.0e-10:
        return np.eye(3) + _skew(value)
    axis_skew = _skew(value / angle)
    return np.eye(3) + math.sin(angle) * axis_skew + (1.0 - math.cos(angle)) * (axis_skew @ axis_skew)


def rotation_log(rotation: np.ndarray) -> np.ndarray:
    matrix = _proper_rotation(rotation, "rotation")
    cosine = float(np.clip((np.trace(matrix) - 1.0) * 0.5, -1.0, 1.0))
    angle = math.acos(cosine)
    vector = np.array(
        [matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0], matrix[1, 0] - matrix[0, 1]]
    )
    if angle < 1.0e-8:
        return 0.5 * vector
    sine = math.sin(angle)
    if abs(sine) < 1.0e-8:
        eigenvalues, eigenvectors = np.linalg.eig(matrix)
        axis = np.real(eigenvectors[:, int(np.argmin(np.abs(eigenvalues - 1.0)))])
        axis /= np.linalg.norm(axis)
        return angle * axis
    return angle * vector / (2.0 * sine)


def matrix_to_quaternion_xyzw(rotation: np.ndarray) -> list[float]:
    matrix = _proper_rotation(rotation, "rotation")
    eigenvalues, eigenvectors = np.linalg.eigh(
        np.array([
            [matrix[0, 0] - matrix[1, 1] - matrix[2, 2], matrix[1, 0] + matrix[0, 1], matrix[2, 0] + matrix[0, 2], matrix[2, 1] - matrix[1, 2]],
            [matrix[1, 0] + matrix[0, 1], matrix[1, 1] - matrix[0, 0] - matrix[2, 2], matrix[2, 1] + matrix[1, 2], matrix[0, 2] - matrix[2, 0]],
            [matrix[2, 0] + matrix[0, 2], matrix[2, 1] + matrix[1, 2], matrix[2, 2] - matrix[0, 0] - matrix[1, 1], matrix[1, 0] - matrix[0, 1]],
            [matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0], matrix[1, 0] - matrix[0, 1], np.trace(matrix)],
        ]) / 3.0
    )
    quaternion = eigenvectors[:, int(np.argmax(eigenvalues))]
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[3] < 0.0:
        quaternion *= -1.0
    return [float(value) for value in quaternion]


def _rotation_mean(rotations: list[np.ndarray], huber_delta: float) -> tuple[np.ndarray, np.ndarray]:
    aggregate = np.sum(rotations, axis=0)
    u, _, vt = np.linalg.svd(aggregate)
    mean = u @ np.diag([1.0, 1.0, np.linalg.det(u @ vt)]) @ vt
    for _ in range(30):
        residuals = np.asarray([rotation_log(mean.T @ value) for value in rotations])
        norms = np.linalg.norm(residuals, axis=1)
        weights = np.ones_like(norms)
        mask = norms > huber_delta
        weights[mask] = huber_delta / norms[mask]
        update = np.average(residuals, axis=0, weights=weights)
        mean = mean @ rotation_exp(update)
        if np.linalg.norm(update) < 1.0e-11:
            break
    return mean, np.asarray([rotation_log(mean.T @ value) for value in rotations])


def solve_orientation(
    samples: Iterable[OrientationSample],
    current_rotation: np.ndarray,
    gates: OrientationGates = OrientationGates(),
) -> OrientationSolution:
    values = list(samples)
    if len(values) < gates.min_samples:
        raise ValueError(f"sample_count {len(values)} is below {gates.min_samples}")
    epochs = {int(sample.tracking_epoch) for sample in values}
    if len(epochs) != 1 or next(iter(epochs)) <= 0:
        raise ValueError("tracking_epoch must be one explicit positive value")
    stamps = [int(sample.stamp_ns) for sample in values]
    if any(second <= first for first, second in zip(stamps, stamps[1:])):
        raise ValueError("sample timestamps must be strictly increasing")
    rotations = [
        _proper_rotation(sample.controller_rotation, "controller_rotation").T
        @ gravity_leveled_heading_rotation(sample.head_rotation)
        for sample in values
    ]
    mean, residuals = _rotation_mean(rotations, gates.huber_delta_rad)
    norms = np.linalg.norm(residuals, axis=1)
    inliers = norms <= max(3.0 * gates.huber_delta_rad, gates.max_orientation_rms_rad)
    if int(np.count_nonzero(inliers)) < gates.min_samples:
        raise ValueError("orientation inlier sample_count is below gate")
    inlier_residuals = residuals[inliers]
    rms = float(np.sqrt(np.mean(np.sum(inlier_residuals * inlier_residuals, axis=1))))
    if rms > gates.max_orientation_rms_rad:
        raise ValueError(f"orientation_rms_rad {rms:.6f} exceeds gate")
    current = _proper_rotation(current_rotation, "current_rotation")
    correction = float(np.linalg.norm(rotation_log(current.T @ mean)))
    if correction > gates.max_correction_angle_rad:
        raise ValueError(f"correction_angle_rad {correction:.6f} exceeds gate")
    covariance = (
        np.cov(inlier_residuals, rowvar=False, ddof=1)
        if len(inlier_residuals) > 1
        else np.zeros((3, 3))
    )
    covariance = np.asarray(covariance, dtype=float) + np.eye(3) * 1.0e-9
    return OrientationSolution(
        rotation=mean,
        covariance=covariance,
        sample_count=int(np.count_nonzero(inliers)),
        tracking_epoch=next(iter(epochs)),
        orientation_rms_rad=rms,
        correction_angle_rad=correction,
    )


def _translation_values(document: dict) -> list[float]:
    value = document.get("translation_m")
    if isinstance(value, dict):
        value = [value.get(axis) for axis in "xyz"]
    values = np.asarray(value, dtype=float)
    if values.shape != (3,) or not np.all(np.isfinite(values)):
        raise ValueError("translation_m must contain three finite values")
    return [float(item) for item in values]


def translation_fingerprint(document: dict) -> str:
    payload = {
        "schema": "pico_tcp_translation_v1",
        "side": document.get("side"),
        "transform_convention": document.get("transform_convention"),
        "translation_m": _translation_values(document),
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return hashlib.sha256(encoded).hexdigest()


def _covariance_from_upper(values: object) -> np.ndarray:
    raw = np.asarray(values, dtype=float)
    if raw.shape != (21,) or not np.all(np.isfinite(raw)):
        return np.zeros((6, 6))
    covariance = np.zeros((6, 6))
    cursor = 0
    for row in range(6):
        for column in range(row, 6):
            covariance[row, column] = raw[cursor]
            covariance[column, row] = raw[cursor]
            cursor += 1
    return covariance


def _upper_from_covariance(covariance: np.ndarray) -> list[float]:
    return [float(covariance[row, column]) for row in range(6) for column in range(row, 6)]


def build_orientation_only_update(
    document: dict,
    solution: OrientationSolution,
    old_sha256: str,
) -> dict:
    if not isinstance(old_sha256, str) or len(old_sha256) != 64:
        raise ValueError("old_sha256 must contain 64 hexadecimal characters")
    int(old_sha256, 16)
    updated = deepcopy(document)
    original_translation = deepcopy(document.get("translation_m"))
    fingerprint = translation_fingerprint(document)
    existing_fingerprint = document.get("translation_fingerprint_sha256")
    if existing_fingerprint is not None and existing_fingerprint != fingerprint:
        raise ValueError("TCP translation fingerprint is inconsistent")
    calibration_revision = int(document.get("calibration_revision", 0))
    if calibration_revision <= 0:
        raise ValueError("calibration_revision must be positive")
    translation_revision = int(document.get("translation_revision", calibration_revision))
    orientation_revision = int(document.get("orientation_revision", 0))
    ancestors = list(document.get("orientation_only_ancestor_sha256", []))
    if old_sha256 not in ancestors:
        ancestors.append(old_sha256)
    updated["translation_m"] = original_translation
    updated["quaternion_xyzw"] = matrix_to_quaternion_xyzw(solution.rotation)
    updated["calibration_revision"] = calibration_revision + 1
    updated["translation_revision"] = translation_revision
    updated["orientation_revision"] = orientation_revision + 1
    updated["translation_fingerprint_sha256"] = fingerprint
    updated["orientation_only_ancestor_sha256"] = ancestors
    updated["orientation_calibrated"] = True
    updated["orientation_reference"] = "gravity_leveled_hmd_heading"
    updated["orientation_calibration"] = {
        "method": "gravity_leveled_hmd_heading_bilateral_forward_palms_facing",
        "tracking_epoch": int(solution.tracking_epoch),
        "sample_count": int(solution.sample_count),
        "orientation_rms_rad": float(solution.orientation_rms_rad),
        "correction_angle_rad": float(solution.correction_angle_rad),
        "calibrated_at": datetime.now(timezone.utc).isoformat(),
        "lineage": [updated.get("source_topic"), "/pico/pose/head"],
    }
    covariance = _covariance_from_upper(document.get("covariance_upper_triangle_6x6"))
    covariance[3:6, 3:6] = solution.covariance
    updated["covariance_upper_triangle_6x6"] = _upper_from_covariance(covariance)
    if updated.get("translation_m") != original_translation:
        raise AssertionError("orientation-only update changed translation_m")
    return updated


def file_sha256(path: str | Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def atomic_replace_if_unchanged(path: str | Path, old_sha256: str, document: dict) -> None:
    target = Path(path).expanduser()
    if file_sha256(target) != old_sha256:
        raise ValueError("TCP artifact changed during calibration")
    temporary = target.with_name(f".{target.name}.{os.getpid()}.tmp")
    target.parent.mkdir(parents=True, exist_ok=True)
    try:
        with temporary.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(document, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
        directory_fd = os.open(target.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        temporary.unlink(missing_ok=True)
