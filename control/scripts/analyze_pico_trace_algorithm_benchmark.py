#!/usr/bin/env python3
"""Analyze a reproducible PICO trace algorithm-matrix benchmark."""

from __future__ import annotations

import csv
import math
from pathlib import Path
import argparse
import gc
import json
from typing import Any, Mapping, Sequence

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import welch


PRIMARY_ALGORITHMS = (
    "spark_upper_qpoases_direct",
    "spark_upper_qpoases_velocity_qp",
    "spark_upper_qpoases_cartesian_otg_velocity_qp",
    "spark_upper_qpoases_feedforward_velocity_qp",
    "spark_upper_qpoases_headroom_feedforward_velocity_qp",
)
DISPLAY_NAMES = {
    "spark_upper_qpoases_direct": "SPARK two-stage IK direct",
    "spark_upper_qpoases_velocity_qp": "SPARK + Velocity QP",
    "spark_upper_qpoases_cartesian_otg_velocity_qp":
        "SPARK + Cartesian OTG + Velocity QP",
    "spark_upper_qpoases_feedforward_velocity_qp":
        "SPARK + Feedforward Velocity QP",
    "spark_upper_qpoases_headroom_feedforward_velocity_qp":
        "SPARK + Headroom Feedforward Velocity QP",
    "hierarchical_qp": "Hierarchical QP",
    "nullspace_dls": "Nullspace DLS",
    "spark_guided_velocity_qp": "SPARK-guided Velocity QP",
    "spark_direct_velocity_qp": "SPARK-direct Velocity QP",
    "spark_pose_velocity_qp": "SPARK-pose Velocity QP",
}
COLORS = (
    "#1769aa", "#d97706", "#2f855a", "#7c3aed", "#c026d3",
    "#0891b2", "#dc2626", "#65a30d", "#9333ea", "#475569",
)


def _position(row: Mapping[str, str], side: str, prefix: str) -> np.ndarray:
    return np.asarray(
        [float(row[f"{side}_{prefix}_p{axis}"]) for axis in "xyz"], dtype=float
    )


def _quaternion(row: Mapping[str, str], side: str, prefix: str) -> np.ndarray:
    value = np.asarray(
        [float(row[f"{side}_{prefix}_q{axis}"]) for axis in "xyzw"], dtype=float
    )
    norm = float(np.linalg.norm(value))
    if norm <= 0.0:
        raise ValueError("zero-norm quaternion in telemetry")
    return value / norm


def quaternion_distance(left: np.ndarray, right: np.ndarray) -> float:
    dot = abs(float(np.dot(left, right)))
    return 2.0 * math.acos(float(np.clip(dot, -1.0, 1.0)))


def _timestamp_ns(row: Mapping[str, str]) -> int:
    return int(float(row["pico_left_source_timestamp_ns"]))


def _target_agrees(reference: Mapping[str, str], candidate: Mapping[str, str],
                   tolerance_m: float, tolerance_rad: float = 1.0e-4) -> bool:
    for side in ("left", "right"):
        if np.linalg.norm(
            _position(reference, side, "target")
            - _position(candidate, side, "target")
        ) > tolerance_m:
            return False
        if quaternion_distance(
            _quaternion(reference, side, "target"),
            _quaternion(candidate, side, "target"),
        ) > tolerance_rad:
            return False
    return True


def select_common_sequences(
    inputs: Mapping[str, Mapping[int, Mapping[str, str]]], *, canonical: str,
    recovery_ms: float = 300.0, target_agreement_tolerance_m: float = 1.0e-5,
    require_target_agreement: bool = True,
) -> tuple[list[int], list[int]]:
    if canonical not in inputs or not inputs:
        raise ValueError("canonical algorithm is absent")
    all_valid = sorted(set.intersection(*(set(rows) for rows in inputs.values())))
    if not all_valid:
        raise ValueError("algorithms have no common PICO source frames")
    canonical_rows = inputs[canonical]
    first_sequence = all_valid[0]
    first_timestamps = {
        name: _timestamp_ns(rows[first_sequence]) for name, rows in inputs.items()
    }
    canonical_first_timestamp = first_timestamps[canonical]
    recovery_ns = round(recovery_ms * 1.0e6)
    recovery_starts = [_timestamp_ns(canonical_rows[all_valid[0]])]
    previous = canonical_rows[all_valid[0]]
    counter_names = (
        "pico_resynchronizations", "pico_reset_applies", "pico_jump_rejections",
    )
    for sequence in all_valid[1:]:
        row = canonical_rows[sequence]
        changed = row.get("pico_tracking_epoch") != previous.get("pico_tracking_epoch")
        changed = changed or any(
            int(float(row.get(name, "0"))) > int(float(previous.get(name, "0")))
            for name in counter_names
        )
        if changed:
            recovery_starts.append(_timestamp_ns(row))
        previous = row

    clean: list[int] = []
    for sequence in all_valid:
        row = canonical_rows[sequence]
        timestamp = _timestamp_ns(row)
        if any(start <= timestamp <= start + recovery_ns for start in recovery_starts):
            continue
        canonical_relative = timestamp - canonical_first_timestamp
        if any(abs((_timestamp_ns(candidate[sequence]) - first_timestamps[name])
                   - canonical_relative) > 1_000
               for name, candidate in inputs.items()):
            continue
        if require_target_agreement and not all(
                _target_agrees(row, candidate[sequence],
                               target_agreement_tolerance_m)
                for candidate in inputs.values()):
            continue
        clean.append(sequence)
    if not clean:
        raise ValueError("no clean common target-agreeing source frames")
    return all_valid, clean


def _tracking_errors(
    actual_rows: Mapping[int, Mapping[str, str]],
    target_rows: Mapping[int, Mapping[str, str]],
    sequences: Sequence[int], lag_frames: int,
) -> tuple[np.ndarray, np.ndarray]:
    valid = set(sequences)
    position: list[float] = []
    orientation: list[float] = []
    for sequence in sequences:
        target_sequence = sequence - lag_frames
        if target_sequence not in valid:
            continue
        for side in ("left", "right"):
            position.append(float(np.linalg.norm(
                _position(actual_rows[sequence], side, "actual")
                - _position(target_rows[target_sequence], side, "target")
            )))
            orientation.append(quaternion_distance(
                _quaternion(actual_rows[sequence], side, "actual"),
                _quaternion(target_rows[target_sequence], side, "target"),
            ))
    if not position:
        raise ValueError("no samples remain at requested lag")
    return np.asarray(position), np.asarray(orientation)


def _source_period_ms(rows: Mapping[int, Mapping[str, str]],
                      sequences: Sequence[int]) -> float:
    deltas = []
    for previous, current in zip(sequences, sequences[1:]):
        if current != previous + 1:
            continue
        delta = _timestamp_ns(rows[current]) - _timestamp_ns(rows[previous])
        if delta > 0:
            deltas.append(delta * 1.0e-6)
    return float(np.median(deltas)) if deltas else 10.0


def _amplitude_ratio(actual_rows: Mapping[int, Mapping[str, str]],
                     target_rows: Mapping[int, Mapping[str, str]],
                     sequences: Sequence[int]) -> float:
    actual_values = []
    target_values = []
    for side in ("left", "right"):
        actual_values.append(np.stack([
            _position(actual_rows[s], side, "actual") for s in sequences
        ]))
        target_values.append(np.stack([
            _position(target_rows[s], side, "target") for s in sequences
        ]))
    actual = np.concatenate(actual_values)
    target = np.concatenate(target_values)
    actual -= np.mean(actual, axis=0)
    target -= np.mean(target, axis=0)
    denominator = float(np.sum(np.square(target)))
    return math.sqrt(float(np.sum(np.square(actual))) / denominator) \
        if denominator > 0.0 else math.nan


def compute_tracking_summary(
    actual_rows: Mapping[int, Mapping[str, str]],
    target_rows: Mapping[int, Mapping[str, str]], sequences: Sequence[int],
    max_lag_ms: float = 300.0,
) -> dict[str, float]:
    period_ms = _source_period_ms(target_rows, sequences)
    max_frames = max(0, int(math.floor(max_lag_ms / period_ms)))
    zero_position, zero_orientation = _tracking_errors(
        actual_rows, target_rows, sequences, 0
    )
    scans = []
    for lag in range(max_frames + 1):
        position, orientation = _tracking_errors(
            actual_rows, target_rows, sequences, lag
        )
        scans.append((
            lag, float(np.sqrt(np.mean(np.square(position)))),
            float(np.sqrt(np.mean(np.square(orientation)))),
        ))
    position_best = min(scans, key=lambda item: item[1])
    orientation_best = min(scans, key=lambda item: item[2])
    return {
        "sample_count": float(len(zero_position)),
        "source_period_ms": period_ms,
        "position_mean_m": float(np.mean(zero_position)),
        "position_p50_m": float(np.percentile(zero_position, 50)),
        "position_p95_m": float(np.percentile(zero_position, 95)),
        "position_p99_m": float(np.percentile(zero_position, 99)),
        "position_rmse_m": float(np.sqrt(np.mean(np.square(zero_position)))),
        "position_max_m": float(np.max(zero_position)),
        "orientation_mean_rad": float(np.mean(zero_orientation)),
        "orientation_p50_rad": float(np.percentile(zero_orientation, 50)),
        "orientation_p95_rad": float(np.percentile(zero_orientation, 95)),
        "orientation_p99_rad": float(np.percentile(zero_orientation, 99)),
        "orientation_rmse_rad": float(np.sqrt(np.mean(np.square(zero_orientation)))),
        "orientation_max_rad": float(np.max(zero_orientation)),
        "position_lag_ms": position_best[0] * period_ms,
        "position_compensated_rmse_m": position_best[1],
        "orientation_lag_ms": orientation_best[0] * period_ms,
        "orientation_compensated_rmse_rad": orientation_best[2],
        "position_amplitude_ratio": _amplitude_ratio(
            actual_rows, target_rows, sequences
        ),
    }


def band_energy_ratio(values: np.ndarray, time_seconds: np.ndarray,
                      low_hz: float, high_hz: float) -> float:
    values = np.asarray(values, dtype=float)
    time_seconds = np.asarray(time_seconds, dtype=float)
    if len(values) < 8 or len(values) != len(time_seconds):
        raise ValueError("band-energy input is too short or mismatched")
    dt = float(np.median(np.diff(time_seconds)))
    if dt <= 0.0:
        raise ValueError("timestamps must be increasing")
    frequencies, power = welch(
        values - np.mean(values), fs=1.0 / dt,
        nperseg=min(2048, len(values)), detrend="linear",
    )
    total_mask = frequencies > 0.0
    band_mask = (frequencies >= low_hz) & (frequencies <= high_hz)
    total = float(np.trapezoid(power[total_mask], frequencies[total_mask]))
    band = float(np.trapezoid(power[band_mask], frequencies[band_mask]))
    return band / total if total > 0.0 else 0.0


def audit_joint_rows(rows: Sequence[Mapping[str, str]],
                     tolerance: float = 1.0e-7) -> dict[tuple[str, int], dict[str, float]]:
    if not rows:
        raise ValueError("joint telemetry contains no rows")
    result: dict[tuple[str, int], dict[str, float]] = {}
    metrics = (
        ("position", "q", "rad"),
        ("velocity", "qdot", "rad_s"),
        ("acceleration", "qddot", "rad_s2"),
        ("jerk", "jerk", "rad_s3"),
    )
    for side in ("left", "right"):
        for joint in range(1, 8):
            prefix = f"{side}_j{joint}_"
            output: dict[str, float] = {
                "sample_count": float(len(rows)),
                "reset_count": float(sum(int(float(row.get("reset", "0"))) for row in rows)),
            }
            for name, value_suffix, unit in metrics:
                values = np.asarray([
                    float(row[prefix + "reference_" + value_suffix]) for row in rows
                ])
                lower = np.asarray([float(row[prefix + name + "_lower"]) for row in rows])
                upper = np.asarray([float(row[prefix + name + "_upper"]) for row in rows])
                active = (values <= lower + tolerance) | (values >= upper - tolerance)
                violation = (values < lower - tolerance) | (values > upper + tolerance)
                absolute = np.abs(values)
                output[f"{name}_active_count"] = float(np.count_nonzero(active))
                output[f"{name}_violation_count"] = float(np.count_nonzero(violation))
                output[f"{value_suffix}_p50_{unit}"] = float(np.percentile(absolute, 50))
                output[f"{value_suffix}_p95_{unit}"] = float(np.percentile(absolute, 95))
                output[f"{value_suffix}_p99_{unit}"] = float(np.percentile(absolute, 99))
                output[f"{value_suffix}_max_{unit}"] = float(np.max(absolute))
            result[(side, joint)] = output
    return result


def load_live_rows(path: Path) -> tuple[dict[int, dict[str, str]], list[dict[str, str]]]:
    with Path(path).open(newline="") as stream:
        all_rows = list(csv.DictReader(stream))
    if not all_rows:
        raise ValueError(f"empty telemetry: {path}")
    latest: dict[int, dict[str, str]] = {}
    for row in all_rows:
        if row.get("pico_live") == "1" and int(float(row["pico_sequence"])) > 0:
            latest[int(float(row["pico_sequence"]))] = row
    if not latest:
        raise ValueError(f"no PICO-live rows: {path}")
    return latest, all_rows


def load_csv_rows(path: Path) -> list[dict[str, str]]:
    with Path(path).open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"empty CSV: {path}")
    return rows


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    if not rows:
        raise ValueError(f"refusing empty report CSV: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _runtime_summary(rows: Sequence[Mapping[str, str]]) -> dict[str, float]:
    live = [row for row in rows if row.get("pico_live") == "1"]
    cycle = np.asarray([float(row.get("cycle_time_us", "nan")) for row in live])
    solve = np.asarray([
        max(float(row.get("left_solve_time_us", "nan")),
            float(row.get("right_solve_time_us", "nan")))
        for row in live
    ])
    return {
        "cycle_mean_us": float(np.nanmean(cycle)),
        "cycle_p99_us": float(np.nanpercentile(cycle, 99)),
        "solve_mean_us": float(np.nanmean(solve)),
        "solve_p99_us": float(np.nanpercentile(solve, 99)),
        "acceptance_rate": float(np.mean([
            int(float(row.get("accepted", "0"))) for row in live
        ])),
        "deadline_misses": float(max(
            int(float(row.get("deadline_misses", "0"))) for row in rows
        )),
        "control_failures": float(max(
            int(float(row.get("control_failures", "0"))) for row in rows
        )),
    }


def _motion_band_ratio(rows: Mapping[int, Mapping[str, str]],
                       sequences: Sequence[int]) -> float:
    time = np.asarray([_timestamp_ns(rows[s]) for s in sequences], dtype=float) * 1e-9
    if len(time) < 8 or np.any(np.diff(time) <= 0.0):
        return math.nan
    ratios = []
    for side in ("left", "right"):
        values = np.stack([_position(rows[s], side, "actual") for s in sequences])
        for axis in range(3):
            ratios.append(band_energy_ratio(values[:, axis], time, 2.5, 5.0))
    return float(np.mean(ratios))


def _tracking_rows(
    names: Sequence[str], inputs: Mapping[str, Mapping[int, Mapping[str, str]]],
    all_rows: Mapping[str, Sequence[Mapping[str, str]]],
    target_rows: Mapping[int, Mapping[str, str]], all_valid: Sequence[int],
    clean: Sequence[int], *, own_target: bool = False,
) -> list[dict[str, Any]]:
    output = []
    for sample_set, sequences in (("all_valid", all_valid), ("clean_common", clean)):
        for name in names:
            target = inputs[name] if own_target else target_rows
            summary = compute_tracking_summary(inputs[name], target, sequences)
            summary.update(_runtime_summary(all_rows[name]))
            summary.update({
                "algorithm": name, "display_name": DISPLAY_NAMES.get(name, name),
                "sample_set": sample_set,
                "motion_2p5_5hz_energy_ratio": _motion_band_ratio(inputs[name], sequences),
            })
            output.append(summary)
    return output


def _highest_motion_window(target: Mapping[int, Mapping[str, str]],
                           sequences: Sequence[int], size: int = 360) -> list[int]:
    if len(sequences) <= size:
        return list(sequences)
    motion = np.zeros(len(sequences))
    for index in range(1, len(sequences)):
        if sequences[index] != sequences[index - 1] + 1:
            continue
        motion[index] = sum(float(np.linalg.norm(
            _position(target[sequences[index]], side, "target")
            - _position(target[sequences[index - 1]], side, "target")
        )) for side in ("left", "right"))
    score = np.convolve(motion, np.ones(size), mode="valid")
    start = int(np.argmax(score))
    return list(sequences[start:start + size])


def _style(axis: Any) -> None:
    axis.grid(True, color="#d7dce2", linewidth=0.7, alpha=0.8)
    if hasattr(axis, "spines"):
        axis.spines[["top", "right"]].set_visible(False)


def _plot_tracking(output: Path, names: Sequence[str],
                   inputs: Mapping[str, Mapping[int, Mapping[str, str]]],
                   target: Mapping[int, Mapping[str, str]],
                   sequences: Sequence[int], *, window: Sequence[int] | None = None,
                   filename: str = "tracking_timeseries.png",
                   title_prefix: str = "Highest-motion common window") -> None:
    window = list(window) if window is not None else _highest_motion_window(target, sequences)
    timestamps = np.asarray([_timestamp_ns(target[s]) for s in window], dtype=float)
    time = (timestamps - timestamps[0]) * 1e-9
    figure, axes = plt.subplots(2, 3, figsize=(16, 7.5), sharex=True,
                                constrained_layout=True)
    for row, side in enumerate(("left", "right")):
        for column, component in enumerate("xyz"):
            axis = axes[row, column]
            axis.plot(time, [_position(target[s], side, "target")[column] for s in window],
                      color="#20242a", linestyle=":", linewidth=2.2,
                      label="Canonical SPARK intent")
            for index, name in enumerate(names):
                axis.plot(time, [_position(inputs[name][s], side, "actual")[column]
                                 for s in window], color=COLORS[index], linewidth=1.35,
                          label=DISPLAY_NAMES.get(name, name))
            axis.set_title(f"{side.title()} TCP {component.upper()}")
            axis.set_ylabel("Position [m]")
            _style(axis)
    axes[-1, 0].set_xlabel("Window time [s]")
    axes[-1, 1].set_xlabel("Window time [s]")
    axes[-1, 2].set_xlabel("Window time [s]")
    axes[0, 0].legend(fontsize=7, loc="best")
    figure.suptitle(f"{title_prefix} (source sequences {window[0]}–{window[-1]})")
    figure.savefig(output / filename, dpi=180)
    plt.close(figure)


def _plot_tracking_segments(output: Path, names: Sequence[str],
                            inputs: Mapping[str, Mapping[int, Mapping[str, str]]],
                            target: Mapping[int, Mapping[str, str]],
                            sequences: Sequence[int], segment_seconds: float) -> None:
    if segment_seconds <= 0.0:
        return
    if not sequences:
        return
    output = output / "tracking_segments"
    output.mkdir(parents=True, exist_ok=True)
    first_timestamp = _timestamp_ns(target[sequences[0]])
    segment_indices: dict[int, list[int]] = {}
    for sequence in sequences:
        segment = int(((_timestamp_ns(target[sequence]) - first_timestamp) * 1.0e-9)
                      // segment_seconds)
        segment_indices.setdefault(segment, []).append(sequence)
    for segment, window in sorted(segment_indices.items()):
        start_seconds = segment * segment_seconds
        end_seconds = start_seconds + segment_seconds
        filename = f"segment_{segment:03d}_{start_seconds:03.0f}-{end_seconds:03.0f}s.png"
        _plot_tracking(
            output, names, inputs, target, window, window=window,
            filename=filename, title_prefix=f"10-second tracking segment {start_seconds:.0f}–{end_seconds:.0f}s",
        )


def _plot_trajectory(output: Path, names: Sequence[str],
                     inputs: Mapping[str, Mapping[int, Mapping[str, str]]],
                     target: Mapping[int, Mapping[str, str]],
                     sequences: Sequence[int]) -> None:
    figure = plt.figure(figsize=(14, 6), constrained_layout=True)
    for subplot, side in enumerate(("left", "right"), 1):
        axis = figure.add_subplot(1, 2, subplot, projection="3d")
        desired = np.stack([_position(target[s], side, "target") for s in sequences])
        axis.plot(*desired.T, color="#20242a", linestyle=":", linewidth=2,
                  label="Canonical SPARK intent")
        for index, name in enumerate(names):
            actual = np.stack([_position(inputs[name][s], side, "actual")
                               for s in sequences])
            axis.plot(*actual.T, color=COLORS[index], linewidth=1.1,
                      label=DISPLAY_NAMES.get(name, name))
        axis.set_title(f"{side.title()} arm TCP")
        axis.set_xlabel("X [m]"); axis.set_ylabel("Y [m]"); axis.set_zlabel("Z [m]")
        axis.legend(fontsize=7)
    figure.suptitle("Canonical PICO trace — common source frames")
    figure.savefig(output / "trajectory_3d.png", dpi=180)
    plt.close(figure)


def _plot_cdf(output: Path, names: Sequence[str],
              inputs: Mapping[str, Mapping[int, Mapping[str, str]]],
              target: Mapping[int, Mapping[str, str]], sequences: Sequence[int]) -> None:
    figure, axes = plt.subplots(1, 2, figsize=(12, 4.8), constrained_layout=True)
    for index, name in enumerate(names):
        position, orientation = _tracking_errors(inputs[name], target, sequences, 0)
        for axis, values, scale in ((axes[0], position, 1000.0),
                                    (axes[1], orientation, 180.0 / math.pi)):
            sorted_values = np.sort(values * scale)
            axis.plot(sorted_values, np.linspace(0, 1, len(values)),
                      color=COLORS[index], label=DISPLAY_NAMES.get(name, name))
    axes[0].set_xlabel("Position error [mm]")
    axes[1].set_xlabel("Orientation error [deg]")
    for axis in axes:
        axis.set_ylabel("Empirical CDF"); _style(axis)
    axes[0].legend(fontsize=7)
    figure.savefig(output / "error_cdf.png", dpi=180)
    plt.close(figure)


def _plot_lag(output: Path, names: Sequence[str],
              inputs: Mapping[str, Mapping[int, Mapping[str, str]]],
              target: Mapping[int, Mapping[str, str]], sequences: Sequence[int]) -> None:
    period = _source_period_ms(target, sequences)
    max_frames = max(1, int(300.0 / period))
    figure, axes = plt.subplots(1, 2, figsize=(12, 4.8), constrained_layout=True)
    for index, name in enumerate(names):
        p_values, r_values = [], []
        for lag in range(max_frames + 1):
            p_error, r_error = _tracking_errors(inputs[name], target, sequences, lag)
            p_values.append(np.sqrt(np.mean(np.square(p_error))) * 1000.0)
            r_values.append(np.sqrt(np.mean(np.square(r_error))) * 180.0 / math.pi)
        lag_ms = np.arange(max_frames + 1) * period
        axes[0].plot(lag_ms, p_values, color=COLORS[index],
                     label=DISPLAY_NAMES.get(name, name))
        axes[1].plot(lag_ms, r_values, color=COLORS[index])
    axes[0].set_ylabel("Position RMSE [mm]")
    axes[1].set_ylabel("Orientation RMSE [deg]")
    for axis in axes:
        axis.set_xlabel("Causal target lag [ms]"); _style(axis)
    axes[0].legend(fontsize=7)
    figure.savefig(output / "lag_scan.png", dpi=180)
    plt.close(figure)


def _plot_summary(output: Path, names: Sequence[str],
                  summaries: Sequence[Mapping[str, Any]]) -> None:
    clean = {row["algorithm"]: row for row in summaries
             if row["sample_set"] == "clean_common"}
    figure, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    labels = [DISPLAY_NAMES.get(name, name).replace("SPARK + ", "") for name in names]
    metrics = (
        ("position_p95_m", 1000.0, "Position P95 [mm]"),
        ("position_lag_ms", 1.0, "Position lag [ms]"),
        ("orientation_p95_rad", 180.0 / math.pi, "Orientation P95 [deg]"),
        ("cycle_p99_us", 1.0, "Control cycle P99 [µs]"),
    )
    for axis, (key, scale, title) in zip(axes.flat, metrics):
        axis.bar(np.arange(len(names)), [clean[name][key] * scale for name in names],
                 color=COLORS[:len(names)])
        axis.set_xticks(np.arange(len(names)), labels, rotation=18, ha="right", fontsize=8)
        axis.set_title(title); _style(axis)
    figure.savefig(output / "summary.png", dpi=180)
    plt.close(figure)


def _plot_joint_grid(output: Path, side: str, names: Sequence[str],
                     joint_rows: Mapping[str, Sequence[Mapping[str, str]]]) -> None:
    figure, axes = plt.subplots(4, 7, figsize=(22, 11), constrained_layout=True)
    definitions = (
        ("q", "position", "q [rad]"), ("qdot", "velocity", "dq [rad/s]"),
        ("qddot", "acceleration", "ddq [rad/s²]"),
        ("jerk", "jerk", "jerk [rad/s³]"),
    )
    for column in range(7):
        joint = column + 1
        for row_index, (value_suffix, bound_prefix, ylabel) in enumerate(definitions):
            axis = axes[row_index, column]
            for index, name in enumerate(names):
                rows = joint_rows[name]
                stride = max(1, len(rows) // 1500)
                selected = rows[::stride]
                time = np.asarray([float(row["control_time_seconds"]) for row in selected])
                prefix = f"{side}_j{joint}_"
                values = [float(row[prefix + "reference_" + value_suffix]) for row in selected]
                axis.plot(time, values, color=COLORS[index], linewidth=0.75,
                          label=DISPLAY_NAMES.get(name, name))
                if index == 0:
                    lower = [float(row[prefix + bound_prefix + "_lower"]) for row in selected]
                    upper = [float(row[prefix + bound_prefix + "_upper"]) for row in selected]
                    axis.plot(time, lower, color="#666", linestyle="--", linewidth=0.55)
                    axis.plot(time, upper, color="#666", linestyle="--", linewidth=0.55)
            if row_index == 0:
                axis.set_title(f"J{joint}")
            if column == 0:
                axis.set_ylabel(ylabel)
            if row_index == 3:
                axis.set_xlabel("Time [s]")
            _style(axis)
    axes[0, 0].legend(fontsize=6)
    figure.suptitle(f"{side.title()} arm planned joint reference and hard bounds")
    figure.savefig(output / f"joints_{side}_4x7.png", dpi=160)
    plt.close(figure)


def _generate_plot_set(output: Path, names: Sequence[str],
                       inputs: Mapping[str, Mapping[int, Mapping[str, str]]],
                       target: Mapping[int, Mapping[str, str]],
                       sequences: Sequence[int],
                       joint_rows: Mapping[str, Sequence[Mapping[str, str]]],
                       summaries: Sequence[Mapping[str, Any]],
                       segment_seconds: float = 0.0) -> None:
    if not names:
        return
    output.mkdir(parents=True, exist_ok=True)
    _plot_tracking(output, names, inputs, target, sequences)
    _plot_tracking_segments(output, names, inputs, target, sequences, segment_seconds)
    _plot_trajectory(output, names, inputs, target, sequences)
    _plot_cdf(output, names, inputs, target, sequences)
    _plot_lag(output, names, inputs, target, sequences)
    _plot_summary(output, names, summaries)
    for side in ("left", "right"):
        _plot_joint_grid(output, side, names, joint_rows)


def analyze_benchmark(manifest_path: Path, report: Path,
                      segment_seconds: float = 0.0) -> dict[str, Any]:
    manifest_path = Path(manifest_path).resolve()
    benchmark_root = manifest_path.parent
    manifest = json.loads(manifest_path.read_text())
    names = [name for name in manifest["algorithms"]
             if manifest.get("runs", {}).get(name, {}).get("status")
             in ("complete", "complete_with_failures")]
    if not names:
        raise ValueError("manifest has no completed algorithms")
    inputs: dict[str, dict[int, dict[str, str]]] = {}
    all_rows: dict[str, list[dict[str, str]]] = {}
    joint_rows: dict[str, list[dict[str, str]]] = {}
    for name in names:
        run = manifest["runs"][name]
        inputs[name], all_rows[name] = load_live_rows(benchmark_root / run["telemetry"])
        joint_rows[name] = load_csv_rows(benchmark_root / run["joint_telemetry"])
    main_names = [name for name in PRIMARY_ALGORITHMS if name in names]
    canonical = "spark_upper_qpoases_velocity_qp"
    if canonical not in main_names:
        canonical = main_names[0] if main_names else names[0]
    all_main, clean_main = select_common_sequences(
        {name: inputs[name] for name in main_names or [canonical]}, canonical=canonical,
        require_target_agreement=False,
    )
    main_summary = _tracking_rows(
        main_names or [canonical], inputs, all_rows, inputs[canonical],
        all_main, clean_main,
    )
    all_common = sorted(set.intersection(*(set(inputs[name]) for name in names)))
    _, clean_base = select_common_sequences(
        {canonical: inputs[canonical]}, canonical=canonical,
    )
    clean_all = [sequence for sequence in clean_base if sequence in set(all_common)]
    all_summary = _tracking_rows(
        names, inputs, all_rows, inputs[canonical], all_common, clean_all,
        own_target=True,
    )
    report = Path(report)
    report.mkdir(parents=True, exist_ok=True)
    joint_summary: list[dict[str, Any]] = []
    constraint_summary: list[dict[str, Any]] = []
    for name in names:
        audit = audit_joint_rows(joint_rows[name])
        totals = {f"{kind}_{state}_count": 0.0
                  for kind in ("position", "velocity", "acceleration", "jerk")
                  for state in ("active", "violation")}
        for (side, joint), values in audit.items():
            joint_summary.append({"algorithm": name, "side": side, "joint": joint, **values})
            for key in totals:
                totals[key] += values[key]
        constraint_summary.append({
            "algorithm": name,
            "hard_bound_violation_count": sum(
                value for key, value in totals.items()
                if key.endswith("violation_count")
            ),
            **totals,
        })
    constraint_by_algorithm = {
        row["algorithm"]: row for row in constraint_summary
    }
    for summary in main_summary + all_summary:
        constraint = constraint_by_algorithm[summary["algorithm"]]
        violations = float(constraint["hard_bound_violation_count"])
        failures = float(summary["control_failures"])
        summary["hard_bound_violation_count"] = violations
        summary["ranking_eligible"] = int(violations == 0.0 and failures == 0.0)
    _write_csv(report / "summary_main.csv", main_summary)
    _write_csv(report / "summary_all.csv", all_summary)
    _write_csv(report / "joint_summary.csv", joint_summary)
    _write_csv(report / "constraint_audit.csv", constraint_summary)

    _generate_plot_set(
        report / "main", main_names or [canonical], inputs, inputs[canonical],
        clean_main, joint_rows, main_summary, segment_seconds,
    )
    gc.collect()
    appendix = [name for name in names if name not in main_names]
    if appendix:
        appendix_sequences = [s for s in clean_all if all(s in inputs[n] for n in appendix)]
        appendix_summary = [row for row in all_summary if row["algorithm"] in appendix]
        _generate_plot_set(
            report / "appendix", appendix, inputs, inputs[canonical],
            appendix_sequences, joint_rows, appendix_summary, segment_seconds,
        )
        gc.collect()

    clean_rows = [row for row in main_summary if row["sample_set"] == "clean_common"]
    ranking = sorted(
        [row for row in clean_rows if row["ranking_eligible"]],
        key=lambda row: row["position_p95_m"],
    )
    excluded = [row for row in clean_rows if not row["ranking_eligible"]]
    trace_hash = manifest.get("provenance", {}).get("source_trace", {}).get("sha256", "unknown")
    lines = [
        "# PICO 固定轨迹算法矩阵基准", "",
        f"- 源 TJVR SHA-256：`{trace_hash}`",
        f"- 主榜共同帧：{len(all_main)}；清洁共同帧：{len(clean_main)}",
        "- 清洁集剔除启动及重同步/跳变恢复后的 300 ms。",
        "- 主榜统一使用 SPARK 两阶段 IK 目标；附录算法目标语义不同，不参与主榜胜负。",
        "- 推荐主榜要求 control_failures=0 且所有关节硬约束违反计数=0。",
        "", "## 推荐主榜（按位置 P95）", "",
        "| 排名 | 算法 | 位置 P95 | 位置 lag | 姿态 P95 | 周期 P99 |", 
        "|---:|---|---:|---:|---:|---:|",
    ]
    for rank, row in enumerate(ranking, 1):
        lines.append(
            f"| {rank} | {row['display_name']} | {row['position_p95_m']*1000:.2f} mm | "
            f"{row['position_lag_ms']:.1f} ms | "
            f"{row['orientation_p95_rad']*180/math.pi:.2f}° | "
            f"{row['cycle_p99_us']:.1f} µs |"
        )
    if excluded:
        lines.extend(["", "## 主榜排除项", "",
                      "| 算法 | control failures | 硬约束违反 | 排除原因 |",
                      "|---|---:|---:|---|"])
        for row in excluded:
            reasons = []
            if float(row["control_failures"]) > 0.0:
                reasons.append("控制失败")
            if float(row["hard_bound_violation_count"]) > 0.0:
                reasons.append("关节硬约束违反")
            lines.append(
                f"| {row['display_name']} | {row['control_failures']:.0f} | "
                f"{row['hard_bound_violation_count']:.0f} | {'、'.join(reasons)} |"
            )
    lines.extend(["", "## 安全说明", "",
                  "硬约束是否违反以 `constraint_audit.csv` 为准；active-bound 不是违反。",
                  "完整数值见 `summary_main.csv`、`summary_all.csv` 与 `joint_summary.csv`。", ""])
    (report / "README.md").write_text("\n".join(lines))
    return {
        "algorithms": names, "main_algorithms": main_names,
        "all_common_count": len(all_main), "clean_common_count": len(clean_main),
        "ranking": [row["algorithm"] for row in ranking],
        "excluded_main": [row["algorithm"] for row in excluded],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--segment-seconds", type=float, default=0.0)
    arguments = parser.parse_args()
    result = analyze_benchmark(
        arguments.manifest, arguments.output,
        segment_seconds=arguments.segment_seconds,
    )
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
