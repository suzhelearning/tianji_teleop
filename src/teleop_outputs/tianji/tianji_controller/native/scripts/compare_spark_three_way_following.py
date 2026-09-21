#!/usr/bin/env python3
"""Compare raw-intent following for three to five SPARK pipelines."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Mapping, NamedTuple, Sequence

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


class LagResult(NamedTuple):
    position_frames: int
    position_rmse: float
    orientation_frames: int
    orientation_rmse: float


class TrackingSummary(NamedTuple):
    sample_count: int
    position_mean: float
    position_p50: float
    position_p95: float
    position_rmse: float
    position_max: float
    orientation_mean: float
    orientation_p50: float
    orientation_p95: float
    orientation_rmse: float
    orientation_max: float
    position_lag_frames: int
    position_lag_ms: float
    position_compensated_rmse: float
    orientation_lag_frames: int
    orientation_lag_ms: float
    orientation_compensated_rmse: float
    amplitude_ratio: float
    cycle_mean_us: float
    cycle_p99_us: float
    acceptance_rate: float
    control_failures: int
    deadline_misses: int


def quaternion_distance(left: np.ndarray, right: np.ndarray) -> float:
    left_norm = np.linalg.norm(left)
    right_norm = np.linalg.norm(right)
    if left_norm <= 0.0 or right_norm <= 0.0:
        raise ValueError("quaternion norm must be positive")
    dot = abs(float(np.dot(left / left_norm, right / right_norm)))
    return 2.0 * math.acos(float(np.clip(dot, -1.0, 1.0)))


def _position(row: Mapping[str, str], side: str, prefix: str) -> np.ndarray:
    return np.array(
        [float(row[f"{side}_{prefix}_p{axis}"]) for axis in "xyz"],
        dtype=float,
    )


def _quaternion(row: Mapping[str, str], side: str, prefix: str) -> np.ndarray:
    return np.array(
        [float(row[f"{side}_{prefix}_q{axis}"]) for axis in "xyzw"],
        dtype=float,
    )


def tracking_errors(
    actual_rows: Mapping[int, Mapping[str, str]],
    target_rows: Mapping[int, Mapping[str, str]],
    valid_sequences: Sequence[int],
    lag_frames: int,
) -> tuple[np.ndarray, np.ndarray]:
    valid = set(valid_sequences)
    position_errors: list[float] = []
    orientation_errors: list[float] = []
    for sequence in valid_sequences:
        target_sequence = sequence - lag_frames
        if (
            target_sequence not in valid
            or sequence not in actual_rows
            or target_sequence not in target_rows
        ):
            continue
        for side in ("left", "right"):
            position_errors.append(
                float(
                    np.linalg.norm(
                        _position(actual_rows[sequence], side, "actual")
                        - _position(target_rows[target_sequence], side, "target")
                    )
                )
            )
            orientation_errors.append(
                quaternion_distance(
                    _quaternion(actual_rows[sequence], side, "actual"),
                    _quaternion(target_rows[target_sequence], side, "target"),
                )
            )
    if not position_errors:
        raise ValueError("no aligned samples for requested lag")
    return np.asarray(position_errors), np.asarray(orientation_errors)


def best_lag(
    actual_rows: Mapping[int, Mapping[str, str]],
    target_rows: Mapping[int, Mapping[str, str]],
    valid_sequences: Sequence[int],
    max_lag_frames: int,
) -> LagResult:
    if max_lag_frames < 0:
        raise ValueError("max_lag_frames must be non-negative")
    scans: list[tuple[int, float, float]] = []
    for lag in range(max_lag_frames + 1):
        position, orientation = tracking_errors(
            actual_rows, target_rows, valid_sequences, lag
        )
        scans.append(
            (
                lag,
                float(np.sqrt(np.mean(np.square(position)))),
                float(np.sqrt(np.mean(np.square(orientation)))),
            )
        )
    position_best = min(scans, key=lambda item: item[1])
    orientation_best = min(scans, key=lambda item: item[2])
    return LagResult(
        position_frames=position_best[0],
        position_rmse=position_best[1],
        orientation_frames=orientation_best[0],
        orientation_rmse=orientation_best[2],
    )


def _load_live_rows(path: Path) -> tuple[dict[int, dict[str, str]], list[dict[str, str]]]:
    with path.open(newline="") as source:
        rows = list(csv.DictReader(source))
    if not rows:
        raise ValueError(f"telemetry contains no rows: {path}")
    required = {
        "pico_sequence",
        "pico_live",
        "cycle_time_us",
        "control_failures",
        "deadline_misses",
        "accepted",
    }
    for side in ("left", "right"):
        required.update(
            f"{side}_{prefix}_{component}"
            for prefix in ("target", "actual")
            for component in (
                "px", "py", "pz", "qx", "qy", "qz", "qw"
            )
        )
    missing = required.difference(rows[0])
    if missing:
        raise ValueError(f"{path} is missing columns: {sorted(missing)}")
    latest: dict[int, dict[str, str]] = {}
    for row in rows:
        if row["pico_live"] == "1":
            latest[int(row["pico_sequence"])] = row
    if not latest:
        raise ValueError(f"telemetry contains no PICO-live rows: {path}")
    return latest, rows


def _excluded(sequence: int, ranges: Sequence[tuple[int, int]]) -> bool:
    return any(start <= sequence <= end for start, end in ranges)


def _valid_sequences(
    inputs: Mapping[str, Mapping[int, Mapping[str, str]]],
    excluded_ranges: Sequence[tuple[int, int]],
    target_agreement_tolerance_m: float,
) -> list[int]:
    common = set.intersection(*(set(rows) for rows in inputs.values()))
    target_rows = inputs["velocity_qp"]
    comparison_rows = inputs["joint_ruckig"]
    result: list[int] = []
    for sequence in sorted(common):
        if _excluded(sequence, excluded_ranges):
            continue
        agrees = all(
            np.linalg.norm(
                _position(target_rows[sequence], side, "target")
                - _position(comparison_rows[sequence], side, "target")
            )
            <= target_agreement_tolerance_m
            for side in ("left", "right")
        )
        if agrees:
            result.append(sequence)
    if len(result) < 20:
        raise ValueError("fewer than 20 common target-agreeing source frames")
    return result


def _source_period_ms(
    rows: Mapping[int, Mapping[str, str]], sequences: Sequence[int]
) -> float:
    differences = []
    for previous, current in zip(sequences, sequences[1:]):
        if current != previous + 1:
            continue
        current_row = rows[current]
        previous_row = rows[previous]
        key = "pico_left_source_timestamp_ns"
        if key not in current_row:
            continue
        delta = float(current_row[key]) - float(previous_row[key])
        if delta > 0.0:
            differences.append(delta * 1.0e-6)
    return float(np.median(differences)) if differences else 10.0


def _amplitude_ratio(
    actual_rows: Mapping[int, Mapping[str, str]],
    target_rows: Mapping[int, Mapping[str, str]],
    sequences: Sequence[int],
) -> float:
    squared_actual = 0.0
    squared_target = 0.0
    count = 0
    for side in ("left", "right"):
        actual = np.stack(
            [_position(actual_rows[sequence], side, "actual") for sequence in sequences]
        )
        target = np.stack(
            [_position(target_rows[sequence], side, "target") for sequence in sequences]
        )
        actual -= np.mean(actual, axis=0)
        target -= np.mean(target, axis=0)
        squared_actual += float(np.sum(np.square(actual)))
        squared_target += float(np.sum(np.square(target)))
        count += actual.size
    if count == 0 or squared_target <= 0.0:
        return math.nan
    return math.sqrt(squared_actual / squared_target)


def _summary(
    name: str,
    live_rows: Mapping[int, Mapping[str, str]],
    all_rows: Sequence[Mapping[str, str]],
    target_rows: Mapping[int, Mapping[str, str]],
    sequences: Sequence[int],
    max_lag_frames: int,
    source_period_ms: float,
) -> TrackingSummary:
    del name
    position, orientation = tracking_errors(
        live_rows, target_rows, sequences, lag_frames=0
    )
    lag = best_lag(live_rows, target_rows, sequences, max_lag_frames)
    live_all = [row for row in all_rows if row["pico_live"] == "1"]
    cycle = np.asarray([float(row["cycle_time_us"]) for row in live_all])
    return TrackingSummary(
        sample_count=len(position),
        position_mean=float(np.mean(position)),
        position_p50=float(np.percentile(position, 50)),
        position_p95=float(np.percentile(position, 95)),
        position_rmse=float(np.sqrt(np.mean(np.square(position)))),
        position_max=float(np.max(position)),
        orientation_mean=float(np.mean(orientation)),
        orientation_p50=float(np.percentile(orientation, 50)),
        orientation_p95=float(np.percentile(orientation, 95)),
        orientation_rmse=float(np.sqrt(np.mean(np.square(orientation)))),
        orientation_max=float(np.max(orientation)),
        position_lag_frames=lag.position_frames,
        position_lag_ms=lag.position_frames * source_period_ms,
        position_compensated_rmse=lag.position_rmse,
        orientation_lag_frames=lag.orientation_frames,
        orientation_lag_ms=lag.orientation_frames * source_period_ms,
        orientation_compensated_rmse=lag.orientation_rmse,
        amplitude_ratio=_amplitude_ratio(live_rows, target_rows, sequences),
        cycle_mean_us=float(np.mean(cycle)),
        cycle_p99_us=float(np.percentile(cycle, 99)),
        acceptance_rate=float(np.mean([int(row["accepted"]) for row in live_all])),
        control_failures=max(int(row["control_failures"]) for row in all_rows),
        deadline_misses=max(int(row["deadline_misses"]) for row in all_rows),
    )


DISPLAY_NAMES = {
    "velocity_qp": "SPARK + Velocity QP",
    "cartesian_otg": "SPARK + Cartesian OTG + Velocity QP",
    "joint_ruckig": "SPARK + Joint Ruckig",
    "feedforward_velocity_qp": "SPARK + Feedforward Velocity QP",
    "headroom_feedforward_velocity_qp":
        "SPARK + Headroom Feedforward Velocity QP",
}
COLORS = {
    "velocity_qp": "#1769aa",
    "cartesian_otg": "#d97706",
    "joint_ruckig": "#2f855a",
    "feedforward_velocity_qp": "#7c3aed",
    "headroom_feedforward_velocity_qp": "#c026d3",
}
LINESTYLES = {
    "velocity_qp": "-",
    "cartesian_otg": "--",
    "joint_ruckig": "-.",
    "feedforward_velocity_qp": (0, (1, 1)),
    "headroom_feedforward_velocity_qp": (0, (3, 1, 1, 1)),
}


def _style_axis(axis) -> None:
    axis.grid(True, color="#d7dce2", linewidth=0.7, alpha=0.8)
    axis.spines[["top", "right"]].set_visible(False)


def _plot_trajectories(
    output: Path,
    inputs: Mapping[str, Mapping[int, Mapping[str, str]]],
    target_rows: Mapping[int, Mapping[str, str]],
    sequences: Sequence[int],
) -> None:
    figure = plt.figure(figsize=(13, 5.8), constrained_layout=True)
    for index, side in enumerate(("left", "right"), start=1):
        axis = figure.add_subplot(1, 2, index, projection="3d")
        target = np.stack([_position(target_rows[s], side, "target") for s in sequences])
        axis.plot(*target.T, color="#20242a", linewidth=2.0, linestyle=":", label="Raw SPARK palm")
        for name, rows in inputs.items():
            actual = np.stack([_position(rows[s], side, "actual") for s in sequences])
            axis.plot(*actual.T, color=COLORS[name], linestyle=LINESTYLES[name], linewidth=1.3, label=DISPLAY_NAMES[name])
        axis.set_title(f"{side.title()} arm TCP trajectory")
        axis.set_xlabel("X [m]")
        axis.set_ylabel("Y [m]")
        axis.set_zlabel("Z [m]")
        axis.legend(fontsize=8, loc="best")
    figure.suptitle("Raw PICO/SPARK intent versus model TCP — common source frames")
    figure.savefig(output / "trajectory_3d.png", dpi=180)
    plt.close(figure)


def _dynamic_window(
    target_rows: Mapping[int, Mapping[str, str]], sequences: Sequence[int], size: int
) -> list[int]:
    if len(sequences) <= size:
        return list(sequences)
    motion = np.zeros(len(sequences))
    for index in range(1, len(sequences)):
        if sequences[index] != sequences[index - 1] + 1:
            motion[index] = -1.0e6
            continue
        for side in ("left", "right"):
            motion[index] += np.linalg.norm(
                _position(target_rows[sequences[index]], side, "target")
                - _position(target_rows[sequences[index - 1]], side, "target")
            )
    score = np.convolve(motion, np.ones(size), mode="valid")
    start = int(np.argmax(score))
    return list(sequences[start : start + size])


def _plot_timeseries(
    output: Path,
    inputs: Mapping[str, Mapping[int, Mapping[str, str]]],
    target_rows: Mapping[int, Mapping[str, str]],
    sequences: Sequence[int],
    source_period_ms: float,
) -> None:
    window = _dynamic_window(target_rows, sequences, min(360, len(sequences)))
    time = np.arange(len(window)) * source_period_ms * 1.0e-3
    figure, axes = plt.subplots(2, 3, figsize=(15, 7), sharex=True, constrained_layout=True)
    for row_index, side in enumerate(("left", "right")):
        for axis_index, component in enumerate("xyz"):
            axis = axes[row_index, axis_index]
            target = [_position(target_rows[s], side, "target")[axis_index] for s in window]
            axis.plot(time, target, color="#20242a", linewidth=2.0, linestyle=":", label="Raw intent")
            for name, rows in inputs.items():
                actual = [_position(rows[s], side, "actual")[axis_index] for s in window]
                axis.plot(time, actual, color=COLORS[name], linestyle=LINESTYLES[name], linewidth=1.2, label=DISPLAY_NAMES[name])
            axis.set_title(f"{side.title()} TCP {component.upper()}")
            axis.set_ylabel("Position [m]")
            axis.set_xlabel("Window time [s]")
            _style_axis(axis)
    axes[0, 0].legend(fontsize=8, loc="best")
    figure.suptitle(f"Highest-motion window (source sequences {window[0]}–{window[-1]})")
    figure.savefig(output / "tracking_timeseries.png", dpi=180)
    plt.close(figure)


def _plot_cdf(
    output: Path,
    inputs: Mapping[str, Mapping[int, Mapping[str, str]]],
    target_rows: Mapping[int, Mapping[str, str]],
    sequences: Sequence[int],
) -> None:
    figure, axes = plt.subplots(1, 2, figsize=(12, 4.8), constrained_layout=True)
    for name, rows in inputs.items():
        position, orientation = tracking_errors(rows, target_rows, sequences, 0)
        for axis, values, scale in ((axes[0], position, 1000.0), (axes[1], orientation, 1.0)):
            ordered = np.sort(values * scale)
            probability = np.arange(1, len(ordered) + 1) / len(ordered)
            axis.plot(ordered, probability, color=COLORS[name], linestyle=LINESTYLES[name], linewidth=1.7, label=DISPLAY_NAMES[name])
    axes[0].set_xlabel("Raw-intent position error [mm]")
    axes[1].set_xlabel("Raw-intent orientation error [rad]")
    for axis in axes:
        axis.set_ylabel("Empirical CDF")
        axis.set_ylim(0.0, 1.0)
        _style_axis(axis)
    axes[0].legend(fontsize=8, loc="lower right")
    figure.suptitle("Zero-shift end-to-end tracking-error distributions")
    figure.savefig(output / "tracking_error_cdf.png", dpi=180)
    plt.close(figure)


def _lag_scans(
    inputs: Mapping[str, Mapping[int, Mapping[str, str]]],
    target_rows: Mapping[int, Mapping[str, str]],
    sequences: Sequence[int],
    max_lag_frames: int,
) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    result = {}
    for name, rows in inputs.items():
        position = []
        orientation = []
        for lag in range(max_lag_frames + 1):
            p_error, r_error = tracking_errors(rows, target_rows, sequences, lag)
            position.append(math.sqrt(float(np.mean(np.square(p_error)))))
            orientation.append(math.sqrt(float(np.mean(np.square(r_error)))))
        result[name] = (np.asarray(position), np.asarray(orientation))
    return result


def _plot_lag_scan(
    output: Path,
    scans: Mapping[str, tuple[np.ndarray, np.ndarray]],
    source_period_ms: float,
) -> None:
    figure, axes = plt.subplots(1, 2, figsize=(12, 4.8), constrained_layout=True)
    for name, (position, orientation) in scans.items():
        lag_ms = np.arange(len(position)) * source_period_ms
        axes[0].plot(lag_ms, 1000.0 * position, color=COLORS[name], linestyle=LINESTYLES[name], linewidth=1.7, label=DISPLAY_NAMES[name])
        axes[1].plot(lag_ms, orientation, color=COLORS[name], linestyle=LINESTYLES[name], linewidth=1.7, label=DISPLAY_NAMES[name])
        p_index = int(np.argmin(position)); r_index = int(np.argmin(orientation))
        axes[0].scatter(lag_ms[p_index], 1000.0 * position[p_index], color=COLORS[name], s=28)
        axes[1].scatter(lag_ms[r_index], orientation[r_index], color=COLORS[name], s=28)
    axes[0].set_ylabel("Position RMSE [mm]")
    axes[1].set_ylabel("Orientation RMSE [rad]")
    for axis in axes:
        axis.set_xlabel("Applied target delay [ms]")
        _style_axis(axis)
    axes[0].legend(fontsize=8, loc="best")
    figure.suptitle("Delay scan — minima estimate motion-following phase lag")
    figure.savefig(output / "lag_scan.png", dpi=180)
    plt.close(figure)


def _plot_summary(output: Path, summaries: Mapping[str, TrackingSummary]) -> None:
    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    names = list(summaries)
    labels = [DISPLAY_NAMES[name] for name in names]
    colors = [COLORS[name] for name in names]
    metrics = (
        ("Raw-intent position mean [mm]", [1000.0 * summaries[n].position_mean for n in names]),
        ("Raw-intent orientation mean [rad]", [summaries[n].orientation_mean for n in names]),
        ("Best position lag [ms]", [summaries[n].position_lag_ms for n in names]),
        ("TCP motion amplitude / intent", [summaries[n].amplitude_ratio for n in names]),
    )
    for axis, (title, values) in zip(axes.flat, metrics):
        bars = axis.bar(range(len(names)), values, color=colors, edgecolor="#20242a", linewidth=0.8)
        axis.set_title(title)
        axis.set_xticks(range(len(names)), labels, rotation=12, ha="right")
        axis.bar_label(bars, fmt="%.3g", padding=3, fontsize=9)
        axis.set_ylim(0.0, max(values) * 1.20 if max(values) > 0.0 else 1.0)
        _style_axis(axis)
    figure.suptitle(
        f"{len(summaries)}-way raw-PICO-intent following comparison"
    )
    figure.savefig(output / "performance_summary.png", dpi=180)
    plt.close(figure)


def _write_summary_csv(output: Path, summaries: Mapping[str, TrackingSummary]) -> None:
    fieldnames = ["algorithm", *TrackingSummary._fields]
    with (output / "summary.csv").open("w", newline="") as destination:
        writer = csv.DictWriter(destination, fieldnames=fieldnames)
        writer.writeheader()
        for name, summary in summaries.items():
            writer.writerow({"algorithm": name, **summary._asdict()})


def _write_report(
    output: Path,
    summaries: Mapping[str, TrackingSummary],
    sequences: Sequence[int],
    source_period_ms: float,
    excluded_ranges: Sequence[tuple[int, int]],
) -> None:
    fastest = min(summaries, key=lambda name: summaries[name].position_lag_ms)
    lowest_position = min(summaries, key=lambda name: summaries[name].position_mean)
    lines = [
        f"# SPARK {len(summaries)}方案原始意图跟手性对比",
        "",
        f"有效共同源帧：{len(sequences)}；PICO 源周期中位数：{source_period_ms:.3f} ms。",
        f"排除窗口：{', '.join(f'{start}:{end}' for start, end in excluded_ranges) or '无'}。",
        "",
        "| 算法 | 位置均值 | 位置P95 | 姿态均值 | 位置延迟 | 延迟补偿位置RMSE | 幅值保持率 | 周期P99 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for name, summary in summaries.items():
        lines.append(
            f"| {DISPLAY_NAMES[name]} | {1000*summary.position_mean:.2f} mm | "
            f"{1000*summary.position_p95:.2f} mm | {summary.orientation_mean:.4f} rad | "
            f"{summary.position_lag_ms:.1f} ms | "
            f"{1000*summary.position_compensated_rmse:.2f} mm | "
            f"{summary.amplitude_ratio:.3f} | {summary.cycle_p99_us:.1f} us |"
        )
    lines += [
        "",
        f"位置相位延迟最低：**{DISPLAY_NAMES[fastest]}**。",
        f"零时移位置误差最低：**{DISPLAY_NAMES[lowest_position]}**。",
        "",
        "延迟通过 0–max-lag 的离散时移扫描、以位置/姿态 RMSE 最小点估计；它是离线轨迹相位延迟，不包含真机通信与伺服延迟。",
        "内部参考模式的 `target` 不是原始手掌意图，因此本报告统一使用旧 Velocity QP 保存的原始 SPARK palm 作为共同真值。",
        "Joint Ruckig 工程的启动姿态和 J1/J3 模型限位与当前工程不同；启动和重同步 blend 窗口已剔除，但 IK 分支历史仍是剩余限制。",
        "",
        "生成图：`trajectory_3d.png`、`tracking_timeseries.png`、`tracking_error_cdf.png`、`lag_scan.png`、`performance_summary.png`。",
    ]
    (output / "README.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def required_algorithm_names(
    telemetry_paths: Mapping[str, Path],
) -> set[str]:
    base = {"velocity_qp", "cartesian_otg", "joint_ruckig"}
    optional = {
        "feedforward_velocity_qp",
        "headroom_feedforward_velocity_qp",
    }
    supplied = set(telemetry_paths)
    if base.issubset(supplied) and supplied.issubset(base | optional):
        return supplied
    raise ValueError(
        "telemetry names must be the three historical algorithms with an "
        "optional fixed and/or headroom feedforward velocity QP"
    )


def run_analysis(
    telemetry_paths: Mapping[str, Path],
    output_directory: Path,
    excluded_ranges: Sequence[tuple[int, int]],
    max_lag_frames: int,
    target_agreement_tolerance_m: float,
) -> dict[str, TrackingSummary]:
    required_algorithm_names(telemetry_paths)
    inputs: dict[str, dict[int, dict[str, str]]] = {}
    all_rows: dict[str, list[dict[str, str]]] = {}
    for name, path in telemetry_paths.items():
        inputs[name], all_rows[name] = _load_live_rows(Path(path))
    sequences = _valid_sequences(
        inputs, excluded_ranges, target_agreement_tolerance_m
    )
    target_rows = inputs["velocity_qp"]
    source_period_ms = _source_period_ms(target_rows, sequences)
    summaries = {
        name: _summary(
            name,
            rows,
            all_rows[name],
            target_rows,
            sequences,
            max_lag_frames,
            source_period_ms,
        )
        for name, rows in inputs.items()
    }
    output_directory.mkdir(parents=True, exist_ok=True)
    _write_summary_csv(output_directory, summaries)
    _write_report(
        output_directory,
        summaries,
        sequences,
        source_period_ms,
        excluded_ranges,
    )
    _plot_trajectories(output_directory, inputs, target_rows, sequences)
    _plot_timeseries(
        output_directory, inputs, target_rows, sequences, source_period_ms
    )
    _plot_cdf(output_directory, inputs, target_rows, sequences)
    scans = _lag_scans(
        inputs, target_rows, sequences, max_lag_frames
    )
    _plot_lag_scan(output_directory, scans, source_period_ms)
    _plot_summary(output_directory, summaries)
    return summaries


def _parse_ranges(value: str) -> tuple[tuple[int, int], ...]:
    if not value.strip():
        return ()
    ranges = []
    for item in value.split(","):
        start_text, separator, end_text = item.partition(":")
        if not separator:
            raise argparse.ArgumentTypeError("ranges must use start:end")
        start, end = int(start_text), int(end_text)
        if start < 0 or end < start:
            raise argparse.ArgumentTypeError("invalid exclusion range")
        ranges.append((start, end))
    return tuple(ranges)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--velocity-qp", type=Path, required=True)
    parser.add_argument("--cartesian-otg", type=Path, required=True)
    parser.add_argument("--joint-ruckig", type=Path, required=True)
    parser.add_argument("--feedforward-velocity-qp", type=Path)
    parser.add_argument("--headroom-feedforward-velocity-qp", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--exclude", type=_parse_ranges, default=((1, 30), (807, 840)))
    parser.add_argument("--max-lag-frames", type=int, default=80)
    parser.add_argument("--target-agreement-mm", type=float, default=1.0)
    arguments = parser.parse_args()
    telemetry_paths = {
            "velocity_qp": arguments.velocity_qp,
            "cartesian_otg": arguments.cartesian_otg,
            "joint_ruckig": arguments.joint_ruckig,
        }
    if arguments.feedforward_velocity_qp is not None:
        telemetry_paths["feedforward_velocity_qp"] = (
            arguments.feedforward_velocity_qp
        )
    if arguments.headroom_feedforward_velocity_qp is not None:
        telemetry_paths["headroom_feedforward_velocity_qp"] = (
            arguments.headroom_feedforward_velocity_qp
        )
    summaries = run_analysis(
        telemetry_paths,
        arguments.output_dir,
        arguments.exclude,
        arguments.max_lag_frames,
        arguments.target_agreement_mm * 1.0e-3,
    )
    for name, summary in summaries.items():
        print(
            f"algorithm={name} position_mean_mm={1000*summary.position_mean:.3f} "
            f"orientation_mean_rad={summary.orientation_mean:.6f} "
            f"position_lag_ms={summary.position_lag_ms:.3f}"
        )


if __name__ == "__main__":
    main()
