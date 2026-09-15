#!/usr/bin/env python3
"""Analyze PICO hand-tremor recordings.

The input is a directory produced by record_pico_tremor.sh.  The analysis uses
the PICO source timestamps, resamples each stream to its own median-rate grid,
and computes vector PSDs for position and SO(3) orientation residuals.  The
fixed-controller condition is reported as a device-noise reference, not as a
literal subtraction-based human-motion measurement.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import platform
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import detrend, find_peaks, welch
from scipy.spatial.transform import Rotation, Slerp

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


CONDITIONS = ("fixed", "elbow_supported", "arm_unsupported")
CONDITION_LABELS = {
    "fixed": "Fixed controller",
    "elbow_supported": "Elbow supported",
    "arm_unsupported": "Arm unsupported",
}
SIDES = ("left", "right")
HAND_INDEX = {"left": 22, "right": 23}

STREAM_TOPICS = {
    "raw_controller": lambda side: f"/pico/pose/{side}_hand",
    "raw_smpl_hand": "/pico/smpl_raw",
    "corrected_palm": "/pico/smpl_palm_corrected",
    "corrected_ik_palm": "/pico/smpl_palm_corrected_ik",
    "tcp_palm": lambda side: f"/pico/palm_{side}",
}
CORRECTED_STATUS_TOPIC = "/pico/smpl_palm_corrected/status"
STATUS_FILTERED_STREAMS = {"corrected_palm", "corrected_ik_palm"}

SIGNAL_LABELS = {
    "position": "Position residual",
    "orientation": "SO(3) orientation residual",
}

BANDS = (
    ("0.1-0.5 Hz", 0.1, 0.5),
    ("0.5-3 Hz", 0.5, 3.0),
    ("3-6 Hz", 3.0, 6.0),
    ("6-14 Hz", 6.0, 14.0),
    ("8-12 Hz", 8.0, 12.0),
    ("14-25 Hz", 14.0, 25.0),
)


@dataclass
class Stream:
    case: str
    name: str
    side: str
    topic: str
    timestamps_ns: np.ndarray
    positions_m: np.ndarray
    quaternions_xyzw: np.ndarray
    bag_timestamps_ns: np.ndarray
    status_filtered_samples: int = 0


@dataclass
class UniformStream:
    stream: Stream
    time_s: np.ndarray
    position_m: np.ndarray
    orientation_rad: np.ndarray
    fs_hz: float


def stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def pose_values(pose: Any) -> tuple[np.ndarray, np.ndarray]:
    position = np.array(
        [pose.position.x, pose.position.y, pose.position.z], dtype=np.float64
    )
    quaternion = np.array(
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
        dtype=np.float64,
    )
    return position, quaternion


def wanted_topics(side: str) -> dict[str, str]:
    return {
        "raw_controller": STREAM_TOPICS["raw_controller"](side),
        "raw_smpl_hand": STREAM_TOPICS["raw_smpl_hand"],
        "corrected_palm": STREAM_TOPICS["corrected_palm"],
        "corrected_ik_palm": STREAM_TOPICS["corrected_ik_palm"],
        "tcp_palm": STREAM_TOPICS["tcp_palm"](side),
    }


def read_case(root: Path, case: str) -> list[Stream]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(root / case), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    wanted_by_topic: dict[str, list[tuple[str, str]]] = {}
    for side in SIDES:
        for name, topic in wanted_topics(side).items():
            wanted_by_topic.setdefault(topic, []).append((name, side))

    records: dict[tuple[str, str], list[tuple[int, np.ndarray, np.ndarray, int]]] = {}
    status_by_stamp_ns: dict[int, dict[str, Any]] = {}
    message_classes: dict[str, Any] = {}
    while reader.has_next():
        topic, serialized, bag_time_ns = reader.read_next()
        if topic == CORRECTED_STATUS_TOPIC:
            if topic not in message_classes:
                message_classes[topic] = get_message(topic_types[topic])
            message = deserialize_message(serialized, message_classes[topic])
            try:
                payload = json.loads(message.data)
                source_stamp_ns = int(payload.get("source_stamp_ns", 0))
            except (json.JSONDecodeError, AttributeError, TypeError, ValueError):
                continue
            if source_stamp_ns > 0:
                status_by_stamp_ns[source_stamp_ns] = payload
            continue
        if topic not in wanted_by_topic:
            continue
        if topic not in message_classes:
            message_classes[topic] = get_message(topic_types[topic])
        message = deserialize_message(serialized, message_classes[topic])
        source_time_ns = stamp_ns(message.header.stamp)
        if source_time_ns <= 0:
            source_time_ns = int(bag_time_ns)
        for name, side in wanted_by_topic[topic]:
            if hasattr(message, "pose"):
                position, quaternion = pose_values(message.pose)
            else:
                if len(message.poses) <= HAND_INDEX[side]:
                    continue
                position, quaternion = pose_values(message.poses[HAND_INDEX[side]])
            records.setdefault((name, side), []).append(
                (source_time_ns, position, quaternion, int(bag_time_ns))
            )

    streams: list[Stream] = []
    for (name, side), rows in sorted(records.items()):
        rows.sort(key=lambda row: row[0])
        status_filtered_samples = 0
        if name in STATUS_FILTERED_STREAMS:
            original_count = len(rows)
            rows = [
                row
                for row in rows
                if (
                    status_by_stamp_ns.get(row[0], {})
                    .get(side, {})
                    .get("corrected", True)
                )
            ]
            status_filtered_samples = original_count - len(rows)
        streams.append(
            Stream(
                case=case,
                name=name,
                side=side,
                topic=wanted_topics(side)[name],
                timestamps_ns=np.asarray([row[0] for row in rows], dtype=np.int64),
                positions_m=np.asarray([row[1] for row in rows], dtype=np.float64),
                quaternions_xyzw=np.asarray([row[2] for row in rows], dtype=np.float64),
                bag_timestamps_ns=np.asarray([row[3] for row in rows], dtype=np.int64),
                status_filtered_samples=status_filtered_samples,
            )
        )
    return streams


def finite_quaternions(quaternions: np.ndarray) -> tuple[np.ndarray, int]:
    finite = np.isfinite(quaternions).all(axis=1)
    norms = np.linalg.norm(quaternions, axis=1)
    valid = finite & (norms > 1.0e-9)
    normalized = np.zeros_like(quaternions)
    normalized[valid] = quaternions[valid] / norms[valid, None]
    return normalized, int((~valid).sum())


def quality_row(stream: Stream) -> dict[str, Any]:
    timestamps = stream.timestamps_ns.astype(np.float64) * 1.0e-9
    dt = np.diff(timestamps)
    positive_dt = dt[dt > 0.0]
    quaternions, invalid_quaternions = finite_quaternions(stream.quaternions_xyzw)
    finite_values = np.isfinite(stream.positions_m).all(axis=1) & np.isfinite(
        quaternions
    ).all(axis=1)
    if len(timestamps) > 1 and len(positive_dt):
        duration_s = float(timestamps[-1] - timestamps[0])
        median_dt_s = float(np.median(positive_dt))
        rate_hz = 1.0 / median_dt_s
        gaps = positive_dt[positive_dt > 2.0 * median_dt_s]
        max_gap_ms = float(np.max(positive_dt) * 1000.0)
        p50_dt_ms, p95_dt_ms, p99_dt_ms = (
            float(np.percentile(positive_dt, percentile) * 1000.0)
            for percentile in (50, 95, 99)
        )
    else:
        duration_s = 0.0
        rate_hz = 0.0
        gaps = np.array([], dtype=np.float64)
        max_gap_ms = 0.0
        p50_dt_ms = p95_dt_ms = p99_dt_ms = 0.0
    return {
        "case": stream.case,
        "condition": CONDITION_LABELS[stream.case],
        "side": stream.side,
        "stream": stream.name,
        "topic": stream.topic,
        "samples": int(len(timestamps)),
        "duration_s": duration_s,
        "median_rate_hz": rate_hz,
        "dt_p50_ms": p50_dt_ms,
        "dt_p95_ms": p95_dt_ms,
        "dt_p99_ms": p99_dt_ms,
        "max_dt_ms": max_gap_ms,
        "gaps_over_2x_median": int(len(gaps)),
        "duplicate_timestamps": int(np.sum(dt == 0.0)),
        "nonmonotonic_timestamps": int(np.sum(dt < 0.0)),
        "finite_pose_fraction": float(np.mean(finite_values)) if len(finite_values) else 0.0,
        "invalid_quaternions": invalid_quaternions,
        "status_filtered_samples": int(stream.status_filtered_samples),
    }


def deduplicate_stream(stream: Stream) -> Stream:
    if len(stream.timestamps_ns) < 2:
        return stream
    keep = np.concatenate(
        ([True], np.diff(stream.timestamps_ns.astype(np.int64)) > 0)
    )
    return Stream(
        case=stream.case,
        name=stream.name,
        side=stream.side,
        topic=stream.topic,
        timestamps_ns=stream.timestamps_ns[keep],
        positions_m=stream.positions_m[keep],
        quaternions_xyzw=stream.quaternions_xyzw[keep],
        bag_timestamps_ns=stream.bag_timestamps_ns[keep],
        status_filtered_samples=stream.status_filtered_samples,
    )


def uniformize(stream: Stream, trim_s: float = 1.0) -> UniformStream:
    stream = deduplicate_stream(stream)
    timestamps_s = stream.timestamps_ns.astype(np.float64) * 1.0e-9
    positions = stream.positions_m
    quaternions, invalid_count = finite_quaternions(stream.quaternions_xyzw)
    valid = np.isfinite(timestamps_s) & np.isfinite(positions).all(axis=1)
    valid &= np.isfinite(quaternions).all(axis=1)
    timestamps_s = timestamps_s[valid]
    positions = positions[valid]
    quaternions = quaternions[valid]
    if len(timestamps_s) < 32:
        raise ValueError(f"not enough valid samples for {stream.case}/{stream.name}/{stream.side}")
    dt = np.diff(timestamps_s)
    dt = dt[dt > 0.0]
    fs_hz = 1.0 / float(np.median(dt))
    start_s = timestamps_s[0] + trim_s
    end_s = timestamps_s[-1] - trim_s
    if end_s <= start_s:
        raise ValueError(f"trimmed interval is empty for {stream.case}/{stream.name}/{stream.side}")
    grid = np.arange(start_s, end_s, 1.0 / fs_hz, dtype=np.float64)
    if len(grid) < 128:
        raise ValueError(f"trimmed interval is too short for {stream.case}/{stream.name}/{stream.side}")
    position_grid = np.column_stack(
        [np.interp(grid, timestamps_s, positions[:, axis]) for axis in range(3)]
    )
    # Canonicalize signs before Slerp so the interpolation follows the short arc.
    quaternions = quaternions.copy()
    for index in range(1, len(quaternions)):
        if float(np.dot(quaternions[index - 1], quaternions[index])) < 0.0:
            quaternions[index] *= -1.0
    rotations = Rotation.from_quat(quaternions)
    rotation_grid = Slerp(timestamps_s, rotations)(grid)
    reference = rotation_grid[len(rotation_grid) // 2]
    orientation_grid = (rotation_grid * reference.inv()).as_rotvec()
    if invalid_count:
        # The quality table records this; keeping the analysis finite is the priority.
        pass
    return UniformStream(
        stream=stream,
        time_s=grid - grid[0],
        position_m=position_grid,
        orientation_rad=orientation_grid,
        fs_hz=fs_hz,
    )


def vector_psd(values: np.ndarray, fs_hz: float, unit_scale: float) -> tuple[np.ndarray, np.ndarray]:
    values = detrend(values, axis=0, type="linear") * unit_scale
    nperseg = min(1024, len(values))
    if nperseg < 128:
        nperseg = len(values)
    noverlap = nperseg // 2 if nperseg > 1 else 0
    component_psd = []
    frequencies = None
    for axis in range(3):
        frequencies, power = welch(
            values[:, axis],
            fs=fs_hz,
            window="hann",
            nperseg=nperseg,
            noverlap=noverlap,
            detrend="linear",
            scaling="density",
        )
        component_psd.append(power)
    return frequencies, np.sum(np.vstack(component_psd), axis=0)


def band_power(frequencies: np.ndarray, power: np.ndarray, low: float, high: float) -> float:
    mask = (frequencies >= low) & (frequencies <= high)
    if int(mask.sum()) < 2:
        return 0.0
    return float(np.trapz(power[mask], frequencies[mask]))


def peak_metrics(frequencies: np.ndarray, power: np.ndarray) -> tuple[float, float, float, float]:
    # Do not force a classical 8–12 Hz answer.  Search 3–14 Hz and report
    # 8–12 Hz integrated power separately.
    mask = (frequencies >= 3.0) & (frequencies <= 14.0)
    if int(mask.sum()) < 3:
        return math.nan, math.nan, math.nan, math.nan
    selected_frequencies = frequencies[mask]
    selected_power = power[mask]
    peak_index = int(np.argmax(selected_power))
    peak_power = float(selected_power[peak_index])
    peak_frequency = float(selected_frequencies[peak_index])
    peaks, properties = find_peaks(selected_power)
    if len(peaks):
        best = int(peaks[np.argmax(properties.get("prominences", selected_power[peaks]))])
        peak_index = best
        peak_power = float(selected_power[best])
        peak_frequency = float(selected_frequencies[best])
    neighborhood = np.ones_like(selected_power, dtype=bool)
    lower = max(0, peak_index - 2)
    upper = min(len(neighborhood), peak_index + 3)
    neighborhood[lower:upper] = False
    noise_values = selected_power[neighborhood]
    noise_floor = float(np.median(noise_values)) if len(noise_values) else math.nan
    snr_db = (
        float(10.0 * np.log10(peak_power / noise_floor))
        if noise_floor > 0.0 and peak_power > 0.0
        else math.nan
    )
    return peak_frequency, peak_power, noise_floor, snr_db


def calculate_spectral_rows(uniform: UniformStream) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    spectral_rows: list[dict[str, Any]] = []
    metric_rows: list[dict[str, Any]] = []
    signal_values = {
        "position": (uniform.position_m, 1000.0, "mm"),
        "orientation": (uniform.orientation_rad, 180.0 / math.pi, "deg"),
    }
    for signal, (values, unit_scale, unit) in signal_values.items():
        frequencies, power = vector_psd(values, uniform.fs_hz, unit_scale)
        for frequency, psd_value in zip(frequencies, power):
            if frequency <= 25.0:
                spectral_rows.append(
                    {
                        "case": uniform.stream.case,
                        "condition": CONDITION_LABELS[uniform.stream.case],
                        "side": uniform.stream.side,
                        "stream": uniform.stream.name,
                        "signal": signal,
                        "frequency_hz": float(frequency),
                        "psd": float(psd_value),
                        "unit": f"{unit}^2/Hz",
                    }
                )
        peak_frequency, peak_power, noise_floor, snr_db = peak_metrics(frequencies, power)
        detrended_values = detrend(values, axis=0, type="linear") * unit_scale
        row: dict[str, Any] = {
            "case": uniform.stream.case,
            "condition": CONDITION_LABELS[uniform.stream.case],
            "side": uniform.stream.side,
            "stream": uniform.stream.name,
            "signal": signal,
            "unit": unit,
            "fs_hz": float(uniform.fs_hz),
            "samples": int(len(uniform.time_s)),
            "rms_total": float(np.sqrt(np.mean(np.sum(detrended_values**2, axis=1)))),
            "peak_frequency_3_14_hz": peak_frequency,
            "peak_psd_3_14": peak_power,
            "peak_noise_floor_3_14": noise_floor,
            "peak_snr_db_3_14": snr_db,
        }
        for label, low, high in BANDS:
            power_value = band_power(frequencies, power, low, high)
            row[f"band_power_{label.replace('-', '_').replace(' ', '_').replace('.', 'p')}"] = power_value
            row[f"band_rms_{label.replace('-', '_').replace(' ', '_').replace('.', 'p')}"] = math.sqrt(
                max(power_value, 0.0)
            )
        metric_rows.append(row)
    return spectral_rows, metric_rows


def write_csv(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    rows = list(rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("\n", encoding="utf-8")
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def read_manifest(root: Path) -> list[dict[str, str]]:
    with (root / "manifest.tsv").open(encoding="utf-8") as stream:
        return list(csv.DictReader(stream, delimiter="\t"))


def plot_psd(
    output: Path,
    spectral_rows: list[dict[str, Any]],
    side: str,
    signal: str,
    stream_name: str,
) -> None:
    figure, axis = plt.subplots(figsize=(10, 5.5), constrained_layout=True)
    colors = {"fixed": "#4c78a8", "elbow_supported": "#f58518", "arm_unsupported": "#54a24b"}
    subset = [
        row
        for row in spectral_rows
        if row["side"] == side and row["signal"] == signal and row["stream"] == stream_name
    ]
    for case in CONDITIONS:
        rows = [row for row in subset if row["case"] == case]
        rows.sort(key=lambda row: row["frequency_hz"])
        if not rows:
            continue
        axis.semilogy(
            [row["frequency_hz"] for row in rows],
            [max(float(row["psd"]), 1.0e-14) for row in rows],
            label=CONDITION_LABELS[case],
            color=colors[case],
            linewidth=1.8,
        )
    axis.axvspan(0.1, 0.5, color="#9ecae1", alpha=0.16, label="Low drift")
    axis.axvspan(0.5, 3.0, color="#c7e9c0", alpha=0.16, label="Sway / correction")
    axis.axvspan(8.0, 12.0, color="#fdd0a2", alpha=0.24, label="Tremor search band")
    axis.set_xlim(0.0, 25.0)
    axis.set_xlabel("Frequency (Hz)")
    axis.set_ylabel("PSD (mm²/Hz)" if signal == "position" else "PSD (deg²/Hz)")
    axis.set_title(f"{SIGNAL_LABELS[signal]} PSD — {side.capitalize()} — raw controller")
    axis.grid(True, which="both", alpha=0.25)
    handles, labels = axis.get_legend_handles_labels()
    # Keep one legend entry per condition plus the three shaded bands.
    axis.legend(handles, labels, loc="best", fontsize=8, ncol=2)
    figure.savefig(output, dpi=160)
    plt.close(figure)


def plot_pipeline(
    output: Path,
    spectral_rows: list[dict[str, Any]],
    side: str,
    signal: str,
) -> None:
    figure, axes = plt.subplots(1, 3, figsize=(14, 4.2), sharey=True, constrained_layout=True)
    colors = {"raw_controller": "#7f7f7f", "tcp_palm": "#e45756", "corrected_palm": "#2ca02c"}
    labels = {
        "raw_controller": "Raw controller",
        "tcp_palm": "TCP palm",
        "corrected_palm": "Corrected palm",
    }
    for axis, case in zip(axes, CONDITIONS):
        for stream_name in ("raw_controller", "tcp_palm", "corrected_palm"):
            rows = [
                row
                for row in spectral_rows
                if row["side"] == side
                and row["case"] == case
                and row["signal"] == signal
                and row["stream"] == stream_name
            ]
            rows.sort(key=lambda row: row["frequency_hz"])
            if rows:
                axis.semilogy(
                    [row["frequency_hz"] for row in rows],
                    [max(float(row["psd"]), 1.0e-14) for row in rows],
                    label=labels[stream_name],
                    color=colors[stream_name],
                    linewidth=1.4,
                )
        axis.axvspan(8.0, 12.0, color="#fdd0a2", alpha=0.25)
        axis.set_title(CONDITION_LABELS[case])
        axis.set_xlim(0, 25)
        axis.set_xlabel("Hz")
        axis.grid(True, which="both", alpha=0.25)
    axes[0].set_ylabel("PSD (mm²/Hz)" if signal == "position" else "PSD (deg²/Hz)")
    axes[-1].legend(loc="best", fontsize=8)
    figure.suptitle(f"Pipeline PSD comparison — {side.capitalize()} — {signal}")
    figure.savefig(output, dpi=160)
    plt.close(figure)


def plot_band_power(output: Path, metric_rows: list[dict[str, Any]], side: str, signal: str) -> None:
    figure, axis = plt.subplots(figsize=(10, 5), constrained_layout=True)
    labels = [band[0] for band in BANDS]
    x = np.arange(len(labels), dtype=np.float64)
    width = 0.24
    colors = {"fixed": "#4c78a8", "elbow_supported": "#f58518", "arm_unsupported": "#54a24b"}
    signal_rows = [
        row
        for row in metric_rows
        if row["side"] == side and row["signal"] == signal and row["stream"] == "raw_controller"
    ]
    for index, case in enumerate(CONDITIONS):
        row = next((item for item in signal_rows if item["case"] == case), None)
        if row is None:
            continue
        values = [
            float(row[f"band_power_{label.replace('-', '_').replace(' ', '_').replace('.', 'p')}"])
            for label in labels
        ]
        axis.bar(x + (index - 1) * width, values, width, label=CONDITION_LABELS[case], color=colors[case])
    axis.set_xticks(x, labels, rotation=20)
    axis.set_ylabel("Integrated PSD (mm²)" if signal == "position" else "Integrated PSD (deg²)")
    axis.set_title(f"Band power — {side.capitalize()} — raw controller — {signal}")
    axis.grid(axis="y", alpha=0.25)
    axis.legend(fontsize=8)
    figure.savefig(output, dpi=160)
    plt.close(figure)


def plot_time_series(output: Path, uniform_streams: dict[tuple[str, str, str], UniformStream], side: str) -> None:
    figure, axes = plt.subplots(2, 3, figsize=(14, 6), sharex=False, constrained_layout=True)
    for column, case in enumerate(CONDITIONS):
        uniform = uniform_streams[(case, "raw_controller", side)]
        position = detrend(uniform.position_m, axis=0, type="linear") * 1000.0
        orientation = detrend(uniform.orientation_rad, axis=0, type="linear") * (180.0 / math.pi)
        position_norm = np.linalg.norm(position, axis=1)
        orientation_norm = np.linalg.norm(orientation, axis=1)
        axes[0, column].plot(uniform.time_s, position_norm, color="#2ca02c", linewidth=0.7)
        axes[1, column].plot(uniform.time_s, orientation_norm, color="#9467bd", linewidth=0.7)
        axes[0, column].set_title(CONDITION_LABELS[case])
        axes[1, column].set_xlabel("Time (s)")
        for row in range(2):
            axes[row, column].grid(alpha=0.25)
    axes[0, 0].set_ylabel("Position residual norm (mm)")
    axes[1, 0].set_ylabel("Orientation residual norm (deg)")
    figure.suptitle(f"Trimmed raw-controller residuals — {side.capitalize()}")
    figure.savefig(output, dpi=160)
    plt.close(figure)


def make_notebook(output: Path, root: Path, analysis_dir: Path, findings: list[str]) -> None:
    source = [
        "# PICO hand tremor spectral analysis\n",
        "\n",
        "This notebook is the reproducible companion for the generated report.\n",
        "\n",
        "## tl;dr\n",
        "\n",
        *[f"- {finding}\n" for finding in findings],
        "\n",
        "## Context & Methods\n",
        "\n",
        "The analysis uses the PICO source timestamps, trims one second from each end, resamples each stream at its median source rate, and computes Welch PSDs for the 3-D position vector and SO(3) rotation-vector residual. The fixed-controller capture is a device-noise reference; subtraction is treated only as an excess-energy diagnostic.\n",
        "\n",
        "## Data\n",
        f"\nInput root: `{root}`\n\nGenerated outputs: `{analysis_dir}`\n",
        "\n",
        "## Results\n",
        "\nSee `spectral_metrics.csv`, `data_quality.csv`, and the PNG figures in this directory.\n",
        "\n",
        "## Takeaways\n",
        "\n",
        "The report separates stable narrow-band peaks from broad-band noise and documents the recording completeness caveats.\n",
    ]
    cells = [
        {"cell_type": "markdown", "metadata": {}, "source": source},
        {
            "cell_type": "markdown",
            "metadata": {},
            "source": ["### Re-run\n", "\n", f"`../.venv/bin/python scripts/analyze_pico_tremor.py --root {root} --output {analysis_dir}`\n"],
        },
        {
            "cell_type": "code",
            "execution_count": None,
            "metadata": {},
            "outputs": [],
            "source": [
                "from pathlib import Path\n",
                "import csv\n",
                f"analysis_dir = Path({str(analysis_dir)!r})\n",
                "with (analysis_dir / 'spectral_metrics.csv').open() as handle:\n",
                "    rows = list(csv.DictReader(handle))\n",
                "print(f'{len(rows)} spectral metric rows loaded')\n",
                "print(rows[:2])\n",
            ],
        },
    ]
    notebook = {
        "cells": cells,
        "metadata": {
            "kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"},
            "language_info": {"name": "python", "version": platform.python_version()},
        },
        "nbformat": 4,
        "nbformat_minor": 5,
    }
    output.write_text(json.dumps(notebook, ensure_ascii=False, indent=2), encoding="utf-8")


def make_artifact(
    output: Path,
    quality_rows: list[dict[str, Any]],
    metric_rows: list[dict[str, Any]],
    spectral_rows: list[dict[str, Any]],
    findings: list[str],
) -> None:
    def compact(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
        output_rows = []
        for row in rows:
            converted = {}
            for key, value in row.items():
                if isinstance(value, (float, np.floating)):
                    converted[key] = None if not math.isfinite(float(value)) else round(float(value), 9)
                elif isinstance(value, (np.integer,)):
                    converted[key] = int(value)
                else:
                    converted[key] = value
            output_rows.append(converted)
        return output_rows

    position_rows = [
        row
        for row in spectral_rows
        if row["stream"] == "raw_controller" and row["signal"] == "position"
    ]
    orientation_rows = [
        row
        for row in spectral_rows
        if row["stream"] == "raw_controller" and row["signal"] == "orientation"
    ]
    datasets: dict[str, list[dict[str, Any]]] = {
        "quality": compact(quality_rows),
        "spectral_metrics": compact(metric_rows),
        "psd_position_left": compact([row for row in position_rows if row["side"] == "left"]),
        "psd_position_right": compact([row for row in position_rows if row["side"] == "right"]),
        "psd_orientation_left": compact([row for row in orientation_rows if row["side"] == "left"]),
        "psd_orientation_right": compact([row for row in orientation_rows if row["side"] == "right"]),
    }
    source_id = "pico_tremor_analysis"
    charts = []
    for signal, unit, title_prefix in (
        ("position", "mm²/Hz", "Position"),
        ("orientation", "deg²/Hz", "SO(3) orientation"),
    ):
        for side in SIDES:
            dataset = f"psd_{signal}_{side}"
            charts.append(
                {
                    "id": f"{dataset}_chart",
                    "title": f"{title_prefix} PSD — {side.capitalize()} raw controller",
                    "subtitle": "Shaded 8–12 Hz region is the tremor search band; lines are raw-controller vector PSDs.",
                    "type": "line",
                    "dataset": dataset,
                    "sourceId": source_id,
                    "encodings": {
                        "x": {"field": "frequency_hz", "type": "quantitative", "label": "Frequency (Hz)"},
                        "y": {"field": "psd", "type": "quantitative", "label": f"PSD ({unit})"},
                        "color": {"field": "condition", "type": "nominal", "label": "Condition"},
                        "tooltip": [
                            {"field": "case", "type": "nominal", "label": "Case"},
                            {"field": "frequency_hz", "type": "quantitative", "label": "Frequency (Hz)"},
                            {"field": "psd", "type": "quantitative", "label": f"PSD ({unit})"},
                        ],
                    },
                    "xAxisTitle": "Frequency (Hz)",
                    "yAxisTitle": f"PSD ({unit})",
                    "unit": unit,
                    "maxRows": 1800,
                }
            )
    summary_table_rows = []
    for row in metric_rows:
        if row["stream"] != "raw_controller":
            continue
        summary_table_rows.append(
            {
                "condition": row["condition"],
                "side": row["side"],
                "signal": row["signal"],
                "peak_frequency_3_14_hz": row["peak_frequency_3_14_hz"],
                "peak_snr_db_3_14": row["peak_snr_db_3_14"],
                "rms_total": row["rms_total"],
                "band_rms_8_12_Hz": row["band_rms_8_12_Hz"],
            }
        )
    datasets["peak_summary"] = compact(summary_table_rows)
    generated_at = datetime.now(timezone.utc).isoformat()
    manifest = {
        "version": 1,
        "surface": "report",
        "title": "PICO hand tremor spectral analysis",
        "description": "Data-quality and frequency-domain analysis of three 30-second PICO hand-hold conditions.",
        "generatedAt": generated_at,
        "sources": [{"id": source_id, "label": "ROS bag capture and deterministic Python analysis", "path": "analysis_source.sql"}],
        "charts": charts,
        "tables": [
            {
                "id": "quality_table",
                "title": "Recording quality by stream",
                "subtitle": "Source timestamp completeness and cadence checks.",
                "dataset": "quality",
                "sourceId": source_id,
                "defaultSort": {"field": "condition", "direction": "asc"},
                "columns": [
                    {"field": "condition", "label": "Condition", "type": "text"},
                    {"field": "side", "label": "Side", "type": "text"},
                    {"field": "stream", "label": "Stream", "type": "text"},
                    {"field": "samples", "label": "Samples", "type": "number"},
                    {"field": "duration_s", "label": "Duration", "type": "number", "unit": "s"},
                    {"field": "median_rate_hz", "label": "Median rate", "type": "number", "unit": "Hz"},
                    {"field": "max_dt_ms", "label": "Max dt", "type": "number", "unit": "ms"},
                    {"field": "duplicate_timestamps", "label": "Duplicate stamps", "type": "number"},
                    {"field": "finite_pose_fraction", "label": "Finite fraction", "type": "number"},
                ],
            },
            {
                "id": "peak_table",
                "title": "Raw-controller spectral summary",
                "subtitle": "Dominant 3–14 Hz peak and integrated 8–12 Hz energy.",
                "dataset": "peak_summary",
                "sourceId": source_id,
                "defaultSort": {"field": "peak_snr_db_3_14", "direction": "desc"},
                "columns": [
                    {"field": "condition", "label": "Condition", "type": "text"},
                    {"field": "side", "label": "Side", "type": "text"},
                    {"field": "signal", "label": "Signal", "type": "text"},
                    {"field": "peak_frequency_3_14_hz", "label": "Peak 3–14 Hz", "type": "number", "unit": "Hz"},
                    {"field": "peak_snr_db_3_14", "label": "Peak SNR", "type": "number", "unit": "dB"},
                    {"field": "rms_total", "label": "Total RMS", "type": "number"},
                    {"field": "band_rms_8_12_Hz", "label": "8–12 Hz RMS", "type": "number"},
                ],
            },
        ],
        "blocks": [
            {"id": "title", "type": "markdown", "body": "# PICO hand tremor spectral analysis"},
            {
                "id": "summary",
                "type": "markdown",
                "sourceId": source_id,
                "body": "## Technical summary\n\n" + "\n".join(f"- {finding}" for finding in findings),
            },
            {"id": "position_left", "type": "chart", "chartId": "psd_position_left_chart"},
            {"id": "position_right", "type": "chart", "chartId": "psd_position_right_chart"},
            {"id": "orientation_left", "type": "chart", "chartId": "psd_orientation_left_chart"},
            {"id": "orientation_right", "type": "chart", "chartId": "psd_orientation_right_chart"},
            {
                "id": "data_quality",
                "type": "markdown",
                "sourceId": source_id,
                "body": "## Data quality and metric definitions\n\nThe recording grain is one source-timestamped pose sample per stream. Position PSD is the sum of the three Cartesian component PSDs after linear detrending. Orientation PSD is the sum of the three SO(3) rotation-vector component PSDs, expressed in degrees. Each stream is trimmed by one second at both ends before resampling and Welch estimation.",
            },
            {"id": "quality_table_block", "type": "table", "tableId": "quality_table"},
            {"id": "peak_table_block", "type": "table", "tableId": "peak_table"},
            {
                "id": "limitations",
                "type": "markdown",
                "sourceId": source_id,
                "body": "## Limitations and next steps\n\nThe fixed-controller capture is a useful device-noise baseline but is not a matched mechanical posture, so excess-spectrum subtraction is diagnostic only. The elbow-supported derived palm topics start later than the raw controller topics because discovery completed later. A repeat set with the same hand pose across conditions would strengthen causal attribution. Use the measured peak only after checking whether it is narrow-band and absent or much weaker in the fixed condition.",
            },
        ],
    }
    payload = {
        "surface": "report",
        "manifest": manifest,
        "snapshot": {
            "version": 1,
            "generatedAt": generated_at,
            "status": "ready",
            "datasets": datasets,
        },
        "sources": [
            {
                "id": source_id,
                "path": "analysis_source.sql",
                "query": {
                    "engine": "duckdb",
                    "language": "sql",
                    "sql": "SELECT * FROM read_csv_auto('spectral_metrics.csv')",
                    "description": "Reads the bounded spectral metrics generated from the ROS bag source timestamps and SO(3) conversion.",
                    "tables_used": ["spectral_metrics.csv"],
                    "filters": ["three recorded conditions", "one-second trim at each end", "0-25 Hz PSD export"],
                    "executed_at": generated_at,
                },
            }
        ],
    }
    (output / "artifact.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    root = args.root.resolve()
    output = (args.output or root / "analysis").resolve()
    output.mkdir(parents=True, exist_ok=True)
    if not (root / "manifest.tsv").is_file():
        raise SystemExit(f"manifest.tsv not found under {root}")

    all_streams: list[Stream] = []
    for case in CONDITIONS:
        all_streams.extend(read_case(root, case))
    quality_rows = [quality_row(stream) for stream in all_streams]
    stream_map = {(stream.case, stream.name, stream.side): stream for stream in all_streams}
    uniform_streams: dict[tuple[str, str, str], UniformStream] = {}
    spectral_rows: list[dict[str, Any]] = []
    metric_rows: list[dict[str, Any]] = []
    for key, stream in sorted(stream_map.items()):
        uniform = uniformize(stream)
        uniform_streams[key] = uniform
        rows, metrics = calculate_spectral_rows(uniform)
        spectral_rows.extend(rows)
        metric_rows.extend(metrics)

    write_csv(output / "data_quality.csv", quality_rows)
    write_csv(output / "spectral_metrics.csv", metric_rows)
    write_csv(output / "psd_long.csv", spectral_rows)

    # The raw controller pose is the primary human-motion signal.  Derived
    # skeleton/TCP streams remain available for diagnosing retargeting artifacts.
    corrected_metrics = [
        row for row in metric_rows if row["stream"] == "raw_controller"
    ]
    # A compact comparison table makes the fixed-device reference explicit.
    comparison_rows: list[dict[str, Any]] = []
    for side in SIDES:
        fixed_rows = {
            row["signal"]: row
            for row in corrected_metrics
            if row["side"] == side and row["case"] == "fixed"
        }
        for row in corrected_metrics:
            if row["side"] != side:
                continue
            fixed = fixed_rows[row["signal"]]
            band_key = "band_power_8_12_Hz"
            fixed_power = float(fixed[band_key])
            current_power = float(row[band_key])
            comparison_rows.append(
                {
                    "case": row["case"],
                    "condition": row["condition"],
                    "side": side,
                    "signal": row["signal"],
                    "fixed_reference_8_12_power": fixed_power,
                    "condition_8_12_power": current_power,
                    "condition_to_fixed_ratio": current_power / fixed_power if fixed_power > 0 else math.nan,
                    "positive_excess_power": max(0.0, current_power - fixed_power),
                }
            )
    write_csv(output / "fixed_reference_comparison.csv", comparison_rows)

    figures = output / "figures"
    figures.mkdir(exist_ok=True)
    for side in SIDES:
        plot_psd(figures / f"psd_position_{side}.png", spectral_rows, side, "position", "raw_controller")
        plot_psd(figures / f"psd_orientation_{side}.png", spectral_rows, side, "orientation", "raw_controller")
        plot_pipeline(figures / f"pipeline_position_{side}.png", spectral_rows, side, "position")
        plot_pipeline(figures / f"pipeline_orientation_{side}.png", spectral_rows, side, "orientation")
        plot_band_power(figures / f"band_power_position_{side}.png", metric_rows, side, "position")
        plot_band_power(figures / f"band_power_orientation_{side}.png", metric_rows, side, "orientation")
        plot_time_series(figures / f"residual_timeseries_{side}.png", uniform_streams, side)

    fixed_peaks = {
        (row["side"], row["signal"]): row
        for row in corrected_metrics
        if row["case"] == "fixed"
    }
    findings = []
    findings.append(
        f"Three conditions were recorded once each for approximately 30 s; {len(all_streams)} stream-side series were extracted from the ROS bags. The raw controller pose is the primary tremor signal."
    )
    status_filtered_samples = sum(
        stream.status_filtered_samples
        for stream in all_streams
        if stream.name in STATUS_FILTERED_STREAMS
    )
    if status_filtered_samples:
        findings.append(
            f"The corrected-stream spectral diagnostics excluded {status_filtered_samples} topic samples flagged by the per-frame status topic as not currently corrected; these samples are retained in the bag but are not interpolated into the corrected spectra."
        )
    for side in SIDES:
        for signal in ("position", "orientation"):
            rows = [
                row
                for row in corrected_metrics
                if row["side"] == side and row["signal"] == signal
            ]
            strongest = max(rows, key=lambda row: float(row["band_power_8_12_Hz"]))
            fixed = fixed_peaks[(side, signal)]
            ratio = float(strongest["band_power_8_12_Hz"]) / max(float(fixed["band_power_8_12_Hz"]), 1.0e-18)
            findings.append(
                f"{side.capitalize()} {signal}: the largest raw-controller 8–12 Hz integrated power is in {strongest['condition']} ({float(strongest['band_power_8_12_Hz']):.4g} {('mm²' if signal == 'position' else 'deg²')}, {ratio:.2f}× the fixed reference), with the 3–14 Hz peak at {float(strongest['peak_frequency_3_14_hz']):.2f} Hz."
            )
    for side in SIDES:
        for signal in ("position", "orientation"):
            raw_fixed = next(
                row for row in metric_rows
                if row["stream"] == "raw_controller"
                and row["case"] == "fixed"
                and row["side"] == side
                and row["signal"] == signal
            )
            corrected_fixed = next(
                row for row in metric_rows
                if row["stream"] == "corrected_palm"
                and row["case"] == "fixed"
                and row["side"] == side
                and row["signal"] == signal
            )
            raw_power = float(raw_fixed["band_power_8_12_Hz"])
            corrected_power = float(corrected_fixed["band_power_8_12_Hz"])
            if raw_power > 0.0 and corrected_power / raw_power > 100.0:
                findings.append(
                    f"Pipeline diagnostic: status-filtered fixed-condition corrected_palm 8–12 Hz power is {corrected_power / raw_power:.0f}× the raw-controller reference for {side} {signal}; inspect the corrected pipeline before using it as a tremor estimator."
                )
    findings.append(
        "The fixed capture is treated as a device-noise reference; a stronger supported or unsupported spectrum is evidence consistent with human motion, not proof by subtraction alone."
    )
    make_notebook(output / "pico_tremor_analysis.ipynb", root, output, findings)
    (output / "analysis_source.sql").write_text(
        "-- Bounded report source over the generated, reviewed spectral metrics.\n"
        "SELECT * FROM read_csv_auto('spectral_metrics.csv');\n",
        encoding="utf-8",
    )
    make_artifact(output, quality_rows, metric_rows, spectral_rows, findings)
    (output / "analysis_summary.json").write_text(
        json.dumps(
            {
                "input_root": str(root),
                "output_dir": str(output),
                "conditions": list(CONDITIONS),
                "stream_count": len(all_streams),
                "quality_rows": len(quality_rows),
                "spectral_metric_rows": len(metric_rows),
                "psd_rows": len(spectral_rows),
                "findings": findings,
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    print(json.dumps({"output": str(output), "findings": findings}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
