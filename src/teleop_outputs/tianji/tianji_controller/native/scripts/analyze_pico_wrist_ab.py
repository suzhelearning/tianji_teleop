#!/usr/bin/env python3
"""Compare wrist continuity with production and symmetric-J3 model limits."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


SIDES = ("left", "right")
WRIST_JOINTS = (5, 6, 7)


def percentile(values: list[float], fraction: float) -> float:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return 0.0
    index = min(len(finite) - 1, max(0, math.ceil(fraction * len(finite)) - 1))
    return finite[index]


def value(row: dict[str, str], name: str) -> float:
    parsed = float(row[name])
    return parsed if math.isfinite(parsed) else 0.0


def load(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2:
        raise ValueError(f"{path}: expected at least two joint telemetry rows")
    return rows


def summarize_side(
    rows: list[dict[str, str]], side: str, j3_margin: float,
    jump_threshold: float, correlation_window: float,
) -> dict[str, object]:
    bound_times: list[float] = []
    for row in rows:
        q = value(row, f"{side}_j3_reference_q")
        lower = value(row, f"{side}_j3_position_lower")
        upper = value(row, f"{side}_j3_position_upper")
        if min(q - lower, upper - q) <= j3_margin:
            bound_times.append(value(row, "control_time_seconds"))

    joint_metrics: dict[str, object] = {}
    all_qdot: list[float] = []
    all_qddot: list[float] = []
    all_jerk: list[float] = []
    total_variation = 0.0
    jump_events: list[tuple[float, float, int]] = []
    for joint in WRIST_JOINTS:
        positions = [value(row, f"{side}_j{joint}_reference_q") for row in rows]
        qdot = [abs(value(row, f"{side}_j{joint}_reference_qdot")) for row in rows]
        qddot = [abs(value(row, f"{side}_j{joint}_reference_qddot")) for row in rows]
        jerk = [abs(value(row, f"{side}_j{joint}_reference_jerk")) for row in rows]
        deltas = [abs(current - previous) for previous, current in zip(positions, positions[1:])]
        variation = sum(deltas)
        total_variation += variation
        all_qdot.extend(qdot)
        all_qddot.extend(qddot)
        all_jerk.extend(jerk)
        for index, delta in enumerate(deltas, start=1):
            if delta >= jump_threshold:
                jump_events.append(
                    (value(rows[index], "control_time_seconds"), delta, joint)
                )
        joint_metrics[f"j{joint}"] = {
            "q_total_variation_rad": variation,
            "delta_q_max_rad": max(deltas, default=0.0),
            "qdot_p95_rad_s": percentile(qdot, 0.95),
            "qdot_max_rad_s": max(qdot, default=0.0),
            "qddot_p95_rad_s2": percentile(qddot, 0.95),
            "qddot_max_rad_s2": max(qddot, default=0.0),
            "jerk_p95_rad_s3": percentile(jerk, 0.95),
            "jerk_max_rad_s3": max(jerk, default=0.0),
        }

    correlated = [
        event for event in jump_events
        if any(abs(event[0] - bound_time) <= correlation_window
               for bound_time in bound_times)
    ]
    return {
        "rows": len(rows),
        "j3_near_bound_cycles": len(bound_times),
        "wrist_q_total_variation_rad": total_variation,
        "wrist_qdot_p95_rad_s": percentile(all_qdot, 0.95),
        "wrist_qdot_max_rad_s": max(all_qdot, default=0.0),
        "wrist_qddot_p95_rad_s2": percentile(all_qddot, 0.95),
        "wrist_qddot_max_rad_s2": max(all_qddot, default=0.0),
        "wrist_jerk_p95_rad_s3": percentile(all_jerk, 0.95),
        "wrist_jerk_max_rad_s3": max(all_jerk, default=0.0),
        "wrist_jump_cycles": len(jump_events),
        "wrist_jumps_near_j3_bound": len(correlated),
        "joints": joint_metrics,
    }


def analyze(
    current_path: Path, symmetric_path: Path, j3_margin: float,
    jump_threshold: float, correlation_window: float,
) -> dict[str, object]:
    datasets = {
        "current": load(current_path),
        "symmetric_j3": load(symmetric_path),
    }
    summaries = {
        name: {
            side: summarize_side(rows, side, j3_margin, jump_threshold,
                                 correlation_window)
            for side in SIDES
        }
        for name, rows in datasets.items()
    }
    current_left = summaries["current"]["left"]
    symmetric_left = summaries["symmetric_j3"]["left"]
    current_correlated = int(current_left["wrist_jumps_near_j3_bound"])
    symmetric_correlated = int(symmetric_left["wrist_jumps_near_j3_bound"])
    current_variation = float(current_left["wrist_q_total_variation_rad"])
    symmetric_variation = float(symmetric_left["wrist_q_total_variation_rad"])
    if current_correlated > symmetric_correlated:
        diagnosis = "j3_boundary_correlated"
    elif current_variation > 1.25 * max(symmetric_variation, 1.0e-9):
        diagnosis = "current_limits_increase_wrist_motion"
    else:
        diagnosis = "not_proven_from_joint_limits"
    return {
        "thresholds": {
            "j3_bound_margin_rad": j3_margin,
            "wrist_jump_threshold_rad": jump_threshold,
            "correlation_window_seconds": correlation_window,
        },
        "datasets": summaries,
        "diagnosis": diagnosis,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--current", type=Path, required=True)
    parser.add_argument("--symmetric-j3", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--j3-bound-margin", type=float, default=0.02)
    parser.add_argument("--wrist-jump-threshold", type=float, default=0.15)
    parser.add_argument("--correlation-window", type=float, default=0.10)
    arguments = parser.parse_args()
    report = analyze(
        arguments.current, arguments.symmetric_j3,
        arguments.j3_bound_margin, arguments.wrist_jump_threshold,
        arguments.correlation_window,
    )
    serialized = json.dumps(report, indent=2, sort_keys=True) + "\n"
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(serialized, encoding="utf-8")
    print(serialized, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
