#!/usr/bin/env python3
"""Compare the Spark-guided velocity QP against preserved velocity-QP runs."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable


PERCENTILES = (0.50, 0.95, 0.99)


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def percentile(values: Iterable[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    coordinate = (len(ordered) - 1) * fraction
    lower = int(coordinate)
    blend = coordinate - lower
    upper = min(lower + 1, len(ordered) - 1)
    return ordered[lower] * (1.0 - blend) + ordered[upper] * blend


def finite_values(rows: Iterable[dict[str, str]], key: str) -> list[float]:
    result: list[float] = []
    for row in rows:
        try:
            value = float(row[key])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(value):
            result.append(value)
    return result


def summarize_run(path: Path) -> dict[str, float | int | str]:
    rows = read_rows(path)
    live = [
        row
        for row in rows
        if row.get("pico_live") == "1"
        and row.get("left_target_stale") == "0"
        and row.get("right_target_stale") == "0"
    ]
    result: dict[str, float | int | str] = {
        "algorithm": rows[-1].get("algorithm", "unknown"),
        "rows": len(rows),
        "live_rows": len(live),
        "control_failures": int(rows[-1]["control_failures"]),
        "deadline_misses": int(rows[-1]["deadline_misses"]),
    }
    for side in ("left", "right"):
        for metric in ("position_error_m", "orientation_error_rad"):
            values = finite_values(live, f"{side}_{metric}")
            for label, fraction in zip(("p50", "p95", "p99"), PERCENTILES):
                result[f"{side}_{metric}_{label}"] = percentile(values, fraction)
            result[f"{side}_{metric}_max"] = max(values, default=math.nan)
    cycles = finite_values(rows, "cycle_time_us")
    result["cycle_p99_us"] = percentile(cycles, 0.99)
    result["cycle_max_us"] = max(cycles, default=math.nan)
    for side in ("left", "right"):
        for field in ("spark_posture_active", "spark_ik_accepted"):
            values = finite_values(live, f"{side}_{field}")
            result[f"{side}_{field}_mean"] = (
                sum(values) / len(values) if values else math.nan
            )
    return result


def summarize_joints(path: Path) -> dict[str, float | int]:
    rows = read_rows(path)
    result: dict[str, float | int] = {
        "rows": len(rows),
        "resets": sum(row.get("reset") == "1" for row in rows),
    }
    tolerance = 1.0e-6
    for side in ("left", "right"):
        violations = {"q": 0, "qdot": 0, "qddot": 0, "jerk": 0}
        maxima = {"qdot": 0.0, "qddot": 0.0, "jerk": 0.0}
        for row in rows:
            if row.get("reset") == "1":
                continue
            for joint in range(1, 8):
                prefix = f"{side}_j{joint}_"
                samples = {
                    "q": ("reference_q", "position_lower", "position_upper"),
                    "qdot": (
                        "reference_qdot",
                        "velocity_lower",
                        "velocity_upper",
                    ),
                    "qddot": (
                        "reference_qddot",
                        "acceleration_lower",
                        "acceleration_upper",
                    ),
                    "jerk": ("reference_jerk", "jerk_lower", "jerk_upper"),
                }
                for name, (value_name, lower_name, upper_name) in samples.items():
                    value = float(row[prefix + value_name])
                    lower = float(row[prefix + lower_name])
                    upper = float(row[prefix + upper_name])
                    if value < lower - tolerance or value > upper + tolerance:
                        violations[name] += 1
                    if name in maxima:
                        maxima[name] = max(maxima[name], abs(value))
        for name, count in violations.items():
            result[f"{side}_{name}_violations"] = count
        for name, value in maxima.items():
            result[f"{side}_{name}_max_abs"] = value
    return result


def markdown(
    runs: list[tuple[str, dict[str, float | int | str]]],
    joint_audits: list[tuple[str, dict[str, float | int]]],
) -> str:
    lines = [
        "# Spark-guided velocity-QP comparison",
        "",
        "All tracking statistics use live, non-stale PICO samples.",
        "",
        "| run | algorithm | L pos P95 (mm) | R pos P95 (mm) | L rot P95 (rad) | R rot P95 (rad) | posture active L/R | IK accepted L/R | cycle P99 (us) | failures | deadline misses |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for label, data in runs:
        lines.append(
            f"| {label} | {data['algorithm']}"
            f" | {1000.0 * float(data['left_position_error_m_p95']):.2f}"
            f" | {1000.0 * float(data['right_position_error_m_p95']):.2f}"
            f" | {float(data['left_orientation_error_rad_p95']):.4f}"
            f" | {float(data['right_orientation_error_rad_p95']):.4f}"
            f" | {float(data['left_spark_posture_active_mean']):.3f}/"
            f"{float(data['right_spark_posture_active_mean']):.3f}"
            f" | {float(data['left_spark_ik_accepted_mean']):.3f}/"
            f"{float(data['right_spark_ik_accepted_mean']):.3f}"
            f" | {float(data['cycle_p99_us']):.2f}"
            f" | {int(data['control_failures'])} | {int(data['deadline_misses'])} |"
        )
    lines.extend(
        [
            "",
            "## Joint-reference hard-bound audit",
            "",
            "| run | arm | q violations | qdot violations | qddot violations | jerk violations | max |qdot| | max |qddot| | max |jerk| |",
            "|---|---|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for label, joints in joint_audits:
        for side in ("left", "right"):
            lines.append(
                f"| {label} | {side} | {joints[f'{side}_q_violations']}"
                f" | {joints[f'{side}_qdot_violations']}"
                f" | {joints[f'{side}_qddot_violations']}"
                f" | {joints[f'{side}_jerk_violations']}"
                f" | {float(joints[f'{side}_qdot_max_abs']):.6f}"
                f" | {float(joints[f'{side}_qddot_max_abs']):.6f}"
                f" | {float(joints[f'{side}_jerk_max_abs']):.6f} |"
            )
        lines.append(f"\n{label} joint telemetry resets: {joints['resets']}.\n")
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="append", nargs=2, metavar=("LABEL", "CSV"), required=True)
    parser.add_argument(
        "--joints", action="append", nargs=2, metavar=("LABEL", "CSV"), required=True
    )
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    arguments = parse_args()
    runs = [(label, summarize_run(Path(path))) for label, path in arguments.run]
    joint_audits = [
        (label, summarize_joints(Path(path))) for label, path in arguments.joints
    ]
    text = markdown(runs, joint_audits)
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
