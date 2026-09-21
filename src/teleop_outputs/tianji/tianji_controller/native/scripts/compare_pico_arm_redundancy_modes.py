#!/usr/bin/env python3
"""Compare PICO arm-redundancy modes on sequence-aligned Viewer telemetry."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path


MODES = ("pico", "default_down", "outward_only")
SIDES = ("left", "right")


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    return ordered[min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)]


def load_live_by_sequence(path: Path, expected_mode: str) -> tuple[dict[int, dict[str, str]], dict[str, int]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    modes = {row["arm_angle_mode"] for row in rows}
    if modes != {expected_mode}:
        raise ValueError(f"{path}: expected mode {expected_mode}, got {sorted(modes)}")
    live: dict[int, dict[str, str]] = {}
    for row in rows:
        if row["pico_live"] == "1":
            live[int(row["pico_sequence"])] = row
    counters = {
        "rows": len(rows),
        "live_rows": sum(row["pico_live"] == "1" for row in rows),
        "unique_live_sequences": len(live),
        "control_failures": max((int(row["control_failures"]) for row in rows), default=0),
        "deadline_misses": max((int(row["deadline_misses"]) for row in rows), default=0),
    }
    return live, counters


def summarize_mode(rows: list[dict[str, str]], counters: dict[str, int]) -> dict[str, object]:
    summary: dict[str, object] = dict(counters)
    summary["rejected_aligned_cycles"] = sum(row["accepted"] != "1" for row in rows)
    summary["sides"] = {}
    for side in SIDES:
        position = [float(row[f"{side}_position_error_m"]) for row in rows]
        orientation = [float(row[f"{side}_orientation_error_rad"]) for row in rows]
        distance = [float(row[f"{side}_upper_arm_outward_distance_m"]) for row in rows]
        residual = [float(row[f"{side}_upper_arm_outward_residual"]) for row in rows]
        side_summary = {
            "position_error_mean_m": statistics.fmean(position),
            "position_error_p95_m": percentile(position, 0.95),
            "orientation_error_mean_rad": statistics.fmean(orientation),
            "orientation_error_p95_rad": percentile(orientation, 0.95),
            "outward_distance_min_m": min(distance),
            "outward_distance_p05_m": percentile(distance, 0.05),
            "inward_violation_cycles_below_minus_1mm": sum(value < -0.001 for value in distance),
            "barrier_residual_min": min(residual),
            "barrier_violation_cycles": sum(value < -1.0e-7 for value in residual),
            "feasibility_clip_cycles": sum(
                row[f"{side}_upper_arm_outward_feasibility_clipped"] == "1"
                for row in rows
            ),
        }
        summary["sides"][side] = side_summary
    return summary


def compare(paths: dict[str, Path]) -> dict[str, object]:
    loaded = {mode: load_live_by_sequence(paths[mode], mode) for mode in MODES}
    common_sequences = set.intersection(*(set(data[0]) for data in loaded.values()))
    if not common_sequences:
        raise ValueError("no common live PICO sequence across all modes")
    report: dict[str, object] = {
        "alignment": {
            "key": "pico_sequence",
            "common_live_sequences": len(common_sequences),
            "first": min(common_sequences),
            "last": max(common_sequences),
        },
        "modes": {},
    }
    for mode in MODES:
        live, counters = loaded[mode]
        aligned = [live[sequence] for sequence in sorted(common_sequences)]
        report["modes"][mode] = summarize_mode(aligned, counters)
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pico", type=Path, required=True)
    parser.add_argument("--default-down", type=Path, required=True)
    parser.add_argument("--outward-only", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    arguments = parser.parse_args()
    report = compare(
        {
            "pico": arguments.pico,
            "default_down": arguments.default_down,
            "outward_only": arguments.outward_only,
        }
    )
    serialized = json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if arguments.output is not None:
        arguments.output.write_text(serialized, encoding="utf-8")
    print(serialized, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
