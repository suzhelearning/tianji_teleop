#!/usr/bin/env python3
"""Plot q/qdot/qddot/jerk for four SPARK teleoperation pipelines."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


QUANTITIES = (
    ("q", "reference_q", "position_lower", "position_upper", "rad"),
    ("qdot", "reference_qdot", "velocity_lower", "velocity_upper", "rad/s"),
    (
        "qddot",
        "reference_qddot",
        "acceleration_lower",
        "acceleration_upper",
        "rad/s²",
    ),
    ("jerk", "reference_jerk", "jerk_lower", "jerk_upper", "rad/s³"),
)
COLORS = ("#4c78a8", "#f58518", "#54a24b", "#e45756", "#b279a2")
LINE_STYLES = ("-", "--", "-.", ":", (0, (5, 2)))


def load(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as source:
        rows = list(csv.DictReader(source))
    if not rows:
        raise ValueError(f"empty joint telemetry: {path}")
    return rows


def values(rows: list[dict[str, str]], key: str) -> np.ndarray:
    return np.asarray([float(row[key]) for row in rows], dtype=float)


def plot_arm(
    side: str,
    runs: list[tuple[str, list[dict[str, str]]]],
    bound_rows: list[dict[str, str]],
    output: Path,
) -> None:
    figure, axes = plt.subplots(4, 7, figsize=(28, 14), sharex=True)
    for joint in range(1, 8):
        prefix = f"{side}_j{joint}_"
        for row_index, (name, value_name, lower_name, upper_name, unit) in enumerate(
            QUANTITIES
        ):
            axis = axes[row_index, joint - 1]
            for run_index, (label, rows) in enumerate(runs):
                time = values(rows, "control_time_seconds")
                signal = values(rows, prefix + value_name)
                axis.plot(
                    time,
                    signal,
                    color=COLORS[run_index],
                    linestyle=LINE_STYLES[run_index],
                    linewidth=0.75,
                    alpha=0.78,
                    label=label,
                )
            bound_time = values(bound_rows, "control_time_seconds")
            axis.plot(
                bound_time,
                values(bound_rows, prefix + lower_name),
                color="#d62728",
                linestyle="--",
                linewidth=0.55,
                alpha=0.55,
            )
            axis.plot(
                bound_time,
                values(bound_rows, prefix + upper_name),
                color="#d62728",
                linestyle="--",
                linewidth=0.55,
                alpha=0.55,
            )
            axis.grid(True, alpha=0.22)
            if row_index == 0:
                axis.set_title(f"Joint {joint}")
            if joint == 1:
                axis.set_ylabel(f"{name} [{unit}]")
            if row_index == 3:
                axis.set_xlabel("time [s]")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    figure.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.972),
        ncol=len(runs),
    )
    figure.suptitle(
        f"{side.capitalize()} arm command continuity",
        y=0.995,
    )
    figure.text(
        0.5,
        0.947,
        "q/qdot/qddot/jerk at 200 Hz; thin dashed red = feedforward effective bounds",
        ha="center",
        fontsize=9,
        color="#444444",
    )
    figure.tight_layout(rect=(0, 0, 1, 0.925))
    figure.savefig(output, dpi=150)
    plt.close(figure)


def summarize(
    runs: list[tuple[str, list[dict[str, str]]]], output: Path
) -> None:
    fields = [
        "algorithm",
        "arm",
        "quantity",
        "p50_abs",
        "p95_abs",
        "p99_abs",
        "max_abs",
        "max_adjacent_delta",
        "bound_violations",
        "resets",
    ]
    with output.open("w", newline="") as target:
        writer = csv.DictWriter(target, fieldnames=fields)
        writer.writeheader()
        for label, rows in runs:
            resets = sum(row.get("reset") == "1" for row in rows)
            for side in ("left", "right"):
                for name, value_name, lower_name, upper_name, _ in QUANTITIES:
                    samples: list[float] = []
                    deltas: list[float] = []
                    violations = 0
                    for joint in range(1, 8):
                        prefix = f"{side}_j{joint}_"
                        signal = values(rows, prefix + value_name)
                        lower = values(rows, prefix + lower_name)
                        upper = values(rows, prefix + upper_name)
                        reset = np.asarray(
                            [row.get("reset") == "1" for row in rows],
                            dtype=bool,
                        )
                        samples.extend(np.abs(signal[~reset]).tolist())
                        valid_delta = ~(reset[1:] | reset[:-1])
                        deltas.extend(
                            np.abs(np.diff(signal)[valid_delta]).tolist()
                        )
                        violations += int(
                            np.count_nonzero(
                                (~reset)
                                & ((signal < lower - 1.0e-6)
                                   | (signal > upper + 1.0e-6))
                            )
                        )
                    array = np.asarray(samples)
                    writer.writerow(
                        {
                            "algorithm": label,
                            "arm": side,
                            "quantity": name,
                            "p50_abs": np.percentile(array, 50),
                            "p95_abs": np.percentile(array, 95),
                            "p99_abs": np.percentile(array, 99),
                            "max_abs": np.max(array),
                            "max_adjacent_delta": max(deltas, default=0.0),
                            "bound_violations": violations,
                            "resets": resets,
                        }
                    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--run", action="append", nargs=2, metavar=("LABEL", "CSV"), required=True
    )
    parser.add_argument("--bounds-label", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    arguments = parser.parse_args()
    runs = [(label, load(Path(path))) for label, path in arguments.run]
    by_label = dict(runs)
    if arguments.bounds_label not in by_label:
        raise ValueError("--bounds-label must name one of the --run labels")
    arguments.output_dir.mkdir(parents=True, exist_ok=True)
    for side in ("left", "right"):
        plot_arm(
            side,
            runs,
            by_label[arguments.bounds_label],
            arguments.output_dir / f"{side}_all_joints_4x7.png",
        )
    summarize(runs, arguments.output_dir / "joint_continuity_summary.csv")
    print(f"output_dir={arguments.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
