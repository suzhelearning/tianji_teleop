#!/usr/bin/env python3
"""Detect sustained wrist-limit/arm-angle branch instability in Viewer CSVs."""

import argparse
import csv
import math
from collections import defaultdict


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cartesian", required=True)
    parser.add_argument("--joints", required=True)
    parser.add_argument("--rate-hz", type=float, default=200.0)
    parser.add_argument("--limit-margin-rad", type=float, default=0.05)
    args = parser.parse_args()

    with open(args.cartesian, newline="") as source:
        cartesian = {int(row["sequence"]): row for row in csv.DictReader(source)}
    with open(args.joints, newline="") as source:
        joint_rows = list(csv.DictReader(source))

    windows = defaultdict(list)
    for joint_row in joint_rows:
        sequence = int(joint_row["sequence"])
        cartesian_row = cartesian.get(sequence)
        if cartesian_row is not None:
            windows[int(sequence / args.rate_hz)].append(
                (joint_row, cartesian_row)
            )

    failures = []
    for second, rows in sorted(windows.items()):
        for side in ("left", "right"):
            near_limit_total = 0
            wrist_speed_total = 0.0
            maximum_arm_error = 0.0
            for joint_row, cartesian_row in rows:
                wrist_speed_total += sum(
                    abs(float(joint_row[f"{side}_j{joint}_reference_qdot"]))
                    for joint in (5, 6, 7)
                )
                maximum_arm_error = max(
                    maximum_arm_error,
                    abs(float(cartesian_row[f"{side}_arm_angle_control_error_rad"])),
                )
                for joint in (5, 6, 7):
                    position = float(
                        joint_row[f"{side}_j{joint}_reference_q"]
                    )
                    lower = float(
                        joint_row[f"{side}_j{joint}_position_lower"]
                    )
                    upper = float(
                        joint_row[f"{side}_j{joint}_position_upper"]
                    )
                    if min(position - lower, upper - position) < args.limit_margin_rad:
                        near_limit_total += 1
            sample_count = len(rows)
            mean_near_limits = near_limit_total / sample_count
            mean_wrist_speed = wrist_speed_total / sample_count
            if (
                mean_near_limits >= 1.5
                and maximum_arm_error >= math.pi - 0.05
                and mean_wrist_speed >= 1.0
            ):
                failures.append(
                    (second, side, mean_near_limits, maximum_arm_error,
                     mean_wrist_speed)
                )

    if failures:
        second, side, near_limits, arm_error, wrist_speed = failures[0]
        print(
            "WRIST_INSTABILITY "
            f"onset_s={second} side={side} "
            f"mean_near_limits={near_limits:.3f} "
            f"max_arm_error_rad={arm_error:.6f} "
            f"mean_wrist_speed_rad_s={wrist_speed:.3f}"
        )
        return 2

    print("WRIST_STABLE no sustained branch/limit instability detected")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
