#!/usr/bin/env python3
import math
import csv
import json
from pathlib import Path
import sys
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from analyze_pico_trace_algorithm_benchmark import (  # noqa: E402
    analyze_benchmark,
    audit_joint_rows,
    band_energy_ratio,
    compute_tracking_summary,
    select_common_sequences,
)


def cart_row(sequence: int, target_x: float, actual_x: float,
             epoch: int = 1, timestamp_ns: int | None = None) -> dict[str, str]:
    row = {
        "pico_sequence": str(sequence), "pico_live": "1", "accepted": "1",
        "pico_tracking_epoch": str(epoch), "pico_resynchronizations": "0",
        "pico_reset_applies": "0", "pico_jump_rejections": "0",
        "pico_left_source_timestamp_ns": str(
            timestamp_ns if timestamp_ns is not None else sequence * 10_000_000
        ),
        "cycle_time_us": "100", "deadline_misses": "0",
        "control_failures": "0",
    }
    for side in ("left", "right"):
        for prefix, x in (("target", target_x), ("actual", actual_x)):
            row[f"{side}_{prefix}_px"] = str(x)
            row[f"{side}_{prefix}_py"] = "0"
            row[f"{side}_{prefix}_pz"] = "0"
            row[f"{side}_{prefix}_qx"] = "0"
            row[f"{side}_{prefix}_qy"] = "0"
            row[f"{side}_{prefix}_qz"] = "0"
            row[f"{side}_{prefix}_qw"] = "1"
    return row


def joint_row(qdot: float = 0.0, qddot: float = 0.0, jerk: float = 0.0,
              reset: int = 0) -> dict[str, str]:
    row = {"control_time_seconds": "0", "reset": str(reset)}
    for side in ("left", "right"):
        for joint in range(1, 8):
            prefix = f"{side}_j{joint}_"
            values = {
                "reference_q": 0.0, "reference_qdot": qdot,
                "reference_qddot": qddot, "reference_jerk": jerk,
                "position_lower": -1.0, "position_upper": 1.0,
                "velocity_lower": -2.0, "velocity_upper": 2.0,
                "acceleration_lower": -4.0, "acceleration_upper": 4.0,
                "jerk_lower": -10.0, "jerk_upper": 10.0,
            }
            row.update({prefix + key: str(value) for key, value in values.items()})
    return row


class AnalyzerUnitTests(unittest.TestCase):
    def test_common_clean_sequences_exclude_startup_and_epoch_recovery(self):
        canonical = {}
        comparison = {}
        for sequence in range(1, 101):
            epoch = 1 if sequence < 50 else 2
            canonical[sequence] = cart_row(sequence, sequence * 0.001, 0.0, epoch)
            comparison[sequence] = cart_row(sequence, sequence * 0.001, 0.0, epoch)
        all_valid, clean = select_common_sequences(
            {"canonical": canonical, "comparison": comparison},
            canonical="canonical", recovery_ms=100.0,
            target_agreement_tolerance_m=1.0e-9,
        )

        self.assertEqual(all_valid, list(range(1, 101)))
        self.assertNotIn(1, clean)
        self.assertNotIn(50, clean)
        self.assertIn(12, clean)
        self.assertIn(61, clean)

    def test_target_disagreement_is_removed_from_clean_common(self):
        canonical = {i: cart_row(i, i * 0.01, 0.0) for i in range(1, 50)}
        comparison = {i: cart_row(i, i * 0.01, 0.0) for i in range(1, 50)}
        comparison[30] = cart_row(30, 99.0, 0.0)
        _, clean = select_common_sequences(
            {"canonical": canonical, "comparison": comparison},
            canonical="canonical", recovery_ms=0.0,
            target_agreement_tolerance_m=1.0e-4,
        )
        self.assertNotIn(30, clean)

    def test_tracking_summary_recovers_known_causal_lag(self):
        target = {}
        actual = {}
        for sequence in range(1, 101):
            value = math.sin(sequence * 0.17)
            target[sequence] = cart_row(sequence, value, value)
            delayed = math.sin((sequence - 2) * 0.17)
            actual[sequence] = cart_row(sequence, value, delayed)
        summary = compute_tracking_summary(
            actual, target, list(range(5, 101)), max_lag_ms=80.0
        )

        self.assertAlmostEqual(summary["position_lag_ms"], 20.0, delta=0.1)
        self.assertLess(summary["position_compensated_rmse_m"], 1.0e-9)
        self.assertGreater(summary["position_p95_m"], 0.1)

    def test_band_energy_detects_three_hertz_motion(self):
        time = np.arange(0.0, 4.0, 0.005)
        three_hz = np.sin(2.0 * np.pi * 3.0 * time)
        eight_hz = np.sin(2.0 * np.pi * 8.0 * time)

        self.assertGreater(band_energy_ratio(three_hz, time, 2.5, 5.0), 0.9)
        self.assertLess(band_energy_ratio(eight_hz, time, 2.5, 5.0), 0.05)

    def test_joint_audit_counts_active_bounds_and_violations(self):
        rows = [joint_row(qdot=1.0, qddot=2.0, jerk=3.0) for _ in range(20)]
        rows[5] = joint_row(qdot=2.0, qddot=4.0, jerk=10.0)
        rows[10] = joint_row(qdot=2.01, qddot=4.01, jerk=10.01)
        audit = audit_joint_rows(rows, tolerance=1.0e-6)

        left_j1 = audit[("left", 1)]
        self.assertEqual(left_j1["velocity_active_count"], 2)
        self.assertEqual(left_j1["velocity_violation_count"], 1)
        self.assertEqual(left_j1["acceleration_violation_count"], 1)
        self.assertEqual(left_j1["jerk_violation_count"], 1)
        self.assertAlmostEqual(left_j1["jerk_p95_rad_s3"], 10.0005)

    def test_report_generation_writes_requested_artifacts(self):
        import tempfile

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            telemetry_dir = root / "telemetry"
            telemetry_dir.mkdir()
            algorithms = [
                "spark_upper_qpoases_velocity_qp",
                "spark_upper_qpoases_headroom_feedforward_velocity_qp",
            ]
            runs = {}
            for algorithm_index, algorithm in enumerate(algorithms):
                cartesian = telemetry_dir / f"{algorithm}.csv"
                joint = telemetry_dir / f"{algorithm}_joints.csv"
                cart_rows = []
                for sequence in range(1, 81):
                    target = math.sin(sequence * 0.12)
                    row = cart_row(
                        sequence, target,
                        math.sin((sequence - algorithm_index - 1) * 0.12),
                    )
                    row["algorithm"] = algorithm
                    if algorithm_index == 1:
                        row["control_failures"] = "1"
                    row["left_solve_time_us"] = "30"
                    row["right_solve_time_us"] = "35"
                    cart_rows.append(row)
                with cartesian.open("w", newline="") as stream:
                    writer = csv.DictWriter(stream, fieldnames=cart_rows[0].keys())
                    writer.writeheader()
                    writer.writerows(cart_rows)
                joint_rows = []
                for index in range(100):
                    row = joint_row(
                        qdot=0.2 * math.sin(index * 0.1),
                        qddot=0.4 * math.cos(index * 0.1),
                        jerk=0.8 * math.sin(index * 0.1),
                    )
                    row["control_time_seconds"] = str(index * 0.005)
                    joint_rows.append(row)
                with joint.open("w", newline="") as stream:
                    writer = csv.DictWriter(stream, fieldnames=joint_rows[0].keys())
                    writer.writeheader()
                    writer.writerows(joint_rows)
                runs[algorithm] = {
                    "status": ("complete" if algorithm_index == 0
                               else "complete_with_failures"),
                    "telemetry": str(cartesian.relative_to(root)),
                    "joint_telemetry": str(joint.relative_to(root)),
                }
            manifest = root / "manifest.json"
            manifest.write_text(json.dumps({
                "schema_version": 1,
                "provenance": {"source_trace": {"sha256": "abc"}},
                "algorithms": algorithms,
                "runs": runs,
            }))

            report = root / "report"
            result = analyze_benchmark(manifest, report, segment_seconds=10.0)
            self.assertEqual(result["algorithms"], algorithms)
            self.assertEqual(result["ranking"], [algorithms[0]])
            self.assertEqual(result["excluded_main"], [algorithms[1]])

            expected = [
                report / "summary_main.csv",
                report / "summary_all.csv",
                report / "joint_summary.csv",
                report / "constraint_audit.csv",
                report / "README.md",
                report / "main/tracking_timeseries.png",
                report / "main/trajectory_3d.png",
                report / "main/error_cdf.png",
                report / "main/lag_scan.png",
                report / "main/summary.png",
                report / "main/joints_left_4x7.png",
                report / "main/joints_right_4x7.png",
                report / "main/tracking_segments/segment_000_000-010s.png",
            ]
            for path in expected:
                self.assertTrue(path.is_file(), path)
                self.assertGreater(path.stat().st_size, 100, path)


if __name__ == "__main__":
    unittest.main()
