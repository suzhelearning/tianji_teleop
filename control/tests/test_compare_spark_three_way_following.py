import importlib.util
import csv
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "compare_spark_three_way_following.py"
)


def load_analyzer():
    spec = importlib.util.spec_from_file_location("spark_three_way", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def pose_row(sequence, target_x, actual_x):
    row = {"pico_sequence": str(sequence), "pico_live": "1"}
    for side in ("left", "right"):
        row.update(
            {
                f"{side}_target_px": str(target_x),
                f"{side}_target_py": "0",
                f"{side}_target_pz": "0",
                f"{side}_target_qx": "0",
                f"{side}_target_qy": "0",
                f"{side}_target_qz": "0",
                f"{side}_target_qw": "1",
                f"{side}_actual_px": str(actual_x),
                f"{side}_actual_py": "0",
                f"{side}_actual_pz": "0",
                f"{side}_actual_qx": "0",
                f"{side}_actual_qy": "0",
                f"{side}_actual_qz": "0",
                f"{side}_actual_qw": "1",
            }
        )
    return row


class ThreeWayFollowingMetricsTest(unittest.TestCase):
    def test_best_lag_recovers_known_two_frame_delay(self):
        analyzer = load_analyzer()
        target_rows = {
            sequence: pose_row(sequence, math.sin(0.31 * sequence), 0.0)
            for sequence in range(1, 80)
        }
        actual_rows = {
            sequence: pose_row(
                sequence,
                math.sin(0.31 * sequence),
                math.sin(0.31 * (sequence - 2)),
            )
            for sequence in range(1, 80)
        }
        valid_sequences = list(range(3, 80))

        result = analyzer.best_lag(
            actual_rows, target_rows, valid_sequences, max_lag_frames=8
        )

        self.assertEqual(result.position_frames, 2)
        self.assertLess(result.position_rmse, 1.0e-12)

    def test_quaternion_distance_uses_shortest_rotation(self):
        analyzer = load_analyzer()
        identity = np.array([0.0, 0.0, 0.0, 1.0])
        negative_identity = -identity
        self.assertAlmostEqual(
            analyzer.quaternion_distance(identity, negative_identity), 0.0
        )

    def test_analysis_writes_summary_and_figures(self):
        analyzer = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            paths = {}
            for name, lag in (("velocity_qp", 1), ("cartesian_otg", 3),
                              ("joint_ruckig", 2),
                              ("feedforward_velocity_qp", 0),
                              ("headroom_feedforward_velocity_qp", 1)):
                path = root / f"{name}.csv"
                paths[name] = path
                rows = []
                for sequence in range(1, 45):
                    row = pose_row(
                        sequence,
                        math.sin(0.2 * sequence),
                        math.sin(0.2 * (sequence - lag)),
                    )
                    row.update(
                        {
                            "control_time_seconds": str(sequence * 0.005),
                            "pico_left_source_timestamp_ns": str(
                                sequence * 10_000_000
                            ),
                            "cycle_time_us": "500",
                            "control_failures": "0",
                            "deadline_misses": "0",
                            "accepted": "1",
                        }
                    )
                    rows.append(row)
                with path.open("w", newline="") as output:
                    writer = csv.DictWriter(output, fieldnames=list(rows[0]))
                    writer.writeheader()
                    writer.writerows(rows)

            output_directory = root / "report"
            summaries = analyzer.run_analysis(
                paths,
                output_directory,
                excluded_ranges=(),
                max_lag_frames=6,
                target_agreement_tolerance_m=1.0e-6,
            )

            self.assertEqual(summaries["velocity_qp"].position_lag_frames, 1)
            self.assertEqual(summaries["cartesian_otg"].position_lag_frames, 3)
            self.assertEqual(summaries["joint_ruckig"].position_lag_frames, 2)
            self.assertEqual(
                summaries["feedforward_velocity_qp"].position_lag_frames, 0
            )
            self.assertEqual(
                summaries["headroom_feedforward_velocity_qp"].position_lag_frames,
                1,
            )
            self.assertTrue((output_directory / "summary.csv").is_file())
            self.assertTrue((output_directory / "README.md").is_file())
            self.assertTrue((output_directory / "lag_scan.png").is_file())
            self.assertTrue((output_directory / "trajectory_3d.png").is_file())

    def test_analysis_keeps_three_way_input_compatible(self):
        analyzer = load_analyzer()
        self.assertEqual(
            analyzer.required_algorithm_names(
                {
                    "velocity_qp": Path("velocity.csv"),
                    "cartesian_otg": Path("otg.csv"),
                    "joint_ruckig": Path("ruckig.csv"),
                }
            ),
            {"velocity_qp", "cartesian_otg", "joint_ruckig"},
        )


if __name__ == "__main__":
    unittest.main()
