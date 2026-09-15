import csv
import importlib.util
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "compare_spark_guided_velocity_qp.py"
SPEC = importlib.util.spec_from_file_location("spark_compare", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


class SparkComparisonTest(unittest.TestCase):
    def test_markdown_labels_each_joint_audit(self) -> None:
        audit: dict[str, object] = {"resets": 0}
        for side in ("left", "right"):
            for metric in ("q", "qdot", "qddot", "jerk"):
                audit[f"{side}_{metric}_violations"] = 0
            for metric in ("qdot", "qddot", "jerk"):
                audit[f"{side}_{metric}_max_abs"] = 1.0
        text = MODULE.markdown([], [("direct", audit), ("pose", audit)])
        self.assertIn("| direct | left |", text)
        self.assertIn("| pose | right |", text)

    def test_run_summary_uses_only_live_non_stale_rows(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "run.csv"
            base = {
                "pico_live": 1,
                "left_target_stale": 0,
                "right_target_stale": 0,
                "left_position_error_m": 0.01,
                "right_position_error_m": 0.02,
                "left_orientation_error_rad": 0.03,
                "right_orientation_error_rad": 0.04,
                "cycle_time_us": 500,
                "control_failures": 0,
                "deadline_misses": 0,
                "algorithm": "spark_direct_velocity_qp",
                "left_spark_posture_active": 1,
                "right_spark_posture_active": 1,
                "left_spark_ik_accepted": 1,
                "right_spark_ik_accepted": 1,
            }
            stale = dict(base)
            stale.update(left_target_stale=1, left_position_error_m=10.0)
            write_csv(path, [base, stale])
            result = MODULE.summarize_run(path)
            self.assertEqual(result["live_rows"], 1)
            self.assertEqual(result["left_position_error_m_p95"], 0.01)
            self.assertEqual(result["algorithm"], "spark_direct_velocity_qp")
            self.assertEqual(result["left_spark_posture_active_mean"], 1.0)
            self.assertEqual(result["right_spark_ik_accepted_mean"], 1.0)

    def test_joint_summary_detects_reference_bound_violation(self) -> None:
        row: dict[str, object] = {"reset": 0}
        for side in ("left", "right"):
            for joint in range(1, 8):
                prefix = f"{side}_j{joint}_"
                row.update(
                    {
                        prefix + "reference_q": 0.0,
                        prefix + "position_lower": -1.0,
                        prefix + "position_upper": 1.0,
                        prefix + "reference_qdot": 0.0,
                        prefix + "velocity_lower": -2.0,
                        prefix + "velocity_upper": 2.0,
                        prefix + "reference_qddot": 0.0,
                        prefix + "acceleration_lower": -3.0,
                        prefix + "acceleration_upper": 3.0,
                        prefix + "reference_jerk": 0.0,
                        prefix + "jerk_lower": -4.0,
                        prefix + "jerk_upper": 4.0,
                    }
                )
        row["left_j3_reference_jerk"] = 5.0
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "joints.csv"
            write_csv(path, [row])
            result = MODULE.summarize_joints(path)
            self.assertEqual(result["left_jerk_violations"], 1)
            self.assertEqual(result["right_jerk_violations"], 0)


if __name__ == "__main__":
    unittest.main()
