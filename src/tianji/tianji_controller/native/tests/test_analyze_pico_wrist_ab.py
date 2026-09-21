#!/usr/bin/env python3
import csv
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ANALYZER = ROOT / "scripts" / "analyze_pico_wrist_ab.py"


def write_fixture(path: Path, with_correlated_jump: bool) -> None:
    fields = ["sequence", "control_time_seconds"]
    for side in ("left", "right"):
        fields += [
            f"{side}_j3_reference_q",
            f"{side}_j3_position_lower",
            f"{side}_j3_position_upper",
        ]
        for joint in (5, 6, 7):
            fields += [
                f"{side}_j{joint}_reference_q",
                f"{side}_j{joint}_reference_qdot",
                f"{side}_j{joint}_reference_qddot",
                f"{side}_j{joint}_reference_jerk",
            ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for sequence in range(5):
            row = {"sequence": sequence, "control_time_seconds": 0.05 * sequence}
            for side in ("left", "right"):
                row[f"{side}_j3_reference_q"] = (
                    -0.005 if side == "left" and sequence == 2 else -1.0
                )
                row[f"{side}_j3_position_lower"] = -3.0
                row[f"{side}_j3_position_upper"] = 0.0 if side == "left" else 3.0
                for joint in (5, 6, 7):
                    value = (
                        0.35
                        if with_correlated_jump
                        and side == "left"
                        and joint == 6
                        and sequence >= 2
                        else 0.0
                    )
                    row[f"{side}_j{joint}_reference_q"] = value
                    row[f"{side}_j{joint}_reference_qdot"] = 7.0 if value else 0.0
                    row[f"{side}_j{joint}_reference_qddot"] = 140.0 if value else 0.0
                    row[f"{side}_j{joint}_reference_jerk"] = 2800.0 if value else 0.0
            writer.writerow(row)


class AnalyzePicoWristAbTest(unittest.TestCase):
    def test_identifies_left_wrist_jump_correlated_with_j3_boundary(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            current = root / "current.csv"
            symmetric = root / "symmetric.csv"
            output = root / "report.json"
            write_fixture(current, with_correlated_jump=True)
            write_fixture(symmetric, with_correlated_jump=False)

            completed = subprocess.run(
                [
                    sys.executable,
                    str(ANALYZER),
                    "--current",
                    str(current),
                    "--symmetric-j3",
                    str(symmetric),
                    "--output",
                    str(output),
                ],
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            report = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(
                report["datasets"]["current"]["left"]["wrist_jump_cycles"], 1
            )
            self.assertEqual(
                report["datasets"]["current"]["left"]
                ["wrist_jumps_near_j3_bound"],
                1,
            )
            self.assertEqual(
                report["datasets"]["symmetric_j3"]["left"]["wrist_jump_cycles"],
                0,
            )
            self.assertEqual(report["diagnosis"], "j3_boundary_correlated")


if __name__ == "__main__":
    unittest.main()
