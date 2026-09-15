#!/usr/bin/env python3
import argparse
import csv
import subprocess
import tempfile
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory() as tmp:
        outputs = []
        for index in range(2):
            output = Path(tmp) / f"run{index}.csv"
            subprocess.run([
                args.binary,
                "--config", str(root / "config/qp_ik_pico_teleop.yaml"),
                "--model", str(root / "models/marvin_m6_qp_test.xml"),
                "--urdf", str(root / "models/marvin_m6_s_ccs_696_v4_local.urdf"),
                "--algorithm", "hierarchical_qp", "--arm", "left",
                "--working-point", "center", "--channel", "x",
                "--warmup", "0.05", "--chirp", "0.20", "--settle", "0.05",
                "--output", str(output),
            ], check=True)
            with output.open() as stream:
                rows = list(csv.DictReader(line for line in stream if not line.startswith("#")))
            assert len(rows) == 60, len(rows)
            required = {"sample", "time_s", "input", "output", "accepted",
                        "q1", "dq1", "ddq1", "jerk1", "solve_time_us"}
            assert required.issubset(rows[0]), rows[0].keys()
            assert all(float(row["accepted"]) == 1.0 for row in rows)
            outputs.append([(row["time_s"], row["input"]) for row in rows])
        assert outputs[0] == outputs[1]


if __name__ == "__main__":
    main()
