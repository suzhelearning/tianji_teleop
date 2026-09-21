#!/usr/bin/env python3

import csv
import importlib.util
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "compare_modes", ROOT / "scripts" / "compare_pico_arm_redundancy_modes.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def write_mode(root: Path, mode: str, distance: float) -> Path:
    path = root / f"{mode}.csv"
    fields = [
        "arm_angle_mode", "pico_live", "pico_sequence", "accepted",
        "control_failures", "deadline_misses",
    ]
    for side in ("left", "right"):
        fields += [
            f"{side}_position_error_m", f"{side}_orientation_error_rad",
            f"{side}_upper_arm_outward_distance_m",
            f"{side}_upper_arm_outward_residual",
            f"{side}_upper_arm_outward_feasibility_clipped",
        ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for sequence in (1, 2, 3):
            row = {
                "arm_angle_mode": mode, "pico_live": "1",
                "pico_sequence": sequence, "accepted": "1",
                "control_failures": "0", "deadline_misses": "0",
            }
            for side in ("left", "right"):
                row.update({
                    f"{side}_position_error_m": 0.01 * sequence,
                    f"{side}_orientation_error_rad": 0.02 * sequence,
                    f"{side}_upper_arm_outward_distance_m": distance,
                    f"{side}_upper_arm_outward_residual": 0.0,
                    f"{side}_upper_arm_outward_feasibility_clipped": "0",
                })
            writer.writerow(row)
    return path


with tempfile.TemporaryDirectory() as directory:
    root = Path(directory)
    paths = {
        "pico": write_mode(root, "pico", -0.02),
        "default_down": write_mode(root, "default_down", -0.01),
        "outward_only": write_mode(root, "outward_only", 0.001),
    }
    report = MODULE.compare(paths)
    assert report["alignment"]["common_live_sequences"] == 3
    outward = report["modes"]["outward_only"]["sides"]["left"]
    assert outward["outward_distance_min_m"] == 0.001
    assert outward["inward_violation_cycles_below_minus_1mm"] == 0
    pico = report["modes"]["pico"]["sides"]["right"]
    assert pico["inward_violation_cycles_below_minus_1mm"] == 3

print("compare_pico_arm_redundancy_modes: PASS")
