#!/usr/bin/env python3
"""Summarize three-way online runs; source-time alignment and exact-pose subsets."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import struct
import numpy as np
from run_pico_trace_algorithm_benchmark import _read_trace

NAMES = ("spark", "ceres", "dls")
SIDES = ("left", "right")
def quantile(values):
    return float(np.quantile(values, .9, method="inverted_cdf")) if len(values) else None
def key(row):
    return int(row["pico_tracking_epoch"]), int(row["pico_sequence"])
def load(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))
def summarize(folder, trace):
    _, records = _read_trace(trace)
    times = {(struct.unpack_from("<Q", b, 16)[0], struct.unpack_from("<Q", b, 8)[0]): t*1e-9
             for t, b, _ in records}
    assert len(times) == len(records), "duplicate source identities"
    result = {"trace": str(trace), "trace_sha256": hashlib.sha256(trace.read_bytes()).hexdigest(),
              "window_s": [5, 47], "runs": {}}
    last = {}
    for name in NAMES:
        rows = load(folder/f"{name}.csv")
        window = [r for r in rows if 5 <= times.get(key(r), -1) <= 47]
        selected = [r for r in window if r["pico_live"] == "1" and
                    all(r[s+"_target_stale"] == "0" for s in SIDES)]
        assert selected, name
        last[name] = {key(r): r for r in selected}
        data = {
            "cycles": len(selected), "unique_sources": len(last[name]),
            "window_all_cycles": len(window), "excluded_cycles": len(window)-len(selected),
            "rejected_cycles": sum(r["accepted"] != "1" for r in selected),
            "received_frames": max(int(r["pico_datagrams"]) for r in rows),
            "input_accepted": max(int(r["pico_accepted"]) for r in rows),
            "superseded": max(int(r["pico_superseded"]) for r in rows),
            "cycle_us_p90": quantile([float(r["cycle_time_us"]) for r in selected]),
            "cycles_compute_over_5ms": sum(float(r["cycle_time_us"]) > 5000 for r in selected),
            "sides": {}}
        for side in SIDES:
            ik = side+("_spark_palm_position_error_m" if name == "spark" else "_dls_posture_final_position_error_m")
            data["sides"][side] = {
                "command_error_p90_mm": 1000*quantile([float(r[side+"_position_error_m"]) for r in selected]),
                "orientation_p90_rad": quantile([float(r[side+"_orientation_error_rad"]) for r in selected]),
                "ik_p90_mm": 1000*quantile([float(r[ik]) for r in selected]),
                "hold_fraction": sum(r[side+"_spark_settled_hold_active"] == "1" for r in selected)/len(selected),
                "pinocchio_fraction": sum(r[side+"_ee_pinocchio_kinematics"] == "1" for r in selected)/len(selected),
                "ruckig_fraction": sum(r[side+"_ee_ruckig_invoked"] == "1" for r in selected)/len(selected),
            }
        all_joint = load(folder/f"{name}_joints.csv")
        # Telemetry streams have separate publication rates. Select the same
        # control-time interval, not a row-index join.
        lo = min(float(r["control_time_seconds"]) for r in selected)
        hi = max(float(r["control_time_seconds"]) for r in selected)
        joint = [r for r in all_joint if lo <= float(r["control_time_seconds"]) <= hi]
        data["joint_window_samples"] = len(joint)
        data["joint_derivative_peak"] = {}
        for kind in ("qdot", "qddot", "jerk"):
            valid_key = {"qddot": "acceleration", "jerk": "jerk"}.get(kind)
            eligible = [r for r in joint if valid_key is None or
                        all(r[s+"_reference_"+valid_key+"_valid"] == "1" for s in SIDES)]
            values = [max(abs(float(r[f"{s}_j{j}_reference_{kind}"])) for s in SIDES for j in range(1, 8))
                      for r in eligible]
            limits = [4]*7 if kind == "qdot" else ([60]*3+[90]*4 if kind == "qddot" else [3000]*3+[4500]*4)
            data["joint_derivative_peak"][kind] = {
                "valid_samples": len(eligible), "p90": quantile(values), "max": max(values),
                "bound_violation_values": sum(abs(float(r[f"{s}_j{j}_reference_{kind}"])) > limits[j-1]+1e-5
                                             for r in eligible for s in SIDES for j in range(1, 8))}
        data["position_bound_violations"] = sum(
            not float(r[f"{s}_j{j}_position_lower"])-1e-7 <= float(r[f"{s}_j{j}_reference_q"]) <= float(r[f"{s}_j{j}_position_upper"])+1e-7
            for r in all_joint for s in SIDES for j in range(1, 8))
        data["final_max_abs_reference_velocity"] = max(abs(float(all_joint[-1][f"{s}_j{j}_reference_qdot"]))
                                                      for s in SIDES for j in range(1, 8))
        result["runs"][name] = data
    common = sorted(set.intersection(*(set(v) for v in last.values())))
    result["common_unique_sources"] = len(common)
    result["matched_command_subset"] = {}
    for side in SIDES:
        ids = []
        for k in common:
            ref = last["spark"][k]
            same = ref[side+"_spark_settled_hold_active"] == "0"
            for name in ("ceres", "dls"):
                row = last[name][k]
                dp = np.linalg.norm([float(ref[side+"_target_p"+j])-float(row[side+"_target_p"+j]) for j in "xyz"])
                qa = np.array([float(ref[side+"_target_q"+j]) for j in "xyzw"])
                qb = np.array([float(row[side+"_target_q"+j]) for j in "xyzw"])
                same &= dp < 1e-6 and min(np.linalg.norm(qa-qb), np.linalg.norm(qa+qb)) < 1e-6
            if same:
                ids.append(k)
        result["matched_command_subset"][side] = {
            "samples": len(ids), "excluded": len(common)-len(ids),
            "error_p90_mm": {name: 1000*quantile([float(last[name][k][side+"_position_error_m"]) for k in ids])
                             if ids else None for name in NAMES}}
    return result

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    manifest = json.loads((args.directory/"manifest.json").read_text())
    assert len(manifest["runs"]) and all(r["complete"] for r in manifest["runs"])
    output = {"scope": manifest["scope"], "datasets": []}
    for index in sorted({r["dataset"] for r in manifest["runs"]}):
        runs = [r for r in manifest["runs"] if r["dataset"] == index]
        assert {r["algorithm"] for r in runs} == set(NAMES)
        trace = Path(runs[0]["trace"])
        assert hashlib.sha256(trace.read_bytes()).hexdigest() == runs[0]["trace_sha256"]
        output["datasets"].append(summarize(args.directory/f"dataset{index}", trace))
    print(json.dumps(output, indent=2))

if __name__ == "__main__":
    main()
