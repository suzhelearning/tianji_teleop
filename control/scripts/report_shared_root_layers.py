"""Offline action/state diagnostics and paired task ablation; never an enable gate."""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import subprocess

from report_shared_root_actions import ACTIONS, CONTROL, sha256, validate_annotations

METRICS = ("ik_m", "ik_rad", "reference_command_m", "reference_ik_target_m",
           "ik_command_m", "ff_command_m", "reference_ff_m", "headroom_scale")
FLAGS = ("settled_hold", "blend", "guidance_accepted", "target_valid", "ik_valid",
         "reference_attempted", "reference_accepted", "ff_valid", "headroom_valid")
VARIANTS = ("baseline", "shape_soft", "pose_only", "position_only", "iterations_only",
            "precision_only", "precision_iterations")
STATES = ("uninitialized", "tracking", "hold_last_mapped", "recovering", "invalid")


def fields(line):
    result = {}
    for token in line.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        if key in result:
            raise ValueError("duplicate diagnostic field")
        result[key] = value
    return result


def number(value):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("nonfinite diagnostic")
    return result


def flag(value):
    if value not in ("0", "1"):
        raise ValueError("invalid boolean diagnostic")
    return value == "1"


def stats(values):
    values = sorted(values)
    return dict(count=len(values), **{
        name: values[math.ceil(p*len(values))-1] if values else None
        for name, p in (("p50", .5), ("p90", .9), ("p95", .95), ("max", 1))})


def action_at(t, segments, duration):
    if t > duration:
        return "post_trace_tail"
    return next((s["action"] for s in segments if s["start_ns"] <= t < s["end_ns"]
                 or t == s["end_ns"] == duration), "unannotated")


def parse_native(text):
    contract = provenance = summary = None
    projections, samples, trials = [], [], []
    for line in text.splitlines():
        if line.startswith("layer_contract "):
            if contract is not None:
                raise ValueError("duplicate layer contract")
            contract = fields(line)
        elif line.startswith("trace_sha256="):
            if provenance is not None:
                raise ValueError("duplicate provenance")
            provenance = fields(line)
        elif line.startswith("mode=1 "):
            if summary is not None:
                raise ValueError("duplicate cycle summary")
            summary = fields(line)
        elif line.startswith("projection_frame "):
            row = fields(line)
            row["t_ns"], row["sequence"] = int(row["t_ns"]), int(row["sequence"])
            row["valid"] = flag(row["valid"])
            row["shift_m"] = None if row["shift_m"] == "NA" else number(row["shift_m"])
            if ((row["shift_m"] is not None) != row["valid"] or row["t_ns"] < 0
                    or row["sequence"] <= 0 or (row["valid"] and row["shift_m"] < 0)
                    or (projections and row["t_ns"] < projections[-1]["t_ns"])):
                raise ValueError("invalid projection frame")
            projections.append(row)
        elif line.startswith("layer_sample "):
            row = fields(line)
            for key in ("mode", "cycle", "t_ns", "source_t_ns", "sequence"):
                row[key] = int(row[key])
            for key in FLAGS:
                row[key] = flag(row[key])
            for key in METRICS:
                row[key] = None if row[key] == "NA" else number(row[key])
            if (row["mode"] != 1 or row["sequence"] <= 0 or row["state"] not in STATES
                    or row["cycle"] != len(samples)//2
                    or row["side"] != ("left" if len(samples)%2 == 0 else "right")
                    or row["t_ns"] != row["cycle"]*5000000
                    or not 0 <= row["source_t_ns"] <= row["t_ns"]):
                raise ValueError("invalid cycle identity/timeline")
            reference_valid = row["reference_accepted"] and row["target_valid"]
            masks = [row["ik_valid"]]*2 + [reference_valid]*3 + [row["ff_valid"]]*2 + [row["headroom_valid"]]
            if (any((row[key] is not None) != valid for key, valid in zip(METRICS, masks))
                    or any(row[key] is not None and row[key] < 0 for key in METRICS)
                    or (row["ik_valid"] and not row["target_valid"])
                    or (row["reference_accepted"] and not row["reference_attempted"])
                    or (row["reference_attempted"] and not row["guidance_accepted"])
                    or (row["ff_valid"] and not reference_valid)
                    or (row["headroom_valid"] and not row["ff_valid"])):
                raise ValueError("inconsistent metric validity")
            samples.append(row)
        elif line.startswith("task_ablation "):
            row = fields(line)
            for key in ("mode", "cycle", "t_ns", "sequence", "stage1_iterations", "stage2_iterations",
                        "stage1_limit", "stage2_limit"):
                row[key] = int(row[key])
            for key in ("accepted", "budget_exhausted"):
                row[key] = flag(row[key])
            for key in ("original_error_m", "position_error_m", "orientation_error_rad",
                        "safe_limit_clearance_rad", "q_distance_rad", "convergence_delta"):
                row[key] = number(row[key])
            if (row["variant"] not in VARIANTS or row["mode"] != 1
                    or row["safe_limit_clearance_rad"] < -1e-9
                    or any(row[key] < 0 for key in ("position_error_m", "orientation_error_rad", "q_distance_rad"))
                    or row["seed"] != "recorded_solution" or row["cold_solver"] != "true"
                    or row["deadline"] != "unbounded"
                    or row["scope"] != "isolated_target_not_control_replay"
                    or any(not 0 <= row[f"stage{s}_iterations"] <= row[f"stage{s}_limit"] for s in (1, 2))):
                raise ValueError("invalid task ablation")
            trials.append(row)
    if (contract is None or provenance is None or summary is None or not samples or not projections
            or contract["schema"] != "1" or contract["control_dt_ns"] != "5000000"
            or contract["time_basis"] != "nanoseconds_since_first_receive"
            or provenance["motion_authorized"] != "false" or provenance["phase_a_accepted"] != "false"
            or len(samples) != 2*int(summary["control_cycles"])
            or len(projections) != int(contract["source_frames"])
            or len(projections) != int(summary["frames"])
            or projections[0]["t_ns"] != 0 or projections[-1]["t_ns"] != int(contract["duration_ns"])):
        raise ValueError("incomplete/inconsistent native diagnostics")
    duration = int(contract["duration_ns"])
    if len(samples) != 2*((duration+4999999)//5000000+1):
        raise ValueError("incorrect 200 Hz cycle count")
    if sum(row["ik_valid"] for row in samples) != int(summary["side_samples"]):
        raise ValueError("IK sample denominator mismatch")
    selected = {(s["cycle"], s["side"]): s for s in samples if s["ik_m"] is not None and s["ik_m"] > .01}
    if len(selected) != int(summary["position_over_10mm"]):
        raise ValueError("ablation selection denominator mismatch")
    seen = set()
    for trial in trials:
        key = (trial["cycle"], trial["side"])
        sample = selected.get(key)
        identity = (*key, trial["variant"])
        if (sample is None or identity in seen or trial["sequence"] != sample["sequence"]
                or trial["t_ns"] != sample["t_ns"] or abs(trial["original_error_m"]-sample["ik_m"]) > 1e-12):
            raise ValueError("unpaired/duplicate ablation")
        seen.add(identity)
    if len(seen) != len(selected)*len(VARIANTS):
        raise ValueError("missing paired ablation")
    return duration, projections, samples, trials, provenance


def summarize_samples(rows):
    result = dict(side_samples=len(rows), ik_excluded=sum(not r["ik_valid"] for r in rows),
                  reference_not_attempted=sum(not r["reference_attempted"] for r in rows),
                  reference_rejected=sum(r["reference_attempted"] and not r["reference_accepted"] for r in rows),
                  details=dict(Counter(r["detail"] for r in rows)),
                  headroom_sources=dict(Counter(r["headroom_source"] for r in rows)),
                  headroom_below_half=sum(r["headroom_scale"] is not None and r["headroom_scale"] < .5 for r in rows))
    result["metrics"] = {key: stats([r[key] for r in rows if r[key] is not None]) for key in METRICS}
    result["reference_command_over_50mm"] = sum(r["reference_command_m"] is not None and r["reference_command_m"] > .05 for r in rows)
    result["ik_over_10mm"] = sum(r["ik_m"] is not None and r["ik_m"] > .01 for r in rows)
    return result


def build_report(text, annotations, trace_hash):
    duration, projections, samples, trials, provenance = parse_native(text)
    if provenance["trace_sha256"] != trace_hash:
        raise ValueError("trace fingerprint mismatch")
    segments = validate_annotations(annotations, trace_hash, duration)
    by_action, by_state, by_action_state, by_side = (defaultdict(list) for _ in range(4))
    for row in samples:
        action = action_at(row["t_ns"], segments, duration)
        state = f'{row["state"]}|settled_hold={int(row["settled_hold"])}|blend={int(row["blend"])}'
        by_action[action].append(row)
        by_state[state].append(row)
        by_action_state[f"{action}/{state}"].append(row)
        by_side[row["side"]].append(row)
    projection_groups = defaultdict(list)
    for row in projections:
        projection_groups[action_at(row["t_ns"], segments, duration)].append(row)
    ablations = {}
    baselines = {(r["cycle"], r["side"]): r for r in trials if r["variant"] == "baseline"}
    for variant in VARIANTS:
        rows = [r for r in trials if r["variant"] == variant]
        paired = [r for r in rows if r["accepted"] and baselines[r["cycle"], r["side"]]["accepted"]]
        ablations[variant] = dict(samples=len(rows), accepted=sum(r["accepted"] for r in rows),
            paired_accepted=len(paired), actions=dict(Counter(action_at(r["t_ns"], segments, duration) for r in rows)),
            position_m=stats([r["position_error_m"] for r in rows if r["accepted"]]),
            orientation_rad=stats([r["orientation_error_rad"] for r in rows if r["accepted"]]),
            position_improvement_over_cold_baseline_m=stats([
                baselines[r["cycle"], r["side"]]["position_error_m"]-r["position_error_m"] for r in paired]),
            under_10mm=sum(r["accepted"] and r["position_error_m"] < .01 for r in rows),
            near_safe_limit=sum(r["accepted"] and r["safe_limit_clearance_rad"] < .01 for r in rows),
            stops=dict(Counter(r["stop"] for r in rows)))
    return dict(schema_version=1, scope="offline_200hz_reference_layers_and_isolated_task_ablation",
        actual="unavailable", phase_a_accepted=False, motion_authorized=False, thresholds_frozen=False,
        provenance=provenance, duration_ns=duration, overall=summarize_samples(samples),
        by_action={k: summarize_samples(v) for k, v in by_action.items()},
        by_state={k: summarize_samples(v) for k, v in by_state.items()},
        by_action_state={k: summarize_samples(v) for k, v in by_action_state.items()},
        by_side={k: summarize_samples(v) for k, v in by_side.items()},
        missing_actions=[a for a in ACTIONS if not by_action.get(a)],
        mapping_projection_by_action={k: dict(unique_source_frames=len(v), rejected=sum(not r["valid"] for r in v),
            shift_m=stats([r["shift_m"] for r in v if r["valid"]])) for k, v in projection_groups.items()},
        task_ablations=ablations,
        limitations=["control statistics count side-ticks; mapping statistics count unique source frames",
                     "action assignment uses evaluation time; source time is retained in raw evidence",
                     "post-trace tick is isolated; no unobserved EOF interval is labelled",
                     "projection costs are standalone mapping, not controller recovery history",
                     "IK target is solver input; command target is post-HOLD; quantiles cannot be added",
                     "ablation selects IK error >10mm; paired cold restarts, not live rollout or global feasibility",
                     "50mm and 10mm counts are diagnostic, not activation thresholds"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--profile", type=Path, default=CONTROL / "config/qp_ik_pico_shared_root_reachable.yaml")
    parser.add_argument("--annotations", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, help="new exclusive diagnostic directory; never overwritten")
    args = parser.parse_args()
    try:
        paths = (args.trace, args.profile, args.annotations)
        hashes = tuple(sha256(p) for p in paths)
        if args.output_dir and args.output_dir.exists():
            raise ValueError("output directory already exists")
        result = subprocess.run([str(CONTROL / "build/tianji_shared_root_trace_audit"),
            str(args.profile), str(args.trace), "--layer-diagnostics"],
            capture_output=True, text=True, check=True, timeout=300)
        report = build_report(result.stdout, json.loads(args.annotations.read_text()), hashes[0])
        if tuple(sha256(p) for p in paths) != hashes or report["provenance"]["profile_sha256"] != hashes[1]:
            raise ValueError("inputs changed or profile fingerprint mismatched")
        report["annotations_sha256"] = hashes[2]
        encoded = json.dumps(report, indent=2, allow_nan=False)
        if args.output_dir:
            args.output_dir.mkdir()  # No parents/exist_ok: do not replace existing evidence.
            with (args.output_dir / "native.txt").open("x") as stream:
                stream.write(result.stdout)
            with (args.output_dir / "report.json").open("x") as stream:
                stream.write(encoded + "\n")
            print(json.dumps(dict(output_dir=str(args.output_dir), side_samples=report["overall"]["side_samples"],
                missing_actions=report["missing_actions"], motion_authorized=False)))
        else:
            print(encoded)
    except (OSError, ValueError, KeyError, IndexError, TypeError, subprocess.SubprocessError) as error:
        parser.exit(2, f"{error}\n")


if __name__ == "__main__":
    main()
