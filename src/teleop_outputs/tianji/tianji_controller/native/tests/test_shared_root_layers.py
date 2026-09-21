import importlib.util
from pathlib import Path
import sys
import struct
import subprocess

import pytest
from tianji_runtime import ResourceNotFound, controller_profile, native_executable, workspace

CONTROL = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CONTROL / "scripts"))
spec = importlib.util.spec_from_file_location("layer_report", CONTROL / "scripts/report_shared_root_layers.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def annotations():
    return dict(schema_version=1, trace_sha256="trace", time_basis="nanoseconds_since_first_receive",
                segments=[dict(action="natural_reach", start_ns=0, end_ns=9000000)])


def encode(prefix, row):
    return prefix + " " + " ".join(f"{key}={value}" for key, value in row.items())


def evidence():
    lines = ["layer_contract schema=1 control_dt_ns=5000000 duration_ns=9000000 source_frames=2 time_basis=nanoseconds_since_first_receive",
             "projection_frame t_ns=0 sequence=1 valid=1 shift_m=0.002 reason=accepted",
             "projection_frame t_ns=9000000 sequence=2 valid=0 shift_m=NA reason=unreachable"]
    for cycle in range(3):
        for side in ("left", "right"):
            valid = cycle != 1
            row = dict(mode=1, cycle=cycle, t_ns=cycle*5000000, source_t_ns=0 if cycle < 2 else 9000000,
                       sequence=1 if cycle < 2 else 2, side=side,
                       state="tracking" if valid else "invalid", settled_hold=int(cycle == 2), blend=0,
                       guidance_accepted=1, target_valid=int(valid), ik_valid=int(valid),
                       reference_attempted=1, reference_accepted=1, ff_valid=int(valid),
                       headroom_valid=int(valid), headroom_source="jerk" if valid else "unavailable", detail="ok")
            row.update({key: (0.001 if valid else "NA") for key in module.METRICS})
            if valid:
                row["ik_m"] = .02 if cycle == 0 else .005
                row["reference_command_m"] = .08
                row["reference_ik_target_m"] = .12
                row["headroom_scale"] = .3
            lines.append(encode("layer_sample", row))
    for side in ("left", "right"):
        for variant in module.VARIANTS:
            row = dict(mode=1, cycle=0, t_ns=0, sequence=1, side=side, variant=variant,
                       original_error_m=.02, accepted=1, position_error_m=.019 if variant == "baseline" else .005,
                       orientation_error_rad=.01, safe_limit_clearance_rad=0, stop="converged",
                       stage1_iterations=2, stage2_iterations=2, stage1_limit=10, stage2_limit=10,
                       convergence_delta=.0001, budget_exhausted=0, q_distance_rad=.01,
                       seed="recorded_solution", cold_solver="true", deadline="unbounded",
                       scope="isolated_target_not_control_replay")
            lines.append(encode("task_ablation", row))
    lines += ["mode=1 frames=2 control_cycles=3 side_samples=4 position_over_10mm=2",
              "trace_sha256=trace profile_sha256=profile motion_authorized=false phase_a_accepted=false"]
    return "\n".join(lines)


def test_action_state_denominators_exclusions_and_tail():
    report = module.build_report(evidence(), annotations(), "trace")
    assert report["overall"]["side_samples"] == 6
    assert report["overall"]["ik_excluded"] == 2
    assert report["by_action"]["natural_reach"]["side_samples"] == 4
    assert report["by_action"]["post_trace_tail"]["side_samples"] == 2
    assert report["by_action"]["natural_reach"]["metrics"]["ik_m"]["count"] == 2
    assert report["overall"]["metrics"]["reference_command_m"]["p90"] == .08
    assert report["overall"]["metrics"]["reference_ik_target_m"]["p90"] == .12
    assert report["mapping_projection_by_action"]["natural_reach"]["unique_source_frames"] == 2
    assert report["mapping_projection_by_action"]["natural_reach"]["rejected"] == 1
    assert report["by_state"]["invalid|settled_hold=0|blend=0"]["ik_excluded"] == 2
    assert report["task_ablations"]["pose_only"]["position_improvement_over_cold_baseline_m"]["p90"] == pytest.approx(.014)
    assert len(report["missing_actions"]) == 6
    assert not report["motion_authorized"] and not report["phase_a_accepted"] and not report["thresholds_frozen"]


@pytest.mark.parametrize("old,new", [
    ("profile_sha256=profile", "profile_sha256=profile motion_authorized=true"),
    ("control_cycles=3", "control_cycles=4"), ("source_frames=2", "source_frames=3"),
    ("side_samples=4", "side_samples=3"), ("position_over_10mm=2", "position_over_10mm=1"),
    ("side=left", "side=unknown"), ("cycle=0", "cycle=-1"),
    ("ik_m=0.02", "ik_m=nan"), ("ik_m=0.02", "ik_m=NA"),
    ("headroom_scale=0.3", "headroom_scale=-1"), ("valid=0 shift_m=NA", "valid=0 shift_m=0"),
    ("source_t_ns=0", "source_t_ns=1"), ("t_ns=9000000 sequence=2", "t_ns=8000000 sequence=2"),
    ("safe_limit_clearance_rad=0", "safe_limit_clearance_rad=-0.1"),
    ("cold_solver=true", "cold_solver=false"), ("seed=recorded_solution", "seed=unknown"),
    ("original_error_m=0.02", "original_error_m=0.03"),
    ("stage1_iterations=2", "stage1_iterations=20"),
])
def test_invalid_evidence_rejected(old, new):
    text = evidence().replace(old, new, 1)
    with pytest.raises(ValueError):
        module.build_report(text, annotations(), "trace")


def test_duplicate_or_missing_trial_and_truncated_output_rejected():
    lines = evidence().splitlines()
    trial = next(line for line in lines if line.startswith("task_ablation "))
    for text in (evidence() + "\n" + trial, "\n".join(line for line in lines if line != trial),
                 "\n".join(lines[:-1])):
        with pytest.raises(ValueError):
            module.build_report(text, annotations(), "trace")


def test_no_labels_stays_unknown_and_annotation_hash_is_checked():
    document = annotations()
    document["segments"] = []
    report = module.build_report(evidence(), document, "trace")
    assert report["by_action"]["unannotated"]["side_samples"] == 4
    document["trace_sha256"] = "wrong"
    with pytest.raises(ValueError):
        module.build_report(evidence(), document, "trace")


def test_half_open_action_boundaries_and_final_sample():
    segments = [dict(action="natural_reach", start_ns=0, end_ns=5),
                dict(action="crossing", start_ns=5, end_ns=10)]
    assert module.action_at(5, segments, 10) == "crossing"
    assert module.action_at(10, segments, 10) == "crossing"
    assert module.action_at(11, segments, 10) == "post_trace_tail"
    assert module.stats([])["p90"] is None


def test_native_layer_replay_and_report_pairing(tmp_path):
    source = workspace() / "recordings/shared_root/zhoujie_actions_20260918_041020/input.tjvr"
    try:
        binary = native_executable("tianji_shared_root_trace_audit")
    except ResourceNotFound as error:
        pytest.skip(str(error))
    if not source.is_file():
        pytest.skip("requires local action recording and built audit")
    raw = source.read_bytes()
    magic, version, size, count = struct.unpack_from("<4sHHQ", raw)
    assert magic == b"TJVT" and version == 1 and size == 656 and count >= 720
    trace = tmp_path / "layer-prefix.tjvr"
    trace.write_bytes(struct.pack("<4sHHQ", magic, version, size, 720) + raw[16:16+720*(size+8)])
    result = subprocess.run([str(binary), str(controller_profile("qp_ik_pico_shared_root_reachable.yaml")),
                             str(trace), "--layer-diagnostics"], capture_output=True, text=True, timeout=90)
    assert result.returncode == 0, result.stderr
    document = dict(schema_version=1, trace_sha256=module.sha256(trace),
                    time_basis="nanoseconds_since_first_receive", segments=[])
    report = module.build_report(result.stdout, document, document["trace_sha256"])
    summary = module.fields(next(line for line in result.stdout.splitlines() if line.startswith("mode=1 ")))
    reference = module.fields(next(line for line in result.stdout.splitlines() if line.startswith("reference_loop mode=1 ")))
    assert report["overall"]["metrics"]["ik_m"]["p90"] == float(summary["palm_p90_m"])
    assert report["overall"]["metrics"]["reference_command_m"]["p90"] == float(reference["reference_palm_p90_m"])
    assert report["overall"]["side_samples"] == 2*int(summary["control_cycles"])
    assert report["overall"]["ik_excluded"] == 2*int(summary["excluded"])
    assert report["mapping_projection_by_action"]["unannotated"]["unique_source_frames"] == 720
    for variant in module.VARIANTS:
        assert report["task_ablations"][variant]["samples"] == int(summary["position_over_10mm"])
    assert len(report["missing_actions"]) == 7
    assert report["overall"]["metrics"]["ik_command_m"]["max"] > .1
