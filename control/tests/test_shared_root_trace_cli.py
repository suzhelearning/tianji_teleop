"""Reject malformed trace files before any model or guidance is constructed."""
from pathlib import Path
import os
import struct
import subprocess

import pytest

CONTROL = Path(__file__).resolve().parents[1]
BINARY = CONTROL / "build/tianji_shared_root_trace_audit"


@pytest.mark.parametrize("mode", ["--reference-loop-200hz", "--worst-ik-multiseed"])
def test_projected_hold_does_not_relabel_command_gap_as_ik_error(tmp_path, mode):
    source = CONTROL.parent / "recordings/shared_root/zhoujie_actions_20260918_041020/input.tjvr"
    if not source.is_file() or not BINARY.is_file():
        pytest.skip("requires recorded hold regression trace and built native audit")
    raw = source.read_bytes()
    magic, version, size, count = struct.unpack_from("<4sHHQ", raw)
    assert magic == b"TJVT" and version == 1 and size == 656 and count >= 720
    # Includes sequence 3115 / 6.555 s: post-IK HOLD changes command target by
    # about 173 mm while the solver's own palm residual is sub-micrometre.
    trace = tmp_path / "hold-prefix.tjvr"
    trace.write_bytes(struct.pack("<4sHHQ", magic, version, size, 720) + raw[16:16+720*(size+8)])
    result = subprocess.run([str(BINARY), str(CONTROL / "config/qp_ik_pico_shared_root_reachable.yaml"),
                             str(trace), mode], capture_output=True, text=True, timeout=90)
    assert result.returncode == 0, result.stderr
    line = next(x for x in result.stdout.splitlines() if x.startswith("ik_target_pairing mode=1 "))
    fields = dict(x.split("=", 1) for x in line.split()[1:])
    assert float(fields["max_ik_vs_command_target_m"]) > .1
    assert float(fields["max_residual_disagreement_m"]) < 1e-5
    assert fields["ik_target"] == "solver_input"
    assert fields["reference_target"] == "post_hold_command"
    worst = next(x for x in result.stdout.splitlines() if x.startswith("worst_ik_sample mode=1 "))
    sample = dict(x.split("=", 1) for x in worst.split()[1:])
    assert abs(float(sample["fk_position_error_m"]) - float(sample["solver_position_error_m"])) < 1e-5
    assert sample["side"] in ("left", "right")
    assert sample["classification"] == "CauseUndetermined"
    for stage in ("stage1", "stage2"):
        assert 0 < int(sample[stage + "_iterations"]) <= int(sample[stage + "_iteration_limit"])
    probe = next(x for x in result.stdout.splitlines() if x.startswith("residual_probe mode=1 "))
    summary = dict(x.split("=", 1) for x in probe.split()[1:])
    assert sample["sequence"] == summary["worst_sequence"]
    assert sample["side"] == summary["worst_side"]
    assert sample["fk_position_error_m"] == summary["worst_error_m"]
    trials = [dict(token.split("=", 1) for token in line.split()[1:])
              for line in result.stdout.splitlines() if line.startswith("ik_multiseed mode=1 ")]
    if mode == "--worst-ik-multiseed":
        assert len(trials) == 16
        assert [int(t["trial"]) for t in trials] == list(range(16))
        assert trials[0]["seed"] == "previous_solution"
        assert trials[1]["seed"] == "safe_midpoint"
        for trial in trials:
            assert trial["sequence"] == sample["sequence"]
            assert trial["side"] == sample["side"]
            assert float(trial["safe_limit_clearance_rad"]) >= -1e-9
            assert trial["cold_solver"] == "true"
            assert trial["scope"] == "isolated_target_not_control_replay"
            assert trial["actual"] == "unavailable"
    else:
        assert not trials


@pytest.mark.parametrize("data,reason", [
    (b"", "invalid TJVT"),
    (struct.pack("<4sHHQ", b"BAD!", 1, 656, 1), "invalid TJVT"),
    (struct.pack("<4sHHQ", b"TJVT", 1, 656, 0), "empty or excessive"),
    (struct.pack("<4sHHQ", b"TJVT", 1, 656, 1000001), "empty or excessive"),
    (struct.pack("<4sHHQ", b"TJVT", 1, 656, 1), "truncated"),
    (struct.pack("<4sHHQ", b"TJVT", 1, 656, 1) + bytes(664), "packet rejected"),
])
@pytest.mark.parametrize("extra", [[], ["--reference-loop"], ["--reference-loop-200hz"], ["--worst-ik-multiseed"], ["--layer-diagnostics"], ["--scale-ablation"], ["--mapping-transitions"], ["--mapping-only"], ["--mapping-frames"]])
def test_native_trace_reader_rejects_bad_files(tmp_path, data, reason, extra):
    if not BINARY.is_file():
        pytest.skip("build tianji_shared_root_trace_audit first")
    trace = tmp_path / "bad.tjvr"
    trace.write_bytes(data)
    env = dict(os.environ)
    lib = CONTROL.parent / ".pixi/envs/default/lib"
    env["LD_LIBRARY_PATH"] = str(lib) + ":" + env.get("LD_LIBRARY_PATH", "")
    result = subprocess.run([str(BINARY), "must-not-load-this-profile", str(trace), *extra],
                            env=env, capture_output=True, text=True, timeout=10)
    assert result.returncode == 2
    assert reason in result.stderr
    assert not result.stdout


def test_mapping_only_trace_geometry_without_ik():
    trace = CONTROL.parent / "recordings/shared_root/zhoujie_50s_20260917_235744_GLTAPL/input.tjvr"
    if not BINARY.is_file() or not trace.is_file():
        pytest.skip("requires built audit and local zhoujie trace")
    result = subprocess.run([str(BINARY), str(CONTROL / "config/qp_ik_pico_shared_root.yaml"),
                             str(trace), "--mapping-only"], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
    assert "ik_executed=false" in result.stdout
    assert "\nmode=" not in result.stdout
    line = next(x for x in result.stdout.splitlines() if x.startswith("mapping_geometry "))
    fields = dict(x.split("=", 1) for x in line.split()[1:])
    assert int(fields["frames"]) == 4388
    assert int(fields["valid"]) == 4376
    assert int(fields["rejected"]) == 12
    assert float(fields["raw_palm_shape_gap_p90_m"]) == 0
    assert float(fields["filtered_palm_shape_gap_p90_m"]) == 0
    coverage = next(x for x in result.stdout.splitlines() if x.startswith("mapping_coverage "))
    stats = dict(x.split("=", 1) for x in coverage.split()[1:])
    assert int(stats["evaluated_unique_frames"]) == 4388
    assert float(stats["geometric_target_valid_ratio"]) == pytest.approx(4376 / 4388, abs=1e-6)
    assert float(stats["mapped_palm_outside_workspace_ratio"]) == pytest.approx(12 / 4388, abs=1e-8)
    assert float(stats["maximum_continuous_invalid_duration_s"]) == pytest.approx(.062382, abs=1e-6)
    assert float(stats["invalid_duration_s"]) == pytest.approx(.136743, abs=1e-6)
    assert stats["action"] == "unannotated"
    assert stats["per_action_acceptance"] == "unavailable"
    assert stats["eof_tail"] == "unobserved"
    assert "closure_side layer=raw side=left reason=outside_workspace frames=12" in result.stdout
    assert "closure_side layer=filtered side=left reason=outside_workspace frames=9" in result.stdout
    for name in ("raw_relation_max_m", "raw_center_max_m", "raw_filtered_bone_max_m",
                 "raw_filtered_shoulder_max_m", "raw_orientation_matrix_max"):
        assert float(fields[name]) <= 1e-9


def test_mapping_rejections_are_speed_conflicts_and_transitions_are_bounded():
    trace = CONTROL.parent / "recordings/shared_root/zhoujie_actions_20260918_041020/input.tjvr"
    if not BINARY.is_file() or not trace.is_file():
        pytest.skip("requires built audit and local action trace")
    result = subprocess.run([str(BINARY), str(CONTROL / "config/qp_ik_pico_shared_root_reachable.yaml"),
                             str(trace), "--mapping-transitions"], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
    def rows(prefix):
        return [dict(token.split("=", 1) for token in line.split()[1:])
                for line in result.stdout.splitlines() if line.startswith(prefix + " ")]
    rejected = rows("projection_rejection")
    assert [int(r["sequence"]) for r in rejected] == [2589, 6863]
    for row in rejected:
        assert row["classification"] == "ProvenDisjointConstraintBalls"
        assert row["pair_b"] == "speed_bound"
        assert float(row["pair_disjoint_gap_m"]) > 1e-4
        assert float(row["residual_4096_m"]) > 1e-4
        assert float(row["no_speed_residual_4096_m"]) <= 1e-10
    summary, = rows("projection_transition_summary")
    assert int(summary["continuous_pairs"]) == 4421
    assert int(summary["history_starts"]) == 4
    assert float(summary["maximum_correction_speed_m_s"]) <= .5 + 1e-9
    assert int(summary["entries"]) == int(summary["exits"]) == 3
    for row in rows("projection_transition"):
        assert row["intent_valid"] == "0"
        assert float(row["correction_step_m"]) <= .5*float(row["source_dt_s"]) + 1e-10
    for row in rows("mapping_continuity"):
        assert row["opposed_direction_events"] == "0"
    spans, = rows("elbow_degenerate_span_summary")
    assert spans["compared"] == "33" and spans["opposed"] == "0" and spans["unresolved"] == "0"
    assert "ik_executed=false motion_authorized=false phase_a_accepted=false" in result.stdout
    assert "\nmode=" not in result.stdout


def test_duplicate_trace_frames_cannot_inflate_coverage(tmp_path):
    source = CONTROL.parent / "recordings/shared_root/zhoujie_50s_20260917_235744_GLTAPL/input.tjvr"
    if not BINARY.is_file() or not source.is_file():
        pytest.skip("requires built audit and local zhoujie trace")
    with source.open("rb") as stream:
        stream.read(16)
        record = stream.read(664)
    trace = tmp_path / "duplicate.tjvr"
    trace.write_bytes(struct.pack("<4sHHQ", b"TJVT", 1, 656, 2) + record + record)
    result = subprocess.run([str(BINARY), "must-not-load-profile", str(trace), "--mapping-only"],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 2
    assert "duplicate epoch/sequence" in result.stderr
    assert not result.stdout


@pytest.mark.parametrize("mode", ["--reference-loop", "--reference-loop-200hz"])
def test_recorded_reference_loop_accepts_stop_and_recovers(mode):
    trace = (CONTROL.parent / "recordings/shared_root/"
             "zhoujie_50s_20260917_235744_GLTAPL/input.tjvr")
    if not BINARY.is_file() or not trace.is_file():
        pytest.skip("requires built audit and local zhoujie acceptance trace")
    env = dict(os.environ)
    env["LD_LIBRARY_PATH"] = str(CONTROL.parent / ".pixi/envs/default/lib") + ":" + env.get("LD_LIBRARY_PATH", "")
    result = subprocess.run(
        [str(BINARY), str(CONTROL / "config/qp_ik_pico_shared_root.yaml"),
         str(trace), mode], env=env, capture_output=True, text=True, timeout=90)
    assert result.returncode == 0, result.stderr
    lines = result.stdout.splitlines()
    summary = next(line for line in lines if line.startswith("reference_loop mode=1 "))
    fields = dict(item.split("=", 1) for item in summary.split()[1:])
    cycle_summary = next(line for line in lines if line.startswith("mode=1 "))
    cycle_fields = dict(item.split("=", 1) for item in cycle_summary.split())
    assert int(fields["accepted"]) == int(cycle_fields["control_cycles"])
    layers = next(line for line in lines if line.startswith("reference_layers mode=1 "))
    layer_fields = dict(item.split("=", 1) for item in layers.split()[1:])
    assert int(layer_fields["feedback_cycles"]) == int(cycle_fields["control_cycles"])
    assert int(cycle_fields["delivered_source_frames"]) + int(cycle_fields["superseded_source_frames"]) == 4388
    assert int(fields["rejected"]) == 0
    assert int(fields["recovery_confirmations"]) > 0
    assert int(fields["tracking_frames"]) > 4000
    closure = next(line for line in lines if line.startswith("closed_control mode=1 "))
    closure_fields = dict(item.split("=", 1) for item in closure.split()[1:])
    assert int(closure_fields["side_samples"]) > 0
    assert float(closure_fields["max_bone_error_m"]) <= 1e-9
    assert float(closure_fields["max_endpoint_error_m"]) <= 1e-9
    assert "shared_root_reference_reset_failed" not in result.stdout
    assert "actual=unavailable" in result.stdout
    assert "motion_authorized=false phase_a_accepted=false" in result.stdout
