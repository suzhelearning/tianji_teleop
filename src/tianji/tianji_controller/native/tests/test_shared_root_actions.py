import importlib.util
import json
from pathlib import Path
import subprocess

import pytest

CONTROL = Path(__file__).resolve().parents[1]
SCRIPT = CONTROL / "scripts/report_shared_root_actions.py"
spec = importlib.util.spec_from_file_location("action_coverage", SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def document(segments):
    return dict(schema_version=1, trace_sha256="trace", time_basis="nanoseconds_since_first_receive",
                segments=segments)


def interval(a, b, action="natural_reach"):
    return dict(start_ns=a, end_ns=b, action=action)


@pytest.mark.parametrize("segments", [
    [interval(-1, 5)], [interval(5, 5)], [interval(0, 21)],
    [interval(0., 5)], [interval(False, 5)], [interval(0, 5, "guess")],
    [interval(0, 10), interval(5, 15)], [interval(10, 15), interval(0, 5)],
])
def test_invalid_intervals_rejected(segments):
    with pytest.raises(ValueError):
        module.validate_annotations(document(segments), "trace", 20)


@pytest.mark.parametrize("key,value", [("trace_sha256", "other"), ("schema_version", 2),
                                      ("schema_version", True),
                                      ("time_basis", "source_timestamp"), ("segments", None)])
def test_annotation_identity_and_schema(key, value):
    doc = document([])
    doc[key] = value
    with pytest.raises(ValueError):
        module.validate_annotations(doc, "trace", 20)


def test_interval_clips_silence_and_preserves_unknown():
    frames = [(0, True), (300, False), (350, False), (400, True), (500, True)]
    segments = [interval(50, 350)]
    result = module.summarize(frames, 100, segments)
    assert result["overall"]["maximum_continuous_invalid_duration_ns"] == 300
    assert result["actions"]["natural_reach"]["maximum_continuous_invalid_duration_ns"] == 250
    assert result["actions"]["natural_reach"]["evaluated_unique_frames"] == 1
    assert result["unannotated"]["evaluated_unique_frames"] == 4
    assert len(result["missing_actions"]) == 6
    assert result["phase_a_accepted"] is False


def test_adjacent_labels_cannot_hide_long_invalid_run():
    frames = [(0, False), (100, False), (200, True)]
    result = module.summarize(frames, 100, [interval(0, 100), interval(100, 200)])
    assert result["actions"]["natural_reach"]["maximum_continuous_invalid_duration_ns"] == 200
    assert result["actions"]["natural_reach"]["evaluated_unique_frames"] == 3
    assert result["unannotated"]["evaluated_unique_frames"] == 0


def test_empty_evidence_is_not_covered_and_valid_event_resets_run():
    frames = [(0, False), (100, True), (100, False), (200, True)]
    result = module.summarize(frames, 10, [interval(10, 20)])
    assert "natural_reach" in result["missing_actions"]
    assert result["overall"]["maximum_continuous_invalid_duration_ns"] == 100


def test_no_labels_and_single_frame_do_not_invent_time():
    result = module.summarize([(0, True)], 100, [])
    assert len(result["missing_actions"]) == 7
    assert result["unannotated"]["evaluated_unique_frames"] == 1
    assert result["overall"]["evaluated_duration_ns"] == 0


@pytest.mark.parametrize("text", [
    "", "mapping_frame 0 1 0\n", "mapping_frame -1 1 0\n",
    "mapping_frame 0 1 2\n", "mapping_frame 1 1 0\nmapping_frame 0 2 0\n",
    "mapping_frame 0 1 1 " + "nan " * 48,
])
def test_malformed_native_output_fails_closed(text):
    with pytest.raises(ValueError):
        module.parse_native(text)


def test_real_trace_unannotated_does_not_grant_acceptance():
    trace = CONTROL.parent / "recordings/shared_root/zhoujie_50s_20260917_235744_GLTAPL/input.tjvr"
    if not trace.is_file() or not (CONTROL / "build/tianji_shared_root_trace_audit").is_file():
        pytest.skip("requires local trace and native audit")
    import sys
    result = subprocess.run([sys.executable, str(SCRIPT), str(trace)],
                            capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    assert report["overall"]["evaluated_unique_frames"] == 4388
    assert report["overall"]["valid_frames"] == 4376
    assert report["overall"]["invalid_duration_ns"] / 1e9 == pytest.approx(.136743, abs=1e-6)
    assert report["overall"]["maximum_continuous_invalid_duration_ns"] / 1e9 == pytest.approx(.062382, abs=1e-6)
    assert report["unannotated"]["evaluated_unique_frames"] == 4388
    assert len(report["missing_actions"]) == 7
    assert not report["motion_authorized"] and not report["phase_a_accepted"]
