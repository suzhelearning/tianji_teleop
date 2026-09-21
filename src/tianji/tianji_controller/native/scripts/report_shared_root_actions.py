"""Read-only action coverage from native mapping output; never an enable gate."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess

CONTROL = Path(__file__).resolve().parents[1]
ACTIONS = ("natural_reach", "hands_approach", "crossing", "unequal_height",
           "single_hand", "bilateral_motion", "extension_boundary")


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def fields(line):
    return dict(token.split("=", 1) for token in line.split() if "=" in token)


def parse_native(text):
    frames, coverage, provenance = [], None, None
    for line in text.splitlines():
        if line.startswith("mapping_frame "):
            parts = line.split()
            stamp, seq, valid = map(int, parts[1:4])
            values = [float(x) for x in parts[4:]]
            if (stamp < 0 or seq <= 0 or valid not in (0, 1)
                    or len(values) != (48 if valid else 0)
                    or not all(math.isfinite(x) for x in values)
                    or (frames and stamp < frames[-1][0])):
                raise ValueError("invalid native mapping frame")
            frames.append((stamp, bool(valid)))
        elif line.startswith("mapping_coverage "):
            if coverage is not None:
                raise ValueError("duplicate native coverage summary")
            coverage = fields(line)
        elif line.startswith("trace_sha256="):
            if provenance is not None:
                raise ValueError("duplicate native provenance")
            provenance = fields(line)
    if not frames or coverage is None or provenance is None:
        raise ValueError("incomplete native mapping output")
    if (provenance.get("ik_executed") != "false"
            or provenance.get("motion_authorized") != "false"
            or provenance.get("phase_a_accepted") != "false"
            or int(coverage["evaluated_unique_frames"]) != len(frames)):
        raise ValueError("native mapping-only contract mismatch")
    freshness = float(coverage["freshness_s"])
    if not math.isfinite(freshness) or freshness <= 0:
        raise ValueError("invalid native freshness")
    origin = frames[0][0]
    return [(t-origin, v) for t, v in frames], round(freshness*1e9), provenance


def validate_annotations(document, trace_hash, duration):
    if (not isinstance(document, dict) or type(document.get("schema_version")) is not int
            or document.get("schema_version") != 1
            or document.get("trace_sha256") != trace_hash):
        raise ValueError("annotation schema/trace hash mismatch")
    if document.get("time_basis") != "nanoseconds_since_first_receive":
        raise ValueError("annotation time basis mismatch")
    segments = document.get("segments")
    if not isinstance(segments, list):
        raise ValueError("segments must be a list")
    last = 0
    for segment in segments:
        if not isinstance(segment, dict) or segment.get("action") not in ACTIONS:
            raise ValueError("unknown action")
        a, b = segment.get("start_ns"), segment.get("end_ns")
        if (type(a) is not int or type(b) is not int
                or not 0 <= a < b <= duration or a < last):
            raise ValueError("invalid, overlapping or unordered annotation interval")
        last = b
    return segments


def invalid_runs(frames, freshness):
    """Match the native timeline: a valid source event resets the invalid run."""
    runs = []
    continuation = False
    for (start, valid), (end, _) in zip(frames, frames[1:]):
        if valid:
            continuation = False
            start = min(end, start + freshness)
        if start < end:
            if continuation:
                runs[-1] = (runs[-1][0], end)
            else:
                runs.append((start, end))
            continuation = True
    return runs


def metrics(frames, runs, intervals, duration):
    # Splitting a single action into adjacent labels must not shorten its
    # reported longest invalid run or double-count boundary samples.
    merged = []
    for a, b in intervals:
        if merged and merged[-1][1] == a:
            merged[-1] = (merged[-1][0], b)
        else:
            merged.append((a, b))
    count = valid = total_invalid = maximum_invalid = 0
    for a, b in merged:
        selected = [v for t, v in frames if a <= t < b or (t == b == duration)]
        count += len(selected)
        valid += sum(selected)
        for start, end in runs:
            overlap = max(0, min(b, end) - max(a, start))
            total_invalid += overlap
            maximum_invalid = max(maximum_invalid, overlap)
    return dict(evaluated_unique_frames=count, valid_frames=valid,
                geometric_target_valid_ratio=valid/count if count else None,
                evaluated_duration_ns=sum(b-a for a, b in intervals),
                invalid_duration_ns=total_invalid,
                maximum_continuous_invalid_duration_ns=maximum_invalid)


def summarize(frames, freshness, segments):
    duration = frames[-1][0]
    runs = invalid_runs(frames, freshness)
    gaps, cursor = [], 0
    for segment in segments:
        if cursor < segment["start_ns"]:
            gaps.append((cursor, segment["start_ns"]))
        cursor = segment["end_ns"]
    if cursor < duration or not segments:
        gaps.append((cursor, duration))
    actions = {}
    for action in ACTIONS:
        intervals = [(s["start_ns"], s["end_ns"]) for s in segments if s["action"] == action]
        actions[action] = metrics(frames, runs, intervals, duration)
        actions[action]["annotated"] = bool(intervals)
    return dict(overall=metrics(frames, runs, [(0, duration)], duration),
                actions=actions, unannotated=metrics(frames, runs, gaps, duration),
                missing_actions=[a for a in ACTIONS if not actions[a]["evaluated_unique_frames"]],
                thresholds_frozen=False, phase_a_accepted=False, motion_authorized=False,
                actual="unavailable", scope="offline_mapping_action_coverage",
                window="first_to_last_receive", eof_tail="unobserved")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--profile", type=Path, default=CONTROL / "config/qp_ik_pico_shared_root.yaml")
    parser.add_argument("--annotations", type=Path)
    args = parser.parse_args()
    try:
        trace_hash, profile_hash = sha256(args.trace), sha256(args.profile)
        annotation_hash = sha256(args.annotations) if args.annotations else None
        result = subprocess.run([str(CONTROL / "build/tianji_shared_root_trace_audit"),
                                 str(args.profile), str(args.trace), "--mapping-frames"],
                                capture_output=True, text=True, check=True, timeout=60)
        frames, freshness, provenance = parse_native(result.stdout)
        if (provenance["trace_sha256"] != trace_hash or provenance["profile_sha256"] != profile_hash
                or sha256(args.trace) != trace_hash or sha256(args.profile) != profile_hash):
            raise ValueError("audit input fingerprint changed/mismatched")
        segments = []
        if args.annotations:
            segments = validate_annotations(json.loads(args.annotations.read_text()), trace_hash, frames[-1][0])
            if sha256(args.annotations) != annotation_hash:
                raise ValueError("annotations changed during audit")
        report = summarize(frames, freshness, segments)
        projection = [fields(line) for line in result.stdout.splitlines()
                      if line.startswith("reachable_projection ")]
        if projection:
            report["reachable_projection"] = projection[0]
            report["target_semantics"] = (
                "bounded_common_translation_approximation" if projection[0]["enabled"] == "1"
                else "original_affine_mapping")
        report.update(provenance=provenance, annotations_sha256=annotation_hash,
                      freshness_ns=freshness,
                      limitations=["requires native-supported ordered unique trace and valid morphology",
                                   "labels are supplied, not independently verified",
                                   "does not measure control recovery, QP accuracy or actual tracking",
                                   "no acceptance thresholds are inferred or applied"])
        print(json.dumps(report, indent=2, allow_nan=False))
    except (OSError, ValueError, KeyError, IndexError, TypeError, subprocess.SubprocessError) as error:
        parser.exit(2, f"{error}\n")


if __name__ == "__main__":
    main()
