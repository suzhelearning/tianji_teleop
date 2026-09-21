#!/usr/bin/env python3
"""Manage a 50 s input-only native TJVR capture and unconfirmed action cues."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from report_shared_root_actions import ACTIONS, sha256, validate_annotations
from run_pico_trace_algorithm_benchmark import _read_trace
from tianji_runtime import workspace

CONTROL = Path(__file__).resolve().parents[1]
ROOT = workspace()
BINARY = CONTROL / "build/tianji_record_action_trace"


def save_new(path, document):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(document, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def candidates(events, duration, trace_hash):
    origins = [e for e in events if e["type"] == "origin"]
    prompts = [e for e in events if e["type"] == "prompt"]
    expected = ["unannotated", *ACTIONS, "unannotated"]
    if (len(origins) != 1 or [e["action"] for e in prompts] != expected
            or [e["scheduled_ns"] for e in prompts] !=
            [n * 10**9 for n in (0, 5, 11, 17, 23, 29, 35, 41, 47)]):
        raise ValueError("incomplete or inconsistent prompt journal")
    stamps = [e["receive_relative_ns"] for e in prompts]
    if any(type(t) is not int or t < 0 for t in stamps) or any(
            b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError("non-monotonic prompt journal")
    segments = []
    for event, end in zip(prompts, stamps[1:]):
        start = event["receive_relative_ns"]
        if event["action"] in ACTIONS and start < min(end, duration):
            segments.append(dict(action=event["action"], start_ns=start,
                                 end_ns=min(end, duration)))
    document = dict(schema_version=1, trace_sha256=trace_hash,
                    time_basis="nanoseconds_since_first_receive", segments=segments)
    validate_annotations(document, trace_hash, duration)
    # Deliberately not an annotations document: feeding this to the report fails.
    document["candidate_segments"] = document.pop("segments")
    document.update(actions_confirmed=False, evidence="prompt_emission_only",
                    explanation="提示发出时间不证明实际动作；须由佩戴者/观察者确认或修正。")
    return document


def snapshot_sources(output, calibration):
    paths = [CONTROL / "config" / name for name in (
        "shared_root_tjvr_input_contract.yaml", "shared_root_robot_geometry.yaml",
        "qp_ik_pico_shared_root.yaml")]
    paths += [BINARY, Path(__file__).resolve(), CONTROL / "apps/record_action_trace.cpp"]
    # Resolve the input contract's file list using the project's YAML dependency.
    import yaml
    contract = yaml.safe_load(paths[0].read_text())
    paths += [ROOT / name for name in contract["tjvr_shared_root_input"]["source_files"]]
    if calibration:
        paths += sorted(p for p in calibration.iterdir()
                        if p.is_file() and p.suffix in (".json", ".yaml", ".yml"))
        if not all((calibration / f"pico_{side}_{kind}.yaml").is_file()
                   for side in ("left", "right")
                   for kind in ("arm_geometry", "palm_tcp", "wrist_pivot")):
            raise ValueError("calibration directory lacks bilateral geometry/TCP/wrist files")
    snapshots = output / "snapshot"
    snapshots.mkdir()
    hashes = {}
    for index, path in enumerate(dict.fromkeys(p.resolve() for p in paths)):
        digest = sha256(path)
        hashes[str(path)] = digest
        if path != BINARY.resolve():
            with path.open("rb") as source, (snapshots / f"{index:02d}_{path.name}").open("xb") as dest:
                dest.write(source.read())
                dest.flush()
                os.fsync(dest.fileno())
            if sha256(snapshots / f"{index:02d}_{path.name}") != digest:
                raise ValueError("source changed during snapshot")
    return hashes


def capture(args):
    if args.source_kind == "live" and (not args.participant or not args.calibration_dir):
        raise ValueError("live capture requires --participant and --calibration-dir")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    meta = dict(schema_version=1, complete=False, duration_requested_s=50,
                source_kind=args.source_kind, participant=args.participant,
                calibration_directory=str(args.calibration_dir.resolve()) if args.calibration_dir else None,
                started_unix_ns=time.time_ns(), motion_authorized=False,
                phase_a_accepted=False, thresholds_frozen=False, actions_confirmed=False)
    child = None
    try:
        hashes = snapshot_sources(output, args.calibration_dir)
        meta["source_sha256"] = hashes
        save_new(output / "capture-start.json", meta)
        command = [str(BINARY), "--output", str(output), "--port", str(args.port),
                   "--countdown", str(args.countdown), "--wait-seconds", str(args.wait_seconds)]
        print(f"录制目录：{output}\n仅采集输入；请由观察者读出提示，或在主机终端查看。", flush=True)
        with (output / "native.stderr.log").open("x") as errors:
            child = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=errors,
                                     text=True, encoding="utf-8", start_new_session=True)
            for line in child.stdout:
                event = json.loads(line)
                kind = event["type"]
                if kind == "waiting":
                    print(f"等待 TJVR 输入，127.0.0.1:{args.port} …", flush=True)
                elif kind == "countdown":
                    print(f"准备：{event['remaining_s']} 秒", flush=True)
                elif kind in ("prompt", "cue"):
                    print(f"\a[{event['receive_relative_ns'] / 1e9:5.2f}s] {event['prompt']}", flush=True)
                elif kind == "finished":
                    print("50 秒采集结束，正在校验并保存。", flush=True)
            result = child.wait()
        if result:
            raise RuntimeError(f"native collector exited {result}; see native.stderr.log")
        partial = output / "input.tjvr.partial"
        size, records = _read_trace(partial)
        events = [json.loads(line) for line in (output / "events.jsonl").read_text().splitlines()]
        if (size != 656 or not records or records[0][0] != 0
                or any(b[0] < a[0] for a, b in zip(records, records[1:]))
                or events[-1]["type"] != "finished"
                or events[-1]["frames"] != len(records)
                or events[-1]["last_receive_relative_ns"] != records[-1][0]
                or events[-1]["stop_relative_ns"] < 50 * 10**9):
            raise ValueError("native finalization/trace mismatch")
        if any(sha256(path) != digest for path, digest in hashes.items()):
            raise ValueError("source/config/binary changed during capture")
        digest = sha256(partial)
        candidate = candidates(events, records[-1][0], digest)
        candidate["events_sha256"] = sha256(output / "events.jsonl")
        save_new(output / "action-candidates.json", candidate)
        save_new(output / "actions-unconfirmed.json", dict(
            schema_version=1, trace_sha256=digest,
            time_basis="nanoseconds_since_first_receive", segments=[]))
        # Exclusive publication: never replace an existing recording.
        os.link(partial, output / "input.tjvr")
        # Keep the partial hard link as recovery evidence, including on later failure.
        directory = os.open(output, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
        meta.update(complete=True, frames=len(records), trace_sha256=digest,
                    events_sha256=candidate["events_sha256"],
                    candidates_sha256=sha256(output / "action-candidates.json"),
                    last_receive_relative_ns=records[-1][0],
                    unobserved_tail_ns=max(0, 50 * 10**9 - records[-1][0]))
    except BaseException as error:
        if child is not None and child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        meta.update(complete=False, error=str(error) or type(error).__name__)
        raise
    finally:
        meta["finished_unix_ns"] = time.time_ns()
        save_new(output / "capture-result.json", meta)
    print(f"已保存 {meta['frames']} 帧；动作待确认，未授予启用或运动权限。", flush=True)


def confirm(args):
    output = args.session.resolve()
    meta = json.loads((output / "capture-result.json").read_text())
    candidate = json.loads((output / "action-candidates.json").read_text())
    digest = sha256(output / "input.tjvr")
    if (meta.get("complete") is not True or digest != meta["trace_sha256"]
            or sha256(output / "events.jsonl") != meta["events_sha256"]
            or sha256(output / "action-candidates.json") != meta["candidates_sha256"]):
        raise ValueError("capture/candidate integrity check failed")
    document = dict(schema_version=1, trace_sha256=digest,
                    time_basis="nanoseconds_since_first_receive",
                    segments=candidate["candidate_segments"],
                    reviewer=args.reviewer, confirmed_unix_ns=time.time_ns(),
                    confirmation="reviewer confirms actions matched prompt intervals",
                    source_kind=meta["source_kind"], phase_a_accepted=False,
                    motion_authorized=False)
    validate_annotations(document, digest, meta["last_receive_relative_ns"])
    save_new(output / "actions-confirmed.json", document)
    print(output / "actions-confirmed.json")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("plan", help="show schedule without binding or writing files")
    record = commands.add_parser("record")
    record.add_argument("--output", type=Path, required=True, help="new, non-existing directory")
    record.add_argument("--participant")
    record.add_argument("--calibration-dir", type=Path)
    record.add_argument("--source-kind", choices=("live", "synthetic"), default="live")
    record.add_argument("--port", type=int, default=15000)
    record.add_argument("--countdown", type=int, default=5)
    record.add_argument("--wait-seconds", type=int, default=30)
    review = commands.add_parser("confirm-prompts", help="explicitly attest actual actions matched cues")
    review.add_argument("--session", type=Path, required=True)
    review.add_argument("--reviewer", required=True)
    args = parser.parse_args()
    if args.command == "plan":
        plan = json.loads(subprocess.check_output([str(BINARY), "--describe"], text=True))
        for stage in plan["stages"]:
            print(f"{stage['scheduled_ns'] // 10**9:2d}s  {stage['prompt']}")
            if stage["action"] == "single_hand":
                print("32s  交换静止手与运动手")
        print("50s  结束。提示时间只是待确认区间。")
    elif args.command == "confirm-prompts":
        confirm(args)
    else:
        if not 1 <= args.port <= 65535 or not 0 <= args.countdown <= 60 or not 1 <= args.wait_seconds <= 3600:
            parser.error("invalid port/countdown/wait-seconds")
        def interrupted(_signal, _frame):
            raise KeyboardInterrupt("capture interrupted")
        signal.signal(signal.SIGTERM, interrupted)
        capture(args)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, KeyboardInterrupt) as error:
        print(f"采集/确认未完成：{error}", file=sys.stderr)
        raise SystemExit(2)
