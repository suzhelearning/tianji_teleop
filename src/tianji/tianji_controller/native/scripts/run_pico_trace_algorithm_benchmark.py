#!/usr/bin/env python3
"""Run one canonical PICO TJVR trace through a deterministic algorithm matrix."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import time
from typing import Any, Iterable


PRIMARY_ALGORITHMS = (
    "spark_upper_qpoases_direct",
    "spark_upper_qpoases_velocity_qp",
    "spark_upper_qpoases_cartesian_otg_velocity_qp",
    "spark_upper_qpoases_feedforward_velocity_qp",
    "spark_upper_qpoases_headroom_feedforward_velocity_qp",
)
APPENDIX_ALGORITHMS = (
    "hierarchical_qp",
    "nullspace_dls",
    "spark_guided_velocity_qp",
    "spark_direct_velocity_qp",
    "spark_pose_velocity_qp",
)
ALL_ALGORITHMS = PRIMARY_ALGORITHMS + APPENDIX_ALGORITHMS
TRACE_HEADER = struct.Struct("<4sHHQ")
RECORD_TIME = struct.Struct("<q")


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_trace(path: Path) -> tuple[int, list[tuple[int, bytes, int]]]:
    with path.open("rb") as stream:
        header = stream.read(TRACE_HEADER.size)
        if len(header) != TRACE_HEADER.size:
            raise ValueError("truncated TJVR trace header")
        magic, version, packet_size, count = TRACE_HEADER.unpack(header)
        if magic != b"TJVT" or version != 1 or packet_size <= 0:
            raise ValueError("invalid TJVR trace header")
        records: list[tuple[int, bytes, int]] = []
        for _ in range(count):
            relative = stream.read(RECORD_TIME.size)
            packet = stream.read(packet_size)
            if len(relative) != RECORD_TIME.size or len(packet) != packet_size:
                raise ValueError("truncated TJVR trace record")
            if packet[:4] != b"TJVR":
                raise ValueError("invalid TJVR packet magic")
            source_ns = struct.unpack_from("<q", packet, 24)[0]
            records.append((RECORD_TIME.unpack(relative)[0], packet, source_ns))
        if stream.read(1):
            raise ValueError("trailing bytes in TJVR trace")
    return packet_size, records


def inspect_trace(path: Path) -> dict[str, Any]:
    packet_size, records = _read_trace(Path(path))
    if not records:
        raise ValueError("empty TJVR trace")
    duration = (records[-1][2] - records[0][2]) * 1.0e-9
    if duration < 0.0:
        raise ValueError("non-monotonic TJVR source timestamps")
    return {
        "path": str(Path(path).resolve()),
        "sha256": _file_sha256(Path(path)),
        "packet_size": packet_size,
        "frame_count": len(records),
        "source_duration_s": duration,
        "first_source_ns": records[0][2],
        "last_source_ns": records[-1][2],
    }


def slice_trace(source: Path, destination: Path, frame_count: int) -> None:
    packet_size, records = _read_trace(Path(source))
    if frame_count <= 0 or frame_count > len(records):
        raise ValueError("slice frame count is outside trace")
    selected = records[:frame_count]
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("wb") as stream:
        stream.write(TRACE_HEADER.pack(b"TJVT", 1, packet_size, len(selected)))
        for relative_ns, packet, _ in selected:
            stream.write(RECORD_TIME.pack(relative_ns))
            stream.write(packet)


def build_viewer_command(*, viewer: Path, config: Path, model: Path,
                         algorithm: str, port: int, duration_s: float,
                         telemetry: Path, joint_telemetry: Path) -> list[str]:
    return [
        str(viewer), "--config", str(config), "--model", str(model),
        "--pico-teleop", "--pico-bind", "127.0.0.1", "--pico-port", str(port),
        "--control-level", "velocity", "--algorithm", algorithm,
        "--model-state-only", "--headless", "--duration", f"{duration_s:.6f}",
        "--telemetry", str(telemetry), "--joint-telemetry", str(joint_telemetry),
    ]


def build_replay_command(*, python: Path, replay_tool: Path, trace: Path,
                         port: int, lead_s: float) -> list[str]:
    return [
        str(python), str(replay_tool), "--input", str(trace), "--host",
        "127.0.0.1", "--port", str(port), "--lead", f"{lead_s:.6f}",
    ]


def ensure_manifest_compatible(existing: dict[str, Any],
                               expected: dict[str, Any]) -> None:
    for key in ("schema_version", "provenance", "algorithms"):
        if existing.get(key) != expected.get(key):
            raise ValueError(f"manifest {key} mismatch; refusing incompatible resume")


def _last_csv_row(path: Path) -> dict[str, str]:
    with Path(path).open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"empty telemetry: {path}")
    return rows[-1]


def _int_alias(row: dict[str, str], *names: str) -> int:
    for name in names:
        if name in row and row[name] != "":
            return int(float(row[name]))
    raise ValueError(f"telemetry missing field: {'/'.join(names)}")


def validate_completed_run(telemetry: Path, algorithm: str,
                           expected_datagrams: int, *,
                           allow_control_failures: bool = False) -> dict[str, Any]:
    row = _last_csv_row(Path(telemetry))
    if row.get("algorithm") != algorithm:
        raise ValueError(
            f"algorithm mismatch: expected {algorithm}, got {row.get('algorithm')}"
        )
    values = {
        "pico_datagrams": _int_alias(row, "pico_datagrams"),
        "pico_malformed": _int_alias(
            row, "pico_malformed", "pico_malformed_packets"
        ),
        "pico_crc_failures": _int_alias(row, "pico_crc_failures"),
        "pico_reordered": _int_alias(row, "pico_reordered", "pico_reordered_packets"),
        "control_failures": _int_alias(row, "control_failures"),
    }
    if values["pico_datagrams"] != expected_datagrams:
        raise ValueError(
            f"pico_datagrams={values['pico_datagrams']} expected={expected_datagrams}"
        )
    required_zero = ["pico_malformed", "pico_crc_failures", "pico_reordered"]
    if not allow_control_failures:
        required_zero.append("control_failures")
    for name in required_zero:
        if values[name] != 0:
            raise ValueError(f"{name}={values[name]}")
    return values


def _atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    try:
        with os.fdopen(fd, "w") as stream:
            json.dump(value, stream, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def _git_commit(root: Path) -> str:
    return subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=root, text=True
    ).strip()


def _run_one(*, root: Path, algorithm: str, trace: Path,
             trace_metadata: dict[str, Any], output: Path, viewer: Path,
             replay_tool: Path, python: Path, config: Path, model: Path,
             port: int, lead_s: float, post_roll_s: float,
             allow_control_failures: bool) -> dict[str, Any]:
    telemetry_dir = output / "telemetry"
    log_dir = output / "logs"
    telemetry_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)
    cart = telemetry_dir / f"{algorithm}.csv"
    joint = telemetry_dir / f"{algorithm}_joints.csv"
    viewer_log = log_dir / f"{algorithm}_viewer.log"
    replay_log = log_dir / f"{algorithm}_replay.log"
    duration = trace_metadata["source_duration_s"] + lead_s + post_roll_s
    viewer_command = build_viewer_command(
        viewer=viewer, config=config, model=model, algorithm=algorithm, port=port,
        duration_s=duration, telemetry=cart, joint_telemetry=joint,
    )
    replay_command = build_replay_command(
        python=python, replay_tool=replay_tool, trace=trace, port=port, lead_s=lead_s,
    )
    started = time.time()
    with viewer_log.open("w") as viewer_stream, replay_log.open("w") as replay_stream:
        viewer_process = subprocess.Popen(
            viewer_command, cwd=root, stdout=viewer_stream,
            stderr=subprocess.STDOUT, start_new_session=True,
        )
        try:
            time.sleep(0.15)
            replay_result = subprocess.run(
                replay_command, cwd=root, stdout=replay_stream,
                stderr=subprocess.STDOUT, timeout=duration + 10.0,
            )
            if replay_result.returncode != 0:
                raise RuntimeError(f"replay failed for {algorithm}")
            viewer_code = viewer_process.wait(timeout=post_roll_s + 10.0)
            if viewer_code != 0 and not (allow_control_failures and viewer_code == 2):
                raise RuntimeError(f"viewer failed for {algorithm}: {viewer_code}")
        finally:
            if viewer_process.poll() is None:
                os.killpg(viewer_process.pid, signal.SIGINT)
                try:
                    viewer_process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    os.killpg(viewer_process.pid, signal.SIGKILL)
                    viewer_process.wait()
    if not joint.exists() or joint.stat().st_size == 0:
        raise ValueError(f"missing joint telemetry for {algorithm}")
    validation = validate_completed_run(
        cart, algorithm, trace_metadata["frame_count"],
        allow_control_failures=allow_control_failures,
    )
    return {
        "status": ("complete_with_failures"
                   if validation["control_failures"] else "complete"),
        "algorithm": algorithm,
        "elapsed_wall_s": time.time() - started,
        "telemetry": str(cart.relative_to(output)),
        "joint_telemetry": str(joint.relative_to(output)),
        "viewer_log": str(viewer_log.relative_to(output)),
        "replay_log": str(replay_log.relative_to(output)),
        "validation": validation,
    }


def _resolve_algorithms(value: str) -> tuple[str, ...]:
    if value == "primary":
        return PRIMARY_ALGORITHMS
    if value == "appendix":
        return APPENDIX_ALGORITHMS
    if value == "all":
        return ALL_ALGORITHMS
    selected = tuple(part.strip() for part in value.split(",") if part.strip())
    unknown = sorted(set(selected) - set(ALL_ALGORITHMS))
    if unknown:
        raise ValueError(f"unknown algorithms: {', '.join(unknown)}")
    return selected


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--viewer", type=Path, default=Path("build/tianji_qp_ik_viewer"))
    parser.add_argument(
        "--replay-tool", type=Path,
        default=Path(__file__).resolve().with_name("replay_pico_udp_trace.py"))
    parser.add_argument("--python", type=Path, default=Path(sys.executable))
    parser.add_argument("--config", type=Path, default=Path("config/qp_ik_pico_teleop.yaml"))
    parser.add_argument("--model", type=Path, default=Path("models/marvin_m6_qp_pico_fast.xml"))
    parser.add_argument("--algorithms", default="all")
    parser.add_argument("--base-port", type=int, default=15100)
    parser.add_argument("--lead", type=float, default=0.25)
    parser.add_argument("--post-roll", type=float, default=0.75)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--preflight-frames", type=int, default=0)
    parser.add_argument("--continue-on-control-failure", action="store_true")
    arguments = parser.parse_args(argv)

    root = Path(__file__).resolve().parents[1]
    trace = arguments.trace.resolve()
    output = arguments.output.resolve()
    algorithms = _resolve_algorithms(arguments.algorithms)
    output.mkdir(parents=True, exist_ok=True)
    run_trace = trace
    if arguments.preflight_frames:
        run_trace = output / "preflight_trace.tjvr"
        slice_trace(trace, run_trace, arguments.preflight_frames)
    metadata = inspect_trace(run_trace)
    provenance = {
        "source_trace": inspect_trace(trace),
        "run_trace": metadata,
        "git_commit": _git_commit(root),
        "viewer_sha256": _file_sha256((root / arguments.viewer).resolve()),
        "config_sha256": _file_sha256((root / arguments.config).resolve()),
        "model_sha256": _file_sha256((root / arguments.model).resolve()),
        "replay_tool_sha256": _file_sha256(arguments.replay_tool.resolve()),
        "lead_s": arguments.lead,
        "post_roll_s": arguments.post_roll,
        "continue_on_control_failure": arguments.continue_on_control_failure,
    }
    expected = {
        "schema_version": 1, "provenance": provenance,
        "algorithms": list(algorithms), "runs": {},
    }
    manifest_path = output / "manifest.json"
    if manifest_path.exists():
        if not arguments.resume:
            raise ValueError(f"output manifest already exists: {manifest_path}")
        manifest = json.loads(manifest_path.read_text())
        ensure_manifest_compatible(manifest, expected)
    else:
        manifest = expected
        _atomic_json(manifest_path, manifest)

    for index, algorithm in enumerate(algorithms):
        previous = manifest["runs"].get(algorithm, {})
        if previous.get("status") in ("complete", "complete_with_failures"):
            validate_completed_run(
                output / previous["telemetry"], algorithm, metadata["frame_count"],
                allow_control_failures=(
                    previous.get("status") == "complete_with_failures"
                ),
            )
            print(f"skip_complete algorithm={algorithm}", flush=True)
            continue
        print(f"run_start algorithm={algorithm}", flush=True)
        try:
            result = _run_one(
                root=root, algorithm=algorithm, trace=run_trace,
                trace_metadata=metadata, output=output,
                viewer=(root / arguments.viewer).resolve(),
                replay_tool=arguments.replay_tool.resolve(), python=arguments.python,
                config=(root / arguments.config).resolve(),
                model=(root / arguments.model).resolve(),
                port=arguments.base_port + index, lead_s=arguments.lead,
                post_roll_s=arguments.post_roll,
                allow_control_failures=arguments.continue_on_control_failure,
            )
        except BaseException as error:
            manifest["runs"][algorithm] = {
                "status": "failed", "error": f"{type(error).__name__}: {error}"
            }
            _atomic_json(manifest_path, manifest)
            raise
        manifest["runs"][algorithm] = result
        _atomic_json(manifest_path, manifest)
        print(f"run_complete algorithm={algorithm}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
