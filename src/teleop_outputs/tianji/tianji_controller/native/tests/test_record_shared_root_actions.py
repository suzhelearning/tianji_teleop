"""Input-only capture lifecycle, real receive clock, and honest cue provenance."""
import argparse
import json
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import time
import zlib

import pytest

CONTROL = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CONTROL / "scripts"))
import record_shared_root_actions as recorder
from report_shared_root_actions import validate_annotations
from run_pico_trace_algorithm_benchmark import _read_trace
from tianji_runtime import native_executable

BINARY = native_executable("tianji_record_action_trace")


def packet(sequence=1):
    data = bytearray(656)
    struct.pack_into("<4sHHQQqqI", data, 0, b"TJVR", 4, 656, sequence, 1,
                     sequence * 10_000_000, time.monotonic_ns(), 0xcf)
    for offset in (44, 100):
        struct.pack_into("<7d", data, offset, 0, 0, 0, 1, 0, 0, 0)
    struct.pack_into("<32d", data, 396, *([1, 0, 0, 0] * 8))
    struct.pack_into("<I", data, 652, zlib.crc32(data[:652]))
    return bytes(data)


def reserve_port():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


def start_native(directory, port, *options):
    child = subprocess.Popen([str(BINARY), "--output", str(directory),
                              "--port", str(port), "--countdown", "0", *options],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    line = child.stdout.readline()
    assert json.loads(line)["type"] == "waiting"
    return child


def journal():
    plan = json.loads(subprocess.check_output([str(BINARY), "--describe"], text=True))
    return [{"type": "origin", "first_receive_monotonic_ns": 123456789}] + [
        dict(stage, type="prompt", receive_relative_ns=stage["scheduled_ns"] + 1234)
        for stage in plan["stages"]]


def test_candidates_use_actual_prompt_clock_and_are_not_annotations():
    doc = recorder.candidates(journal(), 49_999_000_000, "sha")
    assert doc["candidate_segments"][0]["start_ns"] == 5_000_001_234
    assert len(doc["candidate_segments"]) == 7
    assert doc["actions_confirmed"] is False
    with pytest.raises(ValueError, match="segments must"):
        validate_annotations(doc, "sha", 50_000_000_000)


def test_tail_not_invented_and_late_prompts_remain_uncovered():
    doc = recorder.candidates(journal(), 12_000_000_000, "sha")
    assert len(doc["candidate_segments"]) == 2
    assert doc["candidate_segments"][-1]["end_ns"] == 12_000_000_000


@pytest.mark.parametrize("mutation", ["missing", "clock", "action"])
def test_corrupt_journal_rejected(mutation):
    events = journal()
    if mutation == "missing":
        events.pop()
    elif mutation == "clock":
        events[3]["receive_relative_ns"] = 0
    else:
        events[2]["action"] = "guess"
    with pytest.raises(ValueError):
        recorder.candidates(events, 50_000_000_000, "sha")


def test_timeout_retains_partial_and_zero_count(tmp_path):
    child = start_native(tmp_path, reserve_port(), "--wait-seconds", "1")
    _, error = child.communicate(timeout=4)
    assert child.returncode == 2 and "timeout" in error
    assert struct.unpack("<4sHHQ", (tmp_path / "input.tjvr.partial").read_bytes()) == (b"TJVT", 1, 656, 0)
    assert not (tmp_path / "input.tjvr").exists()


def test_port_conflict_does_not_create_trace(tmp_path):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        result = subprocess.run([str(BINARY), "--output", str(tmp_path), "--port",
                                 str(sock.getsockname()[1])], capture_output=True, timeout=5)
    assert result.returncode == 2 and b"bind" in result.stderr
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize("data", [b"bad packet", packet()[:-1], bytes(656)])
def test_bad_datagram_preserved_without_finalization(tmp_path, data):
    port = reserve_port()
    child = start_native(tmp_path, port)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        sender.sendto(data, ("127.0.0.1", port))
    child.communicate(timeout=5)
    assert child.returncode == 2
    assert (tmp_path / "rejected.datagram").read_bytes() == data
    assert not (tmp_path / "input.tjvr").exists()


def test_signal_preserves_original_packet_and_incomplete_header(tmp_path):
    port = reserve_port()
    child = start_native(tmp_path, port)
    data = packet()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        sender.sendto(data, ("127.0.0.1", port))
    deadline = time.monotonic() + 3
    while (tmp_path / "input.tjvr.partial").stat().st_size < 680:
        assert time.monotonic() < deadline
        time.sleep(.01)
    child.send_signal(signal.SIGINT)
    _, error = child.communicate(timeout=5)
    raw = (tmp_path / "input.tjvr.partial").read_bytes()
    assert child.returncode == 2 and "interrupted" in error
    assert struct.unpack_from("<Q", raw, 8)[0] == 0
    assert struct.unpack_from("<Q", raw, 16)[0] == 0
    assert raw[24:] == data


def test_existing_file_is_never_overwritten(tmp_path):
    trace = tmp_path / "input.tjvr.partial"
    trace.write_bytes(b"keep")
    result = subprocess.run([str(BINARY), "--output", str(tmp_path), "--port", str(reserve_port())],
                            capture_output=True, timeout=5)
    assert result.returncode == 2 and trace.read_bytes() == b"keep"


def test_wrapper_failure_metadata(tmp_path):
    output = tmp_path / "failed"
    result = subprocess.run([sys.executable, str(Path(recorder.__file__)), "record",
                             "--output", str(output), "--source-kind", "synthetic",
                             "--port", str(reserve_port()), "--wait-seconds", "1"],
                            capture_output=True, text=True, timeout=10, cwd=tmp_path)
    assert result.returncode == 2, result.stderr
    assert json.loads((output / "capture-result.json").read_text())["complete"] is False
    assert (output / "input.tjvr.partial").exists()
    assert not (output / "actions-confirmed.json").exists()


def test_full_50_second_capture_and_explicit_review(tmp_path):
    """Real wall-time loopback integration; source is explicitly synthetic."""
    output = tmp_path / "complete"
    port = reserve_port()
    child = subprocess.Popen([sys.executable, str(Path(recorder.__file__)), "record",
                              "--output", output.name, "--source-kind", "synthetic",
                              "--port", str(port), "--countdown", "1"],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                             cwd=tmp_path)
    sent = {}
    try:
        deadline = time.monotonic() + 60
        while not (output / "events.jsonl").exists():
            assert child.poll() is None and time.monotonic() < deadline
            time.sleep(.01)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sequence = 0
            while child.poll() is None:
                assert time.monotonic() < deadline
                sequence += 1
                data = packet(sequence)
                sent[sequence] = data
                sender.sendto(data, ("127.0.0.1", port))
                time.sleep(.01)
        stdout, stderr = child.communicate(timeout=5)
        assert child.returncode == 0, (stdout, stderr)
    finally:
        if child.poll() is None:
            child.kill()
            child.wait()
    size, records = _read_trace(output / "input.tjvr")
    assert size == 656 and records[0][0] == 0 and records[-1][0] > 49_000_000_000
    assert all(raw == sent[struct.unpack_from("<Q", raw, 8)[0]] for _, raw, _ in records)
    meta = json.loads((output / "capture-result.json").read_text())
    assert meta["complete"] and not meta["actions_confirmed"] and not meta["motion_authorized"]
    assert meta["source_kind"] == "synthetic"
    assert json.loads((output / "actions-unconfirmed.json").read_text())["segments"] == []
    events = [json.loads(line) for line in (output / "events.jsonl").read_text().splitlines()]
    assert any(e["type"] == "countdown" for e in events)
    assert sum(e["type"] == "cue" for e in events) == 1
    assert max(e["receive_relative_ns"] - e["scheduled_ns"] for e in events if e["type"] == "prompt") < 200_000_000
    recorder.confirm(argparse.Namespace(session=output, reviewer="synthetic-test"))
    confirmed = json.loads((output / "actions-confirmed.json").read_text())
    assert len(validate_annotations(confirmed, meta["trace_sha256"], records[-1][0])) == 7
    assert not confirmed["phase_a_accepted"] and confirmed["source_kind"] == "synthetic"
    with pytest.raises(FileExistsError):
        recorder.confirm(argparse.Namespace(session=output, reviewer="synthetic-test"))
    (output / "events.jsonl").write_text("tampered")
    with pytest.raises(ValueError, match="integrity"):
        recorder.confirm(argparse.Namespace(session=output, reviewer="synthetic-test"))
