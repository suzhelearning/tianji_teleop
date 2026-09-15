"""Per-run session and flight-recorder logs for the real executor.

The launcher enables logging by exporting an absolute ``TIANJI_RUN_LOG_DIR``;
a direct ``run_teleop.py`` invocation without that variable keeps its original
behavior and writes nothing. ``session.json`` holds the run state (config
source and content, selected devices, mode, timing, final outcome and cleanup
errors) and ``flight_recorder.jsonl`` holds the most recent control samples
with the newest sample last:

    input reference (controller packet) vs last successfully sent setpoint vs
    measured feedback, keyed by sample

Sampling is an in-memory append only: the control loop never touches the disk,
never serializes and never copies joint values (frames and feedback objects are
immutable snapshots, so references are stored). Files are written when the
session ends, after the hardware and controller have been stopped.
"""
from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import time

LOG_DIR_ENV = "TIANJI_RUN_LOG_DIR"
SESSION_FILE = "session.json"
FLIGHT_RECORDER_FILE = "flight_recorder.jsonl"
CONTROLLER_CONFIG_FILE = "controller_configuration.yaml"
SCHEMA = 1
# The ring keeps the last WINDOW_NS of samples and never more than CAPACITY.
WINDOW_NS = 10_000_000_000
CAPACITY = 2000


def _wall_clock() -> str:
    return datetime.now(timezone.utc).isoformat()


def _json_safe(value):
    """Keep session.json writable even if fault details carry exotic values (degraded to repr)."""
    try:
        return json.loads(json.dumps(value, allow_nan=False))
    except (TypeError, ValueError):
        return repr(value)


def _atomic_write(path: Path, text: str) -> None:
    """Replace path with text, so a crash never leaves a half-written record."""
    temporary = path.with_name(path.name + ".tmp")
    try:
        temporary.write_text(text, encoding="utf-8")
        os.replace(temporary, path)
    except BaseException:
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            pass
        raise


class FlightRecorder:
    """Bounded in-memory ring of recent control samples; no disk work while running."""

    def __init__(self, devices, window_ns=WINDOW_NS, capacity=CAPACITY):
        self.devices = tuple(devices)
        self.window_ns = int(window_ns)
        self.samples = deque(maxlen=int(capacity))
        self._last_sent = {}

    def record_send(self, device, positions) -> None:
        """Record a setpoint only after the device accepted it; gate output is not a send."""
        self._last_sent[device] = (positions, time.monotonic_ns())

    def append(self, frame, measured, phase, now_ns) -> None:
        """Append one sample; frames and feedback are immutable, so they are stored by reference."""
        self.samples.append({
            "monotonic_ns": now_ns,
            "phase": phase,
            "frame": frame,
            "feedback": measured,
            "sent": dict(self._last_sent),
        })
        # The newest sample (the one a failing check just examined) is never evicted.
        horizon = now_ns - self.window_ns
        while self.samples and self.samples[0]["monotonic_ns"] < horizon:
            self.samples.popleft()

    def write(self, path) -> None:
        """Serialize every retained sample as one readable JSON object per line."""
        lines = [json.dumps(self._serialize(sample), allow_nan=False) for sample in self.samples]
        _atomic_write(Path(path), "".join(line + "\n" for line in lines))

    def _serialize(self, sample) -> dict:
        frame = sample["frame"]
        packet = None
        reference = {}
        if frame is not None:
            packet = {"sequence": frame.sequence, "flags": frame.flags,
                      "tracking_epoch": frame.tracking_epoch, "timestamp_ns": frame.timestamp_ns}
            reference = {device: list(frame.positions(device)) for device in self.devices}
        return {
            "monotonic_ns": sample["monotonic_ns"],
            "phase": sample["phase"],
            "packet": packet,
            "reference": reference,
            "sent": {device: {"positions": list(positions), "monotonic_ns": stamp}
                     for device, (positions, stamp) in sample["sent"].items()},
            "feedback": {device: {"position_rad": list(value.position_rad),
                                  "received_monotonic_ns": value.received_monotonic_ns,
                                  "healthy": bool(value.healthy),
                                  "enabled": bool(value.enabled),
                                  "detail": value.detail}
                         for device, value in (sample["feedback"] or {}).items()},
        }


class SessionLog:
    """One launcher-provided log directory plus the run state written into it."""

    def __init__(self, directory, *, devices, mode, config_source, config):
        directory = Path(directory)
        if not directory.is_absolute():
            raise ValueError(f"{LOG_DIR_ENV} must be an absolute path, got {directory}")
        # Creating the directory and writing the initial state proves the run is
        # recordable before any hardware is created or connected.
        directory.mkdir(parents=True, exist_ok=True)
        self.directory = directory
        self.session_path = directory / SESSION_FILE
        self.flight_path = directory / FLIGHT_RECORDER_FILE
        self.recorder = FlightRecorder(devices)
        self.started_monotonic_ns = time.monotonic_ns()
        self._session = {
            "schema": SCHEMA,
            "outcome": "running",
            "reason": None,
            "result": None,
            "error_details": None,
            "cleanup_errors": [],
            "write_errors": [],
            "mode": mode,
            "devices": list(devices),
            "config_source": str(config_source),
            "config": config,
            "log_directory": str(directory),
            "pid": os.getpid(),
            "python": {"version": sys.version, "executable": sys.executable,
                       "platform": " ".join(os.uname())},
            "started_at": _wall_clock(),
            "started_monotonic_ns": self.started_monotonic_ns,
            "controller_configuration": None,
            "flight_recorder": {"file": FLIGHT_RECORDER_FILE, "window_ns": self.recorder.window_ns,
                                "capacity": self.recorder.samples.maxlen},
        }
        self._flush_session()

    @classmethod
    def from_environment(cls, *, devices, mode, config_source, config):
        """Return a session log when the launcher exported TIANJI_RUN_LOG_DIR, else None."""
        directory = os.environ.get(LOG_DIR_ENV)
        if not directory:
            return None
        return cls(Path(directory), devices=devices, mode=mode,
                   config_source=config_source, config=config)

    def persist_controller_configuration(self, path, *, source) -> None:
        """Keep the generated controller YAML next to the session before the controller starts."""
        content = Path(path).read_text(encoding="utf-8")
        _atomic_write(self.directory / CONTROLLER_CONFIG_FILE, content)
        self._session["controller_configuration"] = {
            "file": CONTROLLER_CONFIG_FILE, "source_template": str(source)}
        self._flush_session()

    def sample(self, frame, measured, phase, now_ns) -> None:
        self.recorder.append(frame, measured, phase, now_ns)

    def record_send(self, device, positions) -> None:
        self.recorder.record_send(device, positions)

    def finish(self, *, outcome, reason, result, cleanup_errors=(), error_details=None) -> None:
        """Write the flight recorder, then the final session state.

        ``error_details`` carries the structured diagnostics a fault attached to
        itself (``getattr(error, "details", None)``); ``reason`` keeps the
        original error prefix. A failed flight-recorder write is recorded in the
        session and re-raised after it, so the caller can report it without
        losing the record.
        """
        errors = list(cleanup_errors)
        write_error = None
        try:
            self.recorder.write(self.flight_path)
        except (OSError, TypeError, ValueError) as error:
            write_error = f"{FLIGHT_RECORDER_FILE}: {error}"
        finished_monotonic_ns = time.monotonic_ns()
        self._session.update({
            "outcome": "failed" if write_error and outcome == "completed" else outcome,
            "reason": f"{reason}; {write_error}" if write_error else reason,
            "result": result or int(write_error is not None),
            "error_details": None if error_details is None else _json_safe(error_details),
            "cleanup_errors": errors,
            "write_errors": [] if write_error is None else [write_error],
            "finished_at": _wall_clock(),
            "finished_monotonic_ns": finished_monotonic_ns,
            "duration_s": (finished_monotonic_ns - self.started_monotonic_ns) / 1e9,
            "flight_recorder": {"file": FLIGHT_RECORDER_FILE, "samples": len(self.recorder.samples),
                                "window_ns": self.recorder.window_ns,
                                "capacity": self.recorder.samples.maxlen},
        })
        self._flush_session()
        if write_error:
            raise OSError(write_error)

    def _flush_session(self) -> None:
        _atomic_write(self.session_path, json.dumps(self._session, indent=2, allow_nan=False) + "\n")
