"""R/S/D episode lifecycle: real HDF5 files, stubbed acquisition and cameras.

The session owns episode state and writer lifetime only. These tests drive it
through the same command surface the operator keyboard uses and assert on the
files that result, so the "save publishes a success file / discard and abort
keep it partial" contract is verified against real bytes rather than fakes.
"""

from __future__ import annotations

import json
import sys
import threading
import time
from pathlib import Path

import h5py
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from data_collector import runtime as runtime_module  # noqa: E402
from data_collector.dataset import DATASET_CONFIG_NAME  # noqa: E402
from data_collector.session import CollectionSession  # noqa: E402
from tianji_runtime.constants import (  # noqa: E402
    ARMS_COUNT,
    CAMERA_FPS,
    HAND_COUNT,
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
)

COLLECTION_CONFIG = {
    "robot_config": "tianji_wuji2_v1",
    "state_rate_hz": 120,
    "cameras": {"top": "111", "left_wrist": "222", "right_wrist": "333"},
}


class StubMonitor:
    """Camera readiness without a driver."""

    def __init__(self, ready=True, detail=""):
        self._ready = ready
        self._detail = detail

    def status(self):
        return self._ready, self._detail


class StubRuntime:
    """Acquisition stand-in that writes through the real writer slot.

    Records what begin/end did so the session's ordering can be asserted
    independently of the acquisition implementation.
    """

    instances: list["StubRuntime"] = []

    def __init__(self, writer, *, boot_id, cameras, **kwargs):
        self.writer = writer
        self.cameras = cameras
        self.boot_id = boot_id
        self.events: list[str] = []
        self.recording = False
        self.episode_started = False
        self._missing: list[str] = []
        self._episode_start_ns = 0
        self.fault = None
        StubRuntime.instances.append(self)

    def start(self):
        self.events.append("start")
        return self

    def request_stop(self):
        self.events.append("request_stop")

    def close(self):
        self.events.append("close")
        try:
            self.writer.stop()
        except Exception:  # noqa: BLE001 - mirrors the real close contract
            pass

    def begin_episode(self, now_ns):
        self.writer.start(now_ns)
        self._episode_start_ns = now_ns
        self.recording = True
        self.episode_started = True
        self.events.append("begin_episode")

    def end_episode(self):
        if not self.recording:
            return False
        self.recording = False
        self.events.append("end_episode")
        self.writer.stop()
        return True

    def check(self):
        if self.fault:
            raise RuntimeError(self.fault)
        self.writer.check()

    def check_episode(self):
        pass

    def missing_inputs(self):
        return list(self._missing)

    def publish(self, start_ns: int | None = None, count: int = 1):
        """Emit samples stamped relative to the episode's own origin.

        The writer rejects samples older than the episode start (they would be
        pre-episode caches), so timestamps must be derived from it rather than
        from an arbitrary baseline.
        """
        base = self._episode_start_ns if start_ns is None else start_ns
        self.write_samples(base, count)

    def write_samples(self, start_ns: int, count: int = 4):
        """Emit a few complete, timestamp-ordered samples into the writer."""
        for index in range(count):
            stamp = start_ns + index * 1_000_000
            arms = np.linspace(0.0, 0.1, ARMS_COUNT, dtype=np.float32) + index * 0.001
            # The stored hand vector is the two sides combined: 2 * HAND_COUNT.
            left = np.linspace(0.0, 0.2, HAND_COUNT, dtype=np.float32) + index * 0.001
            right = np.linspace(0.0, -0.2, HAND_COUNT, dtype=np.float32) + index * 0.001
            hands = np.concatenate([left, right])
            self.writer.append_arms(stamp, tuple(float(v) for v in arms))
            self.writer.append_hands(stamp, tuple(float(v) for v in hands))
            for role in self.cameras:
                rgb = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 3), dtype=np.uint8)
                rgb[:, :, 0] = index
                self.writer.append_rgb(role, stamp, rgb)


@pytest.fixture(autouse=True)
def stub_runtime(monkeypatch):
    # The session imports CollectionRuntime from data_collector.runtime inside
    # start(), so that is the name that has to be replaced.
    StubRuntime.instances.clear()
    monkeypatch.setattr(runtime_module, "CollectionRuntime", StubRuntime, raising=False)
    return StubRuntime


@pytest.fixture()
def session(tmp_path):
    config_path = tmp_path / "collect.json"
    config_path.write_text(json.dumps(COLLECTION_CONFIG))
    model_path = _write_model(tmp_path / "model.xml")
    dataset = tmp_path / "raw"

    created = CollectionSession(dataset, "pick-place", config_path, model_path,
                                camera_monitor=StubMonitor())
    created.start(boot_id="boot-1")
    yield created
    created.finish()


def _write_model(path: Path) -> Path:
    """A chain of named hinge joints; only the joint names are read from it.

    One joint per body: MuJoCo allows at most six DOFs per body. Each moving
    body carries an explicit mass and inertia because MuJoCo refuses massless
    moving bodies, and these carry no collision geometry to infer them from.
    """
    # The names the controller's own grouping recognizes: arms are
    # Joint1..7_L/Joint1..7_R, hands are l_* and r_*.
    joint_names = ([f"Joint{n}_{side}" for side in ("L", "R") for n in range(1, 8)]
                   + [f"l_{index}" for index in range(HAND_COUNT)]
                   + [f"r_{index}" for index in range(HAND_COUNT)])
    lines = ['<mujoco model="test">', '  <compiler angle="radian"/>', '  <worldbody>']
    indent = "  "
    for depth, name in enumerate(joint_names):
        lines.append(f'{indent}<body name="link_{depth}" pos="0 0 {0.05 * (depth + 1):.3f}">')
        lines.append(f'{indent}  <inertial pos="0 0 0" mass="0.01" diaginertia="1e-5 1e-5 1e-5"/>')
        lines.append(f'{indent}  <joint name="{name}" type="hinge" axis="1 0 0"/>')
        indent += "  "
    for depth in reversed(range(len(joint_names))):
        lines.append("  " * (depth + 2) + "</body>")
    lines += ['  </worldbody>', '</mujoco>']
    path.write_text("\n".join(lines) + "\n")
    return path


def _wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def _wait_for_recording(session, timeout=5.0):
    """Wait until the episode is recording *and* its writer is attached.

    `state` flips to RECORDING in the manager thread, and the writer is attached
    just before that; asserting on both keeps a test from publishing into a slot
    that is not wired yet.
    """
    if not _wait_for(lambda: session.state == "RECORDING"
                     and session.current_writer() is not None, timeout):
        return False
    return True


def _episodes(dataset: Path):
    return sorted(dataset.glob("**/*.h5"))


def test_startup_prepares_the_writer_before_any_recording(session):
    # A prepared writer proves the directory, config and HDF5 setup work before
    # an operator presses R.
    assert session.prepared
    ready, detail = session.check_ready()
    assert ready, detail


def test_startup_requires_the_cameras_to_be_ready(tmp_path):
    config_path = tmp_path / "collect.json"
    config_path.write_text(json.dumps(COLLECTION_CONFIG))
    model_path = _write_model(tmp_path / "model.xml")
    created = CollectionSession(tmp_path / "raw", "task", config_path, model_path,
                                camera_monitor=StubMonitor(False, "top: no frame"))
    try:
        ready, detail = created.check_ready()
        assert not ready
        assert "top: no frame" in detail
    finally:
        created.finish()


def test_save_publishes_a_success_episode(session):
    session.command("r", "TELEOP")
    assert _wait_for_recording(session), session.state
    # An empty episode is refused by the schema validator, so a save has to be
    # backed by real samples; the session freezes the segment itself on 's'.
    StubRuntime.instances[-1].publish(count=3)
    session.command("s", "TELEOP")
    assert _wait_for(lambda: session.state == "IDLE"), session.state

    assert session.saved_paths, "saving must produce a success file"
    published = Path(session.saved_paths[-1])
    assert published.is_file()
    assert not published.name.endswith(".partial.h5")
    # The completed episode is the only .h5: the partial was published in place.
    complete = [p for p in _episodes(session.dataset_dir) if not p.name.endswith(".partial.h5")]
    assert complete == [published]


def test_discard_removes_only_the_current_segment(session):
    session.command("r", "TELEOP")
    assert _wait_for_recording(session)
    StubRuntime.instances[-1].publish(count=2)
    session.command("d", "TELEOP")
    assert _wait_for(lambda: session.state == "IDLE")
    assert not session.saved_paths
    assert _episodes(session.dataset_dir) == []


def test_unsaved_segment_is_kept_as_a_partial_file(session):
    session.command("r", "TELEOP")
    assert _wait_for_recording(session)
    runtime = StubRuntime.instances[-1]
    runtime.write_samples(1_000_000_000)
    session.end_episode()
    assert _wait_for(lambda: session.state in ("IDLE", "FAILED"))
    partials = [p for p in _episodes(session.dataset_dir) if p.name.endswith(".partial.h5")]
    assert partials, "an unsaved segment must survive as a partial file"
    assert not session.saved_paths


def test_command_outside_teleop_is_ignored(session):
    session.command("r", "READY")
    assert session.state == "IDLE"
    assert not session.saved_paths


def test_repeated_start_in_teleop_does_not_open_a_second_episode(session):
    session.command("r", "TELEOP")
    assert _wait_for_recording(session)
    session.command("r", "TELEOP")
    assert session.state == "RECORDING"
    assert len(StubRuntime.instances) == 1




def test_finish_is_idempotent(session):
    session.finish()
    session.finish()
    assert session.state in ("CLOSED", "FAILED")


# ---------------------------------------------------------------------------
# Writer ownership and concurrency. These are the cases where a naive
# implementation loses an episode or blocks the control loop, so they are
# asserted against the real writer and real files.
# ---------------------------------------------------------------------------


def test_record_save_discard_and_exit_are_independent_segments(session):
    from data_collector.dataset import EpisodeWriter

    runtime = StubRuntime.instances[-1]

    # A segment that is saved becomes a success file with the schema-v1 shape.
    session.command("r", "TELEOP")
    assert _wait_for_recording(session)
    runtime.publish()
    session.command("s", "TELEOP")
    assert _wait_for(lambda: session.state == "IDLE")
    saved = Path(session.saved_paths[0])
    with h5py.File(saved) as episode:
        assert set(episode) == {"observations", "images"}
        # Observations only: no derived action is ever stored.
        assert "action_type" not in episode.attrs
        assert bool(episode.attrs["success"])
        assert episode["observations/arms/qpos"].shape == (1, ARMS_COUNT)
        assert episode["observations/hands/qpos"].shape == (1, 2 * HAND_COUNT)
        assert set(episode["images"]) == {"top", "left_wrist", "right_wrist"}
        assert episode["images/right_wrist/rgb"].shape == (1, IMAGE_HEIGHT, IMAGE_WIDTH, 3)

    # A discarded segment leaves no file and never touches the saved one.
    session.command("r", "TELEOP")
    assert _wait_for_recording(session)
    runtime.publish()
    session.command("d", "TELEOP")
    assert _wait_for(lambda: session.state == "IDLE")
    assert saved.exists()
    assert not list(session.dataset_dir.rglob("*.partial.h5"))

    # Discarding with no active segment must not delete anything.
    session.command("d", "TELEOP")
    assert saved.exists()


def test_exit_without_saving_keeps_a_partial_episode(session):
    runtime = StubRuntime.instances[-1]
    session.command("r", "TELEOP")
    assert _wait_for_recording(session)
    runtime.publish()
    session.finish()  # no S pressed
    partials = list(session.dataset_dir.rglob("*.partial.h5"))
    assert len(partials) == 1
    with h5py.File(partials[0]) as episode:
        # Never silently marked as a successful capture.
        assert "success" not in episode.attrs
    assert session.saved_paths == []


def test_save_returns_before_the_background_writer_finishes(session, monkeypatch):
    from data_collector.dataset import EpisodeWriter

    entered, release = threading.Event(), threading.Event()
    original = EpisodeWriter.finish

    def delayed_finish(writer, success):
        entered.set()
        if not release.wait(5):
            raise RuntimeError("test did not release the writer")
        return original(writer, success)

    monkeypatch.setattr(EpisodeWriter, "finish", delayed_finish)
    session.command("r", "TELEOP")
    assert _wait_for_recording(session)
    StubRuntime.instances[-1].publish(count=2)
    caller = threading.Thread(target=session.command, args=("s", "TELEOP"))
    try:
        started = time.monotonic()
        caller.start()
        caller.join(0.5)
        assert not caller.is_alive(), "S blocked the caller on disk I/O"
        assert time.monotonic() - started < 0.5
        assert entered.wait(2)
        assert session.state == "SAVING"
        session.check()  # the control loop keeps checking while HDF5 finalizes
    finally:
        release.set()
        caller.join(2)
    assert _wait_for(lambda: session.state == "IDLE")
    assert len(session.saved_paths) == 1


def test_capture_fault_preserves_a_partial_without_raising(session):
    session.command("r", "TELEOP")
    assert _wait_for_recording(session)
    runtime = StubRuntime.instances[-1]
    runtime.publish()
    # A capture fault is reported through check(); the caller is a control loop
    # that must not receive an exception from the collector.
    runtime.fault = "camera disconnected"
    runtime.missing_inputs = lambda: ["camera top: no image"]
    session.check()
    session.end_episode()
    assert _wait_for(lambda: session.state in ("IDLE", "FAILED"))
    assert session.saved_paths == []
    assert len(list(session.dataset_dir.rglob("*.partial.h5"))) == 1


def test_every_lifecycle_transition_reaches_the_observer(tmp_path):
    config_path = tmp_path / "collect.json"
    config_path.write_text(json.dumps(COLLECTION_CONFIG))
    transitions = []
    created = CollectionSession(
        tmp_path / "raw", "transitions", config_path, _write_model(tmp_path / "model.xml"),
        camera_monitor=StubMonitor(),
        on_transition=lambda state, active, saved, error: transitions.append(
            (state, active, saved, error)))
    created.start(boot_id="boot-1")
    try:
        created.command("r", "TELEOP")
        assert _wait_for_recording(created)
        created.runtime.publish(count=2)
        created.command("s", "TELEOP")
        assert _wait_for(lambda: created.state == "IDLE")
        assert [event[0] for event in transitions] == ["STARTING", "RECORDING", "SAVING", "IDLE"]
        assert transitions[1][1].endswith(".partial.h5")
        assert Path(transitions[-1][2]).is_file()
        created.command("r", "TELEOP")
        assert _wait_for_recording(created)
        created.runtime.publish()
        created.end_episode()
        assert _wait_for(lambda: created.state == "IDLE")
        assert [event[0] for event in transitions[-4:]] == [
            "STARTING", "RECORDING", "ABORTING", "IDLE"]
    finally:
        created.finish()
    assert transitions[-1][0] == "CLOSED"
