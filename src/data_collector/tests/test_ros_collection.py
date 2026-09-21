"""Schema-v1 collection over real DDS processes.

Run by `pixi run test-ros`. The collector and an input fixture are separate
processes on a dedicated domain, so the subscriptions, services, QoS and
identity checks are exercised the way they run in the field rather than through
in-process mocks. No hardware is contacted and no production fake-hardware mode
is added: the fixture publishes messages, it does not impersonate a device.

The camera payload encodes each frame's source number, which lets a recorded
episode be checked against what was actually published.
"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

import h5py
import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from data_collector.dataset import ensure_dataset_config, validate_episode
from tianji_interfaces.msg import CollectionStatus, ExecutorState
from tianji_interfaces.srv import RecordingCommand

# A test domain, deliberately not the deployment default, so a running
# production session can never be observed or commanded by these tests.
TEST_DOMAIN = "121"
STATUS_TOPIC = "/tianji/collection/status"
COMMAND_SERVICE = "/tianji/collection/command"
READY_SERVICE = "/tianji/collection/check_ready"

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[2]
MODEL_CONFIG = WORKSPACE / "config" / "robot.json"

STATUS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def _environment() -> dict[str, str]:
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = TEST_DOMAIN
    return environment


def _write_config(path: Path) -> Path:
    """Strict config whose serials the fixture driver must actually certify."""
    path.write_text(json.dumps({
        "robot_config": "tianji_wuji2_v1",
        "state_rate_hz": 120,
        "cameras": {"top": "TESTTOP1", "left_wrist": "TESTLEFT1",
                    "right_wrist": "TESTRIGHT1"},
    }), encoding="utf-8")
    return path


class Harness:
    """One collector process plus one fixture process, torn down together."""

    def __init__(self, root: Path, *, session_id: str, fixture: list[str],
                 expect_shutdown: bool = True, collector_fixture: bool = False):
        self.root = root
        self.session_id = session_id
        self.dataset = root / "dataset"
        self.dataset.mkdir(parents=True, exist_ok=True)
        self.config = _write_config(root / "collect_test.json")
        self.logs = root / "logs"
        self.logs.mkdir(exist_ok=True)
        self._fixture_arguments = fixture
        self._expect_shutdown = expect_shutdown
        self._collector_fixture = collector_fixture
        self.collector: subprocess.Popen | None = None
        self.fixture: subprocess.Popen | None = None

    def start(self) -> None:
        environment = _environment()
        entry = ([str(HERE / "ros_collector_fixture.py"),
                  "--writer-control", str(self.root / "control.json")]
                 if self._collector_fixture else ["-m", "data_collector.node"])
        self.collector = subprocess.Popen(
            [sys.executable, *entry,
             "--dataset", str(self.dataset), "--task", "dds acceptance",
             "--config", str(self.config), "--robot-config", str(MODEL_CONFIG)],
            env=environment, stdout=(self.logs / "collector.log").open("wb"),
            stderr=subprocess.STDOUT, start_new_session=True)
        self.start_fixture()

    def start_fixture(self) -> None:
        """Restart only the test inputs/monitor, preserving the collector process."""
        environment = _environment()
        ready_file = self.root / "fixture-ready"
        ready_file.unlink(missing_ok=True)
        self.fixture = subprocess.Popen(
            [sys.executable, str(HERE / "ros_fixture.py"),
             "--session-id", self.session_id, "--ready-file", str(ready_file),
             "--control-file", str(self.root / "control.json"),
             "--config", str(self.config),
             *self._fixture_arguments],
            env=environment, stdout=(self.logs / "fixture.log").open("wb"),
            stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            if ready_file.exists():
                return
            if self.collector.poll() is not None:
                raise AssertionError(
                    f"collector exited early: {self.collector.returncode}\n"
                    + self.collector_log())
            time.sleep(0.05)
        raise AssertionError(f"fixture did not become ready\n{self.fixture_log()}")

    def control(self, **values):
        temporary = self.root / "control.tmp"
        temporary.write_text(json.dumps(values))
        temporary.replace(self.root / "control.json")

    def collector_log(self) -> str:
        return (self.logs / "collector.log").read_text(errors="replace")[-4000:]

    def fixture_log(self) -> str:
        return (self.logs / "fixture.log").read_text(errors="replace")[-2000:]

    def stop(self) -> None:
        for process in (self.fixture, self.collector):
            if process is None or process.poll() is not None:
                continue
            process.terminate()
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)

    def completed_episodes(self) -> list[Path]:
        return sorted(path for path in self.dataset.rglob("*.h5")
                      if not path.name.endswith(".partial.h5"))

    def partial_episodes(self) -> list[Path]:
        return sorted(self.dataset.rglob("*.partial.h5"))


class Client(Node):
    """Service client and status observer for one collector session."""

    def __init__(self):
        super().__init__("collection_test_client")
        self.status: CollectionStatus | None = None
        self.executor_state = None
        self.create_subscription(ExecutorState, "/tianji/executor/state",
                                 self._on_executor, QoSProfile(depth=8))
        self.create_subscription(CollectionStatus, STATUS_TOPIC, self._on_status,
                                 STATUS_QOS)
        self._command = self.create_client(RecordingCommand, COMMAND_SERVICE)
        self._ready = self.create_client(Trigger, READY_SERVICE)

    def _on_status(self, message: CollectionStatus) -> None:
        if self.status is None or message.published_monotonic_ns >= self.status.published_monotonic_ns:
            self.status = message

    def _on_executor(self, message):
        self.executor_state = message

    def wait_for_services(self, timeout: float = 20.0) -> None:
        assert self._command.wait_for_service(timeout_sec=timeout), "command service absent"
        assert self._ready.wait_for_service(timeout_sec=timeout), "ready service absent"

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_ready(self, timeout=15.0):
        deadline = time.monotonic() + timeout
        while True:
            ready, detail = self.check_ready()
            if ready:
                return
            assert time.monotonic() < deadline, detail
            self.spin_for(0.05)

    def check_ready(self) -> tuple[bool, str]:
        future = self._ready.call_async(Trigger.Request())
        self._wait(future)
        response = future.result()
        return bool(response.success), response.message

    def command(self, name: str) -> tuple[bool, str, str]:
        request = RecordingCommand.Request()
        request.command = name
        request.session_id = self.session_id or ""
        request.phase_revision = self.phase_revision
        future = self._command.call_async(request)
        self._wait(future)
        response = future.result()
        return bool(response.accepted), response.state, response.message

    session_id: str = ""
    phase_revision: int = 1

    def _wait(self, future, timeout: float = 20.0) -> None:
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        assert future.done(), "service call did not complete"

    def wait_for_state(self, wanted: str, timeout: float = 30.0) -> CollectionStatus:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.status is not None and self.status.state == wanted:
                return self.status
        observed = self.status.state if self.status else "no status"
        raise AssertionError(f"never reached {wanted!r} (saw {observed!r})")


@pytest.fixture(scope="module")
def ros():
    rclpy.init(domain_id=int(TEST_DOMAIN))
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    client = Client()
    try:
        yield client
    finally:
        client.destroy_node()


@pytest.fixture
def harness(tmp_path):
    started: list[Harness] = []

    def make(fixture: list[str], session_id: str | None = None, **kwargs) -> Harness:
        instance = Harness(tmp_path / f"run{len(started)}",
                           session_id=session_id or uuid.uuid4().hex,
                           fixture=fixture, **kwargs)
        started.append(instance)
        instance.start()
        return instance

    try:
        yield make
    finally:
        for instance in started:
            instance.stop()


def _record(client: Client, harness: Harness, *, seconds: float,
            expect_ready: bool = True) -> Path:
    """Start, record, save; returns the saved episode path."""
    client.session_id = harness.session_id
    client.wait_for_services()
    if expect_ready:
        deadline = time.monotonic() + 15.0
        while True:
            ready, detail = client.check_ready()
            if ready:
                break
            assert time.monotonic() < deadline, (
                f"collector not ready: {detail}\n{harness.collector_log()}\n{harness.fixture_log()}")
            client.spin_for(0.05)
    client.wait_for_state("IDLE")

    accepted, _, message = client.command("start")
    assert accepted, message
    client.wait_for_state("RECORDING")
    client.spin_for(seconds)
    accepted, _, message = client.command("save")
    assert accepted, message

    status = client.wait_for_state("IDLE", timeout=60.0)
    assert status.last_saved_path, f"no saved path in status: {status.error!r}"
    saved = Path(status.last_saved_path)
    assert saved.is_file(), f"saved episode missing: {saved}"
    return saved


def _read_episode(path: Path) -> dict:
    with h5py.File(path, "r") as episode:
        return {
            "success": bool(episode.attrs["success"]),
            "arms_ts": episode["observations/arms/timestamp_ns"][:],
            "arms": episode["observations/arms/qpos"][:],
            "hands_ts": episode["observations/hands/timestamp_ns"][:],
            "hands": episode["observations/hands/qpos"][:],
            "images": {
                role: (episode[f"images/{role}/timestamp_ns"][:],
                       episode[f"images/{role}/rgb"][:])
                for role in ("top", "left_wrist", "right_wrist")
            },
        }


def _frame_number(rgb: np.ndarray) -> int:
    first = rgb[0, 0]
    return int(first[0]) | (int(first[1]) << 8)


def test_rejects_start_without_a_fresh_real_teleop_session(node, harness):
    """Authorization is by fresh identity, not by the service existing."""
    case = harness(["--mode", "dry_run", "--phase", "TELEOP"])
    node.session_id = case.session_id
    node.wait_for_services()

    accepted, state, message = node.command("start")
    assert not accepted, "dry-run executor must not authorize recording"
    assert "real" in message
    assert state == "IDLE"


    assert not case.completed_episodes()


def test_records_a_validated_episode(node, harness):
    case = harness([])
    saved = _record(node, case, seconds=3.0)

    config = json.loads((saved.parent / "dataset_config.json").read_text())
    result = validate_episode(saved, config, require_success=True)
    assert result["counts"]["hands"] > 0
    assert result["counts"]["arms"] > 0

    episode = _read_episode(saved)
    assert episode["success"] is True
    assert episode["arms"].shape[1] == 14
    assert episode["hands"].shape[1] == 40
    for role, (timestamps, frames) in episode["images"].items():
        assert frames.shape[1:] == (720, 1280, 3), role
        assert timestamps.size > 0, role
        assert np.all(np.diff(timestamps) >= 0), role
    assert np.all(np.diff(episode["arms_ts"]) >= 0)
    assert np.all(np.diff(episode["hands_ts"]) >= 0)

    # Streams are independent: three cameras produce three separate series, and
    # the state streams are not forced onto the camera cadence.
    image_counts = {role: frames.shape[0]
                    for role, (_, frames) in episode["images"].items()}
    assert all(count > 0 for count in image_counts.values()), image_counts
    assert episode["arms_ts"].size > max(image_counts.values())

    # Each stored frame embeds its source counter, with no duplicate images.
    for _, frames in episode["images"].values():
        numbers = [_frame_number(frame) for frame in frames]
        assert all(later > earlier for earlier, later in zip(numbers, numbers[1:]))
        assert np.all(frames[:, 0, 0, 2] == 0x5A)

    # A saved episode leaves no partial behind for the same range.
    assert not saved.with_suffix(".partial.h5").exists()

    with saved.open("rb") as source:
        source_digest = hashlib.file_digest(source, "sha256").hexdigest()
    destination = case.root / "explicit-jpeg-output"
    converted = subprocess.run(
        [sys.executable, "-m", "data_collector.compress", str(saved.parent), str(destination)],
        env=_environment(), capture_output=True, text=True, timeout=90)
    assert converted.returncode == 0, converted.stdout + converted.stderr
    compressed = destination / saved.name
    compressed_config = json.loads((destination / "dataset_config.json").read_text())
    assert compressed_config["image_encoding"] == "jpeg"
    assert compressed_config["jpeg_quality"] == 50
    validate_episode(compressed, compressed_config, require_success=True)
    assert not destination.with_name(destination.name + "_compressed").exists()
    with saved.open("rb") as source:
        assert hashlib.file_digest(source, "sha256").hexdigest() == source_digest
    with h5py.File(compressed) as output:
        np.testing.assert_array_equal(output["observations/arms/timestamp_ns"], episode["arms_ts"])
        np.testing.assert_array_equal(output["observations/hands/timestamp_ns"], episode["hands_ts"])


def test_partial_segment_is_kept_when_feedback_stops(node, harness):
    case = harness([])
    node.session_id = case.session_id
    node.wait_for_services()
    node.wait_ready()
    node.wait_for_state("IDLE")
    accepted, _, message = node.command("start")
    assert accepted, message
    node.wait_for_state("RECORDING")
    case.control(stop_feedback=True)

    # Feedback stops; the segment must end partial without a success file.
    status = node.wait_for_state("IDLE", timeout=30.0)
    assert not status.inputs_ready
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline and not case.partial_episodes():
        node.spin_for(0.2)
    assert case.partial_episodes(), "an unsaved segment must remain as partial"
    assert not case.completed_episodes()

    with h5py.File(case.partial_episodes()[0], "r") as episode:
        assert "success" not in episode.attrs


def test_duplicate_hand_token_and_frame_number_add_no_records(node, harness):
    """Repeated source identity is not new information."""
    case = harness([])
    node.session_id = case.session_id
    node.wait_for_services()
    node.wait_ready()
    node.wait_for_state("IDLE")
    accepted, _, message = node.command("start")
    assert accepted, message
    node.wait_for_state("RECORDING")
    node.spin_for(0.5)
    case.control(repeat_hand_token="right_hand", repeat_frame_number=True)
    status = node.wait_for_state("IDLE", timeout=10.0)
    assert not status.last_saved_path
    assert not status.inputs_ready
    case.stop()
    assert not case.completed_episodes()
    partials = case.partial_episodes()
    assert len(partials) == 1
    with h5py.File(partials[0], "r") as episode:
        assert "success" not in episode.attrs
        arms = episode["observations/arms/timestamp_ns"][:]
        hands = episode["observations/hands/timestamp_ns"][:]
        # Left keeps updating during the freshness timeout but no new bilateral
        # state may be assembled from a frozen right hand.
        assert arms[-1] - hands[-1] > 100_000_000
        for role in ("top", "left_wrist", "right_wrist"):
            pixels = episode[f"images/{role}/rgb"][:, 0, 0, :]
            numbers = pixels[:, 0].astype(int) + 256 * pixels[:, 1].astype(int)
            assert len(numbers) >= 2
            assert np.all(np.diff(numbers) > 0)


def test_saved_episode_survives_a_later_discard(node, harness):
    case = harness([])
    saved = _record(node, case, seconds=1.5)
    before = saved.read_bytes()

    accepted, _, message = node.command("start")
    assert accepted, message
    node.wait_for_state("RECORDING")
    node.spin_for(1.0)
    accepted, _, message = node.command("discard")
    assert accepted, message
    node.wait_for_state("IDLE", timeout=60.0)

    assert saved.is_file() and saved.read_bytes() == before
    assert len(case.completed_episodes()) == 1


@pytest.mark.parametrize("fault", [
    {"stop_images": True},
    {"stop_state": True},
    {"phase": "HOME_REACHED"},
    {"mode": "dry_run"},
    {"boot_id": "another-host"},
    {"session_id": "replacement-executor"},
    {"source_offset_ns": 1_000_000_000},
    {"frame_number": 0},
    {"header_age_s": 3},
    {"restart_publisher": True},
])
def test_stream_or_executor_fault_never_publishes_success(node, harness, fault):
    case = harness([])
    node.session_id = case.session_id
    node.wait_for_services()
    node.wait_ready()
    assert node.command("start")[0]
    node.wait_for_state("RECORDING")
    node.spin_for(0.3)
    case.control(**fault)
    status = node.wait_for_state("IDLE", timeout=8.0)
    assert not status.last_saved_path
    case.stop()
    assert not case.completed_episodes()
    partials = case.partial_episodes()
    assert len(partials) == 1
    with h5py.File(partials[0]) as episode:
        assert "success" not in episode.attrs


def test_camera_preparation_precedes_feedback_and_authority(node, harness):
    case = harness(["--stop-feedback-after", "0"])
    node.session_id = case.session_id
    node.wait_for_services()
    deadline = time.monotonic() + 10
    while node.status is None or not node.status.prepared:
        assert time.monotonic() < deadline, case.collector_log()
        node.spin_for(0.05)
    assert not node.status.inputs_ready
    assert not node.command("start")[0]
    assert not case.completed_episodes()


def test_commands_require_exact_session_revision_and_live_teleop(node, harness):
    case = harness([])
    node.session_id = case.session_id
    node.wait_for_services()
    node.wait_ready()
    node.session_id = "not-the-current-executor"
    assert not node.command("start")[0]
    node.session_id = case.session_id
    node.phase_revision = 0
    assert not node.command("start")[0]
    node.phase_revision = 2
    assert not node.command("start")[0]
    node.phase_revision = 1
    case.control(stop_state=True)
    node.spin_for(0.5)
    assert not node.command("start")[0]
    assert not case.completed_episodes()


@pytest.mark.parametrize("arguments", [
    ["--profile", "640,480,30"],
    ["--device-serial", "WRONGSERIAL"],
    ["--monitor-owner", "foreign-owner"],
    ["--duplicate-monitor"],
    ["--monitor-other-config"],
])
def test_standalone_requires_matching_monitor_and_effective_profile(node, harness, arguments):
    case = harness(arguments)
    node.session_id = case.session_id
    node.wait_for_services()
    node.spin_for(4.0)  # Enough for serial/profile checks, not merely first images.
    assert node.status is not None and not node.status.prepared
    ready, detail = node.check_ready()
    assert not ready and detail
    assert not node.command("start")[0]
    assert not case.completed_episodes()


def test_same_config_path_with_changed_bytes_cannot_recertify(node, harness):
    case = harness([])
    node.session_id = case.session_id
    node.wait_for_services()
    node.wait_ready()
    case.fixture.terminate()
    case.fixture.wait(timeout=10)
    case.config.write_bytes(case.config.read_bytes() + b"\n")
    case.start_fixture()
    node.spin_for(4.0)
    monitor = node.create_client(Trigger, "/tianji/cameras/check_ready")
    try:
        assert monitor.wait_for_service(timeout_sec=5)
        future = monitor.call_async(Trigger.Request())
        node._wait(future)
        assert future.result().success  # New monitor certifies the very same profile/serial.
    finally:
        node.destroy_client(monitor)
    assert node.status is not None and not node.status.prepared
    assert not node.check_ready()[0]
    assert not case.completed_episodes()


@pytest.mark.parametrize("fault", [
    {"restart_publisher": True},
    {"reset_frames": True},
    {"profile": "640,480,30"},
])
def test_recertified_camera_requires_a_new_start_and_preserves_partial(node, harness, fault):
    case = harness([])
    node.session_id = case.session_id
    node.wait_for_services()
    node.wait_ready()
    assert node.command("start")[0]
    node.wait_for_state("RECORDING")
    node.spin_for(0.5)
    case.control(**fault)
    status = node.wait_for_state("IDLE", timeout=8)
    assert not status.last_saved_path
    partials = case.partial_episodes()
    assert len(partials) == 1
    prior = partials[0].read_bytes()
    case.control()
    node.wait_ready()
    node.spin_for(0.5)
    assert node.status.state == "IDLE"
    saved = _record(node, case, seconds=0.6)
    assert partials[0].read_bytes() == prior
    assert case.completed_episodes() == [saved]
    result = _read_episode(saved)
    assert result["success"]
    assert np.all(result["arms_ts"] >= 0)
    for timestamps, frames in result["images"].values():
        assert timestamps[0] > 0
        numbers = [_frame_number(frame) for frame in frames]
        assert all(b > a for a, b in zip(numbers, numbers[1:]))


def test_transition_status_is_published_without_waiting_for_periodic_timer(node, harness):
    case = harness([], collector_fixture=True)  # Periodic status is only every 20s.
    node.session_id = case.session_id
    node.wait_for_services()
    node.wait_ready()
    assert node.command("start")[0]
    node.wait_for_state("RECORDING", timeout=2)
    node.spin_for(0.3)
    assert node.command("discard")[0]
    node.wait_for_state("IDLE", timeout=2)
    assert not case.completed_episodes()
    assert not case.partial_episodes()


def test_writer_queue_overflow_keeps_partial_and_executor_continues(node, harness):
    case = harness([], collector_fixture=True)
    node.session_id = case.session_id
    node.wait_for_services()
    node.wait_ready()
    assert node.command("start")[0]
    node.wait_for_state("RECORDING", timeout=2)
    node.spin_for(0.3)
    before = int(node.executor_state.detail.removeprefix("gate_steps="))
    case.control(block_writer=True)
    status = node.wait_for_state("ABORTING", timeout=3)
    assert "overflow" in status.error
    assert int(node.executor_state.detail.removeprefix("gate_steps=")) > before
    assert node.executor_state.phase == "TELEOP"
    assert not node.executor_state.faulted
    case.control()
    node.wait_for_state("IDLE", timeout=8)
    assert not case.completed_episodes()
    partials = case.partial_episodes()
    assert len(partials) == 1
    with h5py.File(partials[0]) as episode:
        assert "success" not in episode.attrs
    before = int(node.executor_state.detail.removeprefix("gate_steps="))
    case.collector.terminate()
    case.collector.wait(timeout=10)
    node.spin_for(0.3)
    assert int(node.executor_state.detail.removeprefix("gate_steps=")) > before
    assert node.executor_state.phase == "TELEOP" and not node.executor_state.faulted


def test_abort_is_scoped_to_the_recording_executor_without_teleop_authority(node, harness):
    case = harness([])
    node.session_id = case.session_id
    node.wait_for_services()
    node.wait_ready()
    assert node.command("start")[0]
    node.wait_for_state("RECORDING")
    node.session_id = "another-executor"
    assert not node.command("abort")[0]
    node.spin_for(0.1)
    assert node.status.state == "RECORDING"
    node.session_id = case.session_id
    node.phase_revision = 999  # Abort reduces state; it is not a motion authorization.
    assert node.command("abort")[0]
    node.wait_for_state("IDLE")
    assert len(case.partial_episodes()) == 1
    assert not case.completed_episodes()
