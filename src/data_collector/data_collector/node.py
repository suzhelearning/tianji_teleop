"""ROS node that records schema-v1 episodes on operator command.

Responsibilities, and deliberately nothing more:

* subscribe to the three measured-feedback topics and the executor state;
* subscribe to the RGB pairs published by the official RealSense driver;
* expose ``RecordingCommand`` and ``check_ready``;
* publish ``CollectionStatus`` so an operator can see what actually happened.

The node never opens a device, holds no SDK handle and exposes no service that
could enable motion. A collector that dies must not be able to stop the robot,
so nothing here is on the executor's critical path.
"""

from __future__ import annotations

import argparse
import queue
import threading
import time
from pathlib import Path
from functools import partial

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterEvent, ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_parameter_events,
)

from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tianji_interfaces.msg import CollectionStatus, DeviceFeedback, ExecutorState
from tianji_interfaces.srv import RecordingCommand
from tianji_runtime import config_path, workspace
from realsense2_camera_msgs.msg import Metadata as CameraMetadata

from tianji_runtime.camera_stream import (
    CameraStreamValidator,
    ImageRecord,
    MetadataJsonParser,
    StreamFault,
    split_stamp,
)
from tianji_runtime.constants import (
    CAMERA_FPS,
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
)

from .session import CollectionSession

# Measured feedback is a high-rate latest-value stream: dropping an old sample is
# correct, queuing it is not.
FEEDBACK_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Executor state is low-rate and every transition matters. Deliberately NOT
# transient-local: a late subscriber must not receive a stale TELEOP revision
# and treat it as the current instruction.
STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=8,
)

# Status is a latch for observers (including ones that attach later).
STATUS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# The driver publishes with SensorDataQoS; match it rather than request reliable.
IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# Executor state older than this is not a basis for a control action.
STATE_MAX_AGE_S = 0.3

STATUS_RATE_HZ = 10.0

FEEDBACK_TOPICS = {
    "arms": "/tianji/feedback/arms",
    "left_hand": "/tianji/feedback/left_hand",
    "right_hand": "/tianji/feedback/right_hand",
}
EXECUTOR_STATE_TOPIC = "/tianji/executor/state"
STATUS_TOPIC = "/tianji/collection/status"
COMMAND_SERVICE = "/tianji/collection/command"
READY_SERVICE = "/tianji/collection/check_ready"
CAMERA_READY_SERVICE = "/tianji/cameras/check_ready"
MONITOR_NODE = ("tianji_camera_monitor", "/")
CERTIFICATE_PERIOD_S = 0.25
CERTIFICATE_TIMEOUT_S = 1.0


def read_boot_id() -> str:
    """This host's boot identity.

    Published feedback carries the publisher's boot id; a mismatch means the
    sample came from another machine and cannot share this host's monotonic
    clock, so it is refused rather than rescaled.
    """
    try:
        return Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    except OSError:
        return ""


class CollectorNode(Node):
    """Wire subscriptions, the session and the operator services together."""

    def __init__(self, *, dataset_dir: Path, task: str, config_path_: Path,
                 model_path: Path, status_rate_hz: float = STATUS_RATE_HZ,
                 owner_token: str = ""):
        super().__init__("data_collector")
        self._boot_id = read_boot_id()
        if not self._boot_id:
            raise RuntimeError("cannot read this host's boot id; refusing to collect")

        self._owner_token = owner_token
        self._config_path = str(Path(config_path_).resolve())
        self._transition_events = queue.SimpleQueue()
        self._status_guard = self.create_guard_condition(self._drain_status)
        self._session = CollectionSession(
            dataset_dir, task, config_path_, model_path, camera_monitor=self,
            on_transition=self._queue_status)
        for name, value in {
            "owner_token": owner_token,
            "config_path": self._config_path,
            "config_digest": self._session.config_digest,
            "dataset_path": str(Path(dataset_dir).resolve()),
            "task": task,
            "model_path": str(Path(model_path).resolve()),
            "model_digest": self._session.model_digest,
        }.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True), ignore_override=True)
        self._validators: dict[str, CameraStreamValidator] = {}
        self._metadata_json: dict[str, MetadataJsonParser] = {}
        self._executor: ExecutorState | None = None
        self._executor_received_ns: int | None = None
        self._recording_executor = None
        self._retired_executors = set()
        self._runtime_session = None
        self._lock = threading.Lock()
        self._certificate_time = 0.0
        self._certificate_detail = "waiting for matching camera monitor certification"
        self._certificate_generation = 0
        self._certificate_pending = None
        self._camera_rearm_required = False
        self._stopping = False

        self._status_pub = self.create_publisher(CollectionStatus, STATUS_TOPIC, STATUS_QOS)
        self.create_subscription(ExecutorState, EXECUTOR_STATE_TOPIC, self._on_executor, STATE_QOS)
        for device, topic in FEEDBACK_TOPICS.items():
            self.create_subscription(
                DeviceFeedback, topic,
                lambda message, name=device: self._on_feedback(name, message), FEEDBACK_QOS)

        self._cameras = dict(self._session.cameras)
        for role in self._cameras:
            validator = CameraStreamValidator(role, width=IMAGE_WIDTH, height=IMAGE_HEIGHT)
            self._validators[role] = validator
            self._metadata_json[role] = MetadataJsonParser(role)
            base = f"/cameras/{role}/color"
            self.create_subscription(
                Image, f"{base}/image_raw",
                partial(self._on_image, role), IMAGE_QOS)
            self.create_subscription(
                CameraInfo, f"{base}/camera_info",
                lambda message, name=role: self._on_info(name, message), IMAGE_QOS)
            self.create_subscription(
                CameraMetadata, f"{base}/metadata",
                partial(self._on_metadata, role), IMAGE_QOS)

        self._monitor_parameters = self.create_client(
            GetParameters, "/tianji_camera_monitor/get_parameters")
        self._monitor_ready = self.create_client(Trigger, CAMERA_READY_SERVICE)
        self.create_subscription(ParameterEvent, "/parameter_events",
                                 self._on_camera_parameters, qos_profile_parameter_events)
        self.create_timer(CERTIFICATE_PERIOD_S, self._certify_cameras)

        self.create_service(RecordingCommand, COMMAND_SERVICE, self._on_command)
        self.create_service(Trigger, READY_SERVICE, self._on_ready)
        self.create_timer(1.0 / status_rate_hz, self._publish_status)
        self.create_timer(1.0 / CAMERA_FPS, self._check_camera_freshness)

        self.get_logger().info(
            f"collector listening (camera certification pending): dataset={self._session.dataset_dir} task={task} "
            f"cameras={','.join(self._cameras)}")

    # ------------------------------------------------------------------ lifecycle

    def begin(self) -> None:
        """Attach the acquisition runtime after subscriptions are in place."""
        self._session.start(boot_id=self._boot_id)

    def shutdown(self) -> None:
        if self._stopping:
            return
        self._stopping = True
        self._session.finish()
        if rclpy.ok():
            self._drain_status()

    # -------------------------------------------------------------------- inputs

    def _on_feedback(self, device: str, message: DeviceFeedback) -> None:
        if (self._executor is None or self._executor.mode != "real" or self._executor.faulted
                or message.session_id != self._runtime_session):
            return
        received = time.monotonic_ns()
        if device == "arms":
            self._session.on_arms(message, received)
        else:
            self._session.on_hand(device, message, received)

    def status(self) -> tuple[bool, str]:
        """``(ready, detail)`` across every configured camera role.

        This is the camera-readiness half of the session's readiness contract,
        with the same shape as `tianji_cameras.monitor.status`, so the collector
        and the standalone monitor answer identically.
        """
        problems = []
        if time.monotonic() - self._certificate_time > CERTIFICATE_TIMEOUT_S:
            problems.append(self._certificate_detail)
        if self._camera_rearm_required:
            problems.append("waiting for the previous episode to drain and cameras to recertify")
        for role, validator in self._validators.items():
            try:
                validator.check_fresh()
            except StreamFault as error:
                problems.append(str(error))
        if problems:
            return False, "; ".join(problems)
        return True, "ready"

    def _on_executor(self, message: ExecutorState) -> None:
        now = time.monotonic_ns()
        previous = self._executor
        if message.session_id in self._retired_executors:
            self._session.end_episode()
            return
        if (previous is not None and message.session_id != previous.session_id
                and (now - previous.published_monotonic_ns <= STATE_MAX_AGE_S * 1e9
                     or self._session.state != "IDLE"
                     or self._session.current_writer() is not None
                     or message.mode != "real")):
            self._session.end_episode()
            return
        valid = (
            message.boot_id == self._boot_id and bool(message.session_id)
            and 0 <= now - message.published_monotonic_ns <= STATE_MAX_AGE_S * 1e9
            and (previous is None or message.session_id != previous.session_id
                 or (message.sequence > previous.sequence
                     and message.phase_revision >= previous.phase_revision))
        )
        if not valid:
            self._session.end_episode()
            return
        if previous is not None and message.session_id != previous.session_id:
            self._retired_executors.add(previous.session_id)
        with self._lock:
            self._executor = message
            self._executor_received_ns = now
        if self._recording_executor is not None and (
            (message.session_id, message.phase_revision) != self._recording_executor
            or message.mode != "real" or message.phase != "TELEOP" or message.faulted
        ):
            self._session.end_episode()

    def _on_image(self, role: str, message: Image, info) -> None:
        if self._camera_rearm_required:
            return
        validator = self._validators[role]
        received = time.monotonic_ns()
        try:
            validator.note_image_publisher(
                self._publisher_gid(f"/cameras/{role}/color/image_raw"))
            self._check_header(message.header)
            record = ImageRecord(
                stamp_ns=split_stamp(message.header.stamp),
                frame_id=message.header.frame_id,
                width=int(message.width),
                height=int(message.height),
                step=int(message.step),
                encoding=message.encoding,
                rgb=_rgb_view(message),
            )
            frame = validator.accept_image(record, received)
        except (StreamFault, ValueError) as error:
            self._camera_failed(role, str(error))
            return
        if frame is not None:
            self._session.on_image(role, frame)

    def _on_metadata(self, role: str, message, info) -> None:
        """Consume the driver's Metadata payload for one frame.

        The frame number lives in the JSON document the driver publishes, not in
        the message header, so the document is parsed here before the
        validator can order or de-duplicate the frame.
        """
        if self._camera_rearm_required:
            return
        validator = self._validators[role]
        received = time.monotonic_ns()
        try:
            validator.note_metadata_publisher(
                self._publisher_gid(f"/cameras/{role}/color/metadata"))
            self._check_header(message.header)
            record = self._metadata_json[role].accept(
                message.header.frame_id, split_stamp(message.header.stamp),
                message.json_data)
            frame = validator.accept_metadata(record, received)
        except (StreamFault, ValueError) as error:
            self._camera_failed(role, str(error))
            return
        if frame is not None:
            self._session.on_image(role, frame)

    def _on_info(self, role: str, message: CameraInfo) -> None:
        """CameraInfo has a separate DDS publisher; never use it as Image identity."""

    def _publisher_gid(self, topic):
        endpoints = self.get_publishers_info_by_topic(topic)
        if len(endpoints) != 1:
            raise StreamFault(f"{topic}: expected one publisher, found {len(endpoints)}")
        endpoint = endpoints[0]
        role = topic.split("/")[2]
        if endpoint.node_name != role or endpoint.node_namespace != "/cameras":
            raise StreamFault(f"{topic}: unexpected publisher node identity")
        gid = bytes(endpoint.endpoint_gid)
        if not gid or not any(gid):
            raise StreamFault(f"{topic}: publisher endpoint has no GID")
        return gid

    def _check_header(self, header):
        age = self.get_clock().now().nanoseconds - split_stamp(header.stamp)
        if not -5_000_000 <= age <= 2_000_000_000:
            raise StreamFault("camera header is stale or in the future")

    def _camera_failed(self, role: str, detail: str) -> None:
        if not self._camera_rearm_required:
            self.get_logger().error(f"camera {role}: {detail}; current episode stays partial")
        self._session.on_camera_fault(role, detail)
        self._revoke_certificate(detail)
        self._session.end_episode()

    def _check_camera_freshness(self) -> None:
        if (self._session.state == "IDLE" and self._executor is not None
                and self._executor.mode == "real" and not self._executor.faulted
                and self._runtime_session != self._executor.session_id):
            self._session.runtime.rearm_feedback(self._executor.session_id)
            self._runtime_session = self._executor.session_id
        if (self._certificate_time and time.monotonic() - self._certificate_time
                > CERTIFICATE_TIMEOUT_S):
            self._revoke_certificate("camera monitor certificate expired")
        if self._camera_rearm_required:
            self._session.end_episode()
        self._session.check()
        if self._session.state in ("STARTING", "RECORDING"):
            state = self._executor
            if (state is None or state.mode != "real" or state.phase != "TELEOP"
                    or state.faulted or time.monotonic_ns() - state.published_monotonic_ns
                    > STATE_MAX_AGE_S * 1e9):
                self._session.end_episode()
        now = time.monotonic_ns()
        for role, validator in self._validators.items():
            if self._session.state != "RECORDING":
                continue
            try:
                validator.check_fresh(now)
            except StreamFault as error:
                self._camera_failed(role, str(error))


    def _on_camera_parameters(self, event):
        if event.node in {f"/cameras/{role}" for role in self._cameras}:
            self._revoke_certificate("driver parameters changed; recertification required")
            self._session.end_episode()

    def _revoke_certificate(self, detail):
        self._certificate_time = 0.0
        self._certificate_detail = detail
        self._certificate_generation += 1
        self._camera_rearm_required = True
        if self._certificate_pending is not None:
            self._certificate_pending[0].cancel()
            self._certificate_pending = None
        self._session.end_episode()

    def _monitor_identity(self):
        """A well-named service is insufficient: reject duplicates and foreign providers."""
        nodes = self.get_node_names_and_namespaces()
        if nodes.count(MONITOR_NODE) != 1:
            raise RuntimeError("expected exactly one /tianji_camera_monitor")
        expected = {
            CAMERA_READY_SERVICE: "std_srvs/srv/Trigger",
            "/tianji_camera_monitor/get_parameters": "rcl_interfaces/srv/GetParameters",
        }
        providers = {service: [] for service in expected}
        for name, namespace in set(nodes):
            services = dict(self.get_service_names_and_types_by_node(name, namespace))
            for service, service_type in expected.items():
                if service in services:
                    if services[service] != [service_type]:
                        raise RuntimeError(f"{service}: unexpected service type")
                    providers[service].append((name, namespace))
        if any(owners != [MONITOR_NODE] for owners in providers.values()):
            raise RuntimeError("camera monitor services have conflicting node ownership")

    def _certify_cameras(self):
        now = time.monotonic()
        pending = self._certificate_pending
        if pending is not None:
            if now - pending[1] < CERTIFICATE_TIMEOUT_S:
                return
            self._revoke_certificate("camera monitor request timed out")
        try:
            self._monitor_identity()
            if not self._monitor_parameters.service_is_ready() or not self._monitor_ready.service_is_ready():
                raise RuntimeError("camera monitor services unavailable")
        except Exception as error:
            self._revoke_certificate(str(error))
            return
        generation = self._certificate_generation
        request = GetParameters.Request()
        request.names = ["owner_token", "config_path", "config_digest"]

        def parameters(response):
            if len(response.values) != 3 or any(
                    value.type != ParameterType.PARAMETER_STRING for value in response.values):
                raise RuntimeError("camera monitor identity parameters are missing")
            owner, path, digest = (value.string_value for value in response.values)
            if owner and owner != self._owner_token:
                raise RuntimeError("camera monitor belongs to another owner")
            if path != self._config_path or digest != self._session.config_digest:
                raise RuntimeError("camera monitor collection config path/digest mismatch")
            self._camera_request(self._monitor_ready, Trigger.Request(), ready, generation)

        def ready(response):
            if not response.success:
                raise RuntimeError(response.message or "camera monitor is not ready")
            self._monitor_identity()
            if self._camera_rearm_required:
                if self._session.state != "IDLE" or self._session.current_writer() is not None:
                    return  # Never revive or extend the retired episode.
                self._validators = {
                    role: CameraStreamValidator(role, width=IMAGE_WIDTH, height=IMAGE_HEIGHT)
                    for role in self._cameras}
                self._metadata_json = {role: MetadataJsonParser(role) for role in self._cameras}
                self._session.runtime.rearm_cameras()
                self._camera_rearm_required = False
            self._certificate_time = time.monotonic()
            self._certificate_detail = "camera monitor certificate expired"

        try:
            self._camera_request(self._monitor_parameters, request, parameters, generation)
        except Exception as error:
            self._revoke_certificate(str(error))

    def _camera_request(self, client, request, accept, generation):
        future = client.call_async(request)
        self._certificate_pending = (future, time.monotonic())

        def completed(result):
            if (self._certificate_pending is None
                    or self._certificate_pending[0] is not result):
                return
            self._certificate_pending = None
            if result.cancelled() or generation != self._certificate_generation:
                return
            try:
                response = result.result()
                if response is None:
                    raise RuntimeError("empty camera monitor response")
                accept(response)
            except Exception as error:
                self._revoke_certificate(str(error))
        future.add_done_callback(completed)

    # ------------------------------------------------------------------ services

    def _on_ready(self, request, response):
        ready, detail = self._session.check_ready()
        response.success = ready
        response.message = "ready" if ready else detail
        return response

    def _on_command(self, request, response):
        command = (request.command or "").strip().lower()
        if command not in ("start", "save", "discard", "abort"):
            response.accepted = False
            response.state = self._session.state
            response.message = f"unknown command {request.command!r}"
            return response

        if command != "abort":
            ok, reason = self._authorize(command, request)
            if not ok:
                response.accepted = False
                response.state = self._session.state
                response.message = reason
                self.get_logger().warn(f"rejected {command}: {reason}")
                return response

        if command == "abort":
            # Reducing recording state only; it never commands the robot.
            if (self._session.state not in ("STARTING", "RECORDING")
                    or self._recording_executor is None
                    or request.session_id != self._recording_executor[0]):
                response.accepted = False
                response.state = self._session.state
                response.message = "no active recording for this executor session"
                return response
            self._session.end_episode()
        else:
            if command == "start":
                ready, detail = self._session.check_ready()
                if not ready:
                    response.accepted = False
                    response.state = self._session.state
                    response.message = detail
                    return response
                self._recording_executor = (
                    self._executor.session_id, self._executor.phase_revision)
            key = {"start": "r", "save": "s", "discard": "d"}[command]
            state_before = self._session.state
            self._session.command(key, self._phase())
            if self._session.state == state_before:
                response.accepted = False
                response.state = self._session.state
                response.message = f"{command} is not applicable in state {state_before}"
                return response

        response.accepted = True
        response.state = self._session.state
        # An enqueue acknowledgement, not a promise that bytes reached disk: the
        # outcome is published on CollectionStatus.
        response.message = f"{command} accepted"
        return response

    def _authorize(self, command: str, request):
        """Refuse a control action that is not backed by a fresh, real session."""
        with self._lock:
            state = self._executor
            received = self._executor_received_ns
        if state is None:
            return False, "no executor state received"
        age = time.monotonic_ns() - state.published_monotonic_ns
        if not 0 <= age <= STATE_MAX_AGE_S * 1e9:
            return False, f"executor state is {age / 1e9:.3f}s old"
        if state.faulted:
            return False, f"executor is faulted: {state.detail or 'no detail'}"
        if state.mode != "real":
            return False, f"executor mode is {state.mode!r}, not 'real'"
        if state.phase != "TELEOP":
            return False, f"executor phase is {state.phase!r}, not 'TELEOP'"
        if request.session_id != state.session_id:
            return False, "operator session id does not match the executor"
        if request.phase_revision != state.phase_revision:
            return False, "operator phase revision does not match the executor"
        return True, ""

    def _phase(self) -> str:
        with self._lock:
            return self._executor.phase if self._executor is not None else ""

    # -------------------------------------------------------------------- status

    def _status_session(self, state):
        executor = self._executor
        if executor is None or (
                state == "IDLE" and time.monotonic_ns() - executor.published_monotonic_ns
                > STATE_MAX_AGE_S * 1e9):
            return ""
        return executor.session_id

    def _queue_status(self, state, active_path, last_saved_path, error):
        # Called by both ROS and writer-manager threads. SimpleQueue never waits
        # for a consumer; only the executor's guard callback performs DDS work.
        self._transition_events.put((
            state, active_path, last_saved_path, error,
            self._status_session(state), time.monotonic_ns()))
        if not self._stopping and rclpy.ok():
            self._status_guard.trigger()

    def _drain_status(self):
        while True:
            try:
                event = self._transition_events.get_nowait()
            except queue.Empty:
                return
            self._publish_status(event)

    def _publish_status(self, event=None) -> None:
        if event is None:
            self._drain_status()
        message = CollectionStatus()
        state = event[0] if event else self._session.state
        message.session_id = event[4] if event else self._status_session(state)
        message.published_monotonic_ns = event[5] if event else time.monotonic_ns()
        message.state = state
        camera_ready, camera_detail = self._session.camera_ready()
        # `prepared` is cameras verified *and* the writer ready; `inputs_ready`
        # additionally requires fresh measured feedback.
        prepared = self._session.prepared and camera_ready
        message.prepared = prepared
        message.inputs_ready = prepared and not self._session.missing_inputs()
        writer = self._session.current_writer()
        message.active_path = event[1] if event else (
            str(writer.partial_path) if writer is not None else "")
        message.last_saved_path = event[2] if event else (
            str(self._session.saved_paths[-1]) if self._session.saved_paths else "")
        message.error = event[3] if event else self._session.last_error or ""
        if not message.error and not camera_ready:
            message.error = camera_detail
        self._status_pub.publish(message)


def _rgb_view(message: Image):
    """Expose the Image payload as a numpy view without copying.

    `Image.data` is already a contiguous uint8 buffer sized
    height*step; the view is read-only so the writer cannot mutate a message it
    does not own. Reshaping to the declared width validates the geometry.
    """
    import numpy as np

    buffer = np.frombuffer(message.data, dtype=np.uint8)
    if buffer.size != message.height * message.step:
        raise ValueError(
            f"image buffer is {buffer.size} bytes, expected {message.height * message.step}")
    return buffer.reshape(message.height, message.step // 3, 3)[:, : message.width, :]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="data_collector",
        description="Record schema-v1 episodes from the executor's feedback and the camera driver.")
    parser.add_argument("--dataset", type=Path, default=None,
                        help="raw dataset root (default: TIANJI_DATASET)")
    parser.add_argument("--task", required=True, help="operator task label for the episodes")
    parser.add_argument("--config", type=Path, default=config_path("collect_real.json"))
    parser.add_argument("--robot-config", type=Path, default=config_path("robot.json"))
    parser.add_argument("--model", type=Path)
    parser.add_argument("--owner-token", default="")
    args = parser.parse_args(argv)

    from tianji_runtime import dataset_dir

    dataset = args.dataset or dataset_dir()
    if args.model is not None:
        model_path = args.model.resolve()
    else:
        robot = __import__("json").loads(args.robot_config.read_text())
        model_path = workspace() / robot["controller_model"]

    rclpy.init()
    node = CollectorNode(dataset_dir=dataset, task=args.task, config_path_=args.config,
                         model_path=model_path, owner_token=args.owner_token)
    node.begin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
