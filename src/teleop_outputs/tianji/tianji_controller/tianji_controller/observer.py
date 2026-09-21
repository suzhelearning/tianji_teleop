"""DDS observation only: all serialization and RPC live off the motion thread."""
from __future__ import annotations

from concurrent.futures import Future
from pathlib import Path
import os
import queue
import threading
import time
import uuid


class ExecutorObserver:
    def __init__(self, mode: str, *, collection=False):
        if mode not in ("real", "dry_run"):
            raise ValueError("unsupported executor observation mode")
        self.mode = mode
        self.session_id = str(uuid.uuid4())
        self.boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        self.collection = collection
        if not self.boot_id:
            raise RuntimeError("local boot identity unavailable")
        self._lock = threading.Lock()
        self._state = ("PREFLIGHT", 0, False, "", time.monotonic_ns())
        self._sampler = None
        self._status = None
        self._status_gid = None
        self._collection_error = None
        self._draining = False
        self._collection_active = not collection
        self._error = None
        self._commands = queue.Queue(maxsize=8)
        self._requests = queue.Queue(maxsize=8)
        self._abort = threading.Event()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread = None
        self._dropped = 0

    def start(self):
        self._thread = threading.Thread(target=self._run, name="executor-dds", daemon=True)
        self._thread.start()
        if not self._ready.wait(10):
            raise RuntimeError("executor DDS initialization timed out")
        if self._error is not None:
            raise RuntimeError(f"executor DDS initialization failed: {self._error}")

    def attach_sampler(self, sampler):
        with self._lock:
            self._sampler = sampler

    def update_state(self, phase, *, faulted=False, detail=""):
        """Bounded in-memory handoff, never ROS work or a service wait."""
        with self._lock:
            previous, revision, _, _, _ = self._state
            self._state = (phase, revision + (phase != previous), faulted,
                           detail, time.monotonic_ns())
        if previous == "TELEOP" and phase != "TELEOP" or faulted:
            self.abort()

    def command(self, key):
        command = {"r": "start", "s": "save", "d": "discard"}.get(key.lower())
        if command is None:
            return False
        with self._lock:
            phase, revision, faulted, _, _ = self._state
        if not self.collection or self._draining or phase != "TELEOP" or faulted:
            return False
        try:
            self._commands.put_nowait((command, self.session_id, revision))
            return True
        except queue.Full:
            self._dropped += 1
            return False

    def abort(self):
        # Abort cannot be lost behind a full recording queue.
        if self.collection:
            self._abort.set()

    def status(self):
        with self._lock:
            status = self._status
            error = self._error or self._collection_error
        if error is not None:
            raise RuntimeError(f"executor DDS unavailable: {error}")
        if status is None or not 0 <= time.monotonic_ns() - status.published_monotonic_ns <= 300_000_000:
            return None
        return status

    def request(self, operation, *arguments, timeout=3.0):
        """Startup/teardown only; the control loop never calls this method."""
        result = Future()
        self._requests.put_nowait((operation, arguments, result, time.monotonic() + timeout))
        return result.result(timeout=timeout + 0.1)

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(5)
            if self._thread.is_alive():
                raise RuntimeError("executor DDS thread did not stop")

    @staticmethod
    def _service_owners(node, service):
        owners = []
        for name, namespace in node.get_node_names_and_namespaces():
            for candidate, types in node.get_service_names_and_types_by_node(name, namespace):
                if candidate == service:
                    owners.append((namespace.rstrip("/") + "/" + name, types))
        return owners

    def _validate_collector(self, node):
        nodes = node.get_node_names_and_namespaces()
        if sum(name == "data_collector" and namespace == "/" for name, namespace in nodes) != 1:
            raise RuntimeError("collection node identity missing or ambiguous")
        for service, kind in (
                ("/tianji/collection/command", "tianji_interfaces/srv/RecordingCommand"),
                ("/tianji/collection/check_ready", "std_srvs/srv/Trigger"),
                ("/data_collector/get_parameters", "rcl_interfaces/srv/GetParameters")):
            if self._service_owners(node, service) != [("/data_collector", [kind])]:
                raise RuntimeError(f"collection service identity missing or ambiguous: {service}")
        endpoints = node.get_publishers_info_by_topic("/tianji/collection/status")
        if (len(endpoints) != 1 or endpoints[0].node_name != "data_collector"
                or endpoints[0].node_namespace != "/"):
            raise RuntimeError("collection status publisher identity missing or ambiguous")
        gid = bytes(endpoints[0].endpoint_gid)
        with self._lock:
            if self._status_gid is not None and self._status_gid != gid:
                raise RuntimeError("collection status publisher changed")
        return gid

    def _receive_status(self, message, publisher_gid):
        with self._lock:
            if self._status_gid is None:
                return  # Unverified transient-local status cannot prepare hardware.
            if publisher_gid != self._status_gid:
                self._collection_error = "collection status publisher changed or is ambiguous"
                return
            now = time.monotonic_ns()
            if not 0 <= now - message.published_monotonic_ns <= 300_000_000:
                return
            previous = self._status
            if previous is not None and message.published_monotonic_ns <= previous.published_monotonic_ns:
                return
            if message.session_id not in ("", self.session_id):
                self._collection_error = "collection status belongs to a different executor session"
                return
            if self._status is not None and self._status.session_id and not message.session_id:
                self._collection_error = "collection status lost executor session identity"
                return
            self._status = message
        current = (message.state, message.last_saved_path, message.error)
        if previous is None or current != (previous.state, previous.last_saved_path, previous.error):
            print(f"COLLECTION {message.state}: saved={message.last_saved_path or '-'} "
                  f"error={message.error or '-'}", flush=True)

    def _run(self):
        context = node = executor = None
        pending = []
        try:
            os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
            os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
            os.environ.setdefault("ROS_DOMAIN_ID", "120")
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            from rcl_interfaces.srv import GetParameters
            from std_srvs.srv import Trigger
            from tianji_interfaces.msg import DeviceFeedback, ExecutorState, CollectionStatus
            from tianji_interfaces.srv import RecordingCommand

            context = Context()
            # No global signal handlers: the foreground executor owns SIGINT/SIGTERM.
            context.init(args=[])
            node = Node("tianji_executor_" + self.session_id.replace("-", ""), context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            feedback_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            state_qos = QoSProfile(depth=8, reliability=ReliabilityPolicy.RELIABLE)
            status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
            publishers = {name: node.create_publisher(DeviceFeedback, f"/tianji/feedback/{name}", feedback_qos)
                          for name in ("arms", "left_hand", "right_hand")}
            state_pub = node.create_publisher(ExecutorState, "/tianji/executor/state", state_qos)

            def receive_status(message):
                # Jazzy callback MessageInfo does not expose publisher_gid.
                # Resolve the unique graph endpoint on the DDS thread instead,
                # using the same ownership checks as service dispatch.
                if self._status_gid is None:
                    return
                try:
                    gid = self._validate_collector(node)
                except RuntimeError as error:
                    with self._lock:
                        self._collection_error = str(error)
                    return
                self._receive_status(message, gid)

            node.create_subscription(CollectionStatus, "/tianji/collection/status", receive_status, status_qos)
            command_client = node.create_client(RecordingCommand, "/tianji/collection/command")
            clients = {}
            state_sequence = 0
            feedback_sequences = {name: 0 for name in publishers}
            feedback_tokens = {}
            last_state = None
            next_state = 0.0
            next_identity = 0.0
            self._ready.set()
            while not self._stop.is_set():
                executor.spin_once(timeout_sec=0.002)
                now = time.monotonic()
                if self._status_gid is not None and now >= next_identity:
                    try:
                        self._validate_collector(node)
                    except RuntimeError as error:
                        with self._lock:
                            self._collection_error = str(error)
                    next_identity = now + 0.25
                with self._lock:
                    phase, revision, faulted, detail, touched = self._state
                    sampler = self._sampler
                # A blocked control loop must not be advertised as healthy TELEOP.
                if phase == "TELEOP" and time.monotonic_ns() - touched > 300_000_000:
                    faulted, detail = True, "executor control heartbeat stale"
                state = (phase, revision, faulted, detail)
                if self._collection_active and (now >= next_state or state != last_state):
                    state_sequence += 1
                    state_pub.publish(ExecutorState(
                        boot_id=self.boot_id, session_id=self.session_id, sequence=state_sequence,
                        phase_revision=revision, published_monotonic_ns=time.monotonic_ns(),
                        mode=self.mode, phase=phase, faulted=faulted, detail=detail))
                    last_state, next_state = state, now + 1 / 30
                batch = sampler.drain() if sampler is not None else None
                for name, value in (batch or {}).items():
                    token = (value.received_monotonic_ns, value.healthy, value.enabled, value.detail)
                    if feedback_tokens.get(name) == token:
                        continue
                    feedback_tokens[name] = token
                    feedback_sequences[name] += 1
                    publishers[name].publish(DeviceFeedback(
                        boot_id=self.boot_id, session_id=self.session_id,
                        sequence=feedback_sequences[name], published_monotonic_ns=time.monotonic_ns(),
                        source_monotonic_ns=value.received_monotonic_ns, device=name,
                        position_rad=value.position_rad, healthy=value.healthy,
                        enabled=value.enabled, detail=value.detail))
                if self._abort.is_set() and not any(label == "abort" for _, _, _, label in pending):
                    self._abort.clear()
                    while True:
                        try:
                            self._commands.get_nowait()
                        except queue.Empty:
                            break
                    request = ("abort", self.session_id, revision)
                elif not any(result is None for _, _, result, _ in pending):
                    try:
                        request = self._commands.get_nowait()
                    except queue.Empty:
                        request = None
                else:
                    request = None
                if request is not None:
                    command, session, command_revision = request
                    try:
                        if self._status_gid is None:
                            raise RuntimeError("collector has not been verified")
                        self._validate_collector(node)
                        if command != "abort" and (self._draining or self._collection_error):
                            raise RuntimeError(self._collection_error or "collector is draining")
                        if not command_client.service_is_ready():
                            raise RuntimeError("service unavailable")
                        future = command_client.call_async(RecordingCommand.Request(
                            command=command, session_id=session, phase_revision=command_revision))
                        pending.append((future, now + 2, None, command))
                    except RuntimeError as error:
                        print(f"COLLECTION {command} not sent: {error}; no automatic retry", flush=True)
                if self._dropped:
                    print("COLLECTION key rejected: command queue full", flush=True)
                    self._dropped = 0
                try:
                    operation, arguments, result, deadline = self._requests.get_nowait()
                except queue.Empty:
                    operation = None
                if operation is not None:
                    try:
                        if operation == "graph":
                            result.set_result((node.get_node_names_and_namespaces(), node.get_service_names_and_types()))
                        elif operation == "service_owners":
                            result.set_result(self._service_owners(node, arguments[0]))
                        elif operation == "bind_collector":
                            gid = self._validate_collector(node)
                            with self._lock:
                                self._status_gid = gid
                            result.set_result(True)
                        elif operation == "activate_collection":
                            self._validate_collector(node)
                            status = self.status()
                            if (status is None or status.state != "IDLE" or not status.prepared
                                    or status.error or status.session_id not in ("", self.session_id)):
                                raise RuntimeError("collector no longer idle and prepared")
                            self._collection_active = True
                            result.set_result(True)
                        elif operation == "drain_collection":
                            self._draining = True
                            self.update_state("STOPPING")
                            self.abort()
                            result.set_result(True)
                        elif operation == "collection_pending":
                            result.set_result(self._abort.is_set() or not self._commands.empty()
                                              or any(reply is None for _, _, reply, _ in pending))
                        else:
                            if operation not in ("parameters", "trigger"):
                                raise ValueError(f"unsupported observer request: {operation}")
                            service, *rest = arguments
                            service_type = GetParameters if operation == "parameters" else Trigger
                            client = clients.get(service)
                            if client is None:
                                client = clients[service] = node.create_client(service_type, service)
                            if not client.service_is_ready():
                                raise RuntimeError(f"service not available: {service}")
                            message = (GetParameters.Request(names=rest[0]) if operation == "parameters"
                                       else Trigger.Request())
                            pending.append((client.call_async(message), deadline, result, service))
                    except Exception as error:
                        result.set_exception(error)
                for entry in tuple(pending):
                    future, deadline, result, label = entry
                    if not future.done() and now < deadline:
                        continue
                    pending.remove(entry)
                    try:
                        if not future.done():
                            future.cancel()
                            raise TimeoutError(f"{label}: operation result UNKNOWN; inspect collection status, do not retry")
                        response = future.result()
                        if result is not None:
                            result.set_result(response)
                        else:
                            print(f"COLLECTION {label}: accepted={response.accepted} "
                                  f"state={response.state} {response.message}", flush=True)
                    except Exception as error:
                        if result is not None:
                            result.set_exception(error)
                        else:
                            print(f"COLLECTION {error}", flush=True)
        except Exception as error:
            with self._lock:
                self._error = str(error)
            print(f"EXECUTOR DDS STOPPED (motion gate unchanged): {error}", flush=True)
        finally:
            self._ready.set()
            while True:
                try:
                    _, _, result, _ = self._requests.get_nowait()
                except queue.Empty:
                    break
                if not result.done():
                    result.set_exception(RuntimeError("executor DDS stopped"))
            for _, _, result, _ in pending:
                if result is not None and not result.done():
                    result.set_exception(RuntimeError("executor DDS stopped"))
            if executor is not None:
                executor.shutdown(timeout_sec=1.0)
            if node is not None:
                node.destroy_node()
            if context is not None and context.ok():
                context.shutdown()
