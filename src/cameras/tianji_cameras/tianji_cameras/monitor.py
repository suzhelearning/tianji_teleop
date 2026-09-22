"""Certify effective official-driver settings, identity and fresh RGB/metadata pairs."""
from __future__ import annotations

import argparse
import asyncio
from functools import partial
import hashlib
from pathlib import Path
import signal
import threading
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterEvent, ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_parameter_events,
)
from realsense2_camera_msgs.msg import Metadata
from realsense2_camera_msgs.srv import DeviceInfo
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from tianji_runtime import config_path, workspace
from tianji_runtime.camera_stream import (
    CameraStreamValidator, ImageRecord, MetadataJsonParser, StreamFault, split_stamp,
)
from tianji_runtime.constants import CAMERA_FPS, IMAGE_HEIGHT, IMAGE_WIDTH
from .preflight import load_camera_selection, uses_depth_module_color

IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=5)
READY_SERVICE = "/tianji/cameras/check_ready"
REVALIDATE_PERIOD_S = 1.0
CERTIFICATE_TIMEOUT_S = 2.0


def _parse_profile(value):
    parts = value.lower().replace("x", ",").split(",")
    try:
        return tuple(float(part.strip()) for part in parts) if len(parts) == 3 else None
    except ValueError:
        return None


class _RoleState:
    def __init__(self, role, serial):
        self.role, self.serial = role, serial
        self.generation = 0
        self.channel = None
        self.profile = self.format = None
        self.parameter_time = self.identity_time = 0.0
        self.pending = {}
        self.frames = 0
        self.started = time.monotonic()
        self.ready = False
        self.detail = "waiting for driver identity, effective parameters and a complete pair"
        self.reset_stream()

    def reset_stream(self):
        self.validator = CameraStreamValidator(self.role, width=IMAGE_WIDTH, height=IMAGE_HEIGHT)
        self.metadata = MetadataJsonParser(self.role)

    def revoke(self, reason, *, reset=False):
        self.ready, self.detail = False, reason
        if reset:
            self.generation += 1
            self.parameter_time = self.identity_time = 0.0
            self.reset_stream()


class CameraMonitor(Node):
    def __init__(self, selection, *, config=None, owner_token=""):
        super().__init__("tianji_camera_monitor")
        from data_collector.config import load_collection_config
        self.config = Path(config or config_path("collect_real.json")).resolve()
        config_bytes = self.config.read_bytes()
        _, loaded_selection = load_collection_config(self.config, source_bytes=config_bytes)
        if dict(selection) != loaded_selection:
            raise ValueError("camera selection does not match the loaded collection config")
        descriptor = ParameterDescriptor(read_only=True)
        self.declare_parameter("config_path", str(self.config), descriptor, ignore_override=True)
        self.declare_parameter("config_digest", hashlib.sha256(config_bytes).hexdigest(),
                               descriptor, ignore_override=True)
        self.declare_parameter("owner_token", owner_token, descriptor, ignore_override=True)
        self._roles = {role: _RoleState(role, serial) for role, serial in selection.items()}
        self._lock = threading.RLock()
        self._driver_clients = {}
        for role in self._roles:
            base = f"/cameras/{role}"
            self._driver_clients[role] = {
                "identity": self.create_client(DeviceInfo, f"{base}/device_info"),
                "parameters": self.create_client(GetParameters, f"{base}/get_parameters"),
            }
            # Jazzy callback info has no publisher_gid. Pin each topic's unique
            # graph endpoint separately; image and metadata GIDs are different.
            self.create_subscription(Image, f"{base}/color/image_raw",
                                     partial(self._on_image, role), IMAGE_QOS)
            self.create_subscription(Metadata, f"{base}/color/metadata",
                                     partial(self._on_metadata, role), IMAGE_QOS)
        self.create_subscription(ParameterEvent, "/parameter_events", self._on_parameters,
                                 qos_profile_parameter_events)
        self.create_service(Trigger, READY_SERVICE, self._on_ready)
        self.create_timer(REVALIDATE_PERIOD_S, self._revalidate)
        self.create_timer(0.1, self._check_health)

    def build_driver_service(self, on_signal):
        from launch import LaunchDescription, LaunchService
        from launch.actions import IncludeLaunchDescription, OpaqueFunction
        from launch.launch_description_sources import PythonLaunchDescriptionSource
        # Only pure Python source crosses into the isolated driver environment;
        # never source install/default or import its compiled ROS extensions.
        source = workspace() / "src/cameras/tianji_cameras/launch/cameras.launch.py"
        service = LaunchService()

        def route_signals(context):
            # Launch's default SIGTERM cancels run_async and can orphan drivers.
            # Route signals to graceful shutdown before any node starts.
            loop = asyncio.get_running_loop()
            for sig in (signal.SIGINT, signal.SIGTERM):
                loop.add_signal_handler(sig, on_signal)

        service.include_launch_description(LaunchDescription([
            OpaqueFunction(function=route_signals),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(source)),
                                     launch_arguments={"config": str(self.config)}.items())]))
        return service

    def _publisher(self, state, suffix):
        topic = f"/cameras/{state.role}/color/{suffix}"
        endpoints = self.get_publishers_info_by_topic(topic)
        if len(endpoints) != 1:
            raise StreamFault(f"{topic}: expected one publisher, found {len(endpoints)}")
        endpoint = endpoints[0]
        if endpoint.node_name != state.role or endpoint.node_namespace != "/cameras":
            raise StreamFault(f"{topic}: unexpected publisher node identity")
        gid = bytes(endpoint.endpoint_gid)
        if not gid or not any(gid):
            raise StreamFault(f"{topic}: publisher endpoint has no GID")
        if suffix == "image_raw":
            state.validator.note_image_publisher(gid)
        else:
            state.validator.note_metadata_publisher(gid)

    def _stamp(self, message):
        stamp = split_stamp(message.header.stamp)
        age = self.get_clock().now().nanoseconds - stamp
        if stamp <= 0 or age < -5_000_000 or age > 2_000_000_000:
            raise StreamFault(f"invalid camera ROS header age: {age / 1e9:.6f}s")
        return stamp

    def _on_image(self, role, message):
        received = time.monotonic_ns()
        with self._lock:
            state = self._roles[role]
            try:
                self._publisher(state, "image_raw")
                if len(message.data) != IMAGE_WIDTH * IMAGE_HEIGHT * 3:
                    raise StreamFault(f"camera {role}: invalid RGB payload length {len(message.data)}")
                image = ImageRecord(
                    stamp_ns=self._stamp(message), frame_id=message.header.frame_id,
                    width=message.width, height=message.height, step=message.step,
                    encoding=message.encoding, rgb=None)
                if state.validator.accept_image(image, received) is not None:
                    state.frames += 1
            except (StreamFault, ValueError) as error:
                state.revoke(str(error), reset=True)

    def _on_metadata(self, role, message):
        received = time.monotonic_ns()
        with self._lock:
            state = self._roles[role]
            try:
                self._publisher(state, "metadata")
                record = state.metadata.accept(
                    message.header.frame_id, self._stamp(message), message.json_data)
                if state.validator.accept_metadata(record, received) is not None:
                    state.frames += 1
            except (StreamFault, ValueError) as error:
                state.revoke(str(error), reset=True)

    def _on_parameters(self, event):
        with self._lock:
            for role, state in self._roles.items():
                if event.node == f"/cameras/{role}":
                    # Any runtime setting change invalidates the certificate and
                    # buffered frames, including exposure/serial/stream switches.
                    state.revoke("driver parameters changed; rechecking effective configuration", reset=True)

    def _request(self, state, kind, request, accept):
        now = time.monotonic()
        pending = state.pending.get(kind)
        if pending is not None:
            future, sent = pending
            if not future.done() and now - sent < REVALIDATE_PERIOD_S:
                return
            if not future.done():
                future.cancel()
                state.revoke(f"driver {kind} service timed out", reset=True)
        client = self._driver_clients[state.role][kind]
        if not client.service_is_ready():
            state.revoke(f"driver {kind} service unavailable")
            if kind == "identity":
                state.identity_time = 0.0
            else:
                state.parameter_time = 0.0
            return
        generation = state.generation
        future = client.call_async(request)
        state.pending[kind] = (future, now)

        def completed(result):
            with self._lock:
                if state.pending.get(kind, (None,))[0] is not result:
                    return
                state.pending.pop(kind)
                if generation != state.generation or result.cancelled():
                    return
                try:
                    response = result.result()
                    if response is None:
                        raise RuntimeError("empty service response")
                    accept(response)
                except Exception as error:
                    state.revoke(f"driver {kind} verification failed: {error}", reset=True)
        future.add_done_callback(completed)

    def _revalidate(self):
        with self._lock:
            for state in self._roles.values():
                def identity(response, state=state):
                    if response.serial_number != state.serial:
                        raise RuntimeError(f"serial {response.serial_number!r}, expected {state.serial!r}")
                    if not response.device_name:
                        raise RuntimeError("missing device name")
                    state.channel = ("depth_module" if uses_depth_module_color(response.device_name)
                                     else "rgb_camera")
                    state.identity_time = time.monotonic()
                self._request(state, "identity", DeviceInfo.Request(), identity)
                if state.channel is None:
                    continue
                request = GetParameters.Request()
                channel = state.channel
                request.names = [f"{channel}.color_profile", f"{channel}.color_format", "serial_no"]

                def parameters(response, state=state, channel=channel):
                    if len(response.values) != 3 or any(
                            value.type != ParameterType.PARAMETER_STRING for value in response.values):
                        raise RuntimeError(f"required {channel} profile/format/serial parameters are undeclared")
                    profile, fmt, serial = (value.string_value for value in response.values)
                    if (_parse_profile(profile) != (IMAGE_WIDTH, IMAGE_HEIGHT, CAMERA_FPS)
                            or fmt.strip().upper() != "RGB8" or serial != f"_{state.serial}"):
                        raise RuntimeError(f"effective profile={profile!r}, format={fmt!r}, serial={serial!r}; "
                                           f"expected {IMAGE_WIDTH}x{IMAGE_HEIGHT}x{CAMERA_FPS} RGB8 _{state.serial}")
                    state.profile, state.format = profile, fmt
                    state.parameter_time = time.monotonic()
                self._request(state, "parameters", request, parameters)

    def _check_health(self):
        with self._lock:
            now = time.monotonic()
            for state in self._roles.values():
                try:
                    self._publisher(state, "image_raw")
                    self._publisher(state, "metadata")
                    state.validator.check_fresh()
                except StreamFault as error:
                    state.revoke(str(error))
                    continue
                if now - min(state.parameter_time, state.identity_time) > CERTIFICATE_TIMEOUT_S:
                    state.revoke("waiting for fresh serial and effective-parameter verification")
                    continue
                state.ready, state.detail = True, ""

    def _on_ready(self, request, response):
        self._check_health()
        response.success, response.message = self.status()
        return response

    def status(self):
        with self._lock:
            problems = [f"{s.role}: {s.detail}" for s in self._roles.values() if not s.ready]
            return (False, "; ".join(problems)) if problems else (True, "ready")

    def report(self):
        with self._lock:
            return "\n".join(
                f"{s.role}: ready={s.ready} serial={s.serial} channel={s.channel} "
                f"profile={s.profile!r} format={s.format!r} pairs={s.frames} "
                f"paired_rx_fps={s.frames / max(time.monotonic() - s.started, 1e-9):.2f} "
                f"source_sequence_gaps={s.validator.sequence_gaps} "
                f"max_pair_interval_ms={s.validator.max_pair_interval_ns / 1e6:.3f} "
                f"dropped_unpaired={s.validator.dropped_unpaired} detail={s.detail}"
                for s in self._roles.values())


async def _session(service, monitor, *, timeout, duration, should_stop):
    # LaunchService.run_async MUST execute on the main thread's event loop.
    task = asyncio.create_task(service.run_async()) if service else None
    deadline = time.monotonic() + timeout
    ready = False
    unexpected_exit = False
    try:
        while rclpy.ok() and not should_stop():
            if task is not None and task.done():
                code = await task  # Exceptions are intentionally propagated.
                raise RuntimeError(f"camera launch exited before shutdown (code {code})")
            ready, detail = monitor.status()
            if ready:
                break
            if time.monotonic() >= deadline:
                print(f"CAMERAS NOT READY: {detail}", flush=True)
                return False
            await asyncio.sleep(0.05)
        if not ready:
            return False
        print("CAMERAS READY\n" + monitor.report(), flush=True)
        end = time.monotonic() + duration if duration else float("inf")
        while rclpy.ok() and not should_stop() and time.monotonic() < end:
            if task is not None and task.done():
                code = await task
                unexpected_exit = True
                raise RuntimeError(f"camera launch exited before shutdown (code {code})")
            await asyncio.sleep(0.1)
        print(monitor.report(), flush=True)
        return monitor.status()[0]
    finally:
        if task is not None:
            service.shutdown(force_sync=True)
            code = await task  # Wait for driver exit before releasing the session.
            locks = service.context.get_locals_as_dict().get("tianji_camera_locks")
            if locks is not None:
                locks.close()  # Also release serials whose nodes never started.
            if code and not unexpected_exit:
                raise RuntimeError(f"camera launch failed (code {code})")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--launch", action="store_true")
    parser.add_argument("--config", default=None)
    parser.add_argument("--owner-token", default="")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--duration", type=float, default=0.0,
                        help="seconds after readiness; 0 runs until interrupted")
    args = parser.parse_args(argv)
    if args.timeout <= 0 or args.duration < 0:
        parser.error("timeout must be positive and duration nonnegative")
    config = Path(args.config or config_path("collect_real.json")).resolve()
    selection = load_camera_selection(config)
    rclpy.init()
    monitor = CameraMonitor(selection, config=config, owner_token=args.owner_token)
    stopping = threading.Event()
    stop_spinning = threading.Event()
    failures = []
    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    for sig in previous:
        signal.signal(sig, lambda *_: stopping.set())

    def spin():
        try:
            while rclpy.ok() and not stop_spinning.is_set():
                rclpy.spin_once(monitor, timeout_sec=0.1)
        except Exception as error:
            failures.append(error)
            stopping.set()
    spinner = threading.Thread(target=spin, name="camera-monitor-spin")
    spinner.start()
    try:
        service = monitor.build_driver_service(stopping.set) if args.launch else None
        ready = asyncio.run(_session(service, monitor, timeout=args.timeout,
                                     duration=args.duration, should_stop=stopping.is_set))
        if failures:
            raise failures[0]
        return 0 if ready else 1
    finally:
        stop_spinning.set()
        spinner.join()  # No callback may still be executing during node destruction.
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == "__main__":
    raise SystemExit(main())
