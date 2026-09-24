"""Own three RGB cameras, the top-to-PICO bridge, and the saved RViz window."""
from __future__ import annotations

import argparse
from contextlib import ExitStack
import fcntl
import hashlib
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import uuid

from tianji_runtime import config_path, workspace
from tianji_runtime.constants import CAMERA_ROLES


class IdentityConflict(RuntimeError):
    """A discovered participant is not the component this entry expects."""


class CameraViews:
    CAMERA = "/tianji_camera_monitor"
    PICO = "/tianji_top_camera_pico"
    READY = {CAMERA: "/tianji/cameras/check_ready", PICO: "/tianji/pico_camera/check_ready"}

    def __init__(self, node, executor, config, config_bytes, selection, timeout):
        self.node = node
        self.executor = executor
        self.config = config
        self.config_bytes = config_bytes
        self.selection = selection
        self.timeout = timeout
        self.stopping = False
        self.processes = []
        self.clients = {}
        self.started = False
        self.certified = False
        self.degraded = {}
        self.rviz_node = "/tianji_camera_views_rviz_" + uuid.uuid4().hex
        digest = hashlib.sha256(config_bytes).hexdigest()
        shared = {"owner_token": "", "config_path": str(config)}
        self.expected = {
            self.CAMERA: {**shared, "config_digest": digest},
            self.PICO: {**shared, "config_sha256": digest, "top_serial": selection["top"]},
        }

    def check_workers(self):
        if self.stopping:
            raise KeyboardInterrupt
        if not self.node.context.ok():
            raise RuntimeError("camera views DDS context stopped")
        for label, process in self.processes:
            if process.poll() is not None:
                detail = f"owned {label} exited with status {process.returncode}"
                if not self.started or label not in ("PICO video bridge", "RViz"):
                    raise RuntimeError(detail)
                self.mark_degraded(label, detail)

    def mark_degraded(self, label, detail):
        if label not in self.degraded:
            print(f"CAMERA_VIEWS_DEGRADED: {label}: {detail}; "
                  "camera acquisition remains supervised; recording readiness is not certified",
                  file=sys.stderr, flush=True)
        self.degraded[label] = detail
        self.certified = False

    def mark_recovered(self, label):
        if label in self.degraded:
            del self.degraded[label]
            print(f"CAMERA_VIEWS_RECOVERED: {label}; checking all components before certification",
                  flush=True)

    def worker_alive(self, label):
        return all(process.poll() is None for name, process in self.processes if name == label)

    def report_ready(self):
        if not self.degraded and not self.certified:
            print("CAMERA_VIEWS_READY: left | top | right in RViz; top-to-PICO bridge prepared "
                  "(headset display not certified). Run bash bash/run_teleop.sh --data separately.",
                  flush=True)
            self.certified = True

    def spin(self, timeout=.05):
        self.check_workers()
        self.executor.spin_once(timeout_sec=timeout)

    def names(self):
        return [namespace.rstrip("/") + "/" + name
                for name, namespace in self.node.get_node_names_and_namespaces()]

    def service_owners(self, service):
        owners = []
        for name, namespace in self.node.get_node_names_and_namespaces():
            for candidate, types in self.node.get_service_names_and_types_by_node(name, namespace):
                if candidate == service:
                    owners.append((namespace.rstrip("/") + "/" + name, types))
        return owners

    def require_service(self, owner, service, kind):
        owners = self.service_owners(service)
        if not owners:
            raise TimeoutError(f"service not discovered: {service}")
        if owners != [(owner, [kind])]:
            raise IdentityConflict(f"conflicting service owners for {service}: {owners}")

    def request(self, kind, service, request, deadline):
        client = self.clients.get(service)
        if client is None:
            client = self.node.create_client(kind, service)
            self.clients[service] = client
        if not client.service_is_ready():
            raise TimeoutError(f"service not available: {service}")
        future = client.call_async(request)
        try:
            while not future.done():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"service response timed out: {service}")
                self.spin(min(.05, remaining))
            result = future.result()
            if result is None:
                raise RuntimeError(f"empty service response: {service}")
            return result
        finally:
            if not future.done():
                client.remove_pending_request(future)
                future.cancel()

    def identity(self, owner, deadline):
        from rcl_interfaces.srv import DescribeParameters, GetParameters

        count = self.names().count(owner)
        if count > 1:
            raise IdentityConflict(f"ambiguous node identity: {owner}")
        if count == 0:
            raise TimeoutError(f"node not discovered: {owner}")
        for service, kind in (
                (self.READY[owner], "std_srvs/srv/Trigger"),
                (owner + "/get_parameters", "rcl_interfaces/srv/GetParameters"),
                (owner + "/describe_parameters", "rcl_interfaces/srv/DescribeParameters")):
            self.require_service(owner, service, kind)
        expected = self.expected[owner]
        response = self.request(GetParameters, owner + "/get_parameters",
                                GetParameters.Request(names=list(expected)), deadline)
        actual = {key: value.string_value for key, value in zip(expected, response.values)
                  if value.type == 4}
        if len(response.values) != len(expected) or actual != expected:
            raise IdentityConflict(f"identity/configuration mismatch for {owner}: {actual}")
        response = self.request(DescribeParameters, owner + "/describe_parameters",
                                DescribeParameters.Request(names=list(expected)), deadline)
        if (len(response.descriptors) != len(expected) or
                any(not descriptor.read_only or descriptor.type != 4
                    for descriptor in response.descriptors)):
            raise IdentityConflict(f"identity parameters are not immutable strings: {owner}")

    def ready(self, owner, deadline):
        from std_srvs.srv import Trigger

        self.identity(owner, deadline)
        response = self.request(Trigger, self.READY[owner], Trigger.Request(), deadline)
        if not response.success:
            raise TimeoutError(f"{owner}: {response.message}")
        # Recheck the graph after RPCs: a duplicate must not certify this session.
        count = self.names().count(owner)
        if count == 0:
            raise TimeoutError(f"node disappeared during readiness: {owner}")
        if count > 1:
            raise IdentityConflict(f"node identity changed during readiness: {owner}")
        self.require_service(owner, self.READY[owner], "std_srvs/srv/Trigger")

    def wait_ready(self, owner):
        deadline = time.monotonic() + self.timeout
        detail = "not yet discovered"
        while time.monotonic() < deadline:
            self.check_workers()
            try:
                self.ready(owner, deadline)
                self.check_workers()
                return
            except TimeoutError as error:
                detail = str(error)
            self.spin()
        raise RuntimeError(f"{owner} readiness timed out after {self.timeout:g}s: {detail}")

    def reject_existing(self):
        # Let DDS discovery converge before starting any hardware or video worker.
        deadline = time.monotonic() + min(2.0, self.timeout)
        while True:
            names = self.names()
            conflicts = [name for name in names if name in self.READY or
                         name.startswith("/cameras/") or
                         name.startswith("/tianji_camera_views_rviz_")]
            for service in self.READY.values():
                conflicts.extend(owner for owner, _ in self.service_owners(service))
            for role in CAMERA_ROLES:
                if self.node.get_publishers_info_by_topic(f"/cameras/{role}/color/image_raw"):
                    conflicts.append(f"/cameras/{role}/color/image_raw publisher")
            if conflicts:
                raise IdentityConflict("existing camera/video session: " + ", ".join(sorted(set(conflicts))) +
                                       "; stop its old entry before running bash/run_camera_views.sh")
            if time.monotonic() >= deadline:
                break
            self.spin()
        # Do not connect to the headset control listener as a probe.
        from .pico_video_protocol import CONTROL_PORT
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind(("127.0.0.1", CONTROL_PORT))
            except OSError as error:
                raise IdentityConflict(f"PICO control port {CONTROL_PORT} is occupied; stop the old "
                                       "video entry before running bash/run_camera_views.sh") from error

    def launch(self, label, command):
        self.check_workers()
        process = subprocess.Popen(command, stdin=subprocess.DEVNULL, start_new_session=True)
        self.processes.append((label, process))

    def rviz_ready(self):
        from rclpy.qos import ReliabilityPolicy

        self.check_workers()
        count = self.names().count(self.rviz_node)
        if count > 1:
            raise IdentityConflict(f"ambiguous RViz identity: {self.rviz_node}")
        if count == 0:
            self.rviz_detail = "owned RViz node not discovered"
            return False
        for role in CAMERA_ROLES:
            topic = f"/cameras/{role}/color/image_raw"
            endpoints = [
                endpoint for endpoint in self.node.get_subscriptions_info_by_topic(topic)
                if endpoint.node_namespace.rstrip("/") + "/" + endpoint.node_name == self.rviz_node
            ]
            if not endpoints:
                self.rviz_detail = f"owned RViz image subscription not discovered: {topic}"
                return False
            if any(endpoint.topic_type != "sensor_msgs/msg/Image" or
                   endpoint.qos_profile.reliability != ReliabilityPolicy.BEST_EFFORT
                   for endpoint in endpoints):
                raise IdentityConflict(
                    f"owned RViz subscription must use Image with Best Effort reliability: {topic}")
        self.rviz_detail = ""
        return True

    def check_config(self):
        if self.config.read_bytes() != self.config_bytes:
            raise IdentityConflict("camera configuration changed during this session; stop and restart the image entry")

    def run(self, executable, layout):
        self.reject_existing()
        self.launch("cameras", ["bash", str(workspace() / "bash/run_cameras.sh"),
                                "--config", str(self.config), "--timeout", str(self.timeout)])
        self.wait_ready(self.CAMERA)
        self.check_config()
        self.launch("PICO video bridge", [sys.executable, "-m", "tianji_cameras.pico_stream",
                                         "--config", str(self.config), "--timeout", str(self.timeout)])
        self.wait_ready(self.PICO)
        self.launch("RViz", [executable, "-d", str(layout), "--ros-args",
                             "-r", f"__node:={self.rviz_node.lstrip('/')}", "-r", "__ns:=/"])
        deadline = time.monotonic() + self.timeout
        while not self.rviz_ready():
            if time.monotonic() >= deadline:
                raise RuntimeError(f"owned RViz subscriptions not ready before timeout: {self.rviz_detail}")
            self.spin()
        self.check_health()
        self.started = True
        self.report_ready()
        deadline = time.monotonic() + 1.0
        while True:
            self.spin(.1)
            if time.monotonic() >= deadline:
                self.check_health()
                deadline = time.monotonic() + 1.0

    def check_health(self):
        self.check_workers()
        self.check_config()
        for owner, label in ((self.CAMERA, "cameras"), (self.PICO, "PICO video bridge")):
            if not self.worker_alive(label):
                continue
            try:
                self.ready(owner, time.monotonic() + min(3.0, self.timeout))
            except TimeoutError as error:
                if not self.started:
                    raise
                self.mark_degraded(label, str(error))
            else:
                self.check_workers()
                if self.worker_alive(label):
                    self.mark_recovered(label)
        if self.worker_alive("RViz"):
            if self.rviz_ready():
                self.mark_recovered("RViz")
            elif self.started:
                self.mark_degraded("RViz", self.rviz_detail)
            else:
                raise TimeoutError(self.rviz_detail)
        self.check_workers()
        if self.started:
            self.report_ready()

    def close(self):
        errors = []
        for label, process in reversed(self.processes):
            try:
                # The group can outlive its leader. Never skip surviving owned
                # descendants merely because the wrapper has already exited.
                for sig, timeout in ((signal.SIGINT, 15.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 5.0)):
                    process.poll()
                    try:
                        os.killpg(process.pid, sig)
                    except ProcessLookupError:
                        break
                    deadline = time.monotonic() + timeout
                    while time.monotonic() < deadline:
                        process.poll()
                        try:
                            os.killpg(process.pid, 0)
                        except ProcessLookupError:
                            break
                        time.sleep(.05)
                    else:
                        continue
                    break
                else:
                    raise RuntimeError("owned process group did not exit after SIGKILL")
                process.wait(timeout=1)
            except Exception as error:
                errors.append(f"{label} cleanup failed: {error}")
        self.processes.clear()
        if errors:
            raise RuntimeError("; ".join(errors))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=None, help="collection config with all three camera roles enabled")
    parser.add_argument("--timeout", type=float, default=45.0,
                        help="readiness timeout for each owned component, 0 < seconds <= 300 (default: 45)")
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or not 0 < args.timeout <= 300:
        parser.error("timeout must be finite and greater than zero, at most 300 seconds")
    try:
        # Keep --help independent of ROS, GUI, ADB and camera hardware.
        from data_collector.config import load_collection_config

        config = Path(args.config or config_path("collect_real.json")).resolve()
        config_bytes = config.read_bytes()
        _, selection = load_collection_config(config, source_bytes=config_bytes)
        if set(selection) != set(CAMERA_ROLES):
            raise RuntimeError("camera views requires top, left_wrist and right_wrist; none may be disabled")
        executable = shutil.which("rviz2")
        if executable is None:
            raise RuntimeError("camera views requires rviz2 on PATH")
        layout = workspace() / "config/cameras.rviz"
        if not layout.is_file():
            raise RuntimeError(f"saved RViz layout is missing: {layout}")
        platform = os.environ.get("QT_QPA_PLATFORM", "").split(":")[0]
        if platform not in ("", "xcb", "wayland", "wayland-egl"):
            raise RuntimeError("camera views requires a visible X11 or Wayland RViz window")
        if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            raise RuntimeError("camera views requires a graphical DISPLAY or WAYLAND_DISPLAY")
        if shutil.which("adb") is None:
            raise RuntimeError("ADB is required for wired PICO video; install adb first")

        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node

        with ExitStack() as cleanup:
            domain = os.environ.get("ROS_DOMAIN_ID", "120")
            lock_path = Path(tempfile.gettempdir()) / f"tianji-camera-views-{os.getuid()}-{domain}.lock"
            lock = cleanup.enter_context(lock_path.open("a"))
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise IdentityConflict("another camera views entry is running; stop the old entry first") from error
            context = Context()
            context.init(args=[])
            cleanup.callback(context.try_shutdown)
            node = Node("tianji_camera_views_" + uuid.uuid4().hex, context=context,
                        start_parameter_services=False, enable_rosout=False)
            cleanup.callback(node.destroy_node)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            cleanup.callback(executor.shutdown, timeout_sec=2)
            views = CameraViews(node, executor, config, config_bytes, selection, args.timeout)
            for sig in (signal.SIGINT, signal.SIGTERM):
                previous = signal.signal(sig, lambda *_: setattr(views, "stopping", True))
                cleanup.callback(signal.signal, sig, previous)
            cleanup.callback(views.close)
            try:
                views.run(executable, layout)
            except KeyboardInterrupt:
                pass
        return 0
    except Exception as error:
        print(f"CAMERA_VIEWS_ERROR: {error}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
