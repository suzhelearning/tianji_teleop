"""Verified collection reuse and cleanup of only this executor's workers."""
from __future__ import annotations

import fcntl
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

from tianji_runtime import workspace


class IdentityConflict(RuntimeError):
    """An observed participant cannot safely belong to this collection."""


class CollectionSupervisor:
    CAMERA = "/tianji_camera_monitor"
    COLLECTOR = "/data_collector"
    SERVICES = {
        CAMERA: {"/tianji/cameras/check_ready": "std_srvs/srv/Trigger"},
        COLLECTOR: {"/tianji/collection/check_ready": "std_srvs/srv/Trigger",
                    "/tianji/collection/command": "tianji_interfaces/srv/RecordingCommand"},
    }

    def __init__(self, observer, dataset, task, config, model):
        self.observer = observer
        self.dataset = Path(dataset).resolve()
        self.task = task
        self.config = Path(config).resolve()
        self.model = Path(model).resolve()
        self._processes = []
        self._participants = {}
        self._collector_bound = False
        self._lock = None
        self._failure_reported = False

    def _launch(self, label, command, shutdown_timeout):
        process = subprocess.Popen(command, stdin=subprocess.DEVNULL, start_new_session=True)
        self._processes.append((label, process, shutdown_timeout))

    def _check_workers(self):
        for label, process, _ in self._processes:
            if process.poll() is not None:
                raise RuntimeError(f"owned {label} exited with status {process.returncode}")

    def _wait(self, predicate, description, timeout=15.0):
        deadline = time.monotonic() + timeout
        reason = "not yet ready"
        while time.monotonic() < deadline:
            self._check_workers()
            try:
                if predicate():
                    return
            except IdentityConflict:
                raise
            except (RuntimeError, TimeoutError) as error:
                reason = str(error)
            time.sleep(0.05)
        raise RuntimeError(f"{description} timed out: {reason}")

    @staticmethod
    def _names(nodes):
        return [namespace.rstrip("/") + "/" + name for name, namespace in nodes]

    def _expected(self, node, token):
        expected = {"owner_token": token, "config_path": str(self.config),
                    "config_digest": hashlib.sha256(self.config.read_bytes()).hexdigest()}
        if node == self.COLLECTOR:
            expected.update(dataset_path=str(self.dataset), task=self.task,
                            model_path=str(self.model),
                            model_digest=hashlib.sha256(self.model.read_bytes()).hexdigest())
        return expected

    def _identity(self, node, token):
        nodes, _ = self.observer.request("graph")
        count = self._names(nodes).count(node)
        if count > 1:
            raise IdentityConflict(f"ambiguous identity: {node}")
        if count == 0:
            raise RuntimeError(f"missing identity: {node}")
        services = {node + "/get_parameters": "rcl_interfaces/srv/GetParameters",
                    **self.SERVICES[node]}
        for service, kind in services.items():
            owners = self.observer.request("service_owners", service)
            if not owners:
                raise RuntimeError(f"service not discovered: {service}")
            if owners != [(node, [kind])]:
                raise IdentityConflict(f"conflicting service owners for {service}: {owners}")
        expected = self._expected(node, token)
        response = self.observer.request("parameters", node + "/get_parameters", list(expected))
        actual = {key: value.string_value for key, value in zip(expected, response.values)
                  if value.type == 4}  # rcl_interfaces/ParameterType.PARAMETER_STRING
        if actual != expected:
            raise IdentityConflict(f"worker identity/configuration mismatch for {node}: {actual}")
        return True

    def _discover(self):
        # Give remote graph caches time to converge before deciding to create a
        # worker. Identity and service ownership are rechecked after launching.
        deadline = time.monotonic() + 2.0
        while True:
            nodes, services = self.observer.request("graph")
            names = self._names(nodes)
            for node in self.SERVICES:
                if names.count(node) > 1:
                    raise IdentityConflict(f"ambiguous identity: {node}")
            if time.monotonic() >= deadline:
                break
            time.sleep(0.05)
        present = {node for node in self.SERVICES if node in names}
        reserved = {service for mapping in self.SERVICES.values() for service in mapping}
        for service, _ in services:
            if service in reserved:
                owners = self.observer.request("service_owners", service)
                expected_node = next(node for node, mapping in self.SERVICES.items() if service in mapping)
                if not owners or any(owner != expected_node for owner, _ in owners):
                    raise IdentityConflict(f"conflicting service identity: {service}: {owners}")
                if expected_node not in present:
                    raise IdentityConflict(f"service without unique node identity: {service}")
        if self.CAMERA not in present and (
                self.COLLECTOR in present or any(ns == "/cameras" for _, ns in nodes)):
            raise IdentityConflict("existing camera/collector has no certifying camera monitor")
        return present

    def start(self):
        domain = os.environ.get("ROS_DOMAIN_ID", "120")
        lock_path = Path(tempfile.gettempdir()) / f"tianji-collection-{os.getuid()}-{domain}.lock"
        self._lock = lock_path.open("a")
        try:
            fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError("another executor owns collection startup in this DDS domain") from error
        present = self._discover()
        # Validate every existing participant before creating anything. An empty
        # token denotes standalone, never permission to take over a foreign owner.
        for node in present:
            self._wait(lambda node=node: self._identity(node, ""), "standalone identity")
            self._participants[node] = ""
        token = self.observer.session_id
        if self.CAMERA not in present:
            self._launch("cameras", ["bash", str(workspace() / "bash/run_cameras.sh"),
                                     "--config", str(self.config), "--owner-token", token], 15.0)
            self._participants[self.CAMERA] = token
        self._wait(lambda: self._identity(self.CAMERA, self._participants[self.CAMERA]), "camera identity")
        self._wait(lambda: self.observer.request("trigger", "/tianji/cameras/check_ready").success,
                   "validated camera profiles and streams")
        if self.COLLECTOR not in present:
            self._launch("collector", [sys.executable, "-m", "data_collector.node",
                                       "--dataset", str(self.dataset), "--task", self.task,
                                       "--config", str(self.config), "--model", str(self.model),
                                       "--owner-token", token], 300.0)
            self._participants[self.COLLECTOR] = token
        self._wait(lambda: self._identity(self.COLLECTOR, self._participants[self.COLLECTOR]), "collector identity")
        self._wait(lambda: self.observer.request("bind_collector"), "unique collector status publisher")
        self._wait(self._prepared, "collector writer/camera preparation")
        # Recheck current bytes and all service owners at the SDK-connect boundary.
        for node, owner in self._participants.items():
            self._identity(node, owner)
        self.observer.request("activate_collection")
        self._collector_bound = True

    def _prepared(self):
        status = self.observer.status()
        if status is None:
            return False
        if status.session_id not in ("", self.observer.session_id) or status.state != "IDLE":
            raise IdentityConflict("collector belongs to an active or different executor session")
        return status.prepared and not status.error

    def wait_inputs_ready(self):
        def ready():
            status = self.observer.status()
            return (status is not None and status.session_id == self.observer.session_id
                    and status.prepared and status.inputs_ready and not status.error)
        self._wait(ready, "collector fresh executor feedback")

    def check_before_enable(self):
        self._check_workers()
        status = self.observer.status()
        if not (status is not None and status.session_id == self.observer.session_id
                and status.prepared and status.inputs_ready and not status.error):
            raise RuntimeError("collection readiness revoked before motor authorization")

    def check(self):
        # No graph/RPC/file work on the motion loop. A failed recorder is not a
        # second motion authority: report/abort, leaving the safety gate untouched.
        try:
            self.check_before_enable()
        except RuntimeError as error:
            if not self._failure_reported:
                print(f"COLLECTION UNAVAILABLE (robot control unchanged): {error}", flush=True)
                self.observer.abort()
                self._failure_reported = True

    def _drain(self):
        self.observer.request("drain_collection")
        deadline = time.monotonic() + 300.0
        while time.monotonic() < deadline:
            status = self.observer.status()
            if (status is not None and status.session_id in ("", self.observer.session_id)
                    and status.state in ("IDLE", "FAILED", "CLOSED")
                    and not self.observer.request("collection_pending")):
                return
            time.sleep(0.05)
        raise RuntimeError("collector did not drain this executor's recording within 300s")

    def finish(self):
        errors = []
        try:
            if self._collector_bound:
                try:
                    self._drain()
                except Exception as error:
                    errors.append(f"collector session drain failed: {error}")
                self._collector_bound = False
            for label, process, timeout in reversed(self._processes):
                try:
                    if process.poll() is None:
                        for sig, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
                            try:
                                os.killpg(process.pid, sig)
                            except ProcessLookupError:
                                pass
                            try:
                                process.wait(timeout=wait)
                                break
                            except subprocess.TimeoutExpired:
                                errors.append(f"{label} shutdown exceeded {wait}s after {sig.name}")
                    if process.returncode not in (0, -signal.SIGINT):
                        errors.append(f"{label} exited with status {process.returncode}")
                except Exception as error:
                    errors.append(f"{label} cleanup failed: {error}")
            self._processes.clear()
        finally:
            if self._lock is not None:
                self._lock.close()
                self._lock = None
        if errors:
            raise RuntimeError("; ".join(errors))
