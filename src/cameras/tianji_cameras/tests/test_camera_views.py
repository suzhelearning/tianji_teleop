import io
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from tianji_cameras.camera_views import CameraViews, IdentityConflict


class ReadyGraph:
    """In-memory ROS discovery/RPC boundary; supervisor policy stays real."""

    def __init__(self):
        self.context = SimpleNamespace(ok=lambda: True)
        self.expected = {}
        self.health = {}
        self.unavailable = set()
        self.calls = []

    def get_node_names_and_namespaces(self):
        return [(owner.lstrip("/"), "/") for owner in self.expected]

    def get_service_names_and_types_by_node(self, name, namespace):
        owner = "/" + name
        return [(CameraViews.READY[owner], ["std_srvs/srv/Trigger"]),
                (owner + "/get_parameters", ["rcl_interfaces/srv/GetParameters"]),
                (owner + "/describe_parameters", ["rcl_interfaces/srv/DescribeParameters"])]

    def create_client(self, kind, service):
        def call(request):
            self.calls.append(service)
            if service in CameraViews.READY.values():
                owner = next(owner for owner, ready in CameraViews.READY.items() if ready == service)
                detail = self.health.get(owner, "")
                result = SimpleNamespace(success=not detail, message=detail)
            else:
                owner, operation = service.rsplit("/", 1)
                if operation == "get_parameters":
                    result = SimpleNamespace(values=[SimpleNamespace(type=4, string_value=self.expected[owner][key])
                                                     for key in request.names])
                else:
                    result = SimpleNamespace(descriptors=[SimpleNamespace(type=4, read_only=True)
                                                          for _ in request.names])
            return SimpleNamespace(done=lambda: True, result=lambda: result)

        return SimpleNamespace(service_is_ready=lambda: service not in self.unavailable, call_async=call)


class CameraViewsTest(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        config = Path(directory.name) / "collection.json"
        config.write_bytes(b"{}")
        self.graph = ReadyGraph()
        self.executor = SimpleNamespace(spin_once=lambda **kwargs: None)
        self.views = CameraViews(self.graph, self.executor, config, b"{}", {"top": "top-serial"}, 1)
        self.graph.expected = {owner: dict(values) for owner, values in self.views.expected.items()}
        self.addCleanup(self.views.close)
        rviz = patch.object(self.views, "rviz_ready", return_value=True)
        self.rviz = rviz.start()
        self.addCleanup(rviz.stop)
        self.output = io.StringIO()
        self.errors = io.StringIO()
        self.enterContext(redirect_stdout(self.output))
        self.enterContext(redirect_stderr(self.errors))

    def sleeper(self):
        process = subprocess.Popen(
            [sys.executable, "-c", "import time; time.sleep(300)"],
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)
        self.addCleanup(self.stop_process, process)
        return process

    @staticmethod
    def stop_process(process):
        if process.poll() is None:
            process.terminate()
        process.wait(timeout=5)

    def steady_session(self):
        processes = {label: self.sleeper() for label in ("cameras", "PICO video bridge", "RViz")}
        self.views.processes = list(processes.items())
        self.views.check_health()
        self.views.started = True
        self.views.report_ready()
        self.output.truncate(0)
        self.output.seek(0)
        return processes

    def test_pico_exit_during_running_supervisor_keeps_camera_until_explicit_stop(self):
        processes = {}
        foreign = self.sleeper()
        spins = 0

        def launch(label, command):
            process = self.sleeper()
            processes[label] = process
            self.views.processes.append((label, process))

        def spin_once(**kwargs):
            nonlocal spins
            spins += 1
            if spins == 1:
                processes["PICO video bridge"].terminate()
                processes["PICO video bridge"].wait(timeout=5)
                self.graph.expected.pop(self.views.PICO)
                self.graph.calls.clear()
                self.output.truncate(0)
                self.output.seek(0)
            if spins < 3:
                self.views.check_health()
                self.assertIsNone(processes["cameras"].poll())
                self.assertIsNone(processes["RViz"].poll())
            else:
                self.views.stopping = True

        self.executor.spin_once = spin_once
        with patch.object(self.views, "reject_existing"), patch.object(self.views, "launch", side_effect=launch):
            with self.assertRaises(KeyboardInterrupt):
                self.views.run("unused-rviz", Path("unused-layout"))
        self.assertIsNone(processes["cameras"].poll())
        self.assertEqual(self.errors.getvalue().count("CAMERA_VIEWS_DEGRADED:"), 1)
        self.assertNotIn("CAMERA_VIEWS_READY:", self.output.getvalue())
        self.assertNotIn(self.views.READY[self.views.PICO], self.graph.calls)
        self.assertNotIn(self.views.PICO + "/get_parameters", self.graph.calls)
        self.views.close()
        self.assertIsNotNone(processes["cameras"].poll())
        self.assertIsNotNone(processes["RViz"].poll())
        self.assertIsNone(foreign.poll())

    def test_stale_camera_readiness_recovers_without_stopping_camera(self):
        processes = self.steady_session()
        self.graph.health[self.views.CAMERA] = "source stale/future (250 ms freshness limit)"
        self.views.check_health()
        self.views.check_health()
        self.assertIsNone(processes["cameras"].poll())
        self.assertEqual(self.errors.getvalue().count("CAMERA_VIEWS_DEGRADED:"), 1)
        self.assertNotIn("CAMERA_VIEWS_READY:", self.output.getvalue())
        del self.graph.health[self.views.CAMERA]
        self.views.check_health()
        self.views.check_health()
        self.assertEqual(self.output.getvalue().count("CAMERA_VIEWS_RECOVERED:"), 1)
        self.assertEqual(self.output.getvalue().count("CAMERA_VIEWS_READY:"), 1)
        self.assertIsNone(processes["cameras"].poll())

    def test_recovery_of_one_component_does_not_certify_other_degraded_components(self):
        processes = self.steady_session()
        self.graph.unavailable.add(self.views.PICO + "/get_parameters")
        self.graph.health[self.views.CAMERA] = "source stale"
        self.views.check_health()
        self.graph.health.clear()
        self.views.check_health()
        self.assertNotIn("CAMERA_VIEWS_READY:", self.output.getvalue())
        self.assertIsNone(processes["cameras"].poll())
        self.graph.unavailable.clear()
        self.views.check_health()
        self.assertEqual(self.output.getvalue().count("CAMERA_VIEWS_READY:"), 1)

    def test_identity_conflict_remains_fatal_while_auxiliary_is_degraded(self):
        processes = self.steady_session()
        processes["PICO video bridge"].terminate()
        processes["PICO video bridge"].wait(timeout=5)
        self.graph.expected[self.views.CAMERA]["config_digest"] = "foreign-session"
        with self.assertRaises(IdentityConflict):
            self.views.check_health()
        self.assertNotIn("CAMERA_VIEWS_READY:", self.output.getvalue())

    def test_camera_exit_and_context_shutdown_remain_fatal(self):
        processes = self.steady_session()
        self.graph.context.ok = lambda: False
        with self.assertRaisesRegex(RuntimeError, "context stopped"):
            self.views.check_health()
        self.graph.context.ok = lambda: True
        processes["cameras"].terminate()
        processes["cameras"].wait(timeout=5)
        with self.assertRaisesRegex(RuntimeError, "owned cameras exited"):
            self.views.check_health()

    def test_rviz_exit_degrades_without_stopping_cameras(self):
        processes = self.steady_session()
        processes["RViz"].terminate()
        processes["RViz"].wait(timeout=5)
        self.views.check_health()
        self.views.check_health()
        self.assertIsNone(processes["cameras"].poll())
        self.assertEqual(self.errors.getvalue().count("CAMERA_VIEWS_DEGRADED:"), 1)
        self.assertNotIn("CAMERA_VIEWS_READY:", self.output.getvalue())

    def test_startup_readiness_and_auxiliary_exit_still_fail(self):
        self.graph.health[self.views.CAMERA] = "source stale"
        with self.assertRaises(TimeoutError):
            self.views.check_health()
        auxiliary = self.sleeper()
        self.views.processes.append(("PICO video bridge", auxiliary))
        auxiliary.terminate()
        auxiliary.wait(timeout=5)
        with self.assertRaisesRegex(RuntimeError, "PICO video bridge exited"):
            self.views.check_workers()
