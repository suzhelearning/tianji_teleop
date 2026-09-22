"""Loopback/fake-process coverage; no real SDK load or hardware connection."""
from pathlib import Path

from tianji_runtime import workspace
from tianji_runtime.resources import ResourceNotFound
from contextlib import ExitStack, redirect_stderr, redirect_stdout
import io
import json
import os
import socket
import struct
import subprocess
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
import zlib
import hashlib
import signal
import time

import yaml

from tianji_controller.collection_supervisor import CollectionSupervisor, IdentityConflict
from tianji_controller.observer import ExecutorObserver

# Tests run from the installed package, so the workspace root is resolved
# through the shared contract rather than from this file's location.
ROOT = workspace()
from tianji_runtime import workspace
from tianji_controller.run_teleop import (
    CommandReceiver, controller_configuration, main, load_configuration, poll_enter)
from tianji_controller.safety import MotionGate, SafetyFault
from tianji_controller.tests.test_safety import configuration, frame, feedback, NOW
from tianji_controller.staged_motion import StagedMotionGate
from tianji_controller.protocol import decode_packet


def _fake_native_executable(path):
    """Mirror the real resolver's contract: only an existing executable is returned."""
    def resolve(name):
        if not path.is_file():
            raise ResourceNotFound(f"{name} is not built")
        return path
    return resolve


def packet(sequence, flags=7, epoch=7):
    body = struct.pack("<4sBBHQqQ54d", b"TJRC", 2, flags, 468,
                       sequence, NOW+sequence, epoch, *([0.0]*54))
    return body + struct.pack("<I", zlib.crc32(body))


class ExecutorTests(unittest.TestCase):
    def setUp(self):
        # These deterministic gate tests own a synthetic clock. Background DDS
        # and SDK sampling are exercised separately with real process loopback.
        for target in ("ExecutorObserver", "FeedbackHub"):
            patcher = patch("tianji_controller.run_teleop." + target)
            patcher.start()
            self.addCleanup(patcher.stop)

    def test_collection_startup_failure_never_enables_and_releases_hardware_first(self):
        import time
        from tianji_runtime import Feedback

        for failure_at in ("start", "wait_inputs_ready"):
            with self.subTest(failure_at=failure_at), tempfile.TemporaryDirectory() as folder:
                events = []
                hardware = {}
                for name, count in (("arms", 14), ("left_hand", 20), ("right_hand", 20)):
                    device = Mock()
                    device.connect.side_effect = lambda name=name: events.append(("connect", name))
                    device.stop.side_effect = lambda name=name: events.append(("stop", name))
                    device.close.side_effect = lambda name=name: events.append(("close", name))
                    device.read_feedback.side_effect = lambda count=count: Feedback(
                        (0.0,) * count, time.monotonic_ns(), True, False)
                    hardware[name] = device
                collector = Mock()
                getattr(collector, failure_at).side_effect = RuntimeError("not ready")
                collector.finish.side_effect = lambda: events.append(("collector", "joined"))
                with patch("tianji_controller.run_teleop.native_executable",
                           _fake_native_executable(Path(__file__))), \
                        patch("tianji_controller.run_teleop.CommandReceiver"), \
                        patch("tianji_controller.run_teleop.RealRobotViewer"), \
                        patch("tianji_controller.run_teleop.SessionLog.from_environment", return_value=None), \
                        patch("tianji_controller.run_teleop.CollectionSupervisor", return_value=collector), \
                        patch("tianji_controller.run_teleop.make_hardware", return_value=hardware) as factory, \
                        patch("tianji_controller.run_teleop.sys.stdin.isatty", return_value=True), \
                        redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                    result = main(["--confirm-real", "--devices", "all", "--collection",
                                   "--dataset", folder, "--task", "startup safety"])
                self.assertEqual(result, 1)
                for device in hardware.values():
                    device.enable.assert_not_called()
                    device.send.assert_not_called()
                if failure_at == "start":
                    factory.assert_not_called()
                    self.assertEqual(events, [("collector", "joined")])
                else:
                    self.assertEqual(events, [
                        ("connect", "arms"), ("connect", "left_hand"), ("connect", "right_hand"),
                        ("stop", "right_hand"), ("stop", "left_hand"), ("stop", "arms"),
                        ("close", "right_hand"), ("close", "left_hand"), ("close", "arms"),
                        ("collector", "joined")])

    def test_enter_poll_does_not_block_or_lose_second_confirmation(self):
        read_fd, write_fd = os.pipe()
        with os.fdopen(read_fd, "r") as terminal:
            try:
                with patch("tianji_controller.run_teleop.sys.stdin", terminal):
                    self.assertFalse(poll_enter())
                    os.write(write_fd, b"\n\n")
                    self.assertTrue(poll_enter())
                    self.assertTrue(poll_enter())
                    self.assertFalse(poll_enter())
            finally:
                os.close(write_fd)

    def test_disabled_frame_cannot_be_hidden_by_following_ready_frame(self):
        gate = MotionGate(configuration(), ("right_hand",))
        gate.arm(frame(), feedback(), NOW)
        receiver = CommandReceiver(0)
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                sender.sendto(packet(2, flags=1), ("127.0.0.1", receiver.port))
                sender.sendto(packet(3, flags=3), ("127.0.0.1", receiver.port))
            with self.assertRaises(SafetyFault):
                receiver.drain(lambda value: gate.observe_source(value, NOW+10))
            self.assertIsNotNone(gate.fault)
        finally:
            receiver.close()

    def test_reordered_command_is_not_accepted(self):
        receiver = CommandReceiver(0)
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                sender.sendto(packet(5), ("127.0.0.1", receiver.port))
                sender.sendto(packet(4), ("127.0.0.1", receiver.port))
            with self.assertRaises(SafetyFault):
                receiver.drain()
            self.assertEqual(receiver.latest.sequence, 5)
        finally:
            receiver.close()

    def test_crc_valid_but_unsafe_metadata_is_rejected(self):
        for flags, sequence, timestamp, value in ((8,1,NOW,0.0), (7,0,NOW,0.0),
                                                 (7,1,0,0.0), (7,1,NOW,float("nan"))):
            body = struct.pack("<4sBBHQqQ54d", b"TJRC", 2, flags, 468,
                               sequence, timestamp, 7, *([value]*54))
            with self.assertRaises(ValueError):
                decode_packet(body + struct.pack("<I", zlib.crc32(body)))

    def test_bimanual_packet_slots_and_flags_are_distinct(self):
        values = tuple(index / 100 for index in range(54))
        body = struct.pack("<4sBBHQqQ54d", b"TJRC", 2, 7, 468, 10, NOW, 7, *values)
        decoded = decode_packet(body + struct.pack("<I", zlib.crc32(body)))
        self.assertEqual(decoded.positions("arms"), values[:14])
        self.assertEqual(decoded.positions("left_hand"), values[14:34])
        self.assertEqual(decoded.positions("right_hand"), values[34:54])

    def test_v1_is_rejected_even_with_valid_crc(self):
        for count, size in ((34, 308), (54, 468)):
            body = struct.pack(f"<4sBBHQqQ{count}d", b"TJRC", 1, 3, size, 10, NOW, 7, *([0.0]*count))
            with self.assertRaises(ValueError):
                decode_packet(body + struct.pack("<I", zlib.crc32(body)))

    def test_all_and_hands_select_left_and_right_explicitly(self):
        path = ROOT / "config/robot.json"
        self.assertEqual(load_configuration(path, "all")[1], ("arms", "left_hand", "right_hand"))
        self.assertEqual(load_configuration(path, "hands")[1], ("left_hand", "right_hand"))
        self.assertEqual(load_configuration(path, "left_hand")[1], ("left_hand",))

    def test_duplicate_serials_are_rejected_before_hardware_creation(self):
        config, _ = load_configuration(ROOT / "config/robot.json")
        config["left_hand"]["serial"] = config["right_hand"]["serial"].lower()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "config.json"
            path.write_text(json.dumps(config))
            with self.assertRaisesRegex(ValueError, "distinct device serials"):
                load_configuration(path, "hands")
            # An unselected left channel must not block a right-only session.
            self.assertEqual(load_configuration(path, "right_hand")[1], ("right_hand",))

    def test_real_mode_refuses_noninteractive_invocation_before_sdk_load(self):
        result = subprocess.run([sys.executable, "-m", "tianji_controller.run_teleop",
                                 "--confirm-real", "--config", "/nonexistent-config.json"],
                                input="\n", capture_output=True, text=True, timeout=30)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("operator terminal", result.stderr)
        self.assertNotIn("READ-ONLY CONNECT", result.stdout)

    def test_interrupted_sdk_cleanup_failure_is_reported_and_fails(self):
        class InterruptedDevice:
            def connect(self):
                error = KeyboardInterrupt()
                error.add_note("UNCONFIRMED_DISABLE: SDK cleanup failed")
                raise error

            def stop(self):
                pass

            def close(self):
                pass

        stderr = io.StringIO()
        with patch("tianji_controller.run_teleop.make_hardware", return_value={"right_hand": InterruptedDevice()}), \
                redirect_stdout(io.StringIO()), redirect_stderr(stderr):
            result = main(["--inspect", "--devices", "right_hand"])
        self.assertEqual(result, 1)
        self.assertIn("UNCONFIRMED_DISABLE", stderr.getvalue())
        self.assertIn("physical emergency stop", stderr.getvalue())

    def _delayed_preflight(self, delay, *, stop_source_at=None, confirm=True, arm_pose=None,
                           log_dir=None):
        clock = SimpleNamespace(now=NOW, reads=0, streaming=True, entered=False, enabled=False)
        receiver = Mock(port=17001)
        receiver.latest = frame(stamp=clock.now)

        def drain(*args):
            if clock.streaming:
                receiver.latest = frame(stamp=clock.now)
            return receiver.latest

        def read_feedback():
            clock.now += int(delay * 1e9)
            clock.reads += 1
            if stop_source_at is not None and clock.reads >= stop_source_at:
                clock.streaming = False
            reading = feedback(stamp=clock.now, enabled=clock.enabled)["arms"]
            reading.position_rad = tuple(arm_pose) if arm_pose is not None else (0.0,) * 14
            return reading

        def enter():
            if clock.enabled:
                return True  # Stop immediately after fake enable; never send motion.
            if confirm and not clock.entered:
                clock.entered = True
                return True
            return False

        def enable(guard):
            guard()
            clock.enabled = True

        def sleep(seconds):
            clock.now += int(seconds * 1e9)

        receiver.drain.side_effect = drain
        device = Mock()
        device.read_feedback.side_effect = read_feedback
        device.enable.side_effect = enable
        controller = Mock()
        controller.poll.return_value = None
        config_path = ROOT / "config/robot.json"
        output = io.StringIO()
        monitor = Mock()
        monitor.is_running.return_value = True
        viewer_factory = Mock(return_value=monitor)
        with tempfile.TemporaryDirectory() as temporary, ExitStack() as stack:
            viewer = Path(temporary) / "tianji_qp_ik_viewer"
            viewer.touch()
            controller_config = Path(temporary) / "controller.yaml"
            controller_config.write_text("controller:\n  rate_hz: 200\n")
            replacements = {
                "native_executable": _fake_native_executable(viewer),
                "make_hardware": Mock(return_value={"arms": device}),
                "RealRobotViewer": viewer_factory,
                "controller_configuration": Mock(return_value=controller_config),
                "CommandReceiver": Mock(return_value=receiver),
                "subprocess.Popen": Mock(return_value=controller),
                "stop_controller": Mock(),
                "poll_enter": enter,
                "time.monotonic_ns": lambda: clock.now,
                "time.monotonic": lambda: clock.now / 1e9,
                "time.sleep": sleep,
                "sys.stdin.isatty": lambda: True,
            }
            for name, replacement in replacements.items():
                stack.enter_context(patch("tianji_controller.run_teleop." + name, replacement))
            stack.enter_context(patch.dict(os.environ, {}, clear=False))
            os.environ.pop("TIANJI_RUN_LOG_DIR", None)
            if log_dir is not None:
                os.environ["TIANJI_RUN_LOG_DIR"] = str(log_dir)
            stack.enter_context(redirect_stdout(output))
            stack.enter_context(redirect_stderr(output))
            result = main(["--config", str(config_path), "--devices", "arms",
                           "--confirm-real", "--duration", "1"])
        device.send.assert_not_called()
        return result, device.enable.call_count, clock.entered, output.getvalue()

    def test_feedback_read_latency_does_not_make_fresh_feedback_future_dated(self):
        result, enables, entered, output = self._delayed_preflight(.01)
        self.assertEqual((result, enables, entered), (0, 1, True), output)

    def test_preflight_and_confirmation_refresh_targets_after_slow_feedback(self):
        result, enables, entered, output = self._delayed_preflight(.2)
        self.assertEqual((result, enables, entered), (0, 1, True), output)

    def test_source_expiring_during_confirmation_cannot_enable(self):
        result, enables, entered, output = self._delayed_preflight(.2, stop_source_at=3)
        self.assertEqual((result, enables, entered), (1, 0, True), output)

    def test_healthy_slow_feedback_still_requires_operator_confirmation(self):
        result, enables, entered, output = self._delayed_preflight(.01, confirm=False)
        self.assertEqual((result, enables, entered), (0, 0, False), output)

    def test_first_enter_can_authorize_slow_alignment_from_a_different_pose(self):
        result, enables, entered, output = self._delayed_preflight(.01, arm_pose=(0.0, 0.12) + (0.0,) * 12)
        self.assertEqual((result, enables, entered), (0, 1, True), output)


    def test_viewer_failure_happens_before_hardware_factory(self):
        with patch("tianji_controller.run_teleop.RealRobotViewer", side_effect=RuntimeError("display unavailable")), \
                patch("tianji_controller.run_teleop.make_hardware") as hardware_factory, \
                patch("tianji_controller.run_teleop.sys.stdin.isatty", return_value=True), \
                redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            result = main(["--devices", "arms", "--confirm-real"])
        self.assertEqual(result, 1)
        hardware_factory.assert_not_called()

    def test_invalid_home_is_rejected_before_viewer_or_hardware(self):
        config, _ = load_configuration(ROOT / "config/robot.json")
        config["staged_motion"]["home_left_rad"][0] = 999.0
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "config.json"
            path.write_text(json.dumps(config))
            with patch("tianji_controller.run_teleop.RealRobotViewer") as viewer_factory, \
                    patch("tianji_controller.run_teleop.make_hardware") as hardware_factory, \
                    patch("tianji_controller.run_teleop.sys.stdin.isatty", return_value=True), \
                    redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
                main(["--config", str(path), "--devices", "arms", "--confirm-real"])
            self.assertEqual(raised.exception.code, 2)
            viewer_factory.assert_not_called()
            hardware_factory.assert_not_called()

    def test_invalid_dls_profiles_fail_before_collection_or_hardware(self):
        config, _ = load_configuration(ROOT / "config/robot.json")
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            profile = controller_configuration(config, ROOT, {}, folder / "profile.yaml")
            valid = yaml.safe_load(profile.read_text())
            config["controller_config"] = str(profile)
            config_path = folder / "robot.json"
            config_path.write_text(json.dumps(config))
            cases = (
                ("ik", "algorithm", "pico_ee_ceres"),
                ("pico_ee_franka_dls", "enabled", False),
                ("post_smoothing", "mode", "none"),
                ("post_smoothing", "max_jerk_rad_s3", [0.0] * 7),
                ("controller", "model_state_only", False),
                ("control", "level", "acceleration"),
                ("controller", "pico_ee_dls_kinematics_urdf_path", str(folder / "missing.urdf")),
                ("spark_shared_root", "robot_geometry_artifact", str(folder / "missing.yaml")),
            )
            for section, key, value in cases:
                with self.subTest(section=section, key=key):
                    candidate = yaml.safe_load(yaml.safe_dump(valid))
                    target = (candidate["pico_ee_franka_dls"]["post_smoothing"]
                              if section == "post_smoothing" else candidate[section])
                    target[key] = value
                    profile.write_text(yaml.safe_dump(candidate))
                    with patch("tianji_controller.run_teleop.CollectionSupervisor") as collection, \
                            patch("tianji_controller.run_teleop.make_hardware") as hardware, \
                            patch("tianji_controller.run_teleop.RealRobotViewer") as viewer, \
                            patch("tianji_controller.run_teleop.sys.stdin.isatty", return_value=True), \
                            redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
                        main(["--config", str(config_path), "--devices", "all", "--confirm-real",
                              "--collection", "--dataset", str(folder / "dataset"), "--task", "safety"])
                    self.assertEqual(raised.exception.code, 2)
                    collection.assert_not_called()
                    hardware.assert_not_called()
                    viewer.assert_not_called()

    def test_retired_backends_and_real_options_are_rejected_without_devices(self):
        for options in (["--ik-backend", "spark"], ["--ik-backend", "ceres"],
                        ["--ik-backend", "mapped-palm"], ["--mapped-palm-xz-calibration"],
                        ["--mapped-palm-dropout-policy", "hold-300ms"],
                        ["--mapped-palm-resync-policy", "bounded"]):
            with self.subTest(options=options), \
                    patch("tianji_controller.run_teleop.make_hardware") as hardware, \
                    patch("tianji_controller.run_teleop.CollectionSupervisor") as collection, \
                    redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
                main(options)
            self.assertEqual(raised.exception.code, 2)
            hardware.assert_not_called()
            collection.assert_not_called()

    def test_missing_controller_is_refused_before_hardware_connection(self):
        with tempfile.TemporaryDirectory() as folder, \
                patch("tianji_controller.run_teleop.native_executable",
                      _fake_native_executable(Path(folder) / "missing_viewer")), \
                patch("tianji_controller.run_teleop.make_hardware") as hardware_factory, \
                patch("tianji_controller.run_teleop.RealRobotViewer") as viewer_factory, \
                patch("tianji_controller.run_teleop.sys.stdin.isatty", return_value=True), \
                redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            result = main(["--config", str(ROOT / "config/robot.json"),
                           "--devices", "arms", "--confirm-real"])
        self.assertEqual(result, 1)
        hardware_factory.assert_not_called()
        viewer_factory.assert_not_called()

    def test_occupied_command_port_is_refused_before_hardware_connection(self):
        config, _ = load_configuration(ROOT / "config/robot.json")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as occupied, \
                tempfile.TemporaryDirectory() as folder:
            occupied.bind(("127.0.0.1", 0))
            config["command_port"] = occupied.getsockname()[1]
            config_path = Path(folder) / "config.json"
            config_path.write_text(json.dumps(config))
            with patch("tianji_controller.run_teleop.make_hardware") as hardware_factory, \
                    patch("tianji_controller.run_teleop.RealRobotViewer") as viewer_factory, \
                    patch("tianji_controller.run_teleop.sys.stdin.isatty", return_value=True), \
                    redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                result = main(["--config", str(config_path), "--devices", "arms", "--confirm-real"])
        self.assertEqual(result, 1)
        hardware_factory.assert_not_called()
        viewer_factory.assert_not_called()

    def test_all_stop_requests_precede_any_device_release(self):
        for failing_stop in (None, "right_hand"):
            with self.subTest(failing_stop=failing_stop), tempfile.TemporaryDirectory() as folder:
                viewer = Path(folder) / "tianji_qp_ik_viewer"
                viewer.parent.mkdir(parents=True, exist_ok=True)
                viewer.touch()
                stopped = set()
                released_after = []
                hardware = {}
                for name in ("arms", "left_hand", "right_hand"):
                    device = Mock()
                    def stop(name=name):
                        stopped.add(name)
                        if name == failing_stop:
                            raise RuntimeError("stop failed")
                    device.stop.side_effect = stop
                    device.close.side_effect = lambda: released_after.append(set(stopped))
                    hardware[name] = device
                hardware["right_hand"].connect.side_effect = RuntimeError("connection interrupted")
                with patch("tianji_controller.run_teleop.make_hardware", return_value=hardware), \
                        patch("tianji_controller.run_teleop.native_executable",
                              _fake_native_executable(viewer)), \
                        patch("tianji_controller.run_teleop.RealRobotViewer"), \
                        patch("tianji_controller.run_teleop.sys.stdin.isatty", return_value=True), \
                        redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                    result = main(["--config", str(ROOT / "config/robot.json"),
                                   "--devices", "all", "--confirm-real"])
                self.assertEqual(result, 1)
                self.assertEqual(released_after, [set(hardware)] * 3)

    def test_control_period_includes_work_without_catchup(self):
        for work_s, expected_s in ((.002, .005), (.008, .008)):
            with self.subTest(work_s=work_s):
                result, state, _, output = self._staged_session(cycle_work_s=work_s)
                self.assertEqual(result, 0, output)
                intervals = [right - left for left, right in zip(state.send_times, state.send_times[1:])]
                self.assertTrue(intervals)
                for interval in intervals:
                    self.assertAlmostEqual(interval / 1e9, expected_s, places=8)


    def _staged_session(self, *, close_during=None, abort_during=None, log_dir=None,
                        cleanup_failure=False, cycle_work_s=0):
        config, _ = load_configuration(ROOT / "config/robot.json")
        home = tuple(config["staged_motion"]["home_left_rad"] + config["staged_motion"]["home_right_rad"])
        state = SimpleNamespace(now=NOW, actual=home, enabled=False, first_enter=False,
                                sends=[], send_times=[], gates=[], stopped=False)
        config["staged_motion"]["settle_time_s"] = .02
        goal = list(home)
        goal[0] += .02
        goal[7] -= .02
        device = Mock()
        monitor = Mock()
        controller = Mock()
        controller.poll.return_value = None
        receiver = Mock(port=17001)

        def gate_factory(*args):
            gate = StagedMotionGate(*args)
            state.gates.append(gate)
            return gate

        def receive(validate=None):
            target = list(goal)
            if state.gates[-1].phase == "TELEOP":
                target[0] += .1
            value = frame(stamp=state.now)
            value = type(value)(value.sequence, value.timestamp_ns, value.tracking_epoch,
                                value.flags, tuple(target[:7]), tuple(target[7:]),
                                value.left_hand, value.right_hand)
            if validate is not None:
                validate(value)
            return value

        def measured():
            value = feedback(stamp=state.now, enabled=state.enabled)["arms"]
            value.position_rad = state.actual
            return value

        def enter():
            gate = state.gates[-1]
            if not state.first_enter:
                state.first_enter = True
                return True
            if not state.enabled:
                return False
            if gate.phase == abort_during and state.sends:
                return True
            if gate.phase == "READY":
                return True
            return gate.phase == "TELEOP" and any(phase == "TELEOP" for phase, _ in state.sends)

        def enable(guard):
            guard()
            state.enabled = True

        def send(positions):
            state.actual = tuple(positions)
            state.sends.append((state.gates[-1].phase, state.actual))
            state.send_times.append(state.now)
            state.now += round(cycle_work_s * 1e9)

        def stop():
            state.enabled = False
            state.stopped = True

        receiver.drain.side_effect = receive
        device.read_feedback.side_effect = measured
        device.enable.side_effect = enable
        device.send.side_effect = send
        device.stop.side_effect = stop
        monitor.is_running.side_effect = lambda: not (
            state.gates and state.gates[-1].phase == close_during and state.sends)
        output = io.StringIO()
        with tempfile.TemporaryDirectory() as temporary, ExitStack() as stack:
            base = Path(temporary)
            config_path = base / "config.json"
            config_path.write_text(json.dumps(config))
            viewer = base / "control/build/tianji_qp_ik_viewer"
            viewer.parent.mkdir(parents=True, exist_ok=True)
            viewer.touch()
            controller_config = base / "controller.yaml"
            controller_config.write_text("controller:\n  rate_hz: 200\n")
            if cleanup_failure:
                device.stop.side_effect = RuntimeError("UNCONFIRMED_DISABLE")
            replacements = {
                "native_executable": _fake_native_executable(viewer),
                "StagedMotionGate": gate_factory,
                "RealRobotViewer": Mock(return_value=monitor),
                "make_hardware": Mock(return_value={"arms": device}),
                "controller_configuration": Mock(return_value=controller_config),
                "CommandReceiver": Mock(return_value=receiver),
                "subprocess.Popen": Mock(return_value=controller),
                "stop_controller": Mock(),
                "poll_enter": enter,
                "time.monotonic_ns": lambda: state.now,
                "time.monotonic": lambda: state.now / 1e9,
                "time.sleep": lambda seconds: setattr(state, "now", state.now + int(seconds * 1e9)),
                "sys.stdin.isatty": lambda: True,
            }
            for name, replacement in replacements.items():
                stack.enter_context(patch("tianji_controller.run_teleop." + name, replacement))
            stack.enter_context(patch.dict(os.environ, {}, clear=False))
            os.environ.pop("TIANJI_RUN_LOG_DIR", None)
            if log_dir is not None:
                os.environ["TIANJI_RUN_LOG_DIR"] = str(log_dir)
            stack.enter_context(redirect_stdout(output))
            stack.enter_context(redirect_stderr(output))
            result = main(["--config", str(config_path), "--devices", "arms",
                           "--confirm-real", "--duration", "5"])
        return result, state, home, output.getvalue()

    def test_three_enters_align_then_teleop_then_home_before_disabling(self):
        result, state, home, output = self._staged_session()
        self.assertEqual(result, 0, output)
        phases = [phase for phase, _ in state.sends]
        self.assertIn("ALIGNING", phases)
        self.assertIn("TELEOP", phases)
        self.assertIn("HOMING", phases)
        self.assertLess(phases.index("ALIGNING"), phases.index("TELEOP"))
        self.assertLess(phases.index("TELEOP"), phases.index("HOMING"))
        self.assertEqual(state.gates[-1].phase, "HOME_REACHED", output)
        self.assertEqual(state.actual, home)
        self.assertTrue(state.stopped)
        self.assertFalse(state.enabled)

    def test_monitor_close_stops_without_return_home(self):
        result, state, _, output = self._staged_session(close_during="TELEOP")
        self.assertEqual(result, 1, output)
        self.assertTrue(state.stopped)
        self.assertNotIn("HOMING", [phase for phase, _ in state.sends])

    def test_enter_during_alignment_aborts_without_home(self):
        result, state, _, output = self._staged_session(abort_during="ALIGNING")
        self.assertEqual(result, 0, output)
        self.assertTrue(state.stopped)
        self.assertNotIn("TELEOP", [phase for phase, _ in state.sends])
        self.assertNotIn("HOMING", [phase for phase, _ in state.sends])

    def test_logged_staged_fault_separates_reference_last_send_and_feedback(self):
        with tempfile.TemporaryDirectory() as folder:
            log_dir = Path(folder) / "logs"
            result, state, _, output = self._staged_session(
                close_during="TELEOP", log_dir=log_dir, cleanup_failure=True)
            self.assertEqual(result, 1, output)
            self.assertIn(f"RUN LOG: {log_dir}", output)
            session = json.loads((log_dir / "session.json").read_text())
            self.assertEqual((session["outcome"], session["result"]), ("failed", 1))
            self.assertIn("visualization closed", session["reason"])
            self.assertEqual(session["mode"], "real")
            self.assertEqual(session["devices"], ["arms"])
            self.assertEqual(session["controller_configuration"]["file"], "controller_configuration.yaml")
            self.assertEqual(len(session["cleanup_errors"]), 1)
            self.assertIn("UNCONFIRMED_DISABLE", session["cleanup_errors"][0])
            lines = [json.loads(line) for line in
                     (log_dir / "flight_recorder.jsonl").read_text().splitlines()]
            self.assertEqual(len(lines), session["flight_recorder"]["samples"])
            last = lines[-1]
            self.assertEqual(last["phase"], "TELEOP")
            self.assertTrue(last["packet"]["sequence"] > 0)
            self.assertTrue(last["reference"]["arms"])
            self.assertTrue(last["sent"]["arms"]["positions"])
            self.assertTrue(last["feedback"]["arms"]["enabled"])
            # The recorded setpoint is a completed send from an earlier instant,
            # never the gate output of the step this sample precedes.
            self.assertLess(last["sent"]["arms"]["monotonic_ns"], last["monotonic_ns"])

    def test_logged_preflight_failure_keeps_the_stale_reference_and_no_send(self):
        with tempfile.TemporaryDirectory() as folder:
            log_dir = Path(folder) / "logs"
            result, enables, entered, output = self._delayed_preflight(
                .2, stop_source_at=3, log_dir=log_dir)
            self.assertEqual((result, enables), (1, 0), output)
            session = json.loads((log_dir / "session.json").read_text())
            self.assertEqual((session["outcome"], session["result"]), ("failed", 1))
            self.assertTrue(session["reason"].startswith("SafetyFault:"), session["reason"])
            lines = [json.loads(line) for line in
                     (log_dir / "flight_recorder.jsonl").read_text().splitlines()]
            self.assertTrue(lines)
            self.assertEqual({line["phase"] for line in lines}, {"PREFLIGHT", "WAITING"})
            self.assertTrue(all(not line["sent"] for line in lines))
            last = lines[-1]
            self.assertTrue(last["reference"]["arms"])
            self.assertTrue(last["feedback"]["arms"])
            self.assertGreater(last["monotonic_ns"] - last["packet"]["timestamp_ns"], 150_000_000)



class CollectionOwnershipTests(unittest.TestCase):
    """Exercise ownership decisions without spawning ROS or device processes."""

    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        config, model = root / "collection.json", root / "model.xml"
        config.write_bytes(b'{"cameras": {}}\n')
        model.write_bytes(b"<mujoco/>\n")
        self.observer = Mock(session_id="this-executor")
        self.status = SimpleNamespace(session_id="", state="IDLE", prepared=True,
                                      inputs_ready=False, error="")
        self.observer.status.side_effect = lambda: self.status
        self.supervisor = CollectionSupervisor(self.observer, root / "dataset", "task", config, model)
        shared = {"owner_token": "", "config_path": str(config),
                  "config_digest": hashlib.sha256(config.read_bytes()).hexdigest()}
        self.participants = {
            CollectionSupervisor.CAMERA: dict(shared),
            CollectionSupervisor.COLLECTOR: dict(shared, dataset_path=str(root / "dataset"), task="task",
                model_path=str(model), model_digest=hashlib.sha256(model.read_bytes()).hexdigest()),
        }
        self.duplicates = []
        self.foreign_services = {}
        self.events = []
        self.observer.request.side_effect = self.request
        self.clock = 0.0
        patches = (
            patch("tianji_controller.collection_supervisor.time.monotonic", side_effect=lambda: self.clock),
            patch("tianji_controller.collection_supervisor.time.sleep", side_effect=self.sleep),
            patch("tianji_controller.collection_supervisor.subprocess.Popen", side_effect=self.spawn),
            patch("tianji_controller.collection_supervisor.os.killpg",
                  side_effect=lambda pid, sig: self.events.append(("kill", pid, sig))),
            patch("tianji_controller.collection_supervisor.fcntl.flock"),
        )
        for patcher in patches:
            patcher.start()
            self.addCleanup(patcher.stop)
        self.addCleanup(self.supervisor.finish)

    def sleep(self, duration):
        self.clock += duration

    def request(self, operation, *args):
        if operation == "graph":
            nodes = [(name.lstrip("/"), "/") for name in self.participants] + self.duplicates
            services = [(name, [kind]) for node in self.participants
                        for name, kind in CollectionSupervisor.SERVICES[node].items()]
            services.extend((service, [kind]) for service, (_, kind) in self.foreign_services.items())
            return nodes, services
        if operation == "service_owners":
            service = args[0]
            owners = [(node, [kind]) for node in self.participants
                      for name, kind in {
                          node + "/get_parameters": "rcl_interfaces/srv/GetParameters",
                          **CollectionSupervisor.SERVICES[node]}.items() if name == service]
            if service in self.foreign_services:
                node, kind = self.foreign_services[service]
                owners.append((node, [kind]))
            return owners
        if operation == "parameters":
            values = self.participants[args[0].removesuffix("/get_parameters")]
            return SimpleNamespace(values=[SimpleNamespace(type=4, string_value=values[key]) for key in args[1]])
        if operation == "trigger":
            return SimpleNamespace(success=True)
        if operation in ("bind_collector", "activate_collection"):
            return True
        if operation == "drain_collection":
            self.events.append(("drain", self.observer.session_id))
            self.status.state = "ABORTING"
            self.drain_polls = 0
            def draining():
                self.drain_polls += 1
                if self.drain_polls == 3:
                    self.status.state = "IDLE"
                return self.status
            self.observer.status.side_effect = draining
            return True
        if operation == "collection_pending":
            return False
        raise AssertionError(operation)

    def spawn(self, command, **kwargs):
        node = (CollectionSupervisor.CAMERA if command[0] == "bash"
                else CollectionSupervisor.COLLECTOR)
        self.events.append(("spawn", node))
        self.participants[node] = self.supervisor._expected(node, self.observer.session_id)
        process = Mock(pid=100 + len(self.events), returncode=None)
        process.poll.side_effect = lambda: process.returncode
        process.wait.side_effect = lambda timeout: setattr(process, "returncode", -signal.SIGINT)
        return process

    def test_standalone_reuse_waits_for_feedback_and_drains_without_stopping_workers(self):
        self.supervisor.start()
        with self.assertRaises(RuntimeError):
            self.supervisor.check_before_enable()
        self.status.session_id = self.observer.session_id
        self.status.inputs_ready = True
        self.supervisor.wait_inputs_ready()
        self.supervisor.check_before_enable()
        self.status.state = "RECORDING"
        self.supervisor.finish()
        self.assertEqual(self.events, [("drain", "this-executor")])
        self.assertEqual(self.drain_polls, 3)
        self.assertEqual(set(self.participants), {CollectionSupervisor.CAMERA, CollectionSupervisor.COLLECTOR})

    def test_only_created_collector_is_stopped_after_reusing_camera(self):
        del self.participants[CollectionSupervisor.COLLECTOR]
        self.supervisor.start()
        self.supervisor.finish()
        self.assertEqual(self.events, [
            ("spawn", CollectionSupervisor.COLLECTOR), ("drain", "this-executor"),
            ("kill", 101, signal.SIGINT)])

    def test_created_workers_stop_in_reverse_order_after_session_drain(self):
        self.participants.clear()
        self.supervisor.start()
        self.supervisor.finish()
        self.assertEqual(self.events, [
            ("spawn", CollectionSupervisor.CAMERA), ("spawn", CollectionSupervisor.COLLECTOR),
            ("drain", "this-executor"), ("kill", 102, signal.SIGINT), ("kill", 101, signal.SIGINT)])

    def test_foreign_owner_and_configuration_conflicts_never_launch_workers(self):
        collector = self.participants[CollectionSupervisor.COLLECTOR]
        for key in ("owner_token", "config_path", "config_digest", "dataset_path", "task", "model_path", "model_digest"):
            with self.subTest(parameter=key):
                old = collector[key]
                collector[key] = "foreign"
                try:
                    with self.assertRaises(IdentityConflict):
                        self.supervisor.start()
                finally:
                    collector[key] = old
                    self.supervisor.finish()
                self.assertEqual(self.events, [])

    def test_changed_file_bytes_are_not_hidden_by_matching_paths(self):
        self.supervisor.config.write_bytes(b'{"cameras": {"changed": true}}\n')
        with self.assertRaises(IdentityConflict):
            self.supervisor.start()
        self.assertEqual(self.events, [])

    def test_duplicate_node_and_foreign_service_are_rejected(self):
        self.duplicates.append(("data_collector", "/"))
        with self.assertRaises(IdentityConflict):
            self.supervisor.start()
        self.supervisor.finish()
        self.duplicates.clear()
        self.foreign_services["/start_collect"] = ("/rogue", "tianji_interfaces/srv/StartCollect")
        with self.assertRaises(IdentityConflict):
            self.supervisor.start()
        self.assertEqual(self.events, [])

    def test_orphan_standalone_collector_is_not_paired_with_owned_cameras(self):
        del self.participants[CollectionSupervisor.CAMERA]
        with self.assertRaises(IdentityConflict):
            self.supervisor.start()
        self.assertEqual(self.events, [])

    def test_foreign_active_recording_is_neither_activated_nor_aborted(self):
        self.status.session_id = "another-executor"
        self.status.state = "RECORDING"
        with self.assertRaises(IdentityConflict):
            self.supervisor.start()
        self.supervisor.finish()
        self.assertEqual(self.events, [])
        operations = [call.args[0] for call in self.observer.request.call_args_list]
        self.assertNotIn("activate_collection", operations)
        self.assertNotIn("drain_collection", operations)

    def test_failed_drain_does_not_kill_reused_workers(self):
        self.supervisor.start()
        self.observer.request.side_effect = RuntimeError("collector unavailable")
        with self.assertRaises(RuntimeError):
            self.supervisor.finish()
        self.assertEqual(self.events, [])

    def test_recording_failure_isolated_from_armed_motion_gate(self):
        gate = MotionGate(configuration(), ("right_hand",))
        gate.arm(frame(), feedback(), NOW)
        self.observer.status.side_effect = RuntimeError("recording publisher disappeared")
        self.observer.request.side_effect = AssertionError("control loop must not perform RPC")
        with redirect_stdout(io.StringIO()):
            self.supervisor.check()
            self.supervisor.check()
        self.assertTrue(gate.armed)
        self.assertIsNone(gate.fault)
        self.observer.abort.assert_called_once()


class CollectionObserverTests(unittest.TestCase):
    def status_message(self, observer, **changes):
        fields = dict(session_id=observer.session_id, state="IDLE", prepared=True,
                      inputs_ready=True, error="", last_saved_path="",
                      published_monotonic_ns=time.monotonic_ns())
        fields.update(changes)
        return SimpleNamespace(**fields)

    def test_unverified_status_cannot_prepare_and_wrong_session_revokes_readiness(self):
        observer = ExecutorObserver("real", collection=True)
        info = b"collector"
        message = self.status_message(observer)
        observer._receive_status(message, info)
        self.assertIsNone(observer.status())
        observer._status_gid = b"collector"
        with redirect_stdout(io.StringIO()):
            observer._receive_status(message, info)
        self.assertIs(observer.status(), message)
        observer._receive_status(self.status_message(observer, session_id="other"), info)
        with self.assertRaises(RuntimeError):
            observer.status()

    def test_unknown_publisher_and_stale_status_cannot_restore_readiness(self):
        observer = ExecutorObserver("real", collection=True)
        observer._status_gid = b"collector"
        observer._receive_status(self.status_message(observer, published_monotonic_ns=1),
                                 b"collector")
        self.assertIsNone(observer.status())
        observer._receive_status(self.status_message(observer), b"rogue")
        with self.assertRaises(RuntimeError):
            observer.status()

    def test_second_status_endpoint_or_service_owner_revokes_bound_identity(self):
        observer = ExecutorObserver("real", collection=True)
        node = Mock()
        node.get_node_names_and_namespaces.return_value = [("data_collector", "/")]
        services = [
            ("/data_collector/get_parameters", ["rcl_interfaces/srv/GetParameters"]),
            ("/tianji/collection/check_ready", ["std_srvs/srv/Trigger"]),
            ("/start_collect", ["tianji_interfaces/srv/StartCollect"]),
            ("/stop_collect", ["tianji_interfaces/srv/StopCollect"]),
        ]
        node.get_service_names_and_types_by_node.return_value = services
        endpoint = SimpleNamespace(node_name="data_collector", node_namespace="/", endpoint_gid=b"collector")
        node.get_publishers_info_by_topic.return_value = [endpoint]
        observer._status_gid = observer._validate_collector(node)
        node.get_publishers_info_by_topic.return_value = [endpoint, endpoint]
        with self.assertRaises(RuntimeError):
            observer._validate_collector(node)
        node.get_publishers_info_by_topic.return_value = [endpoint]
        node.get_node_names_and_namespaces.return_value.append(("rogue", "/"))
        with self.assertRaises(RuntimeError):
            observer._validate_collector(node)

    def test_operator_handoff_never_calls_ros_or_waits_for_full_command_queue(self):
        observer = ExecutorObserver("real", collection=True)
        observer.request = Mock(side_effect=AssertionError("control-thread RPC"))
        observer.update_state("TELEOP")
        for _ in range(8):
            self.assertTrue(observer.command("r"))
        self.assertFalse(observer.command("s"))
        observer.update_state("HOME")
        self.assertTrue(observer._abort.is_set())
        self.assertFalse(observer.command("r"))
        observer.request.assert_not_called()

if __name__ == "__main__":
    unittest.main()
