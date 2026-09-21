"""Loopback/fake-process coverage; no real SDK load or hardware connection."""
from pathlib import Path
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

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from real_robot.run_teleop import CommandReceiver, main, load_configuration, poll_enter
from real_robot.safety import MotionGate, SafetyFault
from real_robot.tests.test_safety import configuration, frame, feedback, NOW
from real_robot.staged_motion import StagedMotionGate
from real_robot.protocol import decode_packet


def packet(sequence, flags=7, epoch=7):
    body = struct.pack("<4sBBHQqQ54d", b"TJRC", 2, flags, 468,
                       sequence, NOW+sequence, epoch, *([0.0]*54))
    return body + struct.pack("<I", zlib.crc32(body))


class ExecutorTests(unittest.TestCase):
    def test_enter_poll_does_not_block_or_lose_second_confirmation(self):
        read_fd, write_fd = os.pipe()
        with os.fdopen(read_fd, "r") as terminal:
            try:
                with patch("real_robot.run_teleop.sys.stdin", terminal):
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
        path = ROOT / "real_robot/config.json"
        self.assertEqual(load_configuration(path, "all")[1], ("arms", "left_hand", "right_hand"))
        self.assertEqual(load_configuration(path, "hands")[1], ("left_hand", "right_hand"))
        self.assertEqual(load_configuration(path, "left_hand")[1], ("left_hand",))

    def test_duplicate_serials_are_rejected_before_hardware_creation(self):
        config, _ = load_configuration(ROOT / "real_robot/config.json")
        config["left_hand"]["serial"] = config["right_hand"]["serial"].lower()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "config.json"
            path.write_text(json.dumps(config))
            with self.assertRaisesRegex(ValueError, "distinct device serials"):
                load_configuration(path, "hands")
            # An unselected left channel must not block a right-only session.
            self.assertEqual(load_configuration(path, "right_hand")[1], ("right_hand",))

    def test_real_mode_refuses_noninteractive_invocation_before_sdk_load(self):
        result = subprocess.run([sys.executable, str(ROOT/"real_robot/run_teleop.py"),
                                 "--confirm-real", "--config", "/nonexistent-config.json"],
                                input="\n", capture_output=True, text=True, timeout=5)
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
        with patch("real_robot.run_teleop.make_hardware", return_value={"right_hand": InterruptedDevice()}), \
                redirect_stdout(io.StringIO()), redirect_stderr(stderr):
            result = main(["--inspect", "--devices", "right_hand"])
        self.assertEqual(result, 1)
        self.assertIn("UNCONFIRMED_DISABLE", stderr.getvalue())
        self.assertIn("physical emergency stop", stderr.getvalue())

    def _delayed_preflight(self, delay, *, stop_source_at=None, confirm=True, arm_pose=None, model=None,
                           log_dir=None, enable_delay=0.0, exercise_alignment=False):
        clock = SimpleNamespace(now=NOW, reads=0, streaming=True, entered=False, enabled=False, sent=False)
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
                return not exercise_alignment or clock.sent
            if confirm and not clock.entered:
                clock.entered = True
                return True
            return False

        def enable(guard):
            guard()
            clock.now += int(enable_delay * 1e9)
            guard()
            clock.enabled = True

        def sleep(seconds):
            clock.now += int(seconds * 1e9)

        receiver.drain.side_effect = drain
        device = Mock()
        device.read_feedback.side_effect = read_feedback
        device.enable.side_effect = enable
        def send(positions):
            clock.sent = True
            self.assertEqual(tuple(positions), (0.0,) * 14)
        device.send.side_effect = send
        controller = Mock()
        controller.poll.return_value = None
        config_path = ROOT / "real_robot/config.json"
        output = io.StringIO()
        monitor = Mock()
        monitor.is_running.return_value = True
        viewer_factory = Mock(return_value=monitor)
        if model is not None:
            viewer_factory.side_effect = RuntimeError("model unavailable")
        with tempfile.TemporaryDirectory() as temporary, ExitStack() as stack:
            if model is not None:
                config, _ = load_configuration(config_path)
                config["controller_model"] = model
                config_path = Path(temporary) / "config.json"
                config_path.write_text(json.dumps(config))
            executor_root = Path(temporary) / "real_robot"
            viewer = executor_root.parent / "control/build/tianji_qp_ik_viewer"
            viewer.parent.mkdir(parents=True)
            viewer.touch()
            controller_config = Path(temporary) / "controller.yaml"
            controller_config.write_text("controller:\n  rate_hz: 200\n")
            replacements = {
                "ROOT": executor_root,
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
                stack.enter_context(patch("real_robot.run_teleop." + name, replacement))
            stack.enter_context(patch.dict(os.environ, {}, clear=False))
            os.environ.pop("TIANJI_RUN_LOG_DIR", None)
            if log_dir is not None:
                os.environ["TIANJI_RUN_LOG_DIR"] = str(log_dir)
            stack.enter_context(redirect_stdout(output))
            stack.enter_context(redirect_stderr(output))
            result = main(["--config", str(config_path), "--devices", "arms",
                           "--confirm-real", "--duration", "1"])
        if exercise_alignment:
            device.send.assert_called_once()
        else:
            device.send.assert_not_called()
        return result, device.enable.call_count, clock.entered, output.getvalue()

    def test_slow_sdk_enable_does_not_consume_first_alignment_tick(self):
        result, enables, entered, output = self._delayed_preflight(
            .005, enable_delay=.55, exercise_alignment=True)
        self.assertEqual((result, enables, entered), (0, 1, True), output)

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

    def test_unavailable_required_viewer_cannot_enable(self):
        result, enables, entered, output = self._delayed_preflight(
            .01, model="missing-model.xml")
        self.assertEqual((result, enables, entered), (1, 0, False), output)

    def test_viewer_failure_happens_before_hardware_factory(self):
        with patch("real_robot.run_teleop.RealRobotViewer", side_effect=RuntimeError("display unavailable")), \
                patch("real_robot.run_teleop.make_hardware") as hardware_factory, \
                patch("real_robot.run_teleop.sys.stdin.isatty", return_value=True), \
                redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            result = main(["--devices", "arms", "--confirm-real"])
        self.assertEqual(result, 1)
        hardware_factory.assert_not_called()

    def test_invalid_home_is_rejected_before_viewer_or_hardware(self):
        config, _ = load_configuration(ROOT / "real_robot/config.json")
        config["staged_motion"]["home_left_rad"][0] = 999.0
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "config.json"
            path.write_text(json.dumps(config))
            with patch("real_robot.run_teleop.RealRobotViewer") as viewer_factory, \
                    patch("real_robot.run_teleop.make_hardware") as hardware_factory, \
                    patch("real_robot.run_teleop.sys.stdin.isatty", return_value=True), \
                    redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
                main(["--config", str(path), "--devices", "arms", "--confirm-real"])
            self.assertEqual(raised.exception.code, 2)
            viewer_factory.assert_not_called()
            hardware_factory.assert_not_called()

    def test_missing_controller_is_refused_before_hardware_connection(self):
        with tempfile.TemporaryDirectory() as folder, \
                patch("real_robot.run_teleop.ROOT", Path(folder) / "real_robot"), \
                patch("real_robot.run_teleop.make_hardware") as hardware_factory, \
                patch("real_robot.run_teleop.RealRobotViewer") as viewer_factory, \
                patch("real_robot.run_teleop.sys.stdin.isatty", return_value=True), \
                redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            result = main(["--config", str(ROOT / "real_robot/config.json"),
                           "--devices", "arms", "--confirm-real"])
        self.assertEqual(result, 1)
        hardware_factory.assert_not_called()
        viewer_factory.assert_not_called()

    def test_occupied_command_port_is_refused_before_hardware_connection(self):
        config, _ = load_configuration(ROOT / "real_robot/config.json")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as occupied, \
                tempfile.TemporaryDirectory() as folder:
            occupied.bind(("127.0.0.1", 0))
            config["command_port"] = occupied.getsockname()[1]
            config_path = Path(folder) / "config.json"
            config_path.write_text(json.dumps(config))
            with patch("real_robot.run_teleop.make_hardware") as hardware_factory, \
                    patch("real_robot.run_teleop.RealRobotViewer") as viewer_factory, \
                    patch("real_robot.run_teleop.sys.stdin.isatty", return_value=True), \
                    redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                result = main(["--config", str(config_path), "--devices", "arms", "--confirm-real"])
        self.assertEqual(result, 1)
        hardware_factory.assert_not_called()
        viewer_factory.assert_not_called()

    def test_all_stop_requests_precede_any_device_release(self):
        for failing_stop in (None, "right_hand"):
            with self.subTest(failing_stop=failing_stop), tempfile.TemporaryDirectory() as folder:
                executor_root = Path(folder) / "real_robot"
                viewer = executor_root.parent / "control/build/tianji_qp_ik_viewer"
                viewer.parent.mkdir(parents=True)
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
                with patch("real_robot.run_teleop.make_hardware", return_value=hardware), \
                        patch("real_robot.run_teleop.ROOT", executor_root), \
                        patch("real_robot.run_teleop.RealRobotViewer"), \
                        patch("real_robot.run_teleop.CommandReceiver"), \
                        patch("real_robot.run_teleop.sys.stdin.isatty", return_value=True), \
                        redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                    result = main(["--config", str(ROOT / "real_robot/config.json"),
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
        config, _ = load_configuration(ROOT / "real_robot/config.json")
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
            viewer_binary = base / "control/build/tianji_qp_ik_viewer"
            viewer_binary.parent.mkdir(parents=True)
            viewer_binary.touch()
            controller_config = base / "controller.yaml"
            controller_config.write_text("controller:\n  rate_hz: 200\n")
            if cleanup_failure:
                device.stop.side_effect = RuntimeError("UNCONFIRMED_DISABLE")
            replacements = {
                "ROOT": base / "real_robot",
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
                stack.enter_context(patch("real_robot.run_teleop." + name, replacement))
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



if __name__ == "__main__":
    unittest.main()
