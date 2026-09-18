"""Standalone HOME uses synthetic arm feedback and never accesses real devices."""
import contextlib
import io
import json
from pathlib import Path
import socket
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from real_robot import hardware, return_home
from real_robot.safety import SafetyFault

CONFIG = Path(__file__).resolve().parents[1] / "config.json"


class HomeTests(unittest.TestCase):
    def setUp(self):
        self.config, _ = return_home.load_configuration(CONFIG, "arms")
        self.home = tuple(self.config["staged_motion"]["home_left_rad"] +
                          self.config["staged_motion"]["home_right_rad"])
        self.now = 10_000_000_000
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserved:
            reserved.bind(("127.0.0.1", 0))
            self.config["command_port"] = reserved.getsockname()[1]
        self.config_path = Path(self.temporary.name) / "config.json"
        self.config_path.write_text(json.dumps(self.config))

    def feedback(self, positions=None, *, enabled=True, stamp=None):
        return SimpleNamespace(position_rad=self.home if positions is None else positions,
                               received_monotonic_ns=self.now if stamp is None else stamp,
                               enabled=enabled, healthy=True, detail="")

    def test_stale_feedback_tracking_error_and_clock_stall_latch_home_fault(self):
        for failure in ("stale", "tracking", "clock"):
            with self.subTest(failure=failure):
                gate = return_home.ArmHomeGate(self.config)
                gate.begin(self.feedback(), self.now)
                now = self.now + (200_000_000 if failure == "clock" else 5_000_000)
                positions = list(self.home)
                if failure == "tracking":
                    positions[0] += self.config["safety"]["arms"]["tracking_error_rad"] + .01
                expired = now - round(self.config["safety"]["feedback_timeout_s"] * 1e9) - 1
                measured = self.feedback(tuple(positions), stamp=expired if failure == "stale" else now)
                with self.assertRaises(SafetyFault):
                    gate.advance(measured, now)
                with self.assertRaises(SafetyFault):
                    gate.advance(self.feedback(stamp=now + 5_000_000), now + 5_000_000)

    def run_command(self, *, interrupt=False, enabled=False):
        owner = self
        actual = list(self.home)
        actual[0] += .01
        actual[7] -= .01
        state = SimpleNamespace(actual=tuple(actual), enabled=enabled, sends=[], cleanup=[])

        class Arms:
            def __init__(self, *args, **kwargs):
                pass

            def connect(self):
                pass

            def read_feedback(self):
                owner.now += 100_000
                return owner.feedback(state.actual, enabled=state.enabled)

            def enable(self, guard, *, allow_enabled=False):
                if state.enabled and not allow_enabled:
                    raise RuntimeError("existing control session")
                guard()
                state.enabled = True

            def send(self, positions):
                state.sends.append(tuple(positions))
                if interrupt:
                    raise KeyboardInterrupt
                state.actual = tuple(positions)

            def stop(self):
                state.cleanup.append("stop")
                state.enabled = False

            def close(self):
                state.cleanup.append("close")

        def sleep(seconds):
            self.now += round(seconds * 1e9)

        with patch.object(hardware, "MarvinDevice", Arms), \
                patch.object(hardware, "Hand2Device", side_effect=AssertionError("HOME touched a hand")), \
                patch.object(return_home.sys.stdin, "isatty", return_value=True), \
                patch.object(return_home.time, "monotonic_ns", side_effect=lambda: self.now), \
                patch.object(return_home.time, "monotonic", side_effect=lambda: self.now / 1e9), \
                patch.object(return_home.time, "sleep", side_effect=sleep), \
                contextlib.redirect_stdout(io.StringIO()):
            result = return_home.main(["--config", str(self.config_path), "--confirm-real"])
        return result, state

    def test_both_arms_reach_configured_home_and_disable_without_hand_access(self):
        result, state = self.run_command()
        self.assertEqual(result, 0)
        self.assertEqual(state.actual, self.home)
        self.assertEqual(state.cleanup, ["stop", "close"])
        self.assertFalse(state.enabled)

    def test_enabled_arms_are_homed_then_disabled(self):
        result, state = self.run_command(enabled=True)
        self.assertEqual(result, 0)
        self.assertEqual(state.actual, self.home)
        self.assertEqual(state.cleanup, ["stop", "close"])
        self.assertFalse(state.enabled)

    def test_home_uses_synchronized_bounded_trajectory_from_stationary_hold(self):
        gate = return_home.ArmHomeGate(self.config)
        start = list(self.home)
        start[0] += .1
        start[7] -= .05
        start = tuple(start)
        gate.begin(self.feedback(start), self.now)
        command = start
        old_velocity = (0.0,) * 14
        for _ in range(2000):
            self.now += 5_000_000
            previous = command
            command = gate.advance(self.feedback(previous), self.now)
            velocity = tuple((b - a) / .005 for a, b in zip(previous, command))
            self.assertAlmostEqual((start[0] - command[0]) / .1,
                                   (command[7] - start[7]) / .05, places=11)
            self.assertLessEqual(max(abs(v) for v in velocity), .1 + 1e-10)
            self.assertLessEqual(max(abs(v - old) / .005 for v, old in zip(velocity, old_velocity)),
                                 .2 + 1e-8)
            old_velocity = velocity
            if gate.phase == "HOME_REACHED":
                break
        self.assertEqual(gate.phase, "HOME_REACHED")
        self.assertEqual(command, self.home)
        self.assertTrue(gate.staged_stopped)

    def test_interrupt_stops_without_completing_home_and_closes(self):
        result, state = self.run_command(interrupt=True)
        self.assertEqual(result, 130)
        self.assertNotEqual(state.actual, self.home)
        self.assertEqual(state.cleanup, ["stop", "close"])
        self.assertFalse(state.enabled)

    def test_occupied_teleop_endpoint_refuses_home_before_hardware(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as occupied:
            occupied.bind(("127.0.0.1", self.config["command_port"]))
            with patch.object(return_home.sys.stdin, "isatty", return_value=True), \
                    patch.object(return_home, "make_hardware", side_effect=AssertionError("connected despite live executor")), \
                    contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(return_home.main(["--config", str(self.config_path), "--confirm-real"]), 1)

    def test_default_and_nonterminal_execution_cannot_connect_hardware(self):
        with patch.object(return_home, "make_hardware", side_effect=AssertionError("unexpected device access")), \
                patch.object(return_home.sys.stdin, "isatty", return_value=False), \
                contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(return_home.main([]), 0)
            self.assertEqual(return_home.main(["--confirm-real"]), 1)


if __name__ == "__main__":
    unittest.main()
