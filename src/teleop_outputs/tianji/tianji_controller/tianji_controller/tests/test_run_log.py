"""Run-log and flight-recorder coverage; no SDK load, hardware connection or controller process."""
from pathlib import Path
from contextlib import redirect_stderr, redirect_stdout
import io
import json
import os
import sys
import tempfile
import unittest

from tianji_runtime import config_path as workspace_config
from unittest.mock import patch


from tianji_controller import run_log
from tianji_controller.run_log import SessionLog
from tianji_controller.run_teleop import main
from tianji_controller.tests.test_safety import NOW, feedback, frame


def open_session(directory, devices=("arms",), **overrides):
    values = dict(devices=devices, mode="real", config_source="/fake/config.json",
                  config={"active_devices": list(devices)})
    values.update(overrides)
    return SessionLog(Path(directory), **values)


def read_session(directory):
    return json.loads((Path(directory) / run_log.SESSION_FILE).read_text())


def read_flight_recorder(directory):
    text = (Path(directory) / run_log.FLIGHT_RECORDER_FILE).read_text()
    return [json.loads(line) for line in text.splitlines()]


class FlightRecorderTests(unittest.TestCase):
    def test_ring_is_bounded_in_samples_and_keeps_the_newest(self):
        with tempfile.TemporaryDirectory() as folder:
            log = open_session(folder)
            for index in range(5000):
                stamp = NOW + index * 1_000_000
                log.sample(frame(stamp=stamp, sequence=index + 1), feedback(stamp=stamp), "TELEOP", stamp)
            samples = log.recorder.samples
            self.assertEqual(len(samples), run_log.CAPACITY)
            self.assertEqual(samples[-1]["monotonic_ns"], NOW + 4999 * 1_000_000)
            self.assertEqual(samples[0]["monotonic_ns"], NOW + 3000 * 1_000_000)

    def test_ring_keeps_the_last_window_down_to_its_first_sample(self):
        with tempfile.TemporaryDirectory() as folder:
            log = open_session(folder)
            for index in range(200):
                stamp = NOW + index * 100_000_000
                log.sample(frame(stamp=stamp, sequence=index + 1), feedback(stamp=stamp), "TELEOP", stamp)
            samples = log.recorder.samples
            last = NOW + 199 * 100_000_000
            # Samples at the window horizon stay; anything older is dropped.
            self.assertEqual(len(samples), 101)
            self.assertEqual(samples[0]["monotonic_ns"], last - run_log.WINDOW_NS)
            self.assertEqual(samples[-1]["monotonic_ns"], last)

    def test_fault_sample_survives_with_reference_sent_and_feedback(self):
        with tempfile.TemporaryDirectory() as folder:
            log = open_session(folder)
            for index in range(3000):
                stamp = NOW + index * 1_000_000
                log.sample(frame(stamp=stamp, sequence=index + 1), feedback(stamp=stamp), "TELEOP", stamp)
            # The last accepted setpoint, then the frame a gate check is about to reject.
            log.record_send("arms", (0.25,) * 14)
            stamp = NOW + 3_000_000_000
            log.sample(frame(value=0.5, stamp=stamp, sequence=3001),
                       feedback(enabled=True, value=0.1, stamp=stamp), "TELEOP", stamp)
            log.finish(outcome="failed",
                       reason="SafetyFault: arms: physical feedback is not following the last command",
                       result=1, cleanup_errors=["arms cleanup could not be confirmed: boom"])
            lines = read_flight_recorder(folder)
            self.assertEqual(len(lines), run_log.CAPACITY)
            fault = lines[-1]
            self.assertEqual(fault["phase"], "TELEOP")
            self.assertEqual(fault["packet"]["sequence"], 3001)
            self.assertEqual(fault["reference"]["arms"][0], 0.5)
            self.assertEqual(fault["sent"]["arms"]["positions"][0], 0.25)
            self.assertGreater(fault["sent"]["arms"]["monotonic_ns"], 0)
            self.assertEqual(fault["feedback"]["arms"]["position_rad"][0], 0.1)
            self.assertTrue(fault["feedback"]["arms"]["enabled"])
            session = read_session(folder)
            self.assertEqual(session["outcome"], "failed")
            self.assertEqual(session["result"], 1)
            self.assertIn("not following", session["reason"])
            self.assertEqual(session["cleanup_errors"], ["arms cleanup could not be confirmed: boom"])
            self.assertEqual(session["flight_recorder"]["samples"], run_log.CAPACITY)
            self.assertEqual(session["devices"], ["arms"])
            self.assertEqual(session["mode"], "real")

    def test_finish_always_writes_both_record_files(self):
        with tempfile.TemporaryDirectory() as folder:
            log = open_session(folder, mode="dry-run")
            log.finish(outcome="completed", reason="dry run ended", result=0)
            self.assertEqual((Path(folder) / run_log.FLIGHT_RECORDER_FILE).read_text(), "")
            session = read_session(folder)
            self.assertEqual(session["outcome"], "completed")
            self.assertEqual(session["cleanup_errors"], [])
            self.assertEqual(session["flight_recorder"]["samples"], 0)

    def test_failed_flight_recorder_write_is_recorded_in_the_session_and_raised(self):
        with tempfile.TemporaryDirectory() as folder:
            log = open_session(folder)
            (Path(folder) / run_log.FLIGHT_RECORDER_FILE).mkdir()  # force the replace to fail
            log.sample(frame(stamp=NOW), feedback(stamp=NOW), "TELEOP", NOW)
            with self.assertRaises(OSError):
                log.finish(outcome="completed", reason="session ended", result=0)
            session = read_session(folder)
            self.assertEqual(session["outcome"], "failed")
            self.assertNotEqual(session["result"], 0)
            self.assertEqual(session["cleanup_errors"], [])
            self.assertTrue(session["write_errors"])
            self.assertIn(run_log.FLIGHT_RECORDER_FILE, session["write_errors"][0])
            # The partially written record never replaces the target.
            self.assertFalse((Path(folder) / (run_log.FLIGHT_RECORDER_FILE + ".tmp")).exists())

    def test_structured_fault_details_reach_the_session(self):
        with tempfile.TemporaryDirectory() as folder:
            log = open_session(folder)
            details = {"kind": "tracking_error", "device": "arms", "joint_index": 0,
                       "joint": "Joint1_L", "command_rad": 0.5, "actual_rad": 0.1,
                       "error_rad": 0.4, "absolute_error_rad": 0.4, "limit_rad": 0.14}
            log.finish(outcome="failed", reason="SafetyFault: arms: physical feedback is not following the last command",
                       result=1, error_details=details)
            self.assertEqual(read_session(folder)["error_details"], details)

    def test_unrepresentable_fault_details_do_not_break_the_session_record(self):
        with tempfile.TemporaryDirectory() as folder:
            log = open_session(folder)
            log.finish(outcome="failed", reason="SafetyFault: boom", result=1,
                       error_details={"kind": "tracking_error", "callback": object()})
            session = read_session(folder)
            self.assertEqual((session["outcome"], session["result"]), ("failed", 1))
            self.assertIn("tracking_error", str(session["error_details"]))

    def test_generated_controller_configuration_is_persisted_with_the_session(self):
        with tempfile.TemporaryDirectory() as folder:
            log = open_session(folder)
            generated = Path(folder) / "generated-controller.yaml"
            generated.write_text("controller:\n  rate_hz: 200\n")
            log.persist_controller_configuration(generated, source="/repo/config/qp_ik_pico_teleop.yaml")
            self.assertEqual((Path(folder) / run_log.CONTROLLER_CONFIG_FILE).read_text(),
                             "controller:\n  rate_hz: 200\n")
            self.assertEqual(read_session(folder)["controller_configuration"],
                             {"file": run_log.CONTROLLER_CONFIG_FILE,
                              "source_template": "/repo/config/qp_ik_pico_teleop.yaml"})


class EnvironmentTests(unittest.TestCase):
    def test_viewer_cleanup_crash_marks_interrupted_session_failed(self):
        with tempfile.TemporaryDirectory() as folder:
            with patch.dict(os.environ, {run_log.LOG_DIR_ENV: folder}), \
                    patch("tianji_controller.run_teleop.sys.stdin.isatty", return_value=True), \
                    patch("tianji_controller.run_teleop.Path.is_file", return_value=True), \
                    patch("tianji_controller.run_teleop.CommandReceiver"), \
                    patch("tianji_controller.run_teleop.RealRobotViewer") as viewer, \
                    patch("tianji_controller.run_teleop.ExecutorObserver"), \
                    patch("tianji_controller.run_teleop.make_hardware", side_effect=KeyboardInterrupt), \
                    redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                viewer.return_value.close.side_effect = RuntimeError("viewer process exited with status -11")
                self.assertEqual(main(["--confirm-real", "--devices", "arms"]), 1)
            session = read_session(folder)
            self.assertEqual(session["result"], 1)
            self.assertNotEqual(session["outcome"], "completed")
            self.assertTrue(any("visualization cleanup failed" in error and "-11" in error
                                for error in session["cleanup_errors"]))

    def test_unexpected_exception_is_recorded_as_nonzero_without_swallowing_it(self):
        with tempfile.TemporaryDirectory() as folder:
            with patch.dict(os.environ, {run_log.LOG_DIR_ENV: folder}), \
                    patch("tianji_controller.run_teleop.make_hardware", side_effect=TypeError("unexpected failure")), \
                    redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                with self.assertRaises(TypeError):
                    main(["--inspect", "--devices", "right_hand"])
            session = read_session(folder)
            self.assertEqual(session["outcome"], "exception")
            self.assertNotEqual(session["result"], 0)
            self.assertIn("unexpected failure", session["reason"])

    def test_controller_cleanup_failure_does_not_erase_the_original_failure_log(self):
        with tempfile.TemporaryDirectory() as folder:
            with patch.dict(os.environ, {run_log.LOG_DIR_ENV: folder}), \
                    patch("tianji_controller.run_teleop.make_hardware", side_effect=RuntimeError("original failure")), \
                    patch("tianji_controller.run_teleop.stop_controller", side_effect=OSError("cleanup failed")), \
                    redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                result = main(["--inspect", "--devices", "right_hand"])
            self.assertEqual(result, 1)
            session = read_session(folder)
            self.assertIn("original failure", session["reason"])
            self.assertTrue(any("cleanup failed" in error for error in session["cleanup_errors"]))
            self.assertTrue((Path(folder) / run_log.FLIGHT_RECORDER_FILE).is_file())

    def test_inspection_reports_failure_when_the_final_log_cannot_be_saved(self):
        import time
        from unittest.mock import Mock
        from tianji_controller.hardware import Feedback

        device = Mock()
        device.read_feedback.side_effect = lambda: Feedback((0.,) * 20, time.monotonic_ns(), True, False)
        with tempfile.TemporaryDirectory() as folder:
            with patch.dict(os.environ, {run_log.LOG_DIR_ENV: folder}), \
                    patch("tianji_controller.run_teleop.make_hardware", return_value={"right_hand": device}), \
                    patch.object(SessionLog, "finish", side_effect=OSError("disk full")), \
                    redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                result = main(["--inspect", "--devices", "right_hand"])
            self.assertEqual(result, 1)

    def test_absent_or_empty_variable_keeps_direct_entry_behavior(self):
        with patch.dict(os.environ, {}, clear=True):
            self.assertIsNone(SessionLog.from_environment(
                devices=("arms",), mode="real", config_source="/fake/config.json", config={}))
        with patch.dict(os.environ, {run_log.LOG_DIR_ENV: ""}):
            self.assertIsNone(SessionLog.from_environment(
                devices=("arms",), mode="real", config_source="/fake/config.json", config={}))

    def test_relative_log_directory_is_refused_without_creating_it(self):
        with patch.dict(os.environ, {run_log.LOG_DIR_ENV: "relative-logs"}):
            with self.assertRaises(ValueError):
                SessionLog.from_environment(
                    devices=("arms",), mode="real", config_source="/fake/config.json", config={})
        self.assertFalse(Path("relative-logs").exists())

    def test_unwritable_log_directory_refuses_before_hardware(self):
        with tempfile.TemporaryDirectory() as folder:
            blocker = Path(folder) / "blocker"
            blocker.write_text("not a directory")
            errors = io.StringIO()
            with patch.dict(os.environ, {run_log.LOG_DIR_ENV: str(blocker / "logs")}), \
                    patch("tianji_controller.run_teleop.make_hardware") as factory, \
                    redirect_stdout(io.StringIO()), redirect_stderr(errors):
                result = main(["--inspect", "--devices", "right_hand",
                               "--config", str(workspace_config("robot.json"))])
            self.assertEqual(result, 1)
            self.assertIn("RUN LOG REFUSED", errors.getvalue())
            factory.assert_not_called()

    def test_session_is_written_before_hardware_creation_and_keeps_the_failure(self):
        with tempfile.TemporaryDirectory() as folder:
            log_dir = Path(folder) / "logs"
            with patch.dict(os.environ, {run_log.LOG_DIR_ENV: str(log_dir)}), \
                    patch("tianji_controller.run_teleop.make_hardware", side_effect=RuntimeError("no hardware here")), \
                    redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                result = main(["--inspect", "--devices", "right_hand",
                               "--config", str(workspace_config("robot.json"))])
            self.assertEqual(result, 1)
            session = read_session(log_dir)
            self.assertEqual(session["outcome"], "failed")
            self.assertEqual(session["result"], 1)
            self.assertIn("no hardware here", session["reason"])
            self.assertEqual(session["mode"], "inspect")
            self.assertEqual(session["devices"], ["right_hand"])
            self.assertEqual(session["config_source"], str(workspace_config("robot.json").resolve()))
            self.assertTrue(session["config"]["active_devices"])
            self.assertTrue(session["python"]["version"])
            self.assertTrue(session["started_at"])
            self.assertTrue((log_dir / run_log.FLIGHT_RECORDER_FILE).exists())


if __name__ == "__main__":
    unittest.main()
