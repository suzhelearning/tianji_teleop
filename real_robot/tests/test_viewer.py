"""Viewer snapshot validation and owned-window IPC lifecycle; no GUI is required.

A stub child process replaces the MuJoCo window so the parent side of the
contract (validation, readiness, latest-state delivery, liveness, bounded
close) is exercised without a display.
"""
from pathlib import Path
import json
import sys
import tempfile
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from real_robot.viewer import (
    GROUP_ORDER,
    RealRobotViewer,
    SNAPSHOT_TIMEOUT_NS,
    _child_command,
    _display_state,
    _newest_sample,
    _parse_state_line,
    _state_line,
)

NOW_NS = 10_000_000_000

STUB = """import sys
out = open(sys.argv[1], "w")
{ready}
{suffix}
for line in sys.stdin:
    line = line.strip()
    if line == "STOP":
        break
    out.write(line + "\\n")
    out.flush()
out.close()
"""

ARMS = [0.1 * index for index in range(14)]
HANDS = [0.2] * 20


def positions(count, value=0.0):
    return [value] * count


def snapshot(actual, target, phase="TELEOP", timestamp_ns=NOW_NS):
    return _state_line(actual, target, phase, timestamp_ns)


class SnapshotValidationTests(unittest.TestCase):
    def test_partial_groups_are_kept_and_arm_order_is_preserved(self):
        line = snapshot({"arms": ARMS, "right_hand": HANDS}, {"arms": list(reversed(ARMS))}, "ALIGNING")
        self.assertEqual(line.count(b"\n"), 1)
        message = json.loads(line.decode("ascii"))
        self.assertEqual(message["phase"], "ALIGNING")
        self.assertEqual(message["actual"]["arms"], ARMS)
        self.assertNotIn("left_hand", message["actual"])
        self.assertEqual(message["target"]["arms"], list(reversed(ARMS)))

    def test_missing_groups_are_absent_rather_than_zero(self):
        message = json.loads(snapshot(None, None, "WAITING").decode("ascii"))
        self.assertEqual(message["actual"], {})
        self.assertEqual(message["target"], {})

    def test_wrong_length_and_non_finite_values_are_rejected(self):
        for bad in (positions(13), positions(15), positions(20)):
            with self.assertRaises(ValueError):
                snapshot({"arms": bad}, {})
        with self.assertRaises(ValueError):
            snapshot({"left_hand": positions(20)}, {"arms": positions(14) + [0.0]})
        for value in (float("nan"), float("inf"), float("-inf")):
            with self.assertRaises(ValueError):
                snapshot({"arms": positions(13) + [value]}, {})
        with self.assertRaises(ValueError):
            snapshot({"arms": "0123456789abcd"}, {})
        with self.assertRaises(ValueError):
            snapshot([0.0] * 14, {})

    def test_unsupported_group_and_phase_are_rejected(self):
        with self.assertRaises(ValueError):
            snapshot({"left_arm": positions(7)}, {})
        with self.assertRaises(ValueError):
            snapshot({}, {}, "TELEOP\ninjected")
        with self.assertRaises(ValueError):
            snapshot({}, {}, "")
        with self.assertRaises(ValueError):
            snapshot({}, {}, 7)
        with self.assertRaises(ValueError):
            snapshot({}, {}, "P" * 25)

    def test_lines_round_trip_and_malformed_lines_are_ignored(self):
        line = snapshot({"arms": ARMS, "left_hand": HANDS, "right_hand": HANDS}, {"arms": ARMS}, "READY")
        sample = _parse_state_line(line.rstrip(b"\n"))
        self.assertEqual(sample["phase"], "READY")
        self.assertEqual(sample["timestamp_ns"], NOW_NS)
        self.assertEqual(tuple(sample["actual"]["arms"]), tuple(ARMS))
        self.assertEqual(tuple(sample["target"]["arms"]), tuple(ARMS))
        stamped = '{"timestamp_ns":%d' % NOW_NS
        for broken in (b"", b"not json", b'{"phase":"TELEOP"}',
                       b'{"phase":"TELEOP","actual":[],"target":{},"timestamp_ns":1}',
                       b'{"phase":"TELEOP","actual":{"arms":[1,2]},"target":{},"timestamp_ns":1}',
                       b'{"phase":"","actual":{},"target":{},"timestamp_ns":1}',
                       (stamped + ',"phase":"TELEOP","actual":{},"target":{}}').replace(str(NOW_NS), "").encode(),
                       (stamped + ',"phase":"TELEOP","actual":{},"target":{}}').replace(
                           '"timestamp_ns":%d' % NOW_NS, '"timestamp_ns":0').encode(),
                       (stamped + ',"phase":"TELEOP","actual":{},"target":{}}').replace(
                           '"timestamp_ns":%d' % NOW_NS, '"timestamp_ns":"x"').encode()):
            self.assertIsNone(_parse_state_line(broken))

    def test_timestamp_must_be_a_positive_monotonic_integer(self):
        for bad in (0, -1, NOW_NS * 1.0, "10", None):
            with self.assertRaises(ValueError):
                _state_line({}, {}, "TELEOP", bad)

    def test_stale_snapshots_hide_the_measured_pose_and_say_so(self):
        message = json.loads(snapshot({"arms": ARMS}, {"arms": ARMS}, "TELEOP").decode("ascii"))
        key, sample, stale_s = _display_state(message, NOW_NS + 100_000_000)
        self.assertIsNone(stale_s)
        self.assertEqual(sample["actual"]["arms"], ARMS)
        self.assertEqual(tuple(key), (NOW_NS, None))
        for age in (SNAPSHOT_TIMEOUT_NS + 1, 2 * SNAPSHOT_TIMEOUT_NS, 5_000_000_000):
            key, sample, stale_s = _display_state(message, NOW_NS + age)
            self.assertEqual(sample["actual"], {})
            self.assertEqual(sample["target"]["arms"], ARMS)
            self.assertAlmostEqual(stale_s, age / 1e9, places=1)
            self.assertGreater(key[1], 0.0)
        key, sample, stale_s = _display_state(message, NOW_NS - 5_000_000)
        self.assertIsNone(stale_s)  # a small clock tolerance, as the executor allows
        _, _, stale_s = _display_state(message, NOW_NS - 1_000_000_000)
        self.assertIsNotNone(stale_s)
        self.assertIsNone(_display_state(None, NOW_NS))

    def test_only_the_newest_complete_snapshot_is_used(self):
        buffer = bytearray()
        sample, stop = _newest_sample(buffer, snapshot({"arms": ARMS}, {}, "ALIGNING"))
        self.assertEqual(sample["phase"], "ALIGNING")
        self.assertFalse(stop)
        newest = snapshot({"arms": ARMS}, {}, "TELEOP")
        sample, stop = _newest_sample(buffer, newest[:10])
        self.assertIsNone(sample)  # an incomplete line leaves the shown state alone
        self.assertEqual(bytes(buffer), newest[:10])
        sample, stop = _newest_sample(buffer, newest[10:] + b"STOP\n")
        self.assertEqual(sample["phase"], "TELEOP")
        self.assertTrue(stop)


class GeometryVisibilityTests(unittest.TestCase):
    def test_target_mesh_draws_use_the_same_meshes_as_native_rendering(self):
        import mujoco
        from real_robot.viewer import _apply_state, _fill_ghost_scene, _prepare_model

        root = Path(__file__).resolve().parents[2]
        model = mujoco.MjModel.from_xml_path(
            str(root / "control/models/marvin_m6_wuji2.xml"))
        indices, ghost_geoms = _prepare_model(mujoco, model)
        actual, target = mujoco.MjData(model), mujoco.MjData(model)
        options = mujoco.MjvOption()
        pose = {"arms": ARMS, "left_hand": HANDS, "right_hand": HANDS}
        sample = {"actual": pose, "target": pose}
        _apply_state(model, mujoco, actual, target, indices, options, sample)
        native = mujoco.MjvScene(model, maxgeom=1000)
        mujoco.mjv_updateScene(model, target, options, None, mujoco.MjvCamera(),
                              mujoco.mjtCatBit.mjCAT_ALL, native)
        expected = {
            geom.objid: geom.dataid for geom in native.geoms[:native.ngeom]
            if geom.objtype == mujoco.mjtObj.mjOBJ_GEOM
            and geom.type == mujoco.mjtGeom.mjGEOM_MESH
            and any(geom.objid in group for group in ghost_geoms.values())
        }
        scene = mujoco.MjvScene(model, maxgeom=1000)
        _fill_ghost_scene(model, mujoco, target, ghost_geoms, scene, sample)
        # These are renderer mesh references, not model mesh indices: mixing
        # the two draws another link (or its convex hull) at the correct pose.
        observed = {geom.objid: geom.dataid for geom in scene.geoms[:scene.ngeom]
                    if geom.type == mujoco.mjtGeom.mjGEOM_MESH}
        self.assertEqual(observed, expected)

    def test_hand_world_pose_requires_its_arm_pose(self):
        import mujoco
        from real_robot.viewer import GROUP_IDS, _apply_state, _fill_ghost_scene, _prepare_model

        root = Path(__file__).resolve().parents[2]
        model = mujoco.MjModel.from_xml_path(
            str(root / "control/models/marvin_m6_wuji2.xml"))
        indices, ghost_geoms = _prepare_model(mujoco, model)
        actual, target = mujoco.MjData(model), mujoco.MjData(model)
        options = mujoco.MjvOption()
        scene = mujoco.MjvScene(model, maxgeom=1000)
        sample = {"phase": "WAITING", "actual": {"left_hand": HANDS}, "target": {"left_hand": HANDS}}
        _apply_state(model, mujoco, actual, target, indices, options, sample)
        self.assertEqual(options.geomgroup[GROUP_IDS["left_hand"]], 0)
        _fill_ghost_scene(model, mujoco, target, ghost_geoms, scene, sample)
        self.assertEqual(scene.ngeom, 0)
        sample["actual"]["arms"] = ARMS
        sample["target"]["arms"] = ARMS
        _apply_state(model, mujoco, actual, target, indices, options, sample)
        self.assertEqual(options.geomgroup[GROUP_IDS["left_hand"]], 1)
        _fill_ghost_scene(model, mujoco, target, ghost_geoms, scene, sample)
        self.assertTrue(any(
            scene.geoms[index].objid in ghost_geoms["left_hand"] for index in range(scene.ngeom)))


class WindowLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.model = Path(self.directory.name) / "model.xml"
        self.model.write_text("<mujoco/>")
        import real_robot.viewer as viewer_module
        self.original = viewer_module._child_command
        self.addCleanup(setattr, viewer_module, "_child_command", self.original)

    def stub(self, ready=True, suffix=""):
        directory = Path(self.directory.name)
        recorded = directory / "recorded.jsonl"
        script = directory / "stub_child.py"
        script.write_text(STUB.format(
            ready='print("READY", flush=True)' if ready else "", suffix=suffix))
        command = [sys.executable, "-u", str(script), str(recorded)]
        import real_robot.viewer as viewer_module
        viewer_module._child_command = lambda model_path, python=None: command
        return recorded

    def test_ready_then_snapshots_are_delivered_and_close_is_bounded(self):
        recorded = self.stub()
        viewer = RealRobotViewer(self.model)
        self.assertTrue(viewer.is_running())
        for sequence in range(4):
            viewer.publish({"arms": [float(sequence)] * 14, "left_hand": HANDS, "right_hand": HANDS},
                           {"arms": [0.5] * 14}, "TELEOP")
        viewer.close()
        self.assertFalse(viewer.is_running())
        lines = recorded.read_text().splitlines()
        self.assertTrue(lines)
        self.assertIn("[3.0,", lines[-1])
        message = json.loads(lines[-1])
        self.assertEqual(message["actual"]["left_hand"], HANDS)
        self.assertGreater(message["timestamp_ns"], 0)
        viewer.close()  # idempotent
        self.assertFalse(viewer.is_running())

    def test_publish_never_blocks_when_the_window_stops_reading(self):
        self.stub(suffix="import time\ntime.sleep(5)\n")
        viewer = RealRobotViewer(self.model)
        started = time.monotonic()
        for _ in range(4000):
            viewer.publish({"arms": ARMS, "left_hand": HANDS, "right_hand": HANDS},
                           {"arms": ARMS}, "TELEOP")
        self.assertLess(time.monotonic() - started, 10.0)
        self.assertTrue(viewer.is_running())
        viewer.close()

    def test_publish_after_close_is_a_no_op_but_data_is_still_validated(self):
        self.stub()
        viewer = RealRobotViewer(self.model)
        viewer.close()
        viewer.publish({"arms": ARMS}, {"arms": ARMS}, "TELEOP")
        with self.assertRaises(ValueError):
            viewer.publish({"arms": ARMS[:-1]}, {}, "TELEOP")

    def test_error_report_before_readiness_raises_with_the_child_message(self):
        self.stub(ready=False, suffix='print("ERROR the viewer could not load model", flush=True)')
        with self.assertRaises(RuntimeError) as caught:
            RealRobotViewer(self.model)
        self.assertIn("could not load model", str(caught.exception))

    def test_silent_child_exit_raises_before_any_window_exists(self):
        self.stub(ready=False, suffix="raise SystemExit(3)")
        with self.assertRaises(RuntimeError):
            RealRobotViewer(self.model)

    def test_window_closed_after_ready_surfaces_as_not_running(self):
        self.stub(suffix="raise SystemExit(0)")
        viewer = RealRobotViewer(self.model)
        deadline = time.monotonic() + 5.0
        while viewer.is_running() and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertFalse(viewer.is_running())
        viewer.publish({"arms": ARMS}, {}, "TELEOP")  # a dead window never raises
        viewer.close()

    def test_invalid_arguments_are_rejected_before_spawning(self):
        with self.assertRaises(FileNotFoundError):
            RealRobotViewer(Path(self.directory.name) / "missing.xml")
        with self.assertRaises(ValueError):
            RealRobotViewer(self.model, startup_timeout_s=0.0)
        with self.assertRaises(ValueError):
            RealRobotViewer(self.model, startup_timeout_s=float("nan"))


class ChildCommandTests(unittest.TestCase):
    def test_command_runs_this_module_with_the_model(self):
        command = _child_command(Path("/tmp/model.xml"))
        self.assertEqual(command[1:3], ["-m", "real_robot.viewer"])
        self.assertEqual(command[3:], ["--model", "/tmp/model.xml"])


if __name__ == "__main__":
    unittest.main()
