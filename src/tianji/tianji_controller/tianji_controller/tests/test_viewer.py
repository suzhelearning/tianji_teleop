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

from tianji_runtime import workspace
from unittest.mock import Mock, patch
import subprocess

from tianji_controller.viewer import (
    GROUP_ORDER,
    MAX_STATE_BYTES,
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


def reference_target(**changes):
    result = {
        "wrist_pose": [0.35, -0.4, 0.8, 0.0, 0.0, 0.6, 0.8],
        "hand_joints": HANDS,
        "world_transform": [-0.25, 0.15, 0.3, 0.0, 0.6, 0.0, 0.8],
        "time_s": 0.0, "index": 0, "state": "APPROACH", "calibrated": False,
        "objects": {},
    }
    result.update(changes)
    return result


def object_target(**changes):
    result = {
        "target": [0.7, -0.2, 0.9, 0.0, 0.0, 0.6, 0.8],
        "actual": [-0.3, 0.4, 0.6, 0.0, 0.6, 0.0, 0.8],
        "status": "TRACKED", "timestamp_ns": NOW_NS, "timeout_ns": 100_000_000,
    }
    result.update(changes)
    return result


def snapshot(actual, target, phase="TELEOP", timestamp_ns=NOW_NS, *, reference=None, robot_pose=None):
    return _state_line(actual, target, phase, timestamp_ns,
                       reference=reference, robot_pose=robot_pose)


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


class ReferenceSnapshotTests(unittest.TestCase):
    def test_full_precision_all_groups_and_reference_fit_one_atomic_write(self):
        actual = {group: [-0.12345678901234567] * count
                  for group, count in (("arms", 14), ("left_hand", 20), ("right_hand", 20))}
        target = {group: [0.23456789012345678] * len(values) for group, values in actual.items()}
        pose = [-0.12345678901234567, -0.23456789012345678, -0.34567890123456789,
                -0.18257418583505536, -0.3651483716701107,
                -0.5477225575051661, -0.7302967433402214]
        reference = reference_target(
            hand_joints=[-0.34567890123456789] * 20,
            wrist_pose=pose, world_transform=pose,
            time_s=1234.1234567890123, index=123456,
            state="RUNNING", calibrated=True,
            objects={"hammer": object_target(target=pose, actual=pose)})
        line = snapshot(actual, target, reference=reference, robot_pose=pose)
        self.assertLessEqual(len(line), MAX_STATE_BYTES)
        sample = _parse_state_line(line)
        for label, expected in (("actual", actual), ("target", target)):
            for group, values in expected.items():
                self.assertEqual(sample[label][group], tuple(values))
        self.assertEqual(sample["reference"]["hand_joints"], tuple(reference["hand_joints"]))
        self.assertEqual(sample["reference"]["time_s"], reference["time_s"])
        self.assertEqual(sample["reference"]["objects"]["hammer"]["target"], tuple(pose))
        self.assertEqual(sample["reference"]["objects"]["hammer"]["actual"], tuple(pose))
        self.assertEqual(sample["robot_pose"], tuple(pose))

    def test_invalid_localization_rejects_entire_atomic_snapshot(self):
        for pose in ([0.0] * 7, [0.0] * 6,
                     [float("nan"), 0, 0, 0, 0, 0, 1], "not a pose"):
            with self.subTest(pose=pose):
                with self.assertRaises(ValueError):
                    snapshot({"arms": ARMS}, {}, robot_pose=pose)
                raw = json.loads(snapshot({"arms": ARMS}, {}))
                raw["robot_pose"] = pose
                self.assertIsNone(_parse_state_line(json.dumps(raw).encode()))

    def test_localized_source_states_survive_staleness_without_replacing_actual(self):
        pose = (1.2, -0.3, 0.5, 0.0, 0.0, 0.6, 0.8)
        for state in ("WAITING", "ENABLE", "APPROACH", "READY", "RUNNING", "PAUSED", "COMPLETE",
                      "HOME", "ARMED", "RETURN"):
            with self.subTest(state=state):
                sample = _parse_state_line(snapshot(
                    {"arms": ARMS}, {"arms": ARMS}, state, robot_pose=pose,
                    reference=reference_target(state=state, calibrated=True, objects={
                        "hammer": object_target(timeout_ns=10 * SNAPSHOT_TIMEOUT_NS)})))
                _, stale, _ = _display_state(sample, NOW_NS + SNAPSHOT_TIMEOUT_NS + 1)
                self.assertEqual(stale["robot_pose"], pose)
                self.assertEqual(stale["reference"]["state"], state)
                self.assertEqual(stale["actual"], {})
                self.assertIsNone(stale["reference"]["objects"]["hammer"]["actual"])
                self.assertEqual(stale["reference"]["objects"]["hammer"]["target"],
                                 sample["reference"]["objects"]["hammer"]["target"])

    def test_object_wire_contract_rejects_unknown_or_ambiguous_tracking(self):
        malformed = [
            object_target(target=[0.0] * 7),
            object_target(actual=[float("nan")] * 7),
            object_target(status="UNKNOWN"),
            object_target(status="STALE"),
            object_target(actual=None),
            object_target(timestamp_ns=None),
            object_target(timestamp_ns=True),
            object_target(timeout_ns=0),
            object_target(timeout_ns=1.5),
            {**object_target(), "extra": 1},
        ]
        for item in malformed:
            with self.subTest(item=item):
                reference = reference_target(objects={"hammer": item})
                with self.assertRaises(ValueError):
                    snapshot({}, {}, reference=reference)
                raw = json.loads(snapshot({}, {}))
                raw["reference"] = reference
                self.assertIsNone(_parse_state_line(json.dumps(raw).encode()))
        for objects in (None, [], {"bad\nname": object_target()}):
            with self.assertRaises(ValueError):
                snapshot({}, {}, reference=reference_target(objects=objects))

    def test_object_source_expires_before_snapshot_without_mutating_cache(self):
        sample = _parse_state_line(snapshot(
            {"arms": ARMS}, {"arms": ARMS},
            reference=reference_target(objects={"hammer": object_target()})))
        at_boundary, visible, age = _display_state(sample, NOW_NS + 100_000_000)
        self.assertIsNone(age)
        self.assertIsNotNone(visible["reference"]["objects"]["hammer"]["actual"])
        expired_key, expired, age = _display_state(sample, NOW_NS + 100_000_001)
        self.assertIsNone(age)
        self.assertNotEqual(at_boundary, expired_key)
        self.assertEqual(expired["actual"], sample["actual"])
        self.assertEqual(expired["reference"]["objects"]["hammer"],
                         {**sample["reference"]["objects"]["hammer"], "actual": None, "status": "STALE"})
        self.assertEqual(sample["reference"]["objects"]["hammer"]["status"], "TRACKED")
        self.assertIsNotNone(sample["reference"]["objects"]["hammer"]["actual"])

    def test_global_expiry_hides_objects_even_with_long_source_timeout(self):
        sample = _parse_state_line(snapshot(
            {"arms": ARMS}, {"arms": ARMS},
            reference=reference_target(objects={
                "hammer": object_target(timeout_ns=10 * SNAPSHOT_TIMEOUT_NS)})))
        _, stale, age = _display_state(sample, NOW_NS + SNAPSHOT_TIMEOUT_NS + 1)
        self.assertIsNotNone(age)
        self.assertEqual(stale["actual"], {})
        item = stale["reference"]["objects"]["hammer"]
        self.assertIsNone(item["actual"])
        self.assertEqual(item["status"], "STALE")
        self.assertEqual(item["target"], sample["reference"]["objects"]["hammer"]["target"])
        self.assertIsNotNone(sample["reference"]["objects"]["hammer"]["actual"])

    def test_untracked_objects_never_inherit_the_target_pose(self):
        objects = {name: object_target(status=status, actual=None, timestamp_ns=None)
                   for name, status in (("hammer", "UNTRACKED"), ("block", "UNAVAILABLE"))}
        sample = _parse_state_line(snapshot({}, {}, reference=reference_target(objects=objects)))
        _, visible, _ = _display_state(sample, NOW_NS)
        for name, item in visible["reference"]["objects"].items():
            self.assertIsNone(item["actual"])
            self.assertEqual(item["target"], tuple(objects[name]["target"]))
            self.assertEqual(item["status"], objects[name]["status"])

    def test_invalid_reference_rejects_the_entire_snapshot(self):
        invalid = [
            {"wrist_pose": [0.0] * 7},
            {"wrist_pose": [0.0, 0.0, float("inf"), 0.0, 0.0, 0.0, 1.0]},
            {"world_transform": [0.0] * 6},
            {"hand_joints": [0.0] * 19},
            {"hand_joints": [float("nan")] * 20},
            {"hand_joints": [7.0] * 20},
            {"time_s": -0.1}, {"time_s": float("inf")},
            {"index": -1}, {"index": 1.5}, {"index": True},
            {"state": "RUNNING\n"}, {"calibrated": 1},
        ]
        for changes in invalid:
            with self.subTest(changes=changes):
                reference = reference_target(**changes)
                with self.assertRaises(ValueError):
                    snapshot({"arms": ARMS}, {}, reference=reference)
                raw = json.loads(snapshot({"arms": ARMS}, {}))
                raw["reference"] = reference
                self.assertIsNone(_parse_state_line(json.dumps(raw).encode()))
        for reference in ({}, [], {"wrist_pose": [0.0] * 7}):
            with self.assertRaises(ValueError):
                snapshot({}, {}, reference=reference)

    def test_partial_and_invalid_messages_never_split_reference_from_robot_state(self):
        first = snapshot({"arms": ARMS}, {}, reference=reference_target(index=10))
        second = snapshot({}, {"arms": ARMS}, reference=reference_target(index=11))
        broken = json.loads(second)
        broken["reference"]["hand_joints"] = [0.0]
        buffer = bytearray()
        sample, _ = _newest_sample(buffer, first + second[:40])
        self.assertEqual(sample["reference"]["index"], 10)
        self.assertEqual(sample["actual"]["arms"], tuple(ARMS))
        sample, _ = _newest_sample(buffer, second[40:] + json.dumps(broken).encode() + b"\n")
        self.assertEqual(sample["reference"]["index"], 11)
        self.assertEqual(sample["actual"], {})
        self.assertEqual(sample["target"]["arms"], tuple(ARMS))

    def test_stale_snapshot_retains_reference_and_hides_only_actual(self):
        sample = _parse_state_line(snapshot({"arms": ARMS}, {"arms": ARMS},
                                            reference=reference_target(index=42, time_s=0.84)))
        _, stale, age = _display_state(sample, NOW_NS + SNAPSHOT_TIMEOUT_NS + 1)
        self.assertEqual(stale["actual"], {})
        self.assertEqual(stale["target"], sample["target"])
        self.assertEqual(stale["reference"], sample["reference"])
        self.assertGreater(age, 0.0)


class GeometryVisibilityTests(unittest.TestCase):
    def test_localization_places_robot_sites_static_meshes_and_data_in_one_frame(self):
        import mujoco
        import numpy as np
        from scipy.spatial.transform import Rotation
        from mocap_policy_runtime.integration.targets import model_wrist_pose
        from tianji_controller.viewer import (
            ReferenceOverlay, _apply_state, _fill_ghost_scene, _prepare_model,
        )

        model_path = Path(__file__).resolve().parents[5] / "src/tianji/tianji_description/models/marvin_m6_wuji2.xml"
        model = mujoco.MjModel.from_xml_path(str(model_path))
        indices, ghost_geoms = _prepare_model(mujoco, model)
        actual, control = mujoco.MjData(model), mujoco.MjData(model)
        options = mujoco.MjvOption()
        sample = {"actual": {"arms": ARMS, "right_hand": HANDS},
                  "target": {"arms": [-value for value in ARMS], "right_hand": HANDS}}
        _apply_state(model, mujoco, actual, control, indices, options, sample)
        originals = [
            {name: getattr(data, name).copy() for name in
             ("qpos", "ctrl", "geom_xpos", "geom_xmat", "site_xpos", "site_xmat")}
            for data in (actual, control)
        ]
        physical = {name: getattr(model, name).copy() for name in
                    ("body_pos", "body_quat", "geom_pos", "geom_quat", "site_pos", "site_quat")}
        wrists = [model_wrist_pose(model, data, "right") for data in (actual, control)]
        overlay = ReferenceOverlay(model, mujoco, indices)
        scene = mujoco.MjvScene(model, maxgeom=1000)
        first = np.r_[1.2, -0.3, 0.5, Rotation.from_euler("xyz", [0.2, -0.5, 1.1]).as_quat()]
        second = np.r_[-0.8, 0.4, 0.2, Rotation.from_euler("xyz", [-0.4, 0.3, -0.7]).as_quat()]
        # Repeating, replacing, and finally removing localization must never
        # accumulate a display transform, including on world-body base/stand.
        for pose in (first, first, second, None):
            rotation = Rotation.identity() if pose is None else Rotation.from_quat(pose[3:])
            translation = np.zeros(3) if pose is None else pose[:3]
            sample["robot_pose"] = pose
            _apply_state(model, mujoco, actual, control, indices, options, sample)
            for data, original, wrist in zip((actual, control), originals, wrists):
                np.testing.assert_array_equal(data.qpos, original["qpos"])
                np.testing.assert_array_equal(data.ctrl, original["ctrl"])
                for name in ("geom_xpos", "site_xpos"):
                    np.testing.assert_allclose(getattr(data, name),
                                               rotation.apply(original[name]) + translation, atol=1e-12)
                for name in ("geom_xmat", "site_xmat"):
                    np.testing.assert_allclose(getattr(data, name).reshape(-1, 3, 3),
                                               rotation.as_matrix() @ original[name].reshape(-1, 3, 3),
                                               atol=1e-12)
                shown_wrist = model_wrist_pose(model, data, "right")
                np.testing.assert_allclose(shown_wrist[:3], rotation.apply(wrist[:3]) + translation,
                                           atol=1e-12)
                np.testing.assert_allclose(Rotation.from_quat(shown_wrist[3:]).as_matrix(),
                                           (rotation * Rotation.from_quat(wrist[3:])).as_matrix(),
                                           atol=1e-12)
            reference = reference_target(
                wrist_pose=wrists[0], calibrated=True,
                world_transform=np.r_[rotation.inv().apply(-translation), rotation.inv().as_quat()])
            overlay.update(reference, robot_pose=pose)
            np.testing.assert_allclose(model_wrist_pose(model, overlay.data, "right")[:3],
                                       model_wrist_pose(model, actual, "right")[:3], atol=1e-12)
            np.testing.assert_allclose(overlay.reference["world_transform"][:3], 0, atol=1e-12)
            np.testing.assert_allclose(
                Rotation.from_quat(overlay.reference["world_transform"][3:]).as_matrix(),
                np.eye(3), atol=1e-12)
            scene.ngeom = 0
            overlay.draw(scene)
            for axis in range(3):
                origin_axis = scene.geoms[len(overlay.geoms) + 3 + axis]
                np.testing.assert_allclose(origin_axis.pos, 0.06 * np.eye(3)[axis], atol=1e-6)
                np.testing.assert_allclose(origin_axis.mat.reshape(3, 3)[:, 2],
                                           np.eye(3)[axis], atol=1e-6)
            scene.ngeom = 0
            _fill_ghost_scene(model, mujoco, control, ghost_geoms, scene, sample)
            for geom in scene.geoms[:scene.ngeom]:
                np.testing.assert_allclose(geom.pos, control.geom_xpos[geom.objid], atol=1e-6)
            native = mujoco.MjvScene(model, maxgeom=1000)
            mujoco.mjv_updateScene(model, actual, options, None, mujoco.MjvCamera(),
                                  mujoco.mjtCatBit.mjCAT_ALL, native)
            static = [geom for geom in native.geoms[:native.ngeom]
                      if geom.objtype == mujoco.mjtObj.mjOBJ_GEOM
                      and model.geom_bodyid[geom.objid] == 0]
            self.assertEqual(len(static), 4)
            for geom in static:
                index = geom.objid
                mesh = int(model.geom_dataid[index])
                start = int(model.mesh_vertadr[mesh])
                vertices = model.mesh_vert[start:start + int(model.mesh_vertnum[mesh])]
                original = (vertices @ originals[0]["geom_xmat"][index].reshape(3, 3).T
                            + originals[0]["geom_xpos"][index])
                shown = vertices @ geom.mat.reshape(3, 3).T + geom.pos
                np.testing.assert_allclose(shown, rotation.apply(original) + translation, atol=1e-6)
        for name, original in physical.items():
            np.testing.assert_array_equal(getattr(model, name), original)

    def test_target_mesh_draws_use_the_same_meshes_as_native_rendering(self):
        import mujoco
        from tianji_controller.viewer import _apply_state, _fill_ghost_scene, _prepare_model

        root = workspace()
        model = mujoco.MjModel.from_xml_path(
            str(root / "src/tianji/tianji_description/models/marvin_m6_wuji2.xml"))
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
        from tianji_controller.viewer import GROUP_IDS, _apply_state, _fill_ghost_scene, _prepare_model

        root = workspace()
        model = mujoco.MjModel.from_xml_path(
            str(root / "src/tianji/tianji_description/models/marvin_m6_wuji2.xml"))
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

    def test_reference_hand_world_placement_is_independent_of_both_joint_streams(self):
        import mujoco
        import numpy as np
        from scipy.spatial.transform import Rotation
        from mocap_policy_runtime.integration.targets import model_wrist_pose
        from tianji_controller.viewer import (
            GROUP_IDS, GHOST_RGBA, REFERENCE_RGBA, ReferenceOverlay,
            _apply_state, _fill_ghost_scene, _prepare_model,
        )

        root = workspace()
        model = mujoco.MjModel.from_xml_path(str(root / "src/tianji/tianji_description/models/marvin_m6_wuji2.xml"))
        indices, ghost_geoms = _prepare_model(mujoco, model)
        actual, control = mujoco.MjData(model), mujoco.MjData(model)
        options = mujoco.MjvOption()
        overlay = ReferenceOverlay(model, mujoco, indices)
        reference = reference_target()
        overlay.update(reference)
        wrist = model_wrist_pose(model, overlay.data, "right")
        np.testing.assert_allclose(wrist[:3], reference["wrist_pose"][:3], atol=1e-12)
        np.testing.assert_allclose(Rotation.from_quat(wrist[3:]).as_matrix(),
                                   Rotation.from_quat(reference["wrist_pose"][3:]).as_matrix(), atol=1e-12)
        expected_pos = overlay.data.geom_xpos[overlay.geoms].copy()
        expected_mat = overlay.data.geom_xmat[overlay.geoms].copy()
        for angle in (0.0, 0.7):
            sample = {"actual": {"arms": [angle] * 14, "right_hand": HANDS},
                      "target": {"arms": [-angle] * 14, "right_hand": [0.0] * 20}}
            _apply_state(model, mujoco, actual, control, indices, options, sample)
            overlay.update(reference)
            np.testing.assert_allclose(overlay.data.geom_xpos[overlay.geoms], expected_pos, atol=1e-12)
            np.testing.assert_allclose(overlay.data.geom_xmat[overlay.geoms], expected_mat, atol=1e-12)
            self.assertEqual(options.geomgroup[GROUP_IDS["right_hand"]], 1)
            scene = mujoco.MjvScene(model, maxgeom=1000)
            _fill_ghost_scene(model, mujoco, control, ghost_geoms, scene, sample)
            cyan_count = scene.ngeom
            overlay.draw(scene)
            np.testing.assert_allclose(scene.geoms[0].rgba, GHOST_RGBA)
            np.testing.assert_allclose(scene.geoms[cyan_count].rgba, REFERENCE_RGBA)
            native = mujoco.MjvScene(model, maxgeom=1000)
            mujoco.mjv_updateScene(model, actual, options, None, mujoco.MjvCamera(),
                                  mujoco.mjtCatBit.mjCAT_ALL, native)
            native_meshes = {geom.objid: geom.dataid for geom in native.geoms[:native.ngeom]
                             if geom.type == mujoco.mjtGeom.mjGEOM_MESH}
            for offset, index in enumerate(overlay.geoms):
                geom = scene.geoms[cyan_count + offset]
                self.assertEqual(geom.dataid, native_meshes[index])
                np.testing.assert_allclose(geom.pos, expected_pos[offset], atol=1e-6)
            # Capsule centers and directions locate both independent coordinate
            # systems, including a translated and rotated capture-world origin.
            axes_start = cyan_count + len(overlay.geoms)
            for offset, pose, length in ((0, reference["wrist_pose"], 0.06),
                                         (3, reference["world_transform"], 0.12)):
                rotation = Rotation.from_quat(pose[3:]).as_matrix()
                for axis in range(3):
                    geom = scene.geoms[axes_start + offset + axis]
                    np.testing.assert_allclose(
                        geom.pos, np.asarray(pose[:3]) + length * 0.5 * rotation[:, axis], atol=1e-6)
                    np.testing.assert_allclose(geom.mat.reshape(3, 3)[:, 2], rotation[:, axis], atol=1e-6)
        overlay.update(reference_target(hand_joints=[0.0] * 20))
        self.assertGreater(np.max(np.abs(overlay.data.geom_xpos[overlay.geoms] - expected_pos)), 1e-3)
        np.testing.assert_allclose(model_wrist_pose(model, overlay.data, "right")[:3],
                                   reference["wrist_pose"][:3], atol=1e-12)
        # Unlike cyan hands, a source wrist supplies its own world pose.
        sample = {"actual": {}, "target": {"right_hand": HANDS}}
        _apply_state(model, mujoco, actual, control, indices, options, sample)
        scene.ngeom = 0
        _fill_ghost_scene(model, mujoco, control, ghost_geoms, scene, sample)
        self.assertEqual(scene.ngeom, 0)
        overlay.draw(scene)
        self.assertEqual(scene.ngeom, len(overlay.geoms) + 6)
        overlay.update(None)
        scene.ngeom = 0
        overlay.draw(scene)
        self.assertEqual(scene.ngeom, 0)

    def test_object_axes_are_independent_robot_world_poses_and_expire_without_update(self):
        import mujoco
        import numpy as np
        from scipy.spatial.transform import Rotation
        from tianji_controller.viewer import (
            OBJECT_ACTUAL_RGBA, OBJECT_TARGET_RGBA, ReferenceOverlay, _prepare_model,
        )

        root = workspace()
        model = mujoco.MjModel.from_xml_path(str(root / "src/tianji/tianji_description/models/marvin_m6_wuji2.xml"))
        indices, _ = _prepare_model(mujoco, model)
        overlay = ReferenceOverlay(model, mujoco, indices)
        objects = {
            "hammer": object_target(),
            "block": object_target(actual=None, status="UNAVAILABLE", timestamp_ns=None),
        }
        overlay.update(reference_target(objects=objects))
        scene = mujoco.MjvScene(model, maxgeom=1000)
        with patch("tianji_controller.viewer.time.monotonic_ns", return_value=NOW_NS):
            overlay.draw(scene)
        start = len(overlay.geoms) + 6
        for offset, name, field, label, color in (
                (0, "hammer", "target", "TARGET", OBJECT_TARGET_RGBA),
                (4, "hammer", "actual", "REAL/LIVE", OBJECT_ACTUAL_RGBA),
                (8, "block", "target", "TARGET", OBJECT_TARGET_RGBA)):
            pose = objects[name][field]
            marker = scene.geoms[start + offset]
            self.assertEqual(marker.type, mujoco.mjtGeom.mjGEOM_SPHERE)
            self.assertEqual(marker.label, f"{name} {label}")
            np.testing.assert_allclose(marker.pos, pose[:3], atol=1e-6)
            np.testing.assert_allclose(marker.rgba, color, atol=1e-6)
            rotation = Rotation.from_quat(pose[3:]).as_matrix()
            for axis in range(3):
                geom = scene.geoms[start + offset + axis + 1]
                np.testing.assert_allclose(
                    geom.pos, np.asarray(pose[:3]) + 0.04 * rotation[:, axis], atol=1e-6)
                np.testing.assert_allclose(geom.mat.reshape(3, 3)[:, 2], rotation[:, axis], atol=1e-6)
        self.assertEqual(scene.ngeom, start + 12)
        scene.ngeom = 0
        with patch("tianji_controller.viewer.time.monotonic_ns", return_value=NOW_NS + 100_000_001):
            overlay.draw(scene)
        labels = [geom.label for geom in scene.geoms[:scene.ngeom]]
        self.assertIn("hammer TARGET", labels)
        self.assertIn("block TARGET", labels)
        self.assertNotIn("hammer REAL/LIVE", labels)
        self.assertNotIn("block REAL/LIVE", labels)
        self.assertEqual(overlay.reference["objects"]["hammer"]["status"], "TRACKED")
        for status in ("UNTRACKED", "UNAVAILABLE", "STALE"):
            overlay.update(reference_target(objects={
                "hammer": object_target(actual=None, status=status)}))
            scene.ngeom = 0
            overlay.draw(scene)
            labels = [geom.label for geom in scene.geoms[:scene.ngeom]]
            self.assertIn("hammer TARGET", labels)
            self.assertNotIn("hammer REAL/LIVE", labels)

    def test_hammer_mesh_preserves_obj_frame_and_expires_only_actual(self):
        import mujoco
        import numpy as np
        from scipy.spatial.transform import Rotation
        from tianji_description.model_assets import OBJECT_BODY_NAME, OBJECT_MESH_NAME
        from tianji_controller.viewer import (
            OBJECT_ACTUAL_RGBA, OBJECT_TARGET_RGBA, ReferenceOverlay, _prepare_model, _viewer_model,
        )

        model_path = Path(__file__).resolve().parents[5] / "src/tianji/tianji_description/models/marvin_m6_wuji2.xml"
        source = model_path.read_bytes()
        # Deliberately off-origin/asymmetric: centering or ignoring MuJoCo's
        # principal-axis transform must change the rendered world vertices.
        vertices = np.asarray([[0.12, 0.23, 0.31], [0.18, 0.23, 0.31],
                               [0.12, 0.32, 0.31], [0.12, 0.23, 0.43]])
        with tempfile.TemporaryDirectory() as directory:
            mesh_path = Path(directory) / "hammer.obj"
            mesh_path.write_text("".join(f"v {x} {y} {z}\n" for x, y, z in vertices)
                                 + "f 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n")
            model = _viewer_model(mujoco, model_path, mesh_path)
        baseline = _viewer_model(mujoco, model_path)
        self.assertEqual(model_path.read_bytes(), source)
        self.assertEqual((model.nq, model.nv, model.nu), (baseline.nq, baseline.nv, baseline.nu))
        np.testing.assert_array_equal(model.body_mass[:baseline.nbody], baseline.body_mass)
        np.testing.assert_array_equal(model.body_inertia[:baseline.nbody], baseline.body_inertia)
        indices, _ = _prepare_model(mujoco, model)
        overlay = ReferenceOverlay(model, mujoco, indices)
        index = model.geom(OBJECT_BODY_NAME).id
        mesh = model.mesh(OBJECT_MESH_NAME).id
        self.assertEqual(int(model.geom_contype[index]), 0)
        self.assertEqual(int(model.geom_conaffinity[index]), 0)
        self.assertEqual(model.body_mass[model.geom_bodyid[index]], 0)
        native = mujoco.MjvScene(model, maxgeom=1000)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        mujoco.mjv_updateScene(model, data, mujoco.MjvOption(), None, mujoco.MjvCamera(),
                              mujoco.mjtCatBit.mjCAT_ALL, native)
        self.assertFalse(any(geom.objid == index and geom.objtype == mujoco.mjtObj.mjOBJ_GEOM
                             for geom in native.geoms[:native.ngeom]))
        objects = {"hammer": object_target(), "block": object_target()}
        overlay.update(reference_target(objects=objects))
        scene = mujoco.MjvScene(model, maxgeom=1000)
        with patch("tianji_controller.viewer.time.monotonic_ns", return_value=NOW_NS):
            overlay.draw(scene)
        start = int(model.mesh_vertadr[mesh])
        compiled_vertices = model.mesh_vert[start:start + int(model.mesh_vertnum[mesh])]
        for label, field, color in (("hammer TARGET", "target", OBJECT_TARGET_RGBA),
                                    ("hammer REAL/LIVE", "actual", OBJECT_ACTUAL_RGBA)):
            slot = next(i for i in range(scene.ngeom) if scene.geoms[i].label == label)
            geom = scene.geoms[slot]
            self.assertEqual(geom.type, mujoco.mjtGeom.mjGEOM_MESH)
            self.assertEqual(geom.dataid, 2 * mesh)
            np.testing.assert_allclose(geom.rgba, color, atol=1e-6)
            pose = objects["hammer"][field]
            rotation = Rotation.from_quat(pose[3:]).as_matrix()
            expected = vertices @ rotation.T + pose[:3]
            observed = compiled_vertices @ geom.mat.reshape(3, 3).T + geom.pos
            distances = np.linalg.norm(observed[:, None] - expected[None, :], axis=-1)
            np.testing.assert_allclose(distances.min(axis=0), 0, atol=1e-6)
            np.testing.assert_allclose(distances.min(axis=1), 0, atol=1e-6)
            for axis in range(3):
                np.testing.assert_allclose(scene.geoms[slot + 1 + axis].pos,
                                           np.asarray(pose[:3]) + 0.04 * rotation[:, axis], atol=1e-6)
        block = next(geom for geom in scene.geoms[:scene.ngeom] if geom.label == "block TARGET")
        self.assertEqual(block.type, mujoco.mjtGeom.mjGEOM_SPHERE)
        # Localizing the robot composes object frames once, before applying the
        # compiler-origin compensation; raw OBJ vertices remain the reference.
        display_rotation = Rotation.from_euler("xyz", [0.3, -0.4, 1.1])
        display_translation = np.asarray([1.2, -0.6, 0.4])
        robot_pose = np.r_[display_translation, display_rotation.as_quat()]
        for _ in range(2):
            overlay.update(reference_target(objects=objects), robot_pose=robot_pose)
            scene.ngeom = 0
            with patch("tianji_controller.viewer.time.monotonic_ns", return_value=NOW_NS):
                overlay.draw(scene)
            for label, field in (("hammer TARGET", "target"), ("hammer REAL/LIVE", "actual")):
                slot = next(i for i in range(scene.ngeom) if scene.geoms[i].label == label)
                geom = scene.geoms[slot]
                pose = objects["hammer"][field]
                expected = display_rotation.apply(
                    Rotation.from_quat(pose[3:]).apply(vertices) + pose[:3]) + display_translation
                observed = compiled_vertices @ geom.mat.reshape(3, 3).T + geom.pos
                distances = np.linalg.norm(observed[:, None] - expected[None, :], axis=-1)
                np.testing.assert_allclose(distances.min(axis=0), 0, atol=1e-6)
                np.testing.assert_allclose(distances.min(axis=1), 0, atol=1e-6)
                rotation = display_rotation * Rotation.from_quat(pose[3:])
                origin = display_rotation.apply(pose[:3]) + display_translation
                for axis in range(3):
                    np.testing.assert_allclose(scene.geoms[slot + axis + 1].pos,
                                               origin + 0.04 * rotation.as_matrix()[:, axis], atol=1e-6)
                    np.testing.assert_allclose(scene.geoms[slot + axis + 1].mat.reshape(3, 3)[:, 2],
                                               rotation.as_matrix()[:, axis], atol=1e-6)
            self.assertEqual(overlay.reference["objects"]["hammer"]["timestamp_ns"], NOW_NS)
        scene.ngeom = 0
        with patch("tianji_controller.viewer.time.monotonic_ns", return_value=NOW_NS + 100_000_001):
            overlay.draw(scene)
        hammer_labels = [geom.label for geom in scene.geoms[:scene.ngeom] if geom.objid == index]
        self.assertEqual(hammer_labels, ["hammer TARGET"])

    def test_explicit_missing_or_invalid_mesh_fails_instead_of_axes_fallback(self):
        import mujoco
        from tianji_controller.viewer import _viewer_model

        model_path = Path(__file__).resolve().parents[5] / "src/tianji/tianji_description/models/marvin_m6_wuji2.xml"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "missing.obj"
            with self.assertRaises(FileNotFoundError):
                _viewer_model(mujoco, model_path, path)
            path.write_text("not an OBJ mesh")
            with self.assertRaises(ValueError):
                _viewer_model(mujoco, model_path, path)

    def test_reference_model_limits_are_rejected_not_clipped(self):
        import mujoco
        from tianji_controller.viewer import ReferenceOverlay, _prepare_model

        root = workspace()
        model = mujoco.MjModel.from_xml_path(str(root / "src/tianji/tianji_description/models/marvin_m6_wuji2.xml"))
        indices, _ = _prepare_model(mujoco, model)
        overlay = ReferenceOverlay(model, mujoco, indices)
        joints = HANDS.copy()
        joints[1] = 0.8  # Inside the wire bound, beyond the thumb abduction MJCF limit.
        with self.assertRaises(ValueError):
            overlay.update(reference_target(hand_joints=joints))


class RenderThreadLifecycleTests(unittest.TestCase):
    def test_localization_recenters_once_then_preserves_operator_camera(self):
        from contextlib import nullcontext
        import mujoco
        import numpy as np
        from scipy.spatial.transform import Rotation
        from tianji_controller.viewer import CAMERA_LOOKAT, ReferenceOverlay, _prepare_model, _render_loop

        model_path = Path(__file__).resolve().parents[5] / "src/tianji/tianji_description/models/marvin_m6_wuji2.xml"
        model = mujoco.MjModel.from_xml_path(str(model_path))
        indices, ghost_geoms = _prepare_model(mujoco, model)
        actual, control = mujoco.MjData(model), mujoco.MjData(model)
        overlay = ReferenceOverlay(model, mujoco, indices)
        viewer = Mock()
        viewer.opt = mujoco.MjvOption()
        viewer.cam = mujoco.MjvCamera()
        viewer.cam.lookat[:] = CAMERA_LOOKAT
        viewer.user_scn = mujoco.MjvScene(model, maxgeom=1000)
        viewer.lock.side_effect = nullcontext
        viewer.is_running.return_value = True
        positions = []

        def sync():
            positions.append(viewer.cam.lookat.copy())
            if len(positions) == 2:
                viewer.cam.lookat[:] = [9.0, 8.0, 7.0]  # user drags after localization

        viewer.sync.side_effect = sync
        pose = [1.2, -0.3, 0.5, 0.0, 0.0, 0.6, 0.8]
        moved = [-0.5, 1.0, 0.3, 0.0, 0.6, 0.0, 0.8]
        messages = [
            snapshot({}, {}, timestamp_ns=NOW_NS - 2),
            snapshot({"arms": ARMS}, {}, timestamp_ns=NOW_NS - 1, robot_pose=pose),
            snapshot({"arms": ARMS}, {}, robot_pose=moved,
                     reference=reference_target(wrist_pose=moved)),
            b"",
        ]
        with patch("tianji_controller.viewer.sys.stdin") as stdin, \
                patch("tianji_controller.viewer.os.set_blocking"), \
                patch("tianji_controller.viewer.os.read", side_effect=messages), \
                patch("tianji_controller.viewer.time.monotonic_ns", return_value=NOW_NS), \
                patch("tianji_controller.viewer.time.sleep"):
            stdin.fileno.return_value = 100
            _render_loop(mujoco, model, actual, control, indices, ghost_geoms, viewer, overlay)
        np.testing.assert_allclose(positions[0], CAMERA_LOOKAT)
        np.testing.assert_allclose(positions[1], Rotation.from_quat(pose[3:]).apply(CAMERA_LOOKAT) + pose[:3])
        np.testing.assert_allclose(positions[2], [9.0, 8.0, 7.0])

    def test_close_waits_for_render_teardown_not_only_exit_request(self):
        from threading import Event
        from types import SimpleNamespace
        from tianji_controller.viewer import OwnedPassiveWindow

        exit_requested, destroyed = Event(), Event()
        handle = SimpleNamespace(close=exit_requested.set)

        def launch(model, data, *, handle_return, **kwargs):
            handle_return.put(handle)
            exit_requested.wait()
            destroyed.set()

        mujoco = SimpleNamespace(viewer=SimpleNamespace(_launch_internal=launch),
                                 mj_forward=lambda model, data: None)
        window = OwnedPassiveWindow(mujoco, None, None)
        window.close()
        self.assertTrue(destroyed.is_set())
        self.assertFalse(window.thread.is_alive())

    def test_render_failure_after_startup_is_reported_during_close(self):
        from threading import Event
        from types import SimpleNamespace
        from tianji_controller.viewer import OwnedPassiveWindow

        exit_requested = Event()
        failure = RuntimeError("render teardown failed")

        def launch(model, data, *, handle_return, **kwargs):
            handle_return.put(SimpleNamespace(close=exit_requested.set))
            exit_requested.wait()
            raise failure

        mujoco = SimpleNamespace(viewer=SimpleNamespace(_launch_internal=launch),
                                 mj_forward=lambda model, data: None)
        window = OwnedPassiveWindow(mujoco, None, None)
        with self.assertRaises(RuntimeError) as caught:
            window.close()
        self.assertIs(caught.exception.__cause__, failure)
        self.assertFalse(window.thread.is_alive())


class WindowLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.model = Path(self.directory.name) / "model.xml"
        self.model.write_text("<mujoco/>")
        import tianji_controller.viewer as viewer_module
        self.original = viewer_module._child_command
        self.addCleanup(setattr, viewer_module, "_child_command", self.original)

    def stub(self, ready=True, suffix=""):
        directory = Path(self.directory.name)
        recorded = directory / "recorded.jsonl"
        script = directory / "stub_child.py"
        script.write_text(STUB.format(
            ready='print("READY", flush=True)' if ready else "", suffix=suffix))
        command = [sys.executable, "-u", str(script), str(recorded)]
        import tianji_controller.viewer as viewer_module
        viewer_module._child_command = lambda model_path, python=None, **kwargs: command
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
                           {"arms": ARMS, "left_hand": HANDS, "right_hand": HANDS}, "TELEOP",
                           reference=reference_target(objects={"hammer": object_target()}))
        self.assertLess(time.monotonic() - started, 10.0)
        self.assertTrue(viewer.is_running())
        with self.assertRaisesRegex(RuntimeError, "forced termination"):
            viewer.close()
        viewer.close()

    def test_nonzero_exit_during_close_is_reported_and_resources_released(self):
        self.stub(suffix='sys.stdin.readline(); raise SystemExit(7)')
        viewer = RealRobotViewer(self.model)
        with self.assertRaisesRegex(RuntimeError, "status 7"):
            viewer.close()
        self.assertIsNone(viewer._process)
        self.assertIsNone(viewer._state_fd)
        self.assertIsNone(viewer._status_fd)
        viewer.close()

    def test_child_error_with_zero_exit_is_not_successful_cleanup(self):
        self.stub(suffix='sys.stdin.readline(); print("ERROR render cleanup failed", flush=True); raise SystemExit(0)')
        viewer = RealRobotViewer(self.model)
        with self.assertRaisesRegex(RuntimeError, "render cleanup failed"):
            viewer.close()
        viewer.close()

    def test_crash_already_exited_is_not_silently_accepted(self):
        self.stub()
        viewer = RealRobotViewer(self.model)
        viewer.close()
        process = Mock()
        process.poll.return_value = -11
        viewer._process = process
        with self.assertRaisesRegex(RuntimeError, "status -11"):
            viewer.close()
        self.assertIsNone(viewer._process)

    def test_kill_timeout_retains_process_for_cleanup_retry(self):
        self.stub()
        viewer = RealRobotViewer(self.model)
        viewer.close()
        process = Mock()
        process.poll.return_value = None
        process.wait.side_effect = subprocess.TimeoutExpired('viewer', 2)
        viewer._process = process
        with self.assertRaisesRegex(RuntimeError, "could not be confirmed after kill"):
            viewer.close()
        self.assertIs(viewer._process, process)
        process.kill.assert_called_once()
        process.poll.return_value = -9
        with self.assertRaisesRegex(RuntimeError, "status -9"):
            viewer.close()
        self.assertIsNone(viewer._process)

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
        with self.assertRaises(FileNotFoundError):
            RealRobotViewer(self.model, object_mesh=Path(self.directory.name) / "missing.obj")
        with self.assertRaises(ValueError):
            RealRobotViewer(self.model, object_mesh=self.model)


class ChildCommandTests(unittest.TestCase):
    def test_command_runs_this_module_with_the_model(self):
        command = _child_command(Path("/tmp/model.xml"))
        self.assertEqual(command[1:3], ["-m", "tianji_controller.viewer"])
        self.assertEqual(command[3:], ["--model", "/tmp/model.xml"])


if __name__ == "__main__":
    unittest.main()
