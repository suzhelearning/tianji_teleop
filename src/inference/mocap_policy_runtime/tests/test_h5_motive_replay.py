"""Regressions for capture-world calibration and source-time H5 replay."""
from types import SimpleNamespace
import h5py
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from mocap_policy_runtime.data.geometry import compose_pose, invert_pose
from mocap_policy_runtime.integration import hand
from mocap_policy_runtime.integration.h5_replay import H5Replay, motive_world_transform
from mocap_policy_runtime.replay.mocap_overlay import build_object_overlays, draw_mocap_overlay, overlay_geometry


def _capture(path):
    with h5py.File(path, "w") as stream:
        stream.attrs["h5_version"] = "5.0"
        stream.attrs["schema_name"] = "mocap-acquisition"
        stream.attrs["schema_layout"] = "compact-aligned-60hz-v1"
        stream["time_ns"] = np.arange(5, dtype=np.int64) * 1_000_000_000
        for side in ("left", "right"):
            group = stream.create_group(f"hands/{side}")
            group["valid"] = np.array([0, 1, 0, 1, 0], dtype=np.uint8)
            positions = np.c_[np.arange(5), np.zeros((5, 2))]
            group["wrist_position"] = positions
            group["wrist_quaternion_xyzw"] = np.tile([0., 0., 0., 1.], (5, 1))
            points = np.repeat(positions[:, None, :], 21, axis=1)
            points[:, 1, 0] += .05
            group["keypoints_world"] = points
        stream["hands/right/wuji2_joints"] = np.repeat((np.arange(5) * .1)[:, None], 20, axis=1)
    return path


def test_h5_source_time_trims_invalid_ends_bridges_gap_and_converts_only_wrist(tmp_path):
    replay = H5Replay(_capture(tmp_path / "capture.h5"), speed=4., yaw_deg=90.)
    assert replay.duration_s == 2.
    target = replay.sample(.5)
    # Speed is applied once by the session clock, never again by sampling.
    np.testing.assert_allclose(target.wrist_poses["right"][:3], [0., 1.5, 0.], atol=1e-12)
    expected = Rotation.from_euler("z", 90., degrees=True) * Rotation.from_quat([np.sqrt(.5), 0., -np.sqrt(.5), 0.])
    np.testing.assert_allclose(Rotation.from_quat(target.wrist_poses["right"][3:]).as_matrix(), expected.as_matrix(), atol=1e-12)
    assert set(target.wrist_poses) == {"right"}
    assert target.hand_keypoints == {}
    # A wrist dropout must not throw away otherwise finite canonical joints.
    np.testing.assert_allclose(replay.sample(1.).hand_joints["right"], np.full(20, .2))
    assert replay.sample(100.).index == 3
    assert replay.sample(100.).complete
    np.testing.assert_allclose(replay.sample(-1.).wrist_poses["right"][:3], [0., 1., 0.], atol=1e-12)


def test_home_recovers_robot_placement_and_aligns_overlay_with_target(tmp_path):
    replay = H5Replay(_capture(tmp_path / "capture.h5"), yaw_deg=90.)
    robot_home = np.r_[[.4, -.3, 1.], Rotation.from_euler("xyz", [80., -20., 15.], degrees=True).as_quat()]
    robot_in_mocap = np.r_[[1., 2., .3], Rotation.from_euler("xyz", [12., -25., 70.], degrees=True).as_quat()]
    # The tracker reports the calibrated robot wrist, expressed in mocap world.
    measured_home = compose_pose(robot_in_mocap, robot_home)
    transform = motive_world_transform(robot_home, measured_home)
    expected = invert_pose(robot_in_mocap)
    expected_rotation = Rotation.from_quat(expected[3:])
    np.testing.assert_allclose(transform[:3], expected[:3], atol=1e-12)
    np.testing.assert_allclose(Rotation.from_quat(transform[3:]).as_matrix(),
                               expected_rotation.as_matrix(), atol=1e-12)
    restored_home = compose_pose(transform, measured_home)
    np.testing.assert_allclose(restored_home[:3], robot_home[:3], atol=1e-12)
    np.testing.assert_allclose(Rotation.from_quat(restored_home[3:]).as_matrix(),
                               Rotation.from_quat(robot_home[3:]).as_matrix(), atol=1e-12)

    points, origin, endpoints = overlay_geometry(replay.preview(.5), transform)
    target = compose_pose(transform, replay.sample(.5).wrist_poses["right"])
    np.testing.assert_allclose(origin, expected[:3], atol=1e-12)
    np.testing.assert_allclose(endpoints - origin, expected_rotation.apply(np.eye(3) * .2), atol=1e-12)
    np.testing.assert_allclose(points[0], expected[:3] + expected_rotation.apply([0., 1.5, 0.]), atol=1e-12)
    np.testing.assert_allclose(points[0], target[:3], atol=1e-12)
    # The skeleton retains Manus axes; only the control wrist converts to WuJi.
    np.testing.assert_allclose(points[1] - points[0], expected_rotation.apply([0., .05, 0.]), atol=1e-12)
    expected_wrist_rotation = (expected_rotation * Rotation.from_euler("z", 90., degrees=True)
                               * Rotation.from_quat([np.sqrt(.5), 0., -np.sqrt(.5), 0.]))
    np.testing.assert_allclose(Rotation.from_quat(target[3:]).as_matrix(),
                               expected_wrist_rotation.as_matrix(), atol=1e-12)


def test_explicit_world_rotation_rotates_home_translation_and_axes(tmp_path):
    replay = H5Replay(_capture(tmp_path / "capture.h5"))
    rotation = Rotation.from_euler("z", 30., degrees=True)
    measured = np.array([1., 2., 3., 0., 0., 0., 1.])
    robot = np.array([.4, -.3, 1., 0., 0., 0., 1.])
    transform = motive_world_transform(robot, measured, rotation.as_quat())
    np.testing.assert_allclose(rotation.apply(measured[:3]) + transform[:3], robot[:3])
    _, origin, endpoints = overlay_geometry(replay.preview(0.), transform)
    np.testing.assert_allclose(endpoints - origin, rotation.apply(np.eye(3) * .2), atol=1e-12)


def test_preview_holds_last_valid_shape_but_interpolates_wrist(tmp_path):
    path = _capture(tmp_path / "capture.h5")
    with h5py.File(path, "r+") as stream:
        stream["hands/right/keypoints_world"][3, 1, 0] = 3.15
    replay = H5Replay(path)
    preview = replay.preview(1.)
    np.testing.assert_allclose(preview.wrist_poses["right"][:3], [2., 0., 0.])
    np.testing.assert_allclose(preview.hand_keypoints["right"][1], [.05, 0., 0.], atol=1e-12)
    np.testing.assert_allclose(replay.preview(2.).hand_keypoints["right"][1], [.15, 0., 0.], atol=1e-12)


def _raw_capture(path):
    _capture(path)
    with h5py.File(path, "r+") as stream:
        del stream["hands/right/wuji2_joints"]
        # Distinct valid shapes make chronological solver/filter state visible.
        stream["hands/right/keypoints_world"][3, 1, 0] = 3.15
    return path


def _regrind(path):
    with h5py.File(path, "w") as stream:
        stream["regrind_retargeting_root_pos"] = np.c_[np.arange(5), np.zeros((5, 2))]
        rotation = Rotation.from_euler("xyz", [25., -15., 40.], degrees=True).as_quat()
        stream["regrind_retargeting_root_quat"] = np.tile(rotation[[3, 0, 1, 2]], (5, 1))
        stream["regrind_retargeting_joints"] = np.repeat((np.arange(5) * .1)[:, None], 20, axis=1)
        stream["object_pos"] = np.c_[np.arange(5), np.ones(5), np.zeros(5)]
        stream["object_quat"] = np.tile([1., 0., 0., 0.], (5, 1))
    return path


@pytest.mark.parametrize("rate_hz", [25., 50.])
def test_regrind_replays_wuji_axes_and_source_clock_without_retargeting(tmp_path, monkeypatch, rate_hz):
    def unexpected_solver(*args, **kwargs):
        raise AssertionError("solved H5 must not construct a retargeter")

    monkeypatch.setattr(hand, "HandRetargeter", unexpected_solver)
    replay = H5Replay(_regrind(tmp_path / "ik.h5"), rate_hz=rate_hz, speed=4., yaw_deg=90.)
    assert replay.trajectory.format == "regrind"
    assert replay.hand_mode == "regrind"
    assert replay.duration_s == 4. / rate_hz
    target = replay.sample(1.5 / rate_hz)
    np.testing.assert_allclose(target.hand_joints["right"], np.full(20, .15))
    np.testing.assert_allclose(target.wrist_poses["right"][:3], [0., 1.5, 0.], atol=1e-12)
    yaw = Rotation.from_euler("z", 90., degrees=True)
    expected = yaw * Rotation.from_euler("xyz", [25., -15., 40.], degrees=True)
    np.testing.assert_allclose(Rotation.from_quat(target.wrist_poses["right"][3:]).as_matrix(),
                               expected.as_matrix(), atol=1e-12)
    np.testing.assert_allclose(target.object_poses["hammer"][:3], [-1., 1.5, 0.], atol=1e-12)
    np.testing.assert_allclose(Rotation.from_quat(target.object_poses["hammer"][3:]).as_matrix(),
                               yaw.as_matrix(), atol=1e-12)
    preview = replay.preview(1.5 / rate_hz)
    assert preview.hand_keypoints == {}
    np.testing.assert_allclose(preview.wrist_poses["right"], target.wrist_poses["right"])
    np.testing.assert_allclose(preview.hand_joints["right"], target.hand_joints["right"])
    assert not target.complete
    assert replay.sample(4. / rate_hz).complete


def test_raw_retargeting_is_chronological_precomputed_and_bridges_tracking_gaps(tmp_path, monkeypatch):
    solvers = []

    class StatefulRetargeter:
        def __init__(self, side):
            self.accumulated = 0.
            self.closed = False
            solvers.append(self)

        def retarget(self, points):
            assert not self.closed
            # A stateful filter must never be stepped by preview, seeking or ticks.
            self.accumulated += points[1, 0]
            return np.full(20, self.accumulated)

        def close(self):
            self.closed = True

    monkeypatch.setattr(hand, "HandRetargeter", StatefulRetargeter)
    replay = H5Replay(_raw_capture(tmp_path / "raw.h5"), rate_hz=200., speed=4.)
    assert len(solvers) == 1 and solvers[0].closed
    assert replay.hand_mode == "retargeted"
    assert replay.duration_s == 2.  # Acquisition timestamps ignore rate_hz.
    # Source shapes .05 and .15 solve to .05 then .20 in chronological order.
    np.testing.assert_allclose(replay.hand_joint_track.values, np.repeat([[.05], [.20]], 20, axis=1))
    for time_s, value in ((2., .20), (0., .05), (1., .125), (.5, .0875), (1., .125)):
        target = replay.sample(time_s)
        np.testing.assert_allclose(target.hand_joints["right"], np.full(20, value))
        np.testing.assert_allclose(replay.preview(time_s).hand_joints["right"], np.full(20, value))
    np.testing.assert_allclose(replay.preview(1.).hand_keypoints["right"][1], [.05, 0., 0.])
    np.testing.assert_allclose(replay.sample(1.).wrist_poses["right"][:3], [2., 0., 0.])


@pytest.mark.parametrize("failure", ["exception", "nonfinite", "wrong_shape", "timeout"])
def test_raw_future_retarget_failure_aborts_construction(tmp_path, monkeypatch, failure):
    solvers = []

    class FailingRetargeter:
        def __init__(self, side):
            self.closed = False
            solvers.append(self)

        def retarget(self, points):
            if points[1, 0] < .1:
                return np.full(20, .1)
            if failure == "exception":
                raise ValueError("optimization failed")
            if failure == "timeout":
                raise TimeoutError("hand worker response timed out")
            if failure == "nonfinite":
                return np.full(20, np.nan)
            return np.full(19, .1)

        def close(self):
            self.closed = True

    monkeypatch.setattr(hand, "HandRetargeter", FailingRetargeter)
    with pytest.raises(ValueError, match="source frame 3"):
        H5Replay(_raw_capture(tmp_path / "raw.h5"))
    assert len(solvers) == 1 and solvers[0].closed


@pytest.mark.parametrize("failure,expected", [
    ("startup", RuntimeError),
    ("wrong_shape", RuntimeError),
    ("nonfinite", RuntimeError),
    ("sequence", RuntimeError),
    ("timeout", TimeoutError),
])
def test_isolated_hand_pipe_failure_reaps_worker_without_restarting(tmp_path, monkeypatch, request, failure, expected):
    import subprocess
    import sys

    # Exercise the real bounded pipe transport, without importing the Manus
    # solver or depending on its native ABI in the policy test environment.
    interpreter = tmp_path / ".pixi/envs/manus/bin/python"
    interpreter.parent.mkdir(parents=True)
    interpreter.symlink_to(sys.executable)
    worker = tmp_path / "hand_worker.py"
    worker.write_text(
        "import json, os, sys, time\n"
        f"failure = {failure!r}\n"
        "assert not any(key in os.environ for key in "
        "('PYTHONPATH', 'PYTHONHOME', 'LD_PRELOAD', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH'))\n"
        "assert sys.flags.isolated\n"
        "request = json.loads(sys.stdin.readline())\n"
        "side = request['side']\n"
        "kind = 'error' if failure == 'startup' else 'ready'\n"
        "print(json.dumps(dict(kind=kind, sequence=0, side=side, error='solver initialization failed')), flush=True)\n"
        "if failure == 'startup': sys.stdin.read(); sys.exit(0)\n"
        "request = json.loads(sys.stdin.readline())\n"
        "if failure == 'timeout': time.sleep(10)\n"
        "joints = [float('nan')] * 20 if failure == 'nonfinite' else [0.] * (19 if failure == 'wrong_shape' else 20)\n"
        "sequence = request['sequence'] + (failure == 'sequence')\n"
        "print(json.dumps(dict(kind='joints', sequence=sequence, side=side, joints=joints)), flush=True)\n"
        "sys.stdin.read()\n"
    )
    monkeypatch.setattr(hand, "workspace", lambda: tmp_path)
    monkeypatch.setattr(hand, "__file__", str(tmp_path / "hand.py"))
    for key in ("PYTHONPATH", "PYTHONHOME", "LD_PRELOAD", "LD_LIBRARY_PATH", "AMENT_PREFIX_PATH"):
        monkeypatch.setenv(key, "/foreign/environment")
    processes = []
    popen = subprocess.Popen

    def launch(*args, **kwargs):
        process = popen(*args, **kwargs)
        processes.append(process)
        def reap():
            if process.poll() is None:
                process.kill()
            process.wait()
        request.addfinalizer(reap)
        return process

    monkeypatch.setattr(hand.subprocess, "Popen", launch)
    if failure == "startup":
        with pytest.raises(expected):
            hand.HandRetargeter("right", startup_timeout=3.)
    else:
        with hand.HandRetargeter("right", timeout=.25, startup_timeout=3.) as solver:
            with pytest.raises(expected):
                solver.retarget(np.zeros((21, 3)))
            assert processes[0].poll() is not None
            # A failed solver must not recreate a subprocess or reuse old joints.
            with pytest.raises(RuntimeError):
                solver.retarget(np.zeros((21, 3)))
            solver.close()
    assert len(processes) == 1
    assert processes[0].poll() is not None


def test_canonical_forward_fill_is_not_retargeted_and_rejects_missing_initial_joints(tmp_path, monkeypatch):
    def unexpected_solver(*args, **kwargs):
        raise AssertionError("canonical joints must not construct a retargeter")

    monkeypatch.setattr(hand, "HandRetargeter", unexpected_solver)
    path = _capture(tmp_path / "canonical.h5")
    with h5py.File(path, "r+") as stream:
        stream["hands/right/wuji2_joints"][2] = np.nan
    replay = H5Replay(path)
    np.testing.assert_allclose(replay.sample(1.).hand_joints["right"], np.full(20, .1))
    np.testing.assert_allclose(replay.sample(1.5).hand_joints["right"], np.full(20, .2))
    with h5py.File(path, "r+") as stream:
        stream["hands/right/wuji2_joints"][:2] = np.nan
    with pytest.raises(ValueError, match="unavailable at first valid wrist"):
        H5Replay(path)


def test_acquisition_object_poses_share_trim_and_yaw_but_retain_tracking_invalidity(tmp_path):
    path = _capture(tmp_path / "objects.h5")
    with h5py.File(path, "r+") as stream:
        group = stream.create_group("objects/tool")
        group["valid"] = np.array([1, 1, 0, 1, 1], dtype=np.uint8)
        group["object_position"] = np.c_[np.arange(5), np.ones(5), np.zeros(5)]
        group["object_quaternion_xyzw"] = np.tile([0., 0., 0., 1.], (5, 1))
    replay = H5Replay(path, yaw_deg=90.)
    np.testing.assert_allclose(replay.sample(0.).object_poses["tool"][:3], [-1., 1., 0.], atol=1e-12)
    assert replay.sample(1.).object_poses == {}
    np.testing.assert_allclose(replay.preview(2.).object_poses["tool"][:3], [-1., 3., 0.], atol=1e-12)


def test_object_overlays_share_world_transform_without_wrist_conversion_or_aliasing():
    transform = np.r_[[.4, -.2, .7], Rotation.from_euler("xyz", [35., -10., 60.], degrees=True).as_quat()]
    target = np.r_[[.2, .3, .4], Rotation.from_euler("x", 20., degrees=True).as_quat()]
    measured = np.r_[[.5, .6, .7], Rotation.from_euler("y", 40., degrees=True).as_quat()]
    saved = [pose.copy() for pose in (transform, target, measured)]
    sample = SimpleNamespace(received_at=10., hammer_xyzw=measured)
    objects = build_object_overlays({"hammer": target, "tool": target}, sample, transform,
                                    now=10.1, stale_s=.2)
    hammer = objects["hammer"]
    np.testing.assert_allclose(hammer["target"], compose_pose(transform, target))
    np.testing.assert_allclose(hammer["actual"], compose_pose(transform, measured))
    assert hammer["status"] == "TRACKED"
    assert hammer["timestamp_ns"] == 10_000_000_000
    assert hammer["timeout_ns"] == 200_000_000
    np.testing.assert_allclose(objects["tool"]["target"], compose_pose(transform, target))
    assert objects["tool"]["actual"] is None
    assert objects["tool"]["status"] == "UNAVAILABLE"
    assert objects["tool"]["timestamp_ns"] is None
    for result in objects.values():
        assert set(result) == {"target", "actual", "status", "timestamp_ns", "timeout_ns"}
    hammer["target"][:] = 0
    hammer["actual"][:] = 0
    for original, untouched in zip((transform, target, measured), saved):
        np.testing.assert_array_equal(original, untouched)


def test_object_overlay_freshness_and_tracking_loss_never_substitute_target():
    pose = np.array([0., 0., 0., 0., 0., 0., 1.])
    sample = SimpleNamespace(received_at=10., hammer_xyzw=pose)
    target = pose.copy()
    target[0] = 1.

    def overlay(value, now=10.1, error=None, targets=None):
        return build_object_overlays({"hammer": target} if targets is None else targets,
                                     value, pose, now=now, stale_s=.2, error=error)["hammer"]

    fresh = overlay(sample)
    assert fresh["actual"][0] == 0. and fresh["target"][0] == 1.
    stale = overlay(sample, now=10.3)
    assert stale["status"] == "STALE" and stale["actual"] is None
    assert stale["timestamp_ns"] == fresh["timestamp_ns"]
    np.testing.assert_array_equal(stale["target"], target)
    future = overlay(sample, now=9.9)
    assert future["status"] == "STALE" and future["actual"] is None
    sample.received_at = 10.3
    sample.hammer_xyzw = None
    untracked = overlay(sample, now=10.4)
    assert untracked["status"] == "UNTRACKED" and untracked["actual"] is None
    assert untracked["timestamp_ns"] == 10_300_000_000
    for value, error in ((None, None), (sample, "tracker disconnected")):
        unavailable = overlay(value, now=10.4, error=error)
        assert unavailable["status"] == "UNAVAILABLE"
        assert unavailable["actual"] is None and unavailable["timestamp_ns"] is None
        np.testing.assert_array_equal(unavailable["target"], target)
    sample.hammer_xyzw = pose
    no_target = overlay(sample, now=10.4, targets={})
    assert no_target["status"] == "TRACKED" and no_target["target"] is None
    np.testing.assert_array_equal(no_target["actual"], pose)


def test_source_skeleton_can_omit_axes_without_clearing_other_overlays(tmp_path):
    mujoco = pytest.importorskip("mujoco")
    replay = H5Replay(_capture(tmp_path / "capture.h5"))
    pose = np.array([0., 0., 0., 0., 0., 0., 1.])
    model = mujoco.MjModel.from_xml_string("<mujoco/>")
    scene = mujoco.MjvScene(model, maxgeom=42)
    scene.ngeom = 1
    scene.geoms[0].label = "existing overlay"
    draw_mocap_overlay(scene, replay.preview(0.), pose, clear=False, draw_axes=False)
    assert scene.geoms[0].label == "existing overlay"
    assert scene.ngeom == 42  # Existing overlay + 20 bones + 21 points; no axes.
    points, _, _ = overlay_geometry(replay.preview(0.), pose)
    np.testing.assert_allclose(scene.geoms[21].pos, points[0])
    assert not any(geom.label.startswith("Mocap") for geom in scene.geoms[:scene.ngeom])
    axes_scene = mujoco.MjvScene(model, maxgeom=48)
    draw_mocap_overlay(axes_scene, replay.preview(0.), pose)
    assert {geom.label for geom in axes_scene.geoms[:axes_scene.ngeom] if geom.label} == {
        "Mocap O", "Mocap X", "Mocap Y", "Mocap Z"}


@pytest.mark.parametrize("speed", [0., -1., float("nan"), float("inf")])
def test_simulation_return_rejects_invalid_speed_before_opening_resources(speed):
    from mocap_policy_runtime.integration.h5_simulation import H5SimulationSession, main

    with pytest.raises(ValueError, match="return speed must be positive and finite"):
        H5SimulationSession(None, None, None, None, return_speed=speed)
    with pytest.raises(ValueError, match="return speed must be positive and finite"):
        main(["--h5", "unused.h5", "--return-speed", str(speed)])
