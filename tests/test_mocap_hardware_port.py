"""Offline source-workflow and SDK-mode contracts; never connect real devices."""
import hashlib
from pathlib import Path
import sys
from types import SimpleNamespace

import h5py
import numpy as np
import pytest

from mocap_policy_runtime.integration import real
from real_robot import hardware


def _recording(path, *, joints=True):
    with h5py.File(path, "w") as stream:
        stream.attrs["h5_version"] = "4.0"
        stream.attrs["schema_name"] = "mocap-acquisition"
        stream["time_ns"] = np.array([0, 100_000_000, 200_000_000], dtype=np.int64)
        for side in ("left", "right"):
            hand = stream.create_group(f"hands/{side}")
            hand["valid"] = np.ones(3, dtype=np.uint8)
            hand["wrist_position"] = np.c_[[.004, .005, .006], np.zeros((3, 2))]
            hand["wrist_quaternion_xyzw"] = np.tile([np.sqrt(.5), 0, -np.sqrt(.5), 0], (3, 1))
            hand["keypoints_world"] = np.repeat(hand["wrist_position"][:][:, None, :], 21, axis=1)
            if joints:
                hand["wuji2_joints"] = np.tile([.005, .015, .025], (20, 1)).T
        obj = stream.create_group("objects/hammer")
        obj["valid"] = np.ones(3, dtype=np.uint8)
        obj["object_position"] = np.c_[[.02, .03, .04], np.full(3, .01), np.zeros(3)]
        obj["object_quaternion_xyzw"] = np.tile([0., 0., 0., 1.], (3, 1))
    return path


def _h5_source(monkeypatch, path, mode):
    if mode == "regrind":
        with h5py.File(path, "w") as stream:
            stream["regrind_retargeting_root_pos"] = np.c_[[.004, .005, .006], np.zeros((3, 2))]
            stream["regrind_retargeting_root_quat"] = np.tile([1., 0., 0., 0.], (3, 1))
            stream["regrind_retargeting_joints"] = np.tile([.005, .015, .025], (20, 1)).T
            stream["object_pos"] = np.c_[[.02, .03, .04], np.full(3, .01), np.zeros(3)]
            stream["object_quat"] = np.tile([1., 0., 0., 0.], (3, 1))
        return path
    if mode == "retargeted":
        class Retargeter:
            def __init__(self, root, side):
                self.index = 0
            def retarget(self, points):
                result = np.full(20, .005 + self.index * .01)
                self.index += 1
                return result
        monkeypatch.setitem(sys.modules, "retargeting.example.tj_wuji2_hand_bridge",
                            SimpleNamespace(HandRetargeter=Retargeter))
    return _recording(path, joints=mode == "canonical")


@pytest.mark.parametrize("mode", ["canonical", "retargeted", "regrind"])
@pytest.mark.parametrize("bad_value", [np.nan, 100.])
def test_h5_real_rejects_unsafe_prepared_hand_before_hardware(monkeypatch, tmp_path, mode, bad_value):
    from mocap_policy_runtime.integration import h5_replay

    path = _h5_source(monkeypatch, tmp_path / "unsafe.h5", mode)
    replay_type = h5_replay.H5Replay
    def unsafe_replay(*args, **kwargs):
        replay = replay_type(*args, **kwargs)
        replay.hand_joint_track.values[-1, 0] = bad_value
        return replay
    monkeypatch.setattr(h5_replay, "H5Replay", unsafe_replay)
    monkeypatch.setattr(real.sys.stdin, "isatty", lambda: True)
    monkeypatch.setattr(real, "make_hardware", lambda *a: pytest.fail("hardware opened before hand preflight"))
    assert real.main(["--h5", str(path), "--confirm-real"]) == 1


@pytest.mark.parametrize("bad_value", [np.nan, 100.])
def test_h5_sim_rejects_unsafe_prepared_hand_before_movement(tmp_path, bad_value):
    from mocap_policy_runtime.integration.h5_replay import H5Replay
    from mocap_policy_runtime.integration.h5_simulation import H5SimulationSession
    from mocap_policy_runtime.integration.native import safety_limits

    replay = H5Replay(_recording(tmp_path / "unsafe.h5"))
    replay.hand_joint_track.values[-1, 0] = bad_value
    lower, upper = safety_limits()
    adapter = SimpleNamespace(current_positions=np.zeros(54), lower=lower, upper=upper)
    simulation = SimpleNamespace(data=SimpleNamespace(time=0.))
    # No qpos or command methods: admission must fail before moving simulation.
    with pytest.raises(ValueError):
        H5SimulationSession(replay, adapter, simulation, SimpleNamespace())


def test_regrind_overlay_preserves_world_axes_without_skeleton(monkeypatch, tmp_path):
    from scipy.spatial.transform import Rotation
    from mocap_policy_runtime.integration.h5_replay import H5Replay
    from mocap_policy_runtime.replay.mocap_overlay import overlay_geometry

    replay = H5Replay(_h5_source(monkeypatch, tmp_path / "ik.h5", "regrind"))
    rotation = Rotation.from_euler("xyz", [30., -20., 10.], degrees=True)
    transform = np.r_[[1., 2., 3.], rotation.as_quat()]
    points, origin, endpoints = overlay_geometry(replay.preview(0.), transform)
    assert points.shape == (0, 3)
    np.testing.assert_allclose(origin, transform[:3])
    np.testing.assert_allclose(endpoints, rotation.apply(np.eye(3) * .2) + transform[:3])


def test_h5_inputs_allow_no_hammer_but_never_stale_wrist(monkeypatch):
    pose = np.array([0., 0., 0., 0., 0., 0., 1.])
    sample = SimpleNamespace(received_at=10., frame_number=1, wrist_xyzw=pose, hammer_xyzw=None)
    tracker = SimpleNamespace(error=None, latest=lambda: sample)
    settings = dict(motive_stale_s=.5, arm_stale_s=.15, hand_stale_s=.04, maximum_input_skew_s=.02)
    feedback = {name: hardware.Feedback(tuple(np.zeros(n)), 10_000_000_000, True, True)
                for name, n in (("arms", 14), ("right_hand", 20))}
    monkeypatch.setattr(real.time, "monotonic_ns", lambda: 10_001_000_000)
    inputs = real.PolicyInputs(tracker, settings, require_object=False)
    assert inputs.read(feedback, 10_000_000_000)[0] is sample
    with pytest.raises(real.SafetyFault):
        real.PolicyInputs(tracker, settings).read(feedback, 10_000_000_000)
    tracker.latest = lambda: None
    with pytest.raises(real.SafetyFault):
        inputs.read(feedback, 10_000_000_000)


def _exercise_source(monkeypatch, tmp_path, *, lose_motive=False, lose_object=False,
                     policy_mode=False, hand_mode="canonical", never_enable=False,
                     quit_phase=None, miss_frame0=None, hammer_mismatch=False, legacy_mode=False,
                     foreign_enter=None, localization_jump=False, reauthorize=False, enable_shift=False):
    """Drive the actual run loop + MotionGate with simulated feedback and keys."""
    from mocap_policy_runtime.integration import native, targets, real_keyboard
    from mocap_policy_runtime.policies.regrind import tracking

    config, _ = real.load_configuration(real.ROOT.parent / "real_robot/config.json")
    home = np.array(config["staged_motion"]["home_left_rad"] + config["staged_motion"]["home_right_rad"])
    # Neither arm nor hand begins at configured Home/zero.
    home[0] += .05
    home[7] += .2
    pose = np.array([0., 0., 0., 0., 0., 0., 1.])
    initial_wrist = np.array([.5, .2, -.1, 0., 0., np.sqrt(.5), np.sqrt(.5)])
    first_motive = pose.copy()
    first_motive[0] = .002
    snapshots, source_targets, calibrations, adapter_calls, policy_calls = [], [], [], [], []
    milestones = {}
    from mocap_policy_runtime.data.geometry import compose_pose, invert_pose

    def wrist_fk(q):
        local = pose.copy()
        local[0] = (q[7] - home[7]) * .2
        return compose_pose(initial_wrist, local)

    monkeypatch.setattr(real, "MeasuredWrist", lambda model: wrist_fk)
    clock = SimpleNamespace(now=100.)
    monkeypatch.setattr(real.time, "monotonic", lambda: clock.now)
    monkeypatch.setattr(real.time, "monotonic_ns", lambda: round(clock.now * 1e9))
    monkeypatch.setattr(real.time, "sleep", lambda seconds: setattr(clock, "now", clock.now + max(seconds, .000001)))
    monkeypatch.setattr(real.sys.stdin, "isatty", lambda: True)
    monkeypatch.setattr("builtins.input", lambda: pytest.fail("startup must not require typed authorization"))

    class Device:
        def __init__(self, name):
            self.name, self.enabled, self.closed = name, False, False
            self.position = home.copy() if name == "arms" else np.full(20, .2)
            self.commands = []
            self.modes = []
        def connect(self):
            pass
        def read_feedback(self):
            return hardware.Feedback(tuple(self.position), round(clock.now * 1e9), True, self.enabled)
        def enable(self, guard):
            assert clock.now - 100 >= .2
            assert not self.commands
            milestones.setdefault("enable", clock.now - 100)
            guard()
            clock.now += .08  # SDK latency must not become a loop-stall fault.
            guard()
            if enable_shift and self.name == "arms":
                self.position[0] += .02
            self.enabled = True
        def send(self, command):
            if not (miss_frame0 == "hand" and self.name == "right_hand"):
                self.position = np.array(command)
            self.commands.append((clock.now - 100, np.array(command)))
        def set_right_impedance(self, enabled, *, guard, **kwargs):
            assert self.enabled
            guard()
            clock.now += .08
            guard()
            self.modes.append(enabled)
        def stop(self):
            self.enabled = False
        def close(self):
            self.closed = True

    devices = {}
    def make_hardware(config, selected, base):
        for name in selected:
            devices[name] = Device(name)
        return {name: devices[name] for name in selected}
    monkeypatch.setattr(real, "make_hardware", make_hardware)

    class IK:
        def __init__(self, **kwargs):
            self.lower, self.upper = native.safety_limits()
        def synchronize(self, q):
            pass
        def capture_home(self):
            pass
        def close(self):
            pass
    class Adapter:
        def __init__(self, ik, initial_positions, **kwargs):
            self.q = initial_positions.copy()
            self.home_wrist = {"right": initial_wrist.copy()}
            self.transform = initial_wrist.copy()
        def set_world_transform(self, transform):
            calibrations.append(transform.copy())
            self.transform = transform.copy()
        def frame(self, target):
            adapter_calls.append(clock.now - 100)
            assert not target.hand_keypoints
            source_targets.append(target)
            if legacy_mode:
                self.q[7:14] = target.arm_joints["right"]
            else:
                desired = compose_pose(compose_pose(invert_pose(initial_wrist), self.transform),
                                       target.wrist_poses["right"])
                goal = home[7] + desired[0] * 5.
                self.q[7] += np.clip(goal - self.q[7], -.001, .001)
            self.q[34:] = target.hand_joints["right"]
            return self.q.copy()
        def hold(self, q):
            adapter_calls.append(clock.now - 100)
            self.q = q.copy()
    monkeypatch.setattr(native, "NativeIK", IK)
    monkeypatch.setattr(targets, "TargetAdapter", Adapter)
    def publish(actual, control, phase, *, reference=None, robot_pose=None):
        snapshots.append(dict(
            t=clock.now - 100, actual={k: np.array(v) for k, v in actual.items()},
            control={k: np.array(v) for k, v in control.items()}, phase=phase,
            robot_pose=None if robot_pose is None else robot_pose.copy(),
            enabled=any(device.enabled for device in devices.values()),
            source=source_targets[-1] if source_targets else None,
            reference=None if reference is None else {
                k: v.copy() if isinstance(v, np.ndarray) else v for k, v in reference.items()}))
    monkeypatch.setattr(real, "RealRobotViewer", lambda *a, **kwargs: SimpleNamespace(
        is_running=lambda: True, publish=publish, close=lambda: None))

    class Keyboard:
        def __init__(self, hold_enter):
            assert hold_enter is not legacy_mode
            self.legacy_events = set()
        def poll(self):
            t = clock.now - 100
            reference = snapshots[-1]["reference"] if snapshots else None
            state = reference["state"] if reference is not None else snapshots[-1]["phase"] if snapshots else "WAITING"
            if quit_phase == state:
                milestones["quit"] = t
                return {"q"}, False
            if foreign_enter == "enable" or (foreign_enter == "approach" and state in ("APPROACH", "HOLD")):
                return set(), int(t * 20) % 2 == 1  # Global key state, no terminal input.
            if legacy_mode:
                event = None
                if t >= .2 and state == "ENABLE":
                    event = "enable"
                elif t >= .55 and state in ("APPROACH", "HOLD"):
                    event = "approach"
                elif state == "READY":
                    event = "run"
                    if "run" not in milestones:
                        milestones.update(run=t, pause=t + .07, resume=t + .27)
                elif "run" in milestones:
                    if t >= milestones["resume"]:
                        event = "resume"
                    elif t >= milestones["pause"]:
                        event = "pause"
                if event is not None and event not in self.legacy_events:
                    self.legacy_events.add(event)
                    return {"enter"}, False
                return set(), False
            if never_enable:
                return {"enter", "s", "r", "i"}, True
            if state in ("WAITING", "PLANNING") and "enable" not in milestones:
                return set(), False
            if state == "ENABLE":
                milestones.setdefault("preview", t)
                return ({"enter"}, True) if t >= milestones["preview"] + .1 else (set(), False)
            if "enable" not in milestones:
                return set(), False
            if localization_jump and "ready" in milestones:
                if state == "RELOCALIZING":
                    milestones.setdefault("drift", t)
                if state == "HOLD" and "drift" in milestones:
                    milestones.setdefault("relocked", t)
                if "drift" in milestones:
                    if not reauthorize or "relocked" not in milestones:
                        return {"enter"}, True
                    since = t - milestones["relocked"]
                    if since < .1:
                        return {"enter"}, True
                    if since < .2:
                        return set(), False
                    milestones.setdefault("reauthorize", t)
                    return {"enter"}, True
            milestones.setdefault("enabled_hold", t)
            approach = milestones["enabled_hold"] + .15
            milestones["approach"] = approach
            milestones["release"] = approach + .8
            milestones["approach_resume"] = milestones["release"] + .75
            if t < approach:
                return set(), False
            if t < milestones["release"]:
                return {"enter"}, True
            if t < milestones["approach_resume"]:
                return set(), False
            if state == "READY" and "ready" not in milestones:
                milestones["ready"] = t
                milestones["run"] = t + (.40 if hammer_mismatch else .20)
                milestones["pause"] = milestones["run"] + .07
                milestones["resume"] = milestones["run"] + .27
            if "ready" in milestones:
                ready, run = milestones["ready"], milestones["run"]
                held = (t < ready + .15 or ready + .20 <= t < ready + .23
                        or run <= t < run + .03
                        or milestones["pause"] <= t < milestones["pause"] + .05
                        or milestones["resume"] <= t < milestones["resume"] + .05)
                return {"enter", "s", "r", "i"} if held else set(), held
            return {"enter"}, True
        def close(self):
            pass
    class Tracker:
        error = None
        def __init__(self, session, **kwargs):
            assert kwargs["require_object"] is policy_mode
        def latest(self):
            if clock.now < 100.05:
                return None
            if lose_motive and clock.now - 100 >= milestones.get("pause", float("inf")) + .05:
                return None
            wrist = pose.copy()
            wrist[0] = (devices["arms"].position[7] - home[7]) * .2
            if miss_frame0 == "wrist" and wrist[0] >= .0038:
                wrist[0] += .2
            if localization_jump and "ready" in milestones:
                wrist[0] += .01
            hammer = pose.copy()
            hammer[:3] = [.021 + (clock.now - 100) * .001, .01, 0.]
            if hammer_mismatch and clock.now - 100 < milestones.get("ready", float("inf")) + .35:
                hammer[0] += 1.
            if lose_object and clock.now - 100 >= milestones.get("pause", float("inf")) + .05:
                hammer = None
            return SimpleNamespace(received_at=clock.now, frame_number=round(clock.now * 1e9),
                                   wrist_xyzw=wrist, hammer_xyzw=hammer)
        def close(self):
            pass
    monkeypatch.setattr(real_keyboard, "OperatorKeyboard", Keyboard)
    monkeypatch.setattr(tracking, "RegrindMotiveTracker", Tracker)
    monkeypatch.setattr(tracking, "open_mocap_session", lambda endpoint: SimpleNamespace(close=lambda: None))
    path = _h5_source(monkeypatch, tmp_path / "direct.h5", hand_mode)
    arguments = ["--h5", str(path), "--rate-hz", "10"]
    if legacy_mode:
        from mocap_policy_runtime import data
        from mocap_policy_runtime.data.trajectory import Track, Trajectory
        timeline = np.array([0., .1, .2])
        arm = np.tile(home[7:], (3, 1))
        arm[:, 0] += [.02, .03, .04]
        trajectory = Trajectory(
            path, "session", timeline,
            {"arm_joints": {"right": Track(timeline, arm, np.ones(3, dtype=bool), "linear")},
             "hand_joints": {"right": Track(timeline, np.tile([.005, .015, .025], (20, 1)).T,
                                             np.ones(3, dtype=bool), "linear")}},
            space="robot_world_tcp", metadata={"source_type": "joint_replay"})
        monkeypatch.setattr(data, "load_trajectory", lambda *a, **kwargs: trajectory)
        arguments += ["--mode", "joint"]
    if policy_mode:
        import yaml
        from mocap_policy_runtime.policies.regrind import runtime
        from mocap_policy_runtime.types import TargetFrame
        class Actor:
            def __init__(self, *args, **kwargs):
                self.reference = SimpleNamespace(
                    wrist_pos=np.tile([.004, 0., 0.], (2, 1)),
                    wrist_quat_wxyz=np.tile([1., 0., 0., 0.], (2, 1)),
                    joints=np.full((2, 20), .005), object_pos=np.tile([.02, .01, 0.], (2, 1)),
                    object_quat_wxyz=np.tile([1., 0., 0., 0.], (2, 1)))
                self.complete = False
                self.steps = 0
            def reset(self, wrist, joints):
                policy_calls.append(("reset", clock.now - 100))
                self.steps, self.complete = 0, False
            def observe(self, wrist, joints, **kwargs):
                policy_calls.append(("observe", clock.now - 100))
            def step(self, wrist, hammer, joints):
                policy_calls.append(("step", clock.now - 100))
                self.steps += 1
                self.complete = self.steps >= 10
                return TargetFrame(self.steps * .02, self.steps, complete=self.complete,
                                   wrist_poses={"right": pose + np.array([.004 + self.steps * .0002, 0, 0, 0, 0, 0, 0])},
                                   hand_joints={"right": np.full(20, .005 + self.steps * .002)},
                                   object_poses={"hammer": np.r_[[.02 + self.steps * .002, .01, 0.], pose[3:]]})
        monkeypatch.setattr(runtime, "RegrindPolicy", Actor)
        model = tmp_path / "offline_actor.pt"
        model.write_bytes(b"offline scenario actor, never a hardware model")
        profile = yaml.safe_load((real.ROOT / "configs/regrind.yaml").read_text())
        profile["checkpoint_sha256"] = hashlib.sha256(model.read_bytes()).hexdigest()
        profile["reference_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
        profile_path = tmp_path / "profile.yaml"
        profile_path.write_text(yaml.safe_dump(profile))
        arguments = ["--policy", "regrind", "--model", str(model), "--reference", str(path),
                     "--policy-config", str(profile_path)]
    result = real.main(arguments + ["--confirm-real", "--duration", "12" if localization_jump else "8"])
    evidence = SimpleNamespace(snapshots=snapshots, calibrations=calibrations,
                               initial_wrist=initial_wrist, first_motive=first_motive,
                               milestones=milestones, adapter_calls=adapter_calls, policy_calls=policy_calls)
    return result, devices, home, evidence


@pytest.mark.parametrize("source", ["canonical", "retargeted", "regrind", "policy"])
def test_real_enter_enable_approach_play_pause_and_stop(monkeypatch, tmp_path, source):
    policy_mode = source == "policy"
    if policy_mode:
        pytest.importorskip("torch")
    result, devices, startup, evidence = _exercise_source(
        monkeypatch, tmp_path, policy_mode=policy_mode, hand_mode="canonical" if policy_mode else source)
    assert result == 0
    assert set(devices) == {"arms", "right_hand"}
    assert all(device.closed and not device.enabled for device in devices.values())
    assert devices["arms"].modes == ([True] if policy_mode else [])
    for _, positions in devices["arms"].commands:
        np.testing.assert_allclose(positions[:7], startup[:7])
    assert devices["arms"].commands[-1][1][7] > startup[7]
    hand = devices["right_hand"]
    assert hand.commands[-1][1][0] < .03  # terminal target, not startup/zero/Home
    for t, q in hand.commands:
        if t < evidence.milestones["approach"]:
            np.testing.assert_allclose(q, .2)  # enable never authorizes approach
    milestones = evidence.milestones
    release, resume = milestones["release"], milestones["approach_resume"]
    brake = [(t, q) for t, q in hand.commands if release < t < resume]
    assert brake
    # Release during actual motion: decelerate, then hold without replanning.
    assert np.max(np.abs(brake[-1][1] - brake[0][1])) > 1e-5
    for (t0, q0), (t1, q1) in zip(brake, brake[1:]):
        assert np.max(np.abs(q1 - q0)) <= .1 * (t1 - t0) + 1e-9
    held = [q for t, q in brake if t > release + .6]
    assert len(held) >= 2
    for q in held[1:]:
        np.testing.assert_array_equal(q, held[0])
    assert any(row["phase"] == "BRAKING" for row in evidence.snapshots)
    assert not any(milestones["approach"] < t < resume for t in evidence.adapter_calls)

    milestones = evidence.milestones
    assert "ready" in milestones and "run" in milestones
    paused = [q for t, q in hand.commands if milestones["pause"] + .02 < t < milestones["resume"] - .02]
    assert paused
    for q in paused[1:]:
        np.testing.assert_array_equal(q, paused[0])
    assert not any(milestones["pause"] + .01 < t < milestones["resume"] for t in evidence.adapter_calls)
    assert not any(milestones["pause"] + .01 < t < milestones["resume"] for _, t in evidence.policy_calls)
    assert all(t >= milestones["run"] for _, t in evidence.policy_calls)

    from mocap_policy_runtime.data.geometry import compose_pose
    from mocap_policy_runtime.integration.h5_replay import motive_world_transform
    snapshots = evidence.snapshots
    waiting = [row for row in snapshots if row["t"] < .05]
    assert waiting and all(row["reference"] is None for row in waiting)
    assert all(not row["enabled"] for row in snapshots if row["phase"] in ("WAITING", "PLANNING", "ENABLE"))
    localized = [row for row in snapshots if row["reference"] is not None]
    preview = [row for row in localized if not row["enabled"]]
    assert preview
    np.testing.assert_allclose(preview[0]["reference"]["world_transform"], evidence.initial_wrist)
    np.testing.assert_allclose(evidence.calibrations[-1], evidence.initial_wrist)
    start_pose = np.array([.004, 0., 0., 0., 0., 0., 1.])
    if source in ("canonical", "retargeted"):
        start_pose[6] = -1.
    for row in localized:
        ref = row["reference"]
        np.testing.assert_allclose(compose_pose(row["robot_pose"], ref["world_transform"]),
                                   [0., 0., 0., 0., 0., 0., 1.], atol=1e-12)
        obj = ref["objects"]["hammer"]
        assert obj["status"] == "TRACKED"
        measured_object = np.array([.021 + row["t"] * .001, .01, 0., 0., 0., 0., 1.])
        np.testing.assert_allclose(compose_pose(row["robot_pose"], obj["actual"]), measured_object, atol=1e-12)
        assert obj["timestamp_ns"] == round((100. + row["t"]) * 1e9)
        if ref["state"] in ("PLANNING", "ENABLE", "APPROACH", "BRAKING", "HOLD", "READY"):
            assert ref["time_s"] == 0. and ref["index"] == 0
            if row["enabled"]:
                np.testing.assert_allclose(ref["world_transform"], evidence.initial_wrist)
            np.testing.assert_allclose(ref["hand_joints"], .005)
            np.testing.assert_allclose(ref["wrist_pose"], compose_pose(ref["world_transform"], start_pose))
    locked = [row for row in localized if row["phase"] in ("ENABLE", "HOLD", "APPROACH", "BRAKING", "READY")]
    assert locked
    for row in locked:
        np.testing.assert_array_equal(row["control"]["arms"], locked[0]["control"]["arms"])
        np.testing.assert_array_equal(row["control"]["right_hand"], locked[0]["control"]["right_hand"])
    first_run = next(q for t, q in devices["arms"].commands if t >= milestones["run"])
    assert first_run[7] >= startup[7] + .02 - 1e-9
    held_arrival = [row for row in localized if milestones["ready"] < row["t"] < milestones["run"]]
    assert held_arrival and all(row["reference"]["state"] == "READY" for row in held_arrival)
    running = [row for row in localized if row["reference"]["state"] == "RUNNING"]
    for row in running:
        ref, used = row["reference"], row["source"]
        assert ref["time_s"] == used.time_s and ref["index"] == used.index
        np.testing.assert_allclose(ref["hand_joints"], used.hand_joints["right"])
        np.testing.assert_allclose(ref["wrist_pose"], compose_pose(ref["world_transform"], used.wrist_poses["right"]))
    paused_refs = [row["reference"] for row in localized if row["reference"]["state"] == "PAUSED"]
    assert paused_refs and paused_refs[0]["time_s"] > 0.
    for ref in paused_refs[1:]:
        assert (ref["time_s"], ref["index"]) == (paused_refs[0]["time_s"], paused_refs[0]["index"])
        np.testing.assert_array_equal(ref["wrist_pose"], paused_refs[0]["wrist_pose"])
        np.testing.assert_array_equal(ref["hand_joints"], paused_refs[0]["hand_joints"])
        np.testing.assert_array_equal(ref["objects"]["hammer"]["target"], paused_refs[0]["objects"]["hammer"]["target"])
    assert not np.array_equal(paused_refs[0]["objects"]["hammer"]["actual"], paused_refs[-1]["objects"]["hammer"]["actual"])
    assert any(row["reference"]["time_s"] > paused_refs[0]["time_s"] for row in running if row["t"] >= milestones["resume"])
    assert localized[-1]["reference"]["state"] == "COMPLETE"
    assert not any(row["phase"] in ("HOME", "RETURN", "ARMED") for row in snapshots)


@pytest.mark.parametrize("reauthorize", [False, True])
def test_frame0_drift_revokes_ready_and_requires_new_authorization(monkeypatch, tmp_path, reauthorize):
    result, devices, _, evidence = _exercise_source(
        monkeypatch, tmp_path, localization_jump=True, reauthorize=reauthorize)
    assert result == 0
    marks = evidence.milestones
    assert "drift" in marks and "relocked" in marks
    assert not evidence.policy_calls
    assert all(row["reference"] is None or row["reference"]["time_s"] == 0.
               for row in evidence.snapshots)
    end = marks.get("reauthorize", float("inf"))
    held = [q for t, q in devices["arms"].commands if marks["relocked"] <= t < end]
    assert held
    for q in held[1:]:
        np.testing.assert_array_equal(q, held[0])
    if reauthorize:
        resumed = [q for t, q in devices["arms"].commands if t > end + .5]
        assert resumed and np.max(np.abs(resumed[-1] - held[0])) > .001
    else:
        assert all(row["phase"] not in ("APPROACH", "READY", "RUNNING")
                   for row in evidence.snapshots if row["t"] > marks["relocked"])


def test_localization_accepts_travel_and_admitted_skew_but_rejects_persistent_frame_jump():
    monitor = real.StableLocalization(.004, 2., .1, .03, 100_000_000, ())
    identity = np.array([0., 0., 0., 0., 0., 0., 1.])
    for index in range(101):
        now = index * .005
        fk = identity.copy()
        fk[0] = now * .2
        delayed = identity.copy()
        delayed[0] = max(0., now - .08) * .2
        assert monitor.consistent(fk, delayed, identity, now, .1)
    jumped = fk.copy()
    jumped[1] += .03
    assert monitor.consistent(fk, jumped, identity, .505, .1)
    assert not monitor.consistent(fk, jumped, identity, .61, .1)


def test_unselected_arm_holds_verified_enable_seed(monkeypatch, tmp_path):
    result, devices, initial, evidence = _exercise_source(monkeypatch, tmp_path, enable_shift=True)
    assert result == 0
    assert devices["arms"].commands
    expected = initial[:7].copy()
    expected[0] += .02
    for _, command in devices["arms"].commands:
        np.testing.assert_array_equal(command[:7], expected)
    assert any(row["phase"] == "RUNNING" for row in evidence.snapshots)


def test_real_initially_held_enter_and_repeat_bytes_never_enable(monkeypatch, tmp_path):
    result, devices, _, evidence = _exercise_source(monkeypatch, tmp_path, never_enable=True)
    assert result == 0
    assert all(not device.commands and not device.modes for device in devices.values())
    assert not any(row["enabled"] for row in evidence.snapshots)


@pytest.mark.parametrize("phase", ["enable", "approach"])
def test_enter_in_another_window_cannot_authorize_motion(monkeypatch, tmp_path, phase):
    result, devices, initial_arms, evidence = _exercise_source(
        monkeypatch, tmp_path, foreign_enter=phase)
    assert result == 0
    if phase == "enable":
        assert not any(row["enabled"] for row in evidence.snapshots)
        assert all(not device.commands for device in devices.values())
    else:
        assert any(row["enabled"] for row in evidence.snapshots)
        for _, command in devices["arms"].commands:
            np.testing.assert_array_equal(command, initial_arms)
        for _, command in devices["right_hand"].commands:
            np.testing.assert_array_equal(command, np.full(20, .2))
    assert all(device.closed and not device.enabled for device in devices.values())


@pytest.mark.parametrize("phase", ["WAITING", "PLANNING", "ENABLE", "HOLD", "APPROACH", "BRAKING", "READY", "RUNNING", "PAUSED"])
def test_real_quit_stops_directly_in_every_phase(monkeypatch, tmp_path, phase):
    result, devices, _, evidence = _exercise_source(monkeypatch, tmp_path, quit_phase=phase)
    assert result == 0
    assert "quit" in evidence.milestones
    assert all(device.closed and not device.enabled for device in devices.values())
    assert all(t < evidence.milestones["quit"] for device in devices.values() for t, _ in device.commands)


@pytest.mark.parametrize("channel", ["wrist", "hand"])
def test_real_requires_actual_frame0_not_only_native_convergence(monkeypatch, tmp_path, channel):
    result, devices, _, evidence = _exercise_source(monkeypatch, tmp_path, miss_frame0=channel)
    assert result == 0  # duration cleanup, never starts playback
    assert "ready" not in evidence.milestones
    assert all(row["reference"] is None or row["reference"]["time_s"] == 0. for row in evidence.snapshots)
    assert all(device.closed and not device.enabled for device in devices.values())


def test_policy_hammer_start_requires_matching_pose_and_new_press(monkeypatch, tmp_path):
    pytest.importorskip("torch")
    result, devices, _, evidence = _exercise_source(monkeypatch, tmp_path, policy_mode=True, hammer_mismatch=True)
    assert result == 0
    assert evidence.policy_calls
    assert min(t for _, t in evidence.policy_calls) >= evidence.milestones["run"]
    assert all(device.closed and not device.enabled for device in devices.values())


@pytest.mark.parametrize("policy_mode", [False, True])
def test_real_lost_motive_stops_without_finishing_replay(monkeypatch, tmp_path, policy_mode):
    if policy_mode:
        pytest.importorskip("torch")
    result, devices, _, evidence = _exercise_source(monkeypatch, tmp_path, lose_motive=True, policy_mode=policy_mode)
    assert result == 1
    hand = devices["right_hand"]
    assert .005 < hand.commands[-1][1][0] < .025
    assert hand.commands[-1][0] < evidence.milestones["pause"] + .05
    assert all(device.closed and not device.enabled for device in devices.values())
    assert devices["arms"].modes == ([True] if policy_mode else [])


def test_real_h5_object_loss_hides_actual_without_freezing_target_or_stopping(monkeypatch, tmp_path):
    result, devices, _, evidence = _exercise_source(monkeypatch, tmp_path, lose_object=True)
    assert result == 0
    running = [row["reference"] for row in evidence.snapshots
               if row["reference"] is not None and row["reference"]["state"] == "RUNNING"]
    assert any(ref["objects"]["hammer"]["status"] == "TRACKED" for ref in running)
    lost = [ref for ref in running if ref["objects"]["hammer"]["status"] == "UNTRACKED"]
    assert lost
    assert all(ref["objects"]["hammer"]["actual"] is None for ref in lost)
    assert all(ref["objects"]["hammer"]["target"] is not None for ref in lost)
    assert lost[-1]["time_s"] > lost[0]["time_s"]
    assert all(device.closed and not device.enabled for device in devices.values())


def test_policy_object_loss_stops_without_home(monkeypatch, tmp_path):
    pytest.importorskip("torch")
    result, devices, _, evidence = _exercise_source(monkeypatch, tmp_path, policy_mode=True, lose_object=True)
    assert result == 1
    assert all(device.closed and not device.enabled for device in devices.values())
    assert all(t < evidence.milestones["pause"] + .05 for device in devices.values() for t, _ in device.commands)
    assert devices["arms"].modes == [True]


@pytest.mark.parametrize("quit_phase", [None, "RUNNING"])
def test_legacy_replay_completion_and_quit_stop_without_home(monkeypatch, tmp_path, quit_phase):
    result, devices, startup, evidence = _exercise_source(
        monkeypatch, tmp_path, legacy_mode=True, quit_phase=quit_phase)
    assert result == 0
    assert all(device.closed and not device.enabled for device in devices.values())
    assert devices["arms"].commands[-1][1][7] > startup[7]
    if quit_phase is None:
        assert evidence.snapshots[-1]["phase"] == "COMPLETE"
    else:
        assert all(t < evidence.milestones["quit"] for device in devices.values() for t, _ in device.commands)
    assert not any(row["phase"] in ("HOME", "RETURN") for row in evidence.snapshots)


def test_wayland_initial_held_enter_uses_kernel_key_state(monkeypatch):
    from mocap_policy_runtime.integration import real_keyboard
    monkeypatch.setenv("XDG_SESSION_TYPE", "wayland")
    monkeypatch.setattr(real_keyboard.glob, "glob", lambda pattern: ["offline-keyboard"])
    monkeypatch.setattr(real_keyboard.os, "open", lambda *a: 42)
    monkeypatch.setattr(real_keyboard.os, "close", lambda fd: None)
    def ioctl(fd, request, keymap):
        keymap[28 >> 3] |= 1 << (28 & 7)
    monkeypatch.setattr(real_keyboard.fcntl, "ioctl", ioctl)
    events = []
    def read_events(fd, size):
        if events:
            return events.pop(0)
        raise BlockingIOError
    monkeypatch.setattr(real_keyboard.os, "read", read_events)
    keyboard = real_keyboard.X11KeyState()
    try:
        assert keyboard.is_pressed()  # no press event was available at startup
        events.append(keyboard._EVENT.pack(0, 0, 1, 28, 2))
        assert keyboard.is_pressed()  # autorepeat is still the same held press
        events.append(keyboard._EVENT.pack(0, 0, 1, 28, 0))
        assert not keyboard.is_pressed()
        events.append(keyboard._EVENT.pack(0, 0, 1, 28, 1))
        assert keyboard.is_pressed()
    finally:
        keyboard.close()


def test_right_impedance_verifies_echo_and_never_switches_left(monkeypatch):
    from real_robot.tests.test_hardware import Clock, FakeMarvin
    class ImpedanceSDK(FakeMarvin):
        def __init__(self):
            super().__init__()
            self.echo = dict(imp_type=0, joint_k=[0.] * 7, joint_d=[0.] * 7,
                             tool_kine=[0.] * 6, tool_dyn=[0.] * 10)
        def subscribe(self, unused):
            payload = super().subscribe(unused)
            payload["inputs"] = [dict(item) for item in payload["inputs"]]
            payload["inputs"][1].update(self.echo)
            return payload
        def set_tool(self, arm, kine, dyn):
            assert arm == "B"
            self.echo.update(tool_kine=list(kine), tool_dyn=list(dyn))
            return 1
        def set_joint_kd_params(self, arm, k, d):
            assert arm == "B"
            self.echo.update(joint_k=list(k), joint_d=list(d))
            return 1
        def set_impedance_type(self, arm, kind):
            assert arm == "B"
            self.echo["imp_type"] = kind
            return 1
        def set_PD_vel_est_step(self, arm, step):
            assert arm == "B" and step == 5
            return 1
    clock, sdk = Clock(), ImpedanceSDK()
    monkeypatch.setattr(hardware.time, "monotonic_ns", clock.monotonic_ns)
    monkeypatch.setattr(hardware.time, "sleep", clock.sleep)
    monkeypatch.setattr(hardware, "_load_marvin", lambda path: (sdk, object()))
    device = hardware.MarvinDevice(Path("unused"), "127.0.0.1")
    settings = real._policy_settings(real.ROOT / "configs/regrind.yaml")
    try:
        device.connect()
        device.enable()
        real._set_regrind_mode(device, True, None, settings)
        assert sdk.state == [1, 3]
        assert device.read_feedback().healthy
        device.send([0.] * 14)
        sdk.echo["joint_k"][0] += 1
        assert not device.read_feedback().healthy
        with pytest.raises(RuntimeError):
            device.send([0.] * 14)
        sdk.echo["joint_k"][0] -= 1
        real._set_regrind_mode(device, False, None, settings)
        assert sdk.state == [1, 1]
        assert device.read_feedback().healthy
    finally:
        device.close()
