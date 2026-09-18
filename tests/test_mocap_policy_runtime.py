"""Contracts where silent replay corruption could change a robot target."""
from pathlib import Path

import h5py
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from mocap_policy_runtime.data import load_trajectory
from mocap_policy_runtime.data.reference import load_reference
from mocap_policy_runtime.replay.clock import HoldToRunClock


def reference_file(path: Path):
    with h5py.File(path, "w") as stream:
        stream["regrind_retargeting_root_pos"] = [[0., 0., 0.], [0.02, 0., 0.]]
        stream["regrind_retargeting_root_quat"] = [[1., 0., 0., 0.], [0., 0., 0., 1.]]
        stream["regrind_retargeting_joints"] = np.zeros((2, 20))
        stream["object_pos"] = np.zeros((2, 3))
        stream["object_quat"] = [[1., 0., 0., 0.], [1., 0., 0., 0.]]
    return path


def test_reference_sampling_converts_quaternion_order_and_stops(tmp_path):
    trajectory = load_trajectory(reference_file(tmp_path / "reference.h5"), rate_hz=50)
    middle = trajectory.sample(0.01)
    np.testing.assert_allclose(middle.wrist_poses["right"][:3], [0.01, 0, 0])
    actual = Rotation.from_quat(middle.wrist_poses["right"][3:]).apply([1, 0, 0])
    np.testing.assert_allclose(actual, [0, 1, 0], atol=1e-12)
    assert not middle.complete
    endpoint = trajectory.sample(0.04)
    assert endpoint.complete
    np.testing.assert_allclose(endpoint.wrist_poses["right"][:3], [0.02, 0, 0])


def test_reference_external_links_are_rejected_without_following(tmp_path):
    path = reference_file(tmp_path / "reference.h5")
    with h5py.File(path, "a") as stream:
        del stream["object_pos"]
        stream["object_pos"] = h5py.ExternalLink("does-not-exist.h5", "/secret")
    with pytest.raises(ValueError):
        load_reference(path)


def test_reference_zero_quaternion_is_not_silently_repaired(tmp_path):
    path = reference_file(tmp_path / "reference.h5")
    with h5py.File(path, "a") as stream:
        stream["object_quat"][1] = 0
    with pytest.raises(ValueError, match="quaternion"):
        load_reference(path)


def test_pause_does_not_catch_up_and_stall_is_bounded():
    clock = HoldToRunClock(maximum_step_s=0.02)
    assert clock.update(0.0, True) == 0.0
    assert clock.update(0.01, False) == pytest.approx(0.01)
    assert clock.update(5.0, True) == pytest.approx(0.01)
    assert clock.update(5.01, True) == pytest.approx(0.02)
    assert clock.update(10.0, True) == pytest.approx(0.04)
    with pytest.raises(ValueError):
        clock.update(9.0, True)


def test_invalid_acquisition_gap_is_not_interpolated_as_tracking(tmp_path):
    path = tmp_path / "capture.h5"
    with h5py.File(path, "w") as stream:
        stream.attrs["h5_version"] = "4.0"
        stream.attrs["schema_name"] = "mocap-acquisition"
        stream.attrs["schema_layout"] = "compact-aligned-60hz-v1"
        stream.attrs["output_hz"] = 60.0
        stream["time_ns"] = np.array([0, 16666667, 33333333], dtype=np.int64)
        for side in ("left", "right"):
            hand = stream.create_group(f"hands/{side}")
            hand["valid"] = np.array([1, 0, 1], dtype=np.uint8)
            hand["wrist_position"] = np.zeros((3, 3), dtype=np.float32)
            hand["wrist_quaternion_xyzw"] = np.tile([0., 0., 0., 1.], (3, 1)).astype(np.float32)
            hand["keypoints_world"] = np.zeros((3, 21, 3), dtype=np.float32)
    trajectory = load_trajectory(path)
    assert "right" in trajectory.sample(0).wrist_poses
    assert "right" not in trajectory.sample(0.01).wrist_poses
    assert "right" not in trajectory.sample(0.02).wrist_poses
    assert "right" in trajectory.sample(1).wrist_poses


def test_regrind_residual_rotation_is_world_left_multiplied():
    pytest.importorskip("torch")
    from mocap_policy_runtime.policies.regrind.actor import action_to_targets
    reference = Rotation.from_euler("x", 0.7)
    action = np.zeros(26)
    action[0], action[5], action[6] = 2., 1., -2.
    position, orientation, joints = action_to_targets(
        action, np.array([1., 2., 3.]), np.roll(reference.as_quat(), 1), np.zeros(20))
    np.testing.assert_allclose(position, [1.02, 2., 3.])
    np.testing.assert_allclose(joints[0], -0.064)
    expected = Rotation.from_rotvec([0, 0, 0.064]) * reference
    np.testing.assert_allclose(Rotation.from_quat(np.roll(orientation, -1)).as_matrix(), expected.as_matrix())


def test_real_measured_seed_admits_encoder_jitter_but_not_unsafe_pose():
    from mocap_policy_runtime.integration.real import admit_measured_positions
    from real_robot.safety import SafetyFault
    lower, upper = np.full(54, -1.), np.full(54, 1.)
    measured = np.zeros(54)
    measured[0], measured[7] = -1.00005, 1.00005
    bounded = admit_measured_positions(measured, lower, upper)
    np.testing.assert_allclose(bounded[[0, 7]], [-1., 1.])
    measured[0] = -1.02
    with pytest.raises(SafetyFault):
        admit_measured_positions(measured, lower, upper)


def test_live_sample_arriving_after_loop_timestamp_is_fresh(monkeypatch):
    from types import SimpleNamespace
    from mocap_policy_runtime.integration import real
    pose = np.array([0., 0., 0., 0., 0., 0., 1.])
    sample = SimpleNamespace(received_at=10.001, frame_number=1,
                             wrist_xyzw=pose, hammer_xyzw=pose)
    tracker = SimpleNamespace(error=None, latest=lambda: sample)
    settings = dict(motive_stale_s=.12, arm_stale_s=.15, hand_stale_s=.04,
                    maximum_input_skew_s=.02)
    feedback = {name: SimpleNamespace(received_monotonic_ns=10_000_000_000,
                                      position_rad=np.zeros(20))
                for name in ("arms", "right_hand")}
    inputs = real.PolicyInputs(tracker, settings)
    monkeypatch.setattr(real.time, "monotonic_ns", lambda: 10_002_000_000)
    received, _ = inputs.read(feedback, 10_000_000_000)
    assert received.frame_number == 1
    monkeypatch.setattr(real.time, "monotonic_ns", lambda: 11_000_000_000)
    with pytest.raises(real.SafetyFault):
        inputs.read(feedback, 10_000_000_000)


def test_real_mode_without_confirmation_cannot_construct_hardware(monkeypatch):
    from mocap_policy_runtime.integration import real

    def forbidden(*args, **kwargs):
        pytest.fail("hardware was constructed before explicit admission")

    monkeypatch.setattr(real, "make_hardware", forbidden)
    assert real.main(["--trajectory", "not-even-opened.h5"]) == 2


def test_cartesian_conditioning_retains_speed_and_acceleration_bounds():
    from mocap_policy_runtime.integration.conditioning import TargetConditioner, TargetConditioningSettings
    settings = TargetConditioningSettings(
        rate_hz=200, translation_gain=np.ones(3), rotation_gain=1.,
        workspace_relative_radii_m=np.array([.42, .38, .38]), workspace_soft_zone_ratio=.9,
        maximum_linear_speed_m_s=.09, maximum_angular_speed_rad_s=.3875,
        maximum_linear_acceleration_m_s2=.875, maximum_angular_acceleration_rad_s2=2.25)
    limiter = TargetConditioner(np.zeros(3), [0, 0, 0, 1], settings)
    previous_position = np.zeros(3)
    previous_rotation = Rotation.identity()
    previous_velocity = np.zeros(3)
    previous_angular_velocity = np.zeros(3)
    for _ in range(40):
        position, quaternion, _ = limiter.condition([1, 0, 0], Rotation.from_rotvec([0, 0, 1]).as_quat())
        rotation = Rotation.from_quat(quaternion)
        velocity = (position - previous_position) * 200
        angular_velocity = (previous_rotation.inv() * rotation).as_rotvec() * 200
        assert np.linalg.norm(velocity) <= .09 + 1e-12
        assert np.linalg.norm(angular_velocity) <= .3875 + 1e-12
        assert np.linalg.norm(velocity - previous_velocity) * 200 <= .875 + 1e-10
        assert np.linalg.norm(angular_velocity - previous_angular_velocity) * 200 <= 2.25 + 1e-10
        previous_position, previous_rotation = position, rotation
        previous_velocity, previous_angular_velocity = velocity, angular_velocity


def test_real_replay_checks_future_joint_rows_before_execution(tmp_path):
    from mocap_policy_runtime.integration.real import _validate_replay_joint_limits
    from real_robot.safety import SafetyFault
    path = reference_file(tmp_path / "reference.h5")
    with h5py.File(path, "a") as stream:
        stream["regrind_retargeting_joints"][1, 0] = .3
    trajectory = load_trajectory(path)
    safety = {"right_hand": {"lower_rad": [-.2] * 20, "upper_rad": [.2] * 20}}
    with pytest.raises(SafetyFault):
        _validate_replay_joint_limits(trajectory, safety)
