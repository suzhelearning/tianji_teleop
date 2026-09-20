"""Migration boundaries: optional objects never weaken policy or wire contracts."""
import json
from types import SimpleNamespace

import numpy as np
import pytest

from mocap_policy_runtime.policies.regrind.tracking import (
    MOCAP_HANDS_FRAME,
    MOCAP_RIGID_BODY_NAMES,
    RegrindMotiveTracker,
)


class Session:
    def __init__(self):
        self.callbacks = {}

    def declare_subscriber(self, topic, callback):
        self.callbacks[topic] = callback
        return SimpleNamespace(undeclare=lambda: None)

    def send(self, topic, payload):
        self.callbacks[topic](SimpleNamespace(payload=json.dumps(payload).encode()))


def motive_frame(*, wrist_valid=True, hammer=None):
    bodies = [{"id": 1, "position": [1., 2., 3.],
               "quaternion_xyzw": [0., 0., 0., 1.],
               "mean_error": 0., "tracking_valid": wrist_valid}]
    if hammer is not None:
        bodies.append(hammer)
    return {"schema_version": 1, "frame_number": 7, "motive_timestamp": 0.,
            "publisher_received_time_ns": 1,
            "coordinate_system": "motive_x_forward_z_up_right_handed",
            "unit": "meter", "publisher_dropped_frames": 0,
            "markers": [], "rigid_bodies": bodies}


def test_wrist_only_replay_does_not_require_hammer_but_policy_does():
    replay_session, policy_session = Session(), Session()
    identity = np.array([0., 0., 0., 0., 0., 0., 1.])
    replay = RegrindMotiveTracker(replay_session, require_object=False, rigid_to_wrist=identity)
    policy = RegrindMotiveTracker(policy_session)
    for session in (replay_session, policy_session):
        session.send(MOCAP_RIGID_BODY_NAMES, {"names": {"1": "right_wrist"}})
        session.send(MOCAP_HANDS_FRAME, motive_frame())
    sample = replay.latest()
    assert replay.error is None
    np.testing.assert_array_equal(sample.wrist_xyzw, [1., 2., 3., 0., 0., 0., 1.])
    assert sample.hammer_xyzw is None
    assert policy.latest() is None
    assert policy.error is not None
    replay_session.send(MOCAP_HANDS_FRAME, motive_frame(wrist_valid=False))
    assert replay.latest() is None


def test_optional_hammer_can_be_untracked_but_cannot_be_malformed():
    session = Session()
    tracker = RegrindMotiveTracker(session, require_object=False)
    session.send(MOCAP_RIGID_BODY_NAMES, {"names": {"1": "right_wrist", "2": "hammer"}})
    hammer = {"id": 2, "position": [0., 0., 0.],
              "quaternion_xyzw": [0., 0., 0., 1.],
              "mean_error": 0., "tracking_valid": False}
    session.send(MOCAP_HANDS_FRAME, motive_frame(hammer=hammer))
    assert tracker.latest().hammer_xyzw is None
    assert tracker.error is None
    hammer["quaternion_xyzw"] = [0., 0., 0., 0.]
    session.send(MOCAP_HANDS_FRAME, motive_frame(hammer=hammer))
    assert tracker.error is not None
    session.send(MOCAP_HANDS_FRAME, motive_frame())
    assert tracker.error is not None  # A later valid frame cannot clear a wire fault.


def test_policy_rejects_wrist_only_sample_for_object_alignment():
    from mocap_policy_runtime.policies.regrind.live import hammer_alignment
    sample = SimpleNamespace(hammer_xyzw=None)
    with pytest.raises(ValueError):
        hammer_alignment(None, sample)


def test_actor_uses_checkpoint_variance_not_standard_deviation():
    torch = pytest.importorskip("torch")
    from mocap_policy_runtime.policies.regrind.actor import infer
    actor = torch.nn.Sequential(torch.nn.Linear(123, 26))
    with torch.no_grad():
        actor[0].weight.zero_()
        actor[0].bias.zero_()
        actor[0].weight[0, 0] = 1.
        actor[0].weight[1, 1] = 1.
    observation = np.zeros(123, dtype=np.float32)
    observation[:2] = [5., -7.]
    mean = torch.zeros(123)
    mean[:2] = torch.tensor([1., 1.])
    variance = torch.ones(123)
    variance[:2] = torch.tensor([4., 16.])
    action = infer(actor, mean, variance, observation)
    np.testing.assert_allclose(action[:2], [2., -2.], atol=1e-7)
    np.testing.assert_array_equal(action[2:], np.zeros(24))


def test_policy_history_uses_measured_joints_and_hold_preserves_reference(monkeypatch):
    torch = pytest.importorskip("torch")
    from mocap_policy_runtime.data.reference import RegrindReference
    from mocap_policy_runtime.policies.regrind import runtime
    actor = torch.nn.Sequential(torch.nn.Linear(123, 26))
    with torch.no_grad():
        actor[0].weight.zero_()
        actor[0].bias.zero_()
        actor[0].weight[6, 47] = 1.  # Current measured thumb minus training offset.
        actor[0].weight[7, 28] = 1.  # Previous measured second joint.
    monkeypatch.setattr(runtime, "load_actor", lambda *args, **kwargs:
                        (actor, torch.zeros(123), torch.ones(123), 0))
    reference = RegrindReference(np.zeros((3, 3)), np.tile([1., 0., 0., 0.], (3, 1)),
                                 np.zeros((3, 20)), np.array([[.2, .1, .3], [.4, .1, .3], [.6, .1, .3]]),
                                 np.array([[1., 0., 0., 0.], [np.sqrt(.5), 0., 0., np.sqrt(.5)],
                                           [0., 0., 0., 1.]]))
    policy = runtime.RegrindPolicy("unused.pt", reference, reference_speed=.5)
    wrist = np.array([0., 0., 0., 0., 0., 0., 1.])
    hammer = np.array([1., .5, .2, 0., 0., 0., 1.])
    measured = np.zeros(20)
    policy.reset(wrist, measured)
    measured[:2] = [.48, .2]
    first = policy.step(wrist, hammer, measured)
    measured[:2] = [.58, .4]
    hammer[0] += .1
    second = policy.step(wrist, hammer, measured)
    # Changing the measured object must not drag the DATA target while the
    # fractional reference cursor still selects the same reference frame.
    np.testing.assert_allclose(first.object_poses["hammer"], [.2, .1, .3, 0., 0., 0., 1.])
    np.testing.assert_array_equal(second.object_poses["hammer"], first.object_poses["hammer"])
    np.testing.assert_allclose(first.hand_joints["right"][:2], [.0128, 0.], atol=1e-8)
    np.testing.assert_allclose(second.hand_joints["right"][:2], [.0192, .0128], atol=1e-8)
    assert (first.index, second.index, policy.reference_progress) == (0, 0, 1.)
    measured[:2] = [.78, .6]
    policy.observe(wrist, measured, clear_action=True)
    assert policy.reference_progress == 1.
    measured[:2] = [.88, .8]
    third = policy.step(wrist, hammer, measured)
    fourth = policy.step(wrist, hammer, measured)
    np.testing.assert_allclose(third.object_poses["hammer"],
                               [.4, .1, .3, 0., 0., np.sqrt(.5), np.sqrt(.5)])
    np.testing.assert_allclose(third.hand_joints["right"][:2], [.0384, .0384], atol=1e-8)
    assert (third.index, fourth.index) == (1, 1)
    assert fourth.complete
    with pytest.raises(RuntimeError):
        policy.step(wrist, wrist, measured)
