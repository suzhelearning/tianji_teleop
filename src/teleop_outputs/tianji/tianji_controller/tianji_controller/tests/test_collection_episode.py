"""Exercise guarded episode transitions without hardware or a ROS executor."""
from concurrent.futures import Future
from dataclasses import replace
from types import SimpleNamespace

import pytest

from tianji_controller.collection_episode import CollectionEpisodes
from tianji_controller.safety import SafetyFault
from .test_staged_motion import NOW, TICK, HOME_LEFT, HOME_RIGHT, frame, feedback, gate, held

HOME = HOME_LEFT + HOME_RIGHT


class Observer:
    session_id = "episode-test"

    def __init__(self, run):
        self.run = run
        self.requests = []
        self.episode_id = ""
        self.number = -1
        self.phase = ""
        self.message = SimpleNamespace(
            state="IDLE", session_id=self.session_id, published_monotonic_ns=NOW,
            error="", prepared=True, inputs_ready=True, active_path="", last_saved_path="",
            episode_id="", last_episode_id="")

    def status(self):
        return self.message

    def update_state(self, phase, **kwargs):
        self.phase = phase

    def submit_recording(self, command, *, cutoff_monotonic_ns=0):
        if command == "start":
            # The collector accepts TELEOP only after the real gate has stopped.
            assert self.phase == "TELEOP" and self.run.motion.teleop_stopped
            self.number += 1
            self.episode_id = f"episode-{self.number}"
        future = Future()
        self.requests.append((command, future, cutoff_monotonic_ns))
        return future

    def acknowledge(self, accepted=True, state=None):
        command, future, _ = self.requests[-1]
        future.set_result(SimpleNamespace(
            success=accepted, state=state or ("RECORDING" if command == "start" and accepted else "IDLE"),
            session_id=self.session_id, episode_id=self.episode_id,
            saved_path=f"{self.episode_id}.h5" if command == "save" else "",
            message="completed" if accepted else "rejected"))

    def recording_status(self):
        self.message.state = "RECORDING"
        self.message.episode_id = self.episode_id
        self.message.active_path = f"{self.episode_id}.partial.h5"

    def completed_status(self):
        self.message.state = "IDLE"
        self.message.active_path = ""
        self.message.episode_id = ""
        self.message.last_episode_id = self.episode_id
        if self.requests[-1][0] == "save":
            self.message.last_saved_path = f"{self.episode_id}.h5"


class Run:
    def __init__(self):
        self.motion = gate(maximum_speed_rad_s=1.0, maximum_acceleration_rad_s2=2.0)
        self.now = NOW
        self.positions = held()
        self.positions["arms"] = HOME
        self.observer = Observer(self)
        self.episodes = CollectionEpisodes(self.motion, self.observer, notify=lambda _: None,
                                           clock=lambda: self.now)
        self.motion.arm_home(frame(), feedback(values=self.positions), self.now)
        self.episodes.enabled()
        self.until("HOME_READY")

    def tick(self, target=0.0, hand=0.0, *, status_stamp=None):
        self.now += TICK
        arms = tuple(q + target for q in HOME)
        packet = replace(frame(stamp=self.now), left_arm=arms[:7], right_arm=arms[7:],
                         left_hand=(hand,) * 20, right_hand=(hand,) * 20)
        measured = feedback(enabled=True, stamp=self.now, values=self.positions)
        if self.observer.message is not None:
            self.observer.message.published_monotonic_ns = self.now if status_stamp is None else status_stamp
        self.episodes.tick(packet, measured, self.now)
        self.positions = self.motion.step(packet, measured, self.now)
        return self.positions

    def until(self, state, target=0.0, hand=0.0):
        for _ in range(6000):
            self.tick(target, hand)
            if self.episodes.state == state:
                return
        raise AssertionError(f"never reached {state}: {self.episodes.state}/{self.motion.phase}")

    def prepare(self, target=0.0, hand=0.0):
        self.episodes.on_key("r")
        self.until("STARTING", target, hand)

    def start(self):
        self.prepare()
        self.observer.acknowledge()
        self.observer.recording_status()
        self.until("RECORDING")

    def completed(self):
        self.observer.acknowledge()
        self.observer.completed_status()


def test_frozen_approach_accepts_nonzero_hands_and_ignores_live_target_changes():
    run = Run()
    for _ in range(20):
        assert run.tick(target=.2, hand=.3) == {**held(), "arms": HOME}
    run.episodes.on_key("r")
    run.tick(target=.2, hand=.3)
    assert run.episodes.state == "ALIGNING"
    run.episodes.on_key("r")  # A busy key cannot replace the frozen pose.
    for _ in range(6000):
        before = dict(run.positions)
        run.tick(target=.5, hand=.8)
        assert not run.observer.requests
        for device, positions in run.positions.items():
            assert max(abs(q - previous) for q, previous in zip(positions, before[device])) <= .005 + 1e-12
        if run.episodes.state == "ALIGNED":
            break
    else:
        raise AssertionError("frozen approach never settled")
    frozen = {"arms": tuple(q + .2 for q in HOME),
              "left_hand": (.3,) * 20, "right_hand": (.3,) * 20}
    assert run.positions == frozen
    assert not run.motion.teleop_stopped  # Measured rest is rechecked after entering the hold.
    run.until("STARTING", target=.5, hand=.8)
    assert run.positions == frozen and run.motion.teleop_stopped


@pytest.mark.parametrize("status_first", [False, True])
def test_start_requires_recording_reply_and_matching_status_before_live_slew(status_first):
    run = Run()
    run.prepare(target=.2, hand=.3)
    frozen = dict(run.positions)
    if status_first:
        run.observer.recording_status()
    else:
        run.observer.acknowledge()
    for _ in range(40):
        assert run.tick(target=.5, hand=.8) == frozen
        assert run.episodes.state == "STARTING"
    if status_first:
        run.observer.acknowledge()
    else:
        run.observer.recording_status()
    run.until("RECORDING", target=.5, hand=.8)
    for device, positions in run.positions.items():
        deltas = [q - held_q for q, held_q in zip(positions, frozen[device])]
        assert all(0 < delta <= .005 + 1e-12 for delta in deltas)
    assert [request[0] for request in run.observer.requests] == ["start"]


def test_collector_not_ready_does_not_queue_alignment_for_later():
    run = Run()
    run.observer.message.inputs_ready = False
    run.episodes.on_key("r")
    run.tick(target=.4, hand=.3)
    run.observer.message.inputs_ready = True
    for _ in range(100):
        assert run.tick(target=.4, hand=.3)["arms"] == HOME
    assert run.episodes.state == "HOME_READY" and not run.observer.requests
    run.prepare(target=.4, hand=.3)
    assert run.positions["arms"] == tuple(q + .4 for q in HOME)


def test_known_idle_start_rejection_holds_until_explicit_retry_without_realigning():
    run = Run()
    run.prepare(target=.2, hand=.3)
    frozen = dict(run.positions)
    run.episodes.on_key("r")  # STARTING keys cannot retry a future rejection.
    run.observer.acknowledge(accepted=False)
    run.tick(target=.5, hand=.8)
    assert run.episodes.state == "ALIGNED"
    for _ in range(100):
        assert run.tick(target=.5, hand=.8) == frozen
    assert len(run.observer.requests) == 1 and run.motion.teleop_stopped
    run.episodes.on_key("r")
    assert run.tick(target=.5, hand=.8) == frozen
    assert run.episodes.state == "STARTING" and len(run.observer.requests) == 2
    run.observer.acknowledge()
    run.observer.recording_status()
    run.until("RECORDING", target=.5, hand=.8)
    assert run.positions["arms"] != frozen["arms"]


@pytest.mark.parametrize("outcome", ["uncertain", "exception", "timeout"])
def test_unknown_start_outcomes_never_release_hold_or_retry(outcome):
    run = Run()
    run.prepare()
    frozen = dict(run.positions)
    if outcome == "uncertain":
        run.observer.acknowledge(accepted=False, state="STARTING")
    elif outcome == "exception":
        run.observer.requests[-1][1].set_exception(RuntimeError("connection lost"))
    else:
        run.now += 301_000_000_000
    with pytest.raises(SafetyFault):
        run.tick(target=.4, hand=.3)
    assert run.positions == frozen and run.motion.teleop_stopped
    assert len(run.observer.requests) == 1


@pytest.mark.parametrize("key,operation", [("s", "save"), ("d", "discard")])
def test_stop_key_brakes_immediately_far_from_home_then_returns_and_repeats(key, operation):
    run = Run()
    for _ in range(2):
        run.start()
        for _ in range(120):
            run.tick(target=.8, hand=.3)
        assert max(abs(q - home) for q, home in zip(run.positions["arms"], HOME)) > .2
        key_ns = run.now
        hand_at_key = run.positions["left_hand"]
        run.episodes.on_key(key)
        run.tick(target=-.5, hand=.9)
        assert run.observer.requests[-1][0::2] == (operation, key_ns)
        assert run.episodes.state == "STOPPING"
        for _ in range(160):
            run.tick(target=-.5, hand=.9)
        assert run.motion.teleop_stopped
        stopped = dict(run.positions)
        assert stopped["left_hand"] == hand_at_key
        assert max(abs(q - home) for q, home in zip(stopped["arms"], HOME)) > .2
        for _ in range(20):
            assert run.tick(target=.8, hand=.9) == stopped
        assert run.episodes.state == "STOPPING"  # No Home before the save/discard receipt.
        run.completed()
        run.until("HOME_READY", target=.8, hand=.9)
        assert run.positions["arms"] == HOME
        assert run.positions["left_hand"] == (0.,) * 20
        assert run.positions["right_hand"] == (0.,) * 20
        assert run.motion.armed
    assert [request[0] for request in run.observer.requests] == ["start", operation] * 2
    run.episodes.on_key("q")
    run.tick()
    assert run.episodes.done and run.motion.phase == "HOME_REACHED"


def test_rejected_save_never_starts_home_or_next_episode():
    run = Run()
    run.start()
    run.episodes.on_key("s")
    run.tick()
    run.observer.acknowledge(accepted=False)
    with pytest.raises(SafetyFault, match="rejected"):
        run.tick()
    assert run.motion.phase == "TELEOP"


def test_wrong_saved_result_cannot_authorize_home():
    run = Run()
    run.start()
    run.episodes.on_key("s")
    run.tick()
    run.episodes.on_key("r")
    run.observer.acknowledge()
    run.observer.completed_status()
    run.observer.message.last_saved_path = ""
    with pytest.raises(SafetyFault, match="no new saved episode"):
        run.tick()
    assert [request[0] for request in run.observer.requests] == ["start", "save"]


def test_idle_status_without_stop_receipt_can_brake_but_not_start_home():
    run = Run()
    run.start()
    key_ns = run.now
    run.episodes.on_key("d")
    run.tick()
    assert run.observer.requests[-1][0::2] == ("discard", key_ns)
    run.observer.completed_status()
    run.until("STOPPING")
    for _ in range(100):
        run.tick()
    assert run.motion.phase == "TELEOP" and run.motion.teleop_stopped
    run.observer.acknowledge()
    run.until("HOME_READY")


def test_faulted_feedback_during_home_cannot_reach_next_episode():
    run = Run()
    run.start()
    run.episodes.on_key("s")
    run.tick()
    run.completed()
    run.until("HOMING")
    run.now += TICK
    measured = feedback(enabled=True, stamp=run.now, values=run.positions)
    measured["arms"].healthy = False
    with pytest.raises(SafetyFault):
        run.motion.step(frame(stamp=run.now), measured, run.now)
    assert run.motion.fault and run.episodes.state == "HOMING"


def test_finish_during_recording_saves_at_key_and_exits_only_after_prepared_home():
    run = Run()
    run.start()
    for _ in range(120):
        run.tick(target=.8, hand=.3)
    key_ns = run.now
    run.episodes.on_key("q")
    run.tick(target=-.5, hand=.9)
    assert run.observer.requests[-1][0::2] == ("save", key_ns)
    run.completed()
    run.until("DONE", target=.8, hand=.9)
    assert run.positions["arms"] == HOME and run.motion.armed


@pytest.mark.parametrize("invalid_status", ["wrong_episode", "stale"])
def test_completed_start_cannot_release_unmatched_recording_status(invalid_status):
    run = Run()
    run.prepare()
    frozen = dict(run.positions)
    run.observer.acknowledge()
    run.observer.recording_status()
    if invalid_status == "wrong_episode":
        run.observer.message.episode_id = "another-episode"
    for _ in range(10):
        assert run.tick(target=.4, status_stamp=NOW if invalid_status == "stale" else None) == frozen
    assert run.episodes.state == "STARTING" and run.motion.teleop_stopped


@pytest.mark.parametrize("dropout", ["status", "recording"])
def test_recording_dropout_cannot_trigger_automatic_home_or_next_episode(dropout):
    run = Run()
    run.start()
    if dropout == "status":
        run.observer.message = None
    else:
        run.observer.message.state = "IDLE"
    with pytest.raises(SafetyFault):
        run.tick()
    assert run.motion.phase == "TELEOP"
    assert [request[0] for request in run.observer.requests] == ["start"]
