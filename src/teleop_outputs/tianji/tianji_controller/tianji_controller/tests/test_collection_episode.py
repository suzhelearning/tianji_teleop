"""Multi-episode behavior with the real motion gate and controlled DDS replies."""
from concurrent.futures import Future
from types import SimpleNamespace

import pytest

from tianji_controller.collection_episode import CollectionEpisodes
from tianji_controller.safety import SafetyFault
from .test_staged_motion import NOW, TICK, HOME_LEFT, HOME_RIGHT, frame, feedback, gate, held


class Observer:
    session_id = "episode-test"

    def __init__(self):
        self.requests = []
        self.message = SimpleNamespace(
            state="IDLE", session_id=self.session_id, published_monotonic_ns=NOW,
            error="", prepared=True, inputs_ready=True, active_path="", last_saved_path="")

    def status(self):
        return self.message

    def update_state(self, phase):
        self.phase = phase

    def submit_recording(self, command):
        future = Future()
        self.requests.append((command, future))
        return future

    def acknowledge(self, accepted=True):
        self.requests[-1][1].set_result(SimpleNamespace(
            accepted=accepted, state=self.message.state, message="accepted" if accepted else "rejected"))


class Run:
    def __init__(self):
        self.motion = gate(maximum_speed_rad_s=1.0, maximum_acceleration_rad_s2=2.0)
        self.observer = Observer()
        self.episodes = CollectionEpisodes(self.motion, self.observer, notify=lambda _: None)
        self.now = NOW
        self.positions = held()
        self.positions["arms"] = HOME_LEFT + HOME_RIGHT
        self.motion.arm_home(frame(), feedback(values=self.positions), self.now)
        self.episodes.enabled()
        self.until("HOME_READY")

    def tick(self, target=0.0):
        self.now += TICK
        packet = frame(value=target, stamp=self.now)
        measured = feedback(enabled=True, stamp=self.now, values=self.positions)
        self.observer.message.published_monotonic_ns = self.now
        self.episodes.tick(packet, measured, self.now)
        self.positions = self.motion.step(packet, measured, self.now)
        return self.positions

    def until(self, state, target=0.0):
        for _ in range(6000):
            self.tick(target)
            if self.episodes.state == state:
                return
        raise AssertionError(f"never reached {state}: {self.episodes.state}/{self.motion.phase}")

    def start(self, index):
        self.episodes.on_key("r")
        self.until("STARTING")
        assert self.observer.requests[-1][0] == "start"
        self.observer.acknowledge()
        self.observer.message.state = "RECORDING"
        self.observer.message.active_path = f"episode-{index}.partial.h5"
        self.until("RECORDING")

    def saved(self, index):
        self.observer.acknowledge()
        self.observer.message.state = "IDLE"
        self.observer.message.active_path = ""
        self.observer.message.last_saved_path = f"episode-{index}.h5"


def test_three_episodes_home_without_rearming_and_explicit_final_exit():
    run = Run()
    assert run.motion.armed
    for index in range(3):
        # Home hold must not chase the moving operator between samples.
        home = dict(run.positions)
        for _ in range(10):
            assert run.tick(target=.7) == home
        run.start(index)
        for _ in range(10):
            run.tick(target=.2)
        run.episodes.on_key("s")
        run.tick(target=.8)
        assert run.episodes.state == "ENDING"
        assert run.observer.requests[-1][0] == "save"
        # Save completion alone cannot skip braking and actual Home settling.
        run.saved(index)
        run.until("HOME_READY", target=.8)
        assert run.motion.armed and not run.episodes.done
        assert run.positions["arms"] == HOME_LEFT + HOME_RIGHT
    assert [op for op, _ in run.observer.requests] == ["start", "save"] * 3
    run.episodes.on_key("q")
    run.tick()
    assert run.episodes.done and run.motion.phase == "HOME_REACHED"
    # Only the executor's outer cleanup disables devices after this signal.
    assert run.motion.armed


def test_start_acknowledgement_does_not_release_before_recording_status():
    run = Run()
    run.episodes.on_key("r")
    run.until("STARTING")
    alignment = dict(run.positions)
    run.observer.acknowledge()
    for _ in range(100):
        assert run.tick(target=.8) == alignment
        assert run.episodes.state == "STARTING"
    run.observer.message.state = "RECORDING"
    run.observer.message.active_path = "new.partial.h5"
    run.until("RECORDING", target=.8)
    assert run.tick(target=.8)["arms"] != alignment["arms"]


def test_rejected_save_never_starts_home_or_next_episode():
    run = Run()
    run.start(1)
    run.episodes.on_key("s")
    run.tick()
    run.observer.acknowledge(accepted=False)
    with pytest.raises(SafetyFault, match="rejected"):
        run.tick()
    assert run.motion.phase == "TELEOP"
    assert run.episodes.state == "ENDING"


def test_busy_keys_are_not_replayed_and_stale_idle_is_not_save_confirmation():
    run = Run()
    run.start(1)
    run.episodes.on_key("s")
    run.tick()
    run.episodes.on_key("r")
    run.observer.acknowledge()
    run.observer.message.state = "IDLE"
    run.observer.message.active_path = ""
    with pytest.raises(SafetyFault, match="no new saved episode"):
        run.tick()
    assert [op for op, _ in run.observer.requests] == ["start", "save"]


def test_faulted_feedback_during_home_cannot_reach_next_episode():
    run = Run()
    run.start(1)
    run.episodes.on_key("s")
    run.tick()
    run.saved(1)
    run.until("HOMING")
    run.now += TICK
    measured = feedback(enabled=True, stamp=run.now, values=run.positions)
    measured["arms"].healthy = False
    with pytest.raises(SafetyFault):
        run.motion.step(frame(stamp=run.now), measured, run.now)
    assert run.motion.fault and run.episodes.state == "HOMING"


def test_finish_during_recording_saves_and_waits_for_home_before_exit():
    run = Run()
    run.start(1)
    for _ in range(10):
        run.tick(target=.2)
    run.episodes.on_key("q")
    run.tick()
    assert run.episodes.state == "ENDING" and not run.episodes.done
    assert run.observer.requests[-1][0] == "save"
    run.saved(1)
    run.until("HOMING")
    assert not run.episodes.done
    run.until("DONE")
    assert run.positions["arms"] == HOME_LEFT + HOME_RIGHT
    assert run.motion.armed and run.episodes.done
