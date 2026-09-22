"""Safety boundaries for independent, source-timestamped command streams."""

from types import SimpleNamespace

import pytest

from manus_bridge.hand2_core import HandStage


class Clock:
    now = 1_000_000_000

    def __call__(self):
        return self.now


def frame(clock, *, side="left", sequence=1, session="one", glove=10,
          stamp=None, boot="local", valid=True):
    return SimpleNamespace(
        side=side, sequence=sequence, session_id=session, glove_id=glove,
        source_monotonic_ns=clock() if stamp is None else stamp,
        boot_id=boot, valid=valid,
    )


def test_right_traffic_cannot_refresh_silent_left_timestamp():
    clock = Clock()
    left = HandStage("left", "local", lambda value: value, clock=clock)
    right = HandStage("right", "local", lambda value: value, clock=clock)
    original = left.process(frame(clock), [1])
    clock.now += 200_000_000
    assert right.process(frame(clock, side="right"), [2]).valid
    clock.now += 60_000_000
    expired = left.poll()
    assert not expired.valid
    assert expired.values is None
    assert expired.source == original.source
    assert left.poll() is None
    assert right.poll() is None


def test_duplicate_or_regressed_sample_never_extends_source_lifetime():
    clock = Clock()
    stage = HandStage("left", "local", lambda value: value, clock=clock)
    original = stage.process(frame(clock, sequence=8), 1)
    clock.now += 100_000_000
    assert stage.process(frame(clock, sequence=8), 2) is None
    assert stage.process(frame(clock, sequence=7), 3) is None
    assert stage.process(frame(clock, sequence=9, stamp=original.source.source_monotonic_ns), 4) is None
    clock.now += 151_000_000
    expired = stage.poll()
    assert not expired.valid
    assert expired.source == original.source


class HistorySolver:
    def __init__(self):
        self.history = 0

    def step(self, value):
        self.history += value
        return self.history

    def reset(self):
        self.history = 0


@pytest.mark.parametrize("new_identity", [{"session": "two"}, {"glove": 20}])
def test_identity_change_clears_solver_history_and_retires_old_source(new_identity):
    clock = Clock()
    solver = HistorySolver()
    stage = HandStage("left", "local", solver.step, reset=solver.reset, clock=clock)
    assert stage.process(frame(clock), 3).values == 3
    clock.now += 1
    assert stage.process(frame(clock, sequence=2), 4).values == 7
    clock.now += 1
    changed = stage.process(frame(clock, **new_identity), 5)
    assert changed.valid and changed.values == 5
    clock.now += 1
    retired = stage.process(frame(clock, sequence=3), 100)
    assert not retired.valid
    assert retired.values is None
    assert retired.source == changed.source


@pytest.mark.parametrize("changes", [
    {"boot": "remote"}, {"stamp": 1_000_000_001},
    {"stamp": 749_999_999}, {"session": ""}, {"side": "right"},
    {"sequence": 0}, {"glove": 0},
])
def test_untrusted_source_never_reaches_transform(changes):
    clock = Clock()

    def forbidden(_):
        pytest.fail("invalid source reached transform")

    result = HandStage("left", "local", forbidden, clock=clock).process(frame(clock, **changes), None)
    assert not result.valid
    assert result.values is None


def test_solver_work_that_crosses_age_limit_cannot_publish_valid_target():
    clock = Clock()

    def slow_step(value):
        clock.now += 250_000_001
        return value

    message = frame(clock)
    stage = HandStage("left", "local", slow_step, clock=clock)
    result = stage.process(message, [1])
    assert not result.valid
    assert result.source.source_monotonic_ns == message.source_monotonic_ns
    assert result.values is None
    assert stage.poll() is None


def test_ambiguity_invalidates_active_target_and_cannot_switch_same_identity_writer():
    clock = Clock()
    solver = HistorySolver()
    stage = HandStage("left", "local", solver.step, reset=solver.reset, clock=clock)
    original = stage.process(frame(clock), 5, writer=b"first")
    invalid = stage.poll(publisher_count=2)
    assert not invalid.valid and invalid.source == original.source
    clock.now += 1
    assert stage.process(frame(clock, sequence=2), 7, writer=b"second") is None
    restored = stage.process(frame(clock, sequence=2), 7, writer=b"first")
    assert restored.valid and restored.values == 7


def test_explicit_invalidation_with_same_timestamp_does_not_get_deduplicated():
    clock = Clock()
    stage = HandStage("left", "local", lambda value: value, clock=clock)
    original = stage.process(frame(clock), 1)
    invalid = stage.process(frame(clock, valid=False), None)
    assert not invalid.valid
    assert invalid.source == original.source
    assert stage.process(frame(clock), 2) is None


def test_undiscovered_publisher_cannot_initialize_source_or_run_solver():
    clock = Clock()
    solver = HistorySolver()
    stage = HandStage("left", "local", solver.step, reset=solver.reset, clock=clock)
    rejected = stage.process(frame(clock), 100, publisher_count=0)
    assert not rejected.valid
    assert rejected.values is None
    accepted = stage.process(frame(clock), 3, publisher_count=1, writer=b"discovered")
    assert accepted.valid and accepted.values == 3
