"""Measured-state assembly rules: identity, freshness, tokens and hand pairing.

These are the rules that decide whether a byte reaches a dataset, so they are
tested directly and at the boundary they guard rather than through a fake robot.
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from data_collector.runtime import DeviceChannel  # noqa: E402
from data_collector.state import HandPairing, Staleness  # noqa: E402
from tianji_runtime import LockedDevice  # noqa: E402
from tianji_runtime.constants import ARMS_COUNT, HAND_COUNT, STATE_RATE_HZ  # noqa: E402

BOOT = "11111111-2222-3333-4444-555555555555"
NOW = 1_000_000_000_000


class Message:
    """Minimal stand-in for tianji_interfaces/DeviceFeedback."""

    def __init__(self, *, positions, sequence=1, source=NOW, boot_id=BOOT,
                 session_id="session-a", healthy=True, enabled=True, detail=""):
        self.boot_id = boot_id
        self.session_id = session_id
        self.sequence = sequence
        self.published_monotonic_ns = source
        self.source_monotonic_ns = source
        self.position_rad = list(positions)
        self.healthy = healthy
        self.enabled = enabled
        self.detail = detail


def arms_message(**kwargs):
    kwargs.setdefault("positions", [0.1] * ARMS_COUNT)
    return Message(**kwargs)


def hand_message(**kwargs):
    kwargs.setdefault("positions", [0.2] * HAND_COUNT)
    return Message(**kwargs)


@pytest.fixture()
def policy():
    return Staleness()


# --------------------------------------------------------------------- identity

def test_foreign_boot_id_is_refused():
    channel = DeviceChannel("arms", ARMS_COUNT, BOOT)
    reason = channel.accept(arms_message(boot_id="other-host"), NOW, Staleness())
    assert reason and "boot id" in reason
    assert channel.feedback is None


def test_missing_session_id_is_refused():
    channel = DeviceChannel("arms", ARMS_COUNT, BOOT)
    reason = channel.accept(arms_message(session_id=""), NOW, Staleness())
    assert reason and "session id" in reason


def test_session_change_is_refused_rather_than_extended():
    channel = DeviceChannel("arms", ARMS_COUNT, BOOT)
    assert channel.accept(arms_message(sequence=1), NOW, Staleness()) is None
    reason = channel.accept(arms_message(sequence=2, session_id="session-b"), NOW, Staleness())
    assert reason and "session changed" in reason
    # The first session's identity is preserved: a restarted executor cannot
    # silently continue this dataset.
    assert channel.session_id == "session-a"


def test_wrong_length_and_nonfinite_are_refused():
    channel = DeviceChannel("arms", ARMS_COUNT, BOOT)
    assert "expected 14" in channel.accept(arms_message(positions=[0.0]), NOW, Staleness())
    bad = [0.1] * ARMS_COUNT
    bad[3] = float("inf")
    assert "nonfinite" in channel.accept(arms_message(positions=bad), NOW, Staleness())


def test_sequence_rewind_is_refused():
    channel = DeviceChannel("arms", ARMS_COUNT, BOOT)
    assert channel.accept(arms_message(sequence=5), NOW, Staleness()) is None
    previous = channel.feedback
    reason = channel.accept(arms_message(sequence=4, source=NOW), NOW, Staleness())
    assert reason is not None
    assert channel.feedback is previous


def test_stale_and_future_source_stamps_are_refused():
    channel = DeviceChannel("arms", ARMS_COUNT, BOOT)
    stale = NOW - int(1.0e9)
    assert "old" in channel.accept(arms_message(source=stale), NOW, Staleness())
    future = NOW + int(1.0e9)
    assert "future" in channel.accept(arms_message(source=future), NOW, Staleness())


def test_unhealthy_feedback_is_refused_with_its_reason():
    channel = DeviceChannel("arms", ARMS_COUNT, BOOT)
    reason = channel.accept(
        arms_message(healthy=False, detail="servo error"), NOW, Staleness())
    assert reason and "servo error" in reason


def test_valid_sample_is_stored_with_source_stamp_as_token():
    channel = DeviceChannel("arms", ARMS_COUNT, BOOT)
    assert channel.accept(arms_message(source=NOW - 50), NOW, Staleness()) is None
    assert channel.feedback is not None
    # The token is the device stamp: a stationary arm still produces new samples
    # only when the device actually reported anew.
    assert channel.stamp == NOW - 50
    assert channel.feedback.received_monotonic_ns == NOW - 50


# ------------------------------------------------------------------ hand pairing

def test_pairing_needs_a_new_sample_from_both_hands():
    policy = Staleness()
    pairing = HandPairing(policy)

    pairing.stage("left_hand", _as_feedback([0.1] * HAND_COUNT, NOW - 10), NOW)
    assert pairing.take(NOW) is None, "one hand is not a pair"

    pairing.stage("right_hand", _as_feedback([0.2] * HAND_COUNT, NOW - 5), NOW)
    combined = pairing.take(NOW)
    assert combined is not None
    stamp, qpos = combined
    assert len(qpos) == 2 * HAND_COUNT
    assert qpos[:HAND_COUNT] == (0.1,) * HAND_COUNT
    assert qpos[HAND_COUNT:] == (0.2,) * HAND_COUNT
    # Availability stamp, not the device stamp.
    assert stamp >= NOW


def test_staged_samples_are_consumed_so_a_stall_cannot_reuse_them():
    pairing = HandPairing(Staleness())
    pairing.stage("left_hand", _as_feedback([0.1] * HAND_COUNT, NOW - 10), NOW)
    pairing.stage("right_hand", _as_feedback([0.2] * HAND_COUNT, NOW - 10), NOW)
    assert pairing.take(NOW) is not None
    # Nothing was re-staged, so a second take must not fabricate a repeat sample.
    assert pairing.take(NOW + 1) is None


def test_pairing_refuses_a_side_that_went_stale_before_publication():
    policy = Staleness(state_stale_s=0.3)
    pairing = HandPairing(policy)
    pairing.stage("left_hand", _as_feedback([0.1] * HAND_COUNT, NOW - 10), NOW)
    later = NOW + int(0.4e9)
    pairing.stage("right_hand", _as_feedback([0.2] * HAND_COUNT, later - 5), later)
    assert pairing.take(later) is None


def test_invalidate_drops_one_side_after_a_session_change():
    pairing = HandPairing(Staleness())
    pairing.stage("left_hand", _as_feedback([0.1] * HAND_COUNT, NOW - 10), NOW)
    pairing.invalidate("left_hand")
    pairing.stage("right_hand", _as_feedback([0.2] * HAND_COUNT, NOW - 5), NOW)
    assert pairing.take(NOW) is None


def test_duplicate_token_is_not_a_new_hand_sample():
    pairing = HandPairing(Staleness())
    same = _as_feedback([0.1] * HAND_COUNT, NOW - 10)
    pairing.stage("left_hand", same, NOW)
    pairing.stage("left_hand", same, NOW)
    pairing.stage("right_hand", _as_feedback([0.2] * HAND_COUNT, NOW - 10), NOW)
    stamp, qpos = pairing.take(NOW)
    assert qpos[:HAND_COUNT] == (0.1,) * HAND_COUNT


def test_policy_uses_the_validated_state_rate_and_limits():
    policy = Staleness()
    assert policy.state_stale_ns == int(0.3e9)
    assert policy.image_stale_ns == int(2.0e9)
    assert STATE_RATE_HZ == 120


# ------------------------------------------------------------------ locking proxy

class _Recorder:
    def __init__(self):
        self.calls = []

    def read_feedback(self):
        self.calls.append("read")
        return "value"


def test_locked_device_serialises_concurrent_calls():
    device = _Recorder()
    locked = LockedDevice(device)
    errors: list[BaseException] = []

    def worker():
        try:
            for _ in range(200):
                locked.read_feedback()
        except BaseException as error:  # noqa: BLE001 - surfaced below
            errors.append(error)

    threads = [threading.Thread(target=worker) for _ in range(4)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert not errors
    assert len(device.calls) == 800


def _as_feedback(values, stamp):
    from tianji_runtime.device import Feedback
    return Feedback(tuple(values), stamp, True, True, "")
