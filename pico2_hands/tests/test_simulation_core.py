from dataclasses import replace
import unittest
import numpy as np
from pico2_hands.simulation_core import SimulationCore
from pico2_hands.reference.pico import parse_pico_packet
from pico2_hands.tests.test_pico_hand_tracking import _packet


class FakeIk:
    def __init__(self):
        self.resets = self.solves = 0
    def reset(self, q):
        self.resets += 1
    def solve(self, seeds, targets, **kwargs):
        self.solves += 1
        return {s: dict(accepted=True, joints=seeds[i]+.001) for i,s in enumerate(("left", "right"))}


class SimulationCoreTest(unittest.TestCase):
    def frame(self, now, sequence=1, generation=1):
        frame = parse_pico_packet(_packet(), receiver_instance_id="test", connection_generation=generation,
                                  receiver_frame_sequence=sequence, received_timestamp_ns=now)
        return replace(frame, source_timestamp_ms=now//1_000_000)

    def core(self):
        return SimulationCore(np.zeros(54), FakeIk())

    def test_start_without_c_but_not_without_fresh_input(self):
        core = self.core()
        self.assertFalse(core.action("s", 1_000_000_000))
        core.offer(self.frame(1_000_000_000), 1_000_000_000)
        self.assertTrue(core.action("s", 1_000_000_000))
        core.tick(1_000_000_000)
        self.assertEqual(core.ik.solves, 1)
        self.assertFalse(core.action("c", 1_000_000_001))

    def test_new_takeover_and_input_hold_clear_rejection_streak(self):
        core = self.core(); now = 1_000_000_000
        core.offer(self.frame(now), now)
        core.rejections = 19
        core.input_hold = True
        self.assertTrue(core.action("s", now))
        self.assertEqual(core.rejections, 0)
        self.assertFalse(core.input_hold)
        core.rejections = 19
        core.tick(now+100_000_000)
        self.assertEqual(core.rejections, 0)
        self.assertEqual(core.ik.solves, 0)

    def test_h_rejects_s_until_home_and_new_input_then_r_still_works(self):
        core = self.core()
        now = 1_000_000_000
        core.offer(self.frame(now), now)
        core.action("s", now); core.tick(now)
        core.action("h", now)
        self.assertFalse(core.action("s", now))
        self.assertFalse(core.action("r", now))
        core.tick(now+2_000_000_000)
        self.assertEqual(core.state, "idle")
        np.testing.assert_array_equal(core.q, core.home)
        self.assertFalse(core.action("s", now+2_000_000_000))
        stamp = now+2_000_000_001
        core.offer(self.frame(stamp, 2), stamp)
        self.assertTrue(core.action("r", stamp))
        self.assertFalse(core.action("s", stamp))
        stamp += 10_000_000
        core.offer(self.frame(stamp, 3), stamp)
        self.assertTrue(core.action("s", stamp))

    def test_stale_holds_without_advancing_ik_and_recovers_same_mapping(self):
        core = self.core()
        now = 1_000_000_000
        core.offer(self.frame(now), now); core.action("s", now); core.tick(now)
        q = core.q.copy()
        mapper = core.mapping
        core.tick(now+100_000_000)
        np.testing.assert_array_equal(core.q, q)
        self.assertEqual(core.ik.solves, 1)
        core.offer(self.frame(now+110_000_000, 2), now+110_000_000)
        core.tick(now+110_000_000)
        self.assertEqual(core.ik.solves, 2)
        self.assertIs(core.mapping, mapper)

    def test_reconnection_returns_home_and_does_not_auto_takeover(self):
        core = self.core(); now = 1_000_000_000
        core.offer(self.frame(now), now); core.action("s", now); core.tick(now)
        core.offer(self.frame(now+10_000_000, 1, 2), now+10_000_000)
        self.assertEqual(core.state, "homing")
        self.assertFalse(core.action("s", now+10_000_000))

    def test_duplicate_source_timestamp_does_not_renew_freshness(self):
        core = self.core(); now = 1_000_000_000
        frame = self.frame(now)
        core.offer(frame, now)
        duplicate = replace(frame, receiver_frame_sequence=2, received_timestamp_ns=now+1_000_000_000)
        self.assertFalse(core.offer(duplicate, now+1_000_000_000))
        self.assertFalse(core.fresh(now+1_000_000_000))

    def test_q_only_completes_after_home(self):
        core = self.core(); now = 1_000_000_000
        core.action("q", now)
        self.assertFalse(core.done)
        core.tick(now+2_000_000_000)
        self.assertTrue(core.done)
