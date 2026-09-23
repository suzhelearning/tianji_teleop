from types import SimpleNamespace
import unittest

from pico2_hands.runtime_stats import RuntimeStats


def frame(sequence, stamp, generation=1):
    return SimpleNamespace(receiver_instance_id="receiver", connection_generation=generation,
                           receiver_frame_sequence=sequence, received_timestamp_ns=stamp)


class RuntimeStatsTest(unittest.TestCase):
    def test_periodic_summary_counts_unique_input_and_preserves_its_age(self):
        stats = RuntimeStats(1., 0)
        first = frame(1, 10_000_000)
        stats.record_input(first)
        stats.record_input(first)
        stats.accepted_frames += 1
        stats.record_cycle(20_000_000, 24_000_000, first, "TELEOP", None)
        self.assertIsNone(stats.summary(999_999_999))
        stats.record_cycle(990_000_000, 997_000_000, first, "BRAKING", "stale_input")
        result = stats.summary(1_000_000_000)
        self.assertEqual(result["input_hz"], 1.)
        self.assertEqual(result["accepted_input_hz"], 1.)
        self.assertEqual(result["control_hz"], 2.)
        self.assertEqual(result["work_over_5ms"], 1)
        self.assertEqual(result["braking_entries"], {"stale_input": 1})
        self.assertEqual(result["timing_ms"]["receive_age"]["max_ms"], 987.)
        self.assertIsNone(stats.summary(1_000_000_001, final=True))
        stats.record_cycle(1_005_000_000, 1_006_000_000, first, "BRAKING", "stale_input")
        next_window = stats.summary(1_010_000_000, final=True)
        self.assertEqual(next_window["braking_entries"], {})
        self.assertEqual(next_window["input_frames"], 0)
        self.assertEqual(next_window["timing_ms"]["work"]["max_ms"], 1.)

    def test_reconnect_does_not_create_a_spurious_cross_session_input_gap(self):
        stats = RuntimeStats(1., 0)
        stats.record_input(frame(5, 100_000_000))
        stats.record_input(frame(4, 200_000_000))
        stats.record_input(frame(1, 800_000_000, generation=2))
        stats.record_input(frame(2, 820_000_000, generation=2))
        stats.record_cycle(900_000_000, 905_000_000, None, "HOLD", None)
        result = stats.summary(1_000_000_000)
        self.assertEqual(result["input_frames"], 3)
        self.assertEqual(result["timing_ms"]["input_interval"]["count"], 1)
        self.assertEqual(result["timing_ms"]["input_interval"]["mean_ms"], 20.)
        self.assertIsNone(result["timing_ms"]["receive_age"])

    def test_quantiles_keep_slow_outliers_and_report_empty_stages_as_null(self):
        stats = RuntimeStats(1., 0)
        for _ in range(98):
            stats.observe("dls", 1_010_000)
        stats.observe("dls", 30_000_000)
        stats.observe("dls", 2_000_000_000)
        stats.record_cycle(0, 1_000_000, None, "WAITING", None)
        result = stats.summary(1_000_000_000)
        timing = result["timing_ms"]["dls"]
        self.assertEqual(timing["count"], 100)
        self.assertEqual(timing["p95_ms"], 1.1)
        self.assertEqual(timing["p99_ms"], 30.)
        self.assertEqual(timing["max_ms"], 2000.)
        self.assertIsNone(result["timing_ms"]["hands"])
