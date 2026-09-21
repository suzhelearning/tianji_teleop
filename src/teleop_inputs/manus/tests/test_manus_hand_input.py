"""Semantic/raw-stream regressions; no ROS installation or gloves required."""

import math
import unittest

from manus_bridge.manus_hand_input import ManusParser


NOW = 10_000_000_000


def skeleton(side=2):
    """SDK semantics with intentionally non-contiguous IDs and shuffled array order."""
    nodes = [(701, 701, 13, side, 0)]
    next_id = 1001
    for chain in range(5, 10):
        parent = 701
        joints = (1, 2, 3, 5) if chain == 5 else (1, 2, 3, 4, 5)
        for joint in joints:
            nodes.append((next_id, parent, chain, side, joint))
            parent = next_id
            next_id += 17
    return nodes[7:] + nodes[:7]


def metadata(parser, nodes, glove="abc", side="Right"):
    parser.feed_line(f"HAND {glove} {side} {len(nodes)}", now_ns=NOW)
    # NODE records themselves need not arrive in array-index order either.
    for index in reversed(range(len(nodes))):
        fields = " ".join(map(str, nodes[index]))
        parser.feed_line(f"NODE {glove} {index} {fields}", now_ns=NOW)


def pose(nodes, glove="abc", seq=1, source_ns=NOW, bad_value=None):
    values = []
    for node_id, _, _, _, _ in nodes:
        values.extend((node_id / 1000, 0.2, 0.3, 1, 0, 0, 0))
    if bad_value is not None:
        values[-1] = bad_value
    return f"POSE {glove} {seq} {source_ns} 123 " + " ".join(map(str, values))


class ManusParserTests(unittest.TestCase):
    def setUp(self):
        self.parser = ManusParser(stale_timeout=0.25)
        self.nodes = skeleton()

    def test_semantics_not_ids_or_array_order_select_thumb_and_fingers(self):
        metadata(self.parser, self.nodes)
        frame = self.parser.feed_line(pose(self.nodes), now_ns=NOW)
        wanted = [next(n[0] for n in self.nodes if n[2] == 13)]
        for chain in range(5, 10):
            joints = (1, 2, 3, 5) if chain == 5 else (2, 3, 4, 5)
            wanted.extend(next(n[0] for n in self.nodes if n[2] == chain and n[4] == j)
                          for j in joints)
        self.assertEqual(frame.side, "right")
        self.assertEqual(frame.data, tuple(v for n in wanted for v in (n / 1000, 0.2, 0.3)))

    def test_missing_node_metadata_never_publishes(self):
        self.parser.feed_line(f"HAND abc Right {len(self.nodes)}", now_ns=NOW)
        for index, node in enumerate(self.nodes[:-1]):
            self.parser.feed_line(f"NODE abc {index} " + " ".join(map(str, node)), now_ns=NOW)
        self.assertIsNone(self.parser.feed_line(pose(self.nodes), now_ns=NOW))

    def test_duplicate_record_invalidates_metadata_until_new_hand(self):
        metadata(self.parser, self.nodes)
        self.parser.feed_line("NODE abc 0 " + " ".join(map(str, self.nodes[0])), now_ns=NOW)
        self.assertIsNone(self.parser.feed_line(pose(self.nodes), now_ns=NOW))
        metadata(self.parser, self.nodes)
        self.assertIsNotNone(self.parser.feed_line(pose(self.nodes, seq=2, source_ns=NOW + 1), now_ns=NOW + 1))

    def test_ambiguous_semantics_missing_thumb_and_duplicate_ids_are_rejected(self):
        for mutation in ("semantics", "thumb", "ids", "cycle", "side"):
            with self.subTest(mutation=mutation):
                nodes = skeleton()
                thumb = next(i for i, n in enumerate(nodes) if n[2:] == (5, 2, 1))
                node = list(nodes[thumb])
                if mutation == "semantics":
                    node[4] = 2
                elif mutation == "thumb":
                    node[4] = 4  # SDK thumb has no distal joint.
                elif mutation == "ids":
                    node[0] = nodes[(thumb + 1) % len(nodes)][0]
                elif mutation == "cycle":
                    node[1] = node[0]
                else:
                    node[3] = 1
                nodes[thumb] = tuple(node)
                parser = ManusParser()
                metadata(parser, nodes)
                self.assertIsNone(parser.feed_line(pose(nodes), now_ns=NOW))

    def test_bad_right_input_does_not_republish_or_invalidate_left(self):
        metadata(self.parser, self.nodes)
        left = skeleton(side=1)
        metadata(self.parser, left, glove="def", side="Left")
        frame = self.parser.feed_line(pose(left, glove="def"), now_ns=NOW)
        self.assertEqual(frame.side, "left")
        self.assertIsNone(self.parser.feed_line(pose(self.nodes, bad_value=math.nan), now_ns=NOW))
        frame = self.parser.feed_line(pose(left, glove="def", seq=2, source_ns=NOW + 1), now_ns=NOW + 1)
        self.assertEqual(frame.side, "left")

    def test_sequence_and_source_time_must_both_increase(self):
        metadata(self.parser, self.nodes)
        self.assertIsNotNone(self.parser.feed_line(pose(self.nodes), now_ns=NOW))
        self.assertIsNone(self.parser.feed_line(
            pose(self.nodes, seq=1, source_ns=NOW + 1), now_ns=NOW + 1))
        self.assertIsNone(self.parser.feed_line(
            pose(self.nodes, seq=2, source_ns=NOW), now_ns=NOW + 1))
        self.assertIsNotNone(self.parser.feed_line(
            pose(self.nodes, seq=2, source_ns=NOW + 1), now_ns=NOW + 1))

    def test_stale_and_future_frames_do_not_prevent_recovery(self):
        metadata(self.parser, self.nodes)
        self.assertIsNone(self.parser.feed_line(pose(self.nodes), now_ns=NOW + 1_000_000_000))
        self.assertIsNone(self.parser.feed_line(
            pose(self.nodes, seq=2, source_ns=NOW + 2_000_000_000), now_ns=NOW + 1_000_000_000))
        self.assertIsNotNone(self.parser.feed_line(
            pose(self.nodes, seq=3, source_ns=NOW + 1_000_000_001), now_ns=NOW + 1_000_000_001))

    def test_nonfinite_frame_is_rejected_without_allowing_older_replay(self):
        metadata(self.parser, self.nodes)
        self.assertIsNone(self.parser.feed_line(
            pose(self.nodes, seq=2, source_ns=NOW + 1, bad_value=math.inf), now_ns=NOW + 1))
        self.assertIsNone(self.parser.feed_line(pose(self.nodes), now_ns=NOW + 1))
        self.assertIsNotNone(self.parser.feed_line(
            pose(self.nodes, seq=3, source_ns=NOW + 2), now_ns=NOW + 2))

    def test_metadata_refresh_does_not_reset_source_ordering(self):
        metadata(self.parser, self.nodes)
        self.assertIsNotNone(self.parser.feed_line(pose(self.nodes), now_ns=NOW))
        metadata(self.parser, self.nodes)
        self.assertIsNone(self.parser.feed_line(pose(self.nodes), now_ns=NOW))

    def test_optional_non_thumb_metacarpals_do_not_change_selected_points(self):
        nodes = [n for n in self.nodes if not (n[2] in range(6, 10) and n[4] == 1)]
        nodes = [(n[0], 701 if n[2] in range(6, 10) and n[4] == 2 else n[1], *n[2:])
                 for n in nodes]
        metadata(self.parser, self.nodes)
        full = self.parser.feed_line(pose(self.nodes), now_ns=NOW)
        parser = ManusParser()
        metadata(parser, nodes)
        reduced = parser.feed_line(pose(nodes), now_ns=NOW)
        self.assertEqual(reduced.data, full.data)

    def test_live_sdk_thumb_distal_label_maps_to_same_ip_landmark(self):
        # Real Metaglove Pro NODE records label the thumb IP as Distal (4),
        # despite the bundled SDK header comment specifying Intermediate (3).
        metadata(self.parser, self.nodes)
        expected = self.parser.feed_line(pose(self.nodes), now_ns=NOW)
        nodes = [(*n[:4], 4 if n[2] == 5 and n[4] == 3 else n[4]) for n in self.nodes]
        parser = ManusParser()
        metadata(parser, nodes)
        actual = parser.feed_line(pose(nodes), now_ns=NOW)
        self.assertIsNotNone(actual)
        self.assertEqual(actual.data, expected.data)

    def test_thumb_with_two_ip_candidates_is_rejected(self):
        nodes = self.nodes + [(9001, 701, 5, 2, 4)]
        metadata(self.parser, nodes)
        self.assertIsNone(self.parser.feed_line(pose(nodes), now_ns=NOW))

    def test_malformed_line_does_not_kill_stream(self):
        metadata(self.parser, self.nodes)
        for line in ("POSE abc nope", "POSE abc 1 2 3 junk", "NODE garbage", "noise"):
            self.assertIsNone(self.parser.feed_line(line, now_ns=NOW))
        self.assertIsNotNone(self.parser.feed_line(pose(self.nodes), now_ns=NOW))

    def test_two_active_gloves_for_same_side_are_ambiguous(self):
        metadata(self.parser, self.nodes)
        metadata(self.parser, self.nodes, glove="def")
        self.assertIsNotNone(self.parser.feed_line(pose(self.nodes), now_ns=NOW))
        self.assertIsNone(self.parser.feed_line(pose(self.nodes, glove="def"), now_ns=NOW))
        self.assertIsNone(self.parser.feed_line(
            pose(self.nodes, seq=2, source_ns=NOW + 1), now_ns=NOW + 1))

    def test_stale_glove_does_not_block_replacement_on_same_side(self):
        metadata(self.parser, self.nodes)
        self.assertIsNotNone(self.parser.feed_line(pose(self.nodes), now_ns=NOW))
        metadata(self.parser, self.nodes, glove="def")
        later = NOW + 300_000_000
        frame = self.parser.feed_line(
            pose(self.nodes, glove="def", source_ns=later), now_ns=later)
        self.assertEqual(frame.side, "right")

    def test_incomplete_peer_metadata_does_not_block_valid_hand(self):
        metadata(self.parser, self.nodes)
        self.parser.feed_line(f"HAND def Right {len(self.nodes)}", now_ns=NOW)
        self.assertIsNotNone(self.parser.feed_line(pose(self.nodes), now_ns=NOW))


if __name__ == "__main__":
    unittest.main()
