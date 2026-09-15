from __future__ import annotations

from pathlib import Path
import struct
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import zlib

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
from retargeting.example import tj_wuji2_hand_bridge as bridge


def message(data, label=None, *, size=None, stride=None, offset=0):
    dimensions = [] if label is None else [SimpleNamespace(
        label=label, size=len(data) if size is None else size,
        stride=len(data) if stride is None else stride,
    )]
    return SimpleNamespace(
        data=data, layout=SimpleNamespace(dim=dimensions, data_offset=offset),
    )


def landmarks():
    points = np.zeros((21, 3))
    for finger in range(5):
        for joint in range(4):
            points[1 + finger * 4 + joint] = (
                (finger - 2) * 0.018, 0.025 + joint * 0.022, 0.003 * finger,
            )
    return points


class HandInputTests(unittest.TestCase):
    def test_labeled_hands_stay_independent_without_coordinate_reflection(self):
        points = landmarks()
        for side in ("left", "right"):
            hands = bridge.interpret_hand_input(message(points.ravel(), side))
            self.assertEqual(set(hands), {side})
            np.testing.assert_array_equal(hands[side], points)

    def test_unlabeled_single_and_dual_input_order(self):
        right = landmarks()
        left = landmarks() + [0.1, 0.2, 0.3]
        hands = bridge.interpret_hand_input(message(right.ravel()))
        np.testing.assert_array_equal(hands["right"], right)
        hands = bridge.interpret_hand_input(message(left.ravel()), "left")
        np.testing.assert_array_equal(hands["left"], left)
        hands = bridge.interpret_hand_input(message(np.concatenate((right.ravel(), left.ravel()))))
        np.testing.assert_array_equal(hands["right"], right)
        np.testing.assert_array_equal(hands["left"], left)

    def test_invalid_message_edges_are_rejected(self):
        valid = landmarks().ravel()
        nan = valid.copy()
        nan[40] = np.nan
        infinity = valid.copy()
        infinity[62] = np.inf
        malformed = [
            message([]), message(valid[:-1]), message(np.zeros(64)),
            message(valid.reshape(21, 3)), message(nan), message(infinity),
            message(np.zeros(63)), message(np.arange(63, dtype=float)),
            message(np.concatenate((valid, np.zeros(63)))),
            message(valid, "Right"), message(valid, "both"),
            message(np.tile(valid, 2), "left"),
            message(valid, "left", size=21), message(valid, "right", stride=1),
            message(valid, "left", offset=1), message(valid, offset=1),
        ]
        two_dims = message(valid, "left")
        two_dims.layout.dim *= 2
        malformed.append(two_dims)
        for value in malformed:
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    bridge.interpret_hand_input(value)
        with self.assertRaises(ValueError):
            bridge.interpret_hand_input(message(valid), "both")


class PacketTests(unittest.TestCase):
    def test_exact_cpp_wire_contract_and_crc(self):
        left = np.linspace(-0.8, 0.8, 20)
        right = np.linspace(0.7, -0.7, 20)
        packet = bridge.encode_packet(0x0102030405060708, 123456789, left=left, right=right,
                                      left_timestamp_ns=123450000, right_timestamp_ns=123455000)
        self.assertEqual(len(packet), 364)
        self.assertEqual(struct.unpack_from("<4sBBHQqqq", packet),
                         (b"TJH2", 2, 3, 364, 0x0102030405060708,
                          123456789, 123450000, 123455000))
        self.assertEqual(struct.unpack_from("<20d", packet, 40), tuple(left))
        self.assertEqual(struct.unpack_from("<20d", packet, 200), tuple(right))
        self.assertEqual(struct.unpack_from("<I", packet, 360)[0], zlib.crc32(packet[:360]))
        # Bitwise ISO-HDLC reference, independent of the encoder's CRC library.
        crc = 0xFFFFFFFF
        for byte in packet[:360]:
            crc ^= byte
            for _ in range(8):
                crc = (crc >> 1) ^ (0xEDB88320 if crc & 1 else 0)
        self.assertEqual(struct.unpack_from("<I", packet, 360)[0], crc ^ 0xFFFFFFFF)

    def test_absent_side_has_zero_timestamp_and_joints(self):
        left_packet = bridge.encode_packet(1, 10, left=np.arange(20), left_timestamp_ns=8)
        right_packet = bridge.encode_packet(2, 11, right=-np.arange(20), right_timestamp_ns=9)
        self.assertEqual(left_packet[5], 1)
        self.assertEqual(right_packet[5], 2)
        self.assertEqual(struct.unpack_from("<qq", left_packet, 24), (8, 0))
        self.assertEqual(struct.unpack_from("<qq", right_packet, 24), (0, 9))
        self.assertEqual(struct.unpack_from("<20d", left_packet, 200), (0.0,) * 20)
        self.assertEqual(struct.unpack_from("<20d", right_packet, 40), (0.0,) * 20)
        self.assertEqual(struct.unpack_from("<20d", right_packet, 200), tuple(-np.arange(20)))

    def test_invalid_packets_cannot_be_encoded(self):
        for sequence, stamp in ((0, 1), (-1, 1), (2**64, 1), (1, 0), (1, -1), (1, 2**63)):
            with self.subTest(sequence=sequence, stamp=stamp):
                with self.assertRaises(ValueError):
                    bridge.encode_packet(sequence, stamp, left=np.zeros(20), left_timestamp_ns=1)
        for values in (np.zeros(19), np.zeros((5, 4)), [np.nan] * 20, [np.inf] * 20):
            with self.assertRaises(ValueError):
                bridge.encode_packet(1, 1, right=values, right_timestamp_ns=1)
        with self.assertRaises(ValueError):
            bridge.encode_packet(1, 1)
        for source_ns in (-1, 0, 11, 2**63, 1.5):
            with self.subTest(source_ns=source_ns), self.assertRaises(ValueError):
                bridge.encode_packet(1, 10, right=np.zeros(20), right_timestamp_ns=source_ns)
        with self.assertRaises(ValueError):
            bridge.encode_packet(1, 10, left=np.zeros(20), left_timestamp_ns=5,
                                 right_timestamp_ns=5)

    def test_joint_reordering_uses_names_not_urdf_position(self):
        # The C++ viewer's consumer order is thumb, index, middle, ring, pinky.
        stems = ["thumb_cmc_flex", "thumb_cmc_abd", "thumb_mcp", "thumb_ip"]
        for finger in ("index_finger", "middle_finger", "ring_finger", "pinky"):
            stems.extend(f"{finger}_{joint}" for joint in ("mcp_flex", "mcp_abd", "pip", "dip"))
        for side, prefix in (("left", "l_"), ("right", "r_")):
            expected = [prefix + stem for stem in stems]
            source = expected[4:] + expected[:4]
            perm = bridge.joint_permutation(source, side)
            source_values = np.array([expected.index(name) + 0.25 for name in source])
            ordered = source_values[perm]
            packet = bridge.encode_packet(1, 1, **{side: ordered, f"{side}_timestamp_ns": 1})
            self.assertEqual(struct.unpack_from("<20d", packet, 40 if side == "left" else 200),
                             tuple(np.arange(20) + 0.25))
            with self.assertRaises(ValueError):
                bridge.joint_permutation(source[:-1] + [source[0]], side)
            with self.assertRaises(ValueError):
                bridge.joint_permutation(source, "right" if side == "left" else "left")

    def test_official_model_limits_fit_viewer_for_every_named_joint(self):
        viewer = ET.parse(ROOT / "control/models/marvin_m6_wuji2.xml")
        viewer_limits = {joint.attrib["name"]: tuple(map(float, joint.attrib["range"].split()))
                         for joint in viewer.findall(".//joint") if "range" in joint.attrib}
        for side in ("left", "right"):
            model = ET.parse(ROOT / f"retargeting/wuji_retargeting/wuji-description/hand2/hand2_beta1/body/urdf/{side}.urdf")
            joints = [joint for joint in model.findall("joint") if joint.attrib["type"] == "revolute"]
            permutation = bridge.joint_permutation([joint.attrib["name"] for joint in joints], side)
            self.assertEqual(len(permutation), 20)
            for joint in joints:
                limit = joint.find("limit")
                self.assertEqual((float(limit.attrib["lower"]), float(limit.attrib["upper"])),
                                 viewer_limits[joint.attrib["name"]])


class PublicationTests(unittest.TestCase):
    def setUp(self):
        self.now_ns = 1_000_000_000
        self.cache = bridge.HandTargetCache(clock=lambda: self.now_ns)

    def solve(self, side, value, received_ns):
        model = SimpleNamespace(retarget=lambda points: np.full(20, value))
        self.cache.retarget(side, model, None, received_ns)

    def test_no_publication_until_success_and_failed_first_solve_stays_absent(self):
        self.assertIsNone(self.cache.sample())

        def fail(points):
            raise ValueError("optimizer failed")

        with self.assertRaises(ValueError):
            self.cache.retarget("left", SimpleNamespace(retarget=fail), None, self.now_ns)
        self.now_ns += 10_000_000
        self.assertIsNone(self.cache.sample())
        self.solve("right", 0.2, self.now_ns)
        packet = self.cache.sample()
        self.assertEqual(packet[5], 2)
        self.assertEqual(struct.unpack_from("<qq", packet, 24), (0, self.now_ns))

    def test_bilateral_latest_samples_repeat_without_rejuvenating_dropped_side(self):
        self.solve("left", 0.1, self.now_ns - 20_000_000)
        self.solve("right", 0.2, self.now_ns - 10_000_000)
        first = self.cache.sample()
        self.now_ns += 10_000_000
        repeated = self.cache.sample()
        self.assertEqual(struct.unpack_from("<Q", repeated, 8)[0],
                         struct.unpack_from("<Q", first, 8)[0] + 1)
        self.assertEqual(struct.unpack_from("<q", repeated, 16)[0], self.now_ns)
        self.assertEqual(repeated[24:360], first[24:360])
        # Multiple solves between ticks collapse to the latest, never a backlog.
        self.now_ns += 200_000_000
        self.solve("right", 0.3, self.now_ns - 1)
        self.solve("right", 0.4, self.now_ns)
        latest = self.cache.sample()
        self.assertEqual(latest[5], 3)
        self.assertEqual(struct.unpack_from("<qq", latest, 24), (980_000_000, self.now_ns))
        self.assertEqual(struct.unpack_from("<20d", latest, 40), (0.1,) * 20)
        self.assertEqual(struct.unpack_from("<20d", latest, 200), (0.4,) * 20)

    def test_failed_or_invalid_solve_preserves_last_pose_and_original_age(self):
        self.solve("left", 0.1, self.now_ns)
        first = self.cache.sample()

        def fail(points):
            raise RuntimeError("optimizer failed")

        for retarget in (fail, lambda points: np.full(20, np.nan)):
            self.now_ns += 100_000_000
            with self.assertRaises((RuntimeError, ValueError)):
                self.cache.retarget("left", SimpleNamespace(retarget=retarget), None, self.now_ns)
            repeated = self.cache.sample()
            self.assertEqual(repeated[24:360], first[24:360])
            self.assertEqual(struct.unpack_from("<q", repeated, 16)[0], self.now_ns)

    def test_sampling_does_not_wait_for_inflight_solve(self):
        self.solve("left", 0.1, self.now_ns)
        entered = threading.Event()
        release = threading.Event()
        sampled = threading.Event()
        packets = []

        def slow_solve(points):
            entered.set()
            if not release.wait(2):
                raise RuntimeError("test solve not released")
            return np.full(20, 0.2)

        def sample():
            packets.append(self.cache.sample())
            sampled.set()

        solver = threading.Thread(target=self.cache.retarget, args=(
            "right", SimpleNamespace(retarget=slow_solve), None, self.now_ns))
        sampler = threading.Thread(target=sample)
        solver.start()
        try:
            self.assertTrue(entered.wait(1), "solve did not start")
            sampler.start()
            self.assertTrue(sampled.wait(1), "publisher blocked behind solve")
            self.assertEqual(packets[0][5], 1)
        finally:
            release.set()
            solver.join()
            if sampler.ident is not None:
                sampler.join()
        self.assertEqual(self.cache.sample()[5], 3)

    def test_scheduler_samples_actual_time_without_bursting_after_stall(self):
        self.solve("left", 0.1, self.now_ns)
        packets = []
        waits = []

        def wait(timeout):
            waits.append(timeout)
            if len(waits) > 4:
                return True
            self.now_ns += round(timeout * 1_000_000_000)
            if len(waits) == 2:
                self.now_ns += 35_000_000
            return False

        with patch.object(bridge.time, "monotonic_ns", side_effect=lambda: self.now_ns):
            bridge.publish_targets(self.cache,
                                   SimpleNamespace(sendto=lambda packet, address: packets.append(packet)),
                                   ("127.0.0.1", 16000), SimpleNamespace(wait=wait))
        self.assertEqual([struct.unpack_from("<q", packet, 16)[0] for packet in packets],
                         [1_000_000_000, 1_045_000_000, 1_055_000_000, 1_065_000_000])
        self.assertEqual([struct.unpack_from("<q", packet, 24)[0] for packet in packets],
                         [1_000_000_000] * 4)


class ModelWristFrameTests(unittest.TestCase):
    @staticmethod
    def model_landmarks(hand, side, qpos):
        robot = hand.retargeter.optimizer.robot
        prefix = side[0] + "_"
        names = [prefix + "wrist"]
        for finger in ("thumb", "index_finger", "middle_finger", "ring_finger", "pinky"):
            names.extend(prefix + finger + "_" + link
                         for link in ("proximal_abd", "middle", "distal", "tip"))
        robot.compute_forward_kinematics(qpos)
        return np.array([robot.get_link_pose(robot.get_link_index(name))[:3, 3] for name in names])

    def test_open_hand_remains_open_after_wrist_normalization(self):
        for side in ("left", "right"):
            with self.subTest(side=side):
                hand = bridge.HandRetargeter(ROOT / "retargeting", side)
                points = self.model_landmarks(hand, side, np.zeros(20))
                self.assertLess(float(np.max(np.abs(hand.retarget(points)))), 0.035)

    def test_each_finger_flex_drives_its_named_joint_without_cross_talk(self):
        for side in ("left", "right"):
            for stem in ("thumb_mcp", "index_finger_pip", "middle_finger_pip",
                         "ring_finger_pip", "pinky_pip"):
                with self.subTest(side=side, joint=stem):
                    hand = bridge.HandRetargeter(ROOT / "retargeting", side)
                    names = hand.retargeter.optimizer.robot.dof_joint_names
                    qpos = np.zeros(20)
                    qpos[names.index(side[0] + "_" + stem)] = 0.6
                    points = self.model_landmarks(hand, side, qpos)
                    for _ in range(20):
                        output = hand.retarget(points)
                    slot = bridge.JOINT_STEMS.index(stem)
                    self.assertAlmostEqual(float(output[slot]), 0.6, delta=0.05)
                    untouched = [i for i in range(20) if i // 4 != slot // 4]
                    self.assertLess(float(np.max(np.abs(output[untouched]))), 0.05)

    def test_sdk_world_stream_preserves_palmward_finger_flexion(self):
        from scipy.spatial.transform import Rotation
        from manus.manus_hand_input import ManusParser

        rotation = Rotation.from_euler("xyz", [25, -40, 70], degrees=True).as_matrix()
        for side, sdk_side in (("left", 1), ("right", 2)):
            for stem in ("thumb_mcp", "index_finger_pip", "middle_finger_pip",
                         "ring_finger_pip", "pinky_pip"):
                with self.subTest(side=side, joint=stem):
                    hand = bridge.HandRetargeter(ROOT / "retargeting", side)
                    qpos = np.zeros(20)
                    names = hand.retargeter.optimizer.robot.dof_joint_names
                    qpos[names.index(side[0] + "_" + stem)] = 0.6
                    points = self.model_landmarks(hand, side, qpos) @ rotation.T + [1, 2, 3]
                    nodes = [(701, 701, 13, sdk_side, 0, 0)]
                    for finger, chain in enumerate(range(5, 10)):
                        parent = 701
                        joints = (1, 2, 4, 5) if chain == 5 else (2, 3, 4, 5)
                        for offset, joint in enumerate(joints):
                            keypoint = 1 + finger * 4 + offset
                            node_id = 1000 + keypoint * 17
                            nodes.append((node_id, parent, chain, sdk_side, joint, keypoint))
                            parent = node_id
                    nodes.reverse()
                    parser = ManusParser()
                    parser.feed_line(f"HAND abc {side.title()} 21", now_ns=1000)
                    for index, node in enumerate(nodes):
                        parser.feed_line(f"NODE abc {index} " + " ".join(map(str, node[:5])), now_ns=1000)
                    values = [value for node in nodes for value in (*points[node[5]], 1, 0, 0, 0)]
                    frame = parser.feed_line("POSE abc 1 1000 0 " + " ".join(map(str, values)), now_ns=1000)
                    self.assertIsNotNone(frame)
                    decoded = bridge.interpret_hand_input(message(frame.data, side))[side]
                    for _ in range(25):
                        output = hand.retarget(decoded)
                    slot = bridge.JOINT_STEMS.index(stem)
                    self.assertAlmostEqual(float(output[slot]), 0.6, delta=0.05)

    def test_rigid_hand_motion_preserves_model_wrist_targets(self):
        from scipy.spatial.transform import Rotation
        rotation = Rotation.from_euler("xyz", [35, -20, 70], degrees=True).as_matrix()
        for side in ("left", "right"):
            first = bridge.HandRetargeter(ROOT / "retargeting", side)
            second = bridge.HandRetargeter(ROOT / "retargeting", side)
            robot = first.retargeter.optimizer.robot
            qpos = np.zeros(20)
            qpos[robot.dof_joint_names.index(side[0] + "_index_finger_pip")] = 0.5
            points = self.model_landmarks(first, side, qpos)
            _, original = first.retargeter.retarget_verbose(points)
            _, moved = second.retargeter.retarget_verbose(points @ rotation.T + [1, -2, 0.5])
            np.testing.assert_allclose(original["mediapipe_kp"], moved["mediapipe_kp"],
                                       atol=1e-12, rtol=0)


if __name__ == "__main__":
    unittest.main()
