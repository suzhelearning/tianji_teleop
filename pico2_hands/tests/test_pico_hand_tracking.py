from __future__ import annotations

import struct
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from pico2_hands.reference.pico import (
    PICO_TO_MEDIAPIPE,
    parse_pico_packet,
    pico_to_mediapipe,
    tracking_pose_to_current_head,
)


def _pose(index: int) -> list[float]:
    return [float(index), float(index) + 0.1, float(index) + 0.2, 0.0, 0.0, 0.0, 1.0]


def _packet(*, flags: int = 0x07, joint_count: int = 26, version: int = 1) -> bytes:
    payload = bytearray(struct.pack("<BBBB", version, flags, joint_count, 0))
    payload.extend(struct.pack("<7f", *_pose(100)))
    for side_offset in (0, 1000):
        payload.extend(struct.pack("<BBBB", 1, 0, 0, 0))
        payload.extend(struct.pack("<7f", *_pose(200 + side_offset)))
        for joint in range(26):
            payload.extend(struct.pack("<BBBB", 1, 0, 0, 0))
            payload.extend(struct.pack("<7f", *_pose(side_offset + joint)))
            payload.extend(struct.pack("<f", 0.01 * (joint + 1)))
    assert len(payload) == 1968
    return struct.pack("<BBqI", 0xAB, 0x40, 1234, len(payload)) + payload


class PicoHandTrackingTest(unittest.TestCase):
    def test_packet_parser_decodes_complete_raw_frame(self) -> None:
        packet = _packet()
        frame = parse_pico_packet(
            packet,
            receiver_instance_id="receiver",
            connection_generation=2,
            receiver_frame_sequence=9,
            received_timestamp_ns=987,
        )

        self.assertEqual(frame.source_timestamp_ms, 1234)
        self.assertEqual(frame.flags, 0x07)
        self.assertEqual(frame.joint_count, 26)
        self.assertEqual(frame.raw_packet, packet)
        self.assertTrue(frame.hands["left"].valid)
        self.assertEqual(frame.hands["right"].joints[25].name, "little_tip")
        self.assertAlmostEqual(frame.hands["right"].joints[25].radius_m, 0.26, places=6)
        self.assertEqual(frame.association_id, "receiver:2:9")

    def test_26_points_map_to_21_points_without_index_guessing(self) -> None:
        points = np.arange(78, dtype=np.float64).reshape(26, 3)
        valid = np.ones(26, dtype=bool)
        valid[7] = False

        mapped, mapped_valid = pico_to_mediapipe(points, valid)

        self.assertEqual(PICO_TO_MEDIAPIPE, [1, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13, 14, 15, 17, 18, 19, 20, 22, 23, 24, 25])
        np.testing.assert_array_equal(mapped[0], points[1])
        np.testing.assert_array_equal(mapped[5], [0.0, 0.0, 0.0])
        self.assertFalse(bool(mapped_valid[5]))
        np.testing.assert_array_equal(mapped[20], points[25])

    def test_invalid_protocol_headers_are_rejected(self) -> None:
        for packet in (
            bytes([0x00]) + _packet()[1:],
            _packet()[:1] + bytes([41]) + _packet()[2:],
            _packet(version=2),
            _packet(joint_count=25),
            _packet()[:-1],
        ):
            with self.assertRaises(ValueError):
                parse_pico_packet(
                    packet,
                    receiver_instance_id="receiver",
                    connection_generation=1,
                    receiver_frame_sequence=1,
                    received_timestamp_ns=1,
                )

    def test_current_head_transform_returns_head_relative_pose(self) -> None:
        head = np.array([1.0, 2.0, 3.0, *Rotation.from_euler("z", 90, degrees=True).as_quat()])
        wrist = np.array([1.0, 3.0, 3.0, *Rotation.from_euler("z", 90, degrees=True).as_quat()])

        relative = tracking_pose_to_current_head(head, wrist)

        np.testing.assert_allclose(relative[:3], [1.0, 0.0, 0.0], atol=1.0e-12)
        np.testing.assert_allclose(
            Rotation.from_quat(relative[3:]).as_matrix(),
            np.eye(3),
            atol=1.0e-12,
        )


if __name__ == "__main__":
    unittest.main()
