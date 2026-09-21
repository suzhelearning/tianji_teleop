from dataclasses import replace
import struct
import unittest
from unittest.mock import patch

from pico2_hands.observe import ObservationState
from pico2_hands.reference.runtime import PicoPacketStream, PicoTcpReceiver
from pico2_hands.tests.test_pico_hand_tracking import _packet
from pico2_hands.tests import test_official_pico_input


class ObservationTest(unittest.TestCase):
    def frame(self):
        return test_official_pico_input.OfficialPicoInputTest().frame()

    def test_freshness_and_duplicate_not_renewed(self):
        state = ObservationState()
        frame = self.frame()
        self.assertTrue(state.accept(frame))
        self.assertTrue(state.snapshot(1001)["fresh"])
        self.assertFalse(state.accept(replace(frame, received_timestamp_ns=300_000_000,
                                             receiver_frame_sequence=8)))
        result = state.snapshot(300_000_000)
        self.assertFalse(result["fresh"])
        self.assertFalse(any(result["arms"].values()))
        self.assertFalse(result["robot_commands_enabled"])
        self.assertEqual(result["rejected_frames"], 1)
        self.assertEqual(result["gestures"], dict(left="unknown", right="unknown"))

    def test_regression_rejected_new_connection_can_restart_clock(self):
        state = ObservationState()
        frame = self.frame()
        state.accept(frame)
        older = replace(frame, source_timestamp_ms=1, receiver_frame_sequence=8)
        self.assertFalse(state.accept(older))
        self.assertTrue(state.accept(replace(older, connection_generation=4,
                                           receiver_frame_sequence=0)))

    def test_disconnect_and_invalid_hand_clear_gesture(self):
        state = ObservationState()
        frame = self.frame()
        state.accept(frame)
        state.disconnected(ConnectionError("disconnected"))
        self.assertFalse(state.snapshot(1001)["fresh"])
        newer = replace(frame, source_timestamp_ms=1235, receiver_frame_sequence=8,
                        hands={s: replace(h, valid=False) for s, h in frame.hands.items()})
        state.accept(newer)
        result = state.snapshot(1001)
        self.assertFalse(any(result["hands"].values()))
        self.assertEqual(result["gestures"], dict(left="unknown", right="unknown"))

    def test_head_loss_does_not_remove_hand_observation(self):
        state = ObservationState()
        state.accept(replace(self.frame(), head_valid=False))
        result = state.snapshot(1001)
        self.assertFalse(any(result["arms"].values()))
        self.assertTrue(all(result["hands"].values()))

    def test_stream_fragmentation_and_multiple_packets(self):
        stream = PicoPacketStream(receiver_instance_id="test", connection_generation=1)
        packet = _packet()
        self.assertEqual(stream.feed(packet[:17]), [])
        frames = stream.feed(packet[17:] + packet, received_timestamp_ns=1000)
        self.assertEqual([f.receiver_frame_sequence for f in frames], [0, 1])
        self.assertEqual(frames[0].raw_packet, packet)

    def test_wrong_apk_header_rejected(self):
        stream = PicoPacketStream(receiver_instance_id="test", connection_generation=1)
        with self.assertRaises(ValueError):
            stream.feed(struct.pack("<BBqI", 0xAB, 0x41, 1, 1968))

    def test_receive_interface_without_adb_or_devices(self):
        class Socket:
            def __enter__(self): return self
            def __exit__(self, *args): pass
            def close(self): pass
            def settimeout(self, timeout): pass
            def recv(self, size): return _packet()

        receiver = PicoTcpReceiver(host="127.0.0.1", port=10002,
                                   receiver_instance_id="fake", auto_adb_forward=False,
                                   socket_factory=lambda *args, **kwargs: Socket())
        frames = []

        def accept(frame):
            frames.append(frame)
            receiver.stop()

        with patch("pico2_hands.reference.runtime.subprocess.run") as adb:
            receiver.run(accept)
        adb.assert_not_called()
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].connection_generation, 1)
