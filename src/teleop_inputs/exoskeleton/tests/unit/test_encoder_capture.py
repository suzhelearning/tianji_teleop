"""编码器短窗口采样合同。"""

from __future__ import annotations

import unittest
from unittest.mock import patch

from data_glove_wuji_teleop.adapters.glove.encoder_stream import (
    EncoderFrame,
)
from data_glove_wuji_teleop.application.encoder_capture import read_encoder_window


class EncoderCaptureWindowTest(unittest.TestCase):
    def test_remote_timestamp_stall_is_bounded_by_local_monotonic_clock(self) -> None:
        class StalledConnection:
            channels = 21
            cs_by_joint = list(range(21))
            zeroed = False
            range_min = 0.0
            range_max = 360.0
            sequence = 0

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def read_frame(self, *, deadline_monotonic: float):
                self.sequence += 1
                return EncoderFrame(self.sequence, 0, 0, [0.0] * 21)

        with (
            patch(
                "data_glove_wuji_teleop.application.encoder_capture.EncoderConnection.connect",
                return_value=StalledConnection(),
            ),
            patch(
                "data_glove_wuji_teleop.application.encoder_capture.time.monotonic",
                side_effect=(10.0, 11.0),
            ),
        ):
            with self.assertRaisesRegex(RuntimeError, "本机截止时间"):
                read_encoder_window("unused", 5580, 0.1, timeout=0.1)


if __name__ == "__main__":
    unittest.main()
