"""仿真与真机共享 canonical hand_dof 目标流测试。"""

from __future__ import annotations

import json
import unittest

from data_glove_wuji_teleop.adapters.transport.zmq_dof import (
    _decode_hand_target,
)


class ZmqTargetTransportTest(unittest.TestCase):
    def test_decodes_canonical_target_with_frame_metadata(self) -> None:
        target = _decode_hand_target(
            json.dumps(
                {
                    "type": "hand_dof",
                    "hand": "left",
                    "values": [0.1] * 20,
                    "source": "dataglove",
                    "sequence": 12,
                    "timestamp_ns": 34,
                    "dropped": 1,
                }
            ),
            expected_hand="left",
        )

        self.assertEqual(target.hand, "left")
        self.assertEqual(target.values, (0.1,) * 20)
        self.assertEqual(target.sequence, 12)
        self.assertEqual(target.timestamp_ns, 34)
        self.assertEqual(target.dropped, 1)

    def test_rejects_wrong_side_or_invalid_joint_values(self) -> None:
        message = {
            "type": "hand_dof",
            "hand": "right",
            "values": [0.0] * 20,
            "sequence": 1,
            "timestamp_ns": 2,
            "dropped": 0,
        }
        with self.assertRaisesRegex(ValueError, "左右手不匹配"):
            _decode_hand_target(
                json.dumps(message),
                expected_hand="left",
            )

        message["hand"] = "left"
        message["values"] = [0.0] * 19
        with self.assertRaisesRegex(ValueError, "必须包含 20 个关节"):
            _decode_hand_target(
                json.dumps(message),
                expected_hand="left",
            )

        message["values"] = [0.0] * 19 + [float("nan")]
        with self.assertRaisesRegex(ValueError, "有限数值"):
            _decode_hand_target(
                json.dumps(message),
                expected_hand="left",
            )


if __name__ == "__main__":
    unittest.main()
