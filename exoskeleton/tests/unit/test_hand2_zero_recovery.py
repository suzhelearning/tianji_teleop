"""Hand2 手动恢复的实际关节零位确认合同。"""

from __future__ import annotations

import unittest

from data_glove_wuji_teleop.adapters.hardware.hand2_zero_recovery import (
    validate_hand2_zero_positions,
)


class Hand2ZeroRecoveryTest(unittest.TestCase):
    def test_hand2_zero_confirmation_rejects_nonfinite_feedback(self) -> None:
        self.assertEqual(
            validate_hand2_zero_positions((0.01,) * 20),
            0.01,
        )
        for bad in (float("nan"), float("inf")):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                validate_hand2_zero_positions((0.0,) * 19 + (bad,))
        with self.assertRaises(ValueError):
            validate_hand2_zero_positions((0.0,) * 19 + (0.11,))


if __name__ == "__main__":
    unittest.main()
