from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from pico2_hands.hand_worker import NativeHandWorker, ROOT


@unittest.skipUnless((ROOT.parent / "build/pico2-hand/tianji_hand_native_worker").is_file(),
                     "build isolated native Hand2 worker first")
class NativeHandWorkerTest(unittest.TestCase):
    def points(self):
        return np.array([[0., 0., 0.]] + [
            [(finger - 2) * .02, .025 * (joint + 1), .001 * joint * joint]
            for finger in range(5) for joint in range(4)])

    def test_both_sides_protocol_and_clean_close(self):
        for side in ("left", "right"):
            with NativeHandWorker(side) as worker:
                result = worker.retarget(self.points(), 1, 1_000_000)
                self.assertEqual(result.shape, (20,))
                self.assertTrue(np.isfinite(result).all())
                with self.assertRaises(ValueError):
                    worker.retarget(self.points(), 1, 1_000_001)
                self.assertEqual(worker.sequence, 1)
            self.assertEqual(worker.process.returncode, 0)

    def test_transport_failure_latches_until_explicit_close(self):
        with NativeHandWorker("left") as worker:
            with patch.object(worker, "_read", side_effect=TimeoutError("injected")):
                with self.assertRaises(TimeoutError):
                    worker.retarget(self.points(), 1, 1_000_000)
            self.assertTrue(worker.failed)
            with self.assertRaises(RuntimeError):
                worker.retarget(self.points(), 2, 2_000_000)

    def test_wrong_response_is_not_published(self):
        from pico2_hands.hand_worker import RESPONSE
        with NativeHandWorker("right") as worker:
            bad = RESPONSE.pack(b"TJHR", 1, 1, RESPONSE.size, 1, 1_000_000, *([0.] * 40))
            with patch.object(worker, "_read", return_value=bad):
                with self.assertRaises(RuntimeError):
                    worker.retarget(self.points(), 1, 1_000_000)
            self.assertTrue(worker.failed)
