import unittest
from unittest.mock import patch

import numpy as np
import yaml

from pico2_hands.dls_worker import DlsWorker
from pico2_hands.tests.test_shared_root import dls_available
from tianji_runtime.resources import controller_profile


@unittest.skipUnless(dls_available(), "build DLS worker")
class DlsWorkerTest(unittest.TestCase):
    def seeds(self):
        config = yaml.safe_load(controller_profile("qp_ik_pico_shared_root_dls.yaml").read_text())["controller"]
        return np.array([config["initial_left_q_rad"], config["initial_right_q_rad"]])

    def test_unexpected_worker_exit_is_not_hidden_by_close(self):
        worker = DlsWorker()
        worker.process.terminate()
        worker.process.wait(timeout=2)
        with self.assertRaises(RuntimeError):
            worker.close()
        self.assertTrue(worker.failed)

    def test_explicit_reset_start_motion_and_home(self):
        seed = self.seeds()
        with DlsWorker() as worker:
            fk = worker.forward(seed)
            target = np.array([fk[s]["achieved_pose"] for s in ("left", "right")])
            with self.assertRaises(ValueError):
                worker.solve(seed, target, source_time=1., received_time=1., now=1.)
            worker.reset(seed)
            worker.reset(seed)  # Repeated at-rest epoch barriers remain valid.
            self.assertTrue(worker.command(4, seed, 1.)["left"]["accepted"])
            target[:, 0] += .01
            joints = seed.copy()
            for i in range(100):
                stamp = 1.005 + i*.005
                result = worker.solve(joints, target, source_time=stamp, received_time=stamp, now=stamp)
                self.assertTrue(all(result[s]["accepted"] for s in ("left", "right")))
                joints = np.array([result[s]["joints"] for s in ("left", "right")])
            self.assertGreater(np.max(np.abs(joints-seed)), 1e-4)
            worker.command(5, joints, stamp)
            for _ in range(5000):
                stamp += .005
                result = worker.command(7, joints, stamp)
                joints = np.array([result[s]["joints"] for s in ("left", "right")])
                if result["left"]["status"] == "HOME_REACHED":
                    break
            self.assertEqual(result["left"]["status"], "HOME_REACHED")
            np.testing.assert_allclose(joints, seed, atol=1e-6, rtol=0)
        self.assertEqual(worker.process.returncode, 0)

    def test_invalid_geometry_does_not_poison_next_request(self):
        with DlsWorker() as worker:
            seed = self.seeds()
            worker.reset(seed)
            sequence = worker.sequence
            with self.assertRaises(ValueError):
                worker.solve(seed, np.zeros((2, 7)), source_time=1., received_time=1., now=1.)
            self.assertEqual(worker.sequence, sequence)
            self.assertTrue(worker.command(4, seed, 1.)["left"]["accepted"])

    def test_native_clock_rollback_fails_closed(self):
        for source, received, now in ((1., 3., 3.), (2., 1., 1.)):
            with self.subTest(source=source, now=now), DlsWorker() as worker:
                seed = self.seeds()
                fk = worker.reset(seed)
                targets = [fk[s]["achieved_pose"] for s in ("left", "right")]
                worker.command(4, seed, 1.)
                result = worker.solve(seed, targets, source_time=2., received_time=2., now=2.)
                joints = np.array([result[s]["joints"] for s in ("left", "right")])
                with self.assertRaises(RuntimeError):
                    worker.solve(joints, targets, source_time=source, received_time=received, now=now)
                self.assertTrue(worker.failed)
                with self.assertRaises(RuntimeError):
                    worker.reset(joints)

    def test_native_repeated_sequence_fails_closed(self):
        with DlsWorker() as worker:
            seed = self.seeds()
            worker.reset(seed)
            worker.sequence -= 1
            with self.assertRaises(RuntimeError):
                worker.forward(seed)
            self.assertTrue(worker.failed)

    def test_response_association_mismatch_fails_closed(self):
        with DlsWorker() as worker:
            original_read = worker._read

            def wrong_sequence(count, deadline):
                response = original_read(count, deadline)
                response[8] ^= 1
                return response

            with patch.object(worker, "_read", side_effect=wrong_sequence):
                with self.assertRaises(RuntimeError):
                    worker.reset(self.seeds())
            self.assertTrue(worker.failed)

    def test_timeout_latches_client(self):
        with DlsWorker() as worker:
            with patch.object(worker, "_read", side_effect=TimeoutError("injected")):
                with self.assertRaises(TimeoutError):
                    worker.reset(self.seeds())
            self.assertTrue(worker.failed)
            with self.assertRaises(RuntimeError):
                worker.forward(self.seeds())
