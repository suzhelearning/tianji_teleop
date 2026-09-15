import importlib.util
import unittest
from pathlib import Path

import numpy as np
from scipy import signal


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/analyze_cartesian_frf.py"
SPEC = importlib.util.spec_from_file_location("analyze_cartesian_frf", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FrfEstimatorTests(unittest.TestCase):
    def setUp(self):
        self.fs = 200.0
        rng = np.random.default_rng(7)
        self.u = rng.normal(size=65536)

    def test_recovers_first_order_cutoff(self):
        cutoff = 5.0
        b, a = signal.butter(1, cutoff, fs=self.fs)
        y = signal.lfilter(b, a, self.u)
        estimate = MODULE.estimate_frf(self.u, y, self.fs, 4096)
        bandwidth = MODULE.minus_3db_bandwidth(estimate)
        self.assertAlmostEqual(bandwidth, cutoff, delta=0.7)

    def test_recovers_pure_delay(self):
        delay_samples = 4
        y = np.r_[np.zeros(delay_samples), self.u[:-delay_samples]]
        estimate = MODULE.estimate_frf(self.u, y, self.fs, 4096)
        mask = ((estimate.frequency >= 1.0) & (estimate.frequency <= 15.0)
                & (estimate.coherence >= 0.8))
        self.assertAlmostEqual(np.median(estimate.group_delay[mask]),
                               delay_samples / self.fs, delta=0.002)

    def test_independent_noise_has_low_coherence(self):
        y = np.random.default_rng(11).normal(size=self.u.size)
        estimate = MODULE.estimate_frf(self.u, y, self.fs, 4096)
        mask = (estimate.frequency >= 1.0) & (estimate.frequency <= 15.0)
        self.assertLess(np.median(estimate.coherence[mask]), 0.1)


if __name__ == "__main__":
    unittest.main()
