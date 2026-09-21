from pathlib import Path
import tempfile
import unittest
import h5py
import numpy as np
from pico2_hands.sim_recording import SimRecorder
from pico2_hands.tests import test_simulation_core


class RecordingTest(unittest.TestCase):
    def test_exclusive_recording_and_drain(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"session.h5"
            record = SimRecorder(path)
            frame = test_simulation_core.SimulationCoreTest().frame(1_000_000_000)
            record.offer("raw", frame)
            record.offer("command", (1_000_000_000, "idle", np.zeros(54), (1, 1)))
            record.close(True)
            with self.assertRaises(OSError):
                SimRecorder(path)
            with h5py.File(path) as file:
                self.assertTrue(file.attrs["complete"])
                self.assertEqual(file["raw/pico_hand_tracking/packet"][0].tobytes(), frame.raw_packet)
                self.assertEqual(file["simulation/position_rad"].dtype, np.dtype("float64"))
                self.assertEqual(file.attrs["processed"], 2)

    def test_aborted_recording_stays_incomplete(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"session.h5"
            record = SimRecorder(path)
            record.close(False)
            with h5py.File(path) as file:
                self.assertFalse(file.attrs["complete"])
