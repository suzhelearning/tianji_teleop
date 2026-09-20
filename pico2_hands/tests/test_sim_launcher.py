from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import unittest
import h5py
import json

from pico2_hands.ik_worker import ROOT
from pico2_hands.tests.test_pico_hand_tracking import _packet


@unittest.skipUnless((ROOT.parent / "build/pico2-v131/pico2_v131_worker").is_file(),
                     "build V131 before integration test")
class SimLauncherTest(unittest.TestCase):
    def test_fake_tcp_raw_recording_and_timed_home_exit(self):
        self._fake_tcp_raw_recording_and_timed_home_exit([])

    @unittest.skipUnless((ROOT.parent / "control/build/pico2_dls_worker").is_file(), "build DLS worker")
    def test_shared_root_fake_tcp_recording_no_automatic_start(self):
        self._fake_tcp_raw_recording_and_timed_home_exit(["--mapping-mode", "shared-root", "--height-m", "1.62"])

    def _fake_tcp_raw_recording_and_timed_home_exit(self, extra_args):
        stop = threading.Event()
        with socket.socket() as server, tempfile.TemporaryDirectory() as folder:
            server.bind(("127.0.0.1", 0))
            server.listen(1)
            server.settimeout(.1)
            port = server.getsockname()[1]

            def serve():
                connection = None
                try:
                    while not stop.is_set():
                        try:
                            connection, _ = server.accept()
                            break
                        except socket.timeout:
                            continue
                    if connection is None:
                        return
                    with connection:
                        i = 0
                        while not stop.is_set():
                            packet = bytearray(_packet())
                            struct.pack_into("<q", packet, 2, i*10)
                            connection.sendall(packet[:29])
                            connection.sendall(packet[29:])
                            i += 1
                            stop.wait(.01)
                except (ConnectionError, OSError):
                    pass

            thread = threading.Thread(target=serve)
            thread.start()
            output = Path(folder)/"session.h5"
            try:
                child = subprocess.run([sys.executable, "-m", "pico2_hands.run_sim", "--headless",
                    "--disable-hands", "--port", str(port), "--duration-s", ".25", "--record", str(output), *extra_args],
                    cwd=ROOT.parent, capture_output=True, text=True, timeout=15)
            finally:
                stop.set(); thread.join(timeout=2)
            self.assertFalse(thread.is_alive())
            self.assertEqual(child.returncode, 0, child.stdout + child.stderr)
            self.assertIn('"home": true', child.stdout)
            if extra_args:
                self.assertIn("hold 1s", child.stdout)
                self.assertIn("Arm IK: Franka DLS + online Ruckig", child.stdout)
                self.assertNotIn("original V131", child.stdout)
                self.assertNotIn("hold 2s", child.stdout)
            else:
                self.assertIn("Arm IK: original V131", child.stdout)
                self.assertIn("hold 2s", child.stdout)
            with h5py.File(output) as file:
                self.assertTrue(file.attrs["complete"])
                self.assertGreater(len(file["raw/pico_hand_tracking/packet"]), 5)
                self.assertGreater(len(file["simulation/position_rad"]), 5)
                self.assertEqual(file.attrs["accepted"], file.attrs["processed"])
                if extra_args:
                    self.assertNotIn(b"teleop", file["simulation/state"][:])
                    events = [json.loads(v) for v in file["events/json"][:]]
                    calibration = next(e["calibration"] for e in events if "calibration" in e)
                    self.assertEqual(calibration["height_m"], 1.62)
                    self.assertEqual(calibration["ik_backend"], "franka_dls_ruckig")
                    self.assertFalse(calibration["calibration_allows_start"])

    def test_real_flag_is_rejected(self):
        child = subprocess.run([sys.executable, "-m", "pico2_hands.run_sim", "--real"],
                               cwd=ROOT.parent, capture_output=True, text=True, timeout=5)
        self.assertEqual(child.returncode, 2)
