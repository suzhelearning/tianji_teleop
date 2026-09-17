from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import unittest
import h5py

from pico2_hands.ik_worker import ROOT
from pico2_hands.tests.test_pico_hand_tracking import _packet


@unittest.skipUnless((ROOT.parent / "build/pico2-v131/pico2_v131_worker").is_file(),
                     "build V131 before integration test")
class SimLauncherTest(unittest.TestCase):
    def test_fake_tcp_raw_recording_and_timed_home_exit(self):
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
                    "--disable-hands", "--port", str(port), "--duration-s", ".25", "--record", str(output)],
                    cwd=ROOT.parent, capture_output=True, text=True, timeout=15)
            finally:
                stop.set(); thread.join(timeout=2)
            self.assertFalse(thread.is_alive())
            self.assertEqual(child.returncode, 0, child.stdout + child.stderr)
            self.assertIn('"home": true', child.stdout)
            with h5py.File(output) as file:
                self.assertTrue(file.attrs["complete"])
                self.assertGreater(len(file["raw/pico_hand_tracking/packet"]), 5)
                self.assertGreater(len(file["simulation/position_rad"]), 5)
                self.assertEqual(file.attrs["accepted"], file.attrs["processed"])

    def test_real_flag_is_rejected(self):
        child = subprocess.run([sys.executable, "-m", "pico2_hands.run_sim", "--real"],
                               cwd=ROOT.parent, capture_output=True, text=True, timeout=5)
        self.assertEqual(child.returncode, 2)
