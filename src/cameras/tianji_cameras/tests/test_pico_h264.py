import errno
import shutil
import socket
import struct
import subprocess
import threading
import time
import unittest
from unittest.mock import Mock, patch

from tianji_cameras import pico_h264
from tianji_cameras.pico_h264 import H264CleanupError, H264Sender, H264StreamError, RgbFrame
from tianji_runtime.constants import IMAGE_HEIGHT, IMAGE_WIDTH


class SenderFailureTest(unittest.TestCase):
    def sender(self):
        return H264Sender(lambda: None, 64, 32, 1, 100_000)

    def test_reported_stream_failure_does_not_hide_later_cleanup_failure(self):
        sender = self.sender()
        sender._record_error(TimeoutError("stale source"))
        with self.assertRaisesRegex(H264StreamError, "stale source"):
            sender.check()
        sender.close(stream_error_handled=True)
        sender._record_error(OSError("encoder still running"), cleanup=True)
        with self.assertRaisesRegex(H264CleanupError, "encoder still running"):
            sender.close(stream_error_handled=True)

    def test_normal_close_reports_an_unobserved_stream_failure(self):
        sender = self.sender()
        sender._record_error(ConnectionError("receiver disconnected"))
        with self.assertRaisesRegex(H264StreamError, "receiver disconnected"):
            sender.close()

    def test_worker_join_timeout_is_fatal_even_after_stream_error_is_handled(self):
        sender = self.sender()
        sender._thread = Mock(is_alive=Mock(return_value=True))
        with self.assertRaises(H264CleanupError):
            sender.close(stream_error_handled=True)

    def test_socket_cleanup_failure_survives_stream_error_acknowledgement(self):
        sender = self.sender()
        sock = Mock()
        sock.connect_ex.return_value = errno.ECONNREFUSED
        sock.close.side_effect = OSError("socket release failed")
        with patch.object(pico_h264.shutil, "which", return_value="ffmpeg"), \
             patch.object(pico_h264.socket, "socket", return_value=sock):
            sender._run()
        with self.assertRaisesRegex(H264CleanupError, "socket release failed"):
            sender.close(stream_error_handled=True)

    def test_freshness_enforces_both_clocks_without_relaxing_boundaries(self):
        now = 1_000_000_000
        with patch.object(pico_h264.time, "monotonic_ns", return_value=now), \
             patch.object(pico_h264.time, "time_ns", return_value=now):
            H264Sender._fresh(now - 250_000_000, now - 250_000_000)
            H264Sender._fresh(now, now + 5_000_000)
            for received, stamp in ((now - 250_000_001, now),
                                    (now, now - 250_000_001),
                                    (now, now + 5_000_001),
                                    (now + 1, now)):
                with self.subTest(received=received, stamp=stamp):
                    with self.assertRaises(TimeoutError) as raised:
                        H264Sender._fresh(received, stamp)
                    self.assertIn("receive_age=", str(raised.exception))
                    self.assertIn("stamp_age=", str(raised.exception))


@unittest.skipUnless(shutil.which("ffmpeg"), "FFmpeg is required for real encoder coverage")
class EncoderSocketTest(unittest.TestCase):
    def test_real_packet_decodes_and_stale_source_closes_video_connection(self):
        raw = bytes(IMAGE_WIDTH * IMAGE_HEIGHT * 3)
        stale = threading.Event()
        sequence = 0

        def source():
            nonlocal sequence
            sequence += 1
            received, stamp = time.monotonic_ns(), time.time_ns()
            if stale.is_set():
                received -= 300_000_000
            return RgbFrame(sequence, received, stamp, raw)

        with socket.socket() as server:
            server.bind(("127.0.0.1", 0))
            server.listen(1)
            server.settimeout(4)
            sender = H264Sender(source, 64, 32, 1, 100_000)
            # Isolate tests from the fixed production PICO endpoint, using real TCP.
            sender._port = server.getsockname()[1]
            sender.start()
            try:
                peer, _ = server.accept()
                with peer:
                    peer.settimeout(4)

                    def receive_exact(size):
                        data = bytearray()
                        while len(data) < size:
                            part = peer.recv(size - len(data))
                            if not part:
                                sender.check()
                                self.fail("Sender closed before completing its H264 packet")
                            data.extend(part)
                        return bytes(data)

                    size, = struct.unpack(">I", receive_exact(4))
                    self.assertLessEqual(size, 16 * 1024 * 1024)
                    packet = receive_exact(size)
                    stale.set()
                    deadline = time.monotonic() + 4
                    while True:
                        try:
                            sender.check()
                        except H264StreamError as error:
                            self.assertIn("receive_age=", str(error))
                            break
                        if time.monotonic() >= deadline:
                            self.fail("Stale source did not fail closed")
                        time.sleep(.005)
                    sender.close(stream_error_handled=True)
                    self.assertEqual(peer.recv(1), b"")
                decoded = subprocess.run([
                    "ffmpeg", "-hide_banner", "-loglevel", "error", "-threads", "1",
                    "-f", "h264", "-i", "pipe:0", "-frames:v", "1", "-threads", "1",
                    "-f", "rawvideo", "-pix_fmt", "rgb24", "pipe:1",
                ], input=packet, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    timeout=5, check=True)
                self.assertEqual(len(decoded.stdout), 64 * 32 * 3)
                self.assertLessEqual(max(decoded.stdout), 2)
            finally:
                sender.close(stream_error_handled=True)
