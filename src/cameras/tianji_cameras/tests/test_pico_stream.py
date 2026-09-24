import io
import signal
import socket
import struct
import tempfile
import threading
import time
import unittest
from contextlib import ExitStack, redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from tianji_cameras import pico_stream
from tianji_cameras.pico_h264 import H264Sender, RgbFrame


def wire(command, payload=b""):
    command = command.encode()
    return (struct.pack(">I", 8 + len(command) + len(payload))
            + struct.pack("<I", len(command)) + command
            + struct.pack("<I", len(payload)) + payload)


OPEN = wire("OPEN_CAMERA", b"\xca\xfe\x01" + struct.pack(
    "<7i", 640, 480, 30, 4_000_000, 0, 9, 12345) + b"\x00\x00")


class FreshFeed:
    set_bridge_ready = pico_stream.TopCameraFeed.set_bridge_ready
    _on_ready = pico_stream.TopCameraFeed._on_ready

    def __init__(self):
        self._lock = threading.Lock()
        self._bridge_ready = False
        self._bridge_error = "not prepared"
        self.error = ""
        self.snapshot()

    def snapshot(self):
        with self._lock:
            self._frame = RgbFrame(1, time.monotonic_ns(), time.time_ns(), b"")
            return self._frame

    def render_video(self, data):
        return data

    def ready(self):
        self.snapshot()
        return self._on_ready(None, SimpleNamespace()).success

    def destroy_node(self):
        pass


class ControlledSender(H264Sender):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.closes = 0
        self.started = threading.Event()

    def start(self):
        self.started.set()

    def close(self, **kwargs):
        self.closes += 1
        super().close(**kwargs)

    def deliver(self):
        with self._lock:
            self._frames_sent += 1

    def stale(self):
        try:
            self._fresh(time.monotonic_ns() - 300_000_000, time.time_ns())
        except TimeoutError as error:
            self._record_error(error)


class StreamBoundaryTest(unittest.TestCase):
    def wait_for(self, condition):
        deadline = time.monotonic() + 3
        while not condition():
            if time.monotonic() >= deadline:
                self.fail("Timed out waiting for bridge state")
            time.sleep(.005)

    def run_bridge(self, client):
        feed = FreshFeed()
        senders = []
        errors = []
        handlers = {}
        output = io.StringIO()
        with socket.socket() as reservation:
            reservation.bind(("127.0.0.1", 0))
            port = reservation.getsockname()[1]

        def sender(*args, **kwargs):
            value = ControlledSender(*args, **kwargs)
            senders.append(value)
            return value

        def install(sig, handler):
            handlers[sig] = handler
            return signal.SIG_DFL

        def run_client():
            try:
                self.wait_for(feed.ready)
                client(feed, senders, port)
            except BaseException as error:
                errors.append(error)
            finally:
                handler = handlers.get(signal.SIGTERM)
                if callable(handler):
                    handler(signal.SIGTERM, None)

        context = Mock()
        context.ok.return_value = True
        executor = Mock()
        executor.spin_once.side_effect = lambda **_: time.sleep(.002)
        with tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
            config = Path(directory) / "config.json"
            config.write_text("{}")
            stack.enter_context(patch("data_collector.config.load_collection_config", return_value=({}, {"top": "test"})))
            stack.enter_context(patch.object(pico_stream, "Context", return_value=context))
            stack.enter_context(patch.object(pico_stream, "TopCameraFeed", return_value=feed))
            stack.enter_context(patch.object(pico_stream, "SingleThreadedExecutor", return_value=executor))
            stack.enter_context(patch.object(pico_stream, "H264Sender", side_effect=sender))
            stack.enter_context(patch.object(pico_stream, "CONTROL_PORT", port))
            stack.enter_context(patch.object(pico_stream.signal, "signal", side_effect=install))
            stack.enter_context(redirect_stdout(output))
            peer = threading.Thread(target=run_client)
            peer.start()
            try:
                result = pico_stream.main(["--config", str(config), "--no-adb", "--duration", "5"])
            finally:
                peer.join(timeout=4)
            self.assertFalse(peer.is_alive())
        if errors:
            raise errors[0]
        return result, output.getvalue(), senders

    def test_stale_stream_closes_once_and_requires_fresh_explicit_reopen(self):
        def client(feed, senders, port):
            with socket.create_connection(("127.0.0.1", port), timeout=3) as control:
                control.sendall(OPEN)
                self.wait_for(lambda: senders and senders[0].started.is_set())
                self.assertFalse(feed.ready())
                first = senders[0]
                first.deliver()
                self.wait_for(feed.ready)
                first.stale()
                self.assertEqual(control.recv(1), b"")
                self.wait_for(lambda: first.closes == 1)
                self.assertFalse(feed.ready())
            with socket.create_connection(("127.0.0.1", port), timeout=3) as control:
                # Reconnecting with fresh input alone cannot recertify a failed session.
                control.sendall(wire("CLOSE_CAMERA"))
                time.sleep(.05)
                self.assertFalse(feed.ready())
                self.assertEqual(len(senders), 1)
                control.sendall(OPEN)
                self.wait_for(lambda: len(senders) == 2 and senders[1].started.is_set())
                self.assertFalse(feed.ready())
                senders[1].deliver()
                self.wait_for(feed.ready)
                control.sendall(wire("CLOSE_CAMERA"))
                self.wait_for(lambda: senders[1].closes == 1)

        result, output, senders = self.run_bridge(client)
        self.assertEqual(result, 0, output)
        self.assertEqual([sender.closes for sender in senders], [1, 1])
        self.assertEqual(output.count("PICO_CAMERA_STREAM_ERROR:"), 1)
        self.assertNotIn("PICO camera error:", output)

    def test_cleanup_failure_remains_fatal_after_reported_stream_failure(self):
        def client(feed, senders, port):
            with socket.create_connection(("127.0.0.1", port), timeout=3) as control:
                control.sendall(OPEN)
                self.wait_for(lambda: senders and senders[0].started.is_set())
                sender = senders[0]
                original_close = sender.close

                def failed_close(**kwargs):
                    sender._record_error(OSError("encoder could not be reaped"), cleanup=True)
                    original_close(**kwargs)

                sender.close = failed_close
                sender.stale()
                self.assertEqual(control.recv(1), b"")
                self.wait_for(lambda: sender.closes == 1)
                self.assertFalse(feed.ready())

        result, output, senders = self.run_bridge(client)
        self.assertEqual(result, 1, output)
        self.assertIn("PICO video cleanup failed", output)
        self.assertEqual(senders[0].closes, 1)
