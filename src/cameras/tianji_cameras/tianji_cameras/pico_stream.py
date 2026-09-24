"""Configured top RGB topic -> H.264 -> PICO over USB; no camera pipeline owner."""
from __future__ import annotations

import argparse
from contextlib import ExitStack
import hashlib
import math
from pathlib import Path
import select
import signal
import socket
import threading
import time

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from tianji_interfaces.msg import ExecutorState

from tianji_runtime import config_path
from tianji_runtime.camera_stream import split_stamp
from tianji_runtime.constants import CAMERA_FPS, IMAGE_HEIGHT, IMAGE_WIDTH
from .pico_h264 import (
    FRESHNESS_NS, FUTURE_TOLERANCE_NS, H264CleanupError, H264Sender,
    H264StreamError, RgbFrame,
)
from .pico_hud import ExecutorHud
from .pico_video_protocol import (
    AdbVideoBridge, CONTROL_PORT, VIDEO_PORT, parse_camera_request, read_message,
)

TOPIC = "/cameras/top/color/image_raw"
READY_SERVICE = "/tianji/pico_camera/check_ready"
IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class TopCameraFeed(Node):
    """One immutable latest frame; no encoding or socket writes in DDS callbacks."""
    def __init__(self, context, *, config=None, config_sha256="", owner_token="", top_serial=""):
        self._hud = ExecutorHud()
        super().__init__("tianji_top_camera_pico", context=context)
        descriptor = ParameterDescriptor(read_only=True)
        for name, value in {
            "config_path": str(Path(config).resolve()) if config is not None else "",
            "config_sha256": config_sha256,
            "owner_token": owner_token,
            "top_serial": top_serial,
        }.items():
            self.declare_parameter(name, value, descriptor, ignore_override=True)
        self._lock = threading.Lock()
        self._frame = None
        self._sequence = 0
        self._bridge_ready = False
        self._bridge_error = "PICO server and ADB bridge are not prepared"
        self.error = "no top RGB frames received"
        self.create_subscription(
            ExecutorState, "/tianji/executor/state", self._hud.state.receive,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=1))
        self.create_subscription(Image, TOPIC, self._on_image, IMAGE_QOS)
        self.create_service(Trigger, READY_SERVICE, self._on_ready)

    def _on_image(self, message):
        received = time.monotonic_ns()
        stamp = split_stamp(message.header.stamp)
        expected = IMAGE_WIDTH * IMAGE_HEIGHT * 3
        if (message.encoding != "rgb8" or len(message.data) != expected or
                (message.width, message.height, message.step) !=
                (IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_WIDTH * 3)):
            with self._lock:
                self.error = f"expected {IMAGE_WIDTH}x{IMAGE_HEIGHT} tightly packed rgb8"
            return
        if stamp <= 0 or not -FUTURE_TOLERANCE_NS <= time.time_ns() - stamp <= FRESHNESS_NS:
            with self._lock:
                self.error = "top image UTC stamp is stale or in the future"
            return
        data = bytes(message.data)
        with self._lock:
            if self._frame is not None and stamp <= self._frame.stamp_ns:
                self.error = "top image stamp did not advance"
                return
            self._sequence += 1
            self._frame = RgbFrame(self._sequence, received, stamp, data)
            self.error = ""

    def snapshot(self):
        with self._lock:
            return self._frame

    def render_video(self, data):
        return self._hud.render(data)

    def set_bridge_ready(self, ready, detail=""):
        """Publish transport preparation only; this does not certify headset display."""
        with self._lock:
            self._bridge_ready = ready
            self._bridge_error = detail or "PICO server and ADB bridge are not prepared"

    def _on_ready(self, request, response):
        with self._lock:
            frame = self._frame
            if not self._bridge_ready:
                detail = self._bridge_error
            elif self.error:
                detail = self.error
            elif frame is None:
                detail = "no top RGB frames received"
            elif not 0 <= time.monotonic_ns() - frame.received_ns <= FRESHNESS_NS:
                detail = "top RGB receipt is stale"
            elif not -FUTURE_TOLERANCE_NS <= time.time_ns() - frame.stamp_ns <= FRESHNESS_NS:
                detail = "top image UTC stamp is stale or in the future"
            else:
                detail = ""
            response.success = not detail
            response.message = detail or "bridge ready; headset display not certified"
        return response


def _bitrate(value):
    try:
        text = value.strip().upper()
        multiplier = {"K": 1_000, "M": 1_000_000}.get(text[-1:], 1)
        number = float(text[:-1] if multiplier != 1 else text) * multiplier
        if not math.isfinite(number) or not 10_000 <= number <= 100_000_000:
            raise ValueError()
        return int(number)
    except ValueError as error:
        raise argparse.ArgumentTypeError("bitrate must be 10K..100M, e.g. 4M") from error


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=None, help="collection config; only the top role is used")
    parser.add_argument("--bitrate", type=_bitrate, default=4_000_000)
    parser.add_argument("--timeout", type=float, default=15.0, help="initial top RGB wait, seconds")
    parser.add_argument("--duration", type=float, default=0.0, help="server lifetime after ready; zero is unlimited")
    parser.add_argument("--no-adb", action="store_true", help="use already-managed USB mappings or a loopback protocol peer")
    parser.add_argument("--owner-token", default="", help="collection supervisor ownership token")
    args = parser.parse_args(argv)
    if (not math.isfinite(args.timeout) or args.timeout <= 0 or
            not math.isfinite(args.duration) or args.duration < 0):
        parser.error("timeout must be positive and duration nonnegative")
    from data_collector.config import load_collection_config

    config = Path(args.config or config_path("collect_real.json")).resolve()
    config_bytes = config.read_bytes()
    _, selection = load_collection_config(config, source_bytes=config_bytes)
    if "top" not in selection:
        parser.error("top camera is disabled in the collection configuration")
    stop = threading.Event()
    spin_stop = threading.Event()
    failures = []
    sender = control = None
    total_frames = 0

    def close_sender(*, stream_error_handled=False):
        nonlocal sender, total_frames
        if sender is not None:
            current, sender = sender, None
            try:
                current.close(stream_error_handled=stream_error_handled)
            finally:
                total_frames += current.frames_sent

    def close_control():
        nonlocal control
        if control is not None:
            current, control = control, None
            current.close()

    def fail_session(error, *, control_error=False):
        feed.set_bridge_ready(False, f"PICO session failed: {error}; explicit OPEN_CAMERA required")
        label = "CONTROL" if control_error else "STREAM"
        print(f"PICO_CAMERA_{label}_ERROR: {error}", flush=True)
        try:
            close_control()
        finally:
            try:
                close_sender(stream_error_handled=isinstance(error, H264StreamError))
            except H264StreamError as stream_error:
                # A control failure may discover an as-yet-unreported worker failure.
                print(f"PICO_CAMERA_STREAM_ERROR: {stream_error}", flush=True)

    try:
        with ExitStack() as cleanup:
            for sig in (signal.SIGINT, signal.SIGTERM):
                previous = signal.signal(sig, lambda *_: stop.set())
                cleanup.callback(signal.signal, sig, previous)
            context = Context()
            context.init(args=[])
            cleanup.callback(context.try_shutdown)
            feed = TopCameraFeed(
                context, config=config, config_sha256=hashlib.sha256(config_bytes).hexdigest(),
                owner_token=args.owner_token, top_serial=selection["top"])
            cleanup.callback(feed.destroy_node)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(feed)
            cleanup.callback(executor.shutdown, timeout_sec=2)

            def spin():
                try:
                    while not spin_stop.is_set() and context.ok():
                        executor.spin_once(timeout_sec=.1)
                except Exception as error:
                    failures.append(error)
                    stop.set()

            thread = threading.Thread(target=spin, name="top-camera-dds", daemon=True)
            thread.start()

            def stop_spin():
                spin_stop.set()
                thread.join(timeout=2)
                if thread.is_alive():
                    raise RuntimeError("top camera subscriber did not stop")
            cleanup.callback(stop_spin)
            deadline = time.monotonic() + args.timeout
            print(f"Waiting for top={selection['top']} on {TOPIC}; start bash bash/run_cameras.sh separately.", flush=True)
            while feed.snapshot() is None and not stop.is_set():
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"No usable top RGB: {feed.error}; camera driver must already be running")
                stop.wait(.05)
            if failures:
                raise failures[0]
            if stop.is_set():
                return 0
            try:
                server = cleanup.enter_context(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind(("127.0.0.1", CONTROL_PORT))
                server.listen(1)
                if not args.no_adb:
                    cleanup.enter_context(AdbVideoBridge())
                feed.set_bridge_ready(True)
                started = last_report = time.monotonic()
                last_sent = 0
                print(f"PICO_CAMERA_READY: top={selection['top']} control=127.0.0.1:{CONTROL_PORT} "
                      f"video=127.0.0.1:{VIDEO_PORT}; bridge prepared, headset display not certified; "
                      "choose PC video source in PICO, PC address 127.0.0.1", flush=True)
                while not stop.is_set():
                    now = time.monotonic()
                    if args.duration and now - started >= args.duration:
                        break
                    if failures:
                        raise failures[0]
                    if sender is not None:
                        try:
                            sender.check()
                        except H264StreamError as error:
                            fail_session(error)
                        else:
                            if sender.frames_sent:
                                feed.set_bridge_ready(True)
                    watched = control if control is not None else server
                    if select.select([watched], [], [], .1)[0]:
                        if control is None:
                            control, address = server.accept()
                            control.settimeout(2.0)
                            print(f"PICO control connected: {address}", flush=True)
                        else:
                            try:
                                command, payload = read_message(control)
                                if command is None:
                                    try:
                                        close_sender()
                                    finally:
                                        close_control()
                                elif command == "OPEN_CAMERA":
                                    request = parse_camera_request(payload)
                                    close_sender()
                                    feed.set_bridge_ready(False, "PICO stream is starting; no fresh frame delivered yet")
                                    sender = H264Sender(feed.snapshot, request.width, request.height,
                                                        min(request.fps, CAMERA_FPS), args.bitrate,
                                                        host="127.0.0.1", port=VIDEO_PORT,
                                                        video_transform=feed.render_video)
                                    sender.start()
                                    last_sent = 0
                                    last_report = time.monotonic()
                                    print(f"PICO_CAMERA_STREAM_STARTING: {request.width}x{request.height} "
                                          f"{min(request.fps, CAMERA_FPS)}fps duplicated top SBS; no recording", flush=True)
                                elif command == "CLOSE_CAMERA":
                                    close_sender()
                                    print("PICO camera stream closed by client", flush=True)
                            except H264CleanupError:
                                raise
                            except H264StreamError as error:
                                fail_session(error)
                            except (OSError, ValueError, RuntimeError) as error:
                                fail_session(error, control_error=True)
                    if sender is not None and now - last_report >= 2:
                        sent = sender.frames_sent
                        print(f"PICO_CAMERA_FRAMES: total={sent} fps={(sent-last_sent)/(now-last_report):.1f}", flush=True)
                        last_sent, last_report = sent, now
                if failures:
                    raise failures[0]
            finally:
                feed.set_bridge_ready(False)
                try:
                    close_control()
                finally:
                    try:
                        close_sender()
                    except H264StreamError as error:
                        print(f"PICO_CAMERA_STREAM_ERROR: {error}", flush=True)
        print(f"PICO_CAMERA_COMPLETE: frames_sent={total_frames}", flush=True)
        return 0
    except Exception as error:
        print(f"PICO camera error: {error}", flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
