"""Read-only three-camera RGB preview.

A pure subscriber: it never opens a RealSense pipeline and never stops one. That
is what makes it safe to run beside the driver and beside a recording — the
driver stays the only owner of the hardware.
"""

from __future__ import annotations

import argparse
import time

import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from tianji_runtime import config_path

from tianji_runtime.constants import IMAGE_HEIGHT, IMAGE_WIDTH

# Matches the driver's SensorDataQoS; requesting reliability here would only
# build a queue of stale frames.
IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

TILE_WIDTH = 640
TILE_HEIGHT = 360
WINDOW = "tianji cameras (subscriber preview)"


class CameraPreview(Node):
    def __init__(self, roles, window: str = WINDOW):
        super().__init__("tianji_camera_preview")
        self._window = window
        self._roles = list(roles)
        self._frames: dict[str, np.ndarray] = {}
        self._counts = {role: 0 for role in self._roles}
        self._last: dict[str, float] = {}
        self._rx_window: dict[str, list[float]] = {role: [] for role in self._roles}
        self._started = time.monotonic()
        self._ui_count = 0
        self._ui_last = self._started
        self._ui_max_interval = 0.0
        for role in self._roles:
            self.create_subscription(
                Image, f"/cameras/{role}/color/image_raw",
                lambda message, name=role: self._on_image(name, message), IMAGE_QOS)

    def _on_image(self, role: str, message: Image) -> None:
        expected = IMAGE_HEIGHT * IMAGE_WIDTH * 3
        if (message.encoding != "rgb8" or len(message.data) != expected
                or (message.width, message.height, message.step)
                != (IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_WIDTH * 3)):
            # Report once per distinct problem; do not attempt to repair it.
            if self._last.get(role) != -1:
                self.get_logger().error(
                    f"{role}: unexpected image {message.encoding} {len(message.data)}B; "
                    f"expected rgb8 {expected}B")
                self._last[role] = -1
            return
        buffer = np.frombuffer(message.data, dtype=np.uint8)
        self._frames[role] = buffer.reshape(IMAGE_HEIGHT, IMAGE_WIDTH, 3)
        self._counts[role] += 1
        now = time.monotonic()
        self._last[role] = now
        window = self._rx_window[role]
        window.append(now)
        cutoff = now - 5.0
        while window and window[0] < cutoff:
            window.pop(0)

    def rx_fps(self, role: str) -> float:
        window = self._rx_window[role]
        if len(window) < 2:
            return 0.0
        span = window[-1] - window[0]
        return (len(window) - 1) / span if span > 0 else 0.0

    def render(self) -> np.ndarray:
        tiles = []
        for role in self._roles:
            frame = self._frames.get(role)
            if frame is None:
                tile = np.zeros((TILE_HEIGHT, TILE_WIDTH, 3), dtype=np.uint8)
                cv.putText(tile, f"{role}: no frame", (16, TILE_HEIGHT // 2),
                           cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv.LINE_AA)
            else:
                tile = cv.resize(frame, (TILE_WIDTH, TILE_HEIGHT), interpolation=cv.INTER_AREA)
                # BGR display order for OpenCV; the recorded buffer stays RGB.
                tile = cv.cvtColor(tile, cv.COLOR_RGB2BGR)
            header = f"{role}  rx={self.rx_fps(role):5.1f}Hz  n={self._counts[role]}"
            cv.putText(tile, header, (10, 26), cv.FONT_HERSHEY_SIMPLEX, 0.6,
                       (0, 255, 0), 1, cv.LINE_AA)
            tiles.append(tile)
        if not tiles:
            return np.zeros((TILE_HEIGHT, TILE_WIDTH, 3), dtype=np.uint8)
        return np.hstack(tiles) if len(tiles) > 1 else tiles[0]


    def displayed(self):
        now = time.monotonic()
        self._ui_count += 1
        self._ui_max_interval = max(self._ui_max_interval, now - self._ui_last)
        self._ui_last = now

    def report(self, requested_fps):
        elapsed = max(time.monotonic() - self._started, 1e-9)
        return (f"preview requested_ui_hz={requested_fps:.2f} "
                f"actual_ui_hz={self._ui_count / elapsed:.2f} "
                f"max_ui_interval_ms={self._ui_max_interval * 1000:.3f}\n"
                + "\n".join(f"{role}: image_rx_hz={self._counts[role] / elapsed:.2f} "
                            f"images={self._counts[role]}" for role in self._roles))

def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="tianji_cameras.preview",
        description="Subscribe to the three camera streams and show them. "
                    "Never opens or stops a camera pipeline.")
    parser.add_argument("--config", default=None)
    parser.add_argument("--fps", type=float, default=30.0, help="UI refresh rate")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="stop after N seconds (0: until closed)")
    args = parser.parse_args(argv)
    if not 0 < args.fps <= 240 or args.duration < 0:
        parser.error("fps must be in (0, 240] and duration must be nonnegative")

    from data_collector.config import load_collection_config
    _, selection = load_collection_config(args.config or config_path("collect_real.json"))

    rclpy.init()
    node = CameraPreview(selection)
    deadline = time.monotonic() + args.duration if args.duration > 0 else None
    next_render = time.monotonic()
    try:
        while rclpy.ok():
            # Drain DDS between UI deadlines; resizing must not run per callback.
            rclpy.spin_once(node, timeout_sec=max(0.0, min(0.02, next_render - time.monotonic())))
            now = time.monotonic()
            if now >= next_render:
                cv.imshow(node._window, node.render())
                node.displayed()
                if cv.waitKey(1) & 0xFF in (27, ord("q")):
                    break
                next_render = max(next_render + 1.0 / args.fps, now)
            if deadline is not None and time.monotonic() >= deadline:
                break
    finally:
        print(node.report(args.fps), flush=True)
        cv.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
