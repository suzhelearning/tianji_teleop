"""Default-environment ROS input feeding the isolated Manus retargeting worker."""
from __future__ import annotations

import argparse
import os
import select
import subprocess
import time

from tianji_runtime.resources import workspace
from manus_bridge.retarget_worker import FRAME, READY, SIDES, worker_command, worker_environment


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=16000)
    parser.add_argument("--topic", default="/hand_input")
    parser.add_argument("--single-hand-side", choices=SIDES, default="right")
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("--port must be in [1, 65535]")

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float32MultiArray
    # This module's decoder is pure NumPy; its solver imports are lazy and
    # are invoked only inside retarget_worker in the manus environment.
    from example.tj_wuji2_hand_bridge import interpret_hand_input

    worker = subprocess.Popen(
        worker_command(workspace(), "--host", args.host, "--port", str(args.port)),
        env=worker_environment(), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
    )
    node = None
    try:
        # Model construction must finish before consuming ROS observations.
        # Retargeter diagnostics may precede the explicit ready record.
        deadline = time.monotonic() + 60.0
        pending = b""
        while READY not in pending.splitlines(keepends=True):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError("Manus retarget worker startup timed out")
            readable, _, _ = select.select([worker.stdout.fileno()], [], [], remaining)
            if not readable:
                continue
            chunk = os.read(worker.stdout.fileno(), 4096)
            if not chunk:
                raise RuntimeError("Manus retarget worker exited before loading models")
            pending += chunk
        worker.stdout.close()
        os.set_blocking(worker.stdin.fileno(), False)
        rclpy.init(args=[])

        class HandBridge(Node):
            def __init__(self):
                super().__init__("tj_wuji2_hand_bridge")
                qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
                self.subscription = self.create_subscription(
                    Float32MultiArray, args.topic, self.on_hand_input, qos)
                self.health = self.create_timer(0.1, self.check_worker)
                self.get_logger().info(
                    f"Waiting for {args.topic}; isolated Manus Hand2 TJH2 -> "
                    f"{args.host}:{args.port}; 100 Hz bilateral samples; "
                    "per-side timestamps=ROS receive monotonic ns")

            def check_worker(self):
                code = worker.poll()
                if code is not None:
                    raise RuntimeError(f"Manus retarget worker exited ({code})")

            def on_hand_input(self, message):
                received_ns = time.monotonic_ns()
                try:
                    hands = interpret_hand_input(message, args.single_hand_side)
                except (ValueError, TypeError, OverflowError) as error:
                    self.get_logger().warning(f"Rejected /hand_input: {error}", throttle_duration_sec=2.0)
                    return
                self.check_worker()
                for side, points in hands.items():
                    packet = FRAME.pack(SIDES.index(side), received_ns, *points.flat)
                    try:
                        written = os.write(worker.stdin.fileno(), packet)
                    except BlockingIOError:
                        self.get_logger().warning("Manus solver busy; dropped input", throttle_duration_sec=2.0)
                        continue
                    if written != len(packet):
                        raise RuntimeError("partial Manus retarget pipe write")

        node = HandBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        worker.stdin.close()
        worker.stdout.close()
        try:
            worker.wait(timeout=5)
        except subprocess.TimeoutExpired:
            worker.terminate()
            try:
                worker.wait(timeout=5)
            except subprocess.TimeoutExpired:
                worker.kill()
                worker.wait()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
