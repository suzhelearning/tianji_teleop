#!/usr/bin/env python3
"""Observe live matched ROS arm input and rendering without granting motion authority."""
import argparse
import json
import math
import subprocess
from pathlib import Path
import uuid
import time


class ArmInputProgress:
    def __init__(self, boot_id):
        self.boot_id = boot_id
        self.previous = None
        self.advancing = False
        self.published_ns = 0

    def observe(self, message, now_ns):
        self.advancing = False
        try:
            if (not message.valid or message.boot_id != self.boot_id
                    or str(uuid.UUID(message.session_id)) != message.session_id
                    or message.sequence <= 0 or message.source_timestamp_ns <= 0
                    or message.tracking_epoch <= 0
                    or not 0 <= now_ns - message.published_monotonic_ns < 100_000_000):
                raise ValueError("invalid or stale arm input")
            current = (message.session_id, message.tracking_epoch,
                       message.revocation_generation, message.sequence,
                       message.source_timestamp_ns, message.published_monotonic_ns)
            old = self.previous
            self.advancing = bool(old and current[:3] == old[:3]
                                  and all(current[i] > old[i] for i in (3, 4, 5)))
            self.previous = current
            self.published_ns = message.published_monotonic_ns
        except (ValueError, AttributeError, TypeError):
            self.previous = None
            self.published_ns = 0

    def ready(self, now_ns):
        return self.advancing and 0 <= now_ns - self.published_ns < 100_000_000


class ViewerProgress:
    def __init__(self, pane):
        self.pane = pane
        self.stamp = 0
        self.advancing = False

    def observe(self, payload):
        try:
            value = json.loads(payload)
            stamp = value["stamp_ns"]
            if value["pane"] != self.pane or type(stamp) is not int or stamp <= 0:
                raise ValueError("invalid viewer heartbeat")
            self.advancing = self.stamp > 0 and stamp > self.stamp
            self.stamp = stamp
        except (ValueError, KeyError, TypeError):
            self.stamp = 0
            self.advancing = False

    def ready(self, now_ns):
        return self.advancing and 0 <= now_ns - self.stamp < 1_000_000_000


def check_panes(session, checkout, run=subprocess.run):
    def query(*args):
        result = run(["tmux", *args], capture_output=True, text=True, timeout=2, check=True)
        return result.stdout.strip()
    if query("show-options", "-t", session, "-v", "@tianji_checkout") != checkout:
        raise RuntimeError("PICO session ownership changed")
    rows = query("list-panes", "-s", "-t", session, "-F", "#{window_name}:#{pane_dead}").splitlines()
    if sorted(rows) != ["bridge:0", "driver:0", "m0:0"]:
        raise RuntimeError("PICO window missing or exited: " + ", ".join(rows))


def wait_ready(session, checkout, timeout):
    import rclpy
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from tianji_interfaces.msg import PicoArmInput
    from tianji_runtime.resources import config_path
    rclpy.init()
    node = None
    try:
        node = rclpy.create_node("pico_session_startup_check")
        progress = ArmInputProgress(Path("/proc/sys/kernel/random/boot_id").read_text().strip())
        pane = subprocess.run(["tmux", "display-message", "-p", "-t", session + ":m0", "#{pane_id}"],
                              capture_output=True, text=True, check=True, timeout=2).stdout.strip()
        viewer = ViewerProgress(pane)
        viewer_topic = "/pico/skeleton_viewer/status"
        viewer_subscription = node.create_subscription(String, viewer_topic, lambda msg: viewer.observe(msg.data), 10)
        topic = json.loads(config_path("robot.json").read_text())["pico_input_topic"]
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        subscription = node.create_subscription(
            PicoArmInput, topic,
            lambda msg: progress.observe(msg, time.monotonic_ns()), qos)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            check_panes(session, checkout)
            rclpy.spin_once(node, timeout_sec=0.2)
            if progress.ready(time.monotonic_ns()) and viewer.ready(time.monotonic_ns()):
                # Multiple semantic mappers must not be mistaken for one live stream.
                if node.count_publishers(topic) != 1 or node.count_publishers(viewer_topic) != 1:
                    raise RuntimeError("Expected exactly one PICO arm input and viewer publisher")
                check_panes(session, checkout)
                print("PICO input ready: skeleton viewer rendering; fresh matched ROS arm frames observed (no executor authorization)")
                return
        raise RuntimeError("Timed out waiting for advancing PICO arm input and fresh skeleton viewer heartbeat")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True)
    parser.add_argument("--checkout", required=True)
    parser.add_argument("--timeout-s", type=float, default=30)
    args = parser.parse_args()
    if not math.isfinite(args.timeout_s) or args.timeout_s <= 0:
        parser.error("timeout must be finite and positive")
    try:
        wait_ready(args.session, args.checkout, args.timeout_s)
    except (RuntimeError, subprocess.SubprocessError) as error:
        parser.exit(2, str(error) + "\n")


if __name__ == "__main__":
    main()
