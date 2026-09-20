"""Observe startup health without binding TJVR ports or granting motion authority."""
import argparse
import json
import math
import subprocess
import time


class BridgeProgress:
    def __init__(self):
        self.previous = None
        self.ready = False

    def observe(self, payload):
        self.ready = False
        try:
            value = json.loads(payload)
            keys = ("packets_sent", "source_stamp_ns", "tracking_epoch", "send_errors")
            current = tuple(value[key] for key in keys)
            if any(type(x) is not int or x < 0 for x in current):
                raise ValueError("invalid counters")
            old = self.previous
            self.ready = bool(old and current[0] > old[0] and current[1] > old[1]
                              and current[2] == old[2] and current[2] > 0
                              and current[3] == old[3])
            self.previous = current
        except (ValueError, KeyError, TypeError):
            self.previous = None


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
    rclpy.init()
    node = None
    try:
        node = rclpy.create_node("pico_session_startup_check")
        progress = BridgeProgress()
        pane = subprocess.run(["tmux", "display-message", "-p", "-t", session + ":m0", "#{pane_id}"],
                              capture_output=True, text=True, check=True, timeout=2).stdout.strip()
        viewer = ViewerProgress(pane)
        viewer_topic = "/pico/skeleton_viewer/status"
        viewer_subscription = node.create_subscription(String, viewer_topic, lambda msg: viewer.observe(msg.data), 10)
        topic = "/pico/tianji_mujoco_teleop/status"
        subscription = node.create_subscription(String, topic, lambda msg: progress.observe(msg.data), 10)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            check_panes(session, checkout)
            rclpy.spin_once(node, timeout_sec=0.2)
            if progress.ready and viewer.ready(time.monotonic_ns()):
                # Do not interpret interleaved diagnostics from duplicate bridges as progress.
                if node.count_publishers(topic) != 1 or node.count_publishers(viewer_topic) != 1:
                    raise RuntimeError("Expected exactly one PICO bridge status publisher")
                check_panes(session, checkout)
                print("PICO input ready: skeleton viewer rendering; fresh TJVR sends observed (no executor authorization)")
                return
        raise RuntimeError("Timed out waiting for advancing PICO bridge packets and fresh skeleton viewer heartbeat")
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
