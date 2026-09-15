#!/usr/bin/env python3
"""Call one side-specific PICO palm orientation calibration service."""

from __future__ import annotations

import argparse
import sys


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--timeout-s", type=float, default=8.0)
    arguments, ros_arguments = parser.parse_known_args()

    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    rclpy.init(args=ros_arguments)
    node = Node(f"pico_{arguments.side}_palm_orientation_client")
    service_name = f"/pico/palm_orientation/{arguments.side}/calibrate"
    client = node.create_client(Trigger, service_name)
    exit_code = 2
    try:
        if not client.wait_for_service(timeout_sec=arguments.timeout_s):
            print(f"service unavailable: {service_name}", file=sys.stderr)
        else:
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(
                node, future, timeout_sec=arguments.timeout_s
            )
            if not future.done() or future.result() is None:
                print("orientation calibration service timed out", file=sys.stderr)
            else:
                response = future.result()
                stream = sys.stdout if response.success else sys.stderr
                print(response.message, file=stream)
                exit_code = 0 if response.success else 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
