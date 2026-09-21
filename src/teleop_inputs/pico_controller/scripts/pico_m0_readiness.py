#!/usr/bin/env python3
"""Wait for one message from every required PICO M0 runtime stream."""

from __future__ import annotations

import argparse
import os
import time
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any


READINESS_TOPICS = (
    "/pico/smpl_raw",
    "/pico/palm_left",
    "/pico/palm_right",
    "/pico/smpl_palm_corrected",
    "/pico/smpl_palm_corrected_ik",
    "/pico/smpl_palm_corrected/status",
    "/pico/tracking_epoch",
    "/pico/tracking_epoch/status",
)


@dataclass(frozen=True)
class StreamRequirement:
    topic: str
    message_type: type
    qos: Any


class ReadinessTracker:
    def __init__(self, required_topics: Iterable[str]) -> None:
        self._required = tuple(required_topics)
        self._received: set[str] = set()

    def mark_received(self, topic: str) -> None:
        if topic in self._required:
            self._received.add(topic)

    @property
    def missing(self) -> tuple[str, ...]:
        return tuple(topic for topic in self._required if topic not in self._received)

    @property
    def ready(self) -> bool:
        return not self.missing


def wait_for_m0_streams(timeout_s: float) -> tuple[float, tuple[str, ...]]:
    import rclpy
    from geometry_msgs.msg import PoseArray, PoseStamped
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from std_msgs.msg import String, UInt64

    node = None
    subscriptions = []

    try:
        rclpy.init()
        node = Node(f"pico_m0_readiness_{os.getpid()}")
        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        requirements = (
            StreamRequirement(READINESS_TOPICS[0], PoseArray, qos_profile_sensor_data),
            StreamRequirement(READINESS_TOPICS[1], PoseStamped, qos_profile_sensor_data),
            StreamRequirement(READINESS_TOPICS[2], PoseStamped, qos_profile_sensor_data),
            StreamRequirement(READINESS_TOPICS[3], PoseArray, qos_profile_sensor_data),
            StreamRequirement(READINESS_TOPICS[4], PoseArray, qos_profile_sensor_data),
            StreamRequirement(READINESS_TOPICS[5], String, reliable),
            StreamRequirement(READINESS_TOPICS[6], UInt64, transient),
            StreamRequirement(READINESS_TOPICS[7], String, transient),
        )
        tracker = ReadinessTracker(requirement.topic for requirement in requirements)

        for requirement in requirements:
            subscriptions.append(
                node.create_subscription(
                    requirement.message_type,
                    requirement.topic,
                    lambda _message, stream=requirement.topic: tracker.mark_received(
                        stream
                    ),
                    requirement.qos,
                )
            )

        started = time.monotonic()
        deadline = started + timeout_s
        while not tracker.ready:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(node, timeout_sec=min(0.05, remaining))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return time.monotonic() - started, tracker.missing


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-s", type=float, default=20.0)
    arguments = parser.parse_args()
    if arguments.timeout_s < 0.0:
        raise SystemExit("--timeout-s must be non-negative")

    elapsed_s, missing = wait_for_m0_streams(arguments.timeout_s)
    if missing:
        print(
            f"PICO M0 readiness failed after {elapsed_s:.2f}s; "
            f"missing messages: {', '.join(missing)}"
        )
        raise SystemExit(2)
    print(f"PICO M0 streams ready ({elapsed_s:.2f}s)")


if __name__ == "__main__":
    main()
