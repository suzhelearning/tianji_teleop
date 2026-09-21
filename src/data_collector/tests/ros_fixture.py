#!/usr/bin/env python3
"""Publish the executor and camera inputs the schema-v1 collector consumes.

This is a test fixture, not a production mode: it exists so the collector's DDS
contract can be exercised in separate processes on a test domain. It never
touches hardware, never opens a camera and is not importable by production code.

The pixel payload encodes each frame's source number, so a recorded episode can
be checked against what was actually published.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time
from types import SimpleNamespace

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from realsense2_camera_msgs.msg import Metadata
from realsense2_camera_msgs.srv import DeviceInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from tianji_interfaces.msg import DeviceFeedback, ExecutorState
from data_collector.config import load_collection_config
from tianji_cameras.monitor import CameraMonitor
from tianji_runtime import config_path
from tianji_runtime.device import Feedback
from tianji_controller.safety import DEVICE_READY_FLAGS, MotionGate
from tianji_runtime.constants import (
    ARMS_COUNT,
    CAMERA_FPS,
    CAMERA_ROLES,
    DEVICES,
    HAND_COUNT,
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
    STATE_RATE_HZ,
)

# One packed rgb8 row, so the declared step always matches the payload.
RGB_STEP = IMAGE_WIDTH * 3

# The collector subscribes best-effort; matching it keeps the fixture honest
# about what an unreliable transport can deliver.
INPUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

STAMP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=8,
)

DEVICE_DIM = {"arms": ARMS_COUNT, "left_hand": HAND_COUNT, "right_hand": HAND_COUNT}


def read_boot_id() -> str:
    return Path("/proc/sys/kernel/random/boot_id").read_text().strip()


class InputFixture(Node):
    def __init__(self, arguments: argparse.Namespace):
        super().__init__("collection_fixture")
        self._arguments = arguments
        self._boot_id = read_boot_id()
        self._feedback: dict[str, tuple[int, int]] = {
            device: (0, 0) for device in DEVICES
        }
        self._sequence = 0
        self._state_sequence = 0
        self._frame_number = {role: 0 for role in CAMERA_ROLES}
        self._started = time.monotonic()
        self._control = {}
        self.create_timer(0.02, self._read_control)
        _, self._selection = load_collection_config(arguments.config)
        self.drivers = {role: Node(role, namespace="/cameras") for role in self._selection}
        for role, driver in self.drivers.items():
            serial = self._selection[role]
            driver.declare_parameter("rgb_camera.color_profile", arguments.profile)
            driver.declare_parameter("rgb_camera.color_format", "RGB8")
            driver.declare_parameter("serial_no", f"_{serial}")
            driver.create_service(
                DeviceInfo, f"/cameras/{role}/device_info",
                lambda request, response, name=role: self._device_info(name, request, response))

        self._feedback_publishers = {
            device: self.create_publisher(
                DeviceFeedback, f"/tianji/feedback/{device}", INPUT_QOS)
            for device in DEVICE_DIM
        }
        self._state_publisher = (None if arguments.external_executor else self.create_publisher(
            ExecutorState, "/tianji/executor/state", STAMP_QOS))
        self._image_publishers = {
            role: driver.create_publisher(
                Image, f"/cameras/{role}/color/image_raw", INPUT_QOS)
            for role, driver in self.drivers.items()
        }
        self._metadata_publishers = {
            role: driver.create_publisher(
                Metadata, f"/cameras/{role}/color/metadata", INPUT_QOS)
            for role, driver in self.drivers.items()
        }
        # One fixed buffer: the payload is identical except for the frame
        # number, which is written into the first three bytes of every frame.
        self._buffer = bytearray(IMAGE_HEIGHT * RGB_STEP)
        # Exercise a real software gate, but never instantiate an SDK/device or
        # forward its setpoints. Collector failures must leave this loop running.
        self._gate = MotionGate({
            "rate_hz": 200, "command_timeout_s": 0.3, "feedback_timeout_s": 0.3,
            "arms": {"lower_rad": [-1.0] * ARMS_COUNT, "upper_rad": [1.0] * ARMS_COUNT,
                     "alignment_rad": 0.1, "maximum_speed_rad_s": 1.0,
                     "tracking_error_rad": 0.1},
        }, ("arms",))
        self._gate_steps = 0
        self.create_timer(1.0 / 200, self._step_gate)

        rate = arguments.feedback_hz
        self.create_timer(1.0 / rate, self._publish_feedback)
        self.create_timer(1.0 / CAMERA_FPS, self._publish_images)
        self.create_timer(1.0 / 30.0, self._publish_state)
        self.get_logger().info(
            f"fixture publishing on domain {os.environ.get('ROS_DOMAIN_ID')} "
            f"mode={arguments.mode} phase={arguments.phase}")

    # -------------------------------------------------------------------- inputs

    def _step_gate(self):
        now = time.monotonic_ns()
        positions = (0.0,) * ARMS_COUNT
        frame = SimpleNamespace(timestamp_ns=now, tracking_epoch=1,
                                flags=DEVICE_READY_FLAGS["arms"], positions=lambda _: positions)
        feedback = {"arms": Feedback(positions, now, True, True, "")}
        if not self._gate.armed:
            self._gate.arm(frame, feedback, now)
        else:
            self._gate.step(frame, feedback, now)
            self._gate_steps += 1

    def _elapsed(self) -> float:
        return time.monotonic() - self._started

    def _device_info(self, role, request, response):
        response.serial_number = self._control.get(
            "device_serial", self._arguments.device_serial or self._selection[role])
        response.device_name = "Intel RealSense D435"
        return response

    def _read_control(self):
        if self._arguments.control_file is not None:
            try:
                updated = json.loads(self._arguments.control_file.read_text())
                if updated.get("restart_publisher") and not self._control.get("restart_publisher"):
                    for role, driver in self.drivers.items():
                        driver.destroy_publisher(self._metadata_publishers[role])
                        self._metadata_publishers[role] = driver.create_publisher(
                            Metadata, f"/cameras/{role}/color/metadata", INPUT_QOS)
                if updated.get("reset_frames") and not self._control.get("reset_frames"):
                    self._frame_number = {role: 0 for role in self._selection}
                profile = updated.get("profile", self._arguments.profile)
                if profile != self._control.get("profile", self._arguments.profile):
                    for driver in self.drivers.values():
                        driver.set_parameters([Parameter("rgb_camera.color_profile", value=profile)])
                self._control = updated
            except FileNotFoundError:
                pass

    def _publish_feedback(self) -> None:
        if self._control.get("stop_feedback"):
            return
        if self._arguments.stop_feedback_after is not None and \
                self._elapsed() > self._arguments.stop_feedback_after:
            return
        self._sequence += 1
        for device in DEVICES:
            dim = DEVICE_DIM[device]
            repeat = (
                self._control.get("repeat_hand_token", self._arguments.repeat_hand_token) == device
                and self._feedback[device][0] != 0
            )
            if repeat:
                # Re-send the previous payload: an unchanged source token must
                # not be treated as a new hand sample.
                sequence, source = self._feedback[device]
            else:
                sequence, source = self._sequence, time.monotonic_ns()
                self._feedback[device] = (sequence, source)
            message = DeviceFeedback()
            message.boot_id = self._control.get("boot_id", self._boot_id)
            message.session_id = self._control.get("session_id", self._arguments.session_id)
            message.sequence = sequence
            message.source_monotonic_ns = source - int(self._control.get("source_offset_ns", 0))
            message.published_monotonic_ns = time.monotonic_ns()
            message.device = device
            message.position_rad = [
                float(index) * 0.001 for index in range(dim)]
            message.healthy = True
            message.enabled = True
            message.detail = ""
            self._feedback_publishers[device].publish(message)

    def _publish_state(self) -> None:
        if self._state_publisher is None or self._control.get("stop_state"):
            return
        self._state_sequence += 1
        message = ExecutorState()
        message.boot_id = self._boot_id
        message.session_id = self._control.get("session_id", self._arguments.session_id)
        message.sequence = self._state_sequence
        message.phase_revision = self._arguments.phase_revision
        message.published_monotonic_ns = time.monotonic_ns()
        message.mode = self._control.get("mode", self._arguments.mode)
        message.phase = self._control.get(
            "phase", self._arguments.phase if self._gate.armed else "PREFLIGHT")
        message.faulted = self._arguments.faulted or bool(self._gate.fault)
        message.detail = f"gate_steps={self._gate_steps}"
        self._state_publisher.publish(message)

    def _publish_images(self) -> None:
        if self._control.get("stop_images"):
            return
        if self._arguments.stop_images_after is not None and \
                self._elapsed() > self._arguments.stop_images_after:
            return
        stamp = self.get_clock().now().to_msg()
        stamp.sec -= int(self._control.get("header_age_s", 0))
        for role in self._selection:
            if self._control.get("repeat_frame_number", self._arguments.repeat_frame_number) and self._frame_number[role] > 0:
                number = self._frame_number[role]
            else:
                self._frame_number[role] += 1
                number = self._frame_number[role]
            number = int(self._control.get("frame_number", number))
            self._buffer[0] = number & 0xFF
            self._buffer[1] = (number >> 8) & 0xFF
            self._buffer[2] = 0x5A

            header = Header()
            header.stamp = stamp
            header.frame_id = f"{role}_optical_frame"

            image = Image()
            image.header = header
            image.height = IMAGE_HEIGHT
            image.width = IMAGE_WIDTH
            image.encoding = "rgb8"
            image.is_bigendian = False
            image.step = RGB_STEP
            image.data = bytes(self._buffer)
            self._image_publishers[role].publish(image)

            metadata = Metadata()
            metadata.header = header
            metadata.json_data = json.dumps({
                "frame_number": number,
                "clock_domain": "system_time",
                "frame_timestamp": time.monotonic(),
            })
            self._metadata_publishers[role].publish(metadata)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--external-executor", action="store_true")
    parser.add_argument("--config", type=Path, default=config_path("collect_real.json"))
    parser.add_argument("--profile", default="1280,720,30")
    parser.add_argument("--device-serial", default=None)
    parser.add_argument("--monitor-owner", default="")
    parser.add_argument("--duplicate-monitor", action="store_true")
    parser.add_argument("--monitor-other-config", action="store_true")
    parser.add_argument("--mode", default="real")
    parser.add_argument("--phase", default="TELEOP")
    parser.add_argument("--phase-revision", type=int, default=1)
    parser.add_argument("--faulted", action="store_true")
    parser.add_argument("--feedback-hz", type=float, default=float(STATE_RATE_HZ))
    parser.add_argument("--stop-feedback-after", type=float, default=None)
    parser.add_argument("--stop-images-after", type=float, default=None)
    parser.add_argument("--repeat-hand-token", default=None,
                        help="device whose source token stops advancing")
    parser.add_argument("--repeat-frame-number", action="store_true",
                        help="republish the previous frame number with a new stamp")
    parser.add_argument("--ready-file", type=Path, default=None)
    parser.add_argument("--control-file", type=Path)
    parser.add_argument("--stop-after", type=float, default=None)
    arguments = parser.parse_args(argv)

    rclpy.init()
    node = InputFixture(arguments)
    monitor_config = arguments.config
    if arguments.monitor_other_config:
        monitor_config = arguments.config.with_name("other-collect.json")
        monitor_config.write_bytes(arguments.config.read_bytes())
    monitors = [CameraMonitor(node._selection, config=monitor_config,
                              owner_token=arguments.monitor_owner)]
    if arguments.duplicate_monitor:
        monitors.append(CameraMonitor(node._selection, config=monitor_config))
    executor = SingleThreadedExecutor()
    for participant in [node, *node.drivers.values(), *monitors]:
        executor.add_node(participant)
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, stop)
    if arguments.ready_file is not None:
        arguments.ready_file.write_text("ready\n", encoding="utf-8")
    try:
        deadline = (time.monotonic() + arguments.stop_after
                    if arguments.stop_after is not None else float("inf"))
        while rclpy.ok() and not stopping and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        for monitor in monitors:
            print(monitor.report(), flush=True)
        executor.shutdown()
        for participant in [*monitors, *node.drivers.values()]:
            participant.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
