import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

from geometry_msgs.msg import Pose, PoseArray
import pytest
import rclpy
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool, Float32


REPOSITORY = Path(__file__).parents[3]


def _skeleton(sole_height=-1.6):
    message = PoseArray()
    message.header.frame_id = "pico"
    message.poses = [Pose() for _ in range(24)]
    for pose in message.poses:
        pose.orientation.w = 1.0
    message.poses[0].position.x = 0.4
    message.poses[0].position.y = -0.2
    message.poses[0].position.z = -0.8
    message.poses[10].position.z = sole_height + 0.022
    message.poses[11].position.z = sole_height + 0.022
    message.poses[15].position.z = 0.0
    return message


def _spin_publish(node, publisher, message, count):
    for _ in range(count):
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.01)


def test_ros_runtime_requires_a_then_locks_and_resets_ground():
    unique = f"ground_{uuid.uuid4().hex}"
    previous_domain = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = str(120 + int(unique[-4:], 16) % 80)
    raw_topic = f"/test/{unique}/raw"
    output_topic = f"/test/{unique}/canonical"
    reset_topic = f"/test/{unique}/reset"
    ready_topic = f"/test/{unique}/ground_ready"
    executable = (
        REPOSITORY / "install/pico_bridge/lib/pico_bridge/pico_smpl_ground"
    )
    assert executable.exists(), "build pico_bridge before running this integration test"
    process = subprocess.Popen(
        [
            str(executable),
            "--ros-args",
            "-p", f"raw_topic:={raw_topic}",
            "-p", f"output_topic:={output_topic}",
            "-p", f"world_reset_topic:={reset_topic}",
            "-p", f"ground_ready_topic:={ready_topic}",
            "-p", "stable_window_frames:=30",
            "-p", "require_world_reset:=true",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
    )
    node = None
    rclpy.init()
    try:
        node = rclpy.create_node(f"test_{unique}")
        raw_publisher = node.create_publisher(
            PoseArray, raw_topic, qos_profile_sensor_data
        )
        reset_publisher = node.create_publisher(Float32, reset_topic, 10)
        received = []
        ready_states = []
        subscription = node.create_subscription(
            PoseArray,
            output_topic,
            lambda message: received.append(message),
            qos_profile_sensor_data,
        )
        ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        ready_subscription = node.create_subscription(
            Bool,
            ready_topic,
            lambda message: ready_states.append(message.data),
            ready_qos,
        )
        assert subscription is not None
        assert ready_subscription is not None
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and (
            raw_publisher.get_subscription_count() == 0
            or reset_publisher.get_subscription_count() == 0
        ):
            rclpy.spin_once(node, timeout_sec=0.05)
        assert raw_publisher.get_subscription_count() == 1
        assert reset_publisher.get_subscription_count() == 1
        deadline = time.monotonic() + 1.0
        while not ready_states and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert ready_states == [False]

        raw = _skeleton()
        _spin_publish(node, raw_publisher, raw, 35)
        assert not received

        reset_publisher.publish(Float32(data=0.0))
        rclpy.spin_once(node, timeout_sec=0.05)
        assert ready_states[-1] is False
        _spin_publish(node, raw_publisher, raw, 29)
        assert not received
        assert ready_states[-1] is False
        _spin_publish(node, raw_publisher, raw, 1)
        deadline = time.monotonic() + 1.0
        while (
            (not received or ready_states[-1] is not True)
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.05)
        assert received
        assert ready_states[-1] is True
        canonical = received[-1]
        assert canonical.header.frame_id == "pico_ground"
        assert canonical.poses[0].position.x == pytest.approx(0.4)
        assert canonical.poses[0].position.y == pytest.approx(-0.2)
        assert canonical.poses[0].position.z == pytest.approx(0.8, abs=1e-9)
        assert canonical.poses[10].position.z == pytest.approx(0.022, abs=1e-9)
        assert canonical.poses[11].position.z == pytest.approx(0.022, abs=1e-9)
        assert canonical.poses[15].position.z == pytest.approx(1.6, abs=1e-9)
        assert canonical.poses[0].orientation.w == raw.poses[0].orientation.w

        received.clear()
        reset_publisher.publish(Float32(data=1.0))
        rclpy.spin_once(node, timeout_sec=0.05)
        assert ready_states[-1] is False
        _spin_publish(node, raw_publisher, raw, 5)
        assert not received
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate(timeout=1.0)
        if previous_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = previous_domain
        assert process.returncode in (0, -signal.SIGINT), output
