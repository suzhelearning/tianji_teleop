import json
import math
import os
import signal
import socket
import struct
import subprocess
import time
import uuid
import zlib

from geometry_msgs.msg import Pose, PoseArray
import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String


PACKET_SIZE = 656


def _skeleton(stamp_ns, frame_id="pico", pose_count=24):
    message = PoseArray()
    message.header.frame_id = frame_id
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    message.poses = [Pose() for _ in range(pose_count)]
    for pose in message.poses:
        pose.orientation.w = 1.0
    if pose_count == 24:
        message.poses[6].position.z = 1.0
        message.poses[16].position.y = 0.2
        message.poses[16].position.z = 1.5
        message.poses[17].position.y = -0.2
        message.poses[17].position.z = 1.5
        message.poses[18].position.x = 0.25
        message.poses[18].position.y = 0.2
        message.poses[18].position.z = 1.2
        message.poses[19].position.x = 0.25
        message.poses[19].position.y = -0.2
        message.poses[19].position.z = 1.2
        message.poses[20].position.x = 0.5
        message.poses[20].position.y = 0.2
        message.poses[20].position.z = 1.5
        message.poses[21].position.x = 0.5
        message.poses[21].position.y = -0.2
        message.poses[21].position.z = 1.5
        message.poses[22].position.x = 0.5
        message.poses[22].position.y = 0.4
        message.poses[22].position.z = 1.4
        message.poses[23].position.x = 0.5
        message.poses[23].position.y = -0.4
        message.poses[23].position.z = 1.4
    return message


def _status(stamp_ns, *, corrected_left=True, corrected_right=True):
    message = String()
    message.data = json.dumps(
        {
            "tracking_epoch": 9,
            "stream_valid": True,
            "source_frame_id": "pico",
            "source_stamp_ns": stamp_ns,
            "ik_frame_valid": True,
            "left": {"corrected": corrected_left},
            "right": {"corrected": corrected_right},
        }
    )
    return message


def _publish_pair(node, skeleton_publisher, status_publisher, skeleton, status):
    for _ in range(3):
        skeleton_publisher.publish(skeleton)
        status_publisher.publish(status)
        for _ in range(4):
            rclpy.spin_once(node, timeout_sec=0.01)


def _expect_no_packet(udp_socket):
    udp_socket.settimeout(0.15)
    with pytest.raises(socket.timeout):
        udp_socket.recvfrom(2048)


def _publish_record_flag(node, publisher, value):
    publisher.publish(Bool(data=value))
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.01)


def _expected_retargeted_arm_direction():
    upper_length = 0.28756390594092296
    forearm_length = 0.31451550041293674
    upper_norm = math.hypot(0.25, 0.30)
    upper = (0.25 / upper_norm, 0.0, -0.30 / upper_norm)
    forearm_norm = math.hypot(0.25, 0.30)
    forearm = (0.25 / forearm_norm, 0.0, 0.30 / forearm_norm)
    axis = tuple(
        upper_length * upper[index] + forearm_length * forearm[index]
        for index in range(3)
    )
    axis_norm = math.sqrt(sum(component * component for component in axis))
    axis = tuple(component / axis_norm for component in axis)
    radial = tuple(
        upper[index] - axis[index] * sum(axis[j] * upper[j] for j in range(3))
        for index in range(3)
    )
    radial_norm = math.sqrt(sum(component * component for component in radial))
    return tuple(component / radial_norm for component in radial)


def test_corrected_skeleton_is_sent_once_as_atomic_udp_packet():
    unique = f"tianji_teleop_{uuid.uuid4().hex}"
    previous_domain = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = str(120 + int(unique[-4:], 16) % 80)
    skeleton_topic = f"/test/{unique}/skeleton"
    status_topic = f"/test/{unique}/status"
    record_flag_topic = f"/test/{unique}/record_flag"
    diagnostics_topic = f"/test/{unique}/diagnostics"

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind(("127.0.0.1", 0))
    destination_port = udp_socket.getsockname()[1]
    process = subprocess.Popen(
        [
            "ros2", "launch", "tianji_cmd_pub", "start_tianji_mujoco_teleop.launch.py",
            f"skeleton_topic:={skeleton_topic}",
            f"status_topic:={status_topic}",
            f"record_flag_topic:={record_flag_topic}",
            f"diagnostics_topic:={diagnostics_topic}",
            "destination_address:=127.0.0.1",
            f"destination_port:={destination_port}",
            "cache_capacity:=8",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
        start_new_session=True,
    )

    node = None
    rclpy.init()
    try:
        node = rclpy.create_node(f"test_{unique}")
        skeleton_publisher = node.create_publisher(
            PoseArray, skeleton_topic, qos_profile_sensor_data
        )
        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        status_publisher = node.create_publisher(String, status_topic, status_qos)
        record_publisher = node.create_publisher(
            Bool, record_flag_topic, status_qos
        )
        diagnostics = []
        node.create_subscription(
            String, diagnostics_topic,
            lambda message: diagnostics.append(json.loads(message.data)), status_qos,
        )
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and (
            skeleton_publisher.get_subscription_count() == 0
            or status_publisher.get_subscription_count() == 0
            or record_publisher.get_subscription_count() == 0
        ):
            rclpy.spin_once(node, timeout_sec=0.05)
        assert skeleton_publisher.get_subscription_count() == 1
        assert status_publisher.get_subscription_count() == 1
        assert record_publisher.get_subscription_count() == 1
        # Graph discovery can precede best-effort data-plane readiness. Confirm
        # actual receipt using an invalid probe that cannot produce a UDP frame.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not any(
            message["skeleton_messages"] for message in diagnostics
        ):
            skeleton_publisher.publish(_skeleton(1, pose_count=0))
            rclpy.spin_once(node, timeout_sec=0.05)
        assert any(message["skeleton_messages"] for message in diagnostics), diagnostics

        stamp_ns = 1_000_000_011
        _publish_pair(
            node,
            skeleton_publisher,
            status_publisher,
            _skeleton(stamp_ns),
            _status(stamp_ns),
        )
        udp_socket.settimeout(1.0)
        packet, _ = udp_socket.recvfrom(2048)
        assert len(packet) == PACKET_SIZE
        assert packet[:4] == b"TJVR"
        assert struct.unpack_from("<H", packet, 4)[0] == 4
        assert struct.unpack_from("<H", packet, 6)[0] == PACKET_SIZE
        assert struct.unpack_from("<Q", packet, 8)[0] == 1
        assert struct.unpack_from("<Q", packet, 16)[0] == 9
        assert struct.unpack_from("<q", packet, 24)[0] == stamp_ns
        assert struct.unpack_from("<I", packet, 40)[0] == 0xFF
        expected_direction = _expected_retargeted_arm_direction()
        assert struct.unpack_from("<3d", packet, 156) == pytest.approx(expected_direction)
        assert struct.unpack_from("<3d", packet, 180) == pytest.approx(expected_direction)
        expected_skeleton = (
            (0.0, 0.2, 1.121),
            (0.25, 0.2, 0.821),
            (0.5, 0.2, 1.121),
            (0.5, 0.4, 1.021),
            (0.0, -0.2, 1.121),
            (0.25, -0.2, 0.821),
            (0.5, -0.2, 1.121),
            (0.5, -0.4, 1.021),
        )
        for index, expected_point in enumerate(expected_skeleton):
            assert struct.unpack_from("<3d", packet, 204 + 24 * index) == pytest.approx(
                expected_point
            )
        for index in range(8):
            quaternion = struct.unpack_from("<4d", packet, 396 + 32 * index)
            assert sum(component * component for component in quaternion) == pytest.approx(1.0)
        assert zlib.crc32(packet[:652]) == struct.unpack_from("<I", packet, 652)[0]
        _expect_no_packet(udp_socket)

        button_stamp = stamp_ns + 100
        _publish_record_flag(node, record_publisher, True)
        _publish_pair(
            node,
            skeleton_publisher,
            status_publisher,
            _skeleton(button_stamp),
            _status(button_stamp),
        )
        packet, _ = udp_socket.recvfrom(2048)
        assert struct.unpack_from("<I", packet, 40)[0] == 0x1FF

        _publish_record_flag(node, record_publisher, True)
        _publish_pair(
            node,
            skeleton_publisher,
            status_publisher,
            _skeleton(button_stamp + 1),
            _status(button_stamp + 1),
        )
        packet, _ = udp_socket.recvfrom(2048)
        assert struct.unpack_from("<I", packet, 40)[0] == 0x1FF

        _publish_record_flag(node, record_publisher, False)
        _publish_pair(
            node,
            skeleton_publisher,
            status_publisher,
            _skeleton(button_stamp + 2),
            _status(button_stamp + 2),
        )
        packet, _ = udp_socket.recvfrom(2048)
        assert struct.unpack_from("<I", packet, 40)[0] == 0xFF

        invalid_cases = []
        uncorrected_stamp = stamp_ns + 1
        invalid_cases.append(
            (_skeleton(uncorrected_stamp), _status(uncorrected_stamp, corrected_left=False))
        )
        wrong_frame_stamp = stamp_ns + 2
        invalid_cases.append(
            (_skeleton(wrong_frame_stamp, frame_id="world"), _status(wrong_frame_stamp))
        )
        wrong_count_stamp = stamp_ns + 3
        invalid_cases.append(
            (_skeleton(wrong_count_stamp, pose_count=23), _status(wrong_count_stamp))
        )
        invalid_cases.append((_skeleton(0), _status(0)))
        non_finite_stamp = stamp_ns + 4
        non_finite = _skeleton(non_finite_stamp)
        non_finite.poses[22].position.x = math.nan
        invalid_cases.append((non_finite, _status(non_finite_stamp)))
        zero_quaternion_stamp = stamp_ns + 5
        zero_quaternion = _skeleton(zero_quaternion_stamp)
        zero_quaternion.poses[22].orientation.w = 0.0
        invalid_cases.append((zero_quaternion, _status(zero_quaternion_stamp)))

        for skeleton, status in invalid_cases:
            _publish_pair(
                node, skeleton_publisher, status_publisher, skeleton, status
            )
            _expect_no_packet(udp_socket)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        udp_socket.close()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=3.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=1.0)
        if previous_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = previous_domain
        assert process.returncode in (0, -signal.SIGINT), output
