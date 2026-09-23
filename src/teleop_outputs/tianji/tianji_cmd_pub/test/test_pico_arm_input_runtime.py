import json
import math
import os
import signal
import subprocess
import time
import uuid

from geometry_msgs.msg import Pose, PoseArray
import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String
from tianji_interfaces.msg import PicoArmInput
from pathlib import Path


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
        for index in (16, 17, 18, 19, 20, 21):
            message.poses[index].orientation.z = math.sin(index * 0.01 / 2)
            message.poses[index].orientation.w = math.cos(index * 0.01 / 2)
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


def _drain(node, duration=0.15):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)


def _xyz(vector):
    return vector.x, vector.y, vector.z


def _xyzw(quaternion):
    return quaternion.x, quaternion.y, quaternion.z, quaternion.w


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


def test_corrected_skeleton_is_published_once_as_atomic_ros_frame():
    unique = f"pico_arm_input_{uuid.uuid4().hex}"
    previous_domain = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = "121"
    skeleton_topic = f"/test/{unique}/skeleton"
    status_topic = f"/test/{unique}/status"
    record_flag_topic = f"/test/{unique}/record_flag"
    diagnostics_topic = f"/test/{unique}/diagnostics"

    output_topic = f"/test/{unique}/arm_input"
    process = subprocess.Popen(
        [
            "ros2", "launch", "tianji_cmd_pub", "pico_arm_input.launch.py",
            f"skeleton_topic:={skeleton_topic}",
            f"status_topic:={status_topic}",
            f"record_flag_topic:={record_flag_topic}",
            f"diagnostics_topic:={diagnostics_topic}",
            f"output_topic:={output_topic}",
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
        frames = []
        node.create_subscription(
            PicoArmInput, output_topic, frames.append,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and (
            skeleton_publisher.get_subscription_count() == 0
            or status_publisher.get_subscription_count() == 0
            or record_publisher.get_subscription_count() == 0
            or node.count_publishers(output_topic) == 0
        ):
            rclpy.spin_once(node, timeout_sec=0.05)
        assert skeleton_publisher.get_subscription_count() == 1
        assert status_publisher.get_subscription_count() == 1
        assert record_publisher.get_subscription_count() == 1
        # Graph discovery can precede best-effort data-plane readiness. Confirm
        # actual receipt using an invalid probe that cannot produce a valid target.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not any(
            message["skeleton_messages"] for message in diagnostics
        ):
            skeleton_publisher.publish(_skeleton(1, pose_count=0))
            rclpy.spin_once(node, timeout_sec=0.05)
        assert any(message["skeleton_messages"] for message in diagnostics), diagnostics
        _drain(node)
        assert frames and all(not frame.valid for frame in frames)
        frames.clear()
        before_publication = time.monotonic_ns()

        stamp_ns = 1_000_000_011
        _publish_pair(
            node,
            skeleton_publisher,
            status_publisher,
            _skeleton(stamp_ns),
            _status(stamp_ns),
        )
        _drain(node)
        assert len(frames) == 1
        frame = frames.pop()
        assert frame.valid
        assert frame.source_timestamp_ns == stamp_ns
        assert frame.tracking_epoch == 9
        assert before_publication <= frame.published_monotonic_ns <= time.monotonic_ns()
        assert frame.boot_id == Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        assert str(uuid.UUID(frame.session_id)) == frame.session_id
        first_session = frame.session_id
        first_sequence = frame.sequence
        first_generation = frame.revocation_generation
        assert frame.left_arm_direction_valid and frame.right_arm_direction_valid
        assert frame.upper_limb_valid and frame.upper_limb_rotations_valid
        assert not frame.user_button_pressed
        expected_direction = _expected_retargeted_arm_direction()
        assert _xyz(frame.left_arm_direction) == pytest.approx(expected_direction)
        assert _xyz(frame.right_arm_direction) == pytest.approx(expected_direction)
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
            assert _xyz(frame.upper_limb_points[index]) == pytest.approx(expected_point)
        for rotation, joint in zip(frame.upper_limb_rotations, (16, 18, 20, 22, 17, 19, 21, 23)):
            angle = joint * 0.01 if joint not in (22, 23) else 0.0
            assert _xyzw(rotation) == pytest.approx(
                (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)))
        for pose in (frame.left_target, frame.right_target):
            assert sum(component * component for component in _xyzw(pose.orientation)) == pytest.approx(1.0)
        upper_length = 0.28756390594092296
        forearm_length = 0.31451550041293674
        segment_norm = math.hypot(0.25, 0.30)
        expected_x = 0.095 + 0.95 * (upper_length + forearm_length) * 0.25 / segment_norm
        expected_z = 1.121 + 0.95 * (forearm_length - upper_length) * 0.30 / segment_norm
        assert _xyz(frame.left_target.position) == pytest.approx((expected_x, 0.2115, expected_z))
        assert _xyz(frame.right_target.position) == pytest.approx((expected_x, -0.2115, expected_z))
        _drain(node)
        assert not frames  # Cached skeleton/status must not refresh source time.

        button_stamp = stamp_ns + 100
        _publish_record_flag(node, record_publisher, True)
        _publish_pair(
            node,
            skeleton_publisher,
            status_publisher,
            _skeleton(button_stamp),
            _status(button_stamp),
        )
        assert len(frames) == 1 and frames[0].user_button_pressed
        assert frames[0].sequence > first_sequence
        assert frames[0].session_id == first_session
        frames.clear()

        _publish_record_flag(node, record_publisher, True)
        _publish_pair(
            node,
            skeleton_publisher,
            status_publisher,
            _skeleton(button_stamp + 1),
            _status(button_stamp + 1),
        )
        assert len(frames) == 1 and frames[0].user_button_pressed
        frames.clear()

        _publish_record_flag(node, record_publisher, False)
        _publish_pair(
            node,
            skeleton_publisher,
            status_publisher,
            _skeleton(button_stamp + 2),
            _status(button_stamp + 2),
        )
        assert len(frames) == 1 and not frames[0].user_button_pressed
        frames.clear()

        # Per-side correction flags no longer revoke an otherwise valid IK pair.
        uncorrected_stamp = button_stamp + 3
        _publish_pair(
            node, skeleton_publisher, status_publisher,
            _skeleton(uncorrected_stamp),
            _status(uncorrected_stamp, corrected_left=False, corrected_right=False),
        )
        assert len(frames) == 1 and frames[0].valid
        assert frames[0].source_timestamp_ns == uncorrected_stamp
        assert frames[0].revocation_generation == first_generation
        frames.clear()

        invalid_cases = []
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
            _drain(node)
            assert frames and all(not frame.valid for frame in frames)
            assert all(frame.revocation_generation > first_generation for frame in frames)
            frames.clear()

        # Invalid upstream status revokes even without an accompanying skeleton.
        status_publisher.publish(String(data='{"tracking_epoch":9,"stream_valid":false}'))
        _drain(node)
        assert frames and not frames[-1].valid
        frames.clear()

        # Recovered frames carry the discontinuity even if the invalid message
        # was coalesced by a latest-only subscriber.
        recovered_stamp = stamp_ns + 1000
        _publish_pair(node, skeleton_publisher, status_publisher,
                      _skeleton(recovered_stamp), _status(recovered_stamp))
        assert len(frames) == 1 and frames[0].valid
        assert frames[0].source_timestamp_ns == recovered_stamp
        assert frames[0].session_id == first_session
        assert frames[0].revocation_generation > first_generation
        frames.clear()

        # A status and skeleton with different timestamps cannot form a target.
        status_publisher.publish(_status(recovered_stamp + 10))
        skeleton_publisher.publish(_skeleton(recovered_stamp + 11))
        _drain(node)
        assert not frames
        # Status-after-skeleton pairing remains supported.
        status_publisher.publish(_status(recovered_stamp + 11))
        _drain(node)
        assert len(frames) == 1 and frames[0].valid
        assert frames[0].source_timestamp_ns == recovered_stamp + 11
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
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
