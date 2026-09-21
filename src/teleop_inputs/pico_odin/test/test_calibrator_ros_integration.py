import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool, Float32


def _stamp(seconds):
    whole = int(seconds)
    fraction = int((seconds - whole) * 1_000_000_000)
    return rclpy.time.Time(seconds=whole, nanoseconds=fraction).to_msg()


def _multiply(first, second):
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate(quaternion, vector):
    x, y, z, w = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def _axis_angle(axis, angle):
    half = 0.5 * angle
    sine = math.sin(half)
    return (axis[0] * sine, axis[1] * sine, axis[2] * sine, math.cos(half))


def _messages(index, pitch, yaw, frame_id="pico_ground"):
    pico_rotation = _multiply(
        _axis_angle((0.0, 0.0, 1.0), yaw),
        _axis_angle((0.0, 1.0, 0.0), pitch),
    )
    pico_translation = (0.0, 0.0, 1.02)
    mounting_rotation = _multiply(
        _axis_angle((0.0, 0.0, 1.0), math.pi),
        _axis_angle((0.0, 1.0, 0.0), 0.14),
    )
    mounting_translation = (-0.27, 0.01, 0.09)
    odin_rotation = _multiply(pico_rotation, mounting_rotation)
    rotated_mount = _rotate(pico_rotation, mounting_translation)
    odin_translation = tuple(
        pico_translation[axis] + rotated_mount[axis] for axis in range(3)
    )

    skeleton = PoseArray()
    skeleton.header.stamp = _stamp(500.0 + 0.01 * index)
    skeleton.header.frame_id = frame_id
    pelvis = Pose()
    pelvis.position.x, pelvis.position.y, pelvis.position.z = pico_translation
    (
        pelvis.orientation.x,
        pelvis.orientation.y,
        pelvis.orientation.z,
        pelvis.orientation.w,
    ) = pico_rotation
    skeleton.poses = [pelvis]

    odometry = Odometry()
    odometry.header.stamp = _stamp(9_000.0 + 0.01 * index)
    odometry.header.frame_id = "odin"
    odometry.pose.pose.position.x = odin_translation[0]
    odometry.pose.pose.position.y = odin_translation[1]
    odometry.pose.pose.position.z = odin_translation[2]
    (
        odometry.pose.pose.orientation.x,
        odometry.pose.pose.orientation.y,
        odometry.pose.pose.orientation.z,
        odometry.pose.pose.orientation.w,
    ) = odin_rotation
    return skeleton, odometry


def test_controller_reset_waits_for_ground_relock_then_saves_extrinsics():
    previous_domain_id = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = str(100 + uuid.uuid4().int % 100)
    with tempfile.TemporaryDirectory() as directory:
        output_file = Path(directory) / "extrinsics.yaml"
        executable = (
            Path(get_package_prefix("pico_odin"))
            / "lib" / "pico_odin" / "odin_pelvis_calibrator"
        )
        process = subprocess.Popen(
            [
                str(executable), "--ros-args",
                "-p", f"output_file:={output_file}",
                "-p", "neutral_duration_sec:=0.08",
                "-p", "final_neutral_duration_sec:=0.08",
                "-p", "attempt_timeout_sec:=8.0",
                "-p", "min_pitch_range_rad:=0.20",
                "-p", "min_yaw_range_rad:=0.20",
                "-p", "min_samples:=30",
                "-p", "max_receive_lag_sec:=0.10",
                "-p", "max_interpolation_gap_sec:=0.05",
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env=os.environ.copy(),
        )
        node = None
        rclpy.init()
        try:
            node = rclpy.create_node(f"calibrator_test_{uuid.uuid4().hex}")
            pico_publisher = node.create_publisher(
                PoseArray, "/pico/smpl", qos_profile_sensor_data
            )
            odin_publisher = node.create_publisher(
                Odometry, "/raw/odom/odin_highfreq", qos_profile_sensor_data
            )
            reset_publisher = node.create_publisher(
                Float32, "/pico/world_reset", 10
            )
            ready_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            ready_publisher = node.create_publisher(
                Bool, "/pico/smpl_ground_ready", ready_qos
            )
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and (
                pico_publisher.get_subscription_count() == 0
                or odin_publisher.get_subscription_count() == 0
                or reset_publisher.get_subscription_count() == 0
                or ready_publisher.get_subscription_count() == 0
            ):
                rclpy.spin_once(node, timeout_sec=0.05)
            assert pico_publisher.get_subscription_count() == 1
            assert odin_publisher.get_subscription_count() == 1
            assert reset_publisher.get_subscription_count() == 1
            assert ready_publisher.get_subscription_count() == 1

            frame = 0

            def publish(pitch=0.0, yaw=0.0, frame_id="pico_ground"):
                nonlocal frame
                skeleton, odometry = _messages(frame, pitch, yaw, frame_id)
                pico_publisher.publish(skeleton)
                odin_publisher.publish(odometry)
                rclpy.spin_once(node, timeout_sec=0.002)
                time.sleep(0.008)
                frame += 1

            ready_publisher.publish(Bool(data=True))
            for _ in range(30):
                publish()
            reset_publisher.publish(Float32(data=0.0))
            ready_publisher.publish(Bool(data=False))
            for _ in range(60):
                publish()
            assert not output_file.exists()

            ready_publisher.publish(Bool(data=True))
            for _ in range(20):
                publish(frame_id="pico")
            for index in range(70):
                publish(
                    pitch=0.32 * math.sin(2.0 * math.pi * index / 69.0),
                    frame_id="",
                )
            for index in range(70):
                publish(
                    yaw=0.30 * math.sin(2.0 * math.pi * index / 69.0),
                    frame_id="map",
                )
            for _ in range(30):
                publish(frame_id="pico")
            assert not output_file.exists()

            reset_publisher.publish(Float32(data=0.0))
            for _ in range(5):
                ready_publisher.publish(Bool(data=False))
                rclpy.spin_once(node, timeout_sec=0.02)
                time.sleep(0.01)
            ready_publisher.publish(Bool(data=True))
            for _ in range(20):
                publish()
            for index in range(70):
                publish(pitch=0.32 * math.sin(2.0 * math.pi * index / 69.0))
            for index in range(70):
                publish(yaw=0.30 * math.sin(2.0 * math.pi * index / 69.0))
            for _ in range(30):
                publish()

            deadline = time.monotonic() + 3.0
            while not output_file.exists() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            if not output_file.exists():
                process.send_signal(signal.SIGINT)
                output, _ = process.communicate(timeout=5)
                pytest.fail(f"calibrator did not save extrinsics:\n{output}")
            contents = output_file.read_text(encoding="utf-8")
            assert "schema_version: 3" in contents
            assert "time_offset_sec" not in contents
            assert "transform_convention: T_pelvis_odin" in contents
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if previous_domain_id is None:
                os.environ.pop("ROS_DOMAIN_ID", None)
            else:
                os.environ["ROS_DOMAIN_ID"] = previous_domain_id
