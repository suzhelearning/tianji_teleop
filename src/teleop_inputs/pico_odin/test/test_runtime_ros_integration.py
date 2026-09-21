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
from rclpy.qos import qos_profile_sensor_data


def _write_identity_extrinsics(path):
    path.write_text(
        """schema_version: 3
valid: true
transform_convention: T_pelvis_odin
quaternion_order: xyzw
calibrated_at: 2026-07-21T12:00:00Z
source_topics:
  pico: /pico/smpl
  odin: /raw/odom/odin_highfreq
translation_m: {x: 0.0, y: 0.0, z: 0.0}
quaternion_xyzw: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
matrix_row_major: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
quality:
  sample_count: 180
  time_correlation: 0.95
  pitch_range_rad: 0.4
  yaw_range_rad: 0.3
  rotation_rms_rad: 0.01
  translation_rms_m: 0.01
  condition_number: 10.0
  refinement_iterations: 4
""",
        encoding="utf-8",
    )


def _stamp(seconds):
    whole = int(seconds)
    fraction = int((seconds - whole) * 1_000_000_000)
    return rclpy.time.Time(seconds=whole, nanoseconds=fraction).to_msg()


def _odometry(stamp, x=0.0):
    message = Odometry()
    message.header.stamp = _stamp(stamp)
    message.header.frame_id = "odin"
    message.pose.pose.position.x = x
    message.pose.pose.orientation.w = 1.0
    return message


def _skeleton(stamp, x_step=0.0, frame_id="pico_ground"):
    message = PoseArray()
    message.header.stamp = _stamp(stamp)
    message.header.frame_id = frame_id
    message.poses = [Pose() for _ in range(24)]
    for index, pose in enumerate(message.poses):
        pose.orientation.w = 1.0
        pose.position.x = index * x_step
        pose.position.z = 1.03
    return message


def _publish_host_bracket(node, odin_publisher, skeleton_publisher,
                          odin_stamp, pico_stamp, skeleton=None):
    """Bracket PICO by host receipt while device timestamp epochs stay unrelated."""
    odin_publisher.publish(_odometry(odin_stamp))
    rclpy.spin_once(node, timeout_sec=0.005)
    skeleton_publisher.publish(skeleton or _skeleton(pico_stamp))
    rclpy.spin_once(node, timeout_sec=0.005)
    odin_publisher.publish(_odometry(odin_stamp + 0.01))
    rclpy.spin_once(node, timeout_sec=0.02)


def test_fused_skeleton_alone_initializes_runtime_at_pico_height():
    unique = f"run_{uuid.uuid4().hex}"
    previous_domain_id = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = str(100 + int(unique[-4:], 16) % 100)
    odin_topic = f"/test/{unique}/odin_hf"
    raw_topic = f"/test/{unique}/raw_unused"
    fused_topic = f"/test/{unique}/fused"
    low_odin_topic = f"/test/{unique}/odin_low"
    raw_output_topic = f"/test/{unique}/raw_odin"
    output_topic = f"/test/{unique}/fused_odin"
    low_output_topic = f"/test/{unique}/pelvis_low"
    with tempfile.TemporaryDirectory() as directory:
        extrinsics = Path(directory) / "extrinsics.yaml"
        _write_identity_extrinsics(extrinsics)
        executable = (
            Path(get_package_prefix("pico_odin"))
            / "lib"
            / "pico_odin"
            / "odin_pelvis_runtime"
        )
        process = subprocess.Popen(
            [
                str(executable),
                "--ros-args",
                "-p", f"extrinsics_file:={extrinsics}",
                "-p", "alignment_samples:=3",
                "-p", "max_sync_gap_sec:=0.10",
                "-p", "max_pending_age_sec:=0.05",
                "-p", "odin_restart_gap_sec:=0.15",
                "-p", f"raw_odin_highfreq_topic:={odin_topic}",
                "-p", f"raw_odin_topic:={low_odin_topic}",
                "-p", f"pico_smpl_topic:={raw_topic}",
                "-p", f"pico_smpl_fused_topic:={fused_topic}",
                "-p", f"smpl_odin_output_topic:={raw_output_topic}",
                "-p", f"smpl_fused_odin_output_topic:={output_topic}",
                "-p", f"pelvis_output_topic:={low_output_topic}",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=os.environ.copy(),
        )
        node = None
        rclpy.init()
        try:
            node = rclpy.create_node(f"pico_odin_runtime_test_{unique}")
            odin_publisher = node.create_publisher(
                Odometry, odin_topic, qos_profile_sensor_data
            )
            low_odin_publisher = node.create_publisher(
                Odometry, low_odin_topic, qos_profile_sensor_data
            )
            raw_skeleton_publisher = node.create_publisher(
                PoseArray, raw_topic, qos_profile_sensor_data
            )
            skeleton_publisher = node.create_publisher(
                PoseArray, fused_topic, qos_profile_sensor_data
            )
            received = []
            raw_received = []
            low_odom_received = []
            output_subscription = node.create_subscription(
                PoseArray,
                output_topic,
                lambda message: received.append(message),
                qos_profile_sensor_data,
            )
            assert output_subscription is not None
            raw_output_subscription = node.create_subscription(
                PoseArray,
                raw_output_topic,
                lambda message: raw_received.append(message),
                qos_profile_sensor_data,
            )
            low_output_subscription = node.create_subscription(
                Odometry,
                low_output_topic,
                lambda message: low_odom_received.append(message),
                qos_profile_sensor_data,
            )
            assert raw_output_subscription is not None
            assert low_output_subscription is not None

            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and (
                odin_publisher.get_subscription_count() == 0
                or skeleton_publisher.get_subscription_count() == 0
                or raw_skeleton_publisher.get_subscription_count() == 0
                or low_odin_publisher.get_subscription_count() == 0
            ):
                rclpy.spin_once(node, timeout_sec=0.05)
            assert odin_publisher.get_subscription_count() == 1
            assert skeleton_publisher.get_subscription_count() == 1
            assert raw_skeleton_publisher.get_subscription_count() == 1
            assert low_odin_publisher.get_subscription_count() == 1

            # Odin and PICO use deliberately unrelated source-clock epochs.
            # Runtime pairing must therefore depend only on local host receipt
            # time, never on cross-device header.stamp subtraction.
            odin_epoch = 9_000.0
            pico_epoch = 500.0
            deadline = time.monotonic() + 3.0
            index = 0
            while not received and time.monotonic() < deadline:
                _publish_host_bracket(
                    node,
                    odin_publisher,
                    skeleton_publisher,
                    odin_epoch + 0.02 * index,
                    pico_epoch + 0.02 * index,
                )
                index += 1
            assert received
            assert len(received[-1].poses) == 24
            assert received[-1].poses[0].position.z == pytest.approx(1.03, abs=1e-6)

            # Once aligned, raw and fused skeletons remain independent. A
            # non-zero Odin displacement must rigidly translate every joint,
            # not just replace the pelvis root.
            received.clear()
            raw_received.clear()
            deadline = time.monotonic() + 1.5
            attempt = 0
            while (not received or not raw_received) and time.monotonic() < deadline:
                stamp_offset = 0.13 + 0.01 * attempt
                moved_raw = _skeleton(
                    pico_epoch + stamp_offset, x_step=0.01
                )
                moved_fused = _skeleton(
                    pico_epoch + stamp_offset, x_step=0.01
                )
                odin_publisher.publish(
                    _odometry(odin_epoch + stamp_offset, x=0.20)
                )
                rclpy.spin_once(node, timeout_sec=0.005)
                raw_skeleton_publisher.publish(moved_raw)
                skeleton_publisher.publish(moved_fused)
                rclpy.spin_once(node, timeout_sec=0.005)
                odin_publisher.publish(
                    _odometry(odin_epoch + stamp_offset + 0.005, x=0.20)
                )
                rclpy.spin_once(node, timeout_sec=0.02)
                attempt += 1
            assert received and raw_received
            for index in range(24):
                expected_x = 0.20 + 0.01 * index
                assert received[-1].poses[index].position.x == pytest.approx(
                    expected_x, abs=1e-5
                )
                assert raw_received[-1].poses[index].position.x == pytest.approx(
                    expected_x, abs=1e-5
                )

            # Drain already-published valid output before testing rejection.
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.01)
            received.clear()

            # Raw, empty, and unrelated frame IDs must never be silently
            # relabelled as ground-aligned output. Compare source stamps so a
            # late valid sample cannot make this asynchronous test fail.
            for index, frame_id in enumerate(("pico", "", "map")):
                offset = 0.145 + 0.005 * index
                wrong_frame = _skeleton(
                    pico_epoch + offset, frame_id=frame_id
                )
                rejected_stamp = (
                    wrong_frame.header.stamp.sec,
                    wrong_frame.header.stamp.nanosec,
                )
                _publish_host_bracket(
                    node,
                    odin_publisher,
                    skeleton_publisher,
                    odin_epoch + offset,
                    pico_epoch + offset,
                    wrong_frame,
                )
                assert rejected_stamp not in {
                    (message.header.stamp.sec, message.header.stamp.nanosec)
                    for message in received
                }

            # Invalid skeletons are rejected before entering either queue.
            received.clear()
            invalid = _skeleton(pico_epoch + 0.15)
            invalid.poses[7].position.x = float("nan")
            _publish_host_bracket(
                node,
                odin_publisher,
                skeleton_publisher,
                odin_epoch + 0.15,
                pico_epoch + 0.15,
                invalid,
            )
            assert not received

            # A PICO frame that waits too long for its future Odin bracket is
            # evicted even while the high-frequency Odin stream is otherwise
            # continuous.
            skeleton_publisher.publish(_skeleton(pico_epoch + 0.17))
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.08)
            odin_publisher.publish(_odometry(odin_epoch + 0.17))
            rclpy.spin_once(node, timeout_sec=0.05)
            assert not received

            # A receive gap represents an Odin reconnect even when its device
            # timestamps remain monotonic. The runtime must stop publishing
            # against the stale session transform, then reacquire from fresh
            # stationary pairs.
            received.clear()
            time.sleep(0.55)
            restart_odin = odin_epoch + 0.20
            restart_pico = pico_epoch + 0.20
            skeleton = _skeleton(restart_pico)
            _publish_host_bracket(
                node,
                odin_publisher,
                skeleton_publisher,
                restart_odin,
                restart_pico,
                skeleton,
            )
            rclpy.spin_once(node, timeout_sec=0.10)
            assert not received

            for index in range(1, 6):
                _publish_host_bracket(
                    node,
                    odin_publisher,
                    skeleton_publisher,
                    restart_odin + 0.02 * index,
                    restart_pico + 0.02 * index,
                    skeleton,
                )

            deadline = time.monotonic() + 2.0
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            assert received
            assert received[-1].poses[0].position.z == pytest.approx(1.03, abs=1e-6)

            # A discontinuity seen only on the low-rate Odin stream must clear
            # the same shared alignment before that bad sample is published.
            low_odom_received.clear()
            low_odin_publisher.publish(_odometry(8_000.0, x=0.1))
            deadline = time.monotonic() + 1.0
            while not low_odom_received and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            assert low_odom_received
            low_odom_received.clear()
            low_odin_publisher.publish(_odometry(7_000.0, x=0.1))
            rclpy.spin_once(node, timeout_sec=0.10)
            assert not low_odom_received

            # Reacquire from the raw skeleton alone after the low-rate reset.
            raw_received.clear()
            for index in range(6):
                _publish_host_bracket(
                    node,
                    odin_publisher,
                    raw_skeleton_publisher,
                    restart_odin + 0.20 + 0.02 * index,
                    restart_pico + 0.20 + 0.02 * index,
                )
            deadline = time.monotonic() + 1.0
            while not raw_received and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            assert raw_received
            assert raw_received[-1].poses[0].position.z == pytest.approx(
                1.03, abs=1e-6
            )
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
