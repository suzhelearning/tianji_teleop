import json
from pathlib import Path
import sys
import threading
import time

import numpy as np
import yaml


sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))

from pico_palm_orientation_core import OrientationGates, rotation_exp  # noqa: E402
from pico_palm_tcp_publisher import PicoPalmTcpPublisher  # noqa: E402
from pico_palm_tcp_runtime import (  # noqa: E402
    load_tcp_transform,
    matrix_to_quaternion,
)


def _write_tcp(path: Path) -> None:
    path.write_text(
        yaml.safe_dump(
            {
                "artifact_type": "pico_palm_tcp_v2",
                "schema_version": 2,
                "valid": True,
                "side": "left",
                "pose_semantics": "controller_pose",
                "transform_convention": "T_controller_palm",
                "source_topic": "/pico/pose/left_hand",
                "orientation_reference": "hmd_relative_known_palm_pose",
                "translation_m": [0.1, 0.0, 0.0],
                "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                "covariance_upper_triangle_6x6": [0.0] * 21,
                "orientation_calibrated": True,
                "calibration_revision": 4,
                "lineage": [
                    "/pico/pose/left_hand",
                    "/pico/pose/head",
                    "hmd_relative_known_palm_pose",
                ],
                "quality": {
                    "sample_count": 4,
                    "position_rms_m": 0.003,
                    "sample_matrix_rank": 6,
                    "sample_matrix_condition": 12.0,
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )


def test_service_updates_only_orientation_and_hot_swaps_runtime(tmp_path):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from std_msgs.msg import String, UInt64
    from std_srvs.srv import Trigger

    artifact = tmp_path / "tcp.yaml"
    _write_tcp(artifact)
    rclpy.init(domain_id=217)
    publisher = PicoPalmTcpPublisher(
        "left",
        artifact,
        load_tcp_transform(artifact, "left"),
        gates=OrientationGates(min_samples=120),
        capture_timeout_s=2.0,
    )
    source = Node("pico_orientation_test_source")
    controller_pub = source.create_publisher(
        PoseStamped, "/pico/pose/left_hand", qos_profile_sensor_data
    )
    head_pub = source.create_publisher(
        PoseStamped, "/pico/pose/head", qos_profile_sensor_data
    )
    latched_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    epoch_pub = source.create_publisher(UInt64, "/pico/tracking_epoch", latched_qos)
    status_pub = source.create_publisher(
        String, "/pico/tracking_epoch/status", latched_qos
    )
    desired_rotation = rotation_exp(np.array([0.0, 0.0, 0.1]))
    desired = matrix_to_quaternion(desired_rotation)
    pitch_rotation = np.array(
        [
            [np.cos(0.12), 0.0, np.sin(0.12)],
            [0.0, 1.0, 0.0],
            [-np.sin(0.12), 0.0, np.cos(0.12)],
        ]
    )
    head_with_tilt = matrix_to_quaternion(desired_rotation @ pitch_rotation)

    def publish_inputs():
        stamp = source.get_clock().now().to_msg()
        controller = PoseStamped()
        controller.header.frame_id = "pico"
        controller.header.stamp = stamp
        controller.pose.orientation.w = 1.0
        head = PoseStamped()
        head.header.frame_id = "pico"
        head.header.stamp = stamp
        head.pose.orientation.x = float(head_with_tilt[0])
        head.pose.orientation.y = float(head_with_tilt[1])
        head.pose.orientation.z = float(head_with_tilt[2])
        head.pose.orientation.w = float(head_with_tilt[3])
        controller_pub.publish(controller)
        head_pub.publish(head)
        epoch = UInt64()
        epoch.data = 7
        epoch_pub.publish(epoch)
        status = String()
        status.data = json.dumps({"tracking_epoch_source": "tcp_connection"})
        status_pub.publish(status)

    timer = source.create_timer(0.005, publish_inputs)
    client = source.create_client(
        Trigger, "/pico/palm_orientation/left/calibrate"
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(publisher.node)
    executor.add_node(source)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert client.wait_for_service(timeout_sec=2.0)
        time.sleep(0.15)
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 4.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert future.done()
        response = future.result()
        assert response.success, response.message
        updated = yaml.safe_load(artifact.read_text(encoding="utf-8"))
        assert updated["translation_m"] == [0.1, 0.0, 0.0]
        assert updated["translation_revision"] == 4
        assert updated["calibration_revision"] == 5
        assert updated["orientation_reference"] == "gravity_leveled_hmd_heading"
        np.testing.assert_allclose(updated["quaternion_xyzw"], desired, atol=1e-6)
        np.testing.assert_allclose(
            publisher._transform.quaternion_xyzw, desired, atol=1e-6
        )
    finally:
        timer.cancel()
        executor.shutdown()
        thread.join(timeout=2.0)
        source.destroy_node()
        publisher.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
