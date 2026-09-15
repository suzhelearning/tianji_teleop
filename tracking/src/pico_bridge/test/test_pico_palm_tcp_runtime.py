#!/usr/bin/env python3
import math
import sys
from pathlib import Path

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from pico_palm_tcp_runtime import (  # noqa: E402
    TcpTransform,
    apply_tcp_transform,
    load_tcp_transform,
)
from pico_palm_tcp_publisher import build_palm_message  # noqa: E402


def test_tcp_publisher_uses_version_independent_shutdown_handling():
    source = (
        Path(__file__).parents[1]
        / "scripts"
        / "pico_palm_tcp_publisher.py"
    ).read_text(encoding="utf-8")

    assert "from rclpy.exceptions import RCLError" not in source
    assert "except RuntimeError:" in source
    assert "if rclpy.ok():" in source


def _write_artifact(path: Path, side: str = "left") -> None:
    path.write_text(
        "\n".join(
            [
                "schema_version: 2",
                "valid: true",
                f"side: {side}",
                "pose_semantics: controller_pose",
                "transform_convention: T_controller_palm",
                "translation_m: [0.1, 0.0, 0.0]",
                "quaternion_xyzw: [0.0, 0.0, 0.0, 1.0]",
                "orientation_calibrated: true",
            ]
        ),
        encoding="utf-8",
    )


def test_load_tcp_transform_validates_side_and_direction(tmp_path):
    artifact = tmp_path / "left.yaml"
    _write_artifact(artifact)
    transform = load_tcp_transform(artifact, "left")
    np.testing.assert_allclose(transform.translation_m, [0.1, 0.0, 0.0])
    np.testing.assert_allclose(transform.quaternion_xyzw, [0.0, 0.0, 0.0, 1.0])

    with pytest.raises(ValueError, match="side mismatch"):
        load_tcp_transform(artifact, "right")

    document = artifact.read_text(encoding="utf-8").replace(
        "T_controller_palm", "T_palm_controller"
    )
    artifact.write_text(document, encoding="utf-8")
    with pytest.raises(ValueError, match="transform_convention"):
        load_tcp_transform(artifact, "left")


def test_load_tcp_transform_rejects_missing_invalid_and_zero_quaternion(tmp_path):
    with pytest.raises(ValueError, match="does not exist"):
        load_tcp_transform(tmp_path / "missing.yaml", "left")

    artifact = tmp_path / "left.yaml"
    _write_artifact(artifact)
    artifact.write_text(
        artifact.read_text(encoding="utf-8").replace(
            "[0.0, 0.0, 0.0, 1.0]", "[0.0, 0.0, 0.0, 0.0]"
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="quaternion"):
        load_tcp_transform(artifact, "left")


def test_apply_tcp_transform_rotates_translation_and_orientation():
    half = math.sqrt(0.5)
    controller_quaternion = np.array([0.0, 0.0, half, half])
    transform = TcpTransform(
        translation_m=np.array([0.1, 0.0, 0.0]),
        quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    palm_position, palm_quaternion = apply_tcp_transform(
        np.array([1.0, 2.0, 3.0]), controller_quaternion, transform
    )
    np.testing.assert_allclose(palm_position, [1.0, 2.1, 3.0], atol=1e-9)
    np.testing.assert_allclose(palm_quaternion, controller_quaternion, atol=1e-9)


def test_apply_tcp_transform_rejects_nonfinite_controller_pose():
    transform = TcpTransform(
        translation_m=np.zeros(3),
        quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    with pytest.raises(ValueError, match="controller_position"):
        apply_tcp_transform(
            np.array([np.nan, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0, 1.0]),
            transform,
        )


def test_build_palm_message_preserves_source_stamp_and_pico_frame():
    controller = PoseStamped()
    controller.header.frame_id = "pico"
    controller.header.stamp.sec = 12
    controller.header.stamp.nanosec = 345
    controller.pose.position.x = 1.0
    controller.pose.position.y = 2.0
    controller.pose.position.z = 3.0
    controller.pose.orientation.w = 1.0
    transform = TcpTransform(
        translation_m=np.array([0.1, 0.0, 0.0]),
        quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    palm = build_palm_message(controller, transform, PoseStamped)
    assert palm.header.stamp.sec == 12
    assert palm.header.stamp.nanosec == 345
    assert palm.header.frame_id == "pico"
    assert palm.pose.position.x == pytest.approx(1.1)
    assert palm.pose.orientation.w == pytest.approx(1.0)


def test_build_palm_message_rejects_non_pico_controller_frame():
    controller = PoseStamped()
    controller.header.frame_id = "pico_ground"
    controller.pose.orientation.w = 1.0
    transform = TcpTransform(
        translation_m=np.zeros(3),
        quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    with pytest.raises(ValueError, match="frame"):
        build_palm_message(controller, transform, PoseStamped)
