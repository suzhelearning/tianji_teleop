#!/usr/bin/env python3
from collections import deque
import hashlib
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_msgs.msg import String, UInt64

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from pico_palm_skeleton_filter_node import (  # noqa: E402
    LoadedArmGeometry,
    PalmSample,
    PicoPalmSkeletonFilterNode,
    SIDE_INDICES,
    load_arm_geometry_artifact,
    load_left_arm_geometry_artifact,
    load_tcp_calibration_revision,
    load_wrist_pivot_artifact,
    pose_array_with_ik_arm_frames,
    pose_array_to_arrays,
    pose_stamped_to_sample,
)
from pico_palm_orientation_core import translation_fingerprint  # noqa: E402
from pico_palm_skeleton_filter_core import quat_apply  # noqa: E402


def _pose_array(frame_id="pico"):
    message = PoseArray()
    message.header.frame_id = frame_id
    message.header.stamp.sec = 1
    message.header.stamp.nanosec = 0
    message.poses = [Pose() for _ in range(24)]
    for pose in message.poses:
        pose.orientation.w = 1.0
    return message


def _palm(stamp_ns=1_000_000_000, frame_id="pico"):
    message = PoseStamped()
    message.header.frame_id = frame_id
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    message.pose.orientation.w = 1.0
    message.pose.position.x = 0.5
    return message


class _RecordingPublisher:
    def __init__(self, *, label=None, events=None):
        self.messages = []
        self._label = label
        self._events = events

    def publish(self, message):
        self.messages.append(message)
        if self._events is not None:
            self._events.append(self._label)


def _runtime_node_for_raw_callback():
    node = object.__new__(PicoPalmSkeletonFilterNode)
    node._output_publisher = _RecordingPublisher()
    node._ik_output_publisher = _RecordingPublisher()
    node._left_shoulder_local_x_offset_rad = np.pi / 2.0
    node._right_shoulder_local_x_offset_rad = -np.pi / 2.0
    node._previous_ik_orientations_xyzw = None
    node._ik_frame_valid = True
    node._ik_frame_failure_reason = ""
    node._process_side = lambda *args, **kwargs: None
    node._published_status_stamps = []
    node._publish_status = node._published_status_stamps.append
    node._warnings = []
    node.get_logger = lambda: SimpleNamespace(warning=node._warnings.append)
    return node


def _runtime_pose_array():
    message = _pose_array()
    positions = {
        16: (0.0, 0.25, 1.0),
        18: (0.0, 0.25, 0.7),
        20: (0.25, 0.25, 0.7),
        17: (0.0, -0.25, 1.0),
        19: (0.0, -0.25, 0.7),
        21: (-0.25, -0.25, 0.7),
    }
    for index, value in positions.items():
        message.poses[index].position.x = value[0]
        message.poses[index].position.y = value[1]
        message.poses[index].position.z = value[2]
    return message


def _valid_left_geometry_document():
    return {
        "artifact_type": "pico_left_arm_geometry_quick_v3",
        "schema_version": 3,
        "valid": True,
        "candidate_status": "accepted",
        "side": "left",
        "calibration_revision": 3,
        "upper_arm_length_m": 0.302,
        "forearm_length_m": 0.251,
        "covariance_tangent_order": [
            "shoulder_x_m",
            "shoulder_y_m",
            "shoulder_z_m",
            "elbow_x_m",
            "elbow_y_m",
            "elbow_z_m",
            "upper_arm_length_m",
            "forearm_length_m",
        ],
        "tcp_calibration_revision": 8,
        "tcp_artifact_sha256": "tcp123",
        "wrist_pivot_sha256": "abc123",
        "tracking_epoch": 4,
        "tracking_epoch_source": "tcp_connection",
        "lineage": ["/pico/smpl_raw", "/pico/palm_left"],
        "rejection_reasons": [],
        "quality": {
            "length_std_m": 0.003,
            "selected_straight_groups": ["straight_1", "validation"],
            "discarded_straight_group": "straight_2",
            "straight_pair_position_error_m": 0.03,
            "upper_repeat_error_m": 0.01,
            "neutral_pair_position_error_m": 0.02,
            "forearm_repeat_error_m": 0.01,
            "static_motion_rms_m": 0.006,
            "right_angle_error_rad": 0.05,
            "right_angle_error_role": "diagnostic_only",
            "raw_smpl_diagnostic_available": True,
            "transition_direction_error_rad": 0.05,
            "length_closure_error_m": 0.005,
            "shoulder_motion_rms_m": 0.005,
            "straight_1_sample_count": 100,
            "straight_2_sample_count": 100,
            "validation_sample_count": 100,
            "neutral_start_sample_count": 100,
            "neutral_return_sample_count": 100,
            "right_angle_sample_count": 150,
        },
        "gate_results": {"all_geometry_gates": True},
        "covariance_upper_triangle_8x8": [0.0] * 36,
    }


def _valid_geometry_document(side: str):
    document = _valid_left_geometry_document()
    document["artifact_type"] = f"pico_{side}_arm_geometry_quick_v3"
    document["side"] = side
    document["lineage"] = ["/pico/smpl_raw", f"/pico/palm_{side}"]
    return document


def test_pose_array_conversion_requires_24_pico_poses():
    message = _pose_array()
    positions, orientations = pose_array_to_arrays(message)
    assert positions.shape == (24, 3)
    assert orientations.shape == (24, 4)
    invalid_count = PoseArray()
    invalid_count.header.frame_id = "pico"
    with pytest.raises(ValueError, match="24"):
        pose_array_to_arrays(invalid_count)
    with pytest.raises(ValueError, match="pico"):
        pose_array_to_arrays(_pose_array(frame_id="pico_ground"))


def test_ik_frame_pose_array_preserves_positions_and_adapts_shoulders_and_elbows():
    message = _pose_array()
    for index, pose in enumerate(message.poses):
        pose.position.x = float(index)
        pose.position.y = float(-index)
        pose.position.z = float(index) * 0.1

    adapted = pose_array_with_ik_arm_frames(
        message,
        left_offset_rad=np.pi / 2.0,
        right_offset_rad=-np.pi / 2.0,
    )

    assert adapted is not message
    assert adapted.header == message.header
    for source, output in zip(message.poses, adapted.poses):
        assert output.position == source.position
    np.testing.assert_allclose(
        [
            adapted.poses[16].orientation.x,
            adapted.poses[16].orientation.y,
            adapted.poses[16].orientation.z,
            adapted.poses[16].orientation.w,
        ],
        [np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)],
        atol=1.0e-9,
    )
    np.testing.assert_allclose(
        [
            adapted.poses[17].orientation.x,
            adapted.poses[17].orientation.y,
            adapted.poses[17].orientation.z,
            adapted.poses[17].orientation.w,
        ],
        [-np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)],
        atol=1.0e-9,
    )
    for elbow, wrist in ((18, 20), (19, 21)):
        quaternion = np.array(
            [
                adapted.poses[elbow].orientation.x,
                adapted.poses[elbow].orientation.y,
                adapted.poses[elbow].orientation.z,
                adapted.poses[elbow].orientation.w,
            ]
        )
        forearm = np.array(
            [
                message.poses[wrist].position.x - message.poses[elbow].position.x,
                message.poses[wrist].position.y - message.poses[elbow].position.y,
                message.poses[wrist].position.z - message.poses[elbow].position.z,
            ]
        )
        forearm /= np.linalg.norm(forearm)
        np.testing.assert_allclose(
            quat_apply(quaternion, [1.0, 0.0, 0.0]),
            forearm,
            atol=1.0e-9,
        )
    for index in set(range(24)) - {16, 17, 18, 19}:
        assert adapted.poses[index].orientation == message.poses[index].orientation
    assert message.poses[16].orientation.w == 1.0
    assert message.poses[16].orientation.x == 0.0


def test_raw_callback_publishes_original_and_position_identical_ik_copy():
    node = _runtime_node_for_raw_callback()
    events = []
    node._output_publisher = _RecordingPublisher(label="corrected", events=events)
    node._ik_output_publisher = _RecordingPublisher(label="ik", events=events)
    message = _runtime_pose_array()

    node._raw_callback(message)

    assert len(node._output_publisher.messages) == 1
    assert len(node._ik_output_publisher.messages) == 1
    original = node._output_publisher.messages[0]
    adapted = node._ik_output_publisher.messages[0]
    assert original is not adapted
    for source, output in zip(original.poses, adapted.poses):
        assert output.position == source.position
    assert node._ik_frame_valid is True
    assert node._ik_frame_failure_reason == ""
    assert node._published_status_stamps == [1_000_000_000]
    assert events == ["corrected", "ik"]


def test_raw_callback_isolates_degenerate_ik_frame_from_original_stream():
    node = _runtime_node_for_raw_callback()
    message = _pose_array()

    node._raw_callback(message)

    assert len(node._output_publisher.messages) == 1
    assert len(node._ik_output_publisher.messages) == 0
    assert node._ik_frame_valid is False
    assert "right shoulder to left shoulder" in node._ik_frame_failure_reason
    assert node._published_status_stamps == [1_000_000_000]
    assert len(node._warnings) == 1


def test_raw_callback_isolates_unexpected_ik_adapter_exception(
    monkeypatch,
):
    node = _runtime_node_for_raw_callback()
    message = _runtime_pose_array()

    def fail_ik_adapter(*args, **kwargs):
        raise RuntimeError("synthetic IK adapter failure")

    monkeypatch.setattr(
        "pico_palm_skeleton_filter_node.pose_array_with_ik_arm_frames",
        fail_ik_adapter,
    )

    node._raw_callback(message)

    assert len(node._output_publisher.messages) == 1
    assert len(node._ik_output_publisher.messages) == 0
    assert node._ik_frame_valid is False
    assert node._ik_frame_failure_reason == "synthetic IK adapter failure"
    assert node._published_status_stamps == [1_000_000_000]
    assert len(node._warnings) == 1


def test_palm_sample_preserves_source_stamp_and_frame():
    sample = pose_stamped_to_sample(_palm(2_300_000_123))
    assert isinstance(sample, PalmSample)
    assert sample.stamp_ns == 2_300_000_123
    assert sample.frame_id == "pico"
    assert sample.position[0] == pytest.approx(0.5)


def test_nearest_palm_is_timestamp_bounded_and_side_local():
    node = object.__new__(PicoPalmSkeletonFilterNode)
    node._max_skew_s = 0.03
    node._palm_cache = {"left": deque(), "right": deque()}
    node._palm_cache["left"].append(
        PalmSample(990_000_000, np.zeros(3), np.array([0, 0, 0, 1.0]), "pico")
    )
    node._palm_cache["left"].append(
        PalmSample(1_010_000_000, np.ones(3), np.array([0, 0, 0, 1.0]), "pico")
    )
    assert node._nearest_palm(1_000_000_000, "left").stamp_ns == 990_000_000
    assert node._nearest_palm(1_000_000_000, "right") is None
    assert node._nearest_palm(1_040_100_000, "left") is None


def _runtime_side_node_for_stale_palm(previous_corrected_positions=None):
    node = object.__new__(PicoPalmSkeletonFilterNode)
    node._tracking_epoch = 3
    node._tracking_epoch_numeric = 3
    node._tracking_epoch_source = "wire_world_reset"
    node._ratio_max_stretch = 0.995
    node._nearest_palm = lambda *args, **kwargs: None
    node._side_status = {"left": {}}
    node._side_state = {
        "left": SimpleNamespace(
            calibration=object(),
            previous_corrected_positions=previous_corrected_positions,
            previous_corrected_orientations_xyzw=(
                np.tile([0.0, 0.0, 0.0, 1.0], (4, 1))
                if previous_corrected_positions is not None
                else None
            ),
            previous_elbow_position=None,
            wrist_to_palm_distance_m=None,
        )
    }
    return node


def test_stale_palm_holds_last_valid_corrected_side_output():
    indices = np.asarray(SIDE_INDICES["left"], dtype=int)
    held_positions = np.arange(12, dtype=float).reshape(4, 3) + 100.0
    node = _runtime_side_node_for_stale_palm(held_positions)
    input_positions = np.zeros((24, 3), dtype=float)
    input_positions[indices] = -5.0
    input_orientations = np.tile([0.0, 0.0, 0.0, 1.0], (24, 1))
    output_positions = input_positions.copy()
    output_orientations = input_orientations.copy()

    node._process_side(
        "left",
        3_000_000_000,
        input_positions,
        input_orientations,
        output_positions,
        output_orientations,
    )

    np.testing.assert_allclose(output_positions[indices], held_positions)
    np.testing.assert_allclose(output_positions[17], input_positions[17])
    assert node._side_status["left"]["corrected"] is False
    assert node._side_status["left"]["fallback_reason"] == "palm_stale_hold"
    assert node._side_status["left"]["output_mode"] == "hold_last_corrected"


def test_stale_palm_without_previous_correction_keeps_raw_smpl_output():
    indices = np.asarray(SIDE_INDICES["left"], dtype=int)
    node = _runtime_side_node_for_stale_palm()
    input_positions = np.arange(72, dtype=float).reshape(24, 3)
    input_orientations = np.tile([0.0, 0.0, 0.0, 1.0], (24, 1))
    output_positions = input_positions.copy()
    output_orientations = input_orientations.copy()

    node._process_side(
        "left",
        3_000_000_000,
        input_positions,
        input_orientations,
        output_positions,
        output_orientations,
    )

    np.testing.assert_allclose(output_positions[indices], input_positions[indices])
    assert node._side_status["left"]["fallback_reason"] == "palm_stale"
    assert node._side_status["left"]["output_mode"] == "raw_smpl"


def test_status_json_has_independent_side_fields():
    node = object.__new__(PicoPalmSkeletonFilterNode)
    node._tracking_epoch = 7
    node._tracking_epoch_source = "wire_world_reset"
    node._source_frame_id = "pico"
    node._ratio_max_stretch = 0.995
    node._ik_output_topic = "/pico/smpl_palm_corrected_ik"
    node._left_shoulder_local_x_offset_rad = np.pi / 2.0
    node._right_shoulder_local_x_offset_rad = -np.pi / 2.0
    node._ik_frame_valid = False
    node._ik_frame_failure_reason = "elbow to wrist must be non-zero"
    node._tcp_calibration_revisions = {"left": 4, "right": 5}
    node._side_status = {
        "left": {"baseline_ready": True, "corrected": True, "fallback_reason": ""},
        "right": {
            "baseline_ready": False,
            "corrected": False,
            "fallback_reason": "palm_stale",
        },
    }
    payload = node._status_payload()
    assert payload["left"]["corrected"] is True
    assert payload["right"]["fallback_reason"] == "palm_stale"
    assert payload["tracking_epoch"] == 7
    assert payload["tracking_epoch_source"] == "wire_world_reset"
    assert payload["stream_valid"] is True
    assert payload["source_frame_id"] == "pico"
    assert payload["tcp_calibration_revision_left"] == 4
    assert payload["tcp_calibration_revision_right"] == 5
    assert payload["ratio_max_stretch"] == pytest.approx(0.995)
    assert payload["ik_output_topic"] == "/pico/smpl_palm_corrected_ik"
    assert payload["ik_shoulder_frame_semantics"] == "local_x_right_multiply"
    assert payload["ik_elbow_frame_semantics"] == (
        "local_x_forearm_local_y_flexion_axis_body_left_signed_temporal_fallback"
    )
    assert payload["left_shoulder_local_x_offset_rad"] == pytest.approx(np.pi / 2.0)
    assert payload["right_shoulder_local_x_offset_rad"] == pytest.approx(-np.pi / 2.0)
    assert payload["ik_frame_valid"] is False
    assert payload["ik_frame_failure_reason"] == "elbow to wrist must be non-zero"


def test_status_json_rejects_zero_or_inferred_tracking_epoch():
    node = object.__new__(PicoPalmSkeletonFilterNode)
    node._side_status = {"left": {}, "right": {}}
    node._tcp_calibration_revisions = {"left": 1, "right": 1}
    node._source_frame_id = "pico"
    node._ratio_max_stretch = 0.995
    node._tracking_epoch = 0
    node._tracking_epoch_source = "unknown"
    assert node._status_payload()["stream_valid"] is False

    node._tracking_epoch = 2
    node._tracking_epoch_source = "local_inferred"
    assert node._status_payload()["stream_valid"] is False


def test_epoch_transition_clears_world_dependent_runtime_state():
    node = object.__new__(PicoPalmSkeletonFilterNode)
    node._tracking_epoch = 4
    node._tracking_epoch_numeric = 4
    node._tracking_epoch_source = "tcp_connection"
    node._palm_cache = {
        "left": deque([PalmSample(1, np.zeros(3), np.array([0, 0, 0, 1.0]), "pico")]),
        "right": deque([PalmSample(1, np.zeros(3), np.array([0, 0, 0, 1.0]), "pico")]),
    }
    node._side_state = {
        side: SimpleNamespace(
            positions=deque([np.zeros((24, 3))]),
            orientations=deque([np.zeros((24, 4))]),
            palm_positions=deque([np.zeros(3)]),
            palm_orientations=deque([np.array([0, 0, 0, 1.0])]),
            sample_stamps_ns=deque([1]),
            previous_elbow_position=np.ones(3),
            previous_corrected_positions=np.ones((4, 3)),
            previous_corrected_orientations_xyzw=np.tile(
                [0.0, 0.0, 0.0, 1.0], (4, 1)
            ),
        )
        for side in ("left", "right")
    }

    status = String()
    status.data = '{"tracking_epoch":5,"tracking_epoch_source":"wire_world_reset"}'
    node._tracking_epoch_status_callback(status)

    assert node._tracking_epoch == 5
    assert all(not cache for cache in node._palm_cache.values())
    for state in node._side_state.values():
        assert not state.positions
        assert not state.orientations
        assert not state.palm_positions
        assert not state.palm_orientations
        assert not state.sample_stamps_ns
        assert state.previous_elbow_position is None
        assert state.previous_corrected_positions is None
        assert state.previous_corrected_orientations_xyzw is None


def test_first_explicit_epoch_clears_pre_epoch_ik_continuity_reference():
    node = object.__new__(PicoPalmSkeletonFilterNode)
    node._tracking_epoch = 0
    node._tracking_epoch_numeric = 0
    node._tracking_epoch_source = "unknown"
    node._previous_ik_orientations_xyzw = np.tile(
        [0.0, 0.0, 0.0, 1.0], (24, 1)
    )
    node._ik_frame_valid = True
    node._ik_frame_failure_reason = ""
    node._side_state = {
        "left": SimpleNamespace(
            previous_corrected_positions=np.ones((4, 3)),
            previous_corrected_orientations_xyzw=np.tile(
                [0.0, 0.0, 0.0, 1.0], (4, 1)
            ),
        )
    }
    node._side_status = {"left": {}}

    node._handle_epoch_transition(0, 1)

    assert node._previous_ik_orientations_xyzw is None
    assert node._ik_frame_valid is False
    assert node._ik_frame_failure_reason == "tracking_epoch_transition"
    assert node._side_state["left"].previous_corrected_positions is None
    assert node._side_state["left"].previous_corrected_orientations_xyzw is None
    assert node._side_status["left"]["output_mode"] == "raw_smpl"


def test_palm_samples_are_accepted_only_for_consistent_explicit_epoch():
    node = object.__new__(PicoPalmSkeletonFilterNode)
    node._palm_cache = {"left": deque(), "right": deque()}
    node._tracking_epoch = 7
    node._tracking_epoch_numeric = 6
    node._tracking_epoch_source = "tcp_connection"

    node._palm_callback(_palm(), "left")
    assert not node._palm_cache["left"]

    numeric = UInt64()
    numeric.data = 7
    node._tracking_epoch_callback(numeric)
    node._palm_callback(_palm(), "left")
    assert len(node._palm_cache["left"]) == 1
    assert node._palm_cache["left"][0].tracking_epoch == 7


def test_nearest_palm_never_crosses_tracking_epoch():
    node = object.__new__(PicoPalmSkeletonFilterNode)
    node._max_skew_s = 0.03
    node._palm_cache = {"left": deque(), "right": deque()}
    node._palm_cache["left"].append(
        PalmSample(
            1_000_000_000,
            np.zeros(3),
            np.array([0, 0, 0, 1.0]),
            "pico",
            tracking_epoch=4,
        )
    )

    assert node._nearest_palm(1_000_000_000, "left", tracking_epoch=5) is None
    assert (
        node._nearest_palm(1_000_000_000, "left", tracking_epoch=4).tracking_epoch
        == 4
    )

def test_load_wrist_pivot_artifact_converts_legacy_vector_to_distance(tmp_path):
    artifact = tmp_path / "left_wrist_pivot.yaml"
    artifact.write_text(
        "\n".join(
            [
                "valid: true",
                "schema_version: 1",
                "side: left",
                "transform_convention: wrist_to_palm",
                "source_frame: pico",
                "tracking_epoch: 4",
                "tracking_epoch_source: wire_world_reset",
                "wrist_to_palm_m:",
                "  x: 0.06",
                "  y: -0.08",
                "  z: 0.0",
                "quality:",
                "  sample_count: 60",
                "  position_rms_m: 0.005",
                "  condition_number: 20.0",
            ]
        ),
        encoding="utf-8",
    )
    distance, status = load_wrist_pivot_artifact(artifact, "left")
    assert distance == pytest.approx(0.10)
    assert status == "loaded_legacy_vector_norm"


def test_load_tcp_calibration_revision_is_strict(tmp_path):
    artifact = tmp_path / "left_tcp.yaml"
    artifact.write_text(
        "\n".join(
                [
                    "schema_version: 2",
                    "valid: true",
                    "side: left",
                    "pose_semantics: controller_pose",
                    "transform_convention: T_controller_palm",
                    "translation_m: [0.01, 0.0, 0.0]",
                    "quaternion_xyzw: [0.0, 0.0, 0.0, 1.0]",
                    "orientation_calibrated: true",
                    "calibration_revision: 8",
                    "lineage: [/pico/pose/left_hand]",
                    "quality:",
                    "  sample_count: 4",
                    "  position_rms_m: 0.003",
                    "  sample_matrix_rank: 6",
                    "  sample_matrix_condition: 12.0",
                ]
        ),
        encoding="utf-8",
    )
    assert load_tcp_calibration_revision(artifact, "left") == 8
    with pytest.raises(ValueError, match="side mismatch"):
        load_tcp_calibration_revision(artifact, "right")


def test_load_left_arm_geometry_artifact_validates_lineage(tmp_path):
    artifact = tmp_path / "left_geometry.yaml"
    artifact.write_text(yaml.safe_dump(_valid_left_geometry_document()), encoding="utf-8")
    result = load_left_arm_geometry_artifact(
        artifact,
        expected_tcp_revision=8,
        expected_tcp_sha256="tcp123",
        expected_wrist_pivot_sha256="abc123",
    )
    assert result.upper_arm_length_m == pytest.approx(0.302)
    assert result.forearm_length_m == pytest.approx(0.251)
    assert result.calibration_revision == 3


@pytest.mark.parametrize("side", ["left", "right"])
def test_load_arm_geometry_artifact_is_side_specific(tmp_path, side):
    artifact = tmp_path / f"{side}_geometry.yaml"
    artifact.write_text(
        yaml.safe_dump(_valid_geometry_document(side)), encoding="utf-8"
    )
    result = load_arm_geometry_artifact(
        artifact,
        expected_side=side,
        expected_tcp_revision=8,
        expected_tcp_sha256="tcp123",
        expected_wrist_pivot_sha256="abc123",
    )
    assert isinstance(result, LoadedArmGeometry)
    assert result.side == side
    assert result.calibration_revision == 3
    assert result.upper_arm_length_m == pytest.approx(0.302)
    assert result.forearm_length_m == pytest.approx(0.251)


def test_right_loader_rejects_left_geometry_artifact(tmp_path):
    artifact = tmp_path / "left_geometry.yaml"
    artifact.write_text(
        yaml.safe_dump(_valid_geometry_document("left")), encoding="utf-8"
    )
    with pytest.raises(ValueError, match="side mismatch: expected right"):
        load_arm_geometry_artifact(
            artifact,
            expected_side="right",
            expected_tcp_revision=8,
            expected_tcp_sha256="tcp123",
            expected_wrist_pivot_sha256="abc123",
        )


def test_runtime_loader_accepts_semantic_tcp_translation_lineage(tmp_path):
    tcp = tmp_path / "tcp.yaml"
    tcp_document = {
        "schema_version": 2,
        "valid": True,
        "side": "left",
        "pose_semantics": "controller_pose",
        "transform_convention": "T_controller_palm",
        "translation_m": [0.01, 0.0, 0.0],
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        "orientation_calibrated": True,
        "calibration_revision": 9,
        "translation_revision": 8,
        "lineage": ["/pico/pose/left_hand"],
        "quality": {
            "sample_count": 4,
            "position_rms_m": 0.003,
            "sample_matrix_rank": 6,
            "sample_matrix_condition": 12.0,
        },
    }
    tcp_document["translation_fingerprint_sha256"] = translation_fingerprint(
        tcp_document
    )
    tcp.write_text(yaml.safe_dump(tcp_document), encoding="utf-8")
    wrist = tmp_path / "wrist.yaml"
    wrist.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "valid": True,
                "side": "left",
                "transform_convention": "wrist_to_palm",
                "source_topic": "/pico/palm_left",
                "source_frame": "pico",
                "tracking_epoch": 4,
                "tracking_epoch_source": "tcp_connection",
                "wrist_to_palm_m": {"x": 0.08, "y": 0.0, "z": 0.0},
                "quality": {
                    "sample_count": 60,
                    "position_rms_m": 0.002,
                    "condition_number": 20.0,
                },
            }
        ),
        encoding="utf-8",
    )
    geometry = tmp_path / "geometry.yaml"
    geometry_document = _valid_left_geometry_document()
    geometry_document["tcp_translation_revision"] = 8
    geometry_document["tcp_translation_fingerprint_sha256"] = tcp_document[
        "translation_fingerprint_sha256"
    ]
    geometry_document["wrist_pivot_sha256"] = hashlib.sha256(
        wrist.read_bytes()
    ).hexdigest()
    geometry.write_text(yaml.safe_dump(geometry_document), encoding="utf-8")

    loaded = load_arm_geometry_artifact(
        geometry,
        expected_side="left",
        expected_tcp_revision=9,
        expected_tcp_sha256=hashlib.sha256(tcp.read_bytes()).hexdigest(),
        expected_wrist_pivot_sha256=hashlib.sha256(wrist.read_bytes()).hexdigest(),
        tcp_path=tcp,
        wrist_path=wrist,
    )

    assert loaded.upper_arm_length_m == pytest.approx(0.302)


@pytest.mark.parametrize(
    "replacement,match",
    [
        ("side: right", "side"),
        ("valid: false", "valid"),
        ("candidate_status: rejected", "candidate_status"),
        ("schema_version: 1", "schema_version"),
        ("tcp_calibration_revision: 7", "TCP revision"),
        ("tcp_artifact_sha256: wrong", "TCP artifact hash"),
        ("wrist_pivot_sha256: wrong", "wrist pivot"),
    ],
)
def test_load_left_arm_geometry_artifact_rejects_contract_mismatch(
    tmp_path, replacement, match
):
    key = replacement.split(":", 1)[0]
    document = _valid_left_geometry_document()
    value = yaml.safe_load(replacement)[key]
    document[key] = value
    artifact = tmp_path / "left_geometry.yaml"
    artifact.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError, match=match):
        load_left_arm_geometry_artifact(
            artifact,
            expected_tcp_revision=8,
            expected_tcp_sha256="tcp123",
            expected_wrist_pivot_sha256="abc123",
        )


@pytest.mark.parametrize(
    "mutator,match",
    [
        (lambda doc: doc.update(tracking_epoch=0), "tracking_epoch"),
        (lambda doc: doc.update(tracking_epoch_source="local_inferred"), "tracking_epoch_source"),
        (lambda doc: doc.update(rejection_reasons=["failed"]), "rejection_reasons"),
        (lambda doc: doc.pop("quality"), "quality"),
        (lambda doc: doc["quality"].pop("right_angle_error_role"), "right_angle_error_role"),
        (
            lambda doc: doc["quality"].update(right_angle_error_role="hard_gate"),
            "right_angle_error_role",
        ),
        (lambda doc: doc.update(gate_results={"circle": False}), "gate_results"),
        (lambda doc: doc.update(covariance_upper_triangle_8x8=[0.0]), "covariance"),
    ],
)
def test_load_left_arm_geometry_artifact_rejects_incomplete_gate_contract(
    tmp_path, mutator, match
):
    document = _valid_left_geometry_document()
    mutator(document)
    artifact = tmp_path / "left_geometry.yaml"
    artifact.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError, match=match):
        load_left_arm_geometry_artifact(
            artifact,
            expected_tcp_revision=8,
            expected_tcp_sha256="tcp123",
            expected_wrist_pivot_sha256="abc123",
        )


def test_load_wrist_pivot_artifact_prefers_explicit_scalar_distance(tmp_path):
    artifact = tmp_path / "left_wrist_pivot_v2.yaml"
    artifact.write_text(
        "\n".join(
            [
                "valid: true",
                "schema_version: 1",
                "side: left",
                "transform_convention: wrist_to_palm",
                "source_frame: pico",
                "tracking_epoch: 4",
                "tracking_epoch_source: wire_world_reset",
                "wrist_to_palm_distance_m: 0.071",
                "wrist_to_palm_m: [0.5, 0.5, 0.5]",
                "quality:",
                "  sample_count: 60",
                "  position_rms_m: 0.005",
                "  condition_number: 20.0",
            ]
        ),
        encoding="utf-8",
    )
    distance, status = load_wrist_pivot_artifact(artifact, "left")
    assert distance == pytest.approx(0.071)
    assert status == "loaded_scalar_distance"


@pytest.mark.parametrize(
    "document",
    [
        "valid: false\n",
        "valid: true\nside: right\ntransform_convention: wrist_to_palm\n",
        "valid: true\nside: left\ntransform_convention: palm_to_wrist\n",
        "valid: true\nside: left\ntransform_convention: wrist_to_palm\nwrist_to_palm_m: [0.1, 0, 0]\n",
    ],
)
def test_load_wrist_pivot_artifact_rejects_invalid_contract(tmp_path, document):
    artifact = tmp_path / "invalid.yaml"
    artifact.write_text(document, encoding="utf-8")
    offset, status = load_wrist_pivot_artifact(artifact, "left")
    assert offset is None
    assert status.startswith("invalid:")
