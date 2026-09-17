"""Validated native and canonical hand-tracking data models.

The models in this module are transport independent.  They intentionally keep
the source and receiver clocks separate because the native devices do not share
the PC monotonic clock.
"""
from __future__ import annotations

from dataclasses import dataclass
import base64
import math
from typing import Any, Mapping

import numpy as np


SIDES = ("left", "right")
PICO_TRACKING_FRAME = "pico_tracking_initial"
PICO_HEAD_CURRENT_FRAME = "pico_head_current"
PICO_MAPPING_VERSION = "pico26_to_mediapipe21_v1"
PICO_HEAD_MAPPING_VERSION = "pico_head_current_v1"
PICO_JOINT_NAMES = (
    "palm", "wrist",
    "thumb_metacarpal", "thumb_proximal", "thumb_distal", "thumb_tip",
    "index_metacarpal", "index_proximal", "index_intermediate", "index_distal", "index_tip",
    "middle_metacarpal", "middle_proximal", "middle_intermediate", "middle_distal", "middle_tip",
    "ring_metacarpal", "ring_proximal", "ring_intermediate", "ring_distal", "ring_tip",
    "little_metacarpal", "little_proximal", "little_intermediate", "little_distal", "little_tip",
)


def _required_string(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{field} must be a non-empty string")
    return value


def _nonnegative_int(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or int(value) < 0:
        raise ValueError(f"{field} must be a non-negative integer")
    return int(value)


def _optional_nonnegative_int(value: Any, field: str) -> int | None:
    return None if value is None else _nonnegative_int(value, field)


def _finite_array(value: Any, shape: tuple[int, ...], field: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"{field} must be finite with shape {shape}")
    return result.copy()


def _quaternion(value: Any, field: str, *, allow_zero: bool = False) -> np.ndarray:
    result = _finite_array(value, (4,), field)
    norm = float(np.linalg.norm(result))
    if not allow_zero and norm < 1.0e-12:
        raise ValueError(f"{field} must be non-zero")
    return result


def _pose(value: Any, field: str, *, allow_zero_quaternion: bool = False) -> np.ndarray:
    result = _finite_array(value, (7,), field)
    _quaternion(result[3:], f"{field}[3:7]", allow_zero=allow_zero_quaternion)
    return result


def _bool_vector(value: Any, size: int, field: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.bool_)
    if result.shape != (size,):
        raise ValueError(f"{field} must have shape ({size},)")
    return result.copy()


def _object(value: Any, field: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field} must be an object")
    return value


def _exact_keys(value: Mapping[str, Any], expected: set[str], field: str) -> None:
    missing = expected - set(value)
    extra = set(value) - expected
    if missing or extra:
        raise ValueError(
            f"{field} has invalid fields; missing={sorted(missing)}, extra={sorted(extra)}"
        )


def _base64_bytes(value: Any, field: str) -> bytes:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{field} must be a non-empty base64 string")
    try:
        decoded = base64.b64decode(value.encode("ascii"), validate=True)
    except (UnicodeEncodeError, ValueError) as exc:
        raise ValueError(f"{field} is not valid base64") from exc
    if not decoded:
        raise ValueError(f"{field} must decode to non-empty bytes")
    return decoded


@dataclass(frozen=True)
class HandObservation:
    """One canonical 21-point observation for one hand."""

    source: str
    side: str
    source_instance_id: str
    source_sequence: int | None
    source_timestamp_ns: int | None
    received_timestamp_ns: int
    receiver_instance_id: str
    receiver_frame_sequence: int
    coordinate_frame: str
    mapping_version: str
    keypoints_m: np.ndarray
    joint_valid: np.ndarray
    valid: bool
    wrist_pose: np.ndarray | None
    frame_association_id: str

    def __post_init__(self) -> None:
        _required_string(self.source, "source")
        if self.side not in SIDES:
            raise ValueError("side must be left or right")
        _required_string(self.source_instance_id, "source_instance_id")
        _optional_nonnegative_int(self.source_sequence, "source_sequence")
        _optional_nonnegative_int(self.source_timestamp_ns, "source_timestamp_ns")
        _nonnegative_int(self.received_timestamp_ns, "received_timestamp_ns")
        _required_string(self.receiver_instance_id, "receiver_instance_id")
        _nonnegative_int(self.receiver_frame_sequence, "receiver_frame_sequence")
        _required_string(self.coordinate_frame, "coordinate_frame")
        _required_string(self.mapping_version, "mapping_version")
        points = _finite_array(self.keypoints_m, (21, 3), "keypoints_m")
        valid = _bool_vector(self.joint_valid, 21, "joint_valid")
        if not isinstance(self.valid, (bool, np.bool_)):
            raise ValueError("valid must be boolean")
        if bool(self.valid) and not bool(valid.all()):
            raise ValueError("valid observation must have all joints valid")
        pose = None if self.wrist_pose is None else _pose(self.wrist_pose, "wrist_pose")
        _required_string(self.frame_association_id, "frame_association_id")
        object.__setattr__(self, "keypoints_m", points)
        object.__setattr__(self, "joint_valid", valid)
        object.__setattr__(self, "wrist_pose", pose)

    def to_dict(self) -> dict[str, Any]:
        return {
            "source": self.source,
            "side": self.side,
            "source_instance_id": self.source_instance_id,
            "source_sequence": self.source_sequence,
            "source_timestamp_ns": self.source_timestamp_ns,
            "received_timestamp_ns": self.received_timestamp_ns,
            "receiver_instance_id": self.receiver_instance_id,
            "receiver_frame_sequence": self.receiver_frame_sequence,
            "coordinate_frame": self.coordinate_frame,
            "mapping_version": self.mapping_version,
            "keypoints_m": self.keypoints_m.tolist(),
            "joint_valid": self.joint_valid.tolist(),
            "valid": bool(self.valid),
            "wrist_pose": None if self.wrist_pose is None else self.wrist_pose.tolist(),
            "frame_association_id": self.frame_association_id,
        }


@dataclass(frozen=True)
class ArmInputObservation:
    """One tracked wrist pose plus an optional forearm pose before TCP mapping."""

    source: str
    side: str
    tracked_frame: str
    reference_frame: str
    pose: np.ndarray | None
    valid: bool
    source_timestamp_ns: int | None
    received_timestamp_ns: int
    receiver_instance_id: str
    receiver_frame_sequence: int
    mapping_version: str
    frame_association_id: str
    source_sequence: int | None = None
    source_instance_id: str = "unknown"
    elbow_pose: np.ndarray | None = None

    def __post_init__(self) -> None:
        _required_string(self.source, "source")
        if self.side not in SIDES:
            raise ValueError("side must be left or right")
        _required_string(self.tracked_frame, "tracked_frame")
        _required_string(self.reference_frame, "reference_frame")
        if not isinstance(self.valid, (bool, np.bool_)):
            raise ValueError("valid must be boolean")
        pose = None if self.pose is None else _pose(self.pose, "pose")
        if bool(self.valid) and pose is None:
            raise ValueError("valid arm input requires a pose")
        _optional_nonnegative_int(self.source_timestamp_ns, "source_timestamp_ns")
        _nonnegative_int(self.received_timestamp_ns, "received_timestamp_ns")
        _required_string(self.receiver_instance_id, "receiver_instance_id")
        _nonnegative_int(self.receiver_frame_sequence, "receiver_frame_sequence")
        _required_string(self.mapping_version, "mapping_version")
        _required_string(self.frame_association_id, "frame_association_id")
        _optional_nonnegative_int(self.source_sequence, "source_sequence")
        _required_string(self.source_instance_id, "source_instance_id")
        elbow_pose = None if self.elbow_pose is None else _pose(self.elbow_pose, "elbow_pose")
        object.__setattr__(self, "pose", pose)
        object.__setattr__(self, "elbow_pose", elbow_pose)

    def to_dict(self) -> dict[str, Any]:
        return {
            "source": self.source,
            "side": self.side,
            "tracked_frame": self.tracked_frame,
            "reference_frame": self.reference_frame,
            "pose": None if self.pose is None else self.pose.tolist(),
            "valid": bool(self.valid),
            "source_timestamp_ns": self.source_timestamp_ns,
            "source_sequence": self.source_sequence,
            "source_instance_id": self.source_instance_id,
            "received_timestamp_ns": self.received_timestamp_ns,
            "receiver_instance_id": self.receiver_instance_id,
            "receiver_frame_sequence": self.receiver_frame_sequence,
            "mapping_version": self.mapping_version,
            "frame_association_id": self.frame_association_id,
            "elbow_pose": None if self.elbow_pose is None else self.elbow_pose.tolist(),
        }


@dataclass(frozen=True)
class PicoRawJoint:
    index: int
    name: str
    valid: bool
    pose: np.ndarray
    radius_m: float

    def __post_init__(self) -> None:
        _nonnegative_int(self.index, "index")
        if self.index >= 26:
            raise ValueError("PICO joint index must be in 0..25")
        if self.name != PICO_JOINT_NAMES[self.index]:
            raise ValueError("PICO joint name does not match index")
        if not isinstance(self.valid, (bool, np.bool_)):
            raise ValueError("joint valid must be boolean")
        pose = _pose(self.pose, "joint.pose", allow_zero_quaternion=True)
        radius = float(self.radius_m)
        if not math.isfinite(radius) or radius < 0.0:
            raise ValueError("joint radius must be a finite non-negative number")
        object.__setattr__(self, "pose", pose)
        object.__setattr__(self, "radius_m", radius)

    def to_dict(self) -> dict[str, Any]:
        return {
            "index": self.index,
            "name": self.name,
            "valid": bool(self.valid),
            "pose": self.pose.tolist(),
            "radius_m": self.radius_m,
        }

    @classmethod
    def from_dict(cls, value: Any) -> "PicoRawJoint":
        item = _object(value, "PICO joint")
        _exact_keys(item, {"index", "name", "valid", "pose", "radius_m"}, "PICO joint")
        return cls(
            index=item["index"],
            name=item["name"],
            valid=item["valid"],
            pose=item["pose"],
            radius_m=item["radius_m"],
        )


@dataclass(frozen=True)
class PicoRawHand:
    valid: bool
    wrist_valid: bool
    wrist_pose: np.ndarray
    joints: tuple[PicoRawJoint, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.valid, (bool, np.bool_)) or not isinstance(self.wrist_valid, (bool, np.bool_)):
            raise ValueError("PICO hand validity must be boolean")
        pose = _pose(self.wrist_pose, "wrist_pose", allow_zero_quaternion=True)
        if len(self.joints) != 26 or tuple(j.index for j in self.joints) != tuple(range(26)):
            raise ValueError("PICO hand must contain joints 0..25 in order")
        object.__setattr__(self, "wrist_pose", pose)
        object.__setattr__(self, "joints", tuple(self.joints))

    def to_dict(self) -> dict[str, Any]:
        return {
            "valid": bool(self.valid),
            "wrist_valid": bool(self.wrist_valid),
            "wrist_pose": self.wrist_pose.tolist(),
            "joints": [joint.to_dict() for joint in self.joints],
        }

    @classmethod
    def from_dict(cls, value: Any) -> "PicoRawHand":
        item = _object(value, "PICO hand")
        _exact_keys(item, {"valid", "wrist_valid", "wrist_pose", "joints"}, "PICO hand")
        joints_value = item["joints"]
        if not isinstance(joints_value, (list, tuple)):
            raise ValueError("PICO hand joints must be an array")
        return cls(
            valid=item["valid"],
            wrist_valid=item["wrist_valid"],
            wrist_pose=item["wrist_pose"],
            joints=tuple(PicoRawJoint.from_dict(joint) for joint in joints_value),
        )


@dataclass(frozen=True)
class PicoRawFrame:
    source_timestamp_ms: int
    protocol_version: int
    flags: int
    joint_count: int
    head_valid: bool
    head_pose: np.ndarray
    hands: Mapping[str, PicoRawHand]
    raw_packet: bytes
    received_timestamp_ns: int
    receiver_instance_id: str
    connection_generation: int
    receiver_frame_sequence: int

    def __post_init__(self) -> None:
        _nonnegative_int(self.source_timestamp_ms, "source_timestamp_ms")
        if self.protocol_version != 1 or self.joint_count != 26:
            raise ValueError("unsupported PICO protocol metadata")
        if isinstance(self.flags, bool) or not isinstance(self.flags, int) or not 0 <= self.flags <= 0xFF:
            raise ValueError("flags must be an unsigned byte")
        if self.flags & ~0x07:
            raise ValueError("unknown PICO v1 flags")
        if not isinstance(self.head_valid, (bool, np.bool_)):
            raise ValueError("head_valid must be boolean")
        head = _pose(self.head_pose, "head_pose", allow_zero_quaternion=True)
        if set(self.hands) != set(SIDES):
            raise ValueError("PICO raw frame must contain left and right hands")
        if not isinstance(self.raw_packet, (bytes, bytearray)) or not self.raw_packet:
            raise ValueError("raw_packet must be non-empty bytes")
        _nonnegative_int(self.received_timestamp_ns, "received_timestamp_ns")
        _required_string(self.receiver_instance_id, "receiver_instance_id")
        _nonnegative_int(self.connection_generation, "connection_generation")
        _nonnegative_int(self.receiver_frame_sequence, "receiver_frame_sequence")
        object.__setattr__(self, "head_pose", head)
        object.__setattr__(self, "raw_packet", bytes(self.raw_packet))
        object.__setattr__(self, "hands", {side: self.hands[side] for side in SIDES})

    @property
    def source_timestamp_ns(self) -> int:
        return self.source_timestamp_ms * 1_000_000

    @property
    def association_id(self) -> str:
        return f"{self.receiver_instance_id}:{self.connection_generation}:{self.receiver_frame_sequence}"

    def to_dict(self) -> dict[str, Any]:
        return {
            "source_timestamp_ms": self.source_timestamp_ms,
            "source_timestamp_ns": self.source_timestamp_ns,
            "protocol_version": self.protocol_version,
            "flags": self.flags,
            "joint_count": self.joint_count,
            "head_valid": bool(self.head_valid),
            "head_pose": self.head_pose.tolist(),
            "hands": {side: self.hands[side].to_dict() for side in SIDES},
            "raw_packet_base64": base64.b64encode(self.raw_packet).decode("ascii"),
            "received_timestamp_ns": self.received_timestamp_ns,
            "receiver_instance_id": self.receiver_instance_id,
            "connection_generation": self.connection_generation,
            "receiver_frame_sequence": self.receiver_frame_sequence,
            "association_id": self.association_id,
        }

    @classmethod
    def from_dict(cls, value: Any) -> "PicoRawFrame":
        item = _object(value, "PICO raw frame")
        _exact_keys(
            item,
            {
                "source_timestamp_ms", "source_timestamp_ns", "protocol_version", "flags",
                "joint_count", "head_valid", "head_pose", "hands", "raw_packet_base64",
                "received_timestamp_ns", "receiver_instance_id", "connection_generation",
                "receiver_frame_sequence", "association_id",
            },
            "PICO raw frame",
        )
        timestamp_ms = item["source_timestamp_ms"]
        if isinstance(timestamp_ms, bool) or not isinstance(timestamp_ms, (int, np.integer)):
            raise ValueError("source_timestamp_ms must be an integer")
        if item["source_timestamp_ns"] != int(timestamp_ms) * 1_000_000:
            raise ValueError("PICO source timestamp fields disagree")
        hands_value = _object(item["hands"], "PICO raw hands")
        _exact_keys(hands_value, set(SIDES), "PICO raw hands")
        frame = cls(
            source_timestamp_ms=int(timestamp_ms),
            protocol_version=item["protocol_version"],
            flags=item["flags"],
            joint_count=item["joint_count"],
            head_valid=item["head_valid"],
            head_pose=item["head_pose"],
            hands={side: PicoRawHand.from_dict(hands_value[side]) for side in SIDES},
            raw_packet=_base64_bytes(item["raw_packet_base64"], "raw_packet_base64"),
            received_timestamp_ns=item["received_timestamp_ns"],
            receiver_instance_id=item["receiver_instance_id"],
            connection_generation=item["connection_generation"],
            receiver_frame_sequence=item["receiver_frame_sequence"],
        )
        if item["association_id"] != frame.association_id:
            raise ValueError("PICO raw frame association_id does not match metadata")
        return frame


@dataclass(frozen=True)
class LegacyPicoPalmFrame:
    """Decoded TJVR v1-v4 frame used by the historical palm/arm path."""

    protocol_version: int
    packet_size: int
    flags: int
    sequence: int
    tracking_epoch: int
    source_timestamp_ns: int
    bridge_send_monotonic_ns: int
    left_pose: np.ndarray
    right_pose: np.ndarray
    upper_limb_skeleton_valid: bool
    upper_limb_rotations_valid: bool
    upper_limb_points: np.ndarray
    upper_limb_rotations_xyzw: np.ndarray
    raw_packet: bytes
    received_timestamp_ns: int
    receiver_instance_id: str
    receiver_frame_sequence: int

    def __post_init__(self) -> None:
        if self.protocol_version not in (1, 2, 3, 4):
            raise ValueError("unsupported TJVR protocol version")
        expected_size = {1: 160, 2: 208, 3: 400, 4: 656}[self.protocol_version]
        if self.packet_size != expected_size:
            raise ValueError("TJVR packet size does not match protocol version")
        if isinstance(self.flags, bool) or not isinstance(self.flags, int) or self.flags < 0:
            raise ValueError("TJVR flags must be a non-negative integer")
        _nonnegative_int(self.sequence, "sequence")
        _nonnegative_int(self.tracking_epoch, "tracking_epoch")
        _nonnegative_int(self.source_timestamp_ns, "source_timestamp_ns")
        _nonnegative_int(self.bridge_send_monotonic_ns, "bridge_send_monotonic_ns")
        for field, pose in (("left_pose", self.left_pose), ("right_pose", self.right_pose)):
            _pose(pose, field)
        points = _finite_array(self.upper_limb_points, (8, 3), "upper_limb_points")
        rotations = np.asarray(self.upper_limb_rotations_xyzw, dtype=np.float64)
        if rotations.shape != (8, 4) or not np.isfinite(rotations).all():
            raise ValueError("upper_limb_rotations_xyzw must be finite with shape (8, 4)")
        for index, rotation in enumerate(rotations):
            _quaternion(rotation, f"upper_limb_rotations_xyzw[{index}]")
        if not isinstance(self.upper_limb_skeleton_valid, (bool, np.bool_)) or not isinstance(self.upper_limb_rotations_valid, (bool, np.bool_)):
            raise ValueError("TJVR skeleton validity must be boolean")
        if not isinstance(self.raw_packet, (bytes, bytearray)) or len(self.raw_packet) != self.packet_size:
            raise ValueError("raw_packet must contain the complete TJVR packet")
        _nonnegative_int(self.received_timestamp_ns, "received_timestamp_ns")
        _required_string(self.receiver_instance_id, "receiver_instance_id")
        _nonnegative_int(self.receiver_frame_sequence, "receiver_frame_sequence")
        object.__setattr__(self, "left_pose", _pose(self.left_pose, "left_pose"))
        object.__setattr__(self, "right_pose", _pose(self.right_pose, "right_pose"))
        object.__setattr__(self, "upper_limb_points", points)
        object.__setattr__(self, "upper_limb_rotations_xyzw", rotations.copy())
        object.__setattr__(self, "raw_packet", bytes(self.raw_packet))

    @property
    def association_id(self) -> str:
        return f"{self.receiver_instance_id}:{self.tracking_epoch}:{self.sequence}"

    def to_dict(self) -> dict[str, Any]:
        return {
            "protocol_version": self.protocol_version,
            "packet_size": self.packet_size,
            "flags": self.flags,
            "sequence": self.sequence,
            "tracking_epoch": self.tracking_epoch,
            "source_timestamp_ns": self.source_timestamp_ns,
            "bridge_send_monotonic_ns": self.bridge_send_monotonic_ns,
            "left_pose": self.left_pose.tolist(),
            "right_pose": self.right_pose.tolist(),
            "upper_limb_skeleton_valid": bool(self.upper_limb_skeleton_valid),
            "upper_limb_rotations_valid": bool(self.upper_limb_rotations_valid),
            "upper_limb_points": self.upper_limb_points.tolist(),
            "upper_limb_rotations_xyzw": self.upper_limb_rotations_xyzw.tolist(),
            "raw_packet_base64": base64.b64encode(self.raw_packet).decode("ascii"),
            "raw_packet_size": len(self.raw_packet),
            "received_timestamp_ns": self.received_timestamp_ns,
            "receiver_instance_id": self.receiver_instance_id,
            "receiver_frame_sequence": self.receiver_frame_sequence,
            "association_id": self.association_id,
        }

    @classmethod
    def from_dict(cls, value: Any) -> "LegacyPicoPalmFrame":
        item = _object(value, "legacy PICO palm frame")
        _exact_keys(
            item,
            {
                "protocol_version", "packet_size", "flags", "sequence", "tracking_epoch",
                "source_timestamp_ns", "bridge_send_monotonic_ns", "left_pose", "right_pose",
                "upper_limb_skeleton_valid", "upper_limb_rotations_valid", "upper_limb_points",
                "upper_limb_rotations_xyzw", "raw_packet_base64", "raw_packet_size",
                "received_timestamp_ns", "receiver_instance_id", "receiver_frame_sequence",
                "association_id",
            },
            "legacy PICO palm frame",
        )
        raw_packet = _base64_bytes(item["raw_packet_base64"], "raw_packet_base64")
        if item["raw_packet_size"] != len(raw_packet):
            raise ValueError("legacy raw_packet_size does not match decoded packet")
        frame = cls(
            protocol_version=item["protocol_version"],
            packet_size=item["packet_size"],
            flags=item["flags"],
            sequence=item["sequence"],
            tracking_epoch=item["tracking_epoch"],
            source_timestamp_ns=item["source_timestamp_ns"],
            bridge_send_monotonic_ns=item["bridge_send_monotonic_ns"],
            left_pose=item["left_pose"],
            right_pose=item["right_pose"],
            upper_limb_skeleton_valid=item["upper_limb_skeleton_valid"],
            upper_limb_rotations_valid=item["upper_limb_rotations_valid"],
            upper_limb_points=item["upper_limb_points"],
            upper_limb_rotations_xyzw=item["upper_limb_rotations_xyzw"],
            raw_packet=raw_packet,
            received_timestamp_ns=item["received_timestamp_ns"],
            receiver_instance_id=item["receiver_instance_id"],
            receiver_frame_sequence=item["receiver_frame_sequence"],
        )
        if item["association_id"] != frame.association_id:
            raise ValueError("legacy raw frame association_id does not match metadata")
        return frame


@dataclass(frozen=True)
class ManusRawFrame:
    """One raw JSON Manus skeleton frame for one glove."""

    glove_id: str
    side: str
    source_sequence: int
    source_monotonic_ns: int
    sdk_publish_time: int
    node_positions: np.ndarray
    node_quaternions_wxyz: np.ndarray
    node_semantics: tuple[Mapping[str, Any], ...]
    received_timestamp_ns: int
    receiver_instance_id: str
    receiver_frame_sequence: int

    def __post_init__(self) -> None:
        _required_string(self.glove_id, "glove_id")
        if self.side not in SIDES:
            raise ValueError("side must be left or right")
        _nonnegative_int(self.source_sequence, "source_sequence")
        _nonnegative_int(self.source_monotonic_ns, "source_monotonic_ns")
        _nonnegative_int(self.sdk_publish_time, "sdk_publish_time")
        positions = np.asarray(self.node_positions, dtype=np.float64)
        quaternions = np.asarray(self.node_quaternions_wxyz, dtype=np.float64)
        if positions.ndim != 2 or positions.shape[1] != 3 or not 1 <= positions.shape[0] <= 64:
            raise ValueError("node_positions must have shape (1..64, 3)")
        if quaternions.shape != (positions.shape[0], 4) or not np.isfinite(quaternions).all():
            raise ValueError("node_quaternions_wxyz must be finite with one quaternion per node")
        if not np.isfinite(positions).all():
            raise ValueError("node_positions must be finite")
        if not isinstance(self.node_semantics, (list, tuple)):
            raise ValueError("node_semantics must be an array")
        semantics_list: list[dict[str, Any]] = []
        for index, item in enumerate(self.node_semantics):
            if not isinstance(item, Mapping):
                raise ValueError(f"node_semantics[{index}] must be an object")
            semantics_list.append(dict(item))
        semantics = tuple(semantics_list)
        for index, item in enumerate(semantics):
            required = {"array_index", "node_id", "parent_id", "chain_type", "side", "finger_joint_type"}
            if set(item) != required:
                raise ValueError(f"node_semantics[{index}] has an invalid field set")
            for field in ("array_index", "node_id", "parent_id", "chain_type", "side", "finger_joint_type"):
                if isinstance(item[field], bool) or not isinstance(item[field], (int, np.integer)):
                    raise ValueError(f"node_semantics[{index}].{field} must be an integer")
            if not 0 <= int(item["array_index"]) < positions.shape[0]:
                raise ValueError(f"node_semantics[{index}].array_index is out of range")
        _nonnegative_int(self.received_timestamp_ns, "received_timestamp_ns")
        _required_string(self.receiver_instance_id, "receiver_instance_id")
        _nonnegative_int(self.receiver_frame_sequence, "receiver_frame_sequence")
        object.__setattr__(self, "node_positions", positions.copy())
        object.__setattr__(self, "node_quaternions_wxyz", quaternions.copy())
        object.__setattr__(self, "node_semantics", semantics)

    @property
    def association_id(self) -> str:
        return f"{self.receiver_instance_id}:{self.receiver_frame_sequence}"

    def to_dict(self) -> dict[str, Any]:
        return {
            "glove_id": self.glove_id,
            "side": self.side,
            "seq": self.source_sequence,
            "source_monotonic_ns": self.source_monotonic_ns,
            "sdk_publish_time": self.sdk_publish_time,
            "nodes": self.node_positions.tolist(),
            "node_quaternions_wxyz": self.node_quaternions_wxyz.tolist(),
            "node_semantics": [dict(item) for item in self.node_semantics],
            "received_timestamp_ns": self.received_timestamp_ns,
            "receiver_instance_id": self.receiver_instance_id,
            "receiver_frame_sequence": self.receiver_frame_sequence,
            "association_id": self.association_id,
        }

    @classmethod
    def from_dict(cls, value: Any) -> "ManusRawFrame":
        item = _object(value, "Manus raw frame")
        _exact_keys(
            item,
            {
                "glove_id", "side", "seq", "source_monotonic_ns", "sdk_publish_time", "nodes",
                "node_quaternions_wxyz", "node_semantics", "received_timestamp_ns",
                "receiver_instance_id", "receiver_frame_sequence", "association_id",
            },
            "Manus raw frame",
        )
        frame = cls(
            glove_id=item["glove_id"],
            side=item["side"],
            source_sequence=item["seq"],
            source_monotonic_ns=item["source_monotonic_ns"],
            sdk_publish_time=item["sdk_publish_time"],
            node_positions=item["nodes"],
            node_quaternions_wxyz=item["node_quaternions_wxyz"],
            node_semantics=tuple(item["node_semantics"]),
            received_timestamp_ns=item["received_timestamp_ns"],
            receiver_instance_id=item["receiver_instance_id"],
            receiver_frame_sequence=item["receiver_frame_sequence"],
        )
        if item["association_id"] != frame.association_id:
            raise ValueError("Manus raw frame association_id does not match metadata")
        return frame


__all__ = [
    "ArmInputObservation",
    "HandObservation",
    "PICO_HEAD_CURRENT_FRAME",
    "PICO_HEAD_MAPPING_VERSION",
    "PICO_JOINT_NAMES",
    "PICO_MAPPING_VERSION",
    "PICO_TRACKING_FRAME",
    "PicoRawFrame",
    "PicoRawHand",
    "PicoRawJoint",
    "LegacyPicoPalmFrame",
    "ManusRawFrame",
    "SIDES",
]
