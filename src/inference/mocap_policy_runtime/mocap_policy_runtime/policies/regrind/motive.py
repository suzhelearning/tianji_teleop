"""Strict NatNet/Motive frames shared by H5 and diagnostics only."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping

import numpy as np


@dataclass(frozen=True)
class MotiveRigidBody:
    rigid_id: int
    tracking_valid: bool
    pose: np.ndarray | None


@dataclass(frozen=True)
class MotiveFrame:
    frame_number: int
    rigid_bodies: tuple[MotiveRigidBody, ...]
    names: dict[int, str]

    def rigid_pose(self, rigid_id: int) -> np.ndarray | None:
        for body in self.rigid_bodies:
            if body.rigid_id == rigid_id:
                return None if body.pose is None else body.pose.copy()
        return None

    def rigid_name(self, rigid_id: int) -> str | None:
        return self.names.get(rigid_id)


_FRAME_FIELDS = {
    "schema_version",
    "frame_number",
    "motive_timestamp",
    "publisher_received_time_ns",
    "coordinate_system",
    "unit",
    "publisher_dropped_frames",
    "markers",
    "rigid_bodies",
}
_RIGID_FIELDS = {
    "id", "position", "quaternion_xyzw", "mean_error", "tracking_valid"
}
_MARKER_FIELDS = {
    "position",
    "raw_id",
    "model_id",
    "member_id",
    "id_kind",
    "size",
    "residual_m_per_ray",
    "occluded",
    "point_cloud_solved",
    "model_filled",
    "has_model",
    "unlabeled",
    "active",
    "established",
    "measurement",
}
_MARKER_ID_KINDS = {"active", "asset_member", "point_cloud", "unknown"}


def _integer(value: Any, field: str, *, nonnegative: bool = False) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{field} must be an integer")
    if nonnegative and value < 0:
        raise ValueError(f"{field} must be non-negative")
    return value


def _finite(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must be a finite number")
    return result


def _vector(value: Any, size: int, field: str) -> list[float]:
    if (
        not isinstance(value, (list, tuple))
        or len(value) != size
        or any(isinstance(item, bool) for item in value)
    ):
        raise ValueError(f"{field} must contain {size} numbers")
    return [_finite(item, f"{field}[{index}]") for index, item in enumerate(value)]


def _fields(value: Mapping[str, Any], expected: set[str], field: str) -> None:
    missing = expected - set(value)
    extra = set(value) - expected
    if missing:
        raise ValueError(f"{field} missing fields: {', '.join(sorted(missing))}")
    if extra:
        raise ValueError(f"{field} unknown fields: {', '.join(sorted(extra))}")


class MotiveFrameSource:
    """Validate the complete ``mocap/hands/frame`` producer envelope."""

    @staticmethod
    def parse_names(value: Any) -> dict[int, str]:
        """Parse the names topic, preserving canonical decimal JSON ids."""
        if not isinstance(value, Mapping) or set(value) != {"names"}:
            raise ValueError("Motive names must contain exactly the names field")
        names_value = value["names"]
        if not isinstance(names_value, Mapping):
            raise ValueError("Motive names must be an object")
        result: dict[int, str] = {}
        seen_names: set[str] = set()
        for raw_id, name in names_value.items():
            if isinstance(raw_id, bool):
                raise ValueError("Motive rigid body name id must be canonical")
            if isinstance(raw_id, int):
                rigid_id = raw_id
            elif (
                isinstance(raw_id, str)
                and raw_id.isdecimal()
                and str(int(raw_id)) == raw_id
            ):
                rigid_id = int(raw_id)
            else:
                raise ValueError("Motive rigid body name id must be canonical decimal")
            if rigid_id <= 0 or rigid_id in result:
                raise ValueError("Motive rigid body names contain duplicate id")
            if not isinstance(name, str) or not name or name in seen_names:
                raise ValueError("Motive rigid body names contain duplicate/invalid name")
            result[rigid_id] = name
            seen_names.add(name)
        return result

    @staticmethod
    def _parse_marker(item: Any, index: int) -> None:
        field = f"markers[{index}]"
        if not isinstance(item, Mapping):
            raise ValueError(f"{field} must be an object")
        _fields(item, _MARKER_FIELDS, field)
        _vector(item["position"], 3, f"{field}.position")
        for name in ("raw_id", "model_id", "member_id"):
            _integer(item[name], f"{field}.{name}")
        if item["id_kind"] not in _MARKER_ID_KINDS:
            raise ValueError(f"{field}.id_kind is not supported")
        _finite(item["size"], f"{field}.size")
        _finite(item["residual_m_per_ray"], f"{field}.residual_m_per_ray")
        for name in (
            "occluded", "point_cloud_solved", "model_filled", "has_model",
            "unlabeled", "active", "established", "measurement",
        ):
            if not isinstance(item[name], bool):
                raise ValueError(f"{field}.{name} must be a boolean")

    @staticmethod
    def _parse_rigid_body(item: Any, index: int) -> MotiveRigidBody:
        field = f"rigid_bodies[{index}]"
        if not isinstance(item, Mapping):
            raise ValueError(f"{field} must be an object")
        _fields(item, _RIGID_FIELDS, field)
        rigid_id = _integer(item["id"], f"{field}.id")
        if rigid_id <= 0:
            raise ValueError(f"{field}.id must be positive")
        position = _vector(item["position"], 3, f"{field}.position")
        quaternion = _vector(item["quaternion_xyzw"], 4, f"{field}.quaternion_xyzw")
        _finite(item["mean_error"], f"{field}.mean_error")
        valid = item["tracking_valid"]
        if not isinstance(valid, bool):
            raise ValueError(f"{field}.tracking_valid must be a boolean")
        values = np.asarray(position + quaternion, dtype=np.float64)
        norm = float(np.linalg.norm(values[3:]))
        if not 0.999 <= norm <= 1.001:
            raise ValueError(f"{field}.quaternion_xyzw must be normalized")
        values[3:] /= norm
        return MotiveRigidBody(rigid_id, valid, values if valid else None)

    def parse(self, payload: Mapping[str, Any]) -> MotiveFrame:
        if not isinstance(payload, Mapping):
            raise ValueError("Motive frame must be an object")
        _fields(payload, _FRAME_FIELDS, "Motive frame")
        schema_version = _integer(payload["schema_version"], "schema_version")
        if schema_version != 1:
            raise ValueError("schema_version must be 1")
        frame_number = _integer(payload["frame_number"], "frame_number", nonnegative=True)
        _finite(payload["motive_timestamp"], "motive_timestamp")
        _integer(payload["publisher_received_time_ns"], "publisher_received_time_ns")
        if payload["coordinate_system"] != "motive_x_forward_z_up_right_handed":
            raise ValueError("coordinate_system is unsupported")
        if payload["unit"] != "meter":
            raise ValueError("unit must be meter")
        _integer(
            payload["publisher_dropped_frames"],
            "publisher_dropped_frames",
            nonnegative=True,
        )
        markers = payload["markers"]
        if not isinstance(markers, list):
            raise ValueError("markers must be a list")
        for index, item in enumerate(markers):
            self._parse_marker(item, index)
        bodies = payload["rigid_bodies"]
        if not isinstance(bodies, list):
            raise ValueError("Motive rigid_bodies must be a list")
        parsed: list[MotiveRigidBody] = []
        seen_ids: set[int] = set()
        for index, item in enumerate(bodies):
            body = self._parse_rigid_body(item, index)
            if body.rigid_id in seen_ids:
                raise ValueError("Motive rigid bodies contain duplicate id")
            seen_ids.add(body.rigid_id)
            parsed.append(body)
        return MotiveFrame(frame_number, tuple(parsed), {})


__all__ = ["MotiveFrame", "MotiveFrameSource", "MotiveRigidBody"]
