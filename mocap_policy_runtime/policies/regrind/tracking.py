"""Read-only Motive subscriptions and calibrated Regrind wrist/hammer poses."""
from __future__ import annotations

from dataclasses import dataclass
import json
import threading
import time
from typing import Any

import numpy as np

from ...data.geometry import compose_pose
from .motive import MotiveFrame, MotiveFrameSource


MOCAP_HANDS_FRAME = "mocap/hands/frame"
MOCAP_RIGID_BODY_NAMES = "mocap/rigid_body_names"
HAMMER_RIGID_TO_OBJECT = np.asarray(
    [0.002, -0.005, 0.0, 0.7071067811865476, 0.0, 0.0, 0.7071067811865476]
)
WRIST_RIGID_TO_MARKER = np.asarray(
    [0.0, -0.004, 0.004, -0.008678730623841365, -0.10452448314406536,
     -0.0009121713452956539, 0.9944840270218831]
)
MARKER_TO_MOUNT = np.asarray(
    [0.004, 0.0, 0.0, 0.0, -0.7071067811865476, 0.0, 0.7071067811865476]
)
MOUNT_TO_WRIST = np.asarray(
    [0.003, 0.00025016, -0.0285, 0.0, 0.0, 0.0000081994999999, 0.9999999999663841]
)


def open_mocap_session(endpoint: str) -> Any:
    """Connect only to the requested router; declare no control publishers."""
    import zenoh

    endpoint = endpoint.strip()
    if not endpoint:
        raise ValueError("Motive router endpoint must not be empty")
    config = zenoh.Config.from_json5(json.dumps({
        "mode": "client", "connect": {"endpoints": [endpoint]},
        "scouting": {"multicast": {"enabled": False}},
    }))
    return zenoh.open(config)


def _unique_fields(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate field: {key}")
        result[key] = value
    return result


def _reject_constant(value: str) -> Any:
    raise ValueError(f"non-finite JSON constant: {value}")


@dataclass(frozen=True)
class RegrindMotiveSample:
    frame_number: int
    received_at: float
    wrist_xyzw: np.ndarray
    hammer_xyzw: np.ndarray | None


class RegrindMotiveTracker:
    """Strict wire parser; callers must admit both ``error`` and sample age.

    Parser errors remain latched for the session, matching the real policy's
    fail-closed admission. ``received_at`` is local monotonic receive time.
    ``require_object=False`` is reserved for wrist-only H5 playback: it relaxes
    object presence/tracking, never the strict validation of received payloads.
    """

    def __init__(
        self, session: Any, *, wrist_name: str = "right_wrist",
        hammer_name: str = "hammer", rigid_to_wrist: np.ndarray | None = None,
        rigid_to_object: np.ndarray | None = None, require_object: bool = True,
    ) -> None:
        self._parser = MotiveFrameSource()
        self._wrist_name = wrist_name
        self._hammer_name = hammer_name
        self._require_object = require_object
        self._rigid_to_wrist = (
            compose_pose(compose_pose(WRIST_RIGID_TO_MARKER, MARKER_TO_MOUNT), MOUNT_TO_WRIST)
            if rigid_to_wrist is None else np.asarray(rigid_to_wrist, dtype=np.float64)
        )
        self._rigid_to_object = (
            HAMMER_RIGID_TO_OBJECT.copy()
            if rigid_to_object is None else np.asarray(rigid_to_object, dtype=np.float64)
        )
        self._lock = threading.Lock()
        self._names: dict[int, str] = {}
        self._frame: MotiveFrame | None = None
        self._received_at = 0.0
        self._error: str | None = None
        self._names_sub = session.declare_subscriber(
            MOCAP_RIGID_BODY_NAMES, self._decode(self._on_names),
        )
        try:
            self._frame_sub = session.declare_subscriber(
                MOCAP_HANDS_FRAME, self._decode(self._on_frame),
            )
        except BaseException:
            self._names_sub.undeclare()
            raise

    def _decode(self, handler):
        def on_sample(sample) -> None:
            try:
                payload = json.loads(
                    bytes(sample.payload), object_pairs_hook=_unique_fields,
                    parse_constant=_reject_constant,
                )
                handler(payload)
            except (TypeError, ValueError, UnicodeError) as exc:
                with self._lock:
                    self._error = str(exc)
        return on_sample

    @property
    def error(self) -> str | None:
        with self._lock:
            return self._error

    def _on_names(self, payload: Any) -> None:
        try:
            names = self._parser.parse_names(payload)
            required = (self._wrist_name, self._hammer_name) if self._require_object else (self._wrist_name,)
            for wanted in required:
                if list(names.values()).count(wanted) != 1:
                    raise ValueError(f"Motive must contain exactly one rigid body named {wanted!r}")
        except (TypeError, ValueError) as exc:
            with self._lock:
                self._error = str(exc)
            return
        with self._lock:
            self._names = names

    def _on_frame(self, payload: Any) -> None:
        try:
            frame = self._parser.parse(payload)
        except (TypeError, ValueError) as exc:
            with self._lock:
                self._error = str(exc)
            return
        with self._lock:
            self._frame = frame
            self._received_at = time.monotonic()

    def latest(self) -> RegrindMotiveSample | None:
        with self._lock:
            frame, names, received_at = self._frame, self._names, self._received_at
        if frame is None:
            return None
        ids = {name: rigid_id for rigid_id, name in names.items()}
        if self._wrist_name not in ids:
            return None
        wrist_rigid = frame.rigid_pose(ids[self._wrist_name])
        hammer_id = ids.get(self._hammer_name)
        hammer_rigid = frame.rigid_pose(hammer_id) if hammer_id is not None else None
        if wrist_rigid is None or (self._require_object and hammer_rigid is None):
            return None
        return RegrindMotiveSample(
            frame.frame_number, received_at,
            compose_pose(wrist_rigid, self._rigid_to_wrist),
            compose_pose(hammer_rigid, self._rigid_to_object) if hammer_rigid is not None else None,
        )

    def close(self) -> None:
        try:
            self._names_sub.undeclare()
        finally:
            self._frame_sub.undeclare()
