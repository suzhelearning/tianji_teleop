"""Source hand skeleton and fixed Motive world axes in a MuJoCo user scene."""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from ..data.geometry import compose_pose
from ..integration.native import pose_xyzw


HAND_EDGES = tuple((0, 1 + 4 * finger) for finger in range(5)) + tuple(
    (1 + 4 * finger + joint, 2 + 4 * finger + joint)
    for finger in range(5) for joint in range(3)
)
AXIS_LENGTH_M = 0.2


def build_object_overlays(target_poses, sample, world_transform, *, now: float, stale_s: float, error=None):
    """Map independent source targets and fresh tracker objects into robot world.

    Objects already use Motive axes: only the shared world calibration applies,
    never the Manus-to-WuJi wrist conversion. Timestamps describe acquisition,
    not publication, so a receiver can also expire a still-visible snapshot.
    """
    if not np.isfinite(now) or not np.isfinite(stale_s) or stale_s <= 0:
        raise ValueError("object overlay clock and positive timeout must be finite")
    transform = pose_xyzw(world_transform, "object overlay world transform")
    timeout_ns = max(1, round(stale_s * 1e9))
    timestamp_ns = None
    actual = None
    status = "UNAVAILABLE"
    if sample is not None and not error:
        stamp = sample.received_at
        if np.isfinite(stamp) and stamp > 0:
            timestamp_ns = max(1, round(stamp * 1e9))
            if not 0 <= now - stamp <= stale_s:
                status = "STALE"
            elif sample.hammer_xyzw is None:
                status = "UNTRACKED"
            else:
                actual = compose_pose(transform, pose_xyzw(sample.hammer_xyzw, "tracked hammer"))
                status = "TRACKED"
    objects = {}
    for name in sorted(set(target_poses) | {"hammer"}):
        target = target_poses.get(name)
        objects[name] = {
            "target": None if target is None else compose_pose(
                transform, pose_xyzw(target, f"target object {name}")),
            "actual": actual if name == "hammer" else None,
            "status": status if name == "hammer" else "UNAVAILABLE",
            "timestamp_ns": timestamp_ns if name == "hammer" else None,
            "timeout_ns": timeout_ns,
        }
    return objects


def overlay_geometry(preview, world_transform):
    """Return displayed points, origin, XYZ endpoints in the same robot world.

    Use the caller's provisional or locked world calibration, shared with
    control; do not independently align the skeleton or the Motive axes.
    """
    transform = pose_xyzw(world_transform, "overlay world transform")
    rotation = Rotation.from_quat(transform[3:])
    points = np.empty((0, 3))
    if "right" in preview.hand_keypoints:
        wrist = compose_pose(transform, preview.wrist_poses["right"])
        local = np.asarray(preview.hand_keypoints["right"], dtype=float)
        if local.shape != (21, 3) or not np.isfinite(local).all():
            raise ValueError("source skeleton must contain finite 21x3 local points")
        points = Rotation.from_quat(wrist[3:]).apply(local) + wrist[:3]
    origin = transform[:3].copy()
    endpoints = rotation.apply(np.eye(3) * AXIS_LENGTH_M) + origin
    return points, origin, endpoints


def draw_mocap_overlay(scene, preview, world_transform, *, clear=True, draw_axes=True):
    """Draw source skeleton, optionally its world axes, after update_scene()."""
    import mujoco

    points, origin, endpoints = overlay_geometry(preview, world_transform)
    if clear:
        scene.ngeom = 0
    edges = HAND_EDGES if len(points) else ()
    required = len(edges) + len(points) + (7 if draw_axes else 0)
    if scene.ngeom + required > scene.maxgeom:
        raise ValueError("MuJoCo scene has insufficient capacity for source skeleton/axes")

    def sphere(position, radius, color, label=""):
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE,
                           np.array([radius, 0., 0.]), position, np.eye(3).ravel(), np.asarray(color, dtype=np.float32))
        geom.label = label
        scene.ngeom += 1

    def connector(first, second, radius, color, kind):
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, kind, np.zeros(3), np.zeros(3), np.eye(3).ravel(), np.asarray(color, dtype=np.float32))
        mujoco.mjv_connector(geom, kind, radius, first, second)
        scene.ngeom += 1

    for first, second in edges:
        connector(points[first], points[second], 0.0015, (1., 0.7, 0.15, 1.), mujoco.mjtGeom.mjGEOM_CAPSULE)
    for point in points:
        sphere(point, 0.0025, (1., 0.85, 0.2, 1.))
    if draw_axes:
        sphere(origin, 0.004, (1., 1., 1., 1.), "Mocap O")
        for endpoint, color, label in zip(endpoints, ((1., 0., 0., 1.), (0., 1., 0., 1.), (0., 0., 1., 1.)), ("Mocap X", "Mocap Y", "Mocap Z")):
            connector(origin, endpoint, 0.004, color, mujoco.mjtGeom.mjGEOM_ARROW)
            sphere(endpoint, 0.003, color, label)
