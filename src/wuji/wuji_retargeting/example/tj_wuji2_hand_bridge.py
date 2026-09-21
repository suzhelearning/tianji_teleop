#!/usr/bin/env python3
"""ROS2 /hand_input -> official Manus/Hand2 retargeting -> simulation-only TJH2 UDP.

Input coordinates are right-handed world XYZ in meters, ordered as MediaPipe
landmarks; neither adapter nor bridge reflects them. Float32MultiArray has no source timestamp:
TJH2 per-side source timestamps are callback receive-time monotonic clocks,
not glove acquisition times. A 100 Hz publisher repeats the latest bilateral
targets with their original source ages, independently of retargeting callbacks.
The packet's source_timestamp_ns is the monotonic time the publication samples
that cache; repetition never changes either side's receive-time timestamp.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import socket
import sys
import threading
import time

import numpy as np

from tianji import hand_protocol


PUBLICATION_HZ = 100
PUBLICATION_PERIOD_NS = 1_000_000_000 // PUBLICATION_HZ


def _side(side: str) -> str:
    if side not in ("left", "right"):
        raise ValueError(f"unknown hand side {side!r}; expected 'left' or 'right'")
    return side


def _keypoints(points) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)
    if points.shape != (21, 3) or not np.isfinite(points).all():
        raise ValueError("hand landmarks must be finite (21, 3) XYZ coordinates")
    # Retargeter's wrist-frame SVD needs non-collinear wrist/index/middle MCPs.
    # Reject absent/all-zero hands rather than allowing an arbitrary rest pose.
    index = points[5] - points[0]
    middle = points[9] - points[0]
    if np.linalg.norm(np.cross(index, middle)) <= 1e-12:
        raise ValueError("degenerate wrist/index/middle palm frame")
    return points


def interpret_hand_input(message, single_hand_side: str = "right") -> dict[str, np.ndarray]:
    """Decode labeled63, legacy unlabeled63, or unlabeled126 (right then left).

    Accept an empty ROS layout or one flat dimension with matching size/stride.
    A side label always denotes exactly one hand. Invalid messages raise
    ValueError before either retargeter is touched.
    """
    _side(single_hand_side)
    data = np.asarray(message.data, dtype=np.float64)
    if data.ndim != 1 or data.size not in (63, 126) or not np.isfinite(data).all():
        raise ValueError("/hand_input requires exactly 63 or 126 finite floats")
    layout = message.layout
    if layout.data_offset != 0 or len(layout.dim) > 1:
        raise ValueError("/hand_input requires a flat layout with data_offset=0")
    label = ""
    if layout.dim:
        dimension = layout.dim[0]
        label = dimension.label
        if dimension.size != data.size or dimension.stride != data.size:
            raise ValueError("/hand_input layout size and stride must match the flat payload")
    if label not in ("", "left", "right"):
        raise ValueError(f"invalid /hand_input side label {label!r}")
    if data.size == 63:
        return {label or single_hand_side: _keypoints(data.reshape(21, 3))}
    if label:
        raise ValueError("a labeled hand must contain 63 floats, not 126")
    return {"right": _keypoints(data[:63].reshape(21, 3)),
            "left": _keypoints(data[63:].reshape(21, 3))}


def joint_permutation(joint_names, side: str) -> np.ndarray:
    """Map the actual optimizer joint names into the C++ viewer's 20 slots."""
    prefix = "l_" if _side(side) == "left" else "r_"
    expected = tuple(prefix + stem for stem in hand_protocol.JOINT_STEMS)
    names = tuple(joint_names)
    if len(names) != 20 or len(set(names)) != 20 or set(names) != set(expected):
        raise ValueError(f"{side} optimizer joints do not match the official Hand2 TJH2 joints")
    indices = {name: index for index, name in enumerate(names)}
    return np.array([indices[name] for name in expected], dtype=np.intp)


class HandTargetCache:
    """Two immutable latest results; solving and encoding never hold the lock."""

    def __init__(self, *, clock=time.monotonic_ns):
        self._clock = clock
        # The receiver retains its high-water mark across sender restarts.
        self._sequence = clock()
        self._lock = threading.Lock()
        self._targets = {"left": (None, 0), "right": (None, 0)}

    def retarget(self, side, model, points, received_ns):
        _side(side)
        joints = np.asarray(model.retarget(points), dtype=np.float64)
        if joints.shape != (20,) or not np.isfinite(joints).all():
            raise ValueError("Hand2 optimization produced invalid joint angles")
        target = (tuple(joints), received_ns)
        with self._lock:
            self._targets[side] = target

    def sample(self):
        with self._lock:
            left, left_ns = self._targets["left"]
            right, right_ns = self._targets["right"]
            if left is None and right is None:
                return None
            timestamp_ns = self._clock()
            self._sequence += 1
            sequence = self._sequence
        return hand_protocol.encode_packet(
            sequence, timestamp_ns, left=left, right=right,
            left_timestamp_ns=left_ns, right_timestamp_ns=right_ns,
        )


def publish_targets(cache, udp, address, stop):
    """Run one bounded publisher; missed deadlines never produce catch-up bursts."""
    deadline_ns = time.monotonic_ns()
    while not stop.wait(max(0, deadline_ns - time.monotonic_ns()) / 1_000_000_000):
        packet = cache.sample()
        if packet is not None:
            udp.sendto(packet, address)
        deadline_ns += PUBLICATION_PERIOD_NS
        now_ns = time.monotonic_ns()
        if deadline_ns <= now_ns:
            deadline_ns = now_ns + PUBLICATION_PERIOD_NS


class HandRetargeter:
    """One independent official Hand2 optimizer and filter, with named output order."""

    def __init__(self, side: str):
        _side(side)
        from wuji_retargeting import Retargeter
        from scipy.spatial.transform import Rotation
        from wuji_retargeting.mediapipe import (
            OPERATOR2MANO_LEFT, OPERATOR2MANO_RIGHT, estimate_frame_from_hand_points,
        )

        # The Hand2 config ships as package data beside this module.
        config = Path(__file__).resolve().parent / "config" / f"retarget_manus_wuji_hand_2_{side}.yaml"
        self.retargeter = Retargeter.from_yaml(str(config), hand_side=side)
        robot = self.retargeter.optimizer.robot
        self.permutation = joint_permutation(robot.dof_joint_names, side)
        self.limits = np.asarray(robot.joint_limits, dtype=np.float64)
        if (self.limits.shape != (20, 2) or not np.isfinite(self.limits).all()
                or np.any(self.limits[:, 0] >= self.limits[:, 1])):
            raise ValueError(f"{side} Hand2 model has invalid joint limits")

        # Canonical MANO axes are not the Hand2 URDF's axes. Derive their
        # fixed rotation from this model's neutral wrist/MCP landmarks once,
        # rather than borrowing hand-one or input-device Euler corrections.
        prefix = side[0] + "_"
        links = [prefix + "wrist"]
        for finger in ("thumb", "index_finger", "middle_finger", "ring_finger", "pinky"):
            links.extend(prefix + finger + "_" + link
                         for link in ("proximal_abd", "middle", "distal", "tip"))
        robot.compute_forward_kinematics(np.zeros(20))
        neutral = np.array([robot.get_link_pose(robot.get_link_index(link))[:3, 3]
                            for link in links])
        operator = OPERATOR2MANO_LEFT if side == "left" else OPERATOR2MANO_RIGHT
        model_to_canonical = estimate_frame_from_hand_points(neutral) @ operator
        # Retargeter rotates row vectors by R.T, undoing the canonical basis.
        self.retargeter.rotation_xyz = dict(zip(
            ("x", "y", "z"), Rotation.from_matrix(model_to_canonical).as_euler("xyz", degrees=True)
        ))

    def retarget(self, points) -> np.ndarray:
        points = _keypoints(points)
        # The library catches NLopt RuntimeError and returns its initial qpos.
        # Inspect NLopt's status before filtering/sending, never forward fallback.
        qpos = np.asarray(self.retargeter.retarget(points, apply_filter=False), dtype=np.float64)
        if self.retargeter.optimizer.opt.last_optimize_result() <= 0:
            raise ValueError("Hand2 optimization failed; retaining viewer targets")
        if qpos.shape != (20,) or not np.isfinite(qpos).all():
            raise ValueError("Hand2 optimization produced invalid joint angles")
        lower, upper = self.limits[:, 0], self.limits[:, 1]
        if np.any(qpos < lower - 1e-6) or np.any(qpos > upper + 1e-6):
            raise ValueError("Hand2 optimization exceeded model joint limits")
        # The official optimizer returns float32; trim only rounding at bounds.
        qpos = np.clip(qpos, lower, upper)
        qpos = self.retargeter.lp_filter.next(qpos)
        return qpos[self.permutation]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="simulation TJH2 receiver host")
    parser.add_argument("--port", type=int, default=16000, help="simulation TJH2 UDP port")
    parser.add_argument("--single-hand-side", choices=("left", "right"), default="right",
                        help="side for legacy unlabeled 63-float input (default: right)")
    parser.add_argument("--topic", default="/hand_input", help="Float32MultiArray input topic")
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float32MultiArray

    # Fail before subscribing if either official model/config is unavailable.
    models = {side: HandRetargeter(side) for side in ("left", "right")}
    # Resolve once before starting the publisher, never on its bounded send path.
    address = (socket.gethostbyname(args.host), args.port)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
        # A stalled receiver/network must not block the publication or shutdown thread.
        udp.setblocking(False)
        rclpy.init(args=[])
        node = None
        try:
            class HandBridge(Node):
                def __init__(self):
                    super().__init__("tj_wuji2_hand_bridge")
                    self.cache = HandTargetCache()
                    self.publisher_error = None
                    self.publisher_guard = self.create_guard_condition(self.check_publisher)
                    qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                                     durability=DurabilityPolicy.VOLATILE)
                    self.subscription = self.create_subscription(
                        Float32MultiArray, args.topic, self.on_hand_input, qos)
                    self.get_logger().info(
                        f"Waiting for {args.topic}; Hand2 TJH2 -> {args.host}:{args.port}; "
                        "100 Hz bilateral samples; per-side timestamps=receive monotonic ns")

                def check_publisher(self):
                    if self.publisher_error is not None:
                        raise RuntimeError("TJH2 publisher failed") from self.publisher_error

                def on_hand_input(self, message):
                    received_ns = time.monotonic_ns()
                    try:
                        hands = interpret_hand_input(message, args.single_hand_side)
                    except (ValueError, TypeError, OverflowError) as error:
                        self.get_logger().warning(f"Rejected /hand_input: {error}", throttle_duration_sec=2.0)
                        return
                    self.check_publisher()
                    for side, points in hands.items():
                        try:
                            self.cache.retarget(side, models[side], points, received_ns)
                        except (ValueError, RuntimeError, np.linalg.LinAlgError) as error:
                            self.get_logger().warning(f"Rejected {side} solve: {error}", throttle_duration_sec=2.0)

            node = HandBridge()
            stop = threading.Event()

            def publish():
                try:
                    publish_targets(node.cache, udp, address, stop)
                except BaseException as error:
                    node.publisher_error = error
                    node.publisher_guard.trigger()

            publisher = threading.Thread(target=publish, name="tjh2-publisher")
            try:
                publisher.start()
                rclpy.spin(node)
            finally:
                stop.set()
                if publisher.ident is not None:
                    publisher.join()
                node.check_publisher()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
