#!/usr/bin/env python3
"""Manus Integrated SDK text stream -> independent labeled ROS2 hand frames.

NODE array_index describes the POSE order, NOT an SDK node ID. Semantic values
below are from the bundled ManusSDK/include/ManusSDKTypes.h. SDK VUH positions
are right-handed world-space meters and retain their original handedness.

Source timestamps must share this process's Linux CLOCK_MONOTONIC clock/boot.
Recordings need current monotonic timestamps to pass freshness checks. No frame
is emitted on missing, ambiguous, invalid or stale input (including EOF).
"""

import argparse
from dataclasses import dataclass, field
from enum import IntEnum
import math
import select
import sys
import time


class ChainType(IntEnum):
    THUMB = 5
    INDEX = 6
    MIDDLE = 7
    RING = 8
    PINKY = 9
    HAND = 13


class FingerJointType(IntEnum):
    INVALID = 0
    METACARPAL = 1
    PROXIMAL = 2
    INTERMEDIATE = 3
    DISTAL = 4
    TIP = 5


SIDES = {"Left": ("left", 1), "Right": ("right", 2)}
MAX_NODES = 64
FLOAT32_MAX = 3.4028234663852886e38


@dataclass(frozen=True)
class HandFrame:
    side: str
    data: tuple[float, ...]
    sequence: int
    source_monotonic_ns: int


@dataclass(frozen=True)
class _Node:
    node_id: int
    parent_id: int
    chain: ChainType
    side: int
    joint: FingerJointType


@dataclass
class _Hand:
    side: str
    sdk_side: int
    count: int
    nodes: dict[int, _Node] = field(default_factory=dict)
    order: tuple[int, ...] | None = None
    invalid: bool = False
    latest_valid_ns: int = 0


def _keypoint_order(hand: _Hand) -> tuple[int, ...]:
    """Resolve semantic slots and check parent links, never guessing array IDs."""
    by_id = {}
    by_semantic = {}
    for index, node in hand.nodes.items():
        semantic = (node.chain, node.joint)
        if node.side != hand.sdk_side or node.node_id in by_id or semantic in by_semantic:
            raise ValueError("ambiguous node identity, side or semantic joint")
        by_id[node.node_id] = node
        by_semantic[semantic] = index
    wrist_index = by_semantic[(ChainType.HAND, FingerJointType.INVALID)]
    wrist = hand.nodes[wrist_index]
    if wrist.parent_id not in (0, wrist.node_id):
        raise ValueError("wrist must be the raw hand root")
    wanted = [wrist_index]
    allowed = {(ChainType.HAND, FingerJointType.INVALID)}
    for chain in (ChainType.THUMB, ChainType.INDEX, ChainType.MIDDLE, ChainType.RING, ChainType.PINKY):
        if chain == ChainType.THUMB:
            # SDK headers call thumb IP Intermediate; live Metaglove Pro
            # metadata calls it Distal. Exactly one must connect MCP to tip.
            ip = [joint for joint in (FingerJointType.INTERMEDIATE, FingerJointType.DISTAL)
                  if (chain, joint) in by_semantic]
            if len(ip) != 1:
                raise ValueError("thumb must have exactly one IP joint")
            joints = (FingerJointType.METACARPAL, FingerJointType.PROXIMAL,
                      ip[0], FingerJointType.TIP)
        else:
            joints = (FingerJointType.PROXIMAL, FingerJointType.INTERMEDIATE,
                      FingerJointType.DISTAL, FingerJointType.TIP)
        wanted.extend(by_semantic[(chain, joint)] for joint in joints)
        hierarchy = list(joints)
        if chain != ChainType.THUMB and (chain, FingerJointType.METACARPAL) in by_semantic:
            hierarchy.insert(0, FingerJointType.METACARPAL)
        parent_id = wrist.node_id
        for joint in hierarchy:
            allowed.add((chain, joint))
            node = hand.nodes[by_semantic[(chain, joint)]]
            if node.parent_id != parent_id:
                raise ValueError("finger hierarchy disagrees with semantic joints")
            parent_id = node.node_id
    if by_semantic.keys() != allowed:
        raise ValueError("unexpected raw hand semantic nodes")
    return tuple(wanted)


class ManusParser:
    """feed_line(line, now_ns=None) returns one HandFrame or None, without ROS.

    A HAND begins a fresh metadata transaction; malformed/duplicate NODE records
    invalidate it until the next HAND. Sequence/time high-water marks survive
    metadata replacement. Healthy hands remain independent of malformed peers.
    """

    def __init__(self, stale_timeout: float = 0.25):
        if not math.isfinite(stale_timeout) or stale_timeout <= 0:
            raise ValueError("stale timeout must be positive and finite")
        self.stale_ns = int(stale_timeout * 1_000_000_000)
        self.hands: dict[int, _Hand] = {}
        self._last: dict[int, tuple[int, int]] = {}

    def feed_line(self, line: str, now_ns: int | None = None) -> HandFrame | None:
        parts = line.split()
        if not parts or parts[0] not in ("HAND", "NODE", "POSE") or len(parts) < 2:
            return None
        tag = parts[0]
        try:
            glove = int(parts[1], 16)
            if not 0 <= glove <= 0xFFFFFFFF:
                return None
        except ValueError:
            return None
        try:
            if tag == "HAND":
                # Do not leave an old valid mapping alive after a bad replacement.
                self.hands.pop(glove, None)
                if len(parts) != 4:
                    return None
                side, sdk_side = SIDES[parts[2]]
                count = int(parts[3])
                if not 21 <= count <= MAX_NODES:
                    return None
                self.hands[glove] = _Hand(side, sdk_side, count)
                return None
            hand = self.hands.get(glove)
            if hand is None or hand.invalid:
                return None
            if tag == "NODE":
                if len(parts) != 8:
                    raise ValueError("invalid NODE record")
                index, node_id, parent_id, chain, side, joint = map(int, parts[2:])
                if (not 0 <= index < hand.count or index in hand.nodes
                        or not 0 <= node_id <= 0xFFFFFFFF or not 0 <= parent_id <= 0xFFFFFFFF):
                    raise ValueError("invalid or duplicate node index/ID")
                hand.nodes[index] = _Node(node_id, parent_id, ChainType(chain), side, FingerJointType(joint))
                if len(hand.nodes) == hand.count:
                    hand.order = _keypoint_order(hand)
                return None
            if hand.order is None or len(parts) != 5 + hand.count * 7:
                return None
            sequence, source_ns, sdk_time = map(int, parts[2:5])
            if not (0 < sequence <= 0xFFFFFFFFFFFFFFFF and 0 < source_ns <= 0xFFFFFFFFFFFFFFFF
                    and 0 <= sdk_time <= 0xFFFFFFFFFFFFFFFF):
                return None
            now_ns = time.monotonic_ns() if now_ns is None else now_ns
            previous_sequence, previous_ns = self._last.get(glove, (0, 0))
            if sequence <= previous_sequence or source_ns <= previous_ns or source_ns > now_ns:
                return None
            self._last[glove] = (sequence, source_ns)
            if now_ns - source_ns > self.stale_ns:
                return None
            values = tuple(map(float, parts[5:]))
            if any(not math.isfinite(v) or abs(v) > FLOAT32_MAX for v in values):
                return None
            hand.latest_valid_ns = source_ns
            if any(
                other_id != glove and other.side == hand.side
                and other.order is not None and not other.invalid
                and other.latest_valid_ns > 0
                and 0 <= now_ns - other.latest_valid_ns <= self.stale_ns
                for other_id, other in self.hands.items()
            ):
                return None
            data = tuple(value for index in hand.order for value in (
                values[index * 7], values[index * 7 + 1], values[index * 7 + 2]))
            return HandFrame(hand.side, data, sequence, source_ns)
        except (ValueError, KeyError, OverflowError):
            if tag == "NODE" and glove in self.hands:
                self.hands[glove].invalid = True
                self.hands[glove].order = None
            return None


def main(argv: list[str] | None = None) -> int:
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--topic", default="/hand_input", help="ROS2 Float32MultiArray topic (default: /hand_input)")
    cli.add_argument("--stale-timeout", type=float, default=0.25, metavar="SECONDS",
                     help="maximum source monotonic age (default: 0.25 seconds)")
    args = cli.parse_args(argv)
    try:
        parser = ManusParser(stale_timeout=args.stale_timeout)
    except ValueError as exc:
        cli.error(str(exc))

    # Keep the pure parser and --help usable without ROS imports or SDK devices.
    import rclpy
    from std_msgs.msg import Float32MultiArray, MultiArrayDimension

    rclpy.init(args=[])
    node = None
    try:
        node = rclpy.create_node("manus_hand_input")
        publisher = node.create_publisher(Float32MultiArray, args.topic, 10)
        # Read bytes directly: select on TextIOWrapper can strand prefetched lines.
        # Polling lets ROS shutdown/signals terminate even with an idle open pipe.
        import os

        pending = b""
        while rclpy.ok():
            ready, _, _ = select.select([sys.stdin.fileno()], [], [], 0.1)
            if not ready:
                continue
            chunk = os.read(sys.stdin.fileno(), 65536)
            if not chunk:
                if pending:
                    frame = parser.feed_line(pending.decode("utf-8", errors="replace"))
                    if frame is not None:
                        publish_frame(publisher, frame, Float32MultiArray, MultiArrayDimension)
                break
            pending += chunk
            while b"\n" in pending:
                raw_line, pending = pending.split(b"\n", 1)
                frame = parser.feed_line(raw_line.decode("utf-8", errors="replace"))
                if frame is not None:
                    publish_frame(publisher, frame, Float32MultiArray, MultiArrayDimension)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def publish_frame(publisher, frame: HandFrame, message_type, dimension_type) -> None:
    """Publish only this frame's side; ROS message has no source timestamp field."""
    message = message_type()
    message.layout.dim = [dimension_type(label=frame.side, size=63, stride=63)]
    message.layout.data_offset = 0
    message.data = list(frame.data)
    publisher.publish(message)


if __name__ == "__main__":
    raise SystemExit(main())
