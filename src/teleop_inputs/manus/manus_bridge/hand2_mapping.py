"""Semantic MANUS VUH skeleton -> MediaPipe-order SDK input, in metres.

Adapted from wuji-hand-teleop's manus_input_py/manus_input_node.py.
Copyright (c) 2025 Wuji Technology Co., Ltd.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import numpy as np


MEDIAPIPE_SEMANTIC_ORDER = (
    ("hand", "wrist"),
    *(("thumb", joint) for joint in ("mcp", "pip", "dip", "tip")),
    *((finger, joint) for finger in ("index", "middle", "ring", "pinky")
      for joint in ("pip", "ip", "dip", "tip")),
)
_REQUIRED_SEMANTICS = frozenset(MEDIAPIPE_SEMANTIC_ORDER)
_CHAIN_ALIASES = {
    "hand": "hand", "thumb": "thumb", "fingerthumb": "thumb",
    "index": "index", "fingerindex": "index", "middle": "middle",
    "fingermiddle": "middle", "ring": "ring", "fingerring": "ring",
    "pinky": "pinky", "fingerpinky": "pinky",
}
_JOINT_ALIASES = {
    "mcp": "mcp", "metacarpal": "mcp", "pip": "pip", "proximal": "pip",
    "ip": "ip", "intermediate": "ip", "dip": "dip", "distal": "dip",
    "tip": "tip",
}


class ManusMappingError(ValueError):
    """A raw skeleton is incomplete, ambiguous, or nonfinite."""


def _normalise_token(value):
    return str(value).strip().lower().replace("_", "").replace("-", "").replace(" ", "")


def resolve_mediapipe_nodes(raw_nodes):
    """Resolve by anatomy, never by SDK-version-dependent node IDs/order."""
    resolved = {}
    for node in raw_nodes:
        chain = _CHAIN_ALIASES.get(_normalise_token(node.chain_type))
        joint = "wrist" if chain == "hand" else _JOINT_ALIASES.get(
            _normalise_token(node.joint_type)
        )
        key = (chain, joint)
        if key not in _REQUIRED_SEMANTICS:
            continue
        if key in resolved:
            raise ManusMappingError(f"duplicate MANUS semantic node: {chain}/{joint}")
        resolved[key] = node
    missing = _REQUIRED_SEMANTICS.difference(resolved)
    if missing:
        raise ManusMappingError(f"missing MANUS semantic nodes: {sorted(missing)}")
    return tuple(resolved[key] for key in MEDIAPIPE_SEMANTIC_ORDER)


def convert_manus_to_mediapipe(raw_nodes):
    """Return (21,3) points and resolved IDs, applying (x,-y,z) exactly once.

    Acquisition supplies unmodified world-space right-handed VUH XFromViewer,
    Z-up positions in metres. This is the reference retarget-input reflection,
    not a ROS REP-103 frame or a wrist-local pose. The SDK consumes these points
    directly. The extra non-thumb MANUS metacarpals are deliberately omitted.
    """
    nodes = resolve_mediapipe_nodes(raw_nodes)
    points = np.empty((21, 3), dtype=np.float32)
    for index, node in enumerate(nodes):
        position = node.pose.position
        points[index] = (position.x, -position.y, position.z)
    if not np.isfinite(points).all():
        raise ManusMappingError("MANUS positions contain NaN or infinity")
    return points, tuple(int(node.node_id) for node in nodes)
