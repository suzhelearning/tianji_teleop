"""Raw node IDs/order cannot permute anatomy; reflect VUH Y exactly once."""

from types import SimpleNamespace as Obj

import numpy as np
import pytest

from manus_bridge.hand2_mapping import (
    ManusMappingError, convert_manus_to_mediapipe,
)


def raw_node(chain, joint, index):
    return Obj(
        chain_type=chain, joint_type=joint, node_id=900 - index * 7,
        pose=Obj(position=Obj(x=index * 0.001, y=index * 0.002, z=index * 0.003)),
    )


def raw_hand():
    semantics = [("Hand", "Invalid")]
    semantics.extend(("Thumb", joint) for joint in ("MCP", "PIP", "DIP", "TIP"))
    for finger in ("Index", "Middle", "Ring", "Pinky"):
        semantics.extend((finger, joint) for joint in ("PIP", "IP", "DIP", "TIP"))
    return [raw_node(chain, joint, index) for index, (chain, joint) in enumerate(semantics)]


def test_shuffled_ids_and_extra_metacarpals_preserve_anatomy_and_units():
    nodes = raw_hand()
    points, ids = convert_manus_to_mediapipe([
        *reversed(nodes), raw_node("Index", "MCP", 88), raw_node("Ring", "MCP", 89),
    ])
    np.testing.assert_allclose(points, [
        (index * 0.001, -index * 0.002, index * 0.003) for index in range(21)
    ], atol=1e-8)
    assert ids == tuple(node.node_id for node in nodes)


def test_ambiguous_anatomy_is_rejected_instead_of_selecting_first_node():
    nodes = raw_hand()
    with pytest.raises(ManusMappingError):
        convert_manus_to_mediapipe([*nodes, raw_node("finger_index", "proximal", 42)])


def test_incomplete_or_nonfinite_skeleton_cannot_become_target_points():
    nodes = raw_hand()
    with pytest.raises(ManusMappingError):
        convert_manus_to_mediapipe(nodes[:-1])
    nodes[-1].pose.position.z = float("nan")
    with pytest.raises(ManusMappingError):
        convert_manus_to_mediapipe(nodes)
