"""Semantic mapping must preserve anatomy and actual SDK flexion direction."""

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


@pytest.mark.parametrize("side", ["left", "right"])
def test_shuffled_right_handed_fist_flexes_and_reopens_the_same_hand(side):
    import xml.etree.ElementTree as ET
    import mujoco
    from tianji_runtime import workspace
    from manus_bridge.hand2_sdk import WujiHand2Retargeter

    # Official Hand2 kinematics supply a physically known fist, independent of
    # the adapter and solver. Visual meshes are irrelevant to forward kinematics.
    assets = (workspace() / "src/teleop_outputs/wuji/wuji_retargeting"
              / "wuji_retargeting/wuji-description/hand2/hand2_beta1/body/urdf")
    urdf = ET.parse(assets / f"{side}.urdf").getroot()
    for link in urdf.findall("link"):
        for tag in ("visual", "collision"):
            for element in link.findall(tag):
                link.remove(element)
    ET.SubElement(ET.SubElement(urdf, "mujoco"), "compiler", fusestatic="false")
    model = mujoco.MjModel.from_xml_string(ET.tostring(urdf, encoding="unicode"))
    data = mujoco.MjData(model)
    prefix = side[0] + "_"
    fingers = ("thumb", "index_finger", "middle_finger", "ring_finger", "pinky")
    links = [prefix + "wrist"] + [
        prefix + finger + "_" + suffix for finger in fingers
        for suffix in ("proximal_abd", "middle", "distal", "tip")
    ]
    body_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
                for name in links]
    assert min(body_ids) >= 0
    solver = WujiHand2Retargeter(side)

    def solve(folded):
        data.qpos[:] = 0
        if folded:
            for finger in fingers[1:]:
                for suffix, angle in (("mcp_flex", 0.7), ("pip", 0.9), ("dip", 0.6)):
                    joint = mujoco.mj_name2id(
                        model, mujoco.mjtObj.mjOBJ_JOINT, prefix + finger + "_" + suffix)
                    data.qpos[model.jnt_qposadr[joint]] = angle
        mujoco.mj_forward(model, data)
        # A proper world rotation/translation changes orientation, not handedness.
        rotation = np.array([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])
        positions = data.xpos[body_ids] @ rotation.T + [0.3, -0.2, 0.5]
        nodes = raw_hand()
        for node, position in zip(nodes, positions, strict=True):
            node.pose.position = Obj(x=position[0], y=position[1], z=position[2])
        points, _ = convert_manus_to_mediapipe([
            *reversed(nodes), raw_node("Index", "MCP", 88), raw_node("Ring", "MCP", 89),
        ])
        for _ in range(40):
            joints = solver.retarget(points)
        return joints.reshape(5, 4)[1:, [0, 2, 3]]

    opened = solve(False)
    folded = solve(True)
    reopened = solve(False)
    assert np.all(folded > opened + 0.3), (opened, folded)
    assert np.all(folded > reopened + 0.3), (folded, reopened)


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
