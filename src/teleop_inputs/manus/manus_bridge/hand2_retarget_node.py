"""Standalone ROS Wuji Hand2 IK -> 20-joint command publisher, not an actuator."""

from tianji_interfaces.msg import HandJointCommand, HandLandmarks

from manus_bridge.hand2_ros import HandPipelineNode, SIDES, run_node
from manus_bridge.hand2_sdk import WujiHand2Retargeter


class ManusHand2RetargetNode(HandPipelineNode):
    def __init__(self):
        # Exactly one SDK session per side, in this process; no worker or device.
        retargeters = {side: WujiHand2Retargeter(side) for side in SIDES}
        super().__init__(
            "manus_hand2_retarget", HandLandmarks, HandJointCommand,
            "/manus/landmarks/{side}", "/wuji/{side}_hand/joint_commands",
            {side: solver.retarget for side, solver in retargeters.items()},
            {side: solver.reset for side, solver in retargeters.items()},
            commands=True,
        )


def main(args=None):
    run_node(ManusHand2RetargetNode, args)
