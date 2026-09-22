"""Standalone ROS Manus raw-skeleton to MediaPipe landmark adapter."""

from tianji_interfaces.msg import HandLandmarks, ManusGlove

from manus_bridge.hand2_mapping import convert_manus_to_mediapipe
from manus_bridge.hand2_ros import HandPipelineNode, SIDES, run_node


def _convert(nodes):
    return convert_manus_to_mediapipe(nodes)[0]


class ManusAdapterNode(HandPipelineNode):
    def __init__(self):
        super().__init__(
            "manus_adapter", ManusGlove, HandLandmarks,
            "/manus/raw/{side}", "/manus/landmarks/{side}",
            {side: _convert for side in SIDES},
        )


def main(args=None):
    run_node(ManusAdapterNode, args)
