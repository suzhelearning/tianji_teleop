"""ROS boundary for the two command-only Manus/Hand2 stages."""

from pathlib import Path

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from manus_bridge.hand2_core import HandStage, IDENTITY_FIELDS
from manus_bridge.hand2_sdk import HAND2_JOINT_LABELS


SIDES = ("left", "right")


def stream_qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class HandPipelineNode(Node):
    """One subscription/state/publisher per side, with no bilateral cache."""

    def __init__(self, name, input_type, output_type, input_topic, output_topic,
                 transforms, resets=None, *, commands=False):
        super().__init__(name)
        age = float(self.declare_parameter("max_source_age_s", 0.25).value)
        boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        self._commands = commands
        self._output_type = output_type
        self._topics = {side: input_topic.format(side=side) for side in SIDES}
        self._outputs = {
            side: self.create_publisher(output_type, output_topic.format(side=side), stream_qos())
            for side in SIDES
        }
        self._stages = {
            side: HandStage(
                side, boot_id, transforms[side],
                reset=resets[side] if resets else None, max_source_age_s=age,
            ) for side in SIDES
        }
        self._ready = {side: False for side in SIDES}
        self._inputs = [
            self.create_subscription(
                input_type, self._topics[side], self._callback_for(side), stream_qos()
            ) for side in SIDES
        ]
        self._watchdog = self.create_timer(
            min(0.05, max(0.001, age / 2)), self._poll,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self.get_logger().info("Waiting for fresh, unambiguous per-hand input; no hardware output")

    def _source_endpoint(self, side):
        # Jazzy MessageInfo has no publisher_gid. Bind only a uniquely
        # discovered graph endpoint; discovery races must fail closed.
        endpoints = self.get_publishers_info_by_topic(self._topics[side])
        if len(endpoints) != 1:
            return len(endpoints), None
        gid = bytes(endpoints[0].endpoint_gid)
        if not gid or not any(gid):
            return 0, None
        return 1, gid

    def _callback_for(self, side):
        def callback(message):
            payload = message.points if self._commands else message.raw_nodes
            publisher_count, writer = self._source_endpoint(side)
            result = self._stages[side].process(
                message, payload,
                publisher_count=publisher_count,
                writer=writer,
            )
            self._publish(side, result)
        return callback

    def _poll(self):
        for side in SIDES:
            self._publish(side, self._stages[side].poll(
                publisher_count=self._source_endpoint(side)[0]
            ))

    def _publish(self, side, result):
        if result is None:
            return
        message = self._output_type()
        for name in IDENTITY_FIELDS:
            setattr(message, name, getattr(result.source, name))
        message.valid = result.valid
        if self._commands:
            message.joint_names = list(HAND2_JOINT_LABELS)
            message.position_rad = (
                result.values.tolist() if result.valid else [float("nan")] * 20
            )
        else:
            message.points = (
                result.values.reshape(63).tolist() if result.valid else [float("nan")] * 63
            )
        self._outputs[side].publish(message)
        if result.valid and not self._ready[side]:
            label = "20-joint commands" if self._commands else "21-point landmarks"
            self.get_logger().info(f"{side}: published first fresh valid {label}")
            self._ready[side] = True
        elif not result.valid:
            self._ready[side] = False
            self.get_logger().warning(f"{side}: output invalid: {result.reason}")


def run_node(factory, args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = factory()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
