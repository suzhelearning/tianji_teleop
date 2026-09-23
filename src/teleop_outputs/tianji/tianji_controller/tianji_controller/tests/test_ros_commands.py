"""ROS command handoff safety; DDS checks use an isolated domain, never hardware."""
from types import SimpleNamespace
import unittest

from tianji_controller.ros_commands import CommandInbox
from tianji_controller.safety import MotionGate, SafetyFault
from tianji_controller.tests.test_safety import configuration, feedback, NOW

BOOT = '5705f150-b00f-4d45-8431-65ff5dd19bd8'
SESSION = '1e388afd-a697-4240-bd52-0462e34b92b2'


def message(sequence=1, **changes):
    values = dict(boot_id=BOOT, session_id=SESSION, sequence=sequence,
                  produced_monotonic_ns=NOW + sequence, input_monotonic_ns=NOW,
                  tracking_epoch=7, flags=7, left_arm=[0.] * 7, right_arm=[0.] * 7,
                  left_hand=[0.] * 20, right_hand=[0.] * 20)
    values.update(changes)
    return SimpleNamespace(**values)


class CommandInboxTests(unittest.TestCase):
    def setUp(self):
        self.now = NOW + 100
        self.inbox = CommandInbox(configuration(), ('arms', 'left_hand', 'right_hand'),
                                  boot_id=BOOT, clock=lambda: self.now)

    def receive(self, sequence=1, **changes):
        self.inbox.receive(message(sequence, **changes), b'controller')

    def arm(self):
        self.receive()
        gate = MotionGate(configuration(), ('arms', 'left_hand', 'right_hand'))
        gate.arm(self.inbox.drain(), feedback(), self.now)
        return gate

    def test_joint_slots_reach_the_existing_gate_without_wire_serialization(self):
        self.receive(left_arm=[.1] * 7, right_arm=[.2] * 7,
                     left_hand=[.3] * 20, right_hand=[.4] * 20)
        received = self.inbox.drain()
        self.assertEqual(received.positions('arms'), (.1,) * 7 + (.2,) * 7)
        self.assertEqual(received.positions('left_hand'), (.3,) * 20)
        self.assertEqual(received.positions('right_hand'), (.4,) * 20)
        MotionGate(configuration(), ('arms', 'left_hand', 'right_hand')).validate_targets(received, self.now)

    def test_readiness_revocation_survives_a_newer_ready_frame(self):
        gate = self.arm()
        self.receive(2, flags=1)
        self.receive(3)
        with self.assertRaises(SafetyFault):
            self.inbox.drain(lambda value: gate.observe_source(value, self.now))
        self.assertIsNotNone(gate.fault)

    def test_epoch_change_and_return_cannot_hide_between_drains(self):
        gate = self.arm()
        self.receive(2, tracking_epoch=8)
        self.receive(3, tracking_epoch=7)
        with self.assertRaises(SafetyFault):
            self.inbox.drain(lambda value: gate.observe_source(value, self.now))
        self.assertIsNotNone(gate.fault)

    def test_preflight_can_wait_for_initial_source_readiness(self):
        self.receive(flags=0, tracking_epoch=0, input_monotonic_ns=0)
        self.receive(2)
        gate = MotionGate(configuration(), ('arms',))
        gate.check_enable_ready(self.inbox.drain(), feedback(), self.now)
        self.assertFalse(gate.armed)

    def test_foreign_boot_session_and_endpoint_faults_cannot_be_overwritten(self):
        for changes, gid in (({'boot_id': 'foreign'}, b'controller'),
                             ({'session_id': '9e388afd-a697-4240-bd52-0462e34b92b2'}, b'controller'),
                             ({}, b'other-controller')):
            with self.subTest(changes=changes, gid=gid):
                inbox = CommandInbox(configuration(), ('arms',), boot_id=BOOT, clock=lambda: self.now)
                inbox.receive(message(), b'controller')
                inbox.receive(message(2, **changes), gid)
                inbox.receive(message(3), b'controller')
                with self.assertRaises(SafetyFault):
                    inbox.drain()

    def test_reordered_sequence_or_time_rejected_even_if_followed_by_valid_frame(self):
        for invalid in (message(1), message(3, produced_monotonic_ns=NOW + 1),
                        message(3, input_monotonic_ns=NOW - 1)):
            with self.subTest(invalid=invalid):
                inbox = CommandInbox(configuration(), ('arms',), boot_id=BOOT, clock=lambda: self.now)
                inbox.receive(message(2), b'controller')
                inbox.receive(invalid, b'controller')
                inbox.receive(message(4), b'controller')
                with self.assertRaises(SafetyFault):
                    inbox.drain()
                self.assertEqual(inbox.latest.sequence, 2)

    def test_malformed_and_expired_applied_input_are_not_hidden_by_latest(self):
        for changes in ({'flags': 8}, {'left_arm': [float('nan')] * 7},
                        {'right_arm': [3.] * 7}, {'left_hand': [0.] * 19},
                        {'produced_monotonic_ns': NOW - 200_000_000},
                        {'input_monotonic_ns': NOW - 200_000_000},
                        {'input_monotonic_ns': NOW + 10_000_000},
                        {'tracking_epoch': 0}, {'session_id': ''}):
            with self.subTest(changes=changes):
                inbox = CommandInbox(configuration(), ('arms',), boot_id=BOOT, clock=lambda: self.now)
                inbox.receive(message(**changes), b'controller')
                inbox.receive(message(2), b'controller')
                with self.assertRaises(SafetyFault):
                    inbox.drain()

    def test_fresh_production_time_does_not_refresh_old_applied_input(self):
        self.now = NOW + 100_000_000
        self.receive(produced_monotonic_ns=self.now)
        gate = MotionGate(configuration(), ('arms',))
        gate.arm(self.inbox.drain(), feedback(stamp=self.now), self.now)
        self.now += 60_000_000
        with self.assertRaises(SafetyFault):
            self.inbox.drain(lambda value: gate.observe_source(value, self.now))
        self.assertIsNotNone(gate.fault)

    def test_unselected_unready_arms_do_not_block_a_fresh_hand(self):
        inbox = CommandInbox(configuration(), ('right_hand',), boot_id=BOOT, clock=lambda: self.now)
        inbox.receive(message(flags=2, tracking_epoch=0, input_monotonic_ns=0), b'controller')
        MotionGate(configuration(), ('right_hand',)).check_enable_ready(inbox.drain(), feedback(), self.now)


def test_unresolved_discovery_is_not_authority_or_a_permanent_fault():
    from tianji_controller.ros_commands import _controller_publisher_gid
    endpoint = SimpleNamespace(node_name='_NODE_NAME_UNKNOWN_',
                               node_namespace='_NODE_NAMESPACE_UNKNOWN_',
                               topic_type='tianji_interfaces/msg/ControllerJointTargets',
                               endpoint_gid=b'writer')
    assert _controller_publisher_gid([endpoint]) is None
    endpoint.node_name = 'tianji_arm_core'
    endpoint.node_namespace = '/'
    assert _controller_publisher_gid([endpoint]) == b'writer'


def test_actual_dds_receiver_accepts_a_late_discovered_controller(monkeypatch):
    import time
    import uuid
    from pathlib import Path
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from tianji_interfaces.msg import ControllerJointTargets
    from tianji_controller.ros_commands import CommandReceiver
    monkeypatch.setenv('ROS_DOMAIN_ID', '121')
    topic = '/test/arm_commands/s' + uuid.uuid4().hex
    context = Context()
    context.init(args=[])
    node = receiver = None
    try:
        receiver = CommandReceiver(topic, configuration(), ('arms',))
        assert receiver.drain() is None
        node = Node('tianji_arm_core', context=context)
        publisher = node.create_publisher(
            ControllerJointTargets, topic,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        packet = ControllerJointTargets(
            boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            session_id=str(uuid.uuid4()), tracking_epoch=7, flags=1,
            left_arm=[.1] * 7, right_arm=[.2] * 7,
            left_hand=[0.] * 20, right_hand=[0.] * 20)
        deadline = time.monotonic() + 5
        result = None
        while time.monotonic() < deadline:
            packet.sequence += 1
            packet.produced_monotonic_ns = time.monotonic_ns()
            packet.input_monotonic_ns = packet.produced_monotonic_ns
            publisher.publish(packet)
            time.sleep(.01)
            result = receiver.drain()
            if result is not None:
                break
        assert result is not None
        assert result.positions('arms') == (.1,) * 7 + (.2,) * 7
        MotionGate(configuration(), ('arms',)).validate_targets(result, time.monotonic_ns())
    finally:
        if receiver is not None:
            receiver.close()
        if node is not None:
            node.destroy_node()
        context.try_shutdown()


if __name__ == '__main__':
    unittest.main()
