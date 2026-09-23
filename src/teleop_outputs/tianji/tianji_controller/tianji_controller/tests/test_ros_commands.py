"""ROS command handoff safety; DDS checks use an isolated domain, never hardware."""
from types import SimpleNamespace
import unittest

from tianji_controller.ros_commands import CommandInbox, HAND_JOINT_NAMES
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
        for invalid in (message(1), message(3, produced_monotonic_ns=NOW + 1)):
            with self.subTest(invalid=invalid):
                inbox = CommandInbox(configuration(), ('arms',), boot_id=BOOT, clock=lambda: self.now)
                inbox.receive(message(2), b'controller')
                inbox.receive(invalid, b'controller')
                inbox.receive(message(4), b'controller')
                with self.assertRaises(SafetyFault):
                    inbox.drain()
                self.assertEqual(inbox.latest.sequence, 2)

    def test_malformed_commands_are_not_hidden_by_latest(self):
        for changes in ({'flags': 8}, {'left_arm': [float('nan')] * 7},
                        {'right_arm': [3.] * 7}, {'left_hand': [0.] * 19},
                        {'produced_monotonic_ns': 0},
                        {'tracking_epoch': 0}, {'session_id': ''}):
            with self.subTest(changes=changes):
                inbox = CommandInbox(configuration(), ('arms',), boot_id=BOOT, clock=lambda: self.now)
                inbox.receive(message(**changes), b'controller')
                inbox.receive(message(2), b'controller')
                with self.assertRaises(SafetyFault):
                    inbox.drain()

    def test_continuous_commands_ignore_input_age_but_command_dropout_faults(self):
        gate = self.arm()
        for sequence in range(2, 62):
            self.now += 5_000_000
            self.receive(sequence, input_monotonic_ns=NOW - 1_000_000_000)
            self.inbox.drain(lambda value: gate.observe_source(value, self.now))
        self.now += 150_000_000
        self.inbox.drain(lambda value: gate.observe_source(value, self.now))
        self.now += 1
        with self.assertRaises(SafetyFault):
            self.inbox.drain(lambda value: gate.observe_source(value, self.now))
        self.assertIsNotNone(gate.fault)

    def test_command_dropout_cannot_be_hidden_by_recovery_before_drain(self):
        gate = self.arm()
        self.now += 150_000_001
        self.receive(2)
        with self.assertRaises(SafetyFault):
            self.inbox.drain(lambda value: gate.observe_source(value, self.now))
        self.assertIsNotNone(gate.fault)

    def test_unselected_unready_arms_do_not_block_a_fresh_hand(self):
        inbox = CommandInbox(configuration(), ('right_hand',), boot_id=BOOT, clock=lambda: self.now)
        inbox.receive(message(flags=2, tracking_epoch=0, input_monotonic_ns=0), b'controller')
        MotionGate(configuration(), ('right_hand',)).check_enable_ready(inbox.drain(), feedback(), self.now)


class RevocationGenerationTests(unittest.TestCase):
    def test_lost_invalid_sample_is_visible_in_recovered_valid_sample(self):
        now = NOW + 100
        inbox = CommandInbox(configuration(), ('left_hand',),
                             hand_topics={'left_hand': '/test/left'},
                             boot_id=BOOT, clock=lambda: now)
        inbox.receive_hand('left_hand', hand_message(revocation_generation=4), b'manus')
        gate = MotionGate(configuration(), ('left_hand',))
        gate.arm(inbox.drain(), feedback(), now)
        # KEEP_LAST may discard valid=false; the increment survives in every
        # subsequent valid target and cannot be overwritten by another sample.
        inbox.receive_hand('left_hand', hand_message(sequence=2, revocation_generation=5), b'manus')
        inbox.receive_hand('left_hand', hand_message(sequence=3, revocation_generation=5), b'manus')
        with self.assertRaises(SafetyFault):
            inbox.drain(lambda frame: gate.observe_source(frame, now))
        self.assertIsNotNone(gate.fault)

    def test_generation_regression_cannot_be_hidden_by_newer_target(self):
        inbox = CommandInbox(configuration(), ('left_hand',),
                             hand_topics={'left_hand': '/test/left'},
                             boot_id=BOOT, clock=lambda: NOW + 100)
        inbox.receive_hand('left_hand', hand_message(revocation_generation=4), b'manus')
        inbox.receive_hand('left_hand', hand_message(sequence=2, revocation_generation=3), b'manus')
        inbox.receive_hand('left_hand', hand_message(sequence=3, revocation_generation=4), b'manus')
        with self.assertRaises(SafetyFault):
            inbox.drain()


def hand_message(side='left', sequence=1, **changes):
    values = dict(boot_id=BOOT, session_id=SESSION, glove_id=11 if side == 'left' else 22,
                  side=side, sequence=sequence, source_monotonic_ns=NOW + sequence,
                  valid=True, revocation_generation=0,
                  joint_names=list(HAND_JOINT_NAMES), position_rad=[0.] * 20)
    values.update(changes)
    return SimpleNamespace(**values)


class IndependentHandInboxTests(unittest.TestCase):
    def setUp(self):
        self.now = NOW + 100
        self.devices = ('arms', 'left_hand', 'right_hand')
        self.inbox = self.make_inbox(self.devices)

    def make_inbox(self, devices):
        return CommandInbox(
            configuration(), devices, boot_id=BOOT, clock=lambda: self.now,
            hand_topics={device: f'/test/{device}' for device in devices if device != 'arms'})

    def receive_hand(self, side='left', sequence=1, **changes):
        self.inbox.receive_hand(side + '_hand', hand_message(side, sequence, **changes), b'manus')

    def receive_all(self, sequence=1, **hand_changes):
        self.inbox.receive(message(sequence), b'controller')
        for side in ('left', 'right'):
            self.receive_hand(side, sequence, **hand_changes)

    def arm(self):
        self.receive_all()
        gate = MotionGate(configuration(), self.devices)
        gate.arm(self.inbox.drain(), feedback(), self.now)
        return gate

    def observe(self, gate):
        return self.inbox.drain(lambda frame: gate.observe_source(frame, self.now))

    def test_combined_targets_ignore_native_hand_payload_and_readiness(self):
        self.inbox.receive(message(flags=1, left_arm=[.1] * 7, right_arm=[.2] * 7,
                                   left_hand=[float('nan')] * 20, right_hand=[]), b'controller')
        self.receive_hand('left', position_rad=[.3] * 20)
        self.receive_hand('right', position_rad=[.4] * 20)
        frame = self.inbox.drain()
        self.assertEqual(frame.positions('arms'), (.1,) * 7 + (.2,) * 7)
        self.assertEqual(frame.left_hand, (.3,) * 20)
        self.assertEqual(frame.right_hand, (.4,) * 20)
        MotionGate(configuration(), self.devices).validate_targets(frame, self.now)

    def test_hand_only_needs_neither_arm_stream_nor_unselected_hand(self):
        self.inbox = self.make_inbox(('right_hand',))
        self.receive_hand('left', boot_id='foreign', position_rad=[float('nan')] * 20)
        self.inbox.receive(message(boot_id='foreign'), b'bad-controller')
        self.receive_hand('right', position_rad=[.2] * 20)
        frame = self.inbox.drain()
        MotionGate(configuration(), ('right_hand',)).validate_targets(frame, self.now)
        self.assertEqual(frame.right_hand, (.2,) * 20)
        self.assertEqual(frame.tracking_epoch, 0)

    def test_startup_waits_for_each_selected_source_without_latching_gate(self):
        gate = MotionGate(configuration(), self.devices)
        self.assertIsNone(self.inbox.drain())
        self.receive_hand('left')
        with self.assertRaises(SafetyFault):
            self.observe(gate)
        self.assertIsNone(gate.fault)
        self.inbox.receive(message(), b'controller')
        with self.assertRaises(SafetyFault):
            gate.check_enable_ready(self.inbox.drain(), feedback(), self.now)
        self.receive_hand('right')
        gate.check_enable_ready(self.inbox.drain(), feedback(), self.now)

    def test_revocation_with_repeated_metadata_and_nans_survives_new_targets(self):
        gate = self.arm()
        self.receive_hand('left', valid=False, position_rad=[float('nan')] * 20)
        self.receive_hand('left', valid=False, position_rad=[float('nan')] * 20)
        self.receive_hand('left', 2)
        self.receive_hand('right', 2)
        self.inbox.receive(message(2), b'controller')
        with self.assertRaises(SafetyFault):
            self.observe(gate)
        self.assertIsNotNone(gate.fault)

    def test_invalid_message_never_installs_payload_or_refreshes_age(self):
        self.inbox = self.make_inbox(('left_hand',))
        self.receive_hand(position_rad=[.3] * 20)
        stamp = self.inbox.drain().timestamp_ns
        self.now += 200_000_000
        self.receive_hand(valid=False, position_rad=[float('nan')] * 20)
        frame = self.inbox.drain()
        self.assertEqual(frame.timestamp_ns, stamp)
        self.assertEqual(frame.left_hand, (.3,) * 20)
        self.assertFalse(frame.flags & 4)

    def test_initial_revocation_can_recover_during_preflight(self):
        self.inbox = self.make_inbox(('left_hand',))
        gate = MotionGate(configuration(), ('left_hand',))
        self.receive_hand(valid=False, position_rad=[float('nan')] * 20)
        with self.assertRaises(SafetyFault):
            self.observe(gate)
        self.assertIsNone(gate.fault)
        self.receive_hand(sequence=2)
        gate.arm(self.inbox.drain(), feedback(), self.now)
        self.assertTrue(gate.armed)

    def test_continuous_commands_do_not_recheck_operator_sample_ages(self):
        gate = self.arm()
        for sequence in range(2, 62):
            self.now += 5_000_000
            # Producer timestamps advance slowly; local cmd arrivals stay at
            # 200 Hz. Operator-input validity belongs to those producers.
            self.inbox.receive(message(sequence, input_monotonic_ns=NOW - 1_000_000_000),
                               b'controller')
            self.receive_hand('left', sequence)
            self.receive_hand('right', sequence)
            self.observe(gate)
        self.now += 150_000_000
        self.observe(gate)
        self.now += 1
        with self.assertRaises(SafetyFault):
            self.observe(gate)
        self.assertIsNotNone(gate.fault)

    def test_fresh_arm_and_right_hand_cannot_refresh_stale_left_hand(self):
        gate = self.arm()
        self.now += 200_000_000
        self.inbox.receive(message(2, produced_monotonic_ns=self.now,
                                   input_monotonic_ns=self.now), b'controller')
        self.receive_hand('right', 2, source_monotonic_ns=self.now)
        frame = self.inbox.drain()
        self.assertFalse(frame.flags & 4)
        with self.assertRaises(SafetyFault):
            self.observe(gate)
        self.assertIsNotNone(gate.fault)

    def test_source_dropout_recovered_between_drains_still_faults_when_armed(self):
        for source in self.devices:
            with self.subTest(source=source):
                self.setUp()
                gate = self.arm()
                # Keep all other sources fresh while this source misses a full
                # watchdog interval, then recover before the next drain.
                for offset in (100_000_000, 200_000_000):
                    self.now = NOW + offset
                    if source != 'arms':
                        self.inbox.receive(message(offset, produced_monotonic_ns=self.now,
                                                   input_monotonic_ns=self.now), b'controller')
                    for side in ('left', 'right'):
                        if source != side + '_hand':
                            self.receive_hand(side, offset, source_monotonic_ns=self.now)
                if source == 'arms':
                    self.inbox.receive(message(2, produced_monotonic_ns=self.now,
                                               input_monotonic_ns=self.now), b'controller')
                else:
                    self.receive_hand(source.removesuffix('_hand'), 2,
                                      source_monotonic_ns=self.now)
                with self.assertRaises(SafetyFault):
                    self.observe(gate)
                self.assertIsNotNone(gate.fault)

    def test_graph_disappearance_and_recovery_is_not_coalesced_away(self):
        gate = self.arm()
        self.inbox.revoke('left_hand')
        self.receive_hand('left', 2)
        with self.assertRaises(SafetyFault):
            self.observe(gate)
        self.assertIsNotNone(gate.fault)

    def test_identity_changes_cannot_be_hidden_by_original_source_returning(self):
        for changes, gid in (
                ({'boot_id': 'foreign'}, b'manus'), ({'side': 'right'}, b'manus'),
                ({'glove_id': 0}, b'manus'), ({'glove_id': 99}, b'manus'),
                ({'session_id': ''}, b'manus'),
                ({'session_id': '9e388afd-a697-4240-bd52-0462e34b92b2'}, b'manus'),
                ({}, b'other-writer')):
            with self.subTest(changes=changes, gid=gid):
                self.inbox = self.make_inbox(('left_hand',))
                self.receive_hand()
                self.inbox.receive_hand('left_hand', hand_message(sequence=2, **changes), gid)
                self.receive_hand(sequence=3)
                with self.assertRaises(SafetyFault):
                    self.inbox.drain()

    def test_invalid_valid_targets_and_metadata_are_sticky_faults(self):
        cases = (
            {'joint_names': list(reversed(HAND_JOINT_NAMES))},
            {'position_rad': [float('nan')] * 20}, {'position_rad': [0.] * 19},
            {'position_rad': [3.] * 20}, {'source_monotonic_ns': 0},
            {'source_monotonic_ns': NOW + 1}, {'sequence': 1},
        )
        for changes in cases:
            with self.subTest(changes=changes):
                self.inbox = self.make_inbox(('left_hand',))
                self.receive_hand()
                values = dict(sequence=2)
                values.update(changes)
                self.receive_hand(**values)
                self.receive_hand(sequence=3)
                with self.assertRaises(SafetyFault):
                    self.inbox.drain()

    def test_each_source_has_its_own_sequence_and_clock_history(self):
        self.inbox.receive(message(100), b'controller')
        self.receive_hand('left', 20)
        self.receive_hand('right', 2)
        MotionGate(configuration(), self.devices).validate_targets(self.inbox.drain(), self.now)
        self.receive_hand('right', 3)
        self.receive_hand('left', 19)
        self.inbox.receive(message(101), b'controller')
        with self.assertRaises(SafetyFault):
            self.inbox.drain()

    def test_arm_epoch_change_and_return_survives_hand_updates(self):
        gate = self.arm()
        self.inbox.receive(message(2, tracking_epoch=8), b'controller')
        self.receive_hand('left', 2)
        self.inbox.receive(message(3), b'controller')
        self.receive_hand('right', 2)
        with self.assertRaises(SafetyFault):
            self.observe(gate)
        self.assertIsNotNone(gate.fault)


def test_manus_graph_binding_requires_one_expected_resolved_publisher():
    from tianji_controller.ros_commands import _hand_publisher_gid
    endpoint = SimpleNamespace(node_name='_NODE_NAME_UNKNOWN_',
                               node_namespace='_NODE_NAMESPACE_UNKNOWN_',
                               topic_type='tianji_interfaces/msg/HandJointCommand',
                               endpoint_gid=b'manus')
    assert _hand_publisher_gid([endpoint]) is None
    endpoint.node_name = 'manus_hand2_retarget'
    endpoint.node_namespace = '/'
    assert _hand_publisher_gid([endpoint]) == b'manus'
    for changes, endpoints in (
            ({}, [endpoint, endpoint]),
            ({'node_name': 'other'}, [endpoint]),
            ({'node_namespace': '/other'}, [endpoint]),
            ({'topic_type': 'tianji_interfaces/msg/ControllerJointTargets'}, [endpoint])):
        with unittest.TestCase().assertRaises(SafetyFault):
            changed = SimpleNamespace(**vars(endpoint))
            for name, value in changes.items():
                setattr(changed, name, value)
            _hand_publisher_gid([changed] if len(endpoints) == 1 else endpoints)


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


def test_actual_dds_hand_only_receiver_detects_idle_graph_conflict(monkeypatch):
    import time
    import uuid
    from pathlib import Path
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from tianji_interfaces.msg import HandJointCommand
    from tianji_controller.ros_commands import CommandReceiver
    monkeypatch.setenv('ROS_DOMAIN_ID', '121')
    topic = '/test/hand_commands/s' + uuid.uuid4().hex
    context = Context()
    context.init(args=[])
    node = rogue = receiver = None
    try:
        receiver = CommandReceiver(None, configuration(), ('right_hand',),
                                   hand_topics={'right_hand': topic})
        assert receiver.drain() is None
        node = Node('manus_hand2_retarget', context=context)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        publisher = node.create_publisher(HandJointCommand, topic, qos)
        packet = HandJointCommand(
            boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            session_id=str(uuid.uuid4()), glove_id=22, side='right', valid=True,
            revocation_generation=0, joint_names=list(HAND_JOINT_NAMES),
            position_rad=[.2] * 20)
        deadline = time.monotonic() + 5
        result = None
        while time.monotonic() < deadline:
            packet.sequence += 1
            packet.source_monotonic_ns = time.monotonic_ns()
            publisher.publish(packet)
            time.sleep(.01)
            result = receiver.drain()
            if result is not None:
                break
        assert result is not None
        assert result.right_hand == (.2,) * 20
        MotionGate(configuration(), ('right_hand',)).validate_targets(result, time.monotonic_ns())
        # The conflicting writer never publishes: only the periodic graph scan
        # can detect it. No arm publisher or arm subscription is involved.
        rogue = Node('unexpected_hand_source', context=context)
        rogue.create_publisher(HandJointCommand, topic, qos)
        deadline = time.monotonic() + 5
        failure = None
        while time.monotonic() < deadline:
            try:
                receiver.drain()
            except SafetyFault as error:
                failure = str(error)
                break
            time.sleep(.02)
        assert failure is not None and 'multiple publishers' in failure
    finally:
        if receiver is not None:
            receiver.close()
        if rogue is not None:
            rogue.destroy_node()
        if node is not None:
            node.destroy_node()
        context.try_shutdown()


if __name__ == '__main__':
    unittest.main()
