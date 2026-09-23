"""Latest-only ROS commands; DDS and identity checks stay off the motion thread."""
from dataclasses import replace
from pathlib import Path
import fcntl
import os
import threading
import time
import uuid

from .protocol import ARMS_READY, DEVICE_READY_FLAGS, CommandFrame
from .safety import MotionGate, SafetyFault, _finite_vector

HAND_JOINT_NAMES = tuple(
    f'{finger}_S{joint}'
    for finger in ('thumb', 'index', 'middle', 'ring', 'pinky')
    for joint in range(1, 5)
)


def _hand_publisher_gid(endpoints):
    return _publisher_gid(endpoints, 'HandJointCommand', 'manus_hand2_retarget', 'hand')


def _publisher_gid(endpoints, message_type, node_name, label):
    if not endpoints:
        return None
    if len(endpoints) != 1:
        raise SafetyFault(f'{label} topic has multiple publishers')
    endpoint = endpoints[0]
    if endpoint.topic_type != f'tianji_interfaces/msg/{message_type}':
        raise SafetyFault(f'{label} topic has the wrong message type')
    if (endpoint.node_name == '_NODE_NAME_UNKNOWN_' or
            endpoint.node_namespace == '_NODE_NAMESPACE_UNKNOWN_'):
        return None
    if endpoint.node_name != node_name or endpoint.node_namespace != '/':
        raise SafetyFault(f'{label} topic publisher is not /{node_name}')
    gid = bytes(endpoint.endpoint_gid)
    return gid if gid and any(gid) else None


def _controller_publisher_gid(endpoints):
    """DDS may discover a writer before its ROS node metadata arrives."""
    return _publisher_gid(endpoints, 'ControllerJointTargets', 'tianji_arm_core', 'controller')


class ExecutorLease:
    """Machine-wide executor/HOME exclusion, independent of ROS domain or topic."""

    def __init__(self):
        self._fd = os.open('/tmp/tianji-hardware-executor.lock',
                           os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW, 0o666)
        try:
            fcntl.flock(self._fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            os.close(self._fd)
            self._fd = None
            raise SafetyFault('another teleoperation or HOME executor holds the hardware lease')

    def close(self):
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None
        # Never unlink: another waiter could hold the old inode.


class CommandInbox:
    """Latest commands with per-topic receipt watchdogs and sticky unsafe events.

    Producers own operator-input freshness. This boundary honors their readiness
    and revocations without rechecking PICO/Manus sample ages.
    CommandFrame.timestamp_ns is the oldest selected cmd receipt time; producer
    timestamps are retained only for stream ordering, not watchdog deadlines.
    """

    def __init__(self, safety, devices, *, hand_topics=None, boot_id=None, clock=time.monotonic_ns):
        self._boot_id = boot_id or Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self._clock = clock
        self._bounds = MotionGate(safety, devices)
        self._timeout_ns = int(safety['command_timeout_s'] * 1e9)
        self._hand_topics = None if hand_topics is None else {
            device: hand_topics[device] for device in devices if device != 'arms'
        }
        if self._hand_topics is not None:
            topics = tuple(self._hand_topics.values())
            if any(not isinstance(topic, str) or not topic for topic in topics):
                raise ValueError('selected Manus hands require nonempty command topics')
            if len(set(topics)) != len(topics):
                raise ValueError('selected Manus hands require distinct command topics')
        # ROS callbacks are serialized; the motion thread only reads the bounded
        # immutable snapshot under _lock, never the producer-owned source map.
        self._sources = {}
        self._source_ages = ()
        self._lock = threading.Lock()
        self._identity = None
        self._fault = None
        self._pending_flags = 7
        self._epoch_event = None
        self._production_ns = 0
        self.latest = None
        self.count = 0

    def fail(self, reason):
        with self._lock:
            if self._fault is None:
                self._fault = str(reason)

    def receive(self, message, publisher_gid):
        if self._hand_topics is not None:
            if 'arms' in self._bounds.devices:
                self._receive_independent('arms', message, publisher_gid)
            return
        with self._lock:
            if self._fault is not None:
                return
            try:
                now = self._clock()
                if message.boot_id != self._boot_id:
                    raise SafetyFault('controller command has a foreign host boot')
                if str(uuid.UUID(message.session_id)) != message.session_id:
                    raise SafetyFault('controller session must be a canonical UUID')
                gid = bytes(publisher_gid)
                if not gid or not any(gid):
                    raise SafetyFault('controller publisher identity is missing')
                identity = (gid, message.session_id)
                if self._identity is not None and identity != self._identity:
                    raise SafetyFault('controller publisher or session changed')
                stamp = message.produced_monotonic_ns
                if message.sequence <= 0 or stamp <= 0 or message.flags & ~7:
                    raise SafetyFault('invalid controller command metadata')
                if self.latest is not None and (
                        message.sequence <= self.latest.sequence or stamp <= self._production_ns):
                    raise SafetyFault('controller command sequence or timestamp regressed')
                if message.flags & ARMS_READY and message.tracking_epoch <= 0:
                    raise SafetyFault('ready arms lack tracking epoch')
                frame = CommandFrame(message.sequence, now, message.tracking_epoch, message.flags,
                                     _finite_vector(message.left_arm, 7, 'left arm'),
                                     _finite_vector(message.right_arm, 7, 'right arm'),
                                     _finite_vector(message.left_hand, 20, 'left hand'),
                                     _finite_vector(message.right_hand, 20, 'right hand'),
                                     getattr(message, 'reference_id', 0), message.session_id)
                for device in self._bounds.devices:
                    # Even unready frames must not smuggle out-of-range references.
                    self._bounds._check_bounds(device, frame.positions(device), 'target')
                if (self.latest is not None and frame.tracking_epoch != self.latest.tracking_epoch
                        and self._epoch_event is None):
                    self._epoch_event = frame
                self._pending_flags &= frame.flags
                if self.latest is not None and now - self.latest.timestamp_ns > self._timeout_ns:
                    self._pending_flags &= ~self.latest.flags
                self._identity = identity
                self._production_ns = stamp
                self.latest = frame
                self.count += 1
            except (SafetyFault, ValueError, TypeError, OverflowError, AttributeError) as error:
                self._fault = str(error)

    def receive_hand(self, device, message, publisher_gid):
        if self._hand_topics is None:
            raise ValueError('independent hand input requires hand_topics')
        if device in self._hand_topics:
            self._receive_independent(device, message, publisher_gid)

    def _receive_independent(self, device, message, publisher_gid):
        """Validate on the single DDS producer thread, then swap a small snapshot."""
        try:
            now = self._clock()
            previous = self._sources.get(device)
            if message.boot_id != self._boot_id:
                raise SafetyFault(f'{device}: command has a foreign host boot')
            if str(uuid.UUID(message.session_id)) != message.session_id:
                raise SafetyFault(f'{device}: session must be a canonical UUID')
            gid = bytes(publisher_gid)
            if not gid or not any(gid):
                raise SafetyFault(f'{device}: publisher identity is missing')
            identity = (gid, message.session_id)
            if device == 'arms':
                stamp = message.produced_monotonic_ns
                generation = 0
                if message.flags & ~7:
                    raise SafetyFault('invalid controller command flags')
                ready = bool(message.flags & ARMS_READY)
                epoch = message.tracking_epoch
                positions = (_finite_vector(message.left_arm, 7, 'left arm') +
                             _finite_vector(message.right_arm, 7, 'right arm'))
                self._bounds._check_bounds(device, positions, 'target')
                if ready and epoch <= 0:
                    raise SafetyFault('ready arms lack tracking epoch')
                age_ns = now
            else:
                if message.side != device.removesuffix('_hand'):
                    raise SafetyFault(f'{device}: hand side/topic mismatch')
                if message.glove_id <= 0:
                    raise SafetyFault(f'{device}: missing glove identity')
                identity += (message.glove_id,)
                if tuple(message.joint_names) != HAND_JOINT_NAMES:
                    raise SafetyFault(f'{device}: wrong ordered Hand2 joint names')
                stamp = message.source_monotonic_ns
                ready = bool(message.valid)
                epoch = 0
                generation = message.revocation_generation
                if generation < 0:
                    raise SafetyFault(f'{device}: invalid revocation generation')
                if previous is not None and generation < previous['generation']:
                    raise SafetyFault(f'{device}: revocation generation regressed')
                # Invalid producer messages deliberately contain NaNs and may
                # repeat the last source stamp/sequence. They revoke only.
                positions = (self._bounds._check_bounds(device, message.position_rad, 'target')
                             if ready else previous['positions'] if previous else (0.,) * 20)
                age_ns = now if ready else previous['age_ns'] if previous else now
            if previous is not None and identity != previous['identity']:
                raise SafetyFault(f'{device}: publisher, session or glove identity changed')
            if message.sequence <= 0 or stamp <= 0:
                raise SafetyFault(f'{device}: invalid source sequence or timestamp')
            if previous is not None:
                if (message.sequence < previous['sequence'] or stamp < previous['stamp'] or
                        ((device == 'arms' or ready) and
                         (message.sequence == previous['sequence'] or stamp == previous['stamp']))):
                    raise SafetyFault(f'{device}: source sequence or timestamp regressed')
            revoked = not ready or (
                previous is not None and previous['ready'] and
                now - previous['age_ns'] > self._timeout_ns)
            if previous is not None and generation > previous['generation']:
                revoked = True
            epoch_changed = (device == 'arms' and previous is not None and
                             epoch != previous['epoch'])
            self._sources[device] = dict(
                identity=identity, sequence=message.sequence, stamp=stamp,
                age_ns=age_ns, ready=ready, positions=positions,
                epoch=epoch, generation=generation,
                reference_id=getattr(message, 'reference_id', 0) if device == 'arms' else 0)
            self._publish_combined(device, revoked, epoch_changed)
        except (SafetyFault, ValueError, TypeError, OverflowError, AttributeError) as error:
            self.fail(error)

    def _publish_combined(self, device, revoked=False, epoch_changed=False):
        sources = self._sources
        arms = sources.get('arms')
        left = sources.get('left_hand')
        right = sources.get('right_hand')
        flags = sum(DEVICE_READY_FLAGS[name] for name, source in sources.items() if source['ready'])
        ages = tuple((DEVICE_READY_FLAGS[name], source['age_ns']) for name, source in sources.items())
        frame = CommandFrame(
            self.count + 1, min(stamp for _, stamp in ages), arms['epoch'] if arms else 0, flags,
            arms['positions'][:7] if arms else (0.,) * 7,
            arms['positions'][7:] if arms else (0.,) * 7,
            left['positions'] if left else (0.,) * 20,
            right['positions'] if right else (0.,) * 20,
            arms['reference_id'] if arms else 0,
            arms['identity'][1] if arms else '')
        with self._lock:
            if self._fault is not None:
                return
            if revoked:
                self._pending_flags &= ~DEVICE_READY_FLAGS[device]
            if epoch_changed and self._epoch_event is None:
                self._epoch_event = frame
            self._source_ages = ages
            self.latest = frame
            self.count += 1

    def revoke(self, device):
        """Graph disappearance is a source event, even if it recovers before drain."""
        if self._hand_topics is None:
            with self._lock:
                self._pending_flags &= ~sum(DEVICE_READY_FLAGS[name] for name in self._bounds.devices)
            return
        previous = self._sources.get(device)
        if previous is not None and previous['ready']:
            self._sources[device] = dict(previous, ready=False)
            self._publish_combined(device, revoked=True)

    def drain(self, validate=None):
        with self._lock:
            if self._fault is not None:
                raise SafetyFault(self._fault)
            latest = self.latest
            flags, epoch_event = self._pending_flags, self._epoch_event
            source_ages = self._source_ages
            self._pending_flags = 7
            self._epoch_event = None
        if latest is not None:
            # Only accepted cmd receipt renews the local watchdog, not drain()
            # calls or updates from another device.
            if self._hand_topics is None:
                if not 0 <= self._clock() - latest.timestamp_ns <= self._timeout_ns:
                    latest = replace(latest, flags=0)
            else:
                now = self._clock()
                for flag, stamp in source_ages:
                    if not 0 <= now - stamp <= self._timeout_ns:
                        latest = replace(latest, flags=latest.flags & ~flag)
            if validate is not None:
                if flags != 7:
                    validate(replace(latest, flags=latest.flags & flags))
                if epoch_event is not None:
                    validate(epoch_event)
                validate(latest)
        return latest


class CommandReceiver(CommandInbox):
    """Own a background ROS context; never spin or inspect the graph in drain()."""

    def __init__(self, topic, safety, devices, *, hand_topics=None):
        super().__init__(safety, devices, hand_topics=hand_topics)
        if self._hand_topics is not None and topic in self._hand_topics.values():
            raise ValueError('controller and Manus hand command topics must be distinct')
        self._topic = topic
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread = threading.Thread(target=self._run, name='controller-commands-dds', daemon=True)
        self._lease = ExecutorLease()
        try:
            self._thread.start()
        except BaseException:
            self._lease.close()
            raise
        if not self._ready.wait(5):
            self.close()
            raise SafetyFault('controller command ROS initialization timed out')
        if self._fault is not None:
            self.close()
            raise SafetyFault(self._fault)

    def _run(self):
        context = node = executor = None
        try:
            os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
            os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
            os.environ.setdefault("ROS_DOMAIN_ID", "120")
            from rclpy.node import Node
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
            from tianji_interfaces.msg import ControllerJointTargets, HandJointCommand

            context = Context()
            context.init(args=[])
            node = Node('tianji_command_receiver', context=context)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
            streams = []
            if self._hand_topics is None or 'arms' in self._bounds.devices:
                streams.append(('arms', self._topic, ControllerJointTargets, _controller_publisher_gid))
            if self._hand_topics is not None:
                streams.extend((device, topic, HandJointCommand, _hand_publisher_gid)
                               for device, topic in self._hand_topics.items())
            bound = {}
            discovered = {}

            def publishers():
                for device, topic, _, identify in streams:
                    try:
                        gid = identify(node.get_publishers_info_by_topic(topic))
                        if gid is not None:
                            if device in bound and gid != bound[device]:
                                raise SafetyFault(f'{device}: publisher endpoint changed')
                            bound[device] = gid
                        elif device in bound:
                            self.revoke(device)
                        discovered[device] = gid
                    except SafetyFault as error:
                        self.fail(error)
                        discovered[device] = None

            def callback(device):
                def receive(message, info):
                    gid = discovered.get(device)
                    # Some ROS distributions omit publisher_gid. In that case
                    # only a fully resolved, unique graph endpoint is authority.
                    if gid is None:
                        return
                    sample_gid = getattr(info, 'publisher_gid', None)
                    if sample_gid is not None and bytes(sample_gid) != gid:
                        self.fail(f'{device}: sample publisher endpoint changed')
                        return
                    if device == 'arms':
                        self.receive(message, gid)
                    else:
                        self.receive_hand(device, message, gid)
                return receive

            for device, topic, message_type, _ in streams:
                node.create_subscription(message_type, topic, callback(device), qos)
            publishers()
            node.create_timer(0.1, publishers)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            self._ready.set()
            while not self._stop.is_set():
                executor.spin_once(timeout_sec=0.05)
        except Exception as error:
            self.fail(f'controller command ROS receiver failed: {error}')
        finally:
            self._ready.set()
            if executor is not None:
                executor.shutdown()
            if node is not None:
                node.destroy_node()
            if context is not None and context.ok():
                context.shutdown()

    def close(self):
        self._stop.set()
        self._thread.join(timeout=5)
        if self._thread.is_alive():
            raise SafetyFault('controller command ROS receiver did not stop')
        self._lease.close()
