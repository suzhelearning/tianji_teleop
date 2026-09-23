"""Latest-only ROS commands; DDS and identity checks stay off the motion thread."""
from dataclasses import replace
from pathlib import Path
import fcntl
import os
import threading
import time
import uuid

from .protocol import ARMS_READY, CommandFrame
from .safety import MotionGate, SafetyFault, _finite_vector


def _controller_publisher_gid(endpoints):
    """DDS may discover a writer before its ROS node metadata arrives."""
    if not endpoints:
        return None
    if len(endpoints) != 1:
        raise SafetyFault('controller topic has multiple publishers')
    endpoint = endpoints[0]
    if endpoint.topic_type != 'tianji_interfaces/msg/ControllerJointTargets':
        raise SafetyFault('controller topic has the wrong message type')
    if (endpoint.node_name == '_NODE_NAME_UNKNOWN_' or
            endpoint.node_namespace == '_NODE_NAMESPACE_UNKNOWN_'):
        return None  # Not trusted and not consumed; normal discovery is pending.
    if endpoint.node_name != 'tianji_arm_core' or endpoint.node_namespace != '/':
        raise SafetyFault('controller topic publisher is not /tianji_arm_core')
    gid = bytes(endpoint.endpoint_gid)
    return gid if gid and any(gid) else None


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
    """Bounded handoff retaining unsafe events until the motion gate sees them."""

    def __init__(self, safety, devices, *, boot_id=None, clock=time.monotonic_ns):
        self._boot_id = boot_id or Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self._clock = clock
        self._bounds = MotionGate(safety, devices)
        self._timeout_ns = int(safety['command_timeout_s'] * 1e9)
        self._lock = threading.Lock()
        self._identity = None
        self._fault = None
        self._pending_flags = 7
        self._epoch_event = None
        self._input_ns = 0
        self.latest = None
        self.count = 0

    def fail(self, reason):
        with self._lock:
            if self._fault is None:
                self._fault = str(reason)

    def receive(self, message, publisher_gid):
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
                if not -5_000_000 <= now - stamp <= self._timeout_ns:
                    raise SafetyFault('controller production timestamp is stale or invalid')
                if self.latest is not None and (
                        message.sequence <= self.latest.sequence or stamp <= self.latest.timestamp_ns):
                    raise SafetyFault('controller command sequence or timestamp regressed')
                if message.flags & ARMS_READY:
                    if message.tracking_epoch <= 0 or message.input_monotonic_ns <= 0:
                        raise SafetyFault('ready arms lack applied PICO input metadata')
                    if (not -5_000_000 <= now - message.input_monotonic_ns <= self._timeout_ns
                            or message.input_monotonic_ns > stamp + 5_000_000):
                        raise SafetyFault('applied PICO input is stale or its clock is invalid')
                    if (self.latest is not None and self.latest.flags & ARMS_READY
                            and message.tracking_epoch == self.latest.tracking_epoch
                            and message.input_monotonic_ns < self._input_ns):
                        raise SafetyFault('applied PICO input timestamp regressed')
                frame = CommandFrame(message.sequence, stamp, message.tracking_epoch, message.flags,
                                     _finite_vector(message.left_arm, 7, 'left arm'),
                                     _finite_vector(message.right_arm, 7, 'right arm'),
                                     _finite_vector(message.left_hand, 20, 'left hand'),
                                     _finite_vector(message.right_hand, 20, 'right hand'))
                for device in self._bounds.devices:
                    # Even unready frames must not smuggle out-of-range references.
                    self._bounds._check_bounds(device, frame.positions(device), 'target')
                if (self.latest is not None and frame.tracking_epoch != self.latest.tracking_epoch
                        and self._epoch_event is None):
                    self._epoch_event = frame
                self._pending_flags &= frame.flags
                self._identity = identity
                self._input_ns = message.input_monotonic_ns
                self.latest = frame
                self.count += 1
            except (SafetyFault, ValueError, TypeError, OverflowError, AttributeError) as error:
                self._fault = str(error)

    def drain(self, validate=None):
        with self._lock:
            if self._fault is not None:
                raise SafetyFault(self._fault)
            latest = self.latest
            flags, epoch_event = self._pending_flags, self._epoch_event
            input_ns = self._input_ns
            self._pending_flags = 7
            self._epoch_event = None
        if latest is not None:
            # A fresh output timestamp cannot refresh the input actually applied.
            if latest.flags & ARMS_READY and not -5_000_000 <= self._clock() - input_ns <= self._timeout_ns:
                latest = replace(latest, flags=latest.flags & ~ARMS_READY)
            if validate is not None:
                if flags != 7:
                    validate(replace(latest, flags=latest.flags & flags))
                if epoch_event is not None:
                    validate(epoch_event)
                validate(latest)
        return latest


class CommandReceiver(CommandInbox):
    """Own a background ROS context; never spin or inspect the graph in drain()."""

    def __init__(self, topic, safety, devices):
        super().__init__(safety, devices)
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
            from tianji_interfaces.msg import ControllerJointTargets

            context = Context()
            context.init(args=[])
            node = Node('tianji_command_receiver', context=context)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
            bound_gid = None

            def publisher():
                nonlocal bound_gid
                try:
                    gid = _controller_publisher_gid(node.get_publishers_info_by_topic(self._topic))
                except SafetyFault as error:
                    self.fail(str(error))
                    return None
                if gid is None:
                    return None
                if bound_gid is not None and gid != bound_gid:
                    self.fail('controller publisher endpoint changed')
                    return None
                bound_gid = gid
                return gid

            def receive(message):
                gid = publisher()
                # Jazzy MessageInfo has no publisher_gid. Only a fully resolved,
                # unique graph endpoint may supply the identity; until discovery
                # completes, drop samples without refreshing the command cache.
                if gid is not None:
                    self.receive(message, gid)

            node.create_subscription(ControllerJointTargets, self._topic, receive, qos)
            node.create_timer(0.1, publisher)
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
