"""PICO bare-hand simulator output only; no hardware command transport.

The advisory lock excludes other instances of this publisher in this host/domain,
not arbitrary external DDS writers. ROS initialization and publication belong to
one worker; the simulation only replaces an immutable, single-slot snapshot.
"""
from dataclasses import dataclass
import fcntl
import math
import os
from pathlib import Path
import stat
import threading
import time
import uuid


TOPIC = "/spd/tianji_wuji2/v1/joint_command"
ROBOT_CONFIG = "tianji_wuji2_v1"
MAX_AGE_NS = 100_000_000
MAX_FUTURE_NS = 5_000_000
PERIOD_NS = 16_666_667


@dataclass(frozen=True, slots=True)
class _Snapshot:
    joints: tuple
    ready_mask: int
    session_id: str
    sequence: int
    ticket: int
    monotonic_ns: int
    utc_ns: int


class SpdPublisher:
    def __init__(self):
        os.environ.setdefault("ROS_DOMAIN_ID", "120")
        self.domain = int(os.environ["ROS_DOMAIN_ID"])
        if not 0 <= self.domain <= 232:
            raise ValueError("ROS_DOMAIN_ID must be in [0, 232]")
        os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
        os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
        os.environ.pop("ROS_LOCALHOST_ONLY", None)
        os.environ.pop("ROS_STATIC_PEERS", None)
        self._condition = threading.Condition()
        self._ready = threading.Event()
        self._stop = False
        self._finishing = False
        self._error = None
        self._slot = None
        self._session_key = None
        self._session_id = None
        self._barrier = False
        self._sequence = 1
        self._ticket = self._attempted = 0
        self._clock_offset_ns = self._last_clock_ns = None
        self._clock_failed = False
        self._thread = None
        self._lock_fd = None
        # Fixed host temporary directory: per-project TMPDIR must not bypass it.
        path = Path("/tmp") / f"tianji-pico-spd-publisher-domain-{self.domain}.lock"
        fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
        try:
            info = os.fstat(fd)
            if info.st_uid != os.getuid() or not stat.S_ISREG(info.st_mode):
                raise RuntimeError("SPD publisher lock has an unsafe owner or type")
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise RuntimeError(f"another PICO SPD publisher owns ROS domain {self.domain}") from error
        except BaseException:
            os.close(fd)
            raise
        self._lock_fd = fd
        try:
            self._thread = threading.Thread(target=self._run, name="pico-spd-dds", daemon=True)
            self._thread.start()
            if not self._ready.wait(10.0):
                raise TimeoutError("SPD ROS publisher initialization timed out")
            self.check()
        except BaseException:
            self.close()
            raise

    def check(self):
        """Forward worker failures without waiting for DDS."""
        with self._condition:
            error = self._error
        if error is not None:
            raise RuntimeError(f"SPD ROS publisher failed: {error}") from error

    def offer(self, joints: tuple, ready_mask: int, session_key: tuple,
              generated_monotonic_ns: int, generated_utc_ns: int) -> None:
        """Replace the pending output; never serialize or wait on DDS here."""
        if not isinstance(joints, tuple) or len(joints) != 54 or not all(map(math.isfinite, joints)):
            raise ValueError("SPD output requires an immutable tuple of 54 finite joints")
        if type(ready_mask) is not int or not 0 <= ready_mask <= 7:
            raise ValueError("SPD ready_mask must contain only arms/right/left bits")
        if not isinstance(session_key, tuple):
            raise ValueError("SPD session_key must be an immutable tuple")
        hash(session_key)
        if (type(generated_monotonic_ns) is not int or generated_monotonic_ns <= 0
                or type(generated_utc_ns) is not int
                or not 0 < generated_utc_ns < (1 << 31) * 1_000_000_000):
            raise ValueError("SPD output requires positive monotonic and representable UTC nanoseconds")
        with self._condition:
            if self._error is not None:
                raise RuntimeError(f"SPD ROS publisher failed: {self._error}") from self._error
            if self._stop or self._finishing:
                raise RuntimeError("SPD ROS publisher is closing")
            now = time.monotonic_ns()
            wall = time.time_ns()
            if self._session_id is None or session_key != self._session_key:
                self._session_key = session_key
                self._session_id = str(uuid.uuid4())
                self._sequence = 1  # Sequence 1 is the mandatory invalid session barrier.
                self._barrier = True
                self._clock_offset_ns = wall - now
                self._last_clock_ns = now
                self._clock_failed = False
            self._observe_clock(now, wall)
            if abs(generated_utc_ns - generated_monotonic_ns - self._clock_offset_ns) > MAX_FUTURE_NS:
                self._clock_failed = True
            self._sequence += 1
            self._ticket += 1
            self._slot = _Snapshot(joints, ready_mask, self._session_id, self._sequence,
                                   self._ticket, generated_monotonic_ns, generated_utc_ns)
            self._condition.notify_all()

    def _observe_clock(self, now, wall):
        # Called under the condition. A new explicit session is the only reset.
        if (now < self._last_clock_ns
                or abs(wall - now - self._clock_offset_ns) > MAX_FUTURE_NS):
            self._clock_failed = True
        self._last_clock_ns = now

    def _mask(self, snapshot, now, wall):
        self._observe_clock(now, wall)
        if (self._clock_failed
                or not -MAX_FUTURE_NS <= now - snapshot.monotonic_ns <= MAX_AGE_NS
                or not -MAX_FUTURE_NS <= wall - snapshot.utc_ns <= MAX_AGE_NS):
            return 0
        return snapshot.ready_mask

    def finish(self, timeout_s=2.0):
        """Freeze offers and wait for the final snapshot's local publish attempt.

        This is not a remote acknowledgement. A stale final output is attempted
        with mask zero, never with a refreshed stamp or readiness.
        """
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("finite positive finish timeout required")
        deadline = time.monotonic() + timeout_s
        with self._condition:
            self._finishing = True
            target = self._ticket
            while self._attempted < target:
                if self._error is not None:
                    raise RuntimeError(f"SPD ROS publisher failed: {self._error}") from self._error
                if self._stop:
                    raise RuntimeError("SPD ROS publisher stopped before final output")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("SPD final output publish attempt timed out")
                self._condition.wait(remaining)
        self.check()

    def close(self):
        """Stop with bounded waiting; retain exclusivity if DDS is stuck."""
        with self._condition:
            self._stop = True
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(2.0)
            if self._thread.is_alive():
                # Releasing this lock would permit two live publisher workers.
                raise TimeoutError("SPD ROS publisher did not stop; domain lock retained")
        if self._lock_fd is not None:
            os.close(self._lock_fd)
            self._lock_fd = None
        self.check()

    def __enter__(self):
        self.check()
        return self

    def __exit__(self, exc_type, exc, traceback):
        try:
            if exc_type is None:
                self.finish()
        finally:
            self.close()

    def _run(self):
        context = node = None
        try:
            from rclpy.context import Context
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
            from rclpy.utilities import get_rmw_implementation_identifier
            from tianji_spd_interfaces.msg import JointCommand
            from simulation.physics import JOINT_NAMES

            context = Context()
            # Direct Context.init does not install rclpy's process signal handlers.
            context.init(args=[], domain_id=self.domain)
            if get_rmw_implementation_identifier() != "rmw_fastrtps_cpp":
                raise RuntimeError("SPD ROS publisher requires active rmw_fastrtps_cpp")
            node = Node("pico_hand_spd_" + uuid.uuid4().hex, context=context,
                        use_global_arguments=False, enable_rosout=False,
                        start_parameter_services=False)
            qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            publisher = node.create_publisher(JointCommand, TOPIC, qos)
            self._ready.set()
            next_send = 0
            sent_ticket = 0
            sent_mask = 0
            while True:
                with self._condition:
                    if self._stop:
                        break
                    snapshot = self._slot
                    now = time.monotonic_ns()
                    if snapshot is None:
                        self._condition.wait()
                        continue
                    wall = time.time_ns()
                    mask = self._mask(snapshot, now, wall)
                    barrier = self._barrier
                    pending = barrier or snapshot.ticket != sent_ticket or (sent_mask != 0 and mask == 0)
                    if not pending or now < next_send:
                        # Continue clock/expiry observation even without new offers.
                        wait_ns = max(1, next_send - now) if pending else PERIOD_NS
                        self._condition.wait(wait_ns / 1_000_000_000)
                        continue
                    if not barrier and snapshot.ticket == sent_ticket:
                        # This is a new invalidation, not a replay of the expired
                        # command. Never give old healthy output a fresh stamp.
                        # Reserve its sequence here so the next offer is newer.
                        self._sequence += 1
                        snapshot = _Snapshot(snapshot.joints, 0, snapshot.session_id,
                                             self._sequence, snapshot.ticket, now, wall)
                        self._slot = snapshot
                message = JointCommand(
                    schema_version=1, robot_config=ROBOT_CONFIG,
                    session_id=snapshot.session_id, sequence=1 if barrier else snapshot.sequence,
                    ready_mask=0, joint_names=JOINT_NAMES, position_rad=snapshot.joints)
                message.stamp.sec, message.stamp.nanosec = divmod(snapshot.utc_ns, 1_000_000_000)
                with self._condition:
                    if self._stop:
                        break
                    if self._slot is not snapshot:
                        continue
                    # Recheck after allocation/serialization setup, immediately before DDS.
                    mask = self._mask(snapshot, time.monotonic_ns(), time.time_ns())
                    message.ready_mask = 0 if barrier else mask
                publisher.publish(message)
                next_send = time.monotonic_ns() + PERIOD_NS
                with self._condition:
                    if barrier:
                        if self._session_id == snapshot.session_id:
                            self._barrier = False
                    else:
                        sent_ticket, sent_mask = snapshot.ticket, message.ready_mask
                        self._attempted = max(self._attempted, snapshot.ticket)
                    self._condition.notify_all()
        except BaseException as error:
            with self._condition:
                self._error = error
                self._condition.notify_all()
        finally:
            try:
                if node is not None:
                    node.destroy_node()
            except BaseException as error:
                with self._condition:
                    if self._error is None:
                        self._error = error
            finally:
                try:
                    if context is not None:
                        context.try_shutdown()
                except BaseException as error:
                    with self._condition:
                        if self._error is None:
                            self._error = error
                self._ready.set()
                with self._condition:
                    self._condition.notify_all()
