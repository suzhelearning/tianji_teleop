import os
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from pico2_hands import spd_publisher


class _Transport:
    """Controlled DDS boundary; tests exercise the real handoff and worker."""
    def __init__(self):
        self.condition = threading.Condition()
        self.messages = []
        self.on_publish = lambda message: None

    def publish(self, message):
        self.on_publish(message)
        with self.condition:
            self.messages.append(message)
            self.condition.notify_all()

    def wait(self, predicate):
        with self.condition:
            if not self.condition.wait_for(lambda: predicate(self.messages), timeout=2):
                raise AssertionError("expected SPD output was not published")
            return list(self.messages)

    def modules(self):
        transport = self

        class Context:
            def init(self, **kwargs):
                pass

            def try_shutdown(self):
                pass

        class Node:
            def __init__(self, *args, **kwargs):
                pass

            def create_publisher(self, *args):
                return transport

            def destroy_node(self):
                pass

        class JointCommand(SimpleNamespace):
            def __init__(self, **kwargs):
                super().__init__(stamp=SimpleNamespace(sec=0, nanosec=0), **kwargs)

        return {
            "rclpy.context": SimpleNamespace(Context=Context),
            "rclpy.node": SimpleNamespace(Node=Node),
            "rclpy.utilities": SimpleNamespace(
                get_rmw_implementation_identifier=lambda: "rmw_fastrtps_cpp"),
            "rclpy.qos": SimpleNamespace(
                QoSProfile=lambda **kwargs: kwargs,
                ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=1),
                HistoryPolicy=SimpleNamespace(KEEP_LAST=1),
                DurabilityPolicy=SimpleNamespace(VOLATILE=1)),
            "tianji_spd_interfaces.msg": SimpleNamespace(JointCommand=JointCommand),
            "simulation.physics": SimpleNamespace(JOINT_NAMES=tuple(str(i) for i in range(54))),
        }


class SpdPublisherTest(unittest.TestCase):
    def setUp(self):
        self.transport = _Transport()
        # Test the real flock, without contending with a live self-test process.
        directory = tempfile.TemporaryDirectory(prefix="pico-publisher-test-")
        self.addCleanup(directory.cleanup)
        path_type = spd_publisher.Path
        paths = patch.object(
            spd_publisher, "Path",
            side_effect=lambda path: path_type(directory.name) if str(path) == "/tmp" else path_type(path),
        )
        paths.start()
        self.addCleanup(paths.stop)
        environment = patch.dict(os.environ, {"ROS_DOMAIN_ID": "121"})
        modules = patch.dict("sys.modules", self.transport.modules())
        environment.start()
        modules.start()
        self.addCleanup(environment.stop)
        self.addCleanup(modules.stop)

    def offer(self, publisher, value=0.0, session=("source", 1), age_ns=0):
        mono, wall = time.monotonic_ns(), time.time_ns()
        publisher.offer((value,) * 54, 7, session, mono - age_ns, wall - age_ns)
        return wall - age_ns

    def test_latest_session_barrier_survives_coalescing_while_dds_is_blocked(self):
        entered, release, offered = threading.Event(), threading.Event(), threading.Event()

        def block_first(message):
            if not entered.is_set():
                entered.set()
                if not release.wait(2):
                    raise TimeoutError("test DDS release missing")

        self.transport.on_publish = block_first
        with spd_publisher.SpdPublisher() as publisher:
            self.offer(publisher)
            self.assertTrue(entered.wait(2))

            def produce():
                self.offer(publisher, 1.0, ("source", 2))
                self.offer(publisher, 2.0, ("source", 2))
                offered.set()

            producer = threading.Thread(target=produce)
            producer.start()
            try:
                self.assertTrue(offered.wait(1), "producer waited on DDS")
            finally:
                release.set()
                producer.join(2)
            publisher.finish()
        messages = self.transport.messages
        new_session = messages[-1].session_id
        self.assertNotEqual(messages[0].session_id, new_session)
        current = [message for message in messages if message.session_id == new_session]
        self.assertEqual([message.ready_mask for message in current], [0, 7])
        self.assertLess(current[0].sequence, current[1].sequence)
        self.assertEqual(current[1].position_rad, (2.0,) * 54)
        self.assertFalse(any(message.position_rad == (1.0,) * 54 for message in messages))

    def test_expiration_generates_one_new_invalidation_and_reserves_sequence(self):
        with spd_publisher.SpdPublisher() as publisher:
            stamp = self.offer(publisher)
            messages = self.transport.wait(lambda rows: len(rows) >= 3)
            self.assertEqual([message.ready_mask for message in messages], [0, 7, 0])
            self.assertLess(messages[1].sequence, messages[2].sequence)
            healthy_stamp = messages[1].stamp.sec * 1_000_000_000 + messages[1].stamp.nanosec
            invalid_stamp = messages[2].stamp.sec * 1_000_000_000 + messages[2].stamp.nanosec
            self.assertEqual(healthy_stamp, stamp)
            self.assertGreater(invalid_stamp, stamp)
            with self.transport.condition:
                self.assertFalse(self.transport.condition.wait_for(
                    lambda: len(self.transport.messages) > 3, timeout=0.15))
            self.offer(publisher, 1.0)
            publisher.finish()
        self.assertGreater(self.transport.messages[-1].sequence, messages[2].sequence)
        self.assertEqual(self.transport.messages[-1].ready_mask, 7)

    def test_stale_final_home_is_attempted_but_not_advertised_healthy(self):
        with spd_publisher.SpdPublisher() as publisher:
            stamp = self.offer(publisher, 0.75, age_ns=150_000_000)
            publisher.finish()
        self.assertEqual(self.transport.messages[-1].position_rad, (0.75,) * 54)
        self.assertTrue(all(message.ready_mask == 0 for message in self.transport.messages))
        last = self.transport.messages[-1]
        self.assertEqual(last.stamp.sec * 1_000_000_000 + last.stamp.nanosec, stamp)

    def test_clock_jump_latches_until_explicit_new_session(self):
        real_wall = time.time_ns
        offset = [0]
        clock = SimpleNamespace(monotonic_ns=time.monotonic_ns, monotonic=time.monotonic,
                                time_ns=lambda: real_wall() + offset[0])
        with patch.object(spd_publisher, "time", clock), spd_publisher.SpdPublisher() as publisher:
            self.offer(publisher)
            self.transport.wait(lambda rows: any(message.ready_mask == 7 for message in rows))
            offset[0] = 30_000_000
            publisher.offer((1.0,) * 54, 7, ("source", 1), time.monotonic_ns(), clock.time_ns())
            self.transport.wait(lambda rows: rows[-1].position_rad == (1.0,) * 54)
            offset[0] = 0
            self.offer(publisher, 2.0)
            self.transport.wait(lambda rows: rows[-1].position_rad == (2.0,) * 54)
            self.assertEqual(self.transport.messages[-1].ready_mask, 0)
            self.offer(publisher, 3.0, ("source", 2))
            publisher.finish()
        self.assertEqual(self.transport.messages[-1].ready_mask, 7)
        self.assertTrue(all(message.ready_mask == 0 for message in self.transport.messages
                            if message.position_rad in ((1.0,) * 54, (2.0,) * 54)))

    def test_publish_failure_is_forwarded_and_lock_released(self):
        failed = threading.Event()

        def fail(message):
            failed.set()
            raise RuntimeError("injected DDS failure")

        self.transport.on_publish = fail
        publisher = spd_publisher.SpdPublisher()
        self.offer(publisher)
        self.assertTrue(failed.wait(2))
        with self.assertRaisesRegex(RuntimeError, "injected DDS failure"):
            publisher.finish()
        with self.assertRaisesRegex(RuntimeError, "injected DDS failure"):
            publisher.close()
        self.transport.on_publish = lambda message: None
        with spd_publisher.SpdPublisher():
            pass

    def test_domain_lock_rejects_concurrent_owner(self):
        with spd_publisher.SpdPublisher():
            with self.assertRaisesRegex(RuntimeError, "another PICO SPD publisher"):
                spd_publisher.SpdPublisher()

    def test_invalid_numerics_and_mask_never_reach_transport(self):
        with spd_publisher.SpdPublisher() as publisher:
            mono, wall = time.monotonic_ns(), time.time_ns()
            for joints, mask in (((float("nan"),) * 54, 7), ((0.0,) * 54, 8)):
                with self.subTest(mask=mask), self.assertRaises(ValueError):
                    publisher.offer(joints, mask, ("source", 1), mono, wall)
        self.assertEqual(self.transport.messages, [])

    def test_zero_timestamps_never_reach_transport(self):
        with spd_publisher.SpdPublisher() as publisher:
            for mono, wall in ((0, time.time_ns()), (time.monotonic_ns(), 0)):
                with self.subTest(monotonic=mono, utc=wall), self.assertRaises(ValueError):
                    publisher.offer((0.0,) * 54, 7, ("source", 1), mono, wall)
        self.assertEqual(self.transport.messages, [])

    def test_already_loaded_wrong_rmw_fails_startup_and_releases_lock(self):
        with patch.dict("sys.modules", {
                "rclpy.utilities": SimpleNamespace(
                    get_rmw_implementation_identifier=lambda: "rmw_cyclonedds_cpp")}):
            with self.assertRaisesRegex(RuntimeError, "requires active rmw_fastrtps_cpp"):
                spd_publisher.SpdPublisher()
        with spd_publisher.SpdPublisher():
            pass
