"""Wuji 真机控制入口、安全映射与 SDK 生命周期测试。"""

from __future__ import annotations

import math
import unittest
import threading
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace

from data_glove_wuji_teleop.adapters.hardware.command_safety import (
    AdaptiveTargetFilter,
    AngleLowPassFilter,
    FIRMWARE_DOF_ORDER,
    FirmwareCommandMapper,
    MotionLimiter,
    RealtimeVelocityLimiter,
)
from data_glove_wuji_teleop.adapters.hardware.safe_publisher import (
    HardwareSafetyOptions,
    SafeHardwarePublisher,
)
from data_glove_wuji_teleop.adapters.runtime.simulation_session import (
    SimulationLease,
    require_active_simulation,
)
from data_glove_wuji_teleop.adapters.hardware.wuji_sdk_driver import (
    open_wuji_sdk_driver,
    validate_v2_mit_gains,
)
from data_glove_wuji_teleop.adapters.hardware.wuji_v2_limits import (
    load_v2_firmware_limits,
)
from data_glove_wuji_teleop.adapters.simulation.mujoco_wuji_v2 import (
    build_control_bindings,
    load_model,
)
from data_glove_wuji_teleop.domain.hand_target import DOF_ORDER, HandTarget
from data_glove_wuji_teleop.project import (
    PROJECT_ROOT,
    get_hand_resources,
)


class SimulationGuardTest(unittest.TestCase):
    def test_real_control_requires_matching_running_simulator(self) -> None:
        with TemporaryDirectory() as directory:
            runtime_dir = Path(directory)
            with self.assertRaises(RuntimeError):
                require_active_simulation(
                    generation="v2",
                    hand="left",
                    runtime_dir=runtime_dir,
                )

            with SimulationLease(
                generation="v2",
                hand="left",
                runtime_dir=runtime_dir,
            ):
                session = require_active_simulation(
                    generation="v2",
                    hand="left",
                    runtime_dir=runtime_dir,
                )
                self.assertEqual(session.generation, "v2")
                self.assertEqual(session.hand, "left")

                with self.assertRaises(RuntimeError):
                    require_active_simulation(
                        generation="v1",
                        hand="left",
                        runtime_dir=runtime_dir,
                    )

            with self.assertRaises(RuntimeError):
                require_active_simulation(
                    generation="v2",
                    hand="left",
                    runtime_dir=runtime_dir,
                )


class HardwareCommandSafetyTest(unittest.TestCase):
    def test_realtime_filter_holds_noise_and_tracks_fast_motion_in_one_tick(
        self,
    ) -> None:
        angle_filter = AdaptiveTargetFilter(
            initial_positions=(0.0,) * 20,
            deadband_rad=math.radians(0.5),
            min_cutoff_hz=2.0,
            max_cutoff_hz=20.0,
            velocity_for_max_cutoff=1.0,
            initial_time=0.0,
        )

        still = angle_filter.step((0.005,) * 20, now=0.02)
        moving = angle_filter.step((1.0,) * 20, now=0.04)

        self.assertEqual(still, (0.0,) * 20)
        self.assertTrue(all(value > 0.9 for value in moving))

    def test_realtime_limiter_uses_velocity_without_acceleration_ramp(
        self,
    ) -> None:
        limiter = RealtimeVelocityLimiter(
            initial_positions=(0.0,) * 20,
            lower=(-1.0,) * 20,
            upper=(1.0,) * 20,
            max_velocity=4.0,
            initial_time=0.0,
        )

        first = limiter.step((1.0,) * 20, now=0.02)
        second = limiter.step((-1.0,) * 20, now=0.04)

        self.assertEqual(first, (0.08,) * 20)
        self.assertEqual(second, (0.0,) * 20)

    def test_angle_low_pass_filter_smooths_hardware_input(self) -> None:
        angle_filter = AngleLowPassFilter(
            initial_positions=(0.0,) * 20,
            cutoff_hz=5.0,
            initial_time=0.0,
        )

        first = angle_filter.step((1.0,) * 20, now=0.01)
        second = angle_filter.step((1.0,) * 20, now=0.02)

        self.assertTrue(all(0.0 < value < 1.0 for value in first))
        self.assertTrue(
            all(before < after < 1.0 for before, after in zip(first, second))
        )

    def test_v2_hardware_limits_match_official_mjcf_actuators(self) -> None:
        for hand in ("left", "right"):
            resources = get_hand_resources(hand, generation="v2")
            lower, upper = load_v2_firmware_limits(
                model_path=resources.mjcf,
                config_path=resources.mapping,
                hand=hand,
            )
            model, _ = load_model(resources.mjcf)
            by_dof = {
                binding.dof_name: binding
                for binding in build_control_bindings(
                    model,
                    hand=hand,
                    config_path=resources.mapping,
                )
            }

            self.assertEqual(
                lower,
                tuple(by_dof[name].lower for name in FIRMWARE_DOF_ORDER),
            )
            self.assertEqual(
                upper,
                tuple(by_dof[name].upper for name in FIRMWARE_DOF_ORDER),
            )

    def test_v2_right_target_uses_firmware_order_and_generation_mapping(
        self,
    ) -> None:
        semantic = {name: 0.0 for name in DOF_ORDER}
        semantic.update(
            {
                "index1_abd": 0.1,
                "index1_flex": 0.2,
                "thumb2_flex": 0.3,
                "thumb3_flex": 0.4,
            }
        )
        target = HandTarget(
            hand="right",
            values=tuple(semantic[name] for name in DOF_ORDER),
            source="test",
            sequence=1,
            timestamp_ns=1,
            dropped=0,
        )
        mapper = FirmwareCommandMapper.from_config(
            PROJECT_ROOT / "config/hands/wuji_v2/right_mapping.json",
            generation="v2",
            hand="right",
            lower=(-1.0,) * 20,
            upper=(1.0,) * 20,
        )

        command = mapper.map_target(target)

        self.assertEqual(
            command[:8],
            (0.0, 0.0, -0.3, -0.4, 0.2, 0.1, 0.0, 0.0),
        )

    def test_motion_limiter_enforces_velocity_acceleration_and_limits(
        self,
    ) -> None:
        limiter = MotionLimiter(
            initial_positions=(0.0,) * 20,
            lower=(-1.0,) * 20,
            upper=(1.0,) * 20,
            max_velocity=1.0,
            max_acceleration=2.0,
            initial_time=0.0,
        )

        first = limiter.step((2.0,) * 20, now=0.5)
        second = limiter.step((2.0,) * 20, now=1.0)

        self.assertEqual(first, (0.25,) * 20)
        self.assertEqual(second, (0.75,) * 20)

    def test_motion_limiter_rejects_actual_position_outside_limits(
        self,
    ) -> None:
        with self.assertRaisesRegex(ValueError, "初始关节位置超出"):
            MotionLimiter(
                initial_positions=(2.0,) + (0.0,) * 19,
                lower=(-1.0,) * 20,
                upper=(1.0,) * 20,
                max_velocity=1.0,
                max_acceleration=2.0,
                initial_time=0.0,
            )


class WujiSdkDriverTest(unittest.TestCase):
    def test_v2_mit_gain_boundary_rejects_unsafe_values(self) -> None:
        self.assertEqual(validate_v2_mit_gains(8.0, 0.2), (8.0, 0.2))
        for kp, kd in ((0.0, 0.2), (10.1, 0.2), (8.0, 0.0), (8.0, 1.1)):
            with self.subTest(kp=kp, kd=kd):
                with self.assertRaisesRegex(ValueError, "Kp|Kd"):
                    validate_v2_mit_gains(kp, kd)

    def test_v1_clears_faults_before_enable_and_disables_on_close(self) -> None:
        events: list[object] = []

        class FakeController:
            def get_actual_position(self):
                events.append("read_position")
                return [0.2] * 20

        class FakeControllerContext:
            def __enter__(self):
                events.append("controller_enter")
                return FakeController()

            def __exit__(self, *_args):
                events.append("controller_exit")

        class FakeHand:
            serial_number = "V1-LEFT"

            def handedness_name(self):
                return "Left"

            def clear_all_faults(self):
                events.append("clear_faults")

            def get_soft_limits(self):
                return [1.0] * 20, [-1.0] * 20

            def set_all_effort_limit(self, amps):
                events.append(("effort_limit", amps))

            def enable(self):
                events.append("enable")

            def realtime_controller(self, lowpass):
                events.append(("lowpass", lowpass.cutoff_hz))
                return FakeControllerContext()

            def joint_command(self):
                return SimpleNamespace(publish=lambda: FakePublisher())

            def disable(self):
                events.append("disable")

        class FakePublisher:
            def send(self, commands):
                events.append(
                    (
                        "send",
                        tuple(command.position for command in commands),
                    )
                )

            def close(self):
                events.append("publisher_close")

        class FakeManager:
            def scan(self):
                return [
                    SimpleNamespace(
                        sn="V1-LEFT",
                        device_type="v1",
                        address="usb:test",
                    )
                ]

            def connect(self, *, sn, device_name):
                events.append(("connect", sn, device_name))
                return FakeHand()

            def disconnect(self, device_name):
                events.append(("disconnect", device_name))

        class FakeLowPass:
            def __init__(self, cutoff_hz):
                self.cutoff_hz = cutoff_hz

        class FakeJointCommand:
            def __init__(self, position, velocity, effort):
                self.position = position
                self.velocity = velocity
                self.effort = effort

        fake_sdk = SimpleNamespace(
            DeviceType=SimpleNamespace(WujiHand="v1", WujiHand2="v2"),
            WujiHand=FakeHand,
            WujiHand2=type("FakeHand2", (), {}),
            LowPass=FakeLowPass,
            JointCommand=FakeJointCommand,
        )
        driver = open_wuji_sdk_driver(
            generation="v1",
            hand="left",
            serial="V1-LEFT",
            manager=FakeManager(),
            sdk_module=fake_sdk,
        )

        driver.prepare()
        self.assertEqual(driver.read_positions(), (0.2,) * 20)
        driver.send_positions((0.0,) * 20)
        driver.close()

        self.assertLess(events.index("clear_faults"), events.index("enable"))
        self.assertLess(events.index("enable"), events.index("controller_enter"))
        self.assertEqual(events[-4:], ["publisher_close", "controller_exit", "disable", (
            "disconnect",
            "real_v1_left",
        )])

    def test_v2_waits_for_all_joints_enabled_before_publishing(self) -> None:
        events: list[object] = []

        class Resource:
            def __init__(self, name, value=None):
                self.name = name
                self.value = value

            def get(self):
                events.append((self.name, "get"))
                return self.value

            def set(self, value):
                events.append((self.name, "set", value))

            def subscribe(self):
                if self.name == "joint_states":
                    raise AssertionError(
                        "joint_states 必须通过回调持续消费并只保留最新帧"
                    )
                events.append((self.name, "subscribe"))
                return Subscription(self.name)

            def subscribe_with_callback(self, callback):
                events.append((self.name, "subscribe_with_callback"))
                subscription = Subscription(self.name)
                nids = [
                    bus_id * 5 + node_id
                    for bus_id in range(5)
                    for node_id in range(1, 5)
                ]
                for generation in range(3):
                    callback(
                        SimpleNamespace(
                            joints=[
                                SimpleNamespace(
                                    nid=nid,
                                    position=float(generation * 100 + index),
                                )
                                for index, nid in reversed(
                                    list(enumerate(nids))
                                )
                            ]
                        )
                    )
                return subscription

            def publish(self):
                events.append((self.name, "publish"))
                return Publisher()

        class Subscription:
            def __init__(self, name):
                self.name = name

            def recv(self):
                # Feedback nid is the one-based bus node encoding; it is not
                # WujiHand2.joint_id_from_bus_node()'s zero-based joint_id.
                nids = [
                    bus_id * 5 + node_id
                    for bus_id in range(5)
                    for node_id in range(1, 5)
                ]
                if self.name == "diagnostics":
                    status = SimpleNamespace(ext_state=2)
                    joints = [
                        SimpleNamespace(nid=nid, status_word=status)
                        for nid in reversed(nids)
                    ]
                else:
                    joints = [
                        SimpleNamespace(nid=nid, position=float(index))
                        for index, nid in reversed(list(enumerate(nids)))
                    ]
                return SimpleNamespace(joints=joints)

            def close(self):
                events.append((self.name, "close"))

        class Publisher:
            def send(self, commands):
                events.append(
                    (
                        "send",
                        tuple(command.position for command in commands),
                    )
                )

            def close(self):
                events.append("publisher_close")

        class FakeHand2:
            serial_number = "V2-RIGHT"

            def handedness(self):
                return Resource("handedness", "right")

            def clear_fault(self):
                events.append("clear_faults")

            def online_joints_count(self):
                return Resource("online_count", 20)

            def effort_limit(self):
                return Resource("effort_limit")

            def mit_params(self):
                return Resource("mit_params")

            def enable(self):
                events.append("enable")

            def joint_diagnostics(self):
                return Resource("diagnostics")

            def joint_states(self):
                return Resource("joint_states")

            def joint_command(self):
                return Resource("joint_command")

            def disable(self):
                events.append("disable")

            def emergency_stop(self):
                events.append("emergency_stop")

            @staticmethod
            def joint_id_from_bus_node(bus_id, node_id):
                return bus_id * 5 + node_id - 1

        class FakeManager:
            def scan(self):
                return [
                    SimpleNamespace(
                        sn="V2-RIGHT",
                        device_type="v2",
                        address="udp:test",
                    )
                ]

            def connect(self, *, sn, device_name, options):
                events.append(("connect", sn, device_name))
                events.append(("enable_bridge", options.enable_bridge))
                return FakeHand2()

            def disconnect(self, device_name):
                events.append(("disconnect", device_name))

        class FakeJointCommand:
            def __init__(self, position, velocity, effort):
                self.position = position
                self.velocity = velocity
                self.effort = effort

        class FakeConnectOptions:
            def __init__(self, *, enable_bridge):
                self.enable_bridge = enable_bridge

        class FakeHandedness:
            def __init__(self, value):
                self.value = value

            def __str__(self):
                return self.value

        fake_sdk = SimpleNamespace(
            DeviceType=SimpleNamespace(WujiHand="v1", WujiHand2="v2"),
            Handedness=SimpleNamespace(
                Left=FakeHandedness("left"),
                Right=FakeHandedness("right"),
            ),
            WujiHand=type("FakeHand1", (), {}),
            WujiHand2=FakeHand2,
            JointCommand=FakeJointCommand,
            ConnectOptions=FakeConnectOptions,
        )
        driver = open_wuji_sdk_driver(
            generation="v2",
            hand="right",
            serial="V2-RIGHT",
            manager=FakeManager(),
            sdk_module=fake_sdk,
            lower=(-1.0,) * 20,
            upper=(1.0,) * 20,
        )

        driver.prepare()
        self.assertEqual(
            driver.read_positions(),
            tuple(float(200 + index) for index in range(20)),
        )
        self.assertEqual(
            driver.read_positions(),
            tuple(float(200 + index) for index in range(20)),
        )
        self.assertEqual(
            events.count(("joint_states", "subscribe_with_callback")),
            1,
            "状态流必须只建立一个持续消费的最新帧订阅",
        )
        driver.send_positions((0.1,) * 20)
        driver.close()

        self.assertLess(events.index("clear_faults"), events.index("enable"))
        self.assertIn(("mit_params", "set", (8.0, 0.2)), events)
        self.assertIn(("enable_bridge", False), events)
        self.assertLess(
            events.index("enable"),
            events.index(("joint_command", "publish")),
        )
        self.assertIn(("send", (0.1,) * 20), events)
        self.assertEqual(events[-4:], [
            "publisher_close",
            ("joint_states", "close"),
            "disable",
            ("disconnect", "real_v2_right"),
        ])


class SafeHardwarePublisherTest(unittest.TestCase):
    def test_realtime_tracking_uses_fast_online_path_but_safe_home_path(
        self,
    ) -> None:
        sent: list[tuple[float, ...]] = []

        class FakeClock:
            value = 0.0

            def __call__(self):
                return self.value

            def advance(self, seconds):
                self.value += seconds

        class FakeDriver:
            generation = "v2"
            hand = "left"
            serial = "V2-LEFT"
            lower = (-1.0,) * 20
            upper = (1.0,) * 20

            def prepare(self):
                pass

            def read_positions(self):
                return (0.0,) * 20

            def send_positions(self, positions):
                sent.append(tuple(positions))

            def emergency_stop(self):
                pass

            def close(self):
                pass

        clock = FakeClock()
        mapper = FirmwareCommandMapper.from_config(
            PROJECT_ROOT / "config/hands/wuji_v2/left_mapping.json",
            generation="v2",
            hand="left",
            lower=(-1.0,) * 20,
            upper=(1.0,) * 20,
        )
        publisher = SafeHardwarePublisher.open(
            driver=FakeDriver(),
            mapper=mapper,
            simulation_check=lambda: None,
            options=HardwareSafetyOptions(
                max_velocity=1.0,
                max_acceleration=2.0,
                tracking_mode="realtime",
                realtime_max_velocity=4.0,
            ),
            clock=clock,
            sleep=clock.advance,
            start_watchdog=False,
            arm_command_timeout=False,
        )

        clock.advance(0.02)
        sent.clear()
        publisher.publish(
            HandTarget(
                hand="left",
                values=(0.5,) * 20,
                source="test",
                sequence=1,
                timestamp_ns=1,
                dropped=0,
            )
        )

        self.assertGreater(max(abs(value) for value in sent[-1]), 0.07)
        self.assertLessEqual(max(abs(value) for value in sent[-1]), 0.08)
        publisher.close()

    def test_delayed_arm_resets_filter_and_limiter_timebase(self) -> None:
        sent: list[tuple[float, ...]] = []

        class FakeClock:
            value = 0.0

            def __call__(self):
                return self.value

            def advance(self, seconds):
                self.value += seconds

        class FakeDriver:
            generation = "v2"
            hand = "left"
            serial = "V2-LEFT"
            lower = (-1.0,) * 20
            upper = (1.0,) * 20

            def prepare(self):
                pass

            def read_positions(self):
                return (0.0,) * 20

            def send_positions(self, positions):
                sent.append(tuple(positions))

            def emergency_stop(self):
                pass

            def close(self):
                pass

        clock = FakeClock()
        mapper = FirmwareCommandMapper.from_config(
            PROJECT_ROOT / "config/hands/wuji_v2/left_mapping.json",
            generation="v2",
            hand="left",
            lower=(-1.0,) * 20,
            upper=(1.0,) * 20,
        )
        publisher = SafeHardwarePublisher.open(
            driver=FakeDriver(),
            mapper=mapper,
            simulation_check=lambda: None,
            options=HardwareSafetyOptions(
                max_velocity=1.0,
                max_acceleration=2.0,
            ),
            clock=clock,
            sleep=clock.advance,
            start_watchdog=False,
            arm_command_timeout=False,
        )

        clock.advance(5.0)
        publisher.arm_command_timeout()
        clock.advance(0.01)
        sent.clear()
        publisher.publish(
            HandTarget(
                hand="left",
                values=(0.5,) * 20,
                source="test",
                sequence=1,
                timestamp_ns=1,
                dropped=0,
            )
        )

        self.assertLessEqual(max(abs(value) for value in sent[-1]), 0.001)
        publisher.close()

    def test_dual_startup_can_delay_first_frame_timeout_until_armed(
        self,
    ) -> None:
        closed = threading.Event()

        class FakeDriver:
            generation = "v2"
            hand = "left"
            serial = "V2-LEFT"
            lower = (-1.0,) * 20
            upper = (1.0,) * 20

            def prepare(self):
                pass

            def read_positions(self):
                return (0.0,) * 20

            def send_positions(self, _positions):
                pass

            def emergency_stop(self):
                pass

            def close(self):
                closed.set()

        mapper = FirmwareCommandMapper.from_config(
            PROJECT_ROOT / "config/hands/wuji_v2/left_mapping.json",
            generation="v2",
            hand="left",
            lower=(-1.0,) * 20,
            upper=(1.0,) * 20,
        )
        publisher = SafeHardwarePublisher.open(
            driver=FakeDriver(),
            mapper=mapper,
            simulation_check=lambda: None,
            options=HardwareSafetyOptions(
                command_timeout=0.02,
                watchdog_period=0.005,
            ),
            arm_command_timeout=False,
        )

        self.assertFalse(
            closed.wait(0.05),
            "双手另一侧初始化期间不应触发首帧超时",
        )
        publisher.arm_command_timeout()
        self.assertTrue(
            closed.wait(1.0),
            "双手全部就绪后应启动首帧超时",
        )

    def test_interrupt_during_startup_stops_and_closes_driver(self) -> None:
        events: list[str] = []

        class FakeDriver:
            generation = "v1"
            hand = "left"
            serial = "V1-LEFT"
            lower = (-1.0,) * 20
            upper = (1.0,) * 20

            def prepare(self):
                events.append("prepare")

            def read_positions(self):
                raise KeyboardInterrupt

            def send_positions(self, _positions):
                events.append("send")

            def emergency_stop(self):
                events.append("emergency_stop")

            def close(self):
                events.append("close")

        mapper = FirmwareCommandMapper.from_config(
            PROJECT_ROOT / "config/hands/wuji_v1/left_mapping.json",
            generation="v1",
            hand="left",
            lower=(-1.0,) * 20,
            upper=(1.0,) * 20,
        )
        with self.assertRaises(KeyboardInterrupt):
            SafeHardwarePublisher.open(
                driver=FakeDriver(),
                mapper=mapper,
                simulation_check=lambda: None,
                start_watchdog=False,
            )

        self.assertEqual(
            events,
            ["prepare", "emergency_stop", "close"],
        )

    def test_watchdog_timeout_returns_home_before_closing_driver(self) -> None:
        events: list[object] = []
        closed = threading.Event()

        class FakeDriver:
            generation = "v1"
            hand = "left"
            serial = "V1-LEFT"
            lower = (-1.0,) * 20
            upper = (1.0,) * 20

            def prepare(self):
                events.append("prepare")

            def read_positions(self):
                return (0.0,) * 20

            def send_positions(self, positions):
                events.append(("send", tuple(positions)))

            def emergency_stop(self):
                events.append("emergency_stop")

            def close(self):
                events.append("close")
                closed.set()

        mapper = FirmwareCommandMapper.from_config(
            PROJECT_ROOT / "config/hands/wuji_v1/left_mapping.json",
            generation="v1",
            hand="left",
            lower=(-1.0,) * 20,
            upper=(1.0,) * 20,
        )
        publisher = SafeHardwarePublisher.open(
            driver=FakeDriver(),
            mapper=mapper,
            simulation_check=lambda: None,
            options=HardwareSafetyOptions(
                max_velocity=10.0,
                max_acceleration=100.0,
                home_rate_hz=1000.0,
                home_tolerance=1e-12,
                home_timeout=0.5,
                command_timeout=0.02,
                watchdog_period=0.005,
            ),
        )

        self.assertTrue(
            closed.wait(1.0),
            "看门狗没有在等待首帧超时后关闭驱动",
        )
        close_index = events.index("close")
        self.assertEqual(events[close_index - 1], ("send", (0.0,) * 20))
        self.assertNotIn("emergency_stop", events)
        with self.assertRaisesRegex(RuntimeError, "手套目标发布超时"):
            publisher.publish(
                HandTarget(
                    hand="left",
                    values=(0.0,) * 20,
                    source="test",
                    sequence=6,
                    timestamp_ns=6,
                    dropped=0,
                )
            )

    def test_startup_and_shutdown_return_home_around_teleoperation(
        self,
    ) -> None:
        events: list[object] = []
        actual_positions = [0.2] * 20

        class FakeClock:
            value = 0.0

            def __call__(self):
                return self.value

            def sleep(self, seconds):
                self.value += seconds

        class FakeDriver:
            generation = "v1"
            hand = "left"
            serial = "V1-LEFT"
            lower = (-1.0,) * 20
            upper = (1.0,) * 20

            def prepare(self):
                events.append("prepare")

            def read_positions(self):
                events.append("read_positions")
                return tuple(actual_positions)

            def send_positions(self, positions):
                command = tuple(positions)
                events.append(("send", command))
                actual_positions[:] = command

            def emergency_stop(self):
                events.append("emergency_stop")

            def close(self):
                events.append("close")

        clock = FakeClock()
        mapper = FirmwareCommandMapper.from_config(
            PROJECT_ROOT / "config/hands/wuji_v1/left_mapping.json",
            generation="v1",
            hand="left",
            lower=(-1.0,) * 20,
            upper=(1.0,) * 20,
        )
        publisher = SafeHardwarePublisher.open(
            driver=FakeDriver(),
            mapper=mapper,
            simulation_check=lambda: events.append("simulation_check"),
            options=HardwareSafetyOptions(
                max_velocity=10.0,
                max_acceleration=100.0,
                home_rate_hz=10.0,
                home_tolerance=1e-6,
                home_timeout=2.0,
                command_timeout=1.0,
                watchdog_period=0.1,
            ),
            clock=clock,
            sleep=clock.sleep,
            start_watchdog=False,
        )
        sends_after_open = [
            event for event in events if isinstance(event, tuple)
        ]
        self.assertEqual(sends_after_open[-1], ("send", (0.0,) * 20))
        self.assertGreaterEqual(events.count("read_positions"), 2)

        clock.sleep(0.1)
        target = HandTarget(
            hand="left",
            values=(0.5,) * 20,
            source="test",
            sequence=2,
            timestamp_ns=2,
            dropped=0,
        )
        publisher.publish(target)
        self.assertNotEqual(events[-1], ("send", (0.0,) * 20))

        publisher.close()

        final_sends = [
            event for event in events if isinstance(event, tuple)
        ]
        self.assertEqual(final_sends[-1], ("send", (0.0,) * 20))
        self.assertEqual(events[-1], "close")
        self.assertLess(events.index("prepare"), events.index(sends_after_open[0]))

    def test_lost_simulator_returns_home_before_closing_driver(self) -> None:
        events: list[object] = []

        class FakeClock:
            value = 0.0

            def __call__(self):
                return self.value

            def sleep(self, seconds):
                self.value += seconds

        class FakeDriver:
            generation = "v2"
            hand = "right"
            serial = "V2-RIGHT"
            lower = (-1.0,) * 20
            upper = (1.0,) * 20

            def prepare(self):
                events.append("prepare")

            def read_positions(self):
                return (0.0,) * 20

            def send_positions(self, positions):
                events.append(("send", tuple(positions)))

            def emergency_stop(self):
                events.append("emergency_stop")

            def close(self):
                events.append("close")

        simulation_active = True

        def simulation_check():
            if not simulation_active:
                raise RuntimeError("simulator stopped")

        clock = FakeClock()
        mapper = FirmwareCommandMapper.from_config(
            PROJECT_ROOT / "config/hands/wuji_v2/right_mapping.json",
            generation="v2",
            hand="right",
            lower=(-1.0,) * 20,
            upper=(1.0,) * 20,
        )
        publisher = SafeHardwarePublisher.open(
            driver=FakeDriver(),
            mapper=mapper,
            simulation_check=simulation_check,
            options=HardwareSafetyOptions(
                max_velocity=10.0,
                max_acceleration=100.0,
                home_rate_hz=10.0,
                home_tolerance=1e-6,
                home_timeout=2.0,
                command_timeout=1.0,
                watchdog_period=0.1,
            ),
            clock=clock,
            sleep=clock.sleep,
            start_watchdog=False,
        )
        clock.sleep(0.1)
        publisher.publish(
            HandTarget(
                hand="right",
                values=(0.5,) * 20,
                source="test",
                sequence=3,
                timestamp_ns=3,
                dropped=0,
            )
        )

        simulation_active = False
        with self.assertRaisesRegex(
            RuntimeError,
            "仿真守卫失效",
        ):
            publisher.publish(
                HandTarget(
                    hand="right",
                    values=(0.5,) * 20,
                    source="test",
                    sequence=4,
                    timestamp_ns=4,
                    dropped=0,
                )
            )

        self.assertEqual(events[-2:], [("send", (0.0,) * 20), "close"])
        self.assertNotIn("emergency_stop", events)


if __name__ == "__main__":
    unittest.main()
