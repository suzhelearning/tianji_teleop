"""独立真机命令行装配与启动门禁测试。"""

from __future__ import annotations

import unittest
import signal
import threading
import time
from types import SimpleNamespace
from unittest.mock import patch

from data_glove_wuji_teleop.adapters.hardware.safe_publisher import (
    HardwareSafetyOptions,
)
from data_glove_wuji_teleop.domain.hand_target import HandTarget
from data_glove_wuji_teleop.hardware_cli import (
    RealSideSettings,
    _interrupt_on_shutdown_signals,
    _run_publishers,
    _run_target_side,
    build_parser,
    command_real,
    open_real_publisher,
)


class RealControlCliTest(unittest.TestCase):
    def test_rejects_out_of_range_v2_mit_gains(self) -> None:
        cases = (
            ("--kp", "0"),
            ("--kp", "10.1"),
            ("--kd", "0"),
            ("--kd", "1.1"),
        )
        for flag, value in cases:
            with self.subTest(flag=flag, value=value):
                args = build_parser().parse_args(
                    [
                        "--generation",
                        "v2",
                        "--hand",
                        "right",
                        "--hand-sn",
                        "V2-R",
                        flag,
                        value,
                        "--confirm-real",
                    ]
                )
                with self.assertRaisesRegex(ValueError, "Kp|Kd|kp|kd"):
                    command_real(args)

    def test_realtime_resamples_20hz_source_close_to_50hz(self) -> None:
        stop = threading.Event()
        target = HandTarget(
            hand="right",
            values=(0.1,) * 20,
            source="official_wuji_retarget",
            sequence=1,
            timestamp_ns=2,
            dropped=0,
        )
        started = time.monotonic()

        class SourceAt20Hz:
            next_frame_at = started

            def recv(self):
                now = time.monotonic()
                if now - started >= 0.50:
                    stop.set()
                    return None
                if now >= self.next_frame_at:
                    self.next_frame_at += 0.05
                    return target
                time.sleep(0.001)
                return None

        class CountingPublisher:
            sent = 0

            def publish(self, _target):
                self.sent += 1

        publisher = CountingPublisher()

        _run_target_side(
            SimpleNamespace(
                generation="v2",
                tracking_mode="realtime",
                rate=50.0,
                duration=0.0,
                command_timeout=0.5,
            ),
            RealSideSettings(
                hand="right",
                serial="V2-R",
                target_host="127.0.0.1",
                target_port=15558,
            ),
            SourceAt20Hz(),
            publisher,
            stop_event=stop,
        )

        self.assertGreaterEqual(publisher.sent, 22)

    def test_zmq_target_mode_does_not_load_legacy_glove_calibration(
        self,
    ) -> None:
        args = build_parser().parse_args(
            [
                "--generation",
                "v2",
                "--hand",
                "right",
                "--hand-sn",
                "V2-R",
                "--confirm-real",
            ]
        )
        with (
            patch("data_glove_wuji_teleop.hardware_cli._simulation_check"),
            patch(
                "data_glove_wuji_teleop.hardware_cli._run_publishers",
                return_value=0,
            ),
        ):
            self.assertEqual(command_real(args), 0)

    def test_realtime_mode_republishes_latest_fresh_target_at_fixed_rate(
        self,
    ) -> None:
        stop = threading.Event()
        target = HandTarget(
            hand="left",
            values=(0.1,) * 20,
            source="dataglove",
            sequence=7,
            timestamp_ns=2,
            dropped=0,
        )
        started = time.monotonic()

        class FakeReceiver:
            first = True

            def recv(self):
                if self.first:
                    self.first = False
                    return target
                time.sleep(0.002)
                if time.monotonic() - started >= 0.055:
                    stop.set()
                return None

        class FakePublisher:
            def __init__(self):
                self.targets = []

            def publish(self, received):
                self.targets.append(received)

        publisher = FakePublisher()
        result = _run_target_side(
            SimpleNamespace(
                generation="v2",
                tracking_mode="realtime",
                rate=50.0,
                duration=0.0,
                command_timeout=0.5,
            ),
            RealSideSettings(
                hand="left",
                serial="V2-L",
                target_host="127.0.0.1",
                target_port=15559,
            ),
            FakeReceiver(),
            publisher,
            stop_event=stop,
        )

        self.assertEqual(result, 0)
        self.assertGreaterEqual(len(publisher.targets), 2)
        self.assertTrue(all(received is target for received in publisher.targets))

    def test_shared_target_is_first_input_published_to_hardware(self) -> None:
        stop = threading.Event()
        target = HandTarget(
            hand="left",
            values=(0.1,) * 20,
            source="dataglove",
            sequence=1,
            timestamp_ns=2,
            dropped=0,
        )

        class FakeReceiver:
            def recv(self):
                return target

        class FakePublisher:
            def __init__(self):
                self.targets = []

            def publish(self, received):
                self.targets.append(received)
                stop.set()

        publisher = FakePublisher()
        result = _run_target_side(
            SimpleNamespace(
                generation="v2",
                rate=0.0,
                duration=0.0,
                command_timeout=0.5,
            ),
            RealSideSettings(
                hand="left",
                serial="V2-L",
                target_host="127.0.0.1",
                target_port=15559,
            ),
            FakeReceiver(),
            publisher,
            stop_event=stop,
        )

        self.assertEqual(result, 0)
        self.assertEqual(publisher.targets, [target])

    def test_single_hand_delays_timeout_until_first_shared_target(self) -> None:
        events = []

        class FakeReceiver:
            def close(self):
                pass

        class FakePublisher:
            def close(self):
                pass

        def open_receivers(_settings):
            events.append("target_preflight")
            return {"left": FakeReceiver()}

        def open_publisher(**_kwargs):
            events.append("hardware_open")
            return FakePublisher()

        settings = (
            RealSideSettings(
                hand="left",
                serial="V2-L",
                target_host="127.0.0.1",
                target_port=15559,
            ),
        )
        with (
            patch(
                "data_glove_wuji_teleop.hardware_cli.open_real_publisher",
                side_effect=open_publisher,
            ) as open_publisher,
            patch(
                "data_glove_wuji_teleop.hardware_cli._open_target_receivers",
                side_effect=open_receivers,
            ),
            patch(
                "data_glove_wuji_teleop.hardware_cli._run_target_side",
                return_value=0,
            ),
        ):
            self.assertEqual(
                _run_publishers(
                    SimpleNamespace(generation="v2"),
                    settings,
                    HardwareSafetyOptions(),
                ),
                0,
            )

        self.assertFalse(
            open_publisher.call_args.kwargs["arm_command_timeout"],
            "手套第一帧到达前不能启动命令超时看门狗",
        )
        self.assertEqual(open_publisher.call_args.kwargs["kp"], 8.0)
        self.assertEqual(open_publisher.call_args.kwargs["kd"], 0.2)
        self.assertEqual(events, ["target_preflight", "hardware_open"])

    def test_both_mode_closes_left_even_if_right_close_fails(self) -> None:
        events: list[str] = []

        class FakeReceiver:
            def __init__(self, hand):
                self.hand = hand

            def close(self):
                events.append(f"receiver_close:{self.hand}")

        class FakePublisher:
            def __init__(self, hand, *, fail_close=False):
                self.hand = hand
                self.fail_close = fail_close

            def close(self):
                events.append(f"close:{self.hand}")
                if self.fail_close:
                    raise RuntimeError(f"{self.hand} close failed")

        settings = tuple(
            RealSideSettings(
                hand=hand,
                serial=f"SN-{hand}",
                target_host="127.0.0.1",
                target_port=(15559 if hand == "left" else 15558),
            )
            for hand in ("left", "right")
        )
        publishers = [
            FakePublisher("left"),
            FakePublisher("right", fail_close=True),
        ]
        with (
            patch(
                "data_glove_wuji_teleop.hardware_cli._open_target_receivers",
                return_value={
                    hand: FakeReceiver(hand)
                    for hand in ("left", "right")
                },
            ),
            patch(
                "data_glove_wuji_teleop.hardware_cli.open_real_publisher",
                side_effect=publishers,
            ),
            patch(
                "data_glove_wuji_teleop.hardware_cli._run_both",
                return_value=0,
            ),
        ):
            with self.assertRaisesRegex(RuntimeError, "right close failed"):
                _run_publishers(
                    SimpleNamespace(generation="v2"),
                    settings,
                    HardwareSafetyOptions(),
                )

        self.assertEqual(
            events,
            [
                "close:right",
                "close:left",
                "receiver_close:right",
                "receiver_close:left",
            ],
        )

    def test_shutdown_signals_interrupt_initialization_and_are_restored(
        self,
    ) -> None:
        installed: dict[signal.Signals, object] = {}

        def install(signum, handler):
            installed[signum] = handler

        with (
            patch("signal.getsignal", return_value="previous"),
            patch("signal.signal", side_effect=install) as signal_call,
        ):
            with self.assertRaises(KeyboardInterrupt):
                with _interrupt_on_shutdown_signals():
                    handler = installed[signal.SIGTERM]
                    handler(signal.SIGTERM, None)

        self.assertGreaterEqual(signal_call.call_count, 6)
        self.assertEqual(installed[signal.SIGTERM], "previous")




    def test_simulator_is_checked_before_sdk_device_is_opened(self) -> None:
        with (
            patch(
                "data_glove_wuji_teleop.hardware_cli.require_active_simulation",
                side_effect=RuntimeError("simulator missing"),
            ),
            patch(
                "data_glove_wuji_teleop.hardware_cli.open_wuji_sdk_driver",
            ) as sdk_open,
        ):
            with self.assertRaisesRegex(RuntimeError, "simulator missing"):
                open_real_publisher(
                    generation="v1",
                    hand="left",
                    serial="V1-L",
                    safety=HardwareSafetyOptions(),
                )

        sdk_open.assert_not_called()


if __name__ == "__main__":
    unittest.main()
