"""发送入口的离线授权、单侧隔离、新测量与资源关闭合同；不连接真实设备。"""

from __future__ import annotations

import io
import json
import socket
from contextlib import redirect_stderr
import threading
import time
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch

import mujoco
import numpy as np
from tianji import hand_protocol

from data_glove_wuji_teleop.application import tianji_stream as stream
from data_glove_wuji_teleop.adapters.glove.encoder_stream import EncoderFrame
from data_glove_wuji_teleop.adapters.transport.tianji_udp import TianjiHandSender
from data_glove_wuji_teleop.profiles.wuji_v2.model import full_joint_name
from data_glove_wuji_teleop.project import get_hand_resources
from tests.integration.test_official_dual_launcher import _synthetic_profile
from tests.unit.test_tianji_udp import decode_packet


class _Pipeline:
    zero_profile = object()

    def __init__(self):
        self.frames = []

    def validate_stream(self, _connection):
        pass

    def process_frame(self, frame, *, zero_profile):
        if zero_profile is not self.zero_profile:
            raise ValueError("没有使用所选内嵌零位")
        self.frames.append(frame.sequence)
        return frame.sequence


class _Retargeter:
    def __init__(self, hand, model, *, fail=False):
        self.names = tuple(full_joint_name(hand, name) for name in reversed(hand_protocol.JOINT_STEMS))
        self.limits = [model.joint(name).range for name in self.names]
        self.closed = False
        self.fail = fail

    def retarget(self, sequence, *, apply_filter):
        if self.fail:
            raise RuntimeError("测试官方求解失败")
        # 非零且在模型限位内，防止把退出补零或原始帧误当成有效机器人目标。
        qpos = np.asarray([low + (high - low) * (0.3 + sequence * 0.05) for low, high in self.limits])
        return SimpleNamespace(joint_names=self.names, qpos=qpos)

    def close(self):
        self.closed = True


class _Connection:
    def __init__(self, frames=(), *, on_exhaust=None, on_close=None):
        self.frames = iter(frames)
        self.on_exhaust = on_exhaust
        self.on_close = on_close
        self.closed = False

    def read_latest_frame(self):
        try:
            item = next(self.frames)
        except StopIteration:
            if self.on_exhaust is not None:
                self.on_exhaust()
            raise EOFError("测试采集结束") from None
        if isinstance(item, Exception):
            raise item
        return item

    def close(self):
        self.closed = True
        if self.on_close is not None:
            self.on_close()


def _frame(sequence, timestamp=None):
    return EncoderFrame(sequence, sequence * 100 if timestamp is None else timestamp, 0, [42.0] * 21)


class TianjiStreamTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.models = {
            hand: mujoco.MjModel.from_xml_path(str(get_hand_resources(hand, generation="v2").mjcf))
            for hand in ("left", "right")
        }

    def config(self, hand="right", **options):
        args = stream.build_parser().parse_args(["--confirm-send"])
        args.hand = hand
        args.timeout = 0.1
        args.max_frame_age = 1.0
        for key, value in options.items():
            setattr(args, key, value)
        return stream._HandConfig(SimpleNamespace(hand=hand), args, _Pipeline(), self.models[hand])

    def record_sender(self, on_send=None):
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(receiver.close)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(0.05)
        sender = TianjiHandSender("127.0.0.1", receiver.getsockname()[1])
        self.addCleanup(sender.close)
        packets = []
        send = sender.send

        def record(**updates):
            sequence = send(**updates)
            if sequence is not None:
                packet = decode_packet(receiver.recvfrom(8192)[0])
                packets.append(packet)
                if on_send is not None:
                    on_send(packet)
            return sequence

        recording = patch.object(sender, "send", side_effect=record)
        recording.start()
        self.addCleanup(recording.stop)
        return sender, packets, receiver

    def test_no_confirmation_never_opens_network_or_processes(self):
        with patch("socket.socket", side_effect=AssertionError("不允许打开网络")), \
                patch("subprocess.Popen", side_effect=AssertionError("不允许启动进程")):
            status = stream.run(stream.build_parser().parse_args([]))
        self.assertEqual(status, 2)

    def test_remote_destination_rejected_before_discovery_or_worker(self):
        for host in ("192.168.1.5", "0.0.0.0", "::1", "localhost"):
            args = stream.build_parser().parse_args(["--confirm-send", "--udp-host", host])
            with self.subTest(host=host), \
                    patch("socket.socket", side_effect=AssertionError("不能打开网络")), \
                    patch("subprocess.Popen", side_effect=AssertionError("不能启动进程")), \
                    patch.object(stream, "resolve_glove_profiles", side_effect=AssertionError("不能发现设备")), \
                    self.assertRaises(ValueError):
                stream.run(args)

    def test_check_config_remains_offline_even_when_send_is_confirmed(self):
        with TemporaryDirectory() as directory:
            profile = _synthetic_profile(Path(directory), "right")
            args = stream.build_parser().parse_args([
                "--glove-profile", str(profile), "--check-config", "--confirm-send",
                "--commission-directions",
            ])
            with patch("socket.socket", side_effect=AssertionError("离线检查不能打开网络")), \
                    patch("subprocess.Popen", side_effect=AssertionError("离线检查不能启动进程")), \
                    patch.object(stream, "resolve_glove_profiles", side_effect=AssertionError("不能发现设备")):
                self.assertEqual(stream.run(args), 0)

    def test_newest_measurement_replaces_pending_and_rejects_source_replay(self):
        config = self.config()
        state = stream._HandState("right")
        stop = threading.Event()
        connection = _Connection([_frame(1), _frame(1), _frame(2, 99), _frame(3)], on_exhaust=stop.set)
        retargeter = _Retargeter("right", config.model)
        with patch.object(stream, "connect_glove", return_value=connection), \
                patch.object(stream, "launch_official_adapter", return_value=retargeter):
            stream._produce(config, state, threading.Condition(), stop)
        self.assertEqual(config.pipeline.frames, [1, 3])
        self.assertEqual(state.duplicate, 2)
        self.assertEqual(state.pending.source_sequence, 3)
        self.assertEqual(state.pending.source_timestamp_ns, 300)
        sample = state.take(time.monotonic_ns(), 1.0)
        expected = tuple(low + (high - low) * 0.45 for low, high in reversed(retargeter.limits))
        np.testing.assert_allclose(sample.values, expected)
        self.assertIsNone(state.take(time.monotonic_ns(), 1.0), "同一个测量不能被消费两次")
        self.assertTrue(connection.closed and retargeter.closed)

    def test_solver_time_counts_towards_source_age(self):
        config = self.config(max_frame_age=0.2)
        state = stream._HandState("right")
        stop = threading.Event()
        connection = _Connection([_frame(1)], on_exhaust=stop.set)
        retargeter = _Retargeter("right", config.model)
        with patch.object(stream, "connect_glove", return_value=connection), \
                patch.object(stream, "launch_official_adapter", return_value=retargeter), \
                patch.object(stream.time, "monotonic_ns", side_effect=[10_000_000_000, 10_300_000_000]):
            stream._produce(config, state, threading.Condition(), stop)
        self.assertEqual(state.solved, 1)
        self.assertEqual(state.stale, 1)
        self.assertIsNone(state.pending)
        self.assertTrue(connection.closed and retargeter.closed)

    def test_waiting_in_mailbox_cannot_refresh_an_old_measurement(self):
        state = stream._HandState("left", pending=stream._Measurement((0.1,) * 20, 10_000_000_000, 1, 100))
        self.assertIsNone(state.take(10_210_000_000, 0.2))
        self.assertEqual(state.stale, 1)
        self.assertIsNone(state.take(10_220_000_000, 0.2))

    def test_failure_clears_unsent_measurement_before_closing_connection(self):
        config = self.config()
        state = stream._HandState("right")
        closed_mailbox = []
        connection = _Connection([_frame(1)], on_close=lambda: closed_mailbox.append(state.pending))
        retargeter = _Retargeter("right", config.model)
        with patch.object(stream, "connect_glove", return_value=connection), \
                patch.object(stream, "launch_official_adapter", return_value=retargeter):
            stream._produce(config, state, threading.Condition(), threading.Event())
        self.assertIsNotNone(state.failure)
        self.assertEqual(closed_mailbox, [None])
        self.assertIsNone(state.take(time.monotonic_ns(), 1.0))
        self.assertTrue(connection.closed and retargeter.closed)

    def test_conversion_failure_reports_current_raw_frame_not_previous_result(self):
        config = self.config()
        state = stream._HandState("right")
        bad_frame = EncoderFrame(2, 200, 0, [float(index * 10) for index in range(21)])
        connection = _Connection([_frame(1), bad_frame])
        retargeter = _Retargeter("right", config.model)
        stderr = io.StringIO()

        def process(frame, *, zero_profile):
            if frame.sequence == 2:
                raise ValueError("J12 四连杆无法闭合")
            return frame.sequence

        with patch.object(stream, "connect_glove", return_value=connection), \
                patch.object(stream, "launch_official_adapter", return_value=retargeter), \
                patch.object(config.pipeline, "process_frame", side_effect=process), \
                redirect_stderr(stderr):
            stream._produce(config, state, threading.Condition(), threading.Event())
        diagnostic = json.loads(stderr.getvalue().split(": ", 1)[1])
        self.assertEqual(diagnostic["sequence"], 2)
        self.assertEqual(diagnostic["timestamp_ns"], 200)
        self.assertEqual(diagnostic["hand"], "right")
        self.assertEqual(diagnostic["stage"], "glove_kinematics")
        self.assertEqual(diagnostic["angles_deg"],
                         {f"J{index + 1}": value for index, value in enumerate(bad_frame.angles_deg)})
        self.assertIsNone(state.pending)
        self.assertTrue(connection.closed and retargeter.closed)

    def test_one_side_failure_invalidates_already_sent_pose_before_healthy_packet(self):
        configs = [self.config("left"), self.config("right")]
        left_sent = threading.Event()
        left_closed = threading.Event()
        right_sent = [threading.Event(), threading.Event()]

        def wait_for(event):
            if not event.wait(2):
                raise TimeoutError("没有收到前一新帧或侧失效")

        connections = {
            "left": _Connection([_frame(1)], on_exhaust=lambda: wait_for(right_sent[0]),
                                on_close=left_closed.set),
        }

        class RightConnection(_Connection):
            count = 0

            def read_latest_frame(self):
                if self.count == 0:
                    wait_for(left_sent)
                elif self.count == 1:
                    wait_for(left_closed)
                else:
                    wait_for(right_sent[1])
                    raise EOFError("第二侧也断开")
                self.count += 1
                return _frame(self.count)

        connections["right"] = RightConnection()
        workers = {config.args.hand: _Retargeter(config.args.hand, config.model) for config in configs}
        right_count = [0]

        def delivered(packet):
            if packet["flags"] == 1:
                left_sent.set()
            else:
                right_sent[right_count[0]].set()
                right_count[0] += 1

        sender, packets, receiver = self.record_sender(delivered)
        with patch.object(stream, "connect_glove", side_effect=lambda args: connections[args.hand]), \
                patch.object(stream, "launch_official_adapter", side_effect=lambda args, **_: workers[args.hand]), \
                patch.object(stream, "TianjiHandSender", return_value=sender):
            status = stream._run_stream(configs, configs[0].args)
        self.assertEqual(status, 1)
        self.assertEqual([packet["flags"] for packet in packets], [1, 3, 2])
        self.assertEqual(packets[1]["left"], packets[0]["left"])
        self.assertEqual(packets[1]["left_timestamp_ns"], packets[0]["left_timestamp_ns"])
        self.assertNotEqual(packets[1]["right"], packets[2]["right"])
        self.assertEqual(packets[2]["left"], (0.0,) * 20)
        self.assertEqual(packets[2]["left_timestamp_ns"], 0)
        with self.assertRaises(socket.timeout):
            receiver.recvfrom(8192)
        self.assertTrue(all(connection.closed for connection in connections.values()))
        self.assertTrue(all(worker.closed for worker in workers.values()))

    def test_delayed_solution_keeps_read_timestamp_and_exit_never_repeats_or_zeros(self):
        config = self.config(seconds=0.3)
        delivered = threading.Event()
        now = [10_000_000_000]

        def wait_for_exit():
            if not delivered.wait(2):
                raise TimeoutError("首帧未发送")
            time.sleep(0.08)

        class SlowWorker(_Retargeter):
            def retarget(self, sequence, *, apply_filter):
                result = super().retarget(sequence, apply_filter=apply_filter)
                now[0] = 10_100_000_000
                return result

        connection = _Connection([_frame(1)], on_exhaust=wait_for_exit)
        worker = SlowWorker("right", config.model)

        def finish(_packet):
            now[0] = 10_400_000_000
            delivered.set()

        sender, packets, receiver = self.record_sender(finish)
        with patch.object(stream, "connect_glove", return_value=connection), \
                patch.object(stream, "launch_official_adapter", return_value=worker), \
                patch.object(stream, "TianjiHandSender", return_value=sender), \
                patch.object(stream.time, "monotonic", side_effect=lambda: now[0] / 1_000_000_000), \
                patch.object(stream.time, "monotonic_ns", side_effect=lambda: now[0]):
            status = stream._run_stream([config], config.args)
        self.assertEqual(status, 0)
        self.assertEqual(len(packets), 1)
        self.assertEqual(packets[0]["flags"], 2)
        self.assertTrue(any(value != 0 for value in packets[0]["right"]))
        self.assertEqual(packets[0]["right_timestamp_ns"], 10_000_000_000)
        self.assertEqual(packets[0]["timestamp_ns"], 10_100_000_000)
        with self.assertRaises(socket.timeout):
            receiver.recvfrom(8192)
        self.assertTrue(connection.closed and worker.closed)

    def test_finite_run_with_only_expired_solutions_returns_failure(self):
        config = self.config(seconds=0.1, max_frame_age=0.2)
        now = [10_000_000_000]
        connection = _Connection([_frame(1)], on_exhaust=lambda: time.sleep(0.05))

        class SlowWorker(_Retargeter):
            def retarget(self, sequence, *, apply_filter):
                result = super().retarget(sequence, apply_filter=apply_filter)
                now[0] = 10_300_000_000
                return result

        worker = SlowWorker("right", config.model)
        sender, packets, receiver = self.record_sender()
        with patch.object(stream, "connect_glove", return_value=connection), \
                patch.object(stream, "launch_official_adapter", return_value=worker), \
                patch.object(stream, "TianjiHandSender", return_value=sender), \
                patch.object(stream.time, "monotonic", side_effect=lambda: now[0] / 1_000_000_000), \
                patch.object(stream.time, "monotonic_ns", side_effect=lambda: now[0]):
            status = stream._run_stream([config], config.args)
        self.assertEqual(status, 1)
        self.assertEqual(packets, [])
        with self.assertRaises(socket.timeout):
            receiver.recvfrom(8192)
        self.assertTrue(connection.closed and worker.closed)

    def test_solver_exception_closes_both_resources_without_publishing(self):
        config = self.config()
        connection = _Connection([_frame(1)])
        worker = _Retargeter("right", config.model, fail=True)
        sender, packets, receiver = self.record_sender()
        with patch.object(stream, "connect_glove", return_value=connection), \
                patch.object(stream, "launch_official_adapter", return_value=worker), \
                patch.object(stream, "TianjiHandSender", return_value=sender):
            status = stream._run_stream([config], config.args)
        self.assertEqual(status, 1)
        self.assertEqual(packets, [])
        with self.assertRaises(socket.timeout):
            receiver.recvfrom(8192)
        self.assertTrue(connection.closed and worker.closed)


if __name__ == "__main__":
    unittest.main()
