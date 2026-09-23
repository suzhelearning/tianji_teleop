from contextlib import ExitStack
import os
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from pico2_hands.hand_worker import NativeHandWorker, RESPONSE
from tianji_runtime.resources import ResourceNotFound, native_executable


def native_worker_available():
    try:
        native_executable("tianji_hand_native_worker")
    except ResourceNotFound:
        return False
    return True


def points(sequence=1, side="right"):
    result = np.array([[0., 0., 0.]] + [
        [(finger - 2) * .02, .025 * (joint + 1),
         .001 * (joint + 1) ** 2 * np.sin(sequence * .1)]
        for finger in range(5) for joint in range(4)])
    if side == "left":
        result[:, 0] *= -1
    return result


class HandTransportTests(unittest.TestCase):
    def transport(self, side="right"):
        # Real nonblocking pipes exercise framing/deadlines without a native build.
        request_read, request_write = os.pipe()
        response_read, response_write = os.pipe()
        streams = [os.fdopen(fd, mode, buffering=0) for fd, mode in (
            (request_read, "rb"), (request_write, "wb"),
            (response_read, "rb"), (response_write, "wb"))]
        for stream in streams:
            self.addCleanup(stream.close)
        os.set_blocking(request_write, False)
        os.set_blocking(response_read, False)
        worker = NativeHandWorker.__new__(NativeHandWorker)
        worker.side, worker.flag = side, 1 if side == "left" else 2
        worker.timeout_s = 1.
        worker.sequence = worker.timestamp = 0
        worker.failed = worker.closed = False
        worker._pending = None
        worker.process = SimpleNamespace(stdin=streams[1], stdout=streams[2])
        return worker, streams[0], streams[3]

    def response(self, *, flag=2, sequence=1, timestamp=1_000_000, joints=None):
        return RESPONSE.pack(b"TJHR", 1, flag, RESPONSE.size, sequence, timestamp,
                             *(range(40) if joints is None else joints))

    def assert_failed(self, worker):
        self.assertTrue(worker.failed)
        self.assertEqual((worker.sequence, worker.timestamp), (0, 0))
        with self.assertRaises(RuntimeError):
            worker.submit(points(), 2, 2_000_000)
        with self.assertRaises(RuntimeError):
            worker.receive()

    def test_inflight_misuse_does_not_replace_or_consume_pending_frame(self):
        worker, _, reply = self.transport()
        with self.assertRaises(RuntimeError):
            worker.receive()
        worker.submit(points(), 1, 1_000_000)
        with self.assertRaises(RuntimeError):
            worker.submit(points(2), 2, 2_000_000)
        reply.write(self.response())
        np.testing.assert_array_equal(worker.receive(), np.arange(20, 40))
        with self.assertRaises(RuntimeError):
            worker.receive()
        self.assertFalse(worker.failed)
        # Equal source time is allowed; sequence rollback or older time is not.
        for sequence, stamp in ((1, 1_000_001), (2, 999_999)):
            with self.assertRaises(ValueError):
                worker.submit(points(), sequence, stamp)
        worker.submit(points(2), 2, 1_000_000)
        reply.write(self.response(sequence=2))
        np.testing.assert_array_equal(worker.receive(), np.arange(20, 40))
        self.assertEqual((worker.sequence, worker.timestamp), (2, 1_000_000))

    def test_invalid_input_does_not_poison_next_valid_frame(self):
        worker, _, reply = self.transport("left")
        nonfinite = points()
        nonfinite[2, 1] = np.nan
        for values, sequence, stamp in (
                (np.zeros((20, 3)), 1, 1_000_000),
                (nonfinite, 1, 1_000_000),
                (points(), True, 1_000_000),
                (points(), 1, 0),
                (points(), 1, 1 << 63)):
            with self.subTest(sequence=sequence, stamp=stamp, shape=values.shape):
                with self.assertRaises(ValueError):
                    worker.submit(values, sequence, stamp)
        worker.submit(points(), 1, 1_000_000)
        reply.write(self.response(flag=1))
        np.testing.assert_array_equal(worker.receive(), np.arange(20))
        self.assertFalse(worker.failed)

    def test_response_deadline_includes_submit_write_and_is_not_renewed(self):
        worker, _, reply = self.transport()
        now = [10.]
        real_write = os.write

        def slow_write(fd, payload):
            result = real_write(fd, payload)
            now[0] += .75
            return result

        with patch("pico2_hands.hand_worker.time.monotonic", side_effect=lambda: now[0]):
            with patch("pico2_hands.hand_worker.os.write", side_effect=slow_write):
                worker.submit(points(), 1, 1_000_000)
            reply.write(self.response())
            now[0] = 11.01
            # Even a buffered response is too late; receive gets no fresh budget.
            with self.assertRaises(TimeoutError):
                worker.receive()
        self.assert_failed(worker)

    def test_association_mismatch_latches_without_publishing(self):
        for field, replacement in ((0, b"FAIL"), (1, 2), (2, 1),
                                   (3, RESPONSE.size - 1), (4, 2), (5, 1_000_001)):
            with self.subTest(field=field):
                worker, _, reply = self.transport()
                row = list(RESPONSE.unpack(self.response()))
                row[field] = replacement
                worker.submit(points(), 1, 1_000_000)
                reply.write(RESPONSE.pack(*row))
                with self.assertRaises(RuntimeError):
                    worker.receive()
                self.assert_failed(worker)

    def test_nonfinite_output_in_either_half_latches(self):
        for index, value in ((0, np.nan), (20, np.inf)):
            with self.subTest(index=index):
                worker, _, reply = self.transport()
                joints = np.zeros(40)
                joints[index] = value
                worker.submit(points(), 1, 1_000_000)
                reply.write(self.response(joints=joints))
                with self.assertRaises(RuntimeError):
                    worker.receive()
                self.assert_failed(worker)

    def test_truncated_response_latches(self):
        worker, _, reply = self.transport()
        worker.submit(points(), 1, 1_000_000)
        reply.write(self.response()[:-1])
        reply.close()
        with self.assertRaises(RuntimeError):
            worker.receive()
        self.assert_failed(worker)

    def test_closed_request_pipe_latches(self):
        worker, request, _ = self.transport()
        request.close()
        with self.assertRaises(BrokenPipeError):
            worker.submit(points(), 1, 1_000_000)
        self.assert_failed(worker)


@unittest.skipUnless(native_worker_available(), "build isolated native Hand2 worker first")
class NativeHandWorkerTest(unittest.TestCase):
    def test_both_sides_protocol_and_clean_close(self):
        for side in ("left", "right"):
            with NativeHandWorker(side) as worker:
                worker.submit(points(side=side), 1, 1_000_000)
                result = worker.receive()
                self.assertEqual(result.shape, (20,))
                self.assertTrue(np.isfinite(result).all())
                with self.assertRaises(ValueError):
                    worker.submit(points(side=side), 1, 1_000_001)
                self.assertEqual(worker.sequence, 1)
            self.assertEqual(worker.process.returncode, 0)

    def test_transport_failure_latches_until_explicit_close(self):
        with NativeHandWorker("left") as worker:
            worker.submit(points(), 1, 1_000_000)
            with patch.object(worker, "_read", side_effect=TimeoutError("injected")):
                with self.assertRaises(TimeoutError):
                    worker.receive()
            self.assertTrue(worker.failed)
            with self.assertRaises(RuntimeError):
                worker.submit(points(), 2, 2_000_000)

    def test_close_with_outstanding_response_exits_cleanly(self):
        for side in ("left", "right"):
            worker = NativeHandWorker(side)
            with worker:
                worker.submit(points(side=side), 1, 1_000_000)
            self.assertEqual(worker.process.returncode, 0)
            self.assertFalse(worker.failed)
            worker.close()
            with self.assertRaises(RuntimeError):
                worker.receive()

    def test_parallel_results_preserve_sequential_filter_history_and_source_times(self):
        with ExitStack() as stack:
            sequential = {side: stack.enter_context(NativeHandWorker(side))
                          for side in ("left", "right")}
            concurrent = {side: stack.enter_context(NativeHandWorker(side))
                          for side in ("left", "right")}
            # Shared gaps reset history; side-specific gaps and stamps must stay
            # independent even when the other side is submitted/received first.
            for sequence in (1, 2, 3, 4, 100, 101, 102, 103):
                active = [side for side in ("left", "right")
                          if (side, sequence) not in (("left", 3), ("right", 101))]
                stamps = {side: sequence * 10_000_000 + (3_000_000 if side == "left" else 0)
                          for side in active}
                expected = {}
                for side in active:
                    sequential[side].submit(points(sequence, side), sequence, stamps[side])
                    expected[side] = sequential[side].receive()
                for side in active:
                    concurrent[side].submit(points(sequence, side), sequence, stamps[side])
                for side in reversed(active):
                    actual = concurrent[side].receive()
                    np.testing.assert_allclose(actual, expected[side], rtol=0, atol=1e-10)
                    self.assertEqual(concurrent[side].sequence, sequence)
                    self.assertEqual(concurrent[side].timestamp, stamps[side])
