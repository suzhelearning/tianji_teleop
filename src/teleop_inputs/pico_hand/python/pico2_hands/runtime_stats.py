"""Bounded, windowed diagnostics; durations are local wall time, not headset latency.

The control thread owns samples and summaries. The receiver calls record_input
under the same external slot lock used by the control thread for summary().
No per-frame formatting, sample arrays, background queues or clock reads here.
"""
from bisect import bisect_left
import math


# Quantiles are bucket upper bounds: 0.1 ms through 20 ms, then coarser
# buckets through one second. Overflow reports the observed maximum.
_BOUNDS_NS = tuple(range(100_000, 20_000_001, 100_000)) + tuple(
    range(21_000_000, 100_000_001, 1_000_000)) + tuple(
    range(110_000_000, 1_000_000_001, 10_000_000))
_METRICS = ("cycle", "work", "input_interval", "receive_age", "input_processing",
            "dls", "hands", "model", "output")


class _Durations:
    def __init__(self):
        self.reset()

    def reset(self):
        self.bins = [0] * (len(_BOUNDS_NS) + 1)
        self.count = self.total = self.maximum = 0

    def add(self, duration_ns):
        if duration_ns < 0:
            return
        self.bins[bisect_left(_BOUNDS_NS, duration_ns)] += 1
        self.count += 1
        self.total += duration_ns
        self.maximum = max(self.maximum, duration_ns)

    def summary(self):
        if not self.count:
            return None
        ranks = [math.ceil(self.count * p) for p in (.5, .95, .99)]
        quantiles = []
        cumulative = 0
        for index, count in enumerate(self.bins):
            cumulative += count
            while len(quantiles) < 3 and cumulative >= ranks[len(quantiles)]:
                bound = _BOUNDS_NS[index] if index < len(_BOUNDS_NS) else self.maximum
                quantiles.append(min(bound, self.maximum) / 1e6)
            if len(quantiles) == 3:
                break
        return dict(count=self.count, mean_ms=round(self.total / self.count / 1e6, 4),
                    p50_ms=round(quantiles[0], 4), p95_ms=round(quantiles[1], 4),
                    p99_ms=round(quantiles[2], 4), max_ms=round(self.maximum / 1e6, 4))


class RuntimeStats:
    def __init__(self, interval_s, start_ns):
        if not math.isfinite(interval_s) or interval_s <= 0:
            raise ValueError("positive finite stats interval required")
        self.interval_ns = max(1, int(interval_s * 1e9))
        self.started_ns = start_ns
        self.metrics = {name: _Durations() for name in _METRICS}
        self.cycles = self.input_frames = self.accepted_frames = self.overruns = 0
        self.braking_reasons = {}
        self._last_cycle_ns = None
        self._last_phase = None
        self._input_identity = None
        self._input_sequence = -1
        self._input_stamp = None

    def observe(self, name, duration_ns):
        self.metrics[name].add(duration_ns)

    def record_input(self, frame):
        identity = (frame.receiver_instance_id, frame.connection_generation)
        if identity != self._input_identity:
            self._input_identity = identity
            self._input_sequence = -1
            self._input_stamp = None
        if frame.receiver_frame_sequence <= self._input_sequence:
            return
        stamp = frame.received_timestamp_ns
        if self._input_stamp is not None:
            self.observe("input_interval", stamp - self._input_stamp)
        self._input_sequence = frame.receiver_frame_sequence
        self._input_stamp = stamp
        self.input_frames += 1

    def record_cycle(self, start_ns, end_ns, frame, phase, braking_reason):
        if self._last_cycle_ns is not None:
            self.observe("cycle", start_ns - self._last_cycle_ns)
        self._last_cycle_ns = start_ns
        self.observe("work", end_ns - start_ns)
        self.cycles += 1
        self.overruns += end_ns - start_ns > 5_000_000
        if frame is not None:
            self.observe("receive_age", end_ns - frame.received_timestamp_ns)
        if phase == "BRAKING" and self._last_phase != "BRAKING":
            reason = braking_reason or "unspecified"
            self.braking_reasons[reason] = self.braking_reasons.get(reason, 0) + 1
        self._last_phase = phase

    def due(self, now_ns):
        return now_ns - self.started_ns >= self.interval_ns

    def summary(self, now_ns, *, final=False):
        if not self.cycles or (not final and not self.due(now_ns)):
            return None
        elapsed = (now_ns - self.started_ns) / 1e9
        if elapsed <= 0:
            return None
        result = dict(kind="pico2_runtime_stats", final=final, phase=self._last_phase,
                      window_s=round(elapsed, 3),
                      control_hz=round(self.cycles / elapsed, 2),
                      input_hz=round(self.input_frames / elapsed, 2),
                      accepted_input_hz=round(self.accepted_frames / elapsed, 2),
                      cycles=self.cycles, input_frames=self.input_frames,
                      work_over_5ms=self.overruns, braking_entries=dict(self.braking_reasons),
                      timing_ms={name: metric.summary() for name, metric in self.metrics.items()})
        self.started_ns = now_ns
        self.cycles = self.input_frames = self.accepted_frames = self.overruns = 0
        self.braking_reasons.clear()
        for metric in self.metrics.values():
            metric.reset()
        return result
