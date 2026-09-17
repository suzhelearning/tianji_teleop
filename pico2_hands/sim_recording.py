"""Isolated simulation journal schema; not the source project's session schema."""
import queue
import threading
import json
import h5py
import numpy as np


class SimRecorder:
    def __init__(self, path):
        self.file = h5py.File(path, "x")
        self.file.attrs.update(schema="pico2_sim_session_v1", complete=False,
                               scope="simulation_only", joint_units="radians")
        self.raw = self.file.create_dataset("raw/pico_hand_tracking/packet", (0, 1982),
                                            maxshape=(None, 1982), chunks=(64, 1982), dtype="u1")
        self.meta = self.file.create_dataset("raw/pico_hand_tracking/metadata", (0, 4),
                                             maxshape=(None, 4), dtype="i8", chunks=True)
        self.meta.attrs["columns"] = "receive_monotonic_ns,source_ms,connection_generation,receiver_sequence"
        self.commands = self.file.create_dataset("simulation/position_rad", (0, 54),
                                                 maxshape=(None, 54), chunks=True, dtype="f8")
        self.times = self.file.create_dataset("simulation/monotonic_ns", (0,), maxshape=(None,), dtype="i8")
        self.states = self.file.create_dataset("simulation/state", (0,), maxshape=(None,), dtype="S16")
        self.association = self.file.create_dataset("simulation/input_association", (0, 2),
            maxshape=(None, 2), dtype="i8", chunks=True)
        self.association.attrs["columns"] = "connection_generation,receiver_sequence; -1 means absent"
        self.events = self.file.create_dataset("events/json", (0,), maxshape=(None,),
                                               dtype=h5py.string_dtype("utf-8"))
        self.queue = queue.Queue(maxsize=8192)
        self.offer_lock = threading.Lock()
        self.error = None
        self.accepted = self.processed = 0
        self.thread = threading.Thread(target=self._run, name="pico2-sim-record", daemon=True)
        self.thread.start()

    def offer(self, kind, value):
        if self.error:
            raise RuntimeError(self.error)
        with self.offer_lock:
            try:
                self.queue.put_nowait((kind, value))
            except queue.Full as exc:
                raise RuntimeError("simulation recording queue overflow") from exc
            self.accepted += 1

    def _run(self):
        try:
            while True:
                item = self.queue.get()
                if item is None:
                    return
                kind, value = item
                if kind == "raw":
                    i = len(self.raw)
                    self.raw.resize(i+1, axis=0); self.meta.resize(i+1, axis=0)
                    self.raw[i] = np.frombuffer(value.raw_packet, dtype="u1")
                    self.meta[i] = [value.received_timestamp_ns, value.source_timestamp_ms,
                                    value.connection_generation, value.receiver_frame_sequence]
                    self.file.attrs["receiver_instance_id"] = value.receiver_instance_id
                elif kind == "event":
                    i = len(self.events)
                    self.events.resize(i+1, axis=0)
                    self.events[i] = json.dumps(value)
                else:
                    stamp, state, q, association = value
                    i = len(self.commands)
                    for dataset in (self.commands, self.times, self.states, self.association):
                        dataset.resize(i+1, axis=0)
                    self.commands[i], self.times[i], self.states[i] = q, stamp, state
                    self.association[i] = association
                self.processed += 1
        except Exception as exc:
            self.error = repr(exc)

    def close(self, complete):
        if self.thread.is_alive():
            self.queue.put(None, timeout=5)
            self.thread.join(timeout=10)
        if self.thread.is_alive():
            raise RuntimeError("simulation recorder did not stop; file not marked complete")
        try:
            drained = self.error is None and self.accepted == self.processed
            self.file.attrs["complete"] = bool(complete and drained)
            self.file.attrs["accepted"] = self.accepted
            self.file.attrs["processed"] = self.processed
            self.file.flush()
        finally:
            self.file.close()
        if not drained:
            raise RuntimeError(self.error or "simulation recording incomplete")
