"""Receive-only probe. Does not publish joint commands or configure ADB.

Run with the hand-tracking APK, not the VR whole-body APK. Source timestamps
are used only for ordering; freshness uses this computer's monotonic clock.
"""
from __future__ import annotations

import argparse
import json
import math
import threading
import time
import uuid

from .reference.gesture_recognition import classify_hand
from .reference.official_pico import pico_official_hand_observations
from .reference.runtime import PicoTcpReceiver, pico_frame_observations


class ObservationState:
    """Thread-safe observation snapshot, deliberately without motion authority."""

    def __init__(self, freshness_s: float = 0.2):
        if not math.isfinite(freshness_s) or freshness_s <= 0:
            raise ValueError("freshness_s must be finite and positive")
        self.freshness_ns = int(freshness_s * 1e9)
        self.lock = threading.Lock()
        self.identity = None
        self.source_timestamp = None
        self.sequence = None
        self.received = None
        self.frames = 0
        self.rejected_frames = 0
        self.error = None
        self.arms = {side: False for side in ("left", "right")}
        self.hands = dict(self.arms)
        self.gestures = {side: "unknown" for side in self.arms}

    def disconnected(self, error):
        with self.lock:
            self.received = None
            self.error = str(error)
            self.gestures = {side: "unknown" for side in self.arms}

    def accept(self, frame):
        identity = (frame.receiver_instance_id, frame.connection_generation)
        with self.lock:
            if identity != self.identity:
                self.identity = identity
                self.source_timestamp = self.sequence = self.received = None
                self.gestures = {side: "unknown" for side in self.arms}
            if ((self.source_timestamp is not None and
                 frame.source_timestamp_ns <= self.source_timestamp) or
                    (self.sequence is not None and frame.receiver_frame_sequence <= self.sequence)):
                self.rejected_frames += 1
                return False
            if (self.received is None or
                    frame.received_timestamp_ns - self.received > self.freshness_ns):
                self.gestures = {side: "unknown" for side in self.arms}
            pairs = pico_frame_observations(frame)
            official = pico_official_hand_observations(frame)
            for side in self.arms:
                self.arms[side] = bool(pairs[side][1].valid)
                self.hands[side] = bool(official[side].valid)
                gesture = classify_hand(official[side].keypoints_m,
                                        valid=self.hands[side], previous=self.gestures[side])
                self.gestures[side] = gesture.gesture
            self.source_timestamp = frame.source_timestamp_ns
            self.sequence = frame.receiver_frame_sequence
            self.received = frame.received_timestamp_ns
            self.frames += 1
            self.error = None
            return True

    def snapshot(self, now_ns=None):
        now = time.monotonic_ns() if now_ns is None else now_ns
        with self.lock:
            age = None if self.received is None else now - self.received
            fresh = age is not None and 0 <= age <= self.freshness_ns
            return {"scope": "pico2_observation_only", "robot_commands_enabled": False,
                    "frames": self.frames, "rejected_frames": self.rejected_frames,
                    "identity": self.identity, "fresh": fresh,
                    "age_ms": None if age is None else age / 1e6,
                    "arms": {s: fresh and v for s, v in self.arms.items()},
                    "hands": {s: fresh and v for s, v in self.hands.items()},
                    "gestures": {s: v if fresh else "unknown" for s, v in self.gestures.items()},
                    "error": self.error}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=10002)
    parser.add_argument("--duration-s", type=float, default=10.0)
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration_s) or args.duration_s <= 0:
        parser.error("--duration-s must be finite and positive")
    if not 1 <= args.port <= 65535:
        parser.error("--port must be in 1..65535")
    state = ObservationState()
    receiver = PicoTcpReceiver(host=args.host, port=args.port,
                               receiver_instance_id=str(uuid.uuid4()),
                               auto_adb_forward=False, on_error=state.disconnected)
    failure = []

    def receive():
        try:
            receiver.run(state.accept)
        except Exception as exc:
            failure.append(repr(exc))
            state.disconnected(exc)

    thread = threading.Thread(target=receive, name="pico2-observer", daemon=True)
    thread.start()
    end = time.monotonic() + args.duration_s
    interrupted = False
    try:
        while time.monotonic() < end and thread.is_alive():
            time.sleep(min(1.0, max(0.0, end - time.monotonic())))
            print(json.dumps(state.snapshot()), flush=True)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        receiver.stop()
        thread.join(timeout=5.0)
    result = state.snapshot()
    result["receiver_stopped"] = not thread.is_alive()
    result["worker_failure"] = failure
    result["hardware_acceptance_complete"] = False
    print(json.dumps(result), flush=True)
    if interrupted:
        return 130
    return 0 if (not thread.is_alive() and not failure and result["fresh"] and
                 all(result["arms"].values()) and all(result["hands"].values())) else 1


if __name__ == "__main__":
    raise SystemExit(main())
