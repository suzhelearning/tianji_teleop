#!/usr/bin/env python3
"""Explicit arms-only HOME motion; no teleoperation input or hand SDK access."""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import signal
import sys
import time

from tianji_runtime.resources import config_path as workspace_config, package_share, workspace
from .run_teleop import load_configuration, make_hardware
from .ros_commands import ExecutorLease
from .safety import SafetyFault
from .staged_motion import HOME_REACHED, HOMING, StagedMotionGate


class ArmHomeGate(StagedMotionGate):
    """Use existing bounds, feedback, tracking and HOME settling without a fake source."""

    def __init__(self, config):
        super().__init__(config["safety"], ("arms",), config["staged_motion"])

    def begin(self, feedback, now_ns):
        if self.armed or self.fault:
            raise SafetyFault("HOME session already started/faulted; restart required")
        measured = self.check_feedback("arms", feedback, now_ns, require_enabled=True)
        self._last_commands = {"arms": measured}
        self._last_step_ns = now_ns
        self._targets = {"arms": self._home}
        self.armed = True
        self._enter_phase(HOMING, now_ns)

    def advance(self, feedback, now_ns):
        if not self.armed or self.fault or self.phase != HOMING:
            raise SafetyFault(self.fault or "HOME motion is not active")
        try:
            if not 0 < now_ns - self._last_step_ns <= self.command_timeout_ns:
                raise SafetyFault("HOME execution clock stalled or did not advance")
            measured = self.check_feedback("arms", feedback, now_ns, require_enabled=True)
            previous = self._last_commands["arms"]
            self._check_tracking("arms", previous, measured, now_ns,
                                 feedback.received_monotonic_ns)
            dt = (now_ns - self._last_step_ns) / 1e9
            self._last_commands = self._slow_commands(
                {"arms": measured}, {"arms": feedback}, now_ns, dt)
            command = self._last_commands["arms"]
            self._last_step_ns = now_ns
            self._advance_staged(HOMING, self._last_commands, {"arms": measured}, now_ns, {"arms": feedback})
            return command
        except SafetyFault as error:
            self.fault = str(error)
            raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=workspace_config("robot.json"))
    posture = parser.add_mutually_exclusive_group()
    posture.add_argument("--home-config", type=Path, help="alternate dual-arm HOME YAML; default uses robot configuration")
    posture.add_argument("-L", "--L", action="store_true", help="use the dual-arm home-L posture")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--confirm-real", action="store_true", help="authorize immediate dual-arm HOME motion")
    mode.add_argument("--dry-run", action="store_true", help="validate and show HOME without connecting hardware (default)")
    args = parser.parse_args(argv)
    device = receiver = None
    handlers = {}
    result = 0
    try:
        config_path = args.config.resolve()
        if args.L:
            args.home_config = package_share("tianji_description", "config", "home-L.yaml")
        config, _ = load_configuration(config_path, "arms", home_config=args.home_config)
        gate = ArmHomeGate(config)
        for side in ("left", "right"):
            angles = [round(math.degrees(q), 3) for q in config["staged_motion"][f"home_{side}_rad"]]
            print(f"HOME {side} deg={angles}", flush=True)
        print("Arms only; Wuji hands are not connected or commanded.", flush=True)
        if not args.confirm_real:
            print("DRY RUN: configuration valid; no hardware connected.", flush=True)
            return 0
        if not sys.stdin.isatty():
            raise RuntimeError("real HOME requires an operator terminal; keep the physical emergency stop reachable")

        def interrupt(signum, frame):
            raise KeyboardInterrupt

        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, interrupt)
        # Reserve teleop's machine-wide lease before touching hardware. No ROS
        # controller is started and no source commands are fabricated/consumed.
        receiver = ExecutorLease()
        device = make_hardware(config, ("arms",), workspace())["arms"]
        device.connect()

        def enable_guard():
            measured = device.read_feedback()
            gate.check_feedback("arms", measured, time.monotonic_ns())

        enable_guard()
        print("HOME PREPARE | 准备双臂回位；已使能时接管，未使能时使能。", flush=True)
        device.enable(guard=enable_guard, allow_enabled=True)
        measured = device.read_feedback()
        gate.begin(measured, time.monotonic_ns())
        print("HOMING | 双臂慢速回 HOME；Ctrl+C: 直接停止。请保持运动路径无障碍。", flush=True)
        period = 1.0 / gate.rate_hz
        while gate.phase != HOME_REACHED:
            deadline = time.monotonic() + period
            measured = device.read_feedback()
            command = gate.advance(measured, time.monotonic_ns())
            if gate.phase == HOME_REACHED:
                break
            device.send(command)
            remaining = deadline - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
        print("HOME_REACHED | 双臂实测到位，正在停止并失能；Wuji hands untouched.", flush=True)
    except KeyboardInterrupt:
        print("HOME interrupted; stopping arms without continuing the return.", flush=True)
        result = 130
    except Exception as error:
        print(f"HOME STOP: {type(error).__name__}: {error}", file=sys.stderr, flush=True)
        for note in getattr(error, "__notes__", ()):
            print(note, file=sys.stderr, flush=True)
        result = 1
    finally:
        for sig in handlers:
            signal.signal(sig, signal.SIG_IGN)
        if device is not None:
            for operation in ("stop", "close"):
                try:
                    getattr(device, operation)()
                except Exception as error:
                    print(f"arms {operation} failed: {error}; use physical emergency stop", file=sys.stderr, flush=True)
                    result = 1
        if receiver is not None:
            receiver.close()
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
