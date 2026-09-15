#!/usr/bin/env python3
"""Run dual-arm and both Hand2 MuJoCo dynamics, without any hardware connection.

The existing controller supplies reference targets over loopback TJRC v2 UDP.
Missing or stale input holds the last setpoint while physics keeps advancing.
Simulation servo parameters are not real-hardware calibration.
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack, nullcontext
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parent.parent
ARM_ROOT = ROOT / "control"
sys.path.insert(0, str(ROOT))

from real_robot.protocol import DEVICE_READY_FLAGS
from real_robot.run_teleop import CommandReceiver, stop_controller

COMMAND_TIMEOUT_NS = 150_000_000
CLOCK_TOLERANCE_NS = 5_000_000
GROUP_SLICES = {"arms": slice(0, 14), "left_hand": slice(14, 34), "right_hand": slice(34, 54)}


def command_is_fresh(frame, now_ns):
    # Match the existing executor's monotonic-clock tolerance and 150 ms age.
    return frame is not None and -CLOCK_TOLERANCE_NS <= now_ns - frame.timestamp_ns <= COMMAND_TIMEOUT_NS


def run_loop(simulation, receiver, controller, viewer, duration, stop_requested):
    started = time.monotonic()
    timestep = float(simulation.model.opt.timestep)
    if not math.isfinite(timestep) or timestep <= 0:
        raise ValueError("model timestep must be finite and positive")
    next_step = started + timestep
    next_render = started
    next_report = started + 1.0
    deadline = started + duration if duration else math.inf
    ready_packets = dict.fromkeys(DEVICE_READY_FLAGS, 0)
    stale_packets = 0
    missing_sequences = 0
    previous_sequence = None
    steps = 0
    dropped_wall_time = 0.0
    reason = "stopped"

    def accept_frame(frame):
        nonlocal stale_packets, missing_sequences, previous_sequence
        if previous_sequence is not None:
            missing_sequences += frame.sequence - previous_sequence - 1
        previous_sequence = frame.sequence
        if not command_is_fresh(frame, time.monotonic_ns()):
            stale_packets += 1
            return
        # The engine updates READY groups only; no qpos is written here.
        simulation.set_targets(frame)
        for group, flag in DEVICE_READY_FLAGS.items():
            if frame.flags & flag:
                ready_packets[group] += 1

    def report(label):
        frame = receiver.latest
        now_ns = time.monotonic_ns()
        fresh = command_is_fresh(frame, now_ns)
        states = {}
        errors = {}
        positions = simulation.data.qpos[simulation.qpos_indices]
        for group, flag in DEVICE_READY_FLAGS.items():
            states[group] = ("ready" if fresh and frame.flags & flag else
                             "stale-hold" if ready_packets[group] else "waiting")
            indices = GROUP_SLICES[group]
            errors[group] = float(max(abs(positions[indices] - simulation.targets[indices])))
        payload = {
            "wall_time_s": round(time.monotonic() - started, 6),
            "physics_time_s": float(simulation.data.time),
            "physics_steps": steps,
            "packets_received": receiver.count,
            "stale_packets": stale_packets,
            "missing_sequences": missing_sequences,
            "last_sequence": frame.sequence if frame else None,
            "packet_age_s": (now_ns - frame.timestamp_ns) / 1e9 if frame else None,
            "ready_packets": ready_packets,
            "groups": states,
            "max_target_error_rad": errors,
            "dropped_wall_time_s": round(dropped_wall_time, 6),
        }
        if label == "SIM_SUMMARY":
            payload["reason"] = reason
        print(label, json.dumps(payload, sort_keys=True), flush=True)

    try:
        while not stop_requested.is_set():
            now = time.monotonic()
            if now >= deadline:
                reason = "duration"
                break
            returncode = controller.poll()
            if returncode is not None:
                reason = "controller-exit"
                raise RuntimeError(f"controller exited unexpectedly with status {returncode}")
            if viewer is not None and not viewer.is_running():
                reason = "viewer-closed"
                break
            with viewer.lock() if viewer is not None else nullcontext():
                receiver.drain(accept_frame)
                # Bound recovery after scheduling/rendering stalls. Never enlarge
                # the physics timestep or run an unbounded catch-up backlog.
                batch_steps = 0
                while now >= next_step and batch_steps < 8 and not stop_requested.is_set():
                    simulation.step()
                    steps += 1
                    batch_steps += 1
                    next_step += timestep
            now = time.monotonic()
            if now >= next_step and batch_steps == 8:
                dropped_wall_time += now - next_step + timestep
                next_step = now + timestep
            if viewer is not None and now >= next_render:
                viewer.sync()
                next_render = time.monotonic() + 1.0 / 60.0
            if now >= next_report:
                report("SIM_STATUS")
                next_report = time.monotonic() + 1.0
            wake = min(next_step, next_report, deadline, now + 0.02)
            if viewer is not None:
                wake = min(wake, next_render)
            # A minimum yield also bounds CPU consumption when physics cannot
            # keep up with real time; reported physics time remains authoritative.
            stop_requested.wait(max(0.0001, wake - time.monotonic()))
    except BaseException:
        if reason == "stopped":
            reason = "error"
        raise
    finally:
        report("SIM_SUMMARY")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="run the same physics without opening a viewer")
    parser.add_argument("--duration", type=float, default=0.0, help="stop after N wall-clock seconds (0: until stopped)")
    parser.add_argument("--pico-port", type=int, default=15000, help="loopback PICO input port (default: 15000)")
    parser.add_argument("--hand-port", type=int, default=16000, help="loopback hand input port (default: 16000)")
    parser.add_argument("--config", type=Path, default=ARM_ROOT / "config/qp_ik_pico_teleop.yaml",
                        help="controller YAML, also used for the initial arm pose")
    parser.add_argument("--model", type=Path, default=ARM_ROOT / "models/marvin_m6_wuji2.xml",
                        help="dual-arm and both Hand2 MuJoCo XML")
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("--duration must be finite and non-negative")
    for name in ("pico_port", "hand_port"):
        if not 1 <= getattr(args, name) <= 65535:
            parser.error(f"--{name.replace('_', '-')} must be in [1,65535]")
    if args.pico_port == args.hand_port:
        parser.error("--pico-port and --hand-port must be different")
    config_path = args.config.resolve()
    model_path = args.model.resolve()
    stop_requested = threading.Event()

    def request_stop(signum, frame):
        stop_requested.set()

    try:
        with ExitStack() as cleanup:
            for sig in (signal.SIGINT, signal.SIGTERM):
                previous = signal.signal(sig, request_stop)
                cleanup.callback(signal.signal, sig, previous)
            # Delay MuJoCo imports until after argument parsing so --help works
            # without a graphics context or any simulation dependencies loaded.
            from sim.physics import PhysicsSimulation

            simulation = PhysicsSimulation(model_path, config_path)
            if stop_requested.is_set():
                return 0
            executable = ARM_ROOT / "build/tianji_qp_ik_viewer"
            if not executable.is_file():
                raise RuntimeError("build the arm controller before starting --sim")
            receiver = CommandReceiver(0)
            cleanup.callback(receiver.close)
            if receiver.port in (17000, args.pico_port, args.hand_port):
                raise RuntimeError("allocated command output port conflicts with a reserved input or hardware port")
            command = [
                "pixi", "run", "--manifest-path", str(ROOT / "pixi.toml"), str(executable),
                "--config", str(config_path), "--model", str(model_path),
                "--headless", "--continuous", "--model-state-only",
                "--pico-teleop", "--pico-bind", "127.0.0.1", "--pico-port", str(args.pico_port),
                "--hand-teleop", "--hand-bind", "127.0.0.1", "--hand-port", str(args.hand_port),
                "--joint-command-host", "127.0.0.1", "--joint-command-port", str(receiver.port),
            ]
            print("SIM: MuJoCo dynamics; no SDK, device connection, or hardware commands. "
                  "Stale groups hold targets; simulation parameters are not hardware calibration.", flush=True)
            print(f"SIM: command receiver 127.0.0.1:{receiver.port}; "
                  f"PICO={args.pico_port}, hands={args.hand_port}", flush=True)
            controller = subprocess.Popen(command, cwd=ARM_ROOT, start_new_session=True,
                                          stdin=subprocess.DEVNULL)
            cleanup.callback(stop_controller, controller)
            viewer = None
            if not args.headless and not stop_requested.is_set():
                import mujoco.viewer

                viewer = mujoco.viewer.launch_passive(
                    simulation.model, simulation.data, show_left_ui=False, show_right_ui=False,
                )
                cleanup.callback(viewer.close)
                with viewer.lock():
                    viewer.cam.lookat[:] = (0.0, 0.0, 0.85)
                    viewer.cam.distance = 2.5
                    viewer.cam.azimuth = 135.0
                    viewer.cam.elevation = -15.0
                print("SIM_READY: viewer opened; displaying actual MuJoCo dynamics state", flush=True)
            elif args.headless:
                print("SIM_READY: headless MuJoCo dynamics", flush=True)
            run_loop(simulation, receiver, controller, viewer, args.duration, stop_requested)
    except Exception as error:
        print(f"SIM ERROR: {error}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
