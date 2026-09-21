#!/usr/bin/env python3
"""MuJoCo simulation without hardware: Franka DLS + Ruckig with hand input by default.

The default direct viewer uses in-process IK/Ruckig with S/H/P recovery.
Explicit spark/mapped-palm backends retain the legacy TJRC dual-arm/Hand2 path.
Use --ik-backend spark --simulation-mode dynamics for legacy actuator physics.
Missing or stale input holds the last setpoint. Simulation servo parameters
are not real-hardware calibration.
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack, nullcontext
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from tianji_runtime import (  # noqa: E402
    controller_profile,
    native_executable,
    package_share,
    workspace,
)
from tianji_runtime.resources import ResourceNotFound  # noqa: E402

# Model and controller config are description resources, resolved through the
# installed share directory with the source tree as an explicit fallback.
DESCRIPTION_SHARE = "tianji_description"


def _description(*relative) -> Path:
    try:
        return package_share(DESCRIPTION_SHARE, *relative)
    except ResourceNotFound:
        candidate = workspace() / "src" / "tianji" / "tianji_description" / Path(*relative)
        if not candidate.exists():
            raise
        return candidate


from tianji_controller.protocol import DEVICE_READY_FLAGS
from tianji_controller.run_teleop import CommandReceiver, stop_controller

COMMAND_TIMEOUT_NS = 150_000_000
CLOCK_TOLERANCE_NS = 5_000_000
GROUP_SLICES = {"arms": slice(0, 14), "left_hand": slice(14, 34), "right_hand": slice(34, 54)}


def command_is_fresh(frame, now_ns):
    # Match the existing executor's monotonic-clock tolerance and 150 ms age.
    return frame is not None and -CLOCK_TOLERANCE_NS <= now_ns - frame.timestamp_ns <= COMMAND_TIMEOUT_NS


def run_loop(simulation, receiver, controller, viewer, duration, stop_requested, target_overlay=None, recovery_control=None):
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
        if recovery_control is not None and not recovery_control.accept_frame(frame):
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
            "simulation_mode": getattr(simulation, 'mode', 'dynamics'),
            "clock_kind": 'display_only' if getattr(simulation, 'mode', 'dynamics') == 'direct' else 'physics',
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
                if recovery_control is not None:
                    recovery_control.update()
                if target_overlay is not None:
                    target_overlay.update()
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
                if recovery_control is not None:
                    frame=receiver.latest
                    arm_ready=command_is_fresh(frame,time.monotonic_ns()) and recovery_control.accept_frame(frame) and bool(frame.flags & DEVICE_READY_FLAGS['arms'])
                    viewer.set_texts((None,None,
                        'Mapped palm SIM | '+getattr(simulation,'mode','dynamics')+'\n'
                        +'Arms: '+('READY' if arm_ready else 'WAIT / HOLD')+'\n'
                        +'C: calibrate X/Z (hold forward 2s) | S: start\nH: smooth Home, then S | P: hold | R: manual rearm\n'
                        +recovery_control.status,''))
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
    parser.add_argument("--user", help="start/reuse this person's published pico-simple input (DLS/Ceres only)")
    parser.add_argument("--headless", action="store_true", help="run the selected simulation mode without opening a viewer")
    parser.add_argument('--simulation-mode', choices=('dynamics', 'direct'), default='direct',
                        help='direct: joint-state display only (default); dynamics: actuator physics')
    parser.add_argument("--ik-backend", choices=("spark", "mapped-palm", "ceres", "franka-dls"),
                        default="franka-dls", help="default: franka-dls (direct viewer with hand input)")
    parser.add_argument("--mapped-palm-xz-calibration", action="store_true",
                        help="mapped-palm only: viewer C samples X/Z, S starts after success")
    parser.add_argument("--duration", type=float, default=0.0, help="stop after N wall-clock seconds (0: until stopped)")
    parser.add_argument("--pico-port", type=int, default=15000, help="loopback PICO input port (default: 15000)")
    parser.add_argument("--hand-port", type=int, default=16000, help="loopback hand input port (default: 16000)")
    hands = parser.add_mutually_exclusive_group()
    hands.add_argument("--hand-teleop", dest="hand_teleop", action="store_true", default=None,
                       help="DLS/Ceres: receive independently started Manus TJH2 input (default)")
    hands.add_argument("--no-hand-teleop", dest="hand_teleop", action="store_false",
                       help="DLS/Ceres: arms only, do not bind the hand input port")
    parser.add_argument("--config", type=Path,
                        help="controller YAML, also used for the initial arm pose")
    parser.add_argument("--model", type=Path,
                        help="dual-arm and both Hand2 MuJoCo XML")
    parser.add_argument("--sim-allow-pico-jumps", action="store_true",
                        help="simulation only: skip PICO pose jump rejection; retain freshness and motion limits")
    args = parser.parse_args(argv)
    if args.hand_teleop is not None and args.ik_backend not in ("ceres", "franka-dls"):
        parser.error("hand input switches are for DLS/Ceres; legacy backends keep their existing hand input")
    if args.hand_teleop is None:
        args.hand_teleop = True
    if args.hand_teleop and (not 1 <= args.hand_port <= 65535 or args.hand_port == args.pico_port):
        parser.error("hand port must be in [1,65535] and different from PICO port")
    if args.sim_allow_pico_jumps and args.ik_backend not in ("ceres", "franka-dls"):
        parser.error("--sim-allow-pico-jumps requires the DLS/Ceres interactive simulation")
    if args.user and (args.ik_backend not in ("ceres", "franka-dls") or args.pico_port != 15000):
        parser.error("--user requires the DLS/Ceres interactive backend and PICO port 15000")
    if args.ik_backend in ("ceres", "franka-dls"):
        if args.simulation_mode != "direct" or args.mapped_palm_xz_calibration or args.headless:
            parser.error("Franka DLS/Ceres interactive recovery supports the direct viewer; select --ik-backend spark explicitly for legacy headless/dynamics")
        if not math.isfinite(args.duration) or args.duration < 0 or not 1 <= args.pico_port <= 65535:
            parser.error("invalid duration or PICO port")
        from .ceres_session import launch
        return launch(args)
    mapped = args.ik_backend == "mapped-palm"
    if args.mapped_palm_xz_calibration and (not mapped or args.headless):
        parser.error("X/Z calibration requires the mapped-palm viewer")
    args.config = args.config or (
        _description("config", "deployment.yaml") if mapped
        else controller_profile("qp_ik_pico_teleop.yaml"))
    args.model = args.model or (
        _description("mapped_palm", "assets", "mapped_palm", "marvin_m6_wuji2.xml") if mapped
        else _description("models", "marvin_m6_wuji2.xml"))
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
            from .physics import PhysicsSimulation

            simulation_type = PhysicsSimulation
            if args.simulation_mode == 'direct':
                from .direct_state import DirectStateSimulation
                simulation_type = DirectStateSimulation
            simulation = simulation_type(model_path, config_path)
            target_overlay = None
            if mapped:
                from .mapped_overlay import TargetOverlay
                target_overlay = TargetOverlay(simulation)
                cleanup.callback(target_overlay.close)
            if stop_requested.is_set():
                return 0
            # Native binaries are resolved through the same authority the
            # executor uses, so both paths fail the same way when unbuilt.
            try:
                executable = native_executable(
                    "mapped_palm_tjrc_controller" if mapped else "tianji_qp_ik_viewer")
            except ResourceNotFound as error:
                raise RuntimeError(
                    f"build the arm controller before starting --sim: {error}") from error
            receiver = CommandReceiver(0)
            cleanup.callback(receiver.close)
            if receiver.port in (17000, args.pico_port, args.hand_port):
                raise RuntimeError("allocated command output port conflicts with a reserved input or hardware port")
            command = [
                str(executable),
                "--config", str(config_path), "--model", str(model_path),
                "--headless", "--continuous", "--model-state-only",
                "--pico-teleop", "--pico-bind", "127.0.0.1", "--pico-port", str(args.pico_port),
                "--hand-teleop", "--hand-bind", "127.0.0.1", "--hand-port", str(args.hand_port),
                "--joint-command-host", "127.0.0.1", "--joint-command-port", str(receiver.port),
            ]
            print(f"SIM: mode={args.simulation_mode}; no SDK, device connection, or hardware commands. "
                  "Stale groups hold targets; simulation parameters are not hardware calibration.", flush=True)
            if args.simulation_mode == 'direct':
                print('SIM: direct joint-state display; no dynamics/contact response/gravity sag. Not hardware tracking validation.', flush=True)
            print(f"SIM: command receiver 127.0.0.1:{receiver.port}; "
                  f"PICO={args.pico_port}, hands={args.hand_port}", flush=True)
            if args.mapped_palm_xz_calibration:
                command.append("--mapped-palm-xz-calibration")
            if target_overlay is not None:
                command.extend(["--target-overlay-port", str(target_overlay.port)])
            if mapped:
                command.append('--simulation-recovery')
            controller = subprocess.Popen(command, cwd=str(workspace()), start_new_session=True,
                                          stdout=subprocess.PIPE if mapped else None,
                                          stdin=subprocess.PIPE if mapped else subprocess.DEVNULL)
            if controller.stdout is not None:
                os.set_blocking(controller.stdout.fileno(), False)
                cleanup.callback(controller.stdout.close)
            if controller.stdin is not None:
                os.set_blocking(controller.stdin.fileno(), False)
                cleanup.callback(controller.stdin.close)
            cleanup.callback(stop_controller, controller)
            recovery_control = None
            if mapped:
                from .mapped_recovery import MappedRecovery
                recovery_control = MappedRecovery(simulation, controller)
            viewer = None
            if not args.headless and not stop_requested.is_set():
                import mujoco.viewer

                viewer = mujoco.viewer.launch_passive(
                    simulation.model, simulation.data, show_left_ui=False, show_right_ui=False,
                    key_callback=recovery_control.key if recovery_control is not None else None,
                )
                cleanup.callback(viewer.close)
                with viewer.lock():
                    viewer.cam.lookat[:] = (0.0, 0.0, 0.85)
                    viewer.cam.distance = 2.5
                    viewer.cam.azimuth = 135.0
                    viewer.cam.elevation = -15.0
                print(f"SIM_READY: viewer opened; mode={args.simulation_mode}", flush=True)
            elif args.headless:
                print(f"SIM_READY: headless MuJoCo; mode={args.simulation_mode}", flush=True)
            run_loop(simulation, receiver, controller, viewer, args.duration, stop_requested, target_overlay, recovery_control)
    except Exception as error:
        print(f"SIM ERROR: {error}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
