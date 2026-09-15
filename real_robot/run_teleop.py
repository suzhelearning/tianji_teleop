#!/usr/bin/env python3
"""Run the existing controller with an explicit, guarded hardware output boundary.

Default is DRY RUN: no vendor SDK is loaded and no device is contacted.
--inspect performs feedback/identity reads only. --confirm-real additionally
requires an operator terminal, fresh sources, healthy feedback and Enter confirmation.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import select
import signal
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import unicodedata

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent))
from real_robot.protocol import DEVICE_READY_FLAGS, decode_packet
from real_robot.run_log import SessionLog
from real_robot.safety import MotionGate, SafetyFault
from real_robot.staged_motion import StagedMotionGate
from real_robot.viewer import RealRobotViewer

DEVICE_SELECTIONS = {
    "arms": ("arms",),
    "left_hand": ("left_hand",),
    "right_hand": ("right_hand",),
    "hands": ("left_hand", "right_hand"),
    "all": ("arms", "left_hand", "right_hand"),
}

_STAGED_STATUS = {
    "WAITING": "WAITING | 未使能 | Enter: 锁定目标并慢速对齐",
    "ALIGNING": "ALIGNING | 慢速对齐 | Enter: 中止并失能",
    "READY": "READY | 已对齐并保持 | Enter: 开始实时遥操",
    "TELEOP": "TELEOP | 实时遥操 | Enter: 回 HOME | Ctrl+C: 直接停止",
    "HOMING": "HOMING | 双臂回位、双手保持 | Enter: 中止并失能",
    "HOME_REACHED": "HOME_REACHED | 已回位，正在失能",
}


class StatusLine:
    """One terminal row; nonterminal consumers receive state changes as lines."""

    def __init__(self):
        self.stream = sys.stdout
        self.terminal = self.stream.isatty()
        self.last = None
        self.active = False

    def update(self, text):
        text = " ".join(str(text).splitlines())
        if text == self.last:
            return
        self.last = text
        if self.terminal:
            width = max(1, shutil.get_terminal_size((120, 24)).columns - 1)
            visible = []
            used = 0
            for character in text:
                cells = 0 if unicodedata.combining(character) else (
                    2 if unicodedata.east_asian_width(character) in ("W", "F") else 1)
                if used + cells > width:
                    break
                visible.append(character)
                used += cells
            self.stream.write("\r\x1b[2K" + "".join(visible))
            self.stream.flush()
            self.active = True
        else:
            print(text, file=self.stream, flush=True)

    def finish(self):
        if self.active:
            self.stream.write("\n")
            self.stream.flush()
            self.active = False
        self.last = None


def poll_enter():
    """Poll the operator terminal without buffering input or blocking UDP reception."""
    fd = sys.stdin.fileno()
    for _ in range(256):
        if not select.select([fd], [], [], 0)[0]:
            return False
        character = os.read(fd, 1)
        if not character:
            raise SafetyFault("operator terminal closed")
        if character == b"\n":
            return True
    return False


class CommandReceiver:
    def __init__(self, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # Do not reuse ports: a second executor must fail, not split commands.
            self.socket.bind(("127.0.0.1", port))
            self.socket.setblocking(False)
        except BaseException:
            self.socket.close()
            raise
        self.latest = None
        self.count = 0

    @property
    def port(self):
        return self.socket.getsockname()[1]

    def drain(self, validate=None):
        for _ in range(512):
            try:
                data, sender = self.socket.recvfrom(4096)
            except BlockingIOError:
                break
            if sender[0] != "127.0.0.1":
                raise SafetyFault("non-loopback command source")
            frame = decode_packet(data)
            if self.latest and (frame.sequence <= self.latest.sequence or
                                frame.timestamp_ns < self.latest.timestamp_ns):
                raise SafetyFault("controller command sequence or timestamp regressed")
            if validate is not None:
                validate(frame)
            self.latest = frame
            self.count += 1
        return self.latest

    def close(self):
        self.socket.close()


def resolve(base, value):
    path = Path(value)
    return path if path.is_absolute() else (base / path).resolve()


def load_configuration(path, device_selection=None):
    config = json.loads(path.read_text())
    devices = tuple(config["active_devices"]) if device_selection is None else DEVICE_SELECTIONS[device_selection]
    # Validate all output bounds and HOME before loading SDKs.
    if "arms" in devices:
        StagedMotionGate(config["safety"], devices, config["staged_motion"])
    else:
        MotionGate(config["safety"], devices)
    for field in ("command_port", "pico_port", "hand_port"):
        if type(config[field]) is not int or not 1 <= config[field] <= 65535:
            raise ValueError(f"{field} must be an integer in [1, 65535]")
    if len({config[key] for key in ("command_port", "pico_port", "hand_port")}) != 3:
        raise ValueError("command/PICO/hand ports must be distinct")
    if not math.isfinite(config["startup_timeout_s"]) or config["startup_timeout_s"] <= 0:
        raise ValueError("startup_timeout_s must be positive and finite")
    if not 0 < config["controller_velocity_scale"] <= 1:
        raise ValueError("controller_velocity_scale must be in (0, 1]")
    serials = set()
    libraries = set()
    for device in devices:
        if device == "arms":
            continue
        settings = config[device]
        serial = settings.get("serial")
        if not isinstance(serial, str) or not serial.strip():
            raise ValueError(f"configure the exact {device} serial before hardware use")
        identity = serial.strip().casefold()
        if identity in serials:
            raise ValueError("left and right hands must have distinct device serials")
        serials.add(identity)
        libraries.add(resolve(path.parent, settings["sdk_library"]).resolve())
    if len(libraries) > 1:
        raise ValueError("both hands must use the same pinned Wuji SDK library")
    return config, devices


def make_hardware(config, devices, base):
    # Deliberately unreachable from the default dry-run path.
    from real_robot.hardware import MarvinDevice, Hand2Device
    result = {}
    if "arms" in devices:
        settings = config["arms"]
        result["arms"] = MarvinDevice(
            resolve(base, settings["sdk_directory"]), settings["ip"],
            velocity_ratio=settings["velocity_ratio"], acceleration_ratio=settings["acceleration_ratio"])
    for device in ("left_hand", "right_hand"):
        if device not in devices:
            continue
        settings = config[device]
        result[device] = Hand2Device(
            resolve(base, settings["sdk_library"]), settings["serial"].strip(),
            side=device.split("_", 1)[0],
            kp=settings["kp"], kd=settings["kd"], effort_limit_amps=settings["effort_limit_amps"])
    return result


def fresh_feedback(hardware, timeout_s, feedback_timeout_s):
    deadline = time.monotonic() + timeout_s
    reason = "no feedback"
    while time.monotonic() < deadline:
        measured = {}
        try:
            for name, device in hardware.items():
                value = device.read_feedback()
                age = time.monotonic_ns() - value.received_monotonic_ns
                if not value.healthy or value.received_monotonic_ns <= 0 or not 0 <= age <= feedback_timeout_s * 1e9:
                    raise SafetyFault(f"{name}: {value.detail or 'feedback unavailable/stale'}")
                measured[name] = value
            return measured
        except RuntimeError as error:
            reason = str(error)
            time.sleep(.01)
    raise SafetyFault(f"feedback preflight timed out: {reason}")


def controller_configuration(config, base, feedback, destination):
    import yaml
    original = resolve(base, config["controller_config"])
    data = yaml.safe_load(original.read_text())
    data["controller"]["rate_hz"] = config["safety"]["rate_hz"]
    data["controller"]["model_state_only"] = True
    # Preserve the existing IK/control algorithm; constrain its real-mode speed.
    data["joint_limits"]["velocity_scale"] = min(
        data["joint_limits"].get("velocity_scale", 1.0), config["controller_velocity_scale"])
    if "arms" in feedback:
        positions = feedback["arms"].position_rad
        data["controller"]["initial_posture_enabled"] = True
        data["controller"]["initial_left_q_rad"] = list(positions[:7])
        data["controller"]["initial_right_q_rad"] = list(positions[7:])
    destination.write_text(yaml.safe_dump(data, sort_keys=False))
    return destination


def enable_sequence(config, devices):
    """Human-readable arming sequence for the operator prompt."""
    parts = []
    for name in devices:
        ramp = config["safety"][name].get("enable_zero_ramp_seconds")
        parts.append(f"{name} returns to zero then ramps {ramp:g} s to the current input pose"
                     if ramp else f"{name} follows the input from the measured pose (pose match required)")
    return "; ".join(parts)


def stop_controller(process):
    if process is None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()


def log_phase(gate, fallback):
    """Run-log sample phase; the staged gate owns the operator-visible names."""
    return getattr(gate, "phase", None) or fallback


def error_reason(error):
    """Keep the original exception and its machine-readable fault diagnostics."""
    return f"{type(error).__name__}: {error}", getattr(error, "details", None)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config.json")
    parser.add_argument("--devices", choices=tuple(DEVICE_SELECTIONS),
                        help="default from config; hands=left+right, all=arms+left+right")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--inspect", action="store_true", help="read hardware feedback/identity only, then exit")
    mode.add_argument("--confirm-real", action="store_true", help="allow real enable after typed confirmation and preflight")
    parser.add_argument("--duration", type=float, default=0, help="stop after N seconds (0: until Ctrl+C)")
    parser.add_argument("--dataset", type=Path, help="record operator-triggered schema-v1 observation episodes here")
    parser.add_argument("--task", help="operator task label for the recorded episode")
    parser.add_argument("--collection-config", type=Path, default=ROOT.parent / "collection_config.json")
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("--duration must be finite and non-negative")
    if args.confirm_real and not sys.stdin.isatty():
        parser.error("real mode requires an operator terminal; no hardware activity performed")
    config_path = args.config.resolve()
    try:
        config, devices = load_configuration(config_path, args.devices)
    except (OSError, KeyError, TypeError, ValueError, SafetyFault) as error:
        parser.error(str(error))
    if args.dataset is not None and (not args.confirm_real or set(devices) != {"arms", "left_hand", "right_hand"}):
        parser.error("dataset collection requires --confirm-real and all arms/hands")
    if args.dataset is not None and (not args.task or not args.task.strip()):
        parser.error("dataset collection requires --task")
    if args.task is not None and args.dataset is None:
        parser.error("--task requires --dataset")
    base = config_path.parent
    run_mode = "inspect" if args.inspect else ("real" if args.confirm_real else "dry-run")
    try:
        session_log = SessionLog.from_environment(
            devices=devices, mode=run_mode, config_source=str(config_path), config=config)
    except (OSError, ValueError) as error:
        print(f"RUN LOG REFUSED: {error}; no hardware activity performed", file=sys.stderr, flush=True)
        return 1
    if session_log is not None:
        print(f"RUN LOG: {session_log.directory}", flush=True)
    staged = args.confirm_real and "arms" in devices
    real_viewer = None
    status = StatusLine()
    hardware = {}
    controller = None
    receiver = None
    gate = None
    packet = None
    result = 0
    stop_requested = False
    final_reason = None
    outcome = None
    reason = None
    error_details = None
    cleanup_errors = []
    collection = None
    keyboard = None
    enter_pressed = poll_enter

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True
        raise KeyboardInterrupt

    handlers = {sig: signal.signal(sig, request_stop) for sig in (signal.SIGTERM, signal.SIGINT)}
    try:
        measured = {}
        if not args.inspect:
            # Refuse static startup failures before opening cameras or SDK sessions.
            arm_root = ROOT.parent / "control"
            viewer = arm_root / "build/tianji_qp_ik_viewer"
            if not viewer.is_file():
                raise RuntimeError("build the current arm controller before starting the real executor")
            receiver = CommandReceiver(config["command_port"])
        if args.dataset is not None:
            from data_collection.integration import CollectionSession
            collection = CollectionSession(args.dataset, args.task, args.collection_config,
                                           resolve(base, config["controller_model"]))
        if staged:
            # Fail before connecting devices if the required monitor cannot open.
            real_viewer = RealRobotViewer(resolve(base, config["controller_model"]))
        if args.inspect or args.confirm_real:
            hardware = make_hardware(config, devices, base)
            if collection is not None:
                hardware = collection.wrap_hardware(hardware)
            for name, device in hardware.items():
                print(f"READ-ONLY CONNECT {name}", flush=True)
                device.connect()
            measured = fresh_feedback(hardware, config["startup_timeout_s"], config["safety"]["feedback_timeout_s"])
            if session_log is not None:
                session_log.sample(None, measured, "PREFLIGHT", time.monotonic_ns())
            if args.inspect:
                # Confirm read-only resource release before reporting success.
                for device in reversed(tuple(hardware.values())):
                    device.close()
                hardware.clear()
                print(json.dumps({name: asdict(value) for name, value in measured.items()}, indent=2))
                final_reason = "read-only feedback inspected; no motor commands sent"
                if session_log is not None:
                    # Inspection has already released every device, so persist
                    # here before its early return can hide a logging failure.
                    session_log.finish(outcome="completed", reason=final_reason, result=0)
                    session_log = None
                return 0
            if any(value.enabled for value in measured.values()):
                raise SafetyFault("device already enabled; do not take over another control session")
        else:
            print("DRY RUN: no SDK loaded, no device connection, no motor commands", flush=True)
        if collection is not None:
            collection.start(hardware)

        gate = (StagedMotionGate(config["safety"], devices, config["staged_motion"])
                if staged else MotionGate(config["safety"], devices))
        if collection is not None:
            from data_collection.keyboard import CollectionKeyboard
            keyboard = CollectionKeyboard(lambda key: collection.command(key, gate.phase))
            enter_pressed = keyboard.poll_enter
            print("TELEOP recording keys (no Enter needed): r=start, s=save success, d=discard current. "
                  "Recording keys do not change robot mode.", flush=True)
        enabled_devices = set()
        confirmation_prompted = False
        next_visual = 0.0

        def check_monitor():
            if real_viewer is not None and not real_viewer.is_running():
                raise SafetyFault("visualization closed or failed; stopping without HOME")

        def update_monitor(packet, measured, *, force=False):
            nonlocal next_visual
            if real_viewer is None:
                return
            check_monitor()
            now_ns = time.monotonic_ns()
            if not force and now_ns / 1e9 < next_visual:
                return
            actual = {
                name: tuple(value.position_rad) for name, value in measured.items()
                if value.healthy and 0 <= now_ns - value.received_monotonic_ns
                <= config["safety"]["feedback_timeout_s"] * 1e9
            }
            target = (gate.display_targets if gate.armed else {
                name: packet.positions(name) for name in devices
                if packet is not None and packet.flags & DEVICE_READY_FLAGS[name]
                and -5_000_000 <= now_ns - packet.timestamp_ns
                <= config["safety"]["command_timeout_s"] * 1e9
            })
            real_viewer.publish(actual, target, gate.phase)
            next_visual = now_ns / 1e9 + 1.0 / 30.0

        if staged:
            update_monitor(None, measured, force=True)
            print("Enter 1: slow alignment; READY then Enter 2: teleop; "
                  "Enter 3 during teleop: slow HOME then disable.\n"
                  "Enter during ALIGNING/HOMING, Ctrl+C, or closing the window: "
                  "stop immediately without HOME. Keep the physical emergency stop reachable.",
                  flush=True)

        def check_source():
            check_monitor()
            if enter_pressed():
                raise KeyboardInterrupt
            if args.duration and time.monotonic() - started >= args.duration:
                raise SafetyFault("session duration expired before enable completed")
            current = receiver.drain(lambda frame: gate.observe_source(frame, time.monotonic_ns()))
            if current is None:
                raise SafetyFault("controller source disappeared during enable")
            gate.observe_source(current, time.monotonic_ns())
            for name in enabled_devices:
                gate.check_feedback(name, hardware[name].read_feedback(), time.monotonic_ns(),
                                    require_enabled=True)

        with tempfile.TemporaryDirectory(prefix="tianji-real-control-") as temporary:
            controller_config = controller_configuration(config, base, measured, Path(temporary) / "controller.yaml")
            if session_log is not None:
                session_log.persist_controller_configuration(
                    controller_config, source=str(resolve(base, config["controller_config"])))
            pico_port = config["pico_port"]
            if "arms" not in devices:
                # SPARK keeps its original PICO-enabled control contract, but a
                # hand-only executor neither consumes the user's arm port nor
                # authorizes any arm output.
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserved:
                    reserved.bind(("127.0.0.1", 0))
                    pico_port = reserved.getsockname()[1]
            command = ["pixi", "run", "--manifest-path", str(ROOT.parent / "pixi.toml"), str(viewer),
                       "--config", str(controller_config), "--model", str(resolve(base, config["controller_model"])),
                       "--headless", "--continuous", "--hand-teleop", "--hand-port", str(config["hand_port"]),
                       "--pico-port", str(pico_port), "--joint-command-host", "127.0.0.1",
                       "--joint-command-port", str(receiver.port)]
            command.append("--pico-teleop")
            controller = subprocess.Popen(command, cwd=arm_root, start_new_session=True, stdin=subprocess.DEVNULL)
            started = time.monotonic()
            next_report = started
            last_reason = "waiting for controller packets"
            period_s = 1.0 / config["safety"]["rate_hz"]
            while not stop_requested:
                cycle_deadline = time.monotonic() + period_s
                check_monitor()
                if collection is not None:
                    collection.check()
                if args.confirm_real and gate.armed and not staged and enter_pressed():
                    print("ENTER: stopping and disabling selected devices", flush=True)
                    final_reason = "operator stopped and disabled the selected devices"
                    break
                now = time.monotonic()
                if args.duration and now - started >= args.duration:
                    final_reason = "session duration expired"
                    break
                if controller.poll() is not None:
                    raise SafetyFault(f"control process exited ({controller.returncode})")
                packet = receiver.drain(
                    (lambda frame: gate.observe_source(frame, time.monotonic_ns())) if gate.armed else None)
                if packet is None:
                    if now - started > config["startup_timeout_s"]:
                        raise SafetyFault("controller did not publish joint commands")
                    time.sleep(.005)
                    continue
                if not args.confirm_real:
                    now_ns = time.monotonic_ns()
                    if session_log is not None:
                        session_log.sample(packet, measured, log_phase(gate, "DRY"), now_ns)
                    try:
                        gate.validate_targets(packet, now_ns)
                        last_reason = "target bounds and input freshness valid; hardware feedback NOT tested"
                    except SafetyFault as error:
                        last_reason = str(error)
                    if now >= next_report:
                        print(f"DRY packets={receiver.count} flags={packet.flags} status={last_reason}", flush=True)
                        next_report = now + 1
                elif not gate.armed:
                    measured = {name: device.read_feedback() for name, device in hardware.items()}
                    # Blocking SDK reads can outlive the previously received target.
                    # Check the latest packet and feedback against a post-read clock.
                    packet = receiver.drain()
                    now = time.monotonic()
                    update_monitor(packet, measured)
                    now_ns = time.monotonic_ns()
                    if session_log is not None:
                        session_log.sample(packet, measured, log_phase(gate, "WAITING"), now_ns)
                    try:
                        gate.check_enable_ready(packet, measured, now_ns)
                    except SafetyFault as error:
                        last_reason = str(error)
                        if not confirmation_prompted and now - started > config["startup_timeout_s"]:
                            raise SafetyFault(f"arming preflight timed out: {last_reason}")
                        if now >= next_report:
                            status.update(f"NOT ENABLED: {last_reason}")
                            next_report = now + 0.5
                        time.sleep(.005)
                        continue
                    if staged:
                        status.update(_STAGED_STATUS["WAITING"])
                        confirmation_prompted = True
                    elif not confirmation_prompted:
                        print("Physical emergency stop must be reachable.\n"
                              f"Enable sequence: {enable_sequence(config, devices)}.\n"
                              f"Press ENTER to enable {', '.join(devices)}; press ENTER again to stop and disable:",
                              flush=True)
                        confirmation_prompted = True
                    if not enter_pressed():
                        remaining = cycle_deadline - time.monotonic()
                        if remaining > 0:
                            time.sleep(remaining)
                        continue
                    # Reception stays live while waiting. Recheck the latest frame
                    # and feedback after confirmation, before any motor authority.
                    measured = {name: device.read_feedback() for name, device in hardware.items()}
                    packet = receiver.drain()
                    now_ns = time.monotonic_ns()
                    if session_log is not None:
                        session_log.sample(packet, measured, log_phase(gate, "LIVE"), now_ns)
                    gate.arm(packet, measured, now_ns)
                    for name, device in hardware.items():
                        check_source()
                        device.enable(guard=check_source)
                        enabled_devices.add(name)
                        check_source()
                    if staged:
                        status.update(_STAGED_STATUS["ALIGNING"])
                        update_monitor(packet, measured, force=True)
                    else:
                        print("REAL OUTPUT ARMED: returning to zero, then ramping to the current input pose; "
                              "ENTER stops and disables; stale input, reset or feedback fault also stops this session",
                              flush=True)
                else:
                    measured = {name: device.read_feedback() for name, device in hardware.items()}
                    if staged:
                        # Keep every source fault observable during blocking SDK reads.
                        packet = receiver.drain(
                            lambda frame: gate.observe_source(frame, time.monotonic_ns()))
                        if enter_pressed():
                            if gate.phase == "READY":
                                gate.start_teleop(packet, measured, time.monotonic_ns())
                            elif gate.phase == "TELEOP":
                                if collection is not None:
                                    collection.end_episode()
                                gate.start_homing(packet, measured, time.monotonic_ns())
                            else:
                                status.update("STOPPING")
                                final_reason = f"operator stopped during {gate.phase}"
                                break
                    now_ns = time.monotonic_ns()
                    if session_log is not None:
                        # Sampled before the step, so a frame that fails a gate
                        # check stays in the ring with the exact state it saw.
                        session_log.sample(packet, measured, log_phase(gate, "LIVE"), now_ns)
                    outputs = gate.step(packet, measured, now_ns)
                    if staged:
                        status.update(_STAGED_STATUS[gate.phase])
                        update_monitor(packet, measured)
                        if gate.phase == "HOME_REACHED":
                            final_reason = "staged sequence reached HOME_REACHED"
                            break
                    for name, positions in outputs.items():
                        hardware[name].send(positions)
                        if session_log is not None:
                            # Only a successful send is a command; gate output alone is not.
                            session_log.record_send(name, positions)
                # Work belongs to this period. Overruns start a fresh cycle;
                # never accumulate missed deadlines or burst catch-up commands.
                remaining = cycle_deadline - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
            if not args.confirm_real:
                print(f"DRY COMPLETE packets={receiver.count}; no hardware commands sent", flush=True)
                final_reason = f"dry run ended without hardware output; packets={receiver.count}"
                if receiver.count == 0:
                    result = 1
    except KeyboardInterrupt as error:
        status.finish()
        print("Stopping session", flush=True)
        outcome = "interrupted"
        reason = "stop requested before the session completed"
        for note in getattr(error, "__notes__", ()):
            print(f"HARDWARE CLEANUP WARNING: {note}; verify physical emergency stop", file=sys.stderr, flush=True)
            cleanup_errors.append(f"HARDWARE CLEANUP WARNING: {note}")
            result = 1
    except (OSError, RuntimeError, ValueError, KeyError, EOFError) as error:
        status.finish()
        outcome = "failed"
        reason, error_details = error_reason(error)
        print(f"REAL EXECUTOR STOP: {reason}", file=sys.stderr, flush=True)
        for note in getattr(error, "__notes__", ()):
            print(f"HARDWARE CLEANUP WARNING: {note}; verify physical emergency stop", file=sys.stderr, flush=True)
            cleanup_errors.append(f"HARDWARE CLEANUP WARNING: {note}")
        result = 1
    finally:
        pending = sys.exc_info()[1]
        status.finish()
        if keyboard is not None:
            try:
                keyboard.close()
            except Exception as error:
                print(f"operator terminal restore failed: {error}", file=sys.stderr, flush=True)
                cleanup_errors.append(f"operator terminal restore failed: {error}")
                result = 1
        # Ignore repeated terminal interrupts while releasing owned hardware.
        for sig in handlers:
            signal.signal(sig, signal.SIG_IGN)
        if collection is not None:
            try:
                collection.request_stop()
            except Exception as error:
                print(f"collection stop failed: {error}", file=sys.stderr, flush=True)
                cleanup_errors.append(f"collection stop failed: {error}")
                result = 1
        if session_log is not None:
            # Include partial successful sends and failures outside gate.step.
            # This is still an in-memory append; disk writes follow all cleanup.
            session_log.sample(packet, measured, log_phase(gate, "PREFLIGHT"), time.monotonic_ns())
        # Stop every selected device before any close can join callbacks or wait
        # for disconnect/disable feedback and delay another device's stop request.
        for operation in ("stop", "close"):
            for name, device in reversed(tuple(hardware.items())):
                try:
                    getattr(device, operation)()
                except Exception as error:
                    print(f"{name} {operation} could not be confirmed: {error}; use physical emergency stop", file=sys.stderr)
                    cleanup_errors.append(f"{name} {operation} could not be confirmed: {error}")
                    result = 1
        try:
            stop_controller(controller)
        except Exception as error:
            message = f"controller cleanup failed: {error}"
            print(message, file=sys.stderr, flush=True)
            cleanup_errors.append(message)
            result = 1
        if receiver is not None:
            try:
                receiver.close()
            except Exception as error:
                message = f"command receiver cleanup failed: {error}"
                print(message, file=sys.stderr, flush=True)
                cleanup_errors.append(message)
                result = 1
        if real_viewer is not None:
            try:
                real_viewer.close()
            except Exception as error:
                print(f"visualization cleanup failed: {error}", file=sys.stderr, flush=True)
                cleanup_errors.append(f"visualization cleanup failed: {error}")
                result = 1
        if collection is not None:
            # Motors have been released before camera/manager joins or any
            # still-pending background save can block. Unconfirmed segments stay partial.
            try:
                collection.finish()
            except (Exception, KeyboardInterrupt) as error:
                print(f"DATASET NOT PUBLISHED: {error}", file=sys.stderr, flush=True)
                cleanup_errors.append(f"dataset finalization failed: {error}")
                result = 1
        # Hardware and controller are stopped before the records are written.
        if session_log is not None:
            if outcome is None:
                if pending is not None:
                    outcome = "exception"
                    result = 1
                    reason, error_details = error_reason(pending)
                else:
                    outcome = "failed" if result else "completed"
                    reason = final_reason or ("cleanup failed" if cleanup_errors else "session ended")
            try:
                session_log.finish(outcome=outcome, reason=reason, result=result,
                                   cleanup_errors=cleanup_errors, error_details=error_details)
            except (OSError, TypeError, ValueError) as error:
                print(f"RUN LOG WRITE FAILED: {error}; logs may be incomplete", file=sys.stderr, flush=True)
                result = 1
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
