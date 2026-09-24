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
import re
import subprocess
import sys
import tempfile
import time
import unicodedata
import uuid

import yaml

from tianji_runtime import controller_profile, native_executable, package_share, workspace
from tianji_runtime.resources import ResourceNotFound, controller_resource
from tianji_runtime.resources import config_path as workspace_config

from tianji_description.home_config import load_controller_posture, load_home
from .protocol import DEVICE_READY_FLAGS
from .ros_commands import CommandReceiver
from .run_log import SessionLog
from .safety import MotionGate, SafetyFault
from .staged_motion import StagedMotionGate
from .viewer import RealRobotViewer
from .feedback import FeedbackHub
from .observer import ExecutorObserver
from .collection_supervisor import CollectionSupervisor

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
    "TELEOP": "TELEOP | 实时遥操 | 保持输入与机械臂静止后 Enter: 回 HOME | Ctrl+C: 直接停止",
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
    """Poll the operator terminal without buffering input or blocking command reception."""
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




def resolve(base, value):
    path = Path(value)
    return path if path.is_absolute() else (base / path).resolve()


def description_resource(base, value):
    """Use installed description assets; preserve explicit custom resource paths."""
    if not isinstance(value, str) or not value.strip():
        raise ValueError("description resource must be a nonempty path")
    value = value.strip()
    path = Path(value).expanduser()
    prefix = "src/teleop_outputs/tianji/tianji_description/"
    if not path.is_absolute() and value.startswith(prefix):
        path = package_share("tianji_description", value[len(prefix):])
    else:
        path = resolve(base, path)
    if not path.is_file():
        raise ValueError(f"description resource does not exist: {path}")
    return path


def executor_profile(config, base):
    """Validate the DLS-only contract before devices open and rebase runtime resources.

    Shared-root activation is the sole backend override. Invalid algorithms,
    smoothing modes and physical/model control modes are rejected, not repaired.
    The native loader remains responsible for full model/artifact compatibility.
    """
    original = resolve(base, config["controller_config"])
    try:
        data = yaml.safe_load(original.read_text())
    except yaml.YAMLError as error:
        raise ValueError(f"invalid controller configuration: {original}: {error}") from error
    required = {
        "controller": {"model_state_only": True},
        "ik": {"algorithm": "pico_ee_franka_dls"},
        "pico_ee_franka_dls": {"enabled": True},
    }
    if not isinstance(data, dict):
        raise ValueError("controller configuration must be a YAML mapping")
    if "control" in data:
        raise ValueError("retired control-level selection is not supported by the DLS-only core")
    for section, fields in required.items():
        values = data.get(section)
        if not isinstance(values, dict):
            raise ValueError(f"franka-dls executor requires {section} settings")
        for field, expected in fields.items():
            actual = values.get(field)
            if type(actual) is not type(expected) or actual != expected:
                raise ValueError(f"franka-dls executor requires {section}.{field}={expected}")
    shared = data.get("shared_root")
    if not isinstance(shared, dict) or type(shared.get("enabled")) is not bool:
        raise ValueError("franka-dls executor requires shared-root settings")
    smoothing = data["pico_ee_franka_dls"].get("post_smoothing")
    if not isinstance(smoothing, dict) or smoothing.get("mode") != "ruckig":
        raise ValueError("franka-dls executor requires post_smoothing.mode=ruckig")
    for field in ("max_velocity_rad_s", "max_acceleration_rad_s2", "max_jerk_rad_s3"):
        values = smoothing.get(field)
        if (not isinstance(values, list) or len(values) != 7
                or any(type(value) not in (int, float) or not math.isfinite(value) or value <= 0
                       for value in values)):
            raise ValueError(f"post_smoothing.{field} must contain seven positive finite limits")
    tolerance = smoothing.get("validation_tolerance", 1e-8)
    if type(tolerance) not in (int, float) or not math.isfinite(tolerance) or tolerance <= 0:
        raise ValueError("post_smoothing.validation_tolerance must be positive and finite")
    limits = data.get("joint_limits")
    scale = limits.get("velocity_scale", 1.0) if isinstance(limits, dict) else None
    if type(scale) not in (int, float) or not math.isfinite(scale) or not 0 < scale <= 1:
        raise ValueError("joint_limits.velocity_scale must be in (0, 1]")
    arms = config["safety"]["arms"]
    for field in ("lower_rad", "upper_rad"):
        values = arms.get(field)
        if (not isinstance(values, list) or len(values) != 14
                or any(type(value) not in (int, float) or not math.isfinite(value)
                       for value in values)):
            raise ValueError(f"safety.arms.{field} must contain fourteen finite numeric limits")
    if any(lower >= upper for lower, upper in zip(arms["lower_rad"], arms["upper_rad"])):
        raise ValueError("safety.arms: lower limits must be strictly below upper limits")
    # The solve/reference domain must honor the execution gate, never a wider
    # source-profile envelope. Only this runtime copy is changed.
    limits["position_lower_rad"] = list(arms["lower_rad"])
    limits["position_upper_rad"] = list(arms["upper_rad"])
    references = [
        (data["controller"], "pico_ee_dls_kinematics_urdf_path"),
        (shared, "input_contract_artifact"),
        (shared, "robot_geometry_artifact"),
    ]
    if "home_config" in data["controller"]:
        references.append((data["controller"], "home_config"))
    for section, field in references:
        value = section.get(field)
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"franka-dls executor requires a nonempty {field}")
        resource = controller_resource(original, value)
        if not resource.is_file():
            raise ValueError(f"controller resource {field} does not exist: {resource}")
        section[field] = str(resource)
    # Reuse the shared Home/seed contract rather than inventing a second parser.
    load_controller_posture(original)
    shared["enabled"] = True
    return data


def load_configuration(path, device_selection=None, *, hand_source="manus", home_config=None):
    """Load and fully validate the executor configuration.

    Built-in description assets and controller profiles use the installed
    resource authority. Explicit custom paths remain workspace-relative (never
    current-working-directory-relative), or absolute when supplied as such.
    """
    base = workspace()
    config = json.loads(path.read_text())
    devices = tuple(config["active_devices"]) if device_selection is None else DEVICE_SELECTIONS[device_selection]
    if hand_source not in ("manus", "exoskeleton"):
        raise ValueError("hand_source must be manus or exoskeleton")
    hands = tuple(device for device in devices if device != "arms")
    needs_controller = "arms" in devices or (hand_source == "exoskeleton" and bool(hands))
    # Bare names select installed profiles; explicit paths are never replaced
    # by the default profile, even when their contents are invalid.
    profile = config.get("controller_config")
    if not isinstance(profile, str) or not profile.strip():
        raise ValueError("controller_config must be a nonempty profile name or path")
    profile = profile.strip()
    profile_path = Path(profile).expanduser()
    config["controller_config"] = str(
        resolve(base, profile_path) if profile_path.is_absolute() or "/" in profile
        else controller_profile(profile))
    config["controller_model"] = str(description_resource(base, config["controller_model"]))
    if needs_controller:
        executor_profile(config, base)
    staged = config.get("staged_motion")
    if home_config is not None:
        if "arms" not in devices:
            raise ValueError("alternate HOME requires arms in the selected devices")
        if not isinstance(staged, dict):
            raise ValueError("alternate HOME requires staged_motion configuration")
        staged["home_config"] = str(home_config)
    if isinstance(staged, dict) and "home_config" in staged:
        home_path = staged.pop("home_config")
        if not isinstance(home_path, str) or not home_path.strip():
            raise ValueError("staged_motion.home_config must be a nonempty path")
        left, right = load_home(description_resource(base, home_path))
        staged["home_left_rad"] = list(left)
        staged["home_right_rad"] = list(right)
    # Validate all output bounds and HOME before loading SDKs.
    if "arms" in devices:
        StagedMotionGate(config["safety"], devices, config["staged_motion"])
    else:
        MotionGate(config["safety"], devices)
    if hand_source == "exoskeleton" and hands:
        if type(config["hand_port"]) is not int or not 1 <= config["hand_port"] <= 65535:
            raise ValueError("hand_port must be an integer in [1, 65535]")
    topics = {}
    if needs_controller:
        topics["joint_command_topic"] = config["joint_command_topic"]
    if "arms" in devices:
        topics["pico_input_topic"] = config["pico_input_topic"]
    if hand_source == "manus" and hands:
        hand_topics = config.get("hand_command_topics")
        if not isinstance(hand_topics, dict):
            raise ValueError("hand_command_topics must map selected hands to ROS topics")
        for hand in hands:
            topics[f"hand_command_topics.{hand}"] = hand_topics.get(hand)
    for field, topic in topics.items():
        if not isinstance(topic, str) or re.fullmatch(r"(?:/[A-Za-z_][A-Za-z_0-9]*)+", topic) is None:
            raise ValueError(f"{field} must be an absolute nonempty ROS topic with valid names")
    if len(set(topics.values())) != len(topics):
        raise ValueError("selected input and command topics must be distinct")
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
        libraries.add(resolve(base, settings["sdk_library"]).resolve())
    if len(libraries) > 1:
        raise ValueError("both hands must use the same pinned Wuji SDK library")
    return config, devices


def make_hardware(config, devices, base):
    # Deliberately unreachable from the default dry-run path.
    from tianji_controller.hardware import MarvinDevice
    from wuji_controller.hardware import Hand2Device
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
    data = executor_profile(config, base)
    data["controller"]["rate_hz"] = config["safety"]["rate_hz"]
    # Constrain the validated DLS/Ruckig profile without changing its algorithm.
    data["joint_limits"]["velocity_scale"] = min(
        data["joint_limits"].get("velocity_scale", 1.0), config["controller_velocity_scale"])
    # DLS exports this Ruckig reference, not the generic joint-limit integrator.
    # Apply the real-mode speed cap to the limiter that actually drives TJRC.
    smoothing = data["pico_ee_franka_dls"]["post_smoothing"]
    smoothing["max_velocity_rad_s"] = [
        value * data["joint_limits"]["velocity_scale"]
        for value in smoothing["max_velocity_rad_s"]]
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
    parser.add_argument("--config", type=Path, default=workspace_config("robot.json"))
    parser.add_argument("--ik-backend", choices=("franka-dls",), default="franka-dls")
    parser.add_argument("--devices", choices=tuple(DEVICE_SELECTIONS),
                        help="default from config; hands=left+right, all=arms+left+right")
    parser.add_argument("--hand-source", choices=("manus", "exoskeleton"), default="manus",
                        help="Manus ROS hand commands (default), or explicit legacy TJH2 exoskeleton input")
    parser.add_argument("-L", "--L", action="store_true",
                        help="use home-L for startup Home checks and all return-Home motion")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--inspect", action="store_true", help="read hardware feedback/identity only, then exit")
    mode.add_argument("--confirm-real", action="store_true", help="allow real enable after typed confirmation and preflight")
    parser.add_argument("--duration", type=float, default=0, help="stop after N seconds (0: until Ctrl+C)")
    parser.add_argument("--collection", action="store_true",
                        help="require independent camera_views and manage the collector before SDK connection")
    parser.add_argument("--dataset", type=Path, help="record operator-triggered schema-v1 observation episodes here")
    parser.add_argument("--task", help="operator task label for the recorded episode")
    parser.add_argument("--collection-config", type=Path,
                    default=workspace_config("collect_real.json"))
    args = parser.parse_args(argv)
    if args.collection and args.dataset is None:
        from tianji_runtime import dataset_dir
        args.dataset = dataset_dir()
    args.collection = args.collection or args.dataset is not None
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("--duration must be finite and non-negative")
    if args.confirm_real and not sys.stdin.isatty():
        parser.error("real mode requires an operator terminal; no hardware activity performed")
    configuration_path = args.config.resolve()
    base = workspace()
    try:
        home_config = package_share("tianji_description", "config", "home-L.yaml") if args.L else None
        config, devices = load_configuration(configuration_path, args.devices,
                                             hand_source=args.hand_source, home_config=home_config)
    except (OSError, KeyError, TypeError, ValueError, SafetyFault, ResourceNotFound, yaml.YAMLError) as error:
        parser.error(str(error))
    if args.dataset is not None and (not args.confirm_real or set(devices) != {"arms", "left_hand", "right_hand"}):
        parser.error("dataset collection requires --confirm-real and all arms/hands")
    if args.dataset is not None and (not args.task or not args.task.strip()):
        parser.error("dataset collection requires --task")
    if args.task is not None and args.dataset is None:
        parser.error("--task requires --dataset")
    hand_topics = ({name: config["hand_command_topics"][name] for name in devices if name != "arms"}
                   if args.hand_source == "manus" and any(name != "arms" for name in devices) else None)
    needs_controller = "arms" in devices or args.hand_source == "exoskeleton"
    run_mode = "inspect" if args.inspect else ("real" if args.confirm_real else "dry-run")
    try:
        session_log = SessionLog.from_environment(
            devices=devices, mode=run_mode, config_source=str(configuration_path), config=config)
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
    observer = None
    sampler = None
    keyboard = None
    episodes = None
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
            # Independent Manus hands do not require the arm controller or PICO.
            if needs_controller:
                try:
                    viewer = native_executable("tianji_arm_ros")
                except ResourceNotFound as error:
                    raise RuntimeError(
                        f"build the current arm controller before starting the real executor: {error}"
                    ) from error
            receiver = CommandReceiver(config.get("joint_command_topic"), config["safety"], devices,
                                       hand_topics=hand_topics)
        # Rendering failures also precede sensor workers and device sessions.
        if staged:
            # Fail before connecting devices if the required monitor cannot open.
            real_viewer = RealRobotViewer(resolve(base, config["controller_model"]))
        if not args.inspect:
            observer = ExecutorObserver("real" if args.confirm_real else "dry_run",
                                        collection=args.collection)
            observer.start()
        if args.collection:
            collection = CollectionSupervisor(
                observer, args.dataset, args.task, args.collection_config,
                resolve(base, config["controller_model"]))
            collection.start()
        if args.inspect or args.confirm_real:
            hardware = make_hardware(config, devices, base)
            if not args.inspect:
                from tianji_runtime import LockedDevice
                hardware = {name: LockedDevice(device) for name, device in hardware.items()}
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
        if hardware:
            sampler = FeedbackHub(hardware, devices)
            observer.attach_sampler(sampler)
            sampler.start()
        if collection is not None:
            collection.wait_inputs_ready()

        gate = (StagedMotionGate(config["safety"], devices, config["staged_motion"])
                if staged else MotionGate(config["safety"], devices))
        observer.update_state(log_phase(gate, "WAITING"))
        if collection is not None:
            from tianji_runtime import OperatorKeyboard
            from .collection_episode import CollectionEpisodes
            episodes = CollectionEpisodes(
                gate, observer, notify=lambda text: print(text, flush=True))
            keyboard = OperatorKeyboard(episodes.on_key, on_finish=lambda: episodes.on_key("q"))
            enter_pressed = keyboard.poll_enter
            print("采集模式：机器人实测 Home 后，首次 r 使能并缓慢张开双手；请确认空手。\n"
                  "r：冻结当前双臂和双手目标虚影，缓慢靠近；到位停稳、录制确认后开始跟随。\n"
                  "s/d：按键时刻截止保存/丢弃数据，立即停止跟随并限速制动。\n"
                  "实测停稳、数据操作确认后自动缓慢回 Home、张手；无需人先靠近 Home。\n"
                  "q：结束本场（录制中先保存并按相同流程回位）。脚踏单次按键，不加回车。",
                  flush=True)
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
            preview = episodes is not None and episodes.state in ("WAITING", "HOME_READY")
            target = (gate.display_targets if gate.armed and not preview else {
                name: packet.positions(name) for name in devices
                if packet is not None and packet.flags & DEVICE_READY_FLAGS[name]
                and -5_000_000 <= now_ns - packet.timestamp_ns
                <= config["safety"]["command_timeout_s"] * 1e9
            })
            real_viewer.publish(actual, target, "PREVIEW" if preview else gate.phase)
            next_visual = now_ns / 1e9 + 1.0 / 30.0

        if staged:
            update_monitor(None, measured, force=True)
            if episodes is not None:
                print("靠近目标、结束后的制动、回 Home 和张手不进入数据。\n"
                      "r 前请确认目标和运动空间安全；慢速对齐不保证路径无碰撞。\n"
                      "使能后 Enter、Ctrl+C 或关闭监视窗口：立即停止，不自动 Home。\n"
                      "s/d 不是急停；请保持物理急停可及。", flush=True)
            else:
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
            if needs_controller:
                controller_config = controller_configuration(
                    config, base, measured, Path(temporary) / "controller.yaml")
                if session_log is not None:
                    session_log.persist_controller_configuration(
                        controller_config, source=str(resolve(base, config["controller_config"])))
                # The legacy hand-only core must not subscribe to operator arm input.
                pico_topic = (config["pico_input_topic"] if "arms" in devices
                              else f"/tianji/unused_pico/session_{uuid.uuid4().hex}")
                command = [str(viewer), "--franka-dls-executor",
                           "--config", str(controller_config), "--model", str(resolve(base, config["controller_model"])),
                           "--headless", "--continuous", "--pico-teleop",
                           "--pico-topic", pico_topic,
                           "--joint-target-topic", config["joint_command_topic"]]
                if args.hand_source == "exoskeleton" and any(device != "arms" for device in devices):
                    command += ["--hand-teleop", "--hand-source", "exoskeleton", "--hand-bind", "127.0.0.1",
                                "--hand-port", str(config["hand_port"])]
                else:
                    command.append("--no-hand-teleop")
                controller = subprocess.Popen(command, cwd=str(workspace()), start_new_session=True,
                                              stdin=subprocess.DEVNULL)
            started = time.monotonic()
            next_report = started
            last_reason = "waiting for selected command sources"
            period_s = 1.0 / config["safety"]["rate_hz"]
            while not stop_requested:
                cycle_deadline = time.monotonic() + period_s
                observer.update_state(log_phase(gate, "TELEOP" if gate.armed else "WAITING"),
                                      detail=episodes.detail if episodes is not None else "")
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
                if controller is not None and controller.poll() is not None:
                    raise SafetyFault(f"control process exited ({controller.returncode})")
                packet = receiver.drain(
                    (lambda frame: gate.observe_source(frame, time.monotonic_ns())) if gate.armed else None)
                if packet is None:
                    if now - started > config["startup_timeout_s"]:
                        raise SafetyFault("selected sources did not publish joint commands")
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
                        if episodes is not None:
                            gate.check_home_enable_ready(packet, measured, now_ns)
                        else:
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
                        status.update("WAITING | 实测已在 Home，未使能 | 确认空手后 r：使能并缓慢张手"
                                      if episodes is not None else _STAGED_STATUS["WAITING"])
                        confirmation_prompted = True
                    elif not confirmation_prompted:
                        print("Physical emergency stop must be reachable.\n"
                              f"Enable sequence: {enable_sequence(config, devices)}.\n"
                              f"Press ENTER to enable {', '.join(devices)}; press ENTER again to stop and disable:",
                              flush=True)
                        confirmation_prompted = True
                    pressed = enter_pressed()
                    if episodes is not None:
                        pressed = episodes.tick(packet, measured, now_ns)
                        if episodes.done:
                            final_reason = "operator finished before enable"
                            break
                    if not pressed:
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
                    if collection is not None:
                        collection.check_before_enable()
                    if episodes is not None:
                        gate.arm_home(packet, measured, now_ns)
                        episodes.enabled()
                    else:
                        gate.arm(packet, measured, now_ns)
                    observer.update_state(log_phase(gate, "TELEOP"),
                                          detail=episodes.detail if episodes is not None else "")
                    for name, device in hardware.items():
                        check_source()
                        device.enable(guard=check_source)
                        enabled_devices.add(name)
                        check_source()
                    if staged:
                        status.update(f"EPISODE {episodes.state} | {episodes.detail}"
                                      if episodes is not None else _STAGED_STATUS[gate.phase])
                        update_monitor(packet, measured, force=True)
                        # SDK enable is a guarded startup transaction, not a
                        # motion period. Start its clock only after all devices
                        # enabled and the post-enable snapshot was validated.
                        measured = {name: device.read_feedback() for name, device in hardware.items()}
                        packet = receiver.drain(
                            lambda frame: gate.observe_source(frame, time.monotonic_ns()))
                        if packet is None:
                            raise SafetyFault("controller source disappeared after enable")
                        gate.complete_enable(packet, measured, time.monotonic_ns())
                        cycle_deadline = time.monotonic() + period_s
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
                            if episodes is not None:
                                final_reason = "operator interrupted collection; stopping without Home"
                                break
                            if gate.phase == "READY":
                                gate.start_teleop(packet, measured, time.monotonic_ns())
                                observer.update_state(gate.phase)
                            elif gate.phase == "TELEOP":
                                if collection is not None:
                                    observer.abort()
                                gate.start_homing(packet, measured, time.monotonic_ns())
                                observer.update_state(gate.phase)
                            else:
                                status.update("STOPPING")
                                final_reason = f"operator stopped during {gate.phase}"
                                break
                        if episodes is not None:
                            episodes.tick(packet, measured, time.monotonic_ns())
                    now_ns = time.monotonic_ns()
                    if session_log is not None:
                        # Sampled before the step, so a frame that fails a gate
                        # check stays in the ring with the exact state it saw.
                        session_log.sample(packet, measured, log_phase(gate, "LIVE"), now_ns)
                    outputs = gate.step(packet, measured, now_ns)
                    observer.update_state(log_phase(gate, "TELEOP"),
                                          detail=episodes.detail if episodes is not None else "")
                    if staged:
                        status.update(
                            f"EPISODE {episodes.state} | {episodes.detail.replace(chr(10), ' | ')}"
                            if episodes is not None else _STAGED_STATUS[gate.phase])
                        update_monitor(packet, measured)
                        if episodes is not None and episodes.done:
                            final_reason = "collection run completed at Home; disabling"
                            break
                        if episodes is None and gate.phase == "HOME_REACHED":
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
        if observer is not None:
            observer.update_state("STOPPED", faulted=result != 0 or pending is not None,
                                  detail=reason or final_reason or "executor stopping")
            if collection is not None:
                observer.abort()
        if sampler is not None:
            sampler.request_stop()
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
        if sampler is not None:
            try:
                sampler.close()
            except Exception as error:
                cleanup_errors.append(f"feedback sampler cleanup failed: {error}")
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
        if observer is not None:
            try:
                observer.close()
            except Exception as error:
                cleanup_errors.append(f"executor DDS cleanup failed: {error}")
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
