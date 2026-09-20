"""Guarded replay/Regrind hardware execution; no hardware without explicit admission.

H5 and Regrind localize from live measured poses before enable. Physical Enter
enables only, then a released/new hold approaches frame zero; a new press runs.
Playback Enter toggles pause; completion, q and faults stop without Home.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
from collections import deque
import math
import os
from pathlib import Path
import signal
import sys
import time

import numpy as np

from real_robot.protocol import CommandFrame, DEVICE_READY_FLAGS
from real_robot.run_teleop import fresh_feedback, load_configuration, make_hardware, resolve
from real_robot.safety import MotionGate, SafetyFault, _BOUND_SLACK_RAD
from real_robot.staged_motion import ALIGNING, READY, StagedMotionGate
from real_robot.viewer import RealRobotViewer

from ..data.geometry import compose_pose, invert_pose
from ..replay.mocap_overlay import build_object_overlays
from ..types import TargetFrame

ROOT = Path(__file__).resolve().parents[1]


class RuntimeMotionGate(StagedMotionGate):
    """Freeze staged goals; retain the legacy live-execution pause wrapper."""

    def lock_approach(self, frame, now_ns):
        if not self.staged_stopped:
            raise SafetyFault("new frame0 requires a stopped staged command")
        self.validate_targets(frame, now_ns)
        self._targets = {name: frame.positions(name) for name in self.devices}
        self._enter_phase(ALIGNING, now_ns)

    def synchronize_enabled(self, feedback, now_ns):
        """Resume slew from verified SDK holds after potentially slow enables."""
        self._last_commands = {
            name: tuple(self.check_feedback(name, feedback[name], now_ns, require_enabled=True))
            for name in self.devices
        }
        self._last_step_ns = self._phase_start_ns = now_ns
        self.reset_staged_motion()
        return dict(self._last_commands)

    def step(self, frame, feedback, now_ns):
        if not self.paused or self.phase in (ALIGNING, READY):
            return super().step(frame, feedback, now_ns)
        # Live playback keeps its existing immediate bounded-command hold.
        saved = self._targets
        self._targets = dict(self._last_commands)
        self._phase_start_ns += max(0, now_ns - self._last_step_ns)
        held = command_frame(pack_positions(self._last_commands), frame.sequence, frame.timestamp_ns, self.devices)
        try:
            self.observe_source(frame, now_ns)
            return super().step(held, feedback, now_ns)
        finally:
            self._targets = saved


def pack_positions(feedback_or_commands):
    values = np.zeros(54, dtype=np.float64)
    for name, start, stop in (("arms", 0, 14), ("left_hand", 14, 34), ("right_hand", 34, 54)):
        if name in feedback_or_commands:
            item = feedback_or_commands[name]
            values[start:stop] = item.position_rad if hasattr(item, "position_rad") else item
    return values


def admit_measured_positions(q, lower, upper):
    """Project only admitted encoder margin; never use for recorded commands."""
    values = np.asarray(q, dtype=np.float64)
    lower, upper = np.asarray(lower, dtype=np.float64), np.asarray(upper, dtype=np.float64)
    if (values.ndim != 1 or values.shape != lower.shape or values.shape != upper.shape
            or not np.isfinite(values).all() or not np.isfinite(lower).all()
            or not np.isfinite(upper).all() or np.any(lower >= upper)):
        raise SafetyFault("invalid measured seed or joint limits")
    if np.any(values < lower - _BOUND_SLACK_RAD) or np.any(values > upper + _BOUND_SLACK_RAD):
        raise SafetyFault("measured seed exceeds model or configured safety encoder margin")
    return np.clip(values, lower, upper)


def command_frame(values, sequence, timestamp_ns, devices):
    q = np.asarray(values, dtype=np.float64)
    if q.shape != (54,) or not np.isfinite(q).all():
        raise SafetyFault("adapter must produce 54 finite joint positions")
    return CommandFrame(sequence, timestamp_ns, 1, sum(DEVICE_READY_FLAGS[name] for name in devices),
                        tuple(q[:7]), tuple(q[7:14]), tuple(q[14:34]), tuple(q[34:]))


def pose_error(actual, expected):
    a, b = np.asarray(actual), np.asarray(expected)
    if a.shape != (7,) or b.shape != (7,) or not np.isfinite(a).all() or not np.isfinite(b).all():
        raise SafetyFault("invalid measured or reference pose")
    norm = np.linalg.norm(a[3:]) * np.linalg.norm(b[3:])
    if norm < 1e-12:
        raise SafetyFault("invalid pose quaternion")
    angle = math.degrees(2 * math.acos(float(np.clip(abs(np.dot(a[3:], b[3:])) / norm, 0, 1))))
    return float(np.linalg.norm(a[:3] - b[:3])), angle


class MeasuredWrist:
    """Independent FK: monitoring must never reseed the command IK worker."""

    def __init__(self, model_path):
        import mujoco
        from sim.physics import JOINT_NAMES
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.indices = np.array([self.model.joint(name).qposadr[0] for name in JOINT_NAMES])

    def __call__(self, positions):
        import mujoco
        from .targets import model_wrist_pose
        self.data.qpos[self.indices] = positions
        mujoco.mj_forward(self.model, self.data)
        return model_wrist_pose(self.model, self.data, "right")


class StableLocalization:
    """Require a stationary robot and a stable paired FK/Motive transform."""

    def __init__(self, position_limit, angle_limit, stable_s, speed_limit, feedback_timeout_ns, devices):
        from real_robot.settling import FeedbackRest
        self.position_limit, self.angle_limit = position_limit, angle_limit
        self.stable_s = stable_s
        self.rest = {name: FeedbackRest(speed_limit, stable_s * 1e9, feedback_timeout_ns)
                     for name in devices}
        self.reset()

    def reset(self):
        self.anchor = None
        self.since = None
        self.history = deque()
        self.inconsistent_since = None
        for rest in self.rest.values():
            rest.previous, rest.since, rest.resting = None, None, False

    def consistent(self, measured_wrist, motive_wrist, transform, now, maximum_skew_s):
        # Motive and joint feedback are asynchronous. Expected wrist travel
        # through any admitted recent measured FK is not a localization jump.
        self.history.append((now, measured_wrist.copy()))
        while self.history and self.history[0][0] < now - maximum_skew_s:
            self.history.popleft()
        transformed = compose_pose(transform, motive_wrist)
        for _, wrist in reversed(self.history):
            position, angle = pose_error(wrist, transformed)
            if position <= self.position_limit and angle <= self.angle_limit:
                self.inconsistent_since = None
                return True
        if self.inconsistent_since is None:
            self.inconsistent_since = now
        return now - self.inconsistent_since < self.stable_s

    def observe(self, transform, feedback, now):
        stationary = True
        for name, rest in self.rest.items():
            item = feedback[name]
            rest.observe(item.position_rad, item.received_monotonic_ns, True)
            stationary = stationary and rest.resting
        if self.anchor is None or any(value > limit for value, limit in zip(
                pose_error(transform, self.anchor), (self.position_limit, self.angle_limit))):
            self.anchor, self.since = transform.copy(), now
        return stationary and now - self.since >= self.stable_s


def _authorized(path, expected):
    allowed = [expected] if isinstance(expected, str) else expected
    if not isinstance(allowed, list) or not allowed:
        raise ValueError("Regrind artifact whitelist is missing")
    with Path(path).open("rb") as stream:
        digest = hashlib.file_digest(stream, "sha256").hexdigest()
    if digest not in allowed:
        raise SafetyFault(f"{Path(path).name}: SHA256 is not authorized for real Regrind")


def _policy_settings(path):
    import yaml
    settings = yaml.safe_load(Path(path).read_text())
    if not isinstance(settings, dict):
        raise ValueError("Regrind safety configuration must be a mapping")
    keys = ("motive_stale_s", "arm_stale_s", "hand_stale_s", "maximum_input_skew_s",
            "wrist_frame0_position_tolerance_m", "wrist_frame0_orientation_tolerance_deg",
            "hammer_start_position_tolerance_m", "hammer_start_orientation_tolerance_deg",
            "hand_maximum_step_rad", "hand_tracking_error_rad", "hand_tracking_error_duration_s",
            "hand_instant_error_rad")
    for key in keys:
        value = float(settings[key])
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"Regrind {key} must be positive and finite")
        settings[key] = value
    if float(settings["rate"]) != 50.0:
        raise ValueError("Regrind real runtime requires a 50 Hz policy")
    for key in ("real_hand_kp", "real_hand_kd", "real_hand_effort_limit_amps"):
        value = float(settings[key])
        if not math.isfinite(value) or value < 0:
            raise ValueError(f"Regrind {key} must be finite and nonnegative")
        settings[key] = value
    for key, count in (("real_arm_joint_stiffness", 7), ("real_arm_joint_damping", 7),
                       ("real_arm_tool_kinematics", 6), ("real_arm_tool_dynamics", 10)):
        values = np.asarray(settings[key], dtype=float)
        if values.shape != (count,) or not np.isfinite(values).all():
            raise ValueError(f"Regrind {key} requires {count} finite values")
    if (np.any(np.asarray(settings["real_arm_joint_stiffness"]) < 0)
            or np.any(np.asarray(settings["real_arm_joint_stiffness"]) > 22)
            or np.any(np.asarray(settings["real_arm_joint_damping"]) < 0)
            or np.any(np.asarray(settings["real_arm_joint_damping"]) > 1)
            or settings["real_arm_tool_dynamics"][0] < 0):
        raise ValueError("Regrind impedance settings exceed SDK ranges")
    return settings


class PolicyInputs:
    def __init__(self, tracker, settings, *, require_object=True):
        self.tracker, self.settings = tracker, settings
        self.require_object = require_object
        self.last_frame = None
        self.last_received = None
        self.error_since = None

    def read(self, feedback, now_ns):
        if self.tracker.error:
            raise SafetyFault(f"Motive: {self.tracker.error}")
        sample = self.tracker.latest()
        if sample is None:
            raise SafetyFault("Motive wrist/required object unavailable")
        # Snapshot time after the asynchronous Motive sample. A callback can
        # arrive after the caller's loop timestamp but before latest().
        now = time.monotonic_ns() / 1e9
        for name, stamp, limit in (
            ("Motive", sample.received_at, self.settings["motive_stale_s"]),
            ("arm", feedback["arms"].received_monotonic_ns / 1e9, self.settings["arm_stale_s"]),
            ("hand", feedback["right_hand"].received_monotonic_ns / 1e9, self.settings["hand_stale_s"]),
        ):
            if not math.isfinite(stamp) or stamp <= 0 or not 0 <= now - stamp <= limit:
                raise SafetyFault(f"{name} input stale or clock invalid")
        for name in ("arms", "right_hand"):
            stamp = feedback[name].received_monotonic_ns / 1e9
            if abs(sample.received_at - stamp) > self.settings["maximum_input_skew_s"]:
                raise SafetyFault(f"Motive/{name} input time skew exceeded")
        if self.last_received is not None:
            if sample.received_at < self.last_received:
                raise SafetyFault("Motive clock moved backwards")
            if sample.received_at > self.last_received and sample.frame_number <= self.last_frame:
                raise SafetyFault("Motive frame numbering stopped advancing")
        self.last_received, self.last_frame = sample.received_at, sample.frame_number
        pose_error(sample.wrist_xyzw, sample.wrist_xyzw)
        if self.require_object:
            pose_error(sample.hammer_xyzw, sample.hammer_xyzw)
        return sample, np.asarray(feedback["right_hand"].position_rad, dtype=np.float64)

    def check_hand(self, actual, previous_command, now):
        error = float(np.max(np.abs(actual - previous_command)))
        if error > self.settings["hand_instant_error_rad"]:
            raise SafetyFault(f"right hand instantaneous following error {error:.3f} rad")
        if error > self.settings["hand_tracking_error_rad"]:
            if self.error_since is None:
                self.error_since = now
            if now - self.error_since > self.settings["hand_tracking_error_duration_s"]:
                raise SafetyFault(f"right hand sustained following error {error:.3f} rad")
        else:
            self.error_since = None


def _reference_target(policy, index):
    reference = policy.reference
    return TargetFrame(index / 50.0, index,
                       wrist_poses={"right": np.r_[reference.wrist_pos[index], np.roll(reference.wrist_quat_wxyz[index], -1)]},
                       hand_joints={"right": reference.joints[index].copy()},
                       object_poses={"hammer": np.r_[reference.object_pos[index], np.roll(reference.object_quat_wxyz[index], -1)]})


def _require_channels(target, arms, hands):
    for side in arms:
        if side not in target.arm_joints and side not in target.wrist_poses:
            raise SafetyFault(f"{side} arm trajectory input invalid/missing")
    for side in hands:
        if side not in target.hand_joints and side not in target.hand_keypoints:
            raise SafetyFault(f"{side} hand trajectory input invalid/missing")



def _replay_channels_valid(target, channels):
    return all(side in getattr(target, field) for field, sides in channels.items() for side in sides)


def _replay_start(trajectory, start_frame):
    if not 0 <= start_frame < trajectory.frame_count:
        raise ValueError("--start-frame outside trajectory")
    channels = {
        field: {side for side, track in trajectory.tracks.get(field, {}).items() if track.valid.any()}
        for field in ("arm_joints", "wrist_poses", "hand_joints", "hand_keypoints")
    }
    if not channels["arm_joints"] and not channels["wrist_poses"]:
        raise ValueError("real replay requires at least one valid arm channel")
    for index in range(start_frame, trajectory.frame_count):
        target = trajectory.sample(float(trajectory.timeline_s[index]))
        if _replay_channels_valid(target, channels):
            return target, channels
    raise SafetyFault("no all-active-valid trajectory frame at or after --start-frame")


def _validate_replay_joint_limits(trajectory, safety):
    """Reject bad future direct commands before any hardware is connected."""
    for field in ("arm_joints", "hand_joints"):
        for side, track in trajectory.tracks.get(field, {}).items():
            device = "arms" if field == "arm_joints" else side + "_hand"
            lower = np.asarray(safety[device]["lower_rad"])
            upper = np.asarray(safety[device]["upper_rad"])
            if field == "arm_joints":
                region = slice(0, 7) if side == "left" else slice(7, 14)
                lower, upper = lower[region], upper[region]
            values = track.values[track.valid]
            if not np.isfinite(values).all() or np.any(values < lower) or np.any(values > upper):
                raise SafetyFault(f"{side} {field}: recorded joint target exceeds real limits")


def _validate_h5_hand(replay, safety):
    """Admit every prepared hand command before any hardware is connected."""
    track = replay.hand_joint_track
    values = np.asarray(track.values)
    if values.ndim != 2 or values.shape[1] != 20 or not len(values) or not np.all(track.valid):
        raise SafetyFault("h5-real requires a complete prepared right hand trajectory")
    if not np.isfinite(values).all():
        raise SafetyFault("h5-real requires finite prepared right hand joints")
    lower = np.asarray(safety["right_hand"]["lower_rad"])
    upper = np.asarray(safety["right_hand"]["upper_rad"])
    if np.any(values < lower) or np.any(values > upper):
        raise SafetyFault("right hand recorded joint target exceeds real limits")


def _set_regrind_mode(device, enabled, guard, settings):
    device.set_right_impedance(
        enabled, stiffness=settings["real_arm_joint_stiffness"],
        damping=settings["real_arm_joint_damping"],
        tool_kinematics=settings["real_arm_tool_kinematics"],
        tool_dynamics=settings["real_arm_tool_dynamics"], guard=guard)

def run(args):
    """Execute a Namespace produced by parser(); return zero only after cleanup."""
    # This guard precedes SDK construction, imports of native control, and Motive.
    if not args.confirm_real:
        print("REAL REFUSED: --confirm-real required; no hardware activity", file=sys.stderr)
        return 2
    if not sys.stdin.isatty():
        print("REAL REFUSED: an operator TTY is required; no hardware activity", file=sys.stderr)
        return 2
    hardware = {}
    keyboard = ik = tracker = session = viewer = None
    handlers = {}
    result = 1
    try:
        from .native import NativeIK, safety_limits
        from .targets import TargetAdapter
        from .real_keyboard import OperatorKeyboard
        from .h5_replay import H5Replay, motive_world_transform, motive_rigid_to_wrist

        config_path = Path(args.config).resolve()
        config, configured_devices = load_configuration(config_path)
        if float(config["safety"]["rate_hz"]) != 200.0:
            raise ValueError("real runtime requires the target 200 Hz safety rate")
        policy = trajectory = inputs = h5_replay = None
        settings = None
        if args.policy:
            import torch
            torch.set_num_threads(1)
            from ..policies.regrind.runtime import RegrindPolicy
            from ..policies.regrind.tracking import RegrindMotiveTracker, open_mocap_session
            settings = _policy_settings(args.policy_config)
            _authorized(args.model, settings["checkpoint_sha256"])
            _authorized(args.reference, settings["reference_sha256"])
            policy = RegrindPolicy(args.model, args.reference, device=args.device,
                                   start_frame=args.start_frame, reference_speed=args.reference_speed)
            target = _reference_target(policy, args.start_frame)
            arms, hands = {"right"}, {"right"}
        else:
            from ..data import load_trajectory
            if args.trajectory.is_symlink():
                raise SafetyFault("real replay requires a regular non-symlink recording")
            trajectory = load_trajectory(args.trajectory, format=args.format, mode=args.mode, rate_hz=args.rate_hz)
            if trajectory.format in ("acquisition", "regrind") or getattr(args, "h5_replay", False):
                if args.start_frame:
                    raise ValueError("H5 replay starts at the first valid right wrist; --start-frame is for policy/legacy replay")
                h5_replay = H5Replay(args.trajectory, speed=args.reference_speed,
                                     yaw_deg=getattr(args, "yaw_deg", 0.0),
                                     format=args.format, rate_hz=args.rate_hz)
                trajectory = h5_replay.trajectory
                _validate_h5_hand(h5_replay, config["safety"])
                target = h5_replay.sample(0.0)
                replay_channels = {"wrist_poses": {"right"}, "hand_joints": {"right"}}
                replay_time = 0.0
                arms, hands = {"right"}, {"right"}
                print(f"H5 source: format={trajectory.format}, hand_mode={h5_replay.hand_mode}", flush=True)
            else:
                _validate_replay_joint_limits(trajectory, config["safety"])
                if trajectory.format == "session" and args.mode == "target":
                    raise SafetyFault("legacy session target replay is simulation-only; use recorded joint commands")
                if trajectory.format == "session" and trajectory.metadata.get("source_type") != "joint_replay":
                    raise SafetyFault("legacy real joint replay requires source_type=joint_replay")
                target, replay_channels = _replay_start(trajectory, args.start_frame)
                replay_time = target.time_s
                arms = replay_channels["arm_joints"] | replay_channels["wrist_poses"]
                hands = replay_channels["hand_joints"] | replay_channels["hand_keypoints"]
            print(f"Replay start: requested frame {args.start_frame}, using frame {target.index} "
                  f"at {target.time_s:.9f} s", flush=True)
        source_controls = policy is not None or h5_replay is not None
        if args.hold_enter is None:
            args.hold_enter = source_controls
        if source_controls and not args.hold_enter:
            raise SafetyFault("H5/Regrind real execution requires physical Enter release detection")
        if policy is not None:
            config["right_hand"] = dict(
                config["right_hand"], kp=settings["real_hand_kp"], kd=settings["real_hand_kd"],
                effort_limit_amps=min(config["right_hand"]["effort_limit_amps"],
                                      settings["real_hand_effort_limit_amps"]))
            print(f"Regrind profile right-hand MIT kp={settings['real_hand_kp']}, "
                  f"kd={settings['real_hand_kd']}; right arm joint impedance "
                  f"K={settings['real_arm_joint_stiffness']}, D={settings['real_arm_joint_damping']}. "
                  "Configured identities, stricter effort cap and target safety limits retained.", flush=True)
        if not arms <= {"left", "right"} or not hands <= {"left", "right"}:
            raise ValueError("unsupported trajectory side")
        devices = ("arms",) + tuple(side + "_hand" for side in ("left", "right") if side in hands)
        if not set(devices) <= set(configured_devices):
            raise ValueError(f"real config does not authorize required devices {devices}")
        print(f"Selected devices: {', '.join(devices)}. Clear the entire motion path.\n"
              "Keep the physical emergency stop reachable. Speed bounds do not prove collision safety.\n"
              "--confirm-real accepted: connecting read-only; Enter is still required to enable motion.", flush=True)
        keyboard = OperatorKeyboard(args.hold_enter)

        def interrupt(signum, frame):
            raise KeyboardInterrupt

        handlers = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
        model_path = resolve(config_path.parent, config["controller_model"])
        viewer = RealRobotViewer(model_path, object_mesh=args.object_mesh)
        # Build one device at a time so partial construction is always owned by finally.
        for name in devices:
            hardware.update(make_hardware(config, (name,), config_path.parent))
        for name, device in hardware.items():
            print(f"READ-ONLY CONNECT {name}", flush=True)
            device.connect()
        measured = fresh_feedback(hardware, config["startup_timeout_s"], config["safety"]["feedback_timeout_s"])
        if any(value.enabled for value in measured.values()):
            raise SafetyFault("refusing to take over an already-enabled device")
        # Check every raw device sample before any command-model projection.
        feedback_gate = MotionGate(config["safety"], devices)
        stamp = time.monotonic_ns()
        for name in devices:
            feedback_gate.check_feedback(name, measured[name], stamp)
        raw_initial = pack_positions(measured)
        lower, upper = safety_limits()
        initial = admit_measured_positions(raw_initial, lower, upper)
        ik = NativeIK(model=model_path, initial_positions=initial[:14])
        initial = admit_measured_positions(raw_initial, ik.lower, ik.upper)
        conditioning_settings = settings
        if trajectory is not None and trajectory.format in ("acquisition", "regrind"):
            import yaml
            conditioning_settings = yaml.safe_load(Path(args.replay_config).read_text())
        adapter = TargetAdapter(ik, initial_positions=initial, legacy_tcp=args.legacy_tcp,
                                conditioning_settings=conditioning_settings)
        # Localization uses admitted measured FK, never a configured Home pose.
        reference_transform = None
        robot_pose = None
        staged = dict(config["staged_motion"])
        if h5_replay is not None:
            staged["settle_time_s"] = max(staged["settle_time_s"],
                                          conditioning_settings["approach_stable_seconds"])
        # Unselected arms hold the admitted measured seed for the entire session.
        for side, start in (("left", 0), ("right", 7)):
            if side not in arms:
                staged[f"home_{side}_rad"] = initial[start:start + 7].tolist()
        safety = dict(config["safety"])
        if settings is not None:
            safety["right_hand"] = dict(safety["right_hand"])
            safety["right_hand"]["maximum_speed_rad_s"] = min(
                safety["right_hand"]["maximum_speed_rad_s"], settings["hand_maximum_step_rad"] * 50.0)
        gate = RuntimeMotionGate(safety, devices, staged)
        if source_controls:
            from ..policies.regrind.tracking import RegrindMotiveTracker, open_mocap_session
            session = open_mocap_session(args.endpoint)
            input_settings = settings
            if h5_replay is not None:
                input_settings = dict(
                    motive_stale_s=conditioning_settings["motive_stale_s"],
                    arm_stale_s=config["safety"]["feedback_timeout_s"],
                    hand_stale_s=config["safety"]["feedback_timeout_s"],
                    maximum_input_skew_s=max(conditioning_settings["motive_stale_s"],
                                             config["safety"]["feedback_timeout_s"]))
            object_transform = None if policy is None else np.asarray(
                settings["hammer_rigid_to_object_translation_m"]
                + settings["hammer_rigid_to_object_quaternion_xyzw"])
            tracker = RegrindMotiveTracker(
                session, wrist_name=getattr(args, "wrist_name", "right_wrist"),
                rigid_to_wrist=motive_rigid_to_wrist(conditioning_settings),
                require_object=policy is not None, rigid_to_object=object_transform)
            inputs = PolicyInputs(tracker, input_settings, require_object=policy is not None)
            measured_wrist = MeasuredWrist(model_path)
            position_limit = (settings["wrist_frame0_position_tolerance_m"] if policy is not None
                              else conditioning_settings["approach_position_tolerance_m"])
            angle_limit = (settings["wrist_frame0_orientation_tolerance_deg"] if policy is not None
                           else conditioning_settings["approach_orientation_tolerance_deg"])
            # Twice arrival tolerances allow tracking noise, but not a frame jump.
            localization = StableLocalization(
                2 * position_limit, 2 * angle_limit, staged["settle_time_s"],
                staged.get("settle_speed_rad_s", .03), gate.feedback_timeout_ns, devices)
        q = initial.copy() if source_controls else adapter.frame(target)
        state = "WAITING" if source_controls else "ENABLE"
        for side, start in (("left", 0), ("right", 7)):
            if side not in arms:
                q[start:start + 7] = initial[start:start + 7]
        sequence = 1
        enabled = set()
        moving = False
        policy_tick = 0
        last_commands = None
        stable_since = None
        previous_q = q.copy()
        start_time = time.monotonic()
        last_tick = start_time
        next_tick = start_time
        next_visual = start_time
        last_state = None
        previous_held = True  # An initially held Enter is not an operator press.
        release_required = True
        planning_started = None
        planning_stable = None
        recovery_started = None

        def localize(feedback, transform):
            nonlocal adapter, q, reference_transform, robot_pose, planning_started, planning_stable
            measured_seed = admit_measured_positions(pack_positions(feedback), ik.lower, ik.upper)
            ik.synchronize(measured_seed[:14])
            ik.capture_home()
            # Localization comes from measured FK. Planning starts at the held
            # command during recovery, never jumping a command to measured state.
            seed = (admit_measured_positions(pack_positions(last_commands), ik.lower, ik.upper)
                    if gate.armed else measured_seed)
            adapter = TargetAdapter(ik, initial_positions=seed, legacy_tcp=args.legacy_tcp,
                                    conditioning_settings=conditioning_settings)
            reference_transform = transform.copy()
            adapter.set_world_transform(reference_transform)
            robot_pose = invert_pose(reference_transform)
            q = seed.copy()
            planning_started, planning_stable = time.monotonic(), None

        prompts = {
            "WAITING": "等待实时 right_wrist 与关节反馈，自动定位；不使能、不运动。",
            "ENABLE": "定位完成，frame0 已显示。按一次 Enter 使能（不会移动）。",
            "PLANNING": "只读计算 frame0 固定关节目标；不执行运动。",
            "APPROACH": "按住 Enter 缓慢靠近固定 frame0；松开后限加速度减速，仍有少量移动。",
            "BRAKING": "正在减速：松键后仍有少量移动，停止前不能开始回放。",
            "HOLD": "已停止并保持。重新按住 Enter 继续靠近固定 frame0。",
            "RELOCALIZING": "定位不一致：撤销到位，减速保持；静止且定位稳定后重算，再松开并重新按 Enter。",
            "READY": "frame0 已到位并稳定。松开 Enter 后，再按一次开始回放／推理。",
            "RUNNING": "正在执行。再按一次 Enter 暂停；q 停止并失能。",
            "PAUSED": "已暂停，目标与播放时间冻结。再按一次 Enter 继续；q 停止并失能。",
            "COMPLETE": "执行完成，停止并失能；不自动回 Home。",
        }
        print("请在启动命令的终端操作 Enter；q / Ctrl+C 停止并失能，不回 Home。", flush=True)

        def publish(now_ns):
            reference = None
            if reference_transform is not None:
                # Read live objects anew even while DATA time is frozen.
                reference = {
                    "wrist_pose": compose_pose(reference_transform, target.wrist_poses["right"]),
                    "hand_joints": target.hand_joints["right"],
                    "world_transform": reference_transform,
                    "time_s": target.time_s,
                    "index": target.index,
                    "state": state,
                    "calibrated": True,
                    "objects": build_object_overlays(
                        target.object_poses, tracker.latest(), reference_transform,
                        now=time.monotonic(), stale_s=inputs.settings["motive_stale_s"],
                        error=tracker.error),
                }
            display_frame = command_frame(q, sequence, now_ns, devices)
            kwargs = {"reference": reference}
            if source_controls:
                kwargs["robot_pose"] = robot_pose
            viewer.publish(
                {name: measured[name].position_rad for name in devices},
                gate.display_targets if gate.armed else {name: display_frame.positions(name) for name in devices},
                state, **kwargs)

        while True:
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            tick_start = time.monotonic()
            next_tick = tick_start + 0.005
            if gate.armed and tick_start - last_tick > config["safety"]["command_timeout_s"]:
                raise SafetyFault("real execution loop stalled")
            last_tick = tick_start
            if not viewer.is_running():
                raise SafetyFault("real visualization closed or failed")
            keys, held = keyboard.poll()
            # X11/evdev sees global keys. Motion authorization also needs an
            # Enter byte from this operator terminal, not another application.
            enter = "enter" in keys
            if args.hold_enter:
                enter = enter and held and not previous_held
            previous_held = held
            if not held:
                release_required = False
            if "\x03" in keys or "\x1b" in keys:
                raise KeyboardInterrupt
            if "q" in keys or (args.duration and tick_start - start_time >= args.duration):
                result = 0
                break
            measured = {name: device.read_feedback() for name, device in hardware.items()}
            now_ns = time.monotonic_ns()
            for name in devices:
                gate.check_feedback(name, measured[name], now_ns, require_enabled=name in enabled)
            sample = joints = None
            if inputs is not None:
                try:
                    sample, joints = inputs.read(measured, now_ns)
                except SafetyFault:
                    if gate.armed or time.monotonic() - start_time > config["startup_timeout_s"]:
                        raise
                    state, reference_transform, robot_pose = "WAITING", None, None
                    localization.reset()
                    if state != last_state:
                        print(f"REAL {state}: {prompts[state]}", flush=True)
                        last_state = state
                    if tick_start >= next_visual:
                        publish(now_ns)
                        next_visual = tick_start + 1.0 / 30.0
                    continue
                if policy is not None and last_commands is not None:
                    inputs.check_hand(joints, np.asarray(last_commands["right_hand"]), now_ns / 1e9)
                fk = measured_wrist(admit_measured_positions(pack_positions(measured), ik.lower, ik.upper))
                if state not in ("RUNNING", "PAUSED", "WAITING", "RELOCALIZING"):
                    if not localization.consistent(fk, sample.wrist_xyzw, reference_transform, tick_start,
                                                   inputs.settings["maximum_input_skew_s"]):
                        state, moving, release_required = "RELOCALIZING", False, True
                        stable_since = None
                        recovery_started = tick_start
                        localization.reset()
                        if gate.armed:
                            gate.paused = True
                            gate._staged_phase = ALIGNING
                            gate._settle_since_ns = None
                if (state in ("RELOCALIZING", "PLANNING") and recovery_started is not None
                        and tick_start - recovery_started > staged["timeout_s"]):
                    raise SafetyFault("frame0 localization recovery timed out")
                if state in ("WAITING", "RELOCALIZING"):
                    candidate = motive_world_transform(
                        fk, sample.wrist_xyzw,
                        rotation=conditioning_settings.get("motive_to_robot_quaternion_xyzw"))
                    stopped = not gate.armed or gate.staged_stopped
                    stable = localization.observe(candidate, measured, tick_start) if stopped else False
                    if stable:
                        localize(measured, candidate)
                        state, release_required = "PLANNING", True
                        enter = False
                if state == "PLANNING":
                    previous_plan = q
                    q = adapter.frame(target)
                    for side, start in (("left", 0), ("right", 7)):
                        if side not in arms:
                            q[start:start + 7] = initial[start:start + 7]
                    goal_wrist = compose_pose(reference_transform, target.wrist_poses["right"])
                    residual = pose_error(measured_wrist(q), goal_wrist)
                    converged = (np.max(np.abs(q - previous_plan)) <= 1e-5
                                 and residual[0] <= position_limit and residual[1] <= angle_limit)
                    planning_stable = ((planning_stable if planning_stable is not None else tick_start)
                                       if converged else None)
                    if tick_start - planning_started > staged["timeout_s"]:
                        raise SafetyFault("frame0 IK did not converge to the localized goal")
                    if planning_stable is not None and tick_start - planning_stable >= staged["settle_time_s"]:
                        if gate.armed:
                            gate.lock_approach(command_frame(q, sequence, now_ns, devices), now_ns)
                        state = "HOLD" if gate.armed else "ENABLE"
                        moving, release_required, enter = False, True, False
                        recovery_started = None
            if state != last_state:
                print(f"REAL {state}: {prompts[state]}", flush=True)
                last_state = state
                next_visual = tick_start
            if tick_start >= next_visual:
                publish(now_ns)
                next_visual = tick_start + 1.0 / 30.0
            if not gate.armed:
                frame = command_frame(q, sequence, now_ns, devices)
                gate.check_enable_ready(frame, measured, now_ns)
                if state != "ENABLE" or not enter or (args.hold_enter and release_required):
                    continue
                gate.arm(frame, measured, now_ns)

                def enable_guard():
                    nonlocal previous_held
                    if not viewer.is_running():
                        raise SafetyFault("real visualization closed during enable")
                    pending, physical = keyboard.poll()
                    previous_held = physical
                    if pending & {"q", "\x03", "\x1b"}:
                        raise SafetyFault("operator interrupted hardware enable")
                    feedback = {name: device.read_feedback() for name, device in hardware.items()}
                    stamp = time.monotonic_ns()
                    for name in devices:
                        gate.check_feedback(name, feedback[name], stamp, require_enabled=name in enabled)
                    if inputs is not None:
                        inputs.read(feedback, stamp)
                    gate.validate_targets(command_frame(q, sequence, stamp, devices), stamp)

                for name, device in hardware.items():
                    enable_guard()
                    device.enable(guard=enable_guard)
                    enabled.add(name)
                if policy is not None:
                    _set_regrind_mode(hardware["arms"], True, enable_guard, settings)
                measured = {name: device.read_feedback() for name, device in hardware.items()}
                now_ns = time.monotonic_ns()
                if inputs is not None:
                    inputs.read(measured, now_ns)
                seed = admit_measured_positions(pack_positions(measured), ik.lower, ik.upper)
                for side, start in (("left", 0), ("right", 7)):
                    if side not in arms:
                        initial[start:start + 7] = seed[start:start + 7]
                        q[start:start + 7] = seed[start:start + 7]
                # Only unselected arms adopt the verified SDK hold. The selected
                # frame0 and converged adapter history stay locked across enable.
                frame = command_frame(q, sequence, now_ns, devices)
                gate._targets = {name: frame.positions(name) for name in devices}
                last_commands = gate.synchronize_enabled(measured, now_ns)
                # SDK callbacks may have polled many repeat bytes or transitions.
                # None authorizes the next phase: observe a fresh outer-loop release.
                state, moving, release_required = "HOLD", False, True
                previous_held = True
                previous_q = q.copy()
                gate.paused = True
                last_tick = time.monotonic()
                next_tick = last_tick + 0.005
                continue

            actual_ready = True
            if source_controls:
                position, angle = pose_error(sample.wrist_xyzw, target.wrist_poses["right"])
                actual_ready = (position <= position_limit and angle <= angle_limit
                                and np.max(np.abs(joints - target.hand_joints["right"]))
                                <= safety["right_hand"]["alignment_rad"])

            if state in ("APPROACH", "BRAKING", "HOLD"):
                if args.hold_enter:
                    if not held or release_required:
                        moving = False
                    elif enter:
                        moving = True
                elif enter:
                    moving = not moving
                settled = moving and gate.phase == READY and gate.staged_stopped and actual_ready
                stable_since = (stable_since if stable_since is not None else tick_start) if settled else None
                previous_q = q.copy()
                if stable_since is not None and tick_start - stable_since >= staged["settle_time_s"]:
                    state, moving, release_required = "READY", False, True
                else:
                    state = "APPROACH" if moving else ("HOLD" if gate.staged_stopped else "BRAKING")
            elif state == "READY":
                if gate.phase != READY or not actual_ready or not gate.staged_stopped:
                    state, moving, release_required = "HOLD", False, True
                    stable_since = None
                elif enter and (not args.hold_enter or not release_required):
                    start_allowed = True
                    if policy is not None:
                        position, angle = pose_error(sample.hammer_xyzw, target.object_poses["hammer"])
                        start_allowed = (position <= settings["hammer_start_position_tolerance_m"]
                                         and angle <= settings["hammer_start_orientation_tolerance_deg"])
                        if not start_allowed:
                            print(f"暂不开始：锤子偏差 {position * 1000:.1f} mm / {angle:.1f}°；调整后重新按 Enter。", flush=True)
                    if start_allowed:
                        frame = command_frame(q, sequence, now_ns, devices)
                        gate.start_teleop(frame, measured, now_ns)
                        if policy is not None:
                            policy.reset(sample.wrist_xyzw, joints)
                        state, moving, policy_tick = "RUNNING", True, 0
            elif state in ("RUNNING", "PAUSED"):
                if enter:
                    state = "PAUSED" if state == "RUNNING" else "RUNNING"
                moving = state == "RUNNING"
                if moving:
                    if policy is not None and policy_tick % 4 == 0:
                        position, angle = pose_error(sample.wrist_xyzw, target.wrist_poses["right"])
                        # Inference cannot outrun the measured wrist.
                        if not policy.complete and position <= position_limit and angle <= angle_limit:
                            target = policy.step(sample.wrist_xyzw, sample.hammer_xyzw, joints)
                            target = replace(target, hand_joints={"right": np.clip(
                                target.hand_joints["right"], safety["right_hand"]["lower_rad"],
                                safety["right_hand"]["upper_rad"])})
                        else:
                            policy.observe(sample.wrist_xyzw, joints)
                    elif policy is None:
                        target = (h5_replay if h5_replay is not None else trajectory).sample(replay_time)
                        if not _replay_channels_valid(target, replay_channels):
                            raise SafetyFault("active trajectory channel invalid/missing")
                        replay_time += 0.005 * args.reference_speed
                    _require_channels(target, arms, hands)
                    q = adapter.frame(target)
                    for side, start in (("left", 0), ("right", 7)):
                        if side not in arms:
                            q[start:start + 7] = initial[start:start + 7]
                    policy_tick += 1
                    # Stop after the terminal target has actually arrived, not
                    # merely when the source clock first crosses EOF.
                    terminal_ready = target.complete and np.max(np.abs(q - previous_q)) <= 1e-5
                    previous_q = q.copy()
                    if terminal_ready:
                        terminal_frame = command_frame(q, sequence, now_ns, devices)
                        terminal_ready = all(
                            max(abs(a - b) for a, b in zip(terminal_frame.positions(name), measured[name].position_rad))
                            <= safety[name]["alignment_rad"] for name in devices)
                    if terminal_ready and source_controls:
                        position, angle = pose_error(sample.wrist_xyzw, target.wrist_poses["right"])
                        terminal_ready = position <= position_limit and angle <= angle_limit
                    if terminal_ready:
                        state = "COMPLETE"
                        publish(now_ns)
                        print(f"REAL {state}: {prompts[state]}", flush=True)
                        result = 0
                        break
                # Paused execution deliberately does not call adapter.hold/frame
                # or policy.observe: native/conditioner/action history is frozen.
            # A reached READY target is already at rest; keep its measured-rest
            # qualification live without initiating a new pause/resume cycle.
            gate.paused = not moving and state != "READY"
            sequence += 1
            frame = command_frame(q, sequence, now_ns, devices)
            now_ns = time.monotonic_ns()
            if now_ns / 1e9 - tick_start > config["safety"]["command_timeout_s"]:
                raise SafetyFault("control computation exceeded command watchdog")
            if inputs is not None:
                inputs.read(measured, now_ns)
            commands = gate.step(frame, measured, now_ns)
            for name, command in commands.items():
                # Recheck all measured input ages after potentially blocking sends.
                stamp = time.monotonic_ns()
                gate.validate_targets(frame, stamp)
                for feedback_name in devices:
                    gate.check_feedback(feedback_name, measured[feedback_name], stamp, require_enabled=True)
                if inputs is not None:
                    inputs.read(measured, stamp)
                hardware[name].send(command)
            last_commands = commands
    except KeyboardInterrupt:
        print("REAL STOP: interrupted; no forced Home", file=sys.stderr, flush=True)
        result = 130
    except (OSError, RuntimeError, ValueError, KeyError, TypeError, EOFError) as error:
        print(f"REAL STOP: {error}; no forced Home", file=sys.stderr, flush=True)
        result = 1
    finally:
        for sig in handlers:
            signal.signal(sig, signal.SIG_IGN)
        # Stop every device before close can block waiting for disable feedback.
        # Hardware close performs verified disable (there is no public disable()).
        for operation in ("stop", "close"):
            for name, device in reversed(tuple(hardware.items())):
                try:
                    getattr(device, operation)()
                except Exception as error:
                    print(f"{name} {operation} unconfirmed: {error}; USE PHYSICAL EMERGENCY STOP", file=sys.stderr, flush=True)
                    result = 1
        for name, resource in (("Motive tracker", tracker), ("Motive session", session),
                               ("native IK", ik), ("visualization", viewer), ("keyboard", keyboard)):
            if resource is not None:
                try:
                    resource.close()
                except Exception as error:
                    print(f"{name} cleanup failed: {error}", file=sys.stderr, flush=True)
                    result = 1
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    return result


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    source = result.add_mutually_exclusive_group(required=True)
    source.add_argument("--trajectory", "--h5", dest="trajectory", type=Path)
    source.add_argument("--policy", choices=("regrind",))
    result.add_argument("--h5-replay", action="store_true", help="H5 auto-localization and physical Enter enable/approach/playback")
    result.add_argument("--model", type=Path)
    result.add_argument("--object-mesh", type=Path,
                        help="hammer OBJ in metres, preserving its original object-frame origin; display only")
    result.add_argument("--reference", type=Path)
    result.add_argument("--config", type=Path, default=ROOT.parent / "real_robot/config.json")
    result.add_argument("--policy-config", type=Path, default=ROOT / "configs/regrind.yaml")
    result.add_argument("--replay-config", type=Path, default=ROOT / "configs/replay.yaml")
    result.add_argument("--format", default="auto")
    result.add_argument("--mode", choices=("target", "joint"), default="target")
    result.add_argument("--rate-hz", type=float, default=50.0)
    result.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    result.add_argument("--reference-speed", "--speed", dest="reference_speed", type=float, default=1.0)
    result.add_argument("--start-frame", type=int, default=0)
    result.add_argument("--endpoint", default=os.environ.get("TIANJI_ROUTER_ENDPOINT", "tcp/127.0.0.1:7447"))
    result.add_argument("--wrist-name", default="right_wrist")
    result.add_argument("--yaw-deg", type=float, default=0.0)
    result.add_argument("--duration", type=float, default=0.0)
    result.add_argument("--hold-enter", action=argparse.BooleanOptionalAction, default=None,
                        help="physical Enter edges and hold-to-approach (required for H5/Regrind; legacy defaults to toggle)")
    result.add_argument("--viewer", action="store_true", help="real measured/target monitor is always required")
    result.add_argument("--legacy-tcp", choices=("flange", "hand"),
                        help="required convention for legacy Base_L/Base_R TCP recordings")
    result.add_argument("--confirm-real", action="store_true")
    return result


def main(argv=None):
    cli = parser()
    args = cli.parse_args(argv)
    if args.policy and (args.model is None or args.reference is None):
        cli.error("--policy regrind requires --model and --reference")
    if args.h5_replay and args.policy:
        cli.error("--h5-replay requires --h5/--trajectory, not --policy")
    if not math.isfinite(args.yaw_deg):
        cli.error("--yaw-deg must be finite")
    if args.start_frame < 0:
        cli.error("--start-frame must be nonnegative")
    if not math.isfinite(args.reference_speed) or not 0 < args.reference_speed <= 1:
        cli.error("--reference-speed must be in (0, 1] for real execution")
    if not math.isfinite(args.rate_hz) or args.rate_hz <= 0:
        cli.error("--rate-hz must be positive and finite")
    if not math.isfinite(args.duration) or args.duration < 0:
        cli.error("--duration must be nonnegative and finite")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
