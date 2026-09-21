"""Read-only MuJoCo window: measured solid, control cyan, source reference amber.

The window runs in its own process, started and owned by ``RealRobotViewer``.
That process loads no vendor SDK, opens no PICO/Manus port and never writes a
hardware command; it renders the same controller MJCF by forward kinematics.
The parent hands it the latest snapshot over a non-blocking pipe, so a stalled
or closed window can never block the control loop.

Snapshots are plain mappings of device group to joint values in radians:
``arms`` (14, left arm then right arm), ``left_hand`` and ``right_hand`` (20
each), in the TJRC order declared by the model. A group missing from a snapshot
is UNKNOWN and is hidden rather than drawn at a meaningless zero pose. Every
snapshot carries the monotonic time it was published; the window measures age
from that stamp, so a snapshot older than the executor's 150 ms watchdog (or a
backlog that sat in the pipe) is shown as STALE with the measured pose hidden.

Optional ``reference`` snapshots locate the source right-hand gesture directly
in robot world, independently of either joint stream. Its wrist, Mocap origin,
and named object targets/measurements carry xyzw poses; the executor owns
calibration and trajectory timing. Object origins use green TARGET and magenta
REAL/LIVE markers with RGB orientation axes, never simulated object dynamics.
Measurements expire by their capture timestamp, independently of IPC freshness.
Optional ``robot_pose`` is MOCAP_FROM_ROBOT_WORLD (xyzw), used only after FK
to display robot geometry, sites and references together in the mocap frame.
Joint values and model-local transforms remain unchanged.
"""
from __future__ import annotations

import json
import math
import os
import select
import subprocess
import sys
import time
from collections.abc import Mapping
from pathlib import Path

GROUP_SIZES = {"arms": 14, "left_hand": 20, "right_hand": 20}
GROUP_ORDER = ("arms", "left_hand", "right_hand")

# MuJoCo geom/site groups. Rewriting them in the window's own model copy lets an
# unpublished group be hidden without touching the controller's model file.
GROUP_IDS = {"arms": 1, "left_hand": 2, "right_hand": 3}
STATIC_GROUP = 0
DECOR_GROUP = 4  # never enabled: the controller's own frozen target markers

GHOST_RGBA = (0.10, 0.80, 1.00, 0.35)
REFERENCE_RGBA = (1.0, 0.62, 0.08, 0.75)
OBJECT_TARGET_RGBA = (0.15, 1.0, 0.25, 1.0)
OBJECT_ACTUAL_RGBA = (1.0, 0.15, 0.85, 1.0)
OBJECT_STATES = frozenset(("TRACKED", "UNTRACKED", "STALE", "UNAVAILABLE"))
REFERENCE_STATES = frozenset(("WAITING", "PLANNING", "ENABLE", "HOME", "ARMED", "APPROACH",
                              "BRAKING", "HOLD", "RELOCALIZING", "READY",
                              "RUNNING", "PAUSED", "COMPLETE", "RETURN", "FAULT"))
# A snapshot older than the executor's own 150 ms watchdog is not current state:
# the window then hides the measured pose and says so instead of showing it as is.
SNAPSHOT_TIMEOUT_NS = 150_000_000
CLOCK_TOLERANCE_NS = 5_000_000  # same monotonic-clock tolerance as the executor
STATUS_TAIL_CHARS = 16
MAX_STATUS_LINES = 32
MAX_PHASE_CHARS = 24
MAX_STATE_BYTES = 4096  # one atomic pipe write, never a partial snapshot
DEFAULT_STARTUP_TIMEOUT_S = 60.0
CLOSE_TIMEOUT_S = 2.0
RENDER_PERIOD_S = 1.0 / 60.0

# Camera framing for the whole dual-arm robot, matching the simulator window.
# The mounted assembly is wide, so 2.5 m (the simulator default) clips the
# pedestal base; 2.8 m keeps every link, hand and the mount in frame.
CAMERA_LOOKAT = (0.0, 0.0, 0.85)
CAMERA_DISTANCE = 2.8
CAMERA_AZIMUTH = 135.0
CAMERA_ELEVATION = -15.0

FROZEN_PHASES = ("ALIGNING", "READY")
HOME_PHASES = ("HOMING", "HOME_REACHED")


def _finite_positions(group: str, values) -> tuple[float, ...]:
    """Validate one device group's joint values; reject wrong length or non-finite."""
    count = GROUP_SIZES[group]
    if isinstance(values, (str, bytes)) or not hasattr(values, "__iter__"):
        raise ValueError(f"{group}: expected {count} joint values, got {type(values).__name__}")
    numbers = []
    for value in values:
        try:
            number = float(value)
        except (TypeError, ValueError, OverflowError):
            raise ValueError(f"{group}: joint values must be real numbers") from None
        if not math.isfinite(number):
            raise ValueError(f"{group}: joint values must be finite")
        numbers.append(number)
    if len(numbers) != count:
        raise ValueError(f"{group}: expected {count} joint values, got {len(numbers)}")
    return tuple(numbers)


def _valid_phase(phase) -> str:
    if (not isinstance(phase, str) or not phase or len(phase) > MAX_PHASE_CHARS
            or not phase.isascii() or not phase.isprintable()):
        raise ValueError(f"phase must be 1..{MAX_PHASE_CHARS} printable ASCII characters")
    return phase


def _reference_pose(values, name: str) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise ValueError(f"reference {name}: expected a seven-value xyzw pose")
    try:
        pose = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError):
        raise ValueError(f"reference {name}: expected a seven-value xyzw pose") from None
    if len(pose) != 7 or not all(math.isfinite(value) for value in pose):
        raise ValueError(f"reference {name}: expected seven finite values")
    if abs(math.hypot(*pose[3:]) - 1.0) > 1e-3:
        raise ValueError(f"reference {name}: expected a unit xyzw quaternion")
    return pose


def _valid_objects(objects):
    if not isinstance(objects, Mapping):
        raise ValueError("reference objects must be a mapping of names to poses")
    result = {}
    fields = {"target", "actual", "status", "timestamp_ns", "timeout_ns"}
    for name, item in objects.items():
        if (not isinstance(name, str) or not name or len(name) > 64
                or not name.isascii() or not name.isprintable()):
            raise ValueError("reference object name must be 1-64 printable ASCII characters")
        if not isinstance(item, Mapping) or set(item) != fields:
            raise ValueError(f"reference object {name}: expected poses and tracking metadata")
        status = item["status"]
        if not isinstance(status, str) or status not in OBJECT_STATES:
            raise ValueError(f"reference object {name}: invalid tracking status")
        timestamp = item["timestamp_ns"]
        if timestamp is not None and (type(timestamp) is not int or timestamp <= 0):
            raise ValueError(f"reference object {name}: timestamp must be positive or None")
        timeout = item["timeout_ns"]
        if type(timeout) is not int or timeout <= 0:
            raise ValueError(f"reference object {name}: timeout must be positive")
        target = None if item["target"] is None else _reference_pose(item["target"], f"{name} target")
        actual = None if item["actual"] is None else _reference_pose(item["actual"], f"{name} actual")
        if status == "TRACKED":
            if actual is None or timestamp is None:
                raise ValueError(f"reference object {name}: tracked pose needs a source timestamp")
        elif actual is not None:
            raise ValueError(f"reference object {name}: only tracked objects may have an actual pose")
        result[name] = {"target": target, "actual": actual, "status": status,
                        "timestamp_ns": timestamp, "timeout_ns": timeout}
    return result


def _visible_objects(objects, now_ns, *, snapshot_stale=False):
    """Expire source poses without modifying the cached snapshot or targets."""
    result = {}
    for name, item in objects.items():
        timestamp = item["timestamp_ns"]
        expired = (timestamp is not None
                   and not -CLOCK_TOLERANCE_NS <= now_ns - timestamp <= item["timeout_ns"])
        if item["status"] == "TRACKED" and (snapshot_stale or expired):
            item = {**item, "actual": None, "status": "STALE"}
        result[name] = item
    return result


def _valid_reference(reference):
    """Validate the atomic source target without loading a model in the executor."""
    if reference is None:
        return None
    fields = {"wrist_pose", "hand_joints", "world_transform", "time_s", "index",
              "state", "calibrated", "objects"}
    if not isinstance(reference, Mapping) or set(reference) != fields:
        raise ValueError("reference must contain wrist/hand/world poses and source metadata")
    wrist = _reference_pose(reference["wrist_pose"], "wrist_pose")
    world = _reference_pose(reference["world_transform"], "world_transform")
    hand = _finite_positions("right_hand", reference["hand_joints"])
    # A model-independent wire bound; the window additionally checks the MJCF
    # joint limits before FK. Neither layer conditions or clips the source data.
    if any(abs(value) > math.tau for value in hand):
        raise ValueError("reference hand joints exceed the angular wire bound")
    time_s = reference["time_s"]
    if isinstance(time_s, (str, bytes, bool)):
        raise ValueError("reference time_s must be finite and nonnegative")
    try:
        time_s = float(time_s)
    except (TypeError, ValueError, OverflowError):
        raise ValueError("reference time_s must be finite and nonnegative") from None
    if not math.isfinite(time_s) or time_s < 0:
        raise ValueError("reference time_s must be finite and nonnegative")
    index = reference["index"]
    if type(index) is not int or index < 0:
        raise ValueError("reference index must be a nonnegative integer")
    state = reference["state"]
    if not isinstance(state, str) or state not in REFERENCE_STATES:
        raise ValueError("reference state must be an execution state")
    if type(reference["calibrated"]) is not bool:
        raise ValueError("reference calibrated must be boolean")
    return {"wrist_pose": wrist, "hand_joints": hand, "world_transform": world,
            "time_s": time_s, "index": index, "state": state,
            "calibrated": reference["calibrated"], "objects": _valid_objects(reference["objects"])}


def _state_line(actual, target, phase, timestamp_ns: int, *, reference=None, robot_pose=None) -> bytes:
    """Build one snapshot line; anything the window cannot show is rejected here."""
    if type(timestamp_ns) is not int or timestamp_ns <= 0:
        raise ValueError("timestamp_ns must be a positive monotonic integer")
    message = {"phase": _valid_phase(phase), "timestamp_ns": timestamp_ns, "actual": {}, "target": {}}
    for label, positions in (("actual", actual), ("target", target)):
        if positions is None:
            continue
        if not isinstance(positions, Mapping):
            raise ValueError(f"{label}: expected a mapping of device group to joint values")
        for group, values in positions.items():
            if group not in GROUP_SIZES:
                raise ValueError(f"{label}: unsupported device group {group!r}; "
                                 f"expected one of {', '.join(GROUP_ORDER)}")
            message[label][group] = _finite_positions(group, values)
    if reference is not None:
        message["reference"] = _valid_reference(reference)
    if robot_pose is not None:
        message["robot_pose"] = _reference_pose(robot_pose, "robot_pose")
    line = json.dumps(message, allow_nan=False, separators=(",", ":")).encode("ascii") + b"\n"
    if len(line) > MAX_STATE_BYTES:
        raise ValueError(f"snapshot of {len(line)} bytes exceeds the {MAX_STATE_BYTES} byte limit")
    return line


def _parse_state_line(raw: bytes):
    """Validate one received snapshot; unreadable lines return None."""
    try:
        if len(raw) > MAX_STATE_BYTES:
            return None
        message = json.loads(raw.decode("ascii"))
        timestamp_ns = message["timestamp_ns"]
        if type(timestamp_ns) is not int or timestamp_ns <= 0:
            return None
        sample = {"phase": _valid_phase(message["phase"]), "timestamp_ns": timestamp_ns,
                  "actual": {}, "target": {}}
        for label in ("actual", "target"):
            positions = message[label]
            if not isinstance(positions, dict):
                return None
            for group, values in positions.items():
                if group not in GROUP_SIZES:
                    return None
                sample[label][group] = _finite_positions(group, values)
        if message.get("reference") is not None:
            sample["reference"] = _valid_reference(message["reference"])
        if message.get("robot_pose") is not None:
            sample["robot_pose"] = _reference_pose(message["robot_pose"], "robot_pose")
        return sample
    except (ValueError, KeyError, TypeError, AttributeError, OverflowError):
        return None


def _display_state(sample, now_ns: int):
    """Return (redraw key, visible snapshot, stale seconds), without altering cache."""
    if sample is None:
        return None
    age_ns = now_ns - sample["timestamp_ns"]
    is_stale = not -CLOCK_TOLERANCE_NS <= age_ns <= SNAPSHOT_TIMEOUT_NS
    stale_s = round(age_ns / 1e9, 1) if is_stale else None
    shown = {**sample, "actual": {}} if is_stale else sample
    key = (sample["timestamp_ns"], stale_s)
    reference = sample.get("reference")
    if reference is not None and reference["objects"]:
        objects = _visible_objects(reference["objects"], now_ns, snapshot_stale=is_stale)
        shown = {**shown, "reference": {**reference, "objects": objects}}
        # Object capture can expire before the IPC heartbeat does.
        key += (tuple((name, item["status"]) for name, item in objects.items()),)
    return key, shown, stale_s


def _target_label(phase: str, has_target: bool) -> str:
    """Which target the cyan ghost is showing, per the gate's phase."""
    if not has_target:
        return "nothing published"
    if phase == "WAITING":
        return "measured hold"
    if phase == "PLANNING":
        return "frame 0 candidate (not authorized)"
    if phase in ("ENABLE", "APPROACH"):
        return "locked frame 0"
    if phase == "BRAKING":
        return "locked frame 0 (decelerating, still moving)"
    if phase == "HOLD":
        return "locked frame 0 (held)"
    if phase == "RELOCALIZING":
        return "frame 0 invalidated (relocalizing)"
    if phase in ("RUNNING", "PAUSED", "COMPLETE"):
        return "source command"
    if phase in FROZEN_PHASES:
        return "frozen alignment pose"
    if phase in HOME_PHASES:
        return "HOME"
    return "live teleop"


def _world_pose_known(group, positions):
    # Finger world transforms inherit Link7; finger angles alone are not a pose.
    return "arms" in positions and group in positions


def _reference_legend(reference, *, stale_s=None, robot_pose=None):
    if reference is None:
        return ""
    calibration = "locked" if reference["calibrated"] else "provisional"
    frame = "mocap world" if robot_pose is not None else "robot-world"
    legend = (f"amber = DATA target | {reference['state']} | {calibration}"
              f"\nsource frame {reference['index']} | {reference['time_s']:.3f} s"
              + (" | STALE snapshot" if stale_s is not None else "")
              + f"\ndisplayed {frame} axes: DATA wrist, Mocap origin (RGB = XYZ)")
    if reference["objects"]:
        legend += f"\nobjects: green = TARGET | magenta = REAL/LIVE | RGB = XYZ ({frame})"
        for name, item in reference["objects"].items():
            target = "available" if item["target"] is not None else "UNAVAILABLE"
            if stale_s is not None and item["target"] is not None:
                target = "STALE snapshot"
            legend += f"\n{name}: TARGET {target} | REAL/LIVE {item['status']}"
    return legend


def _overlay_texts(sample, stale_s=None) -> tuple[str, str]:
    """Legend (top left) and per-group status (bottom left) for the window overlay."""
    phase = sample["phase"] if sample else "waiting for first sample"
    actual = sample["actual"] if sample else {}
    target = sample["target"] if sample else {}
    measured = (f"snapshot STALE {stale_s:.1f} s: actual hidden, not current state"
                if stale_s is not None else "solid = measured actual joints")
    actual_state = "STALE" if stale_s is not None else None
    legend = "\n".join((
        f"PHASE: {phase}",
        measured,
        f"cyan = CONTROL target ({_target_label(phase, bool(target))}"
        + (", STALE snapshot)" if stale_s is not None else ")"),
    ))
    reference = sample.get("reference") if sample else None
    if reference is not None:
        legend += "\n" + _reference_legend(
            reference, stale_s=stale_s, robot_pose=sample.get("robot_pose"))
    elif sample and sample.get("robot_pose") is not None:
        legend += "\ndisplayed mocap world"
    status = "\n".join(
        f"{group}: actual {actual_state or ('ok' if _world_pose_known(group, actual) else 'UNKNOWN')} | "
        f"target {('STALE' if stale_s is not None else 'ok') if _world_pose_known(group, target) else 'UNKNOWN'}"
        for group in GROUP_ORDER
    )
    return legend, status


def _child_command(model_path: Path, python=None, *, object_mesh=None) -> list[str]:
    """Command line of the owned window process."""
    command = [python or sys.executable, "-m", "tianji_controller.viewer", "--model", str(model_path)]
    if object_mesh is not None:
        command.extend(("--object-mesh", str(object_mesh)))
    return command


class RealRobotViewer:
    """Owned read-only window: measured solid, control cyan, optional DATA amber."""

    def __init__(self, model_path: Path, *, startup_timeout_s: float = DEFAULT_STARTUP_TIMEOUT_S,
                 python=None, object_mesh: Path | None = None):
        path = Path(model_path)
        if not math.isfinite(startup_timeout_s) or startup_timeout_s <= 0:
            raise ValueError("startup_timeout_s must be positive and finite")
        if not path.is_file():
            raise FileNotFoundError(f"viewer model not found: {path}")
        if object_mesh is not None:
            object_mesh = Path(object_mesh).expanduser().resolve()
            if not object_mesh.is_file():
                raise FileNotFoundError(f"object mesh not found: {object_mesh}")
            if object_mesh.suffix.lower() != ".obj":
                raise ValueError(f"object mesh must be an OBJ file: {object_mesh}")
        self._process = None
        self._state_fd = None
        self._status_fd = None
        self._status_buffer = bytearray()
        self._status_lines: list[str] = []
        self._status_eof = False
        self._ready = False
        self._failure = None
        state_read, state_write = os.pipe()
        status_read, status_write = os.pipe()
        try:
            process = subprocess.Popen(_child_command(path, python, object_mesh=object_mesh), stdin=state_read,
                                       stdout=status_write,
                                       cwd=str(Path(__file__).resolve().parent.parent))
        except BaseException:
            for descriptor in (state_read, state_write, status_read, status_write):
                os.close(descriptor)
            raise
        os.close(state_read)
        os.close(status_write)
        os.set_blocking(state_write, False)
        os.set_blocking(status_read, False)
        self._process = process
        self._state_fd = state_write
        self._status_fd = status_read
        try:
            self._await_ready(float(startup_timeout_s))
        except BaseException as error:
            try:
                self.close()
            except Exception as cleanup_error:
                error.add_note(f"viewer cleanup failed: {cleanup_error}")
            raise

    @property
    def failure(self) -> str | None:
        """Reason the window stopped, when the child reported one."""
        return self._failure

    def publish(self, actual, target, phase: str, *, reference=None, robot_pose=None) -> None:
        """Show one atomic snapshot; ``robot_pose`` places FK in mocap world for display only."""
        line = _state_line(actual, target, phase, time.monotonic_ns(),
                           reference=reference, robot_pose=robot_pose)
        descriptor = self._state_fd
        if descriptor is None:
            return
        try:
            os.write(descriptor, line)
        except (BlockingIOError, BrokenPipeError, InterruptedError):
            # The window is behind or gone; the next snapshot replaces this one.
            pass

    def is_running(self) -> bool:
        """False once the window process exited: closed by the user or crashed."""
        process = self._process
        if process is None:
            return False
        self._drain_status()
        return self._failure is None and process.poll() is None

    def close(self) -> None:
        """Stop the owned window; bounded, idempotent, touches no other process."""
        state_fd, status_fd, process = self._state_fd, self._status_fd, self._process
        if state_fd is None and status_fd is None and process is None:
            return
        self._state_fd = None
        if state_fd is not None:
            try:
                os.write(state_fd, b"STOP\n")
            except OSError:
                pass
            try:
                os.close(state_fd)
            except OSError:
                pass
        errors = []
        try:
            if process is not None:
                if process.poll() is None:
                    try:
                        process.wait(timeout=CLOSE_TIMEOUT_S)
                    except subprocess.TimeoutExpired:
                        errors.append("viewer graceful shutdown timed out; forced termination required")
                        try:
                            process.terminate()
                        except ProcessLookupError:
                            pass  # The child may have exited since wait timed out.
                        try:
                            process.wait(timeout=CLOSE_TIMEOUT_S)
                        except subprocess.TimeoutExpired:
                            try:
                                process.kill()
                            except ProcessLookupError:
                                pass
                            try:
                                process.wait(timeout=CLOSE_TIMEOUT_S)
                            except subprocess.TimeoutExpired:
                                errors.append("viewer exit could not be confirmed after kill")
                code = process.poll()
                if code is not None:
                    self._process = None
                    if code != 0:
                        errors.append(f"viewer process exited with status {code}")
                # Retain ownership if still alive, so close can be retried.
            self._drain_status()
            if self._failure is not None:
                errors.append(self._failure)
        finally:
            if status_fd is not None:
                self._status_fd = None
                try:
                    os.close(status_fd)
                except OSError:
                    pass
        if errors:
            self._failure = "; ".join(errors)
            raise RuntimeError(self._failure)

    def _await_ready(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while True:
            self._drain_status()
            if self._ready:
                return
            if self._failure is not None:
                raise RuntimeError(self._failure)
            process = self._process
            if process.poll() is not None:
                raise RuntimeError(f"the viewer window process exited with status "
                                   f"{process.returncode} before reporting readiness")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError(f"the viewer window did not become ready within {timeout_s:g} s")
            try:
                select.select([self._status_fd], [], [], min(remaining, 0.25))
            except (OSError, ValueError):
                time.sleep(0.05)

    def _drain_status(self) -> None:
        """Read the child's status pipe without blocking and keep its diagnostics."""
        if self._status_fd is None or self._status_eof:
            return
        chunk_ready = False
        try:
            chunk_ready = bool(select.select([self._status_fd], [], [], 0)[0])
        except (OSError, ValueError):
            self._status_eof = True
        if chunk_ready:
            try:
                chunk = os.read(self._status_fd, 65536)
            except BlockingIOError:
                chunk = None
            except OSError:
                self._status_eof = True
                chunk = None
            if chunk is not None:
                if chunk:
                    self._status_buffer.extend(chunk)
                    while True:
                        index = self._status_buffer.find(b"\n")
                        if index < 0:
                            break
                        line = bytes(self._status_buffer[:index])
                        del self._status_buffer[:index + 1]
                        self._status_lines.append(line.decode("ascii", "replace"))
                else:
                    self._status_eof = True
                    if self._status_buffer:
                        self._status_lines.append(
                            bytes(self._status_buffer).decode("ascii", "replace"))
                        self._status_buffer.clear()
        if len(self._status_lines) > MAX_STATUS_LINES:
            del self._status_lines[:-STATUS_TAIL_CHARS]
        for line in self._status_lines:
            if line == "READY":
                self._ready = True
            elif line.startswith("ERROR ") and self._failure is None:
                self._failure = line[len("ERROR "):].strip() or "the viewer window failed"
        self._status_lines = [line for line in self._status_lines
                              if line != "READY" and not line.startswith("ERROR ")]


# --------------------------------------------------------------------------- #
# Owned window process
# --------------------------------------------------------------------------- #

def _model_name(model, mujoco, object_type, index: int) -> str:
    name = mujoco.mj_id2name(model, object_type, index)
    if not name:
        raise ValueError(f"viewer model needs named joints to map TJRC groups (index {index})")
    return name


def _joint_group(name: str) -> int:
    """MuJoCo group of a model joint, by the model's own naming convention."""
    if name.startswith("Joint"):
        return GROUP_IDS["arms"]
    if name.startswith("l_"):
        return GROUP_IDS["left_hand"]
    if name.startswith("r_"):
        return GROUP_IDS["right_hand"]
    raise ValueError(f"unrecognized joint name {name!r} in the viewer model")


def _body_group(model, mujoco, body: int) -> int:
    """Group of a body: its own or nearest ancestor joint; static bodies keep group 0."""
    while body != 0:
        if model.body_jntnum[body] > 0:
            name = _model_name(model, mujoco, mujoco.mjtObj.mjOBJ_JOINT,
                               int(model.body_jntadr[body]))
            return _joint_group(name)
        body = int(model.body_parentid[body])
    return STATIC_GROUP


def _decor(model, body: int) -> bool:
    """True for the controller's own mocap target markers, which the window hides."""
    return int(model.body_mocapid[body]) >= 0


def _group_qpos(model, mujoco) -> dict[str, list[int]]:
    """qpos addresses per device group, in TJRC order (left arm, right arm, hands)."""
    arms = {"L": {}, "R": {}}
    hands = {"left_hand": [], "right_hand": []}
    for index in range(model.njnt):
        name = _model_name(model, mujoco, mujoco.mjtObj.mjOBJ_JOINT, index)
        address = int(model.jnt_qposadr[index])
        if name.startswith("Joint") and name.endswith(("_L", "_R")):
            number = name[len("Joint"):-2]
            if not number.isdigit() or not 1 <= int(number) <= 7:
                raise ValueError(f"unexpected TJRC arm joint {name!r} in the viewer model")
            arms[name[-1]][int(number)] = address
        else:
            group = _joint_group(name)
            if group == GROUP_IDS["arms"]:
                raise ValueError(f"unexpected arm joint {name!r} in the viewer model")
            hands["left_hand" if group == GROUP_IDS["left_hand"] else "right_hand"].append(address)
    if any(sorted(arms[side]) != list(range(1, 8)) for side in ("L", "R")):
        raise ValueError("viewer model is missing one of the 14 TJRC arm joints Joint1..7_L/R")
    for group, prefix in (("left_hand", "l_"), ("right_hand", "r_")):
        if len(hands[group]) != GROUP_SIZES[group]:
            raise ValueError(f"viewer model has {len(hands[group])} {prefix}* joints, "
                             f"expected {GROUP_SIZES[group]}")
    return {"arms": [arms["L"][number] for number in range(1, 8)]
                     + [arms["R"][number] for number in range(1, 8)],
            "left_hand": hands["left_hand"], "right_hand": hands["right_hand"]}


def _prepare_model(mujoco, model):
    """Group this process's model copy and map device groups to joints.

    Rewriting the groups in the window's own copy lets an unpublished group be
    hidden instead of drawn at a meaningless zero pose, and leaves the
    controller's model file untouched.
    """
    import numpy as np

    indices = _group_qpos(model, mujoco)
    geom_group = np.zeros(model.ngeom, dtype=np.int32)
    site_group = np.zeros(model.nsite, dtype=np.int32)
    for index in range(model.ngeom):
        body = int(model.geom_bodyid[index])
        geom_group[index] = (DECOR_GROUP if _decor(model, body)
                             else _body_group(model, mujoco, body))
    for index in range(model.nsite):
        body = int(model.site_bodyid[index])
        site_group[index] = (DECOR_GROUP if _decor(model, body)
                             else _body_group(model, mujoco, body))
    model.geom_group[:] = geom_group
    model.site_group[:] = site_group
    ghost_geoms = {group: [index for index in range(model.ngeom)
                           if geom_group[index] == GROUP_IDS[group]]
                   for group in GROUP_ORDER}
    return indices, ghost_geoms


def _apply_state(model, mujoco, data, ghost, indices, options, sample) -> None:
    """Apply one snapshot: group visibility, measured pose and target pose."""
    for group in GROUP_ORDER:
        known = _world_pose_known(group, sample["actual"])
        options.geomgroup[GROUP_IDS[group]] = int(known)
        options.sitegroup[GROUP_IDS[group]] = int(known)
        if known:
            data.qpos[indices[group]] = sample["actual"][group]
        if group in sample["target"]:
            ghost.qpos[indices[group]] = sample["target"][group]
    mujoco.mj_kinematics(model, data)
    mujoco.mj_kinematics(model, ghost)
    robot_pose = sample.get("robot_pose")
    if robot_pose is not None:
        from scipy.spatial.transform import Rotation

        rotation = Rotation.from_quat(robot_pose[3:]).as_matrix()
        # Only derived rendering arrays are placed in capture world. Every
        # update starts with fresh FK, so this never accumulates transforms or
        # changes qpos, controls, or any physical/model-local transform.
        for state in (data, ghost):
            for positions, matrices in ((state.geom_xpos, state.geom_xmat),
                                         (state.site_xpos, state.site_xmat)):
                positions[:] = positions @ rotation.T + robot_pose[:3]
                matrices[:] = (rotation @ matrices.reshape(-1, 3, 3)).reshape(-1, 9)


def _append_model_geom(model, mujoco, data, scene, index, rgba) -> None:
    """Copy a model geom using the renderer's original-mesh (not hull) data id."""
    slot = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(slot, int(model.geom_type[index]), model.geom_size[index],
                        data.geom_xpos[index], data.geom_xmat[index].flatten(), rgba)
    slot.dataid = int(model.geom_dataid[index])
    # Mesh and convex hull occupy adjacent slots in MuJoCo's renderer.
    if slot.type in (mujoco.mjtGeom.mjGEOM_MESH, mujoco.mjtGeom.mjGEOM_SDF):
        slot.dataid *= 2
    slot.objtype = int(mujoco.mjtObj.mjOBJ_GEOM)
    slot.objid = index
    slot.segid = -1
    slot.label = ""
    scene.ngeom += 1


def _fill_ghost_scene(model, mujoco, ghost, ghost_geoms, scene, sample) -> None:
    """Append this model's own geoms at the target pose as transparent cyan ghosts."""
    for group in GROUP_ORDER:
        if not _world_pose_known(group, sample["target"]):
            continue
        for index in ghost_geoms[group]:
            if scene.ngeom >= scene.maxgeom:
                return
            _append_model_geom(model, mujoco, ghost, scene, index, GHOST_RGBA)


class ReferenceOverlay:
    """DATA hand and objects, optionally displayed in the localized mocap frame."""

    def __init__(self, model, mujoco, indices):
        import numpy as np
        from tianji_description.model_assets import OBJECT_BODY_NAME
        from scipy.spatial.transform import Rotation

        self.model = model
        self.mujoco = mujoco
        self.data = mujoco.MjData(model)
        self.addresses = indices["right_hand"]
        joints = [index for index in range(model.njnt)
                  if int(model.jnt_qposadr[index]) in self.addresses]
        self.limits = model.jnt_range[joints].copy()
        # The wrist mesh is fused into Link7 and therefore belongs to the arm
        # visibility group. Select by mesh, not the controlled joint groups.
        self.geoms = np.asarray([
            index for index in range(model.ngeom)
            if model.geom_type[index] == mujoco.mjtGeom.mjGEOM_MESH
            and model.mesh(int(model.geom_dataid[index])).name.startswith("wuji2_r_")
            and model.mesh(int(model.geom_dataid[index])).name != "wuji2_r_mount"
        ], dtype=int)
        self.object_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, OBJECT_BODY_NAME)
        if self.object_geom >= 0:
            # Compiled vertices are centered/rotated by MuJoCo. The compiled
            # geom local transform restores the raw OBJ frame, not its center.
            self.object_position = model.geom_pos[self.object_geom].copy()
            quat = model.geom_quat[self.object_geom]
            self.object_rotation = Rotation.from_quat(quat[[1, 2, 3, 0]]).as_matrix()
        self.reference = None
        self.robot_pose = None

    def update(self, reference, *, robot_pose=None) -> None:
        self.robot_pose = robot_pose
        if reference is None:
            self.reference = None
            return
        import numpy as np
        from scipy.spatial.transform import Rotation
        from mocap_policy_runtime.integration.targets import model_wrist_pose

        reference = _valid_reference(reference)
        if robot_pose is not None:
            robot_pose = _reference_pose(robot_pose, "robot_pose")
            display_rotation = Rotation.from_quat(robot_pose[3:])

            def display_pose(pose):
                if pose is None:
                    return None
                return tuple(np.concatenate((
                    display_rotation.apply(pose[:3]) + robot_pose[:3],
                    (display_rotation * Rotation.from_quat(pose[3:])).as_quat())))

            reference = {
                **reference,
                "wrist_pose": display_pose(reference["wrist_pose"]),
                "world_transform": display_pose(reference["world_transform"]),
                "objects": {
                    name: {**item, "target": display_pose(item["target"]),
                           "actual": display_pose(item["actual"])}
                    for name, item in reference["objects"].items()
                },
            }
        joints = np.asarray(reference["hand_joints"])
        if np.any(joints < self.limits[:, 0] - 1e-6) or np.any(joints > self.limits[:, 1] + 1e-6):
            raise ValueError("reference hand joints exceed model limits")
        self.data.qpos[self.addresses] = joints
        self.mujoco.mj_kinematics(self.model, self.data)
        current = model_wrist_pose(self.model, self.data, "right")
        target = np.asarray(reference["wrist_pose"])
        rotation = Rotation.from_quat(target[3:]) * Rotation.from_quat(current[3:]).inv()
        self.data.geom_xpos[self.geoms] = (
            rotation.apply(self.data.geom_xpos[self.geoms] - current[:3]) + target[:3])
        self.data.geom_xmat[self.geoms] = (
            rotation.as_matrix() @ self.data.geom_xmat[self.geoms].reshape(-1, 3, 3)).reshape(-1, 9)
        self.reference = reference

    def legend(self) -> str:
        if self.reference is None:
            return ""
        reference = {**self.reference,
                     "objects": _visible_objects(self.reference["objects"], time.monotonic_ns())}
        return _reference_legend(reference, robot_pose=self.robot_pose)

    def draw(self, scene) -> None:
        if self.reference is None:
            return
        for index in self.geoms:
            if scene.ngeom >= scene.maxgeom:
                return
            _append_model_geom(self.model, self.mujoco, self.data, scene, index, REFERENCE_RGBA)
        self._axes(scene, self.reference["wrist_pose"], 0.06, "DATA wrist")
        self._axes(scene, self.reference["world_transform"], 0.12, "Mocap origin")
        objects = _visible_objects(self.reference["objects"], time.monotonic_ns())
        for name, item in objects.items():
            for field, label, rgba in (("target", "TARGET", OBJECT_TARGET_RGBA),
                                       ("actual", "REAL/LIVE", OBJECT_ACTUAL_RGBA)):
                pose = item[field]
                if pose is not None:
                    if name == "hammer" and self.object_geom >= 0:
                        self._object_mesh(scene, pose, rgba, f"{name} {label}")
                    else:
                        self._marker(scene, pose, rgba, f"{name} {label}")
                    self._axes(scene, pose, 0.08, "")

    def _object_mesh(self, scene, pose, rgba, label) -> None:
        import numpy as np
        from scipy.spatial.transform import Rotation

        if scene.ngeom >= scene.maxgeom:
            return
        rotation = Rotation.from_quat(pose[3:]).as_matrix()
        index = self.object_geom
        self.data.geom_xpos[index] = np.asarray(pose[:3]) + rotation @ self.object_position
        self.data.geom_xmat[index] = (rotation @ self.object_rotation).reshape(-1)
        _append_model_geom(self.model, self.mujoco, self.data, scene, index, rgba)
        scene.geoms[scene.ngeom - 1].label = label

    def _marker(self, scene, pose, rgba, label) -> None:
        import numpy as np

        if scene.ngeom >= scene.maxgeom:
            return
        geom = scene.geoms[scene.ngeom]
        self.mujoco.mjv_initGeom(geom, self.mujoco.mjtGeom.mjGEOM_SPHERE,
                                np.full(3, 0.009), np.asarray(pose[:3]),
                                np.eye(3).reshape(-1), rgba)
        geom.objtype = int(self.mujoco.mjtObj.mjOBJ_UNKNOWN)
        geom.objid = geom.dataid = geom.segid = -1
        geom.label = label
        scene.ngeom += 1

    def _axes(self, scene, pose, length, label) -> None:
        import numpy as np
        from scipy.spatial.transform import Rotation

        origin = np.asarray(pose[:3])
        rotation = Rotation.from_quat(pose[3:]).as_matrix()
        for axis, color in enumerate(((1, 0.1, 0.1, 1), (0.1, 1, 0.1, 1), (0.1, 0.3, 1, 1))):
            if scene.ngeom >= scene.maxgeom:
                return
            geom = scene.geoms[scene.ngeom]
            self.mujoco.mjv_initGeom(geom, self.mujoco.mjtGeom.mjGEOM_CAPSULE,
                                    np.zeros(3), origin, np.eye(3).reshape(-1), color)
            self.mujoco.mjv_connector(geom, self.mujoco.mjtGeom.mjGEOM_CAPSULE, 0.002,
                                     origin, origin + length * rotation[:, axis])
            geom.objtype = int(self.mujoco.mjtObj.mjOBJ_UNKNOWN)
            geom.objid = geom.dataid = geom.segid = -1
            geom.label = label if axis == 0 else ""
            scene.ngeom += 1


def _newest_sample(buffer: bytearray, raw: bytes):
    """Keep unread bytes, validate every complete line, return the last valid one."""
    buffer.extend(raw)
    sample = None
    while True:
        index = buffer.find(b"\n")
        if index < 0:
            return sample, False
        line = bytes(buffer[:index])
        del buffer[:index + 1]
        if line == b"STOP":
            return sample, True
        candidate = _parse_state_line(line)
        if candidate is not None:
            sample = candidate


class OwnedPassiveWindow:
    """Own the render thread through GL teardown, not just its exit request."""

    def __init__(self, mujoco, model, data):
        import queue
        import threading

        launcher = getattr(mujoco.viewer, "_launch_internal", None)
        if not callable(launcher):
            raise RuntimeError("this MuJoCo version lacks the joinable passive-window launcher")
        handles = queue.Queue(1)
        self.error = None

        def render():
            try:
                # launch_passive discards its daemon thread handle. Its close()
                # only requests exit, allowing glfw.terminate at Python shutdown
                # to race the still-running renderer. Keep this same passive
                # launcher on an owned, joinable thread instead.
                launcher(
                    model, data, run_physics_thread=False, handle_return=handles,
                    show_left_ui=False, show_right_ui=False)
            except BaseException as error:
                self.error = error
                try:
                    handles.put_nowait(None)
                except queue.Full:
                    pass  # The startup handle is already awaiting its consumer.

        mujoco.mj_forward(model, data)
        self.thread = threading.Thread(target=render, name="mujoco-read-only-window", daemon=False)
        self.thread.start()
        self.viewer = handles.get()
        if self.viewer is None:
            self.thread.join()
            raise RuntimeError(f"MuJoCo window startup failed ({self.error!r})") from self.error

    def close(self) -> None:
        try:
            self.viewer.close()
        finally:
            # Leave the parent's process-exit deadline time to observe/report
            # a stuck renderer before its independently bounded forced cleanup.
            self.thread.join(CLOSE_TIMEOUT_S / 2)
        if self.thread.is_alive():
            raise RuntimeError("MuJoCo render thread did not finish shutdown")
        if self.error is not None:
            raise RuntimeError(f"MuJoCo render thread failed ({self.error!r})") from self.error


def _viewer_model(mujoco, model_path: Path, object_mesh: Path | None = None):
    if object_mesh is None:
        return mujoco.MjModel.from_xml_path(str(model_path))
    import xml.etree.ElementTree as ET
    from tianji_description.model_assets import add_object_mesh

    root = ET.parse(model_path).getroot()
    if root.tag != "mujoco" or root.find("include") is not None:
        raise ValueError("object mesh visualization requires a self-contained MJCF model")
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    # Like simulation compilation, from_xml_string needs absolute robot assets.
    assetdir = compiler.get("assetdir", "")
    for asset in root.findall("./asset/*"):
        filename = asset.get("file")
        if filename is not None:
            directory = compiler.get("meshdir" if asset.tag == "mesh" else "texturedir", assetdir)
            asset.set("file", str((model_path.parent / directory / filename).resolve()))
    for attribute in ("meshdir", "texturedir", "assetdir"):
        compiler.attrib.pop(attribute, None)
    add_object_mesh(root, object_mesh)
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def _run_window(model_path: Path, object_mesh: Path | None = None) -> int:
    def report(text: str) -> None:
        print(text, flush=True)

    try:
        import mujoco
        import mujoco.viewer
    except (ImportError, RuntimeError) as error:
        report(f"ERROR mujoco/GLFW unavailable in the viewer process ({error!r}); "
               "run the window with the interpreter that has mujoco and a display")
        return 1
    try:
        model = _viewer_model(mujoco, model_path, object_mesh)
        indices, ghost_geoms = _prepare_model(mujoco, model)
        data = mujoco.MjData(model)
        ghost = mujoco.MjData(model)
        reference_overlay = ReferenceOverlay(model, mujoco, indices)
        mujoco.mj_kinematics(model, data)
        mujoco.mj_kinematics(model, ghost)
    except Exception as error:
        report(f"ERROR the viewer could not load {model_path}: {error!r}")
        return 1
    try:
        window = OwnedPassiveWindow(mujoco, model, data)
        viewer = window.viewer
    except BaseException as error:  # a graphics/GLFW failure must not be silent
        report(f"ERROR the viewer window could not open: {error!r}")
        return 1
    try:
        legend, group_status = _overlay_texts(None)
        with viewer.lock():
            viewer.cam.lookat[:] = CAMERA_LOOKAT
            viewer.cam.distance = CAMERA_DISTANCE
            viewer.cam.azimuth = CAMERA_AZIMUTH
            viewer.cam.elevation = CAMERA_ELEVATION
            for index in range(len(viewer.opt.geomgroup)):
                visible = int(index == STATIC_GROUP)  # measured state arrives by snapshot
                viewer.opt.geomgroup[index] = visible
                viewer.opt.sitegroup[index] = visible
        _set_texts(mujoco, viewer, legend, group_status)
        viewer.sync()
        report("READY")
        result = _render_loop(mujoco, model, data, ghost, indices, ghost_geoms, viewer, reference_overlay)
    except BaseException as error:
        report(f"ERROR the viewer window failed: {error!r}")
        result = 1
    finally:
        try:
            window.close()
        except BaseException as error:
            report(f"ERROR the viewer window could not finish shutdown: {error!r}")
            result = 1
    return result


def _set_texts(mujoco, viewer, legend: str, group_status: str) -> None:
    viewer.set_texts([
        (int(mujoco.mjtFontScale.mjFONTSCALE_150), int(mujoco.mjtGridPos.mjGRID_TOPLEFT),
         legend, ""),
        (int(mujoco.mjtFontScale.mjFONTSCALE_150), int(mujoco.mjtGridPos.mjGRID_BOTTOMLEFT),
         group_status, ""),
    ])


def _render_loop(mujoco, model, data, ghost, indices, ghost_geoms, viewer, reference_overlay) -> int:
    """Show the newest snapshot at a fixed pace; stop with the window or the parent."""
    stdin_fd = sys.stdin.fileno()
    os.set_blocking(stdin_fd, False)
    buffer = bytearray()
    latest = None
    shown = None
    shown_texts = None
    camera_localized = False
    next_wake = time.monotonic()
    stopped = False
    while viewer.is_running() and not stopped:
        try:
            chunk = os.read(stdin_fd, 65536)
        except BlockingIOError:
            chunk = None  # no snapshot waiting yet
        except OSError:
            break  # unusable pipe: this window is no longer wanted
        if chunk == b"":
            # End of the parent's pipe: it stopped or exited, so the window closes too.
            break
        if chunk is not None:
            sample, stopped = _newest_sample(buffer, chunk)
            if sample is not None:
                # Age comes from the snapshot's own monotonic stamp, so a backlog
                # queued in the pipe can never be shown as current state.
                latest = sample
        state = _display_state(latest, time.monotonic_ns())
        if state is not None and state[0] != shown:
            shown, sample, stale_s = state
            legend, group_status = _overlay_texts(sample, stale_s)
            with viewer.lock():
                _apply_state(model, mujoco, data, ghost, indices, viewer.opt, sample)
                viewer.user_scn.ngeom = 0
                _fill_ghost_scene(model, mujoco, ghost, ghost_geoms, viewer.user_scn, sample)
                robot_pose = sample.get("robot_pose")
                if robot_pose is not None and not camera_localized:
                    from scipy.spatial.transform import Rotation

                    viewer.cam.lookat[:] = (
                        Rotation.from_quat(robot_pose[3:]).apply(CAMERA_LOOKAT) + robot_pose[:3])
                    camera_localized = True
                reference_overlay.update(sample.get("reference"), robot_pose=robot_pose)
                reference_overlay.draw(viewer.user_scn)
            # set_texts owns its native UI synchronization; never call it
            # while holding the model/scene lock used by the render thread.
            if (legend, group_status) != shown_texts:
                _set_texts(mujoco, viewer, legend, group_status)
                shown_texts = (legend, group_status)
            # Full sync copies derived visual arrays to the passive renderer.
            # Do not use sync(state_only=True): its mj_forward would discard
            # the display-only mocap placement and rebuild robot-world FK.
            viewer.sync()
        next_wake += RENDER_PERIOD_S
        delay = next_wake - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            next_wake = time.monotonic()
    return 0


def _render_main(argv=None) -> int:
    from argparse import ArgumentParser

    parser = ArgumentParser(description="Owned read-only MuJoCo window for real-robot state.")
    parser.add_argument("--model", type=Path, required=True, help="controller MJCF shown in the window")
    parser.add_argument("--object-mesh", type=Path, help="optional hammer OBJ in source object coordinates")
    arguments = parser.parse_args(argv)
    return _run_window(arguments.model, arguments.object_mesh)


if __name__ == "__main__":
    raise SystemExit(_render_main())
