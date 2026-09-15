"""Read-only MuJoCo window for real-robot joint state: measured solid, target cyan.

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
        except (TypeError, ValueError):
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


def _state_line(actual, target, phase, timestamp_ns: int) -> bytes:
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
    line = json.dumps(message, allow_nan=False, separators=(",", ":")).encode("ascii") + b"\n"
    if len(line) > MAX_STATE_BYTES:
        raise ValueError(f"snapshot of {len(line)} bytes exceeds the {MAX_STATE_BYTES} byte limit")
    return line


def _parse_state_line(raw: bytes):
    """Validate one received snapshot; unreadable lines return None."""
    try:
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
        return sample
    except (ValueError, KeyError, TypeError, AttributeError):
        return None


def _display_state(sample, now_ns: int):
    """Snapshot to show now: (key, sample, stale seconds) or None before the first one.

    A snapshot past the executor's watchdog is no longer current: the measured
    pose is hidden and reported as stale, and the key ages so the window keeps
    saying how old it is. The last target stays visible, labelled as stale too.
    """
    if sample is None:
        return None
    age_ns = now_ns - sample["timestamp_ns"]
    if -CLOCK_TOLERANCE_NS <= age_ns <= SNAPSHOT_TIMEOUT_NS:
        return (sample["timestamp_ns"], None), sample, None
    stale_s = round(age_ns / 1e9, 1)
    stale = {"phase": sample["phase"], "timestamp_ns": sample["timestamp_ns"],
             "actual": {}, "target": sample["target"]}
    return (sample["timestamp_ns"], stale_s), stale, stale_s


def _target_label(phase: str, has_target: bool) -> str:
    """Which target the cyan ghost is showing, per the gate's phase."""
    if not has_target:
        return "nothing published"
    if phase in FROZEN_PHASES:
        return "frozen alignment pose"
    if phase in HOME_PHASES:
        return "HOME"
    return "live teleop"


def _world_pose_known(group, positions):
    # Finger world transforms inherit Link7; finger angles alone are not a pose.
    return "arms" in positions and group in positions


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
        f"cyan = target ({_target_label(phase, bool(target))}"
        + (", STALE snapshot)" if stale_s is not None else ")"),
    ))
    status = "\n".join(
        f"{group}: actual {actual_state or ('ok' if _world_pose_known(group, actual) else 'UNKNOWN')} | "
        f"target {('STALE' if stale_s is not None else 'ok') if _world_pose_known(group, target) else 'UNKNOWN'}"
        for group in GROUP_ORDER
    )
    return legend, status


def _child_command(model_path: Path, python=None) -> list[str]:
    """Command line of the owned window process."""
    return [python or sys.executable, "-m", "real_robot.viewer", "--model", str(model_path)]


class RealRobotViewer:
    """Owned MuJoCo window process; measured state solid, target a cyan ghost."""

    def __init__(self, model_path: Path, *, startup_timeout_s: float = DEFAULT_STARTUP_TIMEOUT_S,
                 python=None):
        path = Path(model_path)
        if not math.isfinite(startup_timeout_s) or startup_timeout_s <= 0:
            raise ValueError("startup_timeout_s must be positive and finite")
        if not path.is_file():
            raise FileNotFoundError(f"viewer model not found: {path}")
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
            process = subprocess.Popen(_child_command(path, python), stdin=state_read,
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

    def publish(self, actual, target, phase: str) -> None:
        """Show the latest snapshot; invalid data raises, a dead window is a no-op."""
        line = _state_line(actual, target, phase, time.monotonic_ns())
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


def _fill_ghost_scene(model, mujoco, ghost, ghost_geoms, scene, sample) -> None:
    """Append this model's own geoms at the target pose as transparent cyan ghosts."""
    for group in GROUP_ORDER:
        if not _world_pose_known(group, sample["target"]):
            continue
        for index in ghost_geoms[group]:
            if scene.ngeom >= scene.maxgeom:
                return
            slot = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(slot, int(model.geom_type[index]), model.geom_size[index],
                                ghost.geom_xpos[index], ghost.geom_xmat[index].flatten(), GHOST_RGBA)
            slot.dataid = int(model.geom_dataid[index])
            # MuJoCo's renderer stores the mesh and its convex hull in adjacent
            # slots: 2 * model mesh id draws the original mesh, not another link.
            if slot.type in (mujoco.mjtGeom.mjGEOM_MESH, mujoco.mjtGeom.mjGEOM_SDF):
                slot.dataid *= 2
            slot.objtype = int(mujoco.mjtObj.mjOBJ_GEOM)
            slot.objid = index
            slot.segid = -1
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


def _run_window(model_path: Path) -> int:
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
        model = mujoco.MjModel.from_xml_path(str(model_path))
        indices, ghost_geoms = _prepare_model(mujoco, model)
        data = mujoco.MjData(model)
        ghost = mujoco.MjData(model)
        mujoco.mj_kinematics(model, data)
        mujoco.mj_kinematics(model, ghost)
    except Exception as error:
        report(f"ERROR the viewer could not load {model_path}: {error!r}")
        return 1
    try:
        viewer = mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)
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
        return _render_loop(mujoco, model, data, ghost, indices, ghost_geoms, viewer)
    except BaseException as error:
        report(f"ERROR the viewer window failed: {error!r}")
        return 1
    finally:
        viewer.close()


def _set_texts(mujoco, viewer, legend: str, group_status: str) -> None:
    viewer.set_texts([
        (int(mujoco.mjtFontScale.mjFONTSCALE_150), int(mujoco.mjtGridPos.mjGRID_TOPLEFT),
         legend, ""),
        (int(mujoco.mjtFontScale.mjFONTSCALE_150), int(mujoco.mjtGridPos.mjGRID_BOTTOMLEFT),
         group_status, ""),
    ])


def _render_loop(mujoco, model, data, ghost, indices, ghost_geoms, viewer) -> int:
    """Show the newest snapshot at a fixed pace; stop with the window or the parent."""
    stdin_fd = sys.stdin.fileno()
    os.set_blocking(stdin_fd, False)
    buffer = bytearray()
    latest = None
    shown = None
    shown_texts = None
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
                if (legend, group_status) != shown_texts:
                    _set_texts(mujoco, viewer, legend, group_status)
                    shown_texts = (legend, group_status)
            viewer.sync()
        next_wake += RENDER_PERIOD_S
        delay = next_wake - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            next_wake = time.monotonic()
    viewer.close()
    return 0


def _render_main(argv=None) -> int:
    from argparse import ArgumentParser

    parser = ArgumentParser(description="Owned read-only MuJoCo window for real-robot state.")
    parser.add_argument("--model", type=Path, required=True, help="controller MJCF shown in the window")
    arguments = parser.parse_args(argv)
    return _run_window(arguments.model)


if __name__ == "__main__":
    raise SystemExit(_render_main())
