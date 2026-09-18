"""Motive-calibrated, physical-Enter H5 simulation using the native 200 Hz IK."""
from __future__ import annotations

import argparse
from contextlib import ExitStack
import json
import os
from pathlib import Path
import signal
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from ..data.geometry import compose_pose, invert_pose
from ..policies.regrind.tracking import RegrindMotiveTracker, open_mocap_session
from ..replay.clock import HoldToRunClock
from ..replay.mocap_overlay import build_object_overlays, draw_mocap_overlay
from .h5_replay import H5Replay, motive_rigid_to_wrist, motive_world_transform
from .native import NativeIK, REPOSITORY, pose_xyzw
from .real_keyboard import OperatorKeyboard
from .simulation import command_frame
from .targets import TargetAdapter, model_wrist_pose


SETTINGS_PATH = Path(__file__).resolve().parents[1] / "configs/replay.yaml"
DEFAULT_RETURN_SPEED = 2.0  # Simulation only; never sourced from real-robot motion limits.


def replay_settings(path=SETTINGS_PATH):
    settings = yaml.safe_load(Path(path).read_text())
    if not isinstance(settings, dict):
        raise ValueError("replay settings must be a mapping")
    for key in ("approach_position_tolerance_m", "approach_orientation_tolerance_deg",
                "approach_solved_position_tolerance_m", "approach_solved_orientation_tolerance_deg",
                "approach_stable_seconds", "motive_stale_s"):
        value = float(settings[key])
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f"{key} must be positive and finite")
    return settings


def _pose_close(actual, desired, position_tolerance, orientation_tolerance_deg):
    return (np.linalg.norm(actual[:3] - desired[:3]) <= position_tolerance
            and (Rotation.from_quat(actual[3:]).inv() * Rotation.from_quat(desired[3:])).magnitude()
            <= np.deg2rad(orientation_tolerance_deg))


class H5SimulationSession:
    """The actual CLI state machine, independently tickable without a TTY/router.

    Inject a tracker exposing ``latest()``/``error`` and optionally a keyboard
    exposing ``poll()``. ``step(now, keys=..., pressed=...)`` exercises exactly
    the same native solver, safety transitions, and MuJoCo execution as main.
    Only the owner of the session may call step/render. No hardware is opened.
    """

    def __init__(self, replay, adapter, simulation, tracker, *, keyboard=None, settings=None,
                 return_speed=DEFAULT_RETURN_SPEED):
        if not np.isfinite(return_speed) or return_speed <= 0:
            raise ValueError("simulation return speed must be positive and finite")
        self.replay, self.adapter, self.simulation = replay, adapter, simulation
        self.tracker, self.keyboard = tracker, keyboard
        self.settings = replay_settings() if settings is None else settings
        safety = json.loads((REPOSITORY / "real_robot/config.json").read_text())
        self._return_speed = float(return_speed)
        self._return_timeout = float(safety["staged_motion"]["timeout_s"])
        self._return_tolerance = float(safety["safety"]["arms"]["alignment_rad"])
        self._return_settle = float(safety["staged_motion"]["settle_time_s"])
        self._hand_tolerance = float(safety["safety"]["right_hand"]["alignment_rad"])
        self.home = adapter.current_positions.copy()
        self.positions = self.home.copy()
        self.phase = "armed"
        self.error = None
        self.quit = False
        self.elapsed_s = 0.0
        self.world_transform = None
        self._clock = HoldToRunClock(maximum_step_s=0.005)
        self._last_now = None
        self._last_motive = None
        self._moving = False
        self._stable_s = 0.0
        self._released = False
        self._exit_after_return = False
        self._last_s = -float("inf")
        self._sequence = 0
        self._simulation_time = simulation.data.time
        track = replay.hand_joint_track
        values = np.asarray(track.values)
        if values.ndim != 2 or values.shape[1] != 20 or not len(values) or not np.all(track.valid):
            raise ValueError("H5 replay requires a complete prepared right hand trajectory")
        if not np.isfinite(values).all():
            raise ValueError("H5 replay requires finite prepared right hand joints")
        if np.any(values < adapter.lower[34:54]) or np.any(values > adapter.upper[34:54]):
            raise ValueError("prepared right hand trajectory exceeds target-project joint limits")
        self._reference_overlay = None
        self._set_source_target(replay.sample(0.))
        # Construction only: start the actual simulator at the same configured
        # Home as NativeIK, rather than physically moving from its XML midpoint.
        import mujoco
        simulation.data.qpos[simulation.qpos_indices] = self.home
        simulation.data.qvel[:] = 0.
        simulation.set_targets(command_frame(self.home, 0))
        mujoco.mj_forward(simulation.model, simulation.data)

    def _set_source_target(self, target):
        """Remember exactly the source used by control, not the running clock."""
        self.source_target = target
        self._source_preview = self.replay.preview(target.time_s)

    def _hold(self):
        if self._moving:
            self.adapter.hold(self.positions)
        self._moving = False
        self._stable_s = 0.0

    def _fresh_motive(self, now):
        if self.tracker.error:
            raise ValueError(f"Motive: {self.tracker.error}")
        sample = self.tracker.latest()
        if sample is None:
            raise ValueError("Motive right wrist unavailable or untracked")
        stamp = float(sample.received_at)
        # Main samples now after poll; a callback can still be a fraction of a
        # tick newer. Reject future clocks beyond one control tick.
        if not np.isfinite(stamp) or not -0.005 <= now - stamp <= self.settings["motive_stale_s"]:
            raise ValueError("Motive right wrist stale or clock invalid")
        if self._last_motive is not None:
            previous_stamp, previous_frame = self._last_motive
            if stamp < previous_stamp or (stamp > previous_stamp and sample.frame_number <= previous_frame):
                raise ValueError("Motive time/frame numbering stopped advancing")
        self._last_motive = stamp, sample.frame_number
        pose_xyzw(sample.wrist_xyzw, "Motive right wrist")
        return sample

    def _begin_return(self, now, exit_after):
        if self.phase == "returning":
            self._exit_after_return |= exit_after
            return
        self._hold()
        self._return_start = self.positions.copy()
        distance = float(np.max(np.abs(self.home[7:14] - self.positions[7:14])))
        hand_distance = float(np.max(np.abs(self.home[34:] - self.positions[34:])))
        self._return_duration = max(0.5, 1.875 * max(distance, hand_distance) / self._return_speed)
        self._return_elapsed = 0.
        self._return_started = now
        self._exit_after_return = exit_after
        self.phase = "returning"
        self._stable_s = 0.

    def _advance_return(self, now, dt):
        if now - self._return_started > max(self._return_timeout, self._return_duration + 5.):
            raise ValueError("simulation Home return did not settle before timeout")
        self._return_elapsed += dt
        fraction = min(1., self._return_elapsed / self._return_duration)
        blend = fraction ** 3 * (10. + fraction * (-15. + 6. * fraction))
        q = self._return_start.copy()
        for region in (slice(7, 14), slice(34, 54)):
            q[region] += blend * (self.home[region] - q[region])
        self.positions = q
        # Direct return must reseed both native OTG and Cartesian conditioning;
        # otherwise the next s/Enter could resume the cancelled trajectory.
        self.adapter.hold(q)
        actual = self.simulation.data.qpos[self.simulation.qpos_indices]
        arrived = (fraction == 1.
                   and np.max(np.abs(actual[7:14] - self.home[7:14])) <= self._return_tolerance
                   and np.max(np.abs(actual[34:54] - self.home[34:54])) <= self._hand_tolerance)
        self._stable_s = self._stable_s + dt if arrived else 0.
        if self._stable_s >= self._return_settle:
            self.positions = self.home.copy()
            self.adapter.hold(self.positions)
            self.elapsed_s = 0.
            self._set_source_target(self.replay.sample(0.))
            self._clock = HoldToRunClock(maximum_step_s=0.005)
            self._stable_s = 0.
            self.world_transform = None
            self.phase = "armed"
            self.error = None
            self.quit = self._exit_after_return

    def _target_stable(self, target):
        desired_wrist = compose_pose(self.world_transform, target.wrist_poses["right"])
        desired_tcp = compose_pose(desired_wrist, invert_pose(self.adapter.ik.tcp_to_wrist["right"]))
        conditioned = self.adapter.last_tcp_targets.get("right")
        if conditioned is None:
            return False
        settings = self.settings
        return (_pose_close(conditioned, desired_tcp, settings["approach_position_tolerance_m"],
                            settings["approach_orientation_tolerance_deg"])
                and _pose_close(self.adapter.ik.current_wrist["right"], desired_wrist,
                                settings["approach_solved_position_tolerance_m"], settings["approach_solved_orientation_tolerance_deg"])
                and _pose_close(model_wrist_pose(self.simulation.model, self.simulation.data, "right"), desired_wrist,
                                settings["approach_solved_position_tolerance_m"], settings["approach_solved_orientation_tolerance_deg"])
                and np.max(np.abs(self.simulation.data.qpos[self.simulation.qpos_indices][34:54]
                                  - target.hand_joints["right"])) <= self._hand_tolerance)

    def _execute(self):
        self._sequence += 1
        self.simulation.set_targets(command_frame(self.positions, self._sequence))
        self._simulation_time += 0.005
        while self.simulation.data.time + 1e-12 < self._simulation_time:
            self.simulation.step()

    def step(self, now=None, *, keys=None, pressed=None):
        if keys is None or pressed is None:
            if self.keyboard is None:
                raise ValueError("provide keys and physical pressed state, or inject a keyboard")
            keys, pressed = self.keyboard.poll()
        now = time.monotonic() if now is None else float(now)
        if not np.isfinite(now) or (self._last_now is not None and now < self._last_now):
            raise ValueError("session monotonic time is invalid")
        dt = 0.005 if self._last_now is None else min(0.005, now - self._last_now)
        self._last_now = now
        keys = set(keys)
        pressed = bool(pressed)
        try:
            if "q" in keys or "\x03" in keys:
                if self.phase == "armed":
                    self.quit = True
                else:
                    self._begin_return(now, True)
            elif "s" in keys and now - self._last_s >= 0.5:
                self._last_s = now
                if self.phase == "armed":
                    if pressed:
                        raise ValueError("release Enter before establishing Motive Home")
                    sample = self._fresh_motive(now)
                    self.world_transform = motive_world_transform(
                        self.adapter.home_wrist["right"], sample.wrist_xyzw,
                        rotation=self.settings.get("motive_to_robot_quaternion_xyzw"))
                    self.adapter.set_world_transform(self.world_transform)
                    self.adapter.hold(self.positions)
                    self.elapsed_s = 0.
                    self._stable_s = 0.
                    self._released = False
                    self.error = None
                    self.phase = "approaching"
                elif self.phase != "returning":
                    self._begin_return(now, False)
            if self.phase == "returning":
                self._advance_return(now, dt)
            elif self.phase in ("approaching", "ready", "replaying"):
                self._fresh_motive(now)
                if self.phase == "ready":
                    self._hold()
                    self._released |= not pressed
                    if "r" in keys and self._released and not pressed:
                        self._clock = HoldToRunClock(maximum_step_s=0.005)
                        self._clock.update(now, False)
                        self.phase = "replaying"
                else:
                    if self.phase == "replaying":
                        self.elapsed_s = min(self.replay.duration_s, self._clock.update(now, pressed) * self.replay.speed)
                    if pressed:
                        target = self.replay.sample(self.elapsed_s)
                        self.positions = self.adapter.frame(target)
                        self._set_source_target(target)
                        self._moving = True
                        stable = (self.phase == "approaching" or target.complete) and self._target_stable(target)
                        self._stable_s = self._stable_s + dt if stable else 0.
                        if self._stable_s >= self.settings["approach_stable_seconds"]:
                            if self.phase == "approaching":
                                self.phase = "ready"
                                self._released = False
                                self._hold()
                            elif target.complete:
                                self._begin_return(now, False)
                    else:
                        self._hold()
            else:
                self._hold()
        except (ValueError, RuntimeError, TimeoutError) as exc:
            self.error = str(exc)
            self.phase = "fault"
            self._hold()
        self._execute()
        return self.status()

    def status(self):
        return {"phase": self.phase, "time_s": self.elapsed_s, "duration_s": self.replay.duration_s,
                "source_format": self.replay.trajectory.format, "hand_mode": self.replay.hand_mode,
                "error": self.error, "quit": self.quit}

    def reference_snapshot(self, now=None):
        """Read-only display snapshot; live objects do not advance source replay."""
        now = time.monotonic() if now is None else float(now)
        sample = self.tracker.latest()
        error = self.tracker.error or (self.error if self.phase == "fault" else None)
        transform = self.world_transform
        if transform is None:
            if error or sample is None:
                return None
            try:
                stamp = float(sample.received_at)
                if not np.isfinite(stamp) or not -0.005 <= now - stamp <= self.settings["motive_stale_s"]:
                    return None
                transform = motive_world_transform(
                    self.adapter.home_wrist["right"], sample.wrist_xyzw,
                    rotation=self.settings.get("motive_to_robot_quaternion_xyzw"))
            except (ValueError, TypeError):
                return None
        target = self.source_target
        states = {"armed": "ARMED", "approaching": "APPROACH", "ready": "READY",
                  "replaying": "RUNNING", "returning": "RETURN", "fault": "FAULT"}
        return {
            "wrist_pose": compose_pose(transform, target.wrist_poses["right"]),
            "hand_joints": target.hand_joints["right"],
            "world_transform": transform,
            "time_s": target.time_s,
            "index": target.index,
            "state": states[self.phase],
            "calibrated": self.world_transform is not None,
            "objects": build_object_overlays(
                target.object_poses, sample, transform, now=now,
                stale_s=self.settings["motive_stale_s"], error=error),
        }

    def render_overlay(self, scene, now=None, *, clear=True):
        """Draw the last applied DATA hand and independently fresh Motive objects."""
        if clear:
            scene.ngeom = 0
        if self._reference_overlay is None:
            import mujoco
            from real_robot.viewer import ReferenceOverlay
            self._reference_overlay = ReferenceOverlay(
                self.simulation.model, mujoco,
                {"right_hand": self.simulation.qpos_indices[34:54]})
        reference = self.reference_snapshot(now)
        self._reference_overlay.update(reference)
        if reference is not None:
            draw_mocap_overlay(scene, self._source_preview, reference["world_transform"],
                               clear=False, draw_axes=False)
            self._reference_overlay.draw(scene)

    def overlay_legend(self):
        """Describe simulation and source separately, never imply robot feedback."""
        legend = "solid = SIMULATION (not hardware)"
        if self._reference_overlay is not None and self._reference_overlay.reference is not None:
            legend += "\n" + self._reference_overlay.legend()
        else:
            target = self.source_target
            legend += (f"\nDATA | {self.phase.upper()} | provisional localization unavailable"
                       f"\nsource frame {target.index} | {target.time_s:.3f} s"
                       "\nMotive object poses UNAVAILABLE")
        if self.phase == "replaying" and not self._moving:
            legend += "\nREPLAY PAUSED (DATA frozen; Motive stays live)"
        if self.error:
            legend += f"\nFAULT: {self.error}"
        return legend


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--h5", type=Path, required=True)
    result.add_argument("--speed", type=float, default=1.)
    result.add_argument("--return-speed", type=float, default=DEFAULT_RETURN_SPEED,
                        help="simulation Home return peak joint speed in rad/s (default: 2); independent of --speed and real motion limits")
    result.add_argument("--yaw-deg", type=float, default=0.)
    result.add_argument("--endpoint", default=os.environ.get("TIANJI_ROUTER_ENDPOINT", "tcp/127.0.0.1:7447"))
    result.add_argument("--wrist-name", default="right_wrist")
    result.add_argument("--headless", action="store_true")
    result.add_argument("--duration", type=float, help="request safe Home return after this many wall seconds")
    result.add_argument("--simulation-mode", choices=("direct", "dynamics"), default="direct")
    result.add_argument("--model", type=Path)
    result.add_argument("--object-mesh", type=Path,
                        help="OBJ in object-local meters for target/live object overlays (visualization only)")
    result.add_argument("--config", type=Path)
    result.add_argument("--replay-config", type=Path, default=SETTINGS_PATH)
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    if not np.isfinite(args.return_speed) or args.return_speed <= 0:
        raise ValueError("simulation return speed must be positive and finite")
    if args.duration is not None and (not np.isfinite(args.duration) or args.duration <= 0):
        raise ValueError("--duration must be positive and finite")
    if not sys.stdin.isatty():
        raise RuntimeError("h5-sim requires an operator TTY and physical Enter; headless only disables the viewer")
    replay = H5Replay(args.h5, speed=args.speed, yaw_deg=args.yaw_deg)
    settings = replay_settings(args.replay_config)
    from sim.direct_state import DirectStateSimulation
    from sim.physics import PhysicsSimulation
    with ExitStack() as cleanup:
        keyboard = OperatorKeyboard(hold_enter=True)
        cleanup.callback(keyboard.close)
        ik = NativeIK(model=args.model, config=args.config)
        cleanup.callback(ik.close)
        adapter = TargetAdapter(ik, alignment="absolute", conditioning_settings=settings)
        simulation_type = DirectStateSimulation if args.simulation_mode == "direct" else PhysicsSimulation
        simulation = simulation_type(ik.model_path, ik.config_path, object_mesh=args.object_mesh)
        transport = open_mocap_session(args.endpoint)
        cleanup.callback(transport.close)
        tracker = RegrindMotiveTracker(transport, wrist_name=args.wrist_name, require_object=False,
                                      rigid_to_wrist=motive_rigid_to_wrist(settings))
        cleanup.callback(tracker.close)
        session = H5SimulationSession(replay, adapter, simulation, tracker, keyboard=keyboard,
                                      settings=settings, return_speed=args.return_speed)
        viewer = None
        if not args.headless:
            import mujoco.viewer
            from real_robot.viewer import OwnedPassiveWindow
            window = OwnedPassiveWindow(mujoco, simulation.model, simulation.data)
            cleanup.callback(window.close)
            viewer = window.viewer
            viewer.cam.lookat[:] = [0., 0., 0.9]
            viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 2.2, 135, -20
        stopping = [False]
        def stop(_signum, _frame):
            stopping[0] = True
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous = signal.signal(signum, stop)
            cleanup.callback(signal.signal, signum, previous)
        print("s: record Motive Home / cancel and return Home; hold physical Enter: move; release: hold; r after frame0 + release: load replay; q: Home then exit", flush=True)
        print("Keep the tracked physical robot at the configured Home pose: the overlay previews live localization until s locks it.", flush=True)
        print(json.dumps({"event": "started", "mode": args.simulation_mode, "publishes_control": False,
                          "source_format": replay.trajectory.format, "hand_mode": replay.hand_mode,
                          "right_wrist": args.wrist_name}), flush=True)
        started = time.monotonic()
        previous_status = None
        next_visual = started
        requested_exit = False
        while not session.quit:
            keys, pressed = keyboard.poll()
            now = time.monotonic()
            if stopping[0] or (args.duration is not None and now - started >= args.duration) or (viewer is not None and not viewer.is_running()):
                if not requested_exit:
                    keys.add("q")
                    requested_exit = True
            status = session.step(now, keys=keys, pressed=pressed)
            change = status["phase"], status["error"]
            if change != previous_status:
                print(json.dumps(status, ensure_ascii=False), flush=True)
                previous_status = change
            if requested_exit and session.phase == "fault":
                raise RuntimeError(session.error or "safe return failed")
            if viewer is not None and viewer.is_running() and now >= next_visual:
                with viewer.lock():
                    session.render_overlay(viewer.user_scn, now)
                # Text updates synchronize internally with the render thread.
                viewer.set_texts([
                    (int(mujoco.mjtFontScale.mjFONTSCALE_150),
                     int(mujoco.mjtGridPos.mjGRID_TOPLEFT),
                     session.overlay_legend(), ""),
                ])
                viewer.sync()
                next_visual = now + 1. / 60.
            time.sleep(max(0., 0.005 - (time.monotonic() - now)))
        print(json.dumps({"event": "completed", **session.status()}), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
