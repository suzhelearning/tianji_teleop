"""Read mocap/reference/session trajectories and replay without hardware access."""
from __future__ import annotations

import argparse
from contextlib import ExitStack
import json
import math
from pathlib import Path
import threading
import time


FORMATS = ("auto", "acquisition", "regrind", "session", "data_collection")


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("command", choices=("inspect", "replay"))
    result.add_argument("trajectory", type=Path)
    result.add_argument("--format", choices=FORMATS, default="auto")
    result.add_argument("--mode", choices=("target", "joint"), default="target")
    result.add_argument("--rate", type=float, default=50.0, help="reference H5 sampling rate (no timestamp column)")
    result.add_argument("--speed", type=float, default=1.0)
    result.add_argument("--duration", type=float, default=0.0, help="wall-clock seconds; zero plays to the end")
    result.add_argument("--start-time", type=float, default=0.0)
    result.add_argument("--headless", action="store_true")
    result.add_argument("--loop", action="store_true")
    result.add_argument("--no-realtime", action="store_true", help="headless offline sampling without wall-clock sleeps")
    display = result.add_mutually_exclusive_group()
    display.add_argument("--reference-only", action="store_true", help="display captured hand/object geometry; no IK or policy")
    display.add_argument("--robot", action="store_true", help="recompute IK and drive target-project robot simulation")
    result.add_argument("--simulation-mode", choices=("direct", "dynamics"), default="direct")
    result.add_argument("--model", type=Path, help="target-project MuJoCo model override")
    result.add_argument("--config", type=Path, help="target-project IK/controller YAML override")
    result.add_argument("--legacy-tcp", choices=("flange", "hand"),
                        help="required for legacy session targets: recorded TCP origin, not inferred")
    result.add_argument("--object-mesh", type=Path, help="optional hammer visual OBJ; otherwise use reference package mesh if present")
    result.add_argument("--snapshot", type=Path, help="write final read-only rendered frame as PNG")
    return result


def run_reference(args, trajectory) -> int:
    from .clock import HoldToRunClock
    from .viewer import ReferenceScene
    mesh = args.object_mesh
    if mesh is None:
        candidate = args.trajectory.resolve().parent.parent / "objects/hammer/visual.obj"
        if candidate.is_file():
            mesh = candidate
    scene = ReferenceScene(args.model, args.config, mesh)
    paused = threading.Event()

    def key(code):
        if code == 32:
            if paused.is_set():
                paused.clear()
            else:
                paused.set()

    with ExitStack() as cleanup:
        viewer = None
        initial = trajectory.sample(args.start_time)
        if initial.space != "mocap_wrist":
            raise ValueError("reference-only view requires mocap wrist coordinates; use --robot for robot targets/joints")
        scene.update(initial)
        if not args.headless:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(scene.model, scene.data, key_callback=key)
            cleanup.callback(viewer.close)
            camera = scene.camera()
            viewer.cam.lookat[:] = camera.lookat
            viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = camera.distance, camera.azimuth, camera.elevation
        print(json.dumps({"event": "started", "mode": "reference_only", "publishes_control": False,
                          "policy_inference": False, "physics": False, "object_mesh": str(mesh) if mesh else None}), flush=True)
        started = time.monotonic()
        clock = HoldToRunClock(maximum_step_s=1.0 / args.rate)
        ticks = 0
        frame = initial
        while viewer is None or viewer.is_running():
            now = time.monotonic()
            if args.duration and now - started >= args.duration:
                break
            elapsed = clock.update(ticks / args.rate if args.no_realtime else now, not paused.is_set()) * args.speed + args.start_time
            if args.loop and elapsed > trajectory.duration_s:
                elapsed %= trajectory.duration_s
            frame = trajectory.sample(min(elapsed, trajectory.duration_s))
            if viewer is None:
                scene.update(frame)
            else:
                with viewer.lock():
                    scene.update(frame)
                    viewer.user_scn.ngeom = 0
                    scene.draw(viewer.user_scn)
                viewer.sync()
            ticks += 1
            if frame.complete and not args.loop:
                break
            if not args.no_realtime:
                time.sleep(max(0.0, 1.0 / args.rate - (time.monotonic() - now)))
        if args.snapshot:
            scene.snapshot(args.snapshot)
        print(json.dumps({"event": "completed", "mode": "reference_only", "samples": ticks,
                          "frame": frame.index, "time_s": frame.time_s, "trajectory_complete": frame.complete,
                          "snapshot": str(args.snapshot) if args.snapshot else None}), flush=True)
    return 0


def main(argv: list[str] | None = None) -> int:
    argument_parser = parser()
    args = argument_parser.parse_args(argv)
    for field in ("rate", "speed"):
        if not math.isfinite(getattr(args, field)) or getattr(args, field) <= 0:
            argument_parser.error(f"--{field} must be finite and positive")
    for field in ("duration", "start_time"):
        if not math.isfinite(getattr(args, field)) or getattr(args, field) < 0:
            argument_parser.error(f"--{field.replace('_', '-')} must be finite and non-negative")
    if args.no_realtime and not args.headless:
        argument_parser.error("--no-realtime requires --headless")
    if args.loop and args.headless and not args.duration:
        argument_parser.error("headless --loop requires a finite --duration")
    from ..data import load_trajectory
    trajectory = load_trajectory(args.trajectory, format=args.format, mode=args.mode, rate_hz=args.rate)
    print(json.dumps(trajectory.summary(), ensure_ascii=False), flush=True)
    if args.command == "inspect":
        return 0
    if args.start_time > trajectory.duration_s:
        argument_parser.error("--start-time exceeds trajectory duration")
    if args.loop and trajectory.duration_s <= 0:
        argument_parser.error("cannot loop a zero-duration trajectory")
    # Regrind reference display intentionally remains policy-free, matching the
    # original hand-replay tool. Robot replay is an explicit separate choice.
    reference_only = args.reference_only or (not args.robot and trajectory.summary().get("format") == "regrind")
    if reference_only:
        return run_reference(args, trajectory)
    from ..integration.simulation import run_replay
    return run_replay(args, trajectory)
