"""200 Hz native target-project IK into existing MuJoCo simulation; no SDKs."""
from __future__ import annotations

from contextlib import ExitStack
import json
import threading
import time

import numpy as np

from real_robot.protocol import CommandFrame
from ..replay.clock import HoldToRunClock
from .native import NativeIK
from .targets import TargetAdapter


def command_frame(positions, sequence: int) -> CommandFrame:
    values = np.asarray(positions, dtype=float)
    if values.shape != (54,) or not np.isfinite(values).all():
        raise ValueError("simulation command must contain 54 finite radians")
    return CommandFrame(sequence, time.monotonic_ns(), 1, 7,
                        tuple(values[:7]), tuple(values[7:14]),
                        tuple(values[14:34]), tuple(values[34:54]))


def run_replay(args, trajectory) -> int:
    from sim.direct_state import DirectStateSimulation
    from sim.physics import PhysicsSimulation
    paused = threading.Event()

    def key(code):
        if code == 32:
            if paused.is_set():
                paused.clear()
            else:
                paused.set()

    with ExitStack() as cleanup:
        ik = NativeIK(model=args.model, config=args.config)
        cleanup.callback(ik.close)
        adapter = TargetAdapter(ik, legacy_tcp=args.legacy_tcp)
        simulation_class = DirectStateSimulation if args.simulation_mode == "direct" else PhysicsSimulation
        simulation = simulation_class(ik.model_path, ik.config_path)
        simulation.set_targets(command_frame(adapter.current_positions, 1))
        simulation.step()
        viewer = None
        if not args.headless:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(simulation.model, simulation.data, key_callback=key)
            cleanup.callback(viewer.close)
            viewer.cam.lookat[:] = (0.15, 0.0, 0.95)
            viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 2.2, 135, -20
        print(json.dumps({"event": "started", "mode": f"robot_{args.simulation_mode}",
                          "publishes_control": False, "policy_inference": False,
                          "ik_rate_hz": 200, "physics": args.simulation_mode == "dynamics"}), flush=True)
        clock = HoldToRunClock(maximum_step_s=0.005)
        started = time.monotonic()
        initial_positions = adapter.current_positions.copy()
        maximum_motion = 0.0
        ticks = 0
        next_visual = 0.0
        frame = trajectory.sample(args.start_time)
        simulation_end = simulation.data.time
        while viewer is None or viewer.is_running():
            now = time.monotonic()
            if args.duration and now - started >= args.duration:
                break
            elapsed = args.start_time + args.speed * clock.update(ticks * 0.005 if args.no_realtime else now,
                                                                 not paused.is_set())
            if args.loop and elapsed > trajectory.duration_s:
                elapsed %= trajectory.duration_s
            frame = trajectory.sample(min(elapsed, trajectory.duration_s))
            positions = adapter.frame(frame)
            simulation.set_targets(command_frame(positions, ticks + 2))
            simulation_end += 0.005
            while simulation.data.time < simulation_end - 1e-12:
                simulation.step()
            maximum_motion = max(maximum_motion, float(np.max(np.abs(positions - initial_positions))))
            ticks += 1
            if viewer is not None and now >= next_visual:
                viewer.sync()
                next_visual = now + 1.0 / 30.0
            if frame.complete and not args.loop:
                break
            if not args.no_realtime:
                time.sleep(max(0.0, 0.005 - (time.monotonic() - now)))
        if args.snapshot:
            import mujoco
            from PIL import Image
            camera = mujoco.MjvCamera()
            camera.lookat[:] = (0.15, 0, 0.95)
            camera.distance, camera.azimuth, camera.elevation = 2.2, 135, -20
            with mujoco.Renderer(simulation.model, height=480, width=640) as renderer:
                renderer.update_scene(simulation.data, camera=camera)
                Image.fromarray(renderer.render()).save(args.snapshot)
        print(json.dumps({"event": "completed", "mode": f"robot_{args.simulation_mode}",
                          "ticks": ticks, "frame": frame.index, "time_s": frame.time_s,
                          "trajectory_complete": frame.complete, "maximum_joint_motion_rad": maximum_motion,
                          "final_positions_rad": adapter.current_positions.tolist(),
                          "snapshot": str(args.snapshot) if args.snapshot else None}), flush=True)
    return 0
