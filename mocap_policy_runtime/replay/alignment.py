"""Reference versus live Motive placement; no inference or motor authority."""
from __future__ import annotations

from dataclasses import replace
import json
import time

import mujoco
import numpy as np

from ..data import load_trajectory
from ..policies.regrind.live import hammer_alignment
from .viewer import ReferenceScene


def run_alignment(args, reference, live) -> int:
    import mujoco.viewer
    trajectory = load_trajectory(args.reference, format="regrind", rate_hz=50.0)
    # Both streams already use the same Motive/training world. In particular,
    # do not register away the object error or infer a world rotation from the
    # robot's Home wrist orientation.
    expected = trajectory.sample(args.start_frame / 50.0)
    candidate = args.reference.resolve().parent.parent / "objects/hammer/visual.obj"
    scene = ReferenceScene(object_mesh=candidate if candidate.is_file() else None, live_overlay=True)
    scene.update(expected)
    started = time.monotonic()
    with mujoco.viewer.launch_passive(scene.model, scene.data) as viewer:
        camera = scene.camera()
        viewer.cam.lookat[:] = camera.lookat
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = camera.distance, camera.azimuth, camera.elevation
        while viewer.is_running():
            now = time.monotonic()
            if args.duration is not None and now - started >= args.duration:
                break
            sample = live.latest()
            fresh = (sample is not None and sample.hammer_xyzw is not None
                     and not live.error and now - sample.received_at <= args.stale_s)
            report = hammer_alignment(reference, sample, args.start_frame) if fresh else None
            objects = dict(expected.object_poses)
            if fresh:
                objects.update(live_hammer=sample.hammer_xyzw, live_wrist=sample.wrist_xyzw)
            frame = replace(expected, object_poses=objects)
            with viewer.lock():
                scene.update(frame)
                viewer.user_scn.ngeom = 0
                scene.draw(viewer.user_scn)
            status = (f"{report['hammer_start_position_error_mm']:.1f} mm / "
                      f"{report['hammer_start_orientation_error_deg']:.1f} deg\n"
                      f"{'ALIGNED' if report['real_start_preflight_passed'] else 'NOT ALIGNED'}") if report else (live.error or "Motive unavailable/stale")
            viewer.set_texts(((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                               "READ ONLY - NO POLICY OR CONTROL\nReference hand / green hammer; live orange hammer\nLive wrist shown as axes, not measured hand joints",
                               status),))
            viewer.sync()
            time.sleep(1.0 / 60.0)
    print(json.dumps({"event": "viewer_closed", "publishes_control": False}), flush=True)
    return 0
