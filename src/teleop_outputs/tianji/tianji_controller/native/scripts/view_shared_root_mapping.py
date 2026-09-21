"""Offline filtered mapping viewer. Static robot; no IK, mj_step, or sockets."""
from contextlib import ExitStack
from pathlib import Path
import argparse
import bisect
import hashlib
import subprocess
import time
import threading

import mujoco
import numpy as np
import yaml

from tianji_runtime import controller_profile, native_executable
from tianji_runtime.resources import controller_resource


def load(profile, trace):
    config = yaml.safe_load(profile.read_text())["spark_shared_root"]
    artifact = controller_resource(profile, config["robot_geometry_artifact"])
    geometry = yaml.safe_load(artifact.read_text())["robot_geometry"]
    model = controller_resource(artifact, geometry["mujoco_xml_path"])
    result = subprocess.run([str(native_executable("tianji_shared_root_trace_audit")),
                             str(profile), str(trace), "--mapping-frames"],
                            capture_output=True, text=True, timeout=60, check=True)
    if "ik_executed=false" not in result.stdout:
        raise ValueError("native mapping-only contract missing")
    if hashlib.sha256(model.read_bytes()).hexdigest() != geometry["mujoco_xml_sha256"]:
        raise ValueError("model changed during audit")
    if ("geometry_sha256=" + hashlib.sha256(artifact.read_bytes()).hexdigest()) not in result.stdout:
        raise ValueError("geometry changed during audit")
    frames = []
    for line in result.stdout.splitlines():
        if not line.startswith("mapping_frame "):
            continue
        fields = line.split()
        stamp, sequence, valid = map(int, fields[1:4])
        values = np.asarray(fields[4:], dtype=float)
        if valid not in (0, 1) or values.size != (48 if valid else 0) or not np.isfinite(values).all():
            raise ValueError("invalid mapping frame")
        if frames and stamp < frames[-1][0]:
            raise ValueError("nonmonotonic mapping timeline")
        frames.append((stamp, sequence, values.reshape(2, 24) if valid else None))
    if not frames:
        raise ValueError("empty mapping output")
    return model, frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--check", action="store_true", help="validate without a window")
    args = parser.parse_args()
    if args.profile is None:
        args.profile = controller_profile("qp_ik_pico_shared_root.yaml")
    model_path, frames = load(args.profile.resolve(), args.trace.resolve())
    print(f"Mapping replay: {len(frames)} frames; FILTERED targets, no guidance blend/IK/physics.", flush=True)
    if args.check:
        return
    import mujoco.viewer
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    for side in ("L", "R"):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "Joint2_" + side)
        data.qpos[model.jnt_qposadr[jid]] = -np.pi / 2
    mujoco.mj_forward(model, data)
    fixed = data.qpos.copy()
    times = [(f[0] - frames[0][0]) * 1e-9 for f in frames]
    state = {"pause": False, "restart": False, "quit": False}

    def key(code):
        if code == ord("P"):
            state["pause"] = not state["pause"]
        elif code == ord("R"):
            state["restart"] = True
        elif code == ord("Q"):
            state["quit"] = True

    print("Orange=left shape; green=right shape; white sphere + RGB axes=palm target; magenta=shape-to-palm gap. P pause; R replay; Q exit. End holds last frame.", flush=True)
    viewer_threads_before = set(threading.enumerate())
    # MuJoCo closes its passive viewer asynchronously. Join the threads created
    # by this launch before Python tears down the model and rendering libraries.
    with ExitStack() as cleanup, mujoco.viewer.launch_passive(model, data, key_callback=key) as viewer:
        for thread in set(threading.enumerate()) - viewer_threads_before:
            cleanup.callback(thread.join)
        viewer.cam.lookat[:] = [.35, 0, 1.12]
        viewer.cam.distance = 2.2
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        elapsed, last = 0., time.monotonic()
        while viewer.is_running() and not state["quit"]:
            now = time.monotonic()
            if not state["pause"]:
                elapsed = min(times[-1], elapsed + now - last)
            last = now
            if state["restart"]:
                elapsed = 0.
                state["restart"] = False
            index = max(0, bisect.bisect_right(times, elapsed) - 1)
            stamp, sequence, targets = frames[index]
            with viewer.lock():
                data.qpos[:] = fixed
                data.qvel[:] = 0
                mujoco.mj_forward(model, data)
                scene = viewer.user_scn
                scene.ngeom = 0

                def geom(kind, point, color, end=None, label=""):
                    if scene.ngeom >= scene.maxgeom:
                        raise RuntimeError("overlay capacity exceeded")
                    g = scene.geoms[scene.ngeom]
                    scene.ngeom += 1
                    mujoco.mjv_initGeom(g, kind, np.array([.012, 0, 0]),
                                       np.asarray(point), np.eye(3).ravel(), np.asarray(color, dtype=np.float32))
                    if end is not None:
                        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_LINE, 4., point, end)
                    g.label = label

                title = f"FILTERED mapping only | {elapsed:.1f}/{times[-1]:.1f}s | seq {sequence} | P pause R replay Q exit"
                if targets is None:
                    title += " | INVALID: targets hidden"
                geom(mujoco.mjtGeom.mjGEOM_SPHERE, [0, 0, 1.6], [1, 1, 1, 1], label=title)
                if targets is not None:
                    for side, row in enumerate(targets):
                        points = row[:15].reshape(5, 3)
                        rotation = row[15:].reshape(3, 3)
                        color = [1, .55, .05, 1] if side == 0 else [.3, 1, .2, 1]
                        for a, b in zip(points[:3], points[1:4]):
                            geom(mujoco.mjtGeom.mjGEOM_LINE, a, color, b)
                        for p in points[:4]:
                            geom(mujoco.mjtGeom.mjGEOM_SPHERE, p, color)
                        palm = points[4]
                        gap = np.linalg.norm(palm - points[3]) * 1000
                        geom(mujoco.mjtGeom.mjGEOM_SPHERE, palm, [1, 1, 1, 1], label=f"{'L' if side == 0 else 'R'} palm gap={gap:.1f}mm")
                        geom(mujoco.mjtGeom.mjGEOM_LINE, points[3], [1, 0, 1, 1], palm)
                        for axis, color_axis in enumerate(([1, 0, 0, 1], [0, 1, 0, 1], [0, .3, 1, 1])):
                            geom(mujoco.mjtGeom.mjGEOM_LINE, palm, color_axis, palm + .07 * rotation[:, axis])
            viewer.sync()
            time.sleep(.02)


if __name__ == "__main__":
    main()
